#!/usr/bin/python3
# SPDX-FileCopyrightText: 2026 Junya Wada
# SPDX-License-Identifier: BSD-3-Clause
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import math
from collections import deque
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import ParticleCloud
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
    qos_profile_sensor_data
)


class Superposition(Node):
    def __init__(self):
        super().__init__("superposition")

        # --- パラメータ ---
        self.declare_parameter('history_length', 5)        # 何世代分の分布を重ねるか
        self.declare_parameter('publish_rate', 2.0)          # パブリッシュ周期[Hz]
        self.declare_parameter('show_best_pose', True)        # 自己位置代表点(重み付き平均)を描画するか
        self.declare_parameter('best_pose_radius', 4)         # 代表点の基準半径[px]
        self.declare_parameter('laser_point_radius', 1)       # レーザー点群の半径[px]

        self.history_length = self.get_parameter('history_length').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.show_best_pose = self.get_parameter('show_best_pose').value
        self.best_pose_radius = self.get_parameter('best_pose_radius').value
        self.laser_point_radius = self.get_parameter('laser_point_radius').value

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # AMCLの /particle_cloud は BEST_EFFORT で配信されるため合わせる
        particle_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # --- マップ関連の状態 ---
        self.base_map_img = None      # マップだけのBGR画像 (背景)
        self.map_info = None          # OccupancyGrid.info を保持(座標変換用)
        self.map_frame_id = 'map'

        # --- 代表点(重み付き平均)の履歴。描画用ピクセル座標とyawを保持 (px, py, yaw) ---
        self.best_pose_history = deque(maxlen=self.history_length)

        # --- 直近の代表点のworld座標(x, y, yaw)。レーザー点群の変換基準に使う ---
        self.latest_best_pose_world = None

        # --- レーザースキャンの履歴(各スキャンの map 座標系ピクセル点リスト) ---
        self.laser_history = deque(maxlen=self.history_length)

        # --- 最新の描画結果(タイマーでパブリッシュする用) ---
        self.latest_overlay = None

        self.cv_bridge = CvBridge()

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)
        self.particle_sub = self.create_subscription(
            ParticleCloud, '/particle_cloud', self.particle_callback, particle_qos)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        self.image_pub = self.create_publisher(Image, '/vlm_context_image', 10)

        # パブリッシュ周期はパーティクル受信頻度と独立させ、タイマーで一定周期に揃える
        period = 1.0 / self.publish_rate if self.publish_rate > 0 else 0.5
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'起動 (publish_rate={self.publish_rate}Hz, history_length={self.history_length})')

    # ------------------------------------------------------------------
    def map_callback(self, msg):
        self.get_logger().info('マップ受信')
        self.map_info = msg.info
        self.map_frame_id = msg.header.frame_id or 'map'

        grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)

        img = np.zeros((msg.info.height, msg.info.width, 3), dtype=np.uint8)
        img[grid == 0] = (255, 255, 255)
        img[grid == 100] = (0, 0, 0)
        img[grid == -1] = (200, 200, 200)

        # OccupancyGridは原点が左下基準なので、画像として見やすいよう上下反転しておく
        self.base_map_img = np.flipud(img).copy()

    # ------------------------------------------------------------------
    def world_to_pixel(self, x, y):
        """world座標(map frame) -> 画像座標(px, py) に変換"""
        info = self.map_info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        px = int((x - ox) / res)
        # flipud しているのでy軸を反転
        py = info.height - 1 - int((y - oy) / res)
        return px, py

    # ------------------------------------------------------------------
    def particle_callback(self, msg: ParticleCloud):
        """受信のたびに重み付き平均(代表点)を計算し、最新の重畳画像を作っておく(送信はタイマー側で行う)"""
        if self.base_map_img is None or self.map_info is None:
            self.get_logger().warn('マップ未受信のためパーティクルをスキップ')
            return

        self.get_logger().info('自己位置取得')

        # 重み付き平均(自己位置の代表点)を求めるための累積変数
        sum_w = 0.0
        sum_x = 0.0
        sum_y = 0.0
        sum_sin = 0.0
        sum_cos = 0.0

        # nav2_msgs/msg/ParticleCloud は particles: nav2_msgs/msg/Particle[]
        # 各 Particle は pose(geometry_msgs/Pose) と weight(float64) を持つ
        for particle in msg.particles:
            w = particle.weight
            sum_w += w
            sum_x += w * particle.pose.position.x
            sum_y += w * particle.pose.position.y

            # 角度は周期性を持つため、単純平均ではなく単位円上のベクトルとして加重平均する
            yaw = self.quaternion_to_yaw(particle.pose.orientation)
            sum_sin += w * math.sin(yaw)
            sum_cos += w * math.cos(yaw)

        # 重み付き平均(weighted mean)で自己位置の代表点を計算
        if self.show_best_pose and sum_w > 0.0:
            mean_x = sum_x / sum_w
            mean_y = sum_y / sum_w
            mean_yaw = math.atan2(sum_sin, sum_cos)

            self.latest_best_pose_world = (mean_x, mean_y, mean_yaw)

            bpx, bpy = self.world_to_pixel(mean_x, mean_y)
            if 0 <= bpx < self.map_info.width and 0 <= bpy < self.map_info.height:
                self.best_pose_history.append((bpx, bpy, mean_yaw))

        self.latest_overlay = self.render_overlay()

    # ------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        self.get_logger().info('センサデータ取得')
        """直近の代表点(自己位置)を基準にレーザー点群をmap座標系へ変換して履歴に追加する"""
        if self.base_map_img is None or self.map_info is None:
            return
        if self.latest_best_pose_world is None:
            return

        robot_x, robot_y, robot_yaw = self.latest_best_pose_world
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)

        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            # 無効な距離値(範囲外、NaN、inf)は除外
            if r < msg.range_min or r > msg.range_max or not math.isfinite(r):
                angle += msg.angle_increment
                continue

            # レーザーフレーム上の点(ロボット正面方向がx軸)
            lx = r * math.cos(angle)
            ly = r * math.sin(angle)

            # ロボットの代表姿勢(x, y, yaw)で回転・並進し、map座標系の点に変換
            wx = robot_x + lx * cos_yaw - ly * sin_yaw
            wy = robot_y + lx * sin_yaw + ly * cos_yaw

            px, py = self.world_to_pixel(wx, wy)
            if 0 <= px < self.map_info.width and 0 <= py < self.map_info.height:
                points.append((px, py))

            angle += msg.angle_increment

        self.laser_history.append(points)
        self.latest_overlay = self.render_overlay()

    # ------------------------------------------------------------------
    def render_overlay(self):
        self.get_logger().info('画像描画')
        """自己位置の代表点と、それを基準にしたレーザー点群を、赤(古)→緑(新)のグラデーションで描画する。
        新しいものほど後に描いて最前面にする。"""
        overlay = self.base_map_img.copy()

        # レーザー点群の履歴を、古い世代から新しい世代の順に重ねて描画
        # -> 後に描かれる新しい世代が最前面に来る
        ln = len(self.laser_history)
        if ln > 0:
            for i, points in enumerate(self.laser_history):
                t = i / (ln - 1) if ln > 1 else 1.0
                color = self.get_particle_color(t)
                for (px, py) in points:
                    cv2.circle(overlay, (px, py), self.laser_point_radius, color, -1)

        # 代表点(重み付き平均)の履歴を、レーザー点群より後(最前面)に描画
        m = len(self.best_pose_history)
        if self.show_best_pose and m > 0:
            for i, (bpx, bpy, yaw) in enumerate(self.best_pose_history):
                t = i / (m - 1) if m > 1 else 1.0
                self.draw_best_pose(overlay, bpx, bpy, yaw, t)

        return overlay

    # ------------------------------------------------------------------
    def draw_best_pose(self, img, px, py, yaw, t):
        """自己位置の代表点(重み付き平均)を描画。
        t(0=最古~1=最新)に応じて 赤(古)→緑(新) のグラデーションにし、
        新しいものほど大きく(最前面感を強調)描画する。"""
        t = max(0.0, min(1.0, t))
        color = self.get_particle_color(t)

        # 新しいほど大きく描く(視覚的に最前面・最新であることを強調)
        r = max(1, int(self.best_pose_radius * (0.4 + 0.6 * t)))

        # 円(縁取り付きで見やすく)
        cv2.circle(img, (px, py), r, (0, 0, 0), 2)
        cv2.circle(img, (px, py), r, color, -1)

        # 向き(yaw)を示す矢印。flipudで画像のy軸が反転しているのでyaw方向も反転させる
        length = r * 3
        ex = int(px + length * math.cos(yaw))
        ey = int(py - length * math.sin(yaw))
        cv2.arrowedLine(img, (px, py), (ex, ey), (0, 0, 0), 2, tipLength=0.4)

    # ------------------------------------------------------------------
    @staticmethod
    def quaternion_to_yaw(q):
        """geometry_msgs/Quaternion -> yaw角[rad]"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ------------------------------------------------------------------
    def get_particle_color(self, t):
        """t(0=最古~1=最新)に応じて 赤(古)→緑(新) のグラデーション色を返す(BGR)"""
        t = max(0.0, min(1.0, t))
        b = 0
        g = int(255 * t)
        r = int(255 * (1 - t))
        return (b, g, r)

    # ------------------------------------------------------------------
    def timer_callback(self):
        """publish_rateで指定した周期でパブリッシュ(新規パーティクル未受信時は最新の状態を再送)"""
        if self.latest_overlay is None:
            return

        out_msg = self.cv_bridge.cv2_to_imgmsg(self.latest_overlay, encoding="bgr8")
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = self.map_frame_id
        self.image_pub.publish(out_msg)
        self.get_logger().info('画像パブリッシュ')


def main(args=None):
    rclpy.init(args=args)
    node = Superposition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
