#!/usr/bin/python3
# SPDX-FileCopyrightText: 2026 Junya Wada
# SPDX-License-Identifier: BSD-3-Clause
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import math
from collections import deque
from datetime import datetime
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import ParticleCloud
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
    qos_profile_sensor_data
)
from std_srvs.srv import Trigger


class Superposition(Node):
    def __init__(self):
        super().__init__("superposition")

        # --- パラメータ ---
        # n: 何秒ごとにスナップショットを取ってパブリッシュするか
        self.declare_parameter('capture_interval_sec', 1.0)
        # m: 何世代分(何秒分)のスナップショットを重ねるか
        self.declare_parameter('snapshot_count', 5)
        # パーティクルトピック名 (AMCL: /particle_cloud, EMCL: /particles など)
        self.declare_parameter('particle_topic', '/particle_cloud')
        # パーティクルメッセージ型 ('ParticleCloud' or 'PoseArray')
        self.declare_parameter('particle_msg_type', 'ParticleCloud')
        # 表示制御パラメータ
        self.declare_parameter('show_particles', True)        # パーティクルを表示するか
        self.declare_parameter('show_laser_scan', True)       # ライダーデータを表示するか
        self.declare_parameter('show_best_pose', True)        # 代表位置を表示するか
        self.declare_parameter('particle_radius', 2)          # パーティクルのサイズ[px]
        self.declare_parameter('best_pose_radius', 4)         # 代表点の基準半径[px]
        self.declare_parameter('laser_point_radius', 1)       # レーザー点群の半径[px]

        self.capture_interval_sec = self.get_parameter('capture_interval_sec').value
        self.snapshot_count = self.get_parameter('snapshot_count').value
        self.particle_topic = self.get_parameter('particle_topic').value
        self.particle_msg_type = self.get_parameter('particle_msg_type').value
        self.show_particles = self.get_parameter('show_particles').value
        self.show_laser_scan = self.get_parameter('show_laser_scan').value
        self.show_best_pose = self.get_parameter('show_best_pose').value
        self.particle_radius = self.get_parameter('particle_radius').value
        self.best_pose_radius = self.get_parameter('best_pose_radius').value
        self.laser_point_radius = self.get_parameter('laser_point_radius').value

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        particle_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # --- マップ関連の状態 ---
        self.base_map_img = None
        self.map_info = None
        self.map_frame_id = 'map'

        # --- 最新メッセージのキャッシュ(capture_timerで使う) ---
        self.latest_particle_msg = None
        self.latest_scan_msg = None

        # --- スナップショット履歴 ---
        # 各エントリ: {'pose': (bpx, bpy, yaw) or None, 'particles': [...], 'laser_points': [...]}
        # deque(maxlen=snapshot_count) で古いものが自動的に捨てられる
        self.snapshot_history = deque(maxlen=self.snapshot_count)

        # --- 最新の描画結果(サービスコールで保存する用) ---
        self.latest_overlay = None

        self.cv_bridge = CvBridge()

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)
        
        # パーティクルトピックをメッセージ型に応じて購読
        if self.particle_msg_type == 'PoseArray':
            self.particle_sub = self.create_subscription(
                PoseArray, self.particle_topic, self.particle_callback_posearray, particle_qos)
        else:  # デフォルト: ParticleCloud
            self.particle_sub = self.create_subscription(
                ParticleCloud, self.particle_topic, self.particle_callback, particle_qos)
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        self.image_pub = self.create_publisher(Image, '/vlm_context_image', 10)

        self.save_service = self.create_service(
            Trigger, '/save_overlay_image', self.save_overlay_callback)

        # capture_interval_sec ごとに「スナップショット取得→描画→パブリッシュ」を一括実行
        self.capture_timer = self.create_timer(
            self.capture_interval_sec, self.capture_callback)

        self.get_logger().info(
            f'起動 (particle_topic={self.particle_topic}, '
            f'particle_msg_type={self.particle_msg_type}, '
            f'capture_interval={self.capture_interval_sec}s, '
            f'snapshot_count={self.snapshot_count}, '
            f'show_particles={self.show_particles}, '
            f'show_laser_scan={self.show_laser_scan}, '
            f'show_best_pose={self.show_best_pose})')

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

        # OccupancyGridは原点が左下基準なので上下反転
        self.base_map_img = np.flipud(img).copy()

    # ------------------------------------------------------------------
    def particle_callback(self, msg: ParticleCloud):
        """最新のParticleCloudメッセージをキャッシュするだけ(履歴反映はcapture_timerで行う)"""
        self.latest_particle_msg = msg

    # ------------------------------------------------------------------
    def particle_callback_posearray(self, msg: PoseArray):
        """最新のPoseArrayメッセージをキャッシュするだけ(履歴反映はcapture_timerで行う)
        PoseArrayを内部形式に変換して保存"""
        # PoseArrayをParticleCloud風に変換(重みは均等に1.0/len(poses))
        if len(msg.poses) == 0:
            self.latest_particle_msg = None
            return
        
        # 簡易的にParticleCloud型のmsgオブジェクトを作成
        # (実際にはParticleCloudではなく、内部的にPoseArrayとして処理)
        self.latest_particle_msg = msg

    # ------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        """最新のスキャンメッセージをキャッシュするだけ(履歴反映はcapture_timerで行う)"""
        self.latest_scan_msg = msg

    # ------------------------------------------------------------------
    def capture_callback(self):
        """capture_interval_sec ごとに呼ばれる。
        その時点の最新センサデータを1スナップショットとして履歴に追加し、
        snapshot_count 世代分を重ねた画像を生成してパブリッシュする。"""
        if self.base_map_img is None or self.map_info is None:
            self.get_logger().warn('マップ未受信のためキャプチャをスキップ')
            return
        if self.latest_particle_msg is None:
            self.get_logger().warn('パーティクル未受信のためキャプチャをスキップ')
            return

        # 自己位置の代表点(重み付き平均)を計算
        pose_px = self.compute_best_pose_pixel(self.latest_particle_msg)

        # パーティクルをピクセル座標に変換
        particle_pixels = []
        if self.show_particles:
            particle_pixels = self.compute_particles_pixels(self.latest_particle_msg)

        # レーザー点群をmap座標系のピクセルに変換
        laser_points = []
        if self.show_laser_scan and self.latest_scan_msg is not None and pose_px is not None:
            best_pose_world = self.compute_best_pose_world(self.latest_particle_msg)
            if best_pose_world is not None:
                laser_points = self.transform_scan_to_pixels(
                    self.latest_scan_msg, best_pose_world)

        # スナップショットをキューに追加(maxlenにより古いものは自動で削除される)
        self.snapshot_history.append({
            'pose': pose_px,
            'particles': particle_pixels,
            'laser_points': laser_points
        })

        self.get_logger().info(
            f'キャプチャ #{len(self.snapshot_history)}/{self.snapshot_count} '
            f'(particles: {len(particle_pixels)}個, laser: {len(laser_points)}点)')

        # 描画してパブリッシュ
        self.latest_overlay = self.render_overlay()
        self.publish_image()

    # ------------------------------------------------------------------
    def compute_best_pose_world(self, particle_msg):
        """ParticleCloud または PoseArray から重み付き平均(world座標)を計算する"""
        sum_w = sum_x = sum_y = sum_sin = sum_cos = 0.0

        if isinstance(particle_msg, ParticleCloud):
            # ParticleCloud型: 重みが設定されている
            for particle in particle_msg.particles:
                w = particle.weight
                sum_w += w
                sum_x += w * particle.pose.position.x
                sum_y += w * particle.pose.position.y
                yaw = self.quaternion_to_yaw(particle.pose.orientation)
                sum_sin += w * math.sin(yaw)
                sum_cos += w * math.cos(yaw)
        else:
            # PoseArray型: 重みは均等(1.0)
            for pose in particle_msg.poses:
                w = 1.0
                sum_w += w
                sum_x += w * pose.position.x
                sum_y += w * pose.position.y
                yaw = self.quaternion_to_yaw(pose.orientation)
                sum_sin += w * math.sin(yaw)
                sum_cos += w * math.cos(yaw)

        if sum_w <= 0.0:
            return None

        return (sum_x / sum_w, sum_y / sum_w, math.atan2(sum_sin, sum_cos))

    # ------------------------------------------------------------------
    def compute_best_pose_pixel(self, particle_msg):
        """ParticleCloud または PoseArray から重み付き平均をピクセル座標で返す。(px, py, yaw) or None"""
        world = self.compute_best_pose_world(particle_msg)
        if world is None:
            return None

        mean_x, mean_y, mean_yaw = world
        bpx, bpy = self.world_to_pixel(mean_x, mean_y)
        if 0 <= bpx < self.map_info.width and 0 <= bpy < self.map_info.height:
            return (bpx, bpy, mean_yaw)
        return None

    # ------------------------------------------------------------------
    def compute_particles_pixels(self, particle_msg):
        """ParticleCloud または PoseArray の全パーティクルをピクセル座標に変換する"""
        particles = []

        if isinstance(particle_msg, ParticleCloud):
            # ParticleCloud型
            for particle in particle_msg.particles:
                x = particle.pose.position.x
                y = particle.pose.position.y
                px, py = self.world_to_pixel(x, y)
                if 0 <= px < self.map_info.width and 0 <= py < self.map_info.height:
                    particles.append((px, py, particle.weight))
        else:
            # PoseArray型: 重みは均等に設定
            weight = 1.0 / len(particle_msg.poses) if len(particle_msg.poses) > 0 else 1.0
            for pose in particle_msg.poses:
                x = pose.position.x
                y = pose.position.y
                px, py = self.world_to_pixel(x, y)
                if 0 <= px < self.map_info.width and 0 <= py < self.map_info.height:
                    particles.append((px, py, weight))
        
        return particles

    # ------------------------------------------------------------------
    def transform_scan_to_pixels(self, scan_msg: LaserScan, robot_pose_world):
        """自己位置(world座標)を基準にレーザー点群をmap座標系のピクセル点に変換する"""
        robot_x, robot_y, robot_yaw = robot_pose_world
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)

        points = []
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            if r < scan_msg.range_min or r > scan_msg.range_max or not math.isfinite(r):
                angle += scan_msg.angle_increment
                continue

            lx = r * math.cos(angle)
            ly = r * math.sin(angle)

            wx = robot_x + lx * cos_yaw - ly * sin_yaw
            wy = robot_y + lx * sin_yaw + ly * cos_yaw

            px, py = self.world_to_pixel(wx, wy)
            if 0 <= px < self.map_info.width and 0 <= py < self.map_info.height:
                points.append((px, py))

            angle += scan_msg.angle_increment

        return points

    # ------------------------------------------------------------------
    def world_to_pixel(self, x, y):
        """world座標(map frame) -> 画像座標(px, py) に変換"""
        info = self.map_info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        px = int((x - ox) / res)
        py = info.height - 1 - int((y - oy) / res)  # flipud分のy軸反転
        return px, py

    # ------------------------------------------------------------------
    def render_overlay(self):
        """スナップショット履歴を古い順(赤)→新しい順(緑)でマップに重ねて描画する。
        show_* パラメータで表示要素をコントロールする。"""
        overlay = self.base_map_img.copy()
        n = len(self.snapshot_history)
        if n == 0:
            return overlay

        for i, snapshot in enumerate(self.snapshot_history):
            # t: 0.0(最古=赤) ~ 1.0(最新=緑)
            t = i / (n - 1) if n > 1 else 1.0
            color = self.get_color(t)

            # パーティクルを描画（重みの大きさで透明度を変動させる）
            if self.show_particles:
                for (px, py, weight) in snapshot['particles']:
                    # 重みが小さいと薄く表示
                    radius = max(1, int(self.particle_radius * (0.5 + weight)))
                    cv2.circle(overlay, (px, py), radius, color, 1)

            # レーザー点群を描画
            if self.show_laser_scan:
                for (px, py) in snapshot['laser_points']:
                    cv2.circle(overlay, (px, py), self.laser_point_radius, color, -1)

            # 自己位置の代表点を描画（最前面）
            if self.show_best_pose and snapshot['pose'] is not None:
                bpx, bpy, yaw = snapshot['pose']
                self.draw_best_pose(overlay, bpx, bpy, yaw, t)

        return overlay

    # ------------------------------------------------------------------
    def draw_best_pose(self, img, px, py, yaw, t):
        """自己位置の代表点を赤(古)→緑(新)グラデーション+矢印で描画する。
        新しいほど半径を大きくして視覚的に最前面であることを強調する。"""
        color = self.get_color(t)
        r = max(1, int(self.best_pose_radius * (0.4 + 0.6 * t)))

        cv2.circle(img, (px, py), r, (0, 0, 0), 2)   # 縁取り
        cv2.circle(img, (px, py), r, color, -1)

        # 向き矢印(flipud分のy軸反転を反映)
        length = r * 3
        ex = int(px + length * math.cos(yaw))
        ey = int(py - length * math.sin(yaw))
        cv2.arrowedLine(img, (px, py), (ex, ey), (0, 0, 0), 2, tipLength=0.4)

    # ------------------------------------------------------------------
    @staticmethod
    def get_color(t):
        """t(0=最古~1=最新)に応じて Jet グラデーション(青→シアン→緑→黄→赤)の色を返す(BGR)。
        OpenCVのHSV変換を使い、H=120(青)→H=0(赤) で均一な明度を保つ。"""
        t = max(0.0, min(1.0, t))
        hue = int(120 * (1.0 - t))           # 古: 120(青) → 新: 0(赤)
        hsv = np.uint8([[[hue, 255, 220]]])   # S=255(鮮やか), V=220(白地でも潰れない明度)
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0][0]
        return (int(bgr[0]), int(bgr[1]), int(bgr[2]))

    # ------------------------------------------------------------------
    @staticmethod
    def quaternion_to_yaw(q):
        """geometry_msgs/Quaternion -> yaw角[rad]"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ------------------------------------------------------------------
    def publish_image(self):
        """latest_overlay を /vlm_context_image にパブリッシュする"""
        if self.latest_overlay is None:
            return

        out_msg = self.cv_bridge.cv2_to_imgmsg(self.latest_overlay, encoding="bgr8")
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = self.map_frame_id
        self.image_pub.publish(out_msg)
        self.get_logger().info('画像パブリッシュ')

    # ------------------------------------------------------------------
    def save_overlay_callback(self, request, response):
        """サービスコール(std_srvs/Trigger): 現在の重畳画像を /tmp に保存する"""
        if self.latest_overlay is None:
            response.success = False
            response.message = 'No overlay image available'
            self.get_logger().warn('画像未生成のため保存をスキップ')
            return response

        try:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
            filename = f'/tmp/overlay_{timestamp}.png'
            cv2.imwrite(filename, self.latest_overlay)
            response.success = True
            response.message = f'Image saved to {filename}'
            self.get_logger().info(f'画像を保存: {filename}')
        except Exception as e:
            response.success = False
            response.message = f'Error saving image: {e}'
            self.get_logger().error(f'画像保存エラー: {e}')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = Superposition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
