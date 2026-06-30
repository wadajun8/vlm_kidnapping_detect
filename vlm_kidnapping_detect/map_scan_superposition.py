#!/usr/bin/python3
# SPDX-FileCopyrightText: 2026 Junya Wada
# SPDX-License-Identifier: BSD-3-Clause
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from collections import deque
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import ParticleCloud
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


class Superposition(Node):
    def __init__(self):
        super().__init__("superposition")

        # --- パラメータ ---
        self.declare_parameter('history_length', 5)        # 何世代分の分布を重ねるか
        self.declare_parameter('particle_radius', 1)        # 描画する点の半径[px]
        self.declare_parameter('publish_rate', 2.0)          # パブリッシュ周期[Hz]

        self.history_length = self.get_parameter('history_length').value
        self.particle_radius = self.get_parameter('particle_radius').value
        self.publish_rate = self.get_parameter('publish_rate').value

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

        # --- パーティクル履歴 (世代ごとのリスト、右端が最新) ---
        self.particle_history = deque(maxlen=self.history_length)

        # --- 最新の描画結果(タイマーでパブリッシュする用) ---
        self.latest_overlay = None

        self.cv_bridge = CvBridge()

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)
        self.particle_sub = self.create_subscription(
            ParticleCloud, '/particle_cloud', self.particle_callback, particle_qos)

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
        """受信のたびに履歴を更新し、最新の重畳画像を作っておく(送信はタイマー側で行う)"""
        if self.base_map_img is None or self.map_info is None:
            self.get_logger().warn('マップ未受信のためパーティクルをスキップ')
            return

        current_points = []
        # nav2_msgs/msg/ParticleCloud は particles: nav2_msgs/msg/Particle[]
        # 各 Particle は pose(geometry_msgs/Pose) と weight(float64) を持つ
        for particle in msg.particles:
            px, py = self.world_to_pixel(
                particle.pose.position.x, particle.pose.position.y)
            if 0 <= px < self.map_info.width and 0 <= py < self.map_info.height:
                current_points.append((px, py))

        self.particle_history.append(current_points)
        self.latest_overlay = self.render_overlay()

    # ------------------------------------------------------------------
    def render_overlay(self):
        """新=緑、古=赤のグラデーションで描画する。新しいものほど後に描いて最前面にする"""
        overlay = self.base_map_img.copy()
        n = len(self.particle_history)
        if n == 0:
            return overlay

        # 古い世代(index 0)から新しい世代(index n-1)の順に描画
        # -> 後に描かれる新しい世代が他の点に上書きされ、最前面に来る
        for i, points in enumerate(self.particle_history):
            # t: 0.0(最古) ~ 1.0(最新)
            t = i / (n - 1) if n > 1 else 1.0
            color = self.get_particle_color(t)

            for (px, py) in points:
                cv2.circle(overlay, (px, py), self.particle_radius, color, -1)

        return overlay

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


def main(args=None):
    rclpy.init(args=args)
    node = Superposition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
