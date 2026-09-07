#!/usr/bin/python3
# SPDX-FileCopyrightText: 2026 Junya Wada
# SPDX-License-Identifier: BSD-3-Clause
"""
パーティクルフィルタ(AMCL等)の推定結果と地図・スキャンを重畳描画し、
VLMへのコンテキスト画像として配信・保存するROS2ノード。

責務ごとに以下のクラスへ分割している:
    MapImage        : OccupancyGrid -> ベース画像、および世界座標<->画素座標変換
    PoseEstimator    : ParticleCloud / PoseArray から推定姿勢・パーティクル画素を計算
    ScanProjector    : LaserScan を画素座標へ投影
    OverlayRenderer  : スナップショット履歴を時系列カラーで描画
    ImageSaver       : 画像のディスク保存
    Superposition    : 上記を束ねるROS2ノード本体(購読/配信/サービス)
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Deque, List, Optional, Sequence, Tuple, Union

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, Quaternion
from nav2_msgs.msg import ParticleCloud
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, LaserScan

# std_srvs.srvのTriggerを削除し、カスタムサービスをインポート
from vlm_kidnapping_detect.srv import SaveOverlayImage

# --- 型エイリアス ---
Pixel = Tuple[int, int]
WeightedPixel = Tuple[int, int, float]
PoseWorld = Tuple[float, float, float]   # x, y, yaw
PosePixel = Tuple[int, int, float]       # px, py, yaw
BGRColor = Tuple[int, int, int]
ParticleMsg = Union[ParticleCloud, PoseArray]

# --- 占有格子の値と描画色 ---
OCC_FREE = 0
OCC_OCCUPIED = 100
OCC_UNKNOWN = -1

COLOR_FREE: BGRColor = (255, 255, 255)
COLOR_OCCUPIED: BGRColor = (0, 0, 0)
COLOR_UNKNOWN: BGRColor = (200, 200, 200)


def quaternion_to_yaw(q: Quaternion) -> float:
    """クォータニオンからYaw角を取り出す"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class Snapshot:
    """1回分のキャプチャ結果(描画に必要な画素情報のみを保持)"""

    pose: Optional[PosePixel]
    particles: List[WeightedPixel] = field(default_factory=list)
    laser_points: List[Pixel] = field(default_factory=list)


class MapImage:
    """OccupancyGridからベース画像を作成し、座標変換を提供する"""

    def __init__(self) -> None:
        self.image: Optional[np.ndarray] = None
        self.info = None
        self.frame_id: str = 'map'

    def update(self, msg: OccupancyGrid) -> None:
        self.info = msg.info
        self.frame_id = msg.header.frame_id or 'map'

        grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

        img = np.zeros((msg.info.height, msg.info.width, 3), dtype=np.uint8)
        img[grid == OCC_FREE] = COLOR_FREE
        img[grid == OCC_OCCUPIED] = COLOR_OCCUPIED
        img[grid == OCC_UNKNOWN] = COLOR_UNKNOWN

        self.image = np.flipud(img).copy()

    @property
    def ready(self) -> bool:
        return self.image is not None and self.info is not None

    def world_to_pixel(self, x: float, y: float) -> Pixel:
        info = self.info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        px = int((x - ox) / res)
        py = info.height - 1 - int((y - oy) / res)
        return px, py

    def in_bounds(self, px: int, py: int) -> bool:
        return 0 <= px < self.info.width and 0 <= py < self.info.height


class PoseEstimator:
    """ParticleCloud / PoseArray の違いを吸収し、推定姿勢とパーティクル画素を計算する"""

    @staticmethod
    def _weighted_poses(particle_msg: ParticleMsg) -> List[Tuple[float, float, float, float]]:
        """(x, y, yaw, weight) のリストへ正規化する"""
        if isinstance(particle_msg, ParticleCloud):
            return [
                (
                    p.pose.position.x,
                    p.pose.position.y,
                    quaternion_to_yaw(p.pose.orientation),
                    p.weight,
                )
                for p in particle_msg.particles
            ]

        poses = particle_msg.poses
        weight = 1.0 / len(poses) if poses else 1.0
        return [
            (p.position.x, p.position.y, quaternion_to_yaw(p.orientation), weight)
            for p in poses
        ]

    @classmethod
    def best_pose_world(cls, particle_msg: ParticleMsg) -> Optional[PoseWorld]:
        poses = cls._weighted_poses(particle_msg)
        sum_w = sum(w for *_, w in poses)
        if sum_w <= 0.0:
            return None

        sum_x = sum(w * x for x, _, _, w in poses)
        sum_y = sum(w * y for _, y, _, w in poses)
        sum_sin = sum(w * math.sin(yaw) for _, _, yaw, w in poses)
        sum_cos = sum(w * math.cos(yaw) for _, _, yaw, w in poses)

        return sum_x / sum_w, sum_y / sum_w, math.atan2(sum_sin, sum_cos)

    @classmethod
    def best_pose_pixel(cls, particle_msg: ParticleMsg, map_image: MapImage) -> Optional[PosePixel]:
        world = cls.best_pose_world(particle_msg)
        if world is None:
            return None

        x, y, yaw = world
        px, py = map_image.world_to_pixel(x, y)
        if map_image.in_bounds(px, py):
            return px, py, yaw
        return None

    @classmethod
    def particle_pixels(cls, particle_msg: ParticleMsg, map_image: MapImage) -> List[WeightedPixel]:
        pixels: List[WeightedPixel] = []
        for x, y, _, w in cls._weighted_poses(particle_msg):
            px, py = map_image.world_to_pixel(x, y)
            if map_image.in_bounds(px, py):
                pixels.append((px, py, w))
        return pixels


class ScanProjector:
    """LaserScanをロボット姿勢基準で世界座標へ変換し、地図の画素座標へ投影する"""

    @staticmethod
    def project(scan_msg: LaserScan, robot_pose_world: PoseWorld, map_image: MapImage) -> List[Pixel]:
        robot_x, robot_y, robot_yaw = robot_pose_world
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)

        points: List[Pixel] = []
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            if scan_msg.range_min <= r <= scan_msg.range_max and math.isfinite(r):
                lx = r * math.cos(angle)
                ly = r * math.sin(angle)

                wx = robot_x + lx * cos_yaw - ly * sin_yaw
                wy = robot_y + lx * sin_yaw + ly * cos_yaw

                px, py = map_image.world_to_pixel(wx, wy)
                if map_image.in_bounds(px, py):
                    points.append((px, py))

            angle += scan_msg.angle_increment

        return points


@dataclass
class RenderConfig:
    show_particles: bool = True
    show_laser_scan: bool = True
    show_best_pose: bool = True
    particle_radius: int = 2
    best_pose_radius: int = 4
    laser_point_radius: int = 1


class OverlayRenderer:
    """スナップショット履歴を時系列カラー(新しいほど赤寄り)でベース画像に描画する"""

    def __init__(self, config: RenderConfig) -> None:
        self.config = config

    def render(self, base_map_img: np.ndarray, history: Sequence[Snapshot]) -> np.ndarray:
        overlay = base_map_img.copy()
        n = len(history)
        if n == 0:
            return overlay

        for i, snap in enumerate(history):
            t = i / (n - 1) if n > 1 else 1.0
            color = self._color_for(t)
            is_latest = i == n - 1

            if self.config.show_particles:
                self._draw_particles(overlay, snap.particles, color)

            if self.config.show_laser_scan and is_latest:
                self._draw_laser_points(overlay, snap.laser_points, color)

            if self.config.show_best_pose and snap.pose is not None:
                self._draw_best_pose(overlay, snap.pose, t)

        return overlay

    def _draw_particles(self, img: np.ndarray, particles: Sequence[WeightedPixel], color: BGRColor) -> None:
        for px, py, weight in particles:
            radius = max(1, int(self.config.particle_radius * (0.5 + weight)))
            cv2.circle(img, (px, py), radius, color, 1)

    def _draw_laser_points(self, img: np.ndarray, points: Sequence[Pixel], color: BGRColor) -> None:
        for px, py in points:
            cv2.circle(img, (px, py), self.config.laser_point_radius, color, -1)

    def _draw_best_pose(self, img: np.ndarray, pose: PosePixel, t: float) -> None:
        px, py, yaw = pose
        color = self._color_for(t)
        r = max(1, int(self.config.best_pose_radius * (0.4 + 0.6 * t)))

        cv2.circle(img, (px, py), r, (0, 0, 0), 2)
        cv2.circle(img, (px, py), r, color, -1)

        length = r * 3
        ex = int(px + length * math.cos(yaw))
        ey = int(py - length * math.sin(yaw))
        cv2.arrowedLine(img, (px, py), (ex, ey), (0, 0, 0), 2, tipLength=0.4)

    @staticmethod
    def _color_for(t: float) -> BGRColor:
        t = max(0.0, min(1.0, t))
        hue = int(120 * (1.0 - t))
        hsv = np.uint8([[[hue, 255, 220]]])
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0][0]
        return int(bgr[0]), int(bgr[1]), int(bgr[2])


class ImageSaver:
    """タイムスタンプ付きでオーバーレイ画像・カメラ画像をディスクへ保存する"""

    def __init__(self, output_dir: Union[str, Path] = '.') -> None:
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def save(
        self,
        overlay: Optional[np.ndarray],
        camera_img: Optional[np.ndarray],
    ) -> Tuple[bool, str]:
        if overlay is None:
            return False, 'No overlay image available'

        timestamp = datetime.now().strftime('%Y_%m%d_%H%M')  # 2026_0906_1350 の形式
        try:
            overlay_path = self.output_dir / f'{timestamp}_overlay.png'
            cv2.imwrite(str(overlay_path), overlay)
            message = f'Overlay saved to {overlay_path}'

            if camera_img is not None:
                perspective_path = self.output_dir / f'{timestamp}_perspective.png'
                cv2.imwrite(str(perspective_path), camera_img)
                message += f', Perspective saved to {perspective_path}'

            return True, message
        except Exception as e:  # 保存失敗はサービス応答として呼び出し元へ返す
            return False, f'Error saving images: {e}'


@dataclass
class NodeParams:
    capture_interval_sec: float = 1.0
    snapshot_count: int = 5
    particle_topic: str = '/particle_cloud'
    particle_msg_type: str = 'ParticleCloud'
    show_particles: bool = True
    show_laser_scan: bool = True
    show_best_pose: bool = True
    particle_radius: int = 2
    best_pose_radius: int = 4
    laser_point_radius: int = 1


class Superposition(Node):
    def __init__(self) -> None:
        super().__init__('superposition')

        self.params = self._load_params()
        self.map_image = MapImage()
        self.renderer = OverlayRenderer(RenderConfig(
            show_particles=self.params.show_particles,
            show_laser_scan=self.params.show_laser_scan,
            show_best_pose=self.params.show_best_pose,
            particle_radius=self.params.particle_radius,
            best_pose_radius=self.params.best_pose_radius,
            laser_point_radius=self.params.laser_point_radius,
        ))
        self.saver = ImageSaver('/tmp')
        self.cv_bridge = CvBridge()

        # --- 最新メッセージのキャッシュ ---
        self.latest_particle_msg: Optional[ParticleMsg] = None
        self.latest_scan_msg: Optional[LaserScan] = None
        self.latest_camera_msg: Optional[Image] = None
        self.latest_overlay: Optional[np.ndarray] = None

        # --- スナップショット履歴 ---
        self.snapshot_history: Deque[Snapshot] = deque(maxlen=self.params.snapshot_count)

        # --- 連続保存用の状態管理変数 ---
        self._continuous_save_timer = None
        self._images_to_save = 0

        self._init_subscriptions_and_services()

        self.capture_timer = self.create_timer(
            self.params.capture_interval_sec, self._on_capture_timer)

        self.get_logger().info(
            f'起動 (particle_topic={self.params.particle_topic}, '
            f'particle_msg_type={self.params.particle_msg_type}, '
            f'capture_interval={self.params.capture_interval_sec}s, '
            f'snapshot_count={self.params.snapshot_count}, '
            f'show_particles={self.params.show_particles}, '
            f'show_laser_scan={self.params.show_laser_scan}, '
            f'show_best_pose={self.params.show_best_pose})')

    # ------------------------------------------------------------------
    # 初期化
    # ------------------------------------------------------------------
    def _load_params(self) -> NodeParams:
        defaults = NodeParams()
        for name, default in vars(defaults).items():
            self.declare_parameter(name, default)
        values = {name: self.get_parameter(name).value for name in vars(defaults)}
        return NodeParams(**values)

    def _init_subscriptions_and_services(self) -> None:
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        particle_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._on_map, map_qos)

        particle_type = PoseArray if self.params.particle_msg_type == 'PoseArray' else ParticleCloud
        self.particle_sub = self.create_subscription(
            particle_type, self.params.particle_topic, self._on_particles, particle_qos)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)

        self.camera_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self._on_camera, qos_profile_sensor_data)

        self.image_pub = self.create_publisher(Image, '/vlm_context_image', 10)

        self.save_service = self.create_service(
            SaveOverlayImage, '/save_overlay_image', self._on_save_overlay_request)

    # ------------------------------------------------------------------
    # 購読コールバック
    # ------------------------------------------------------------------
    def _on_map(self, msg: OccupancyGrid) -> None:
        self.get_logger().info('マップ受信')
        self.map_image.update(msg)

    def _on_particles(self, msg: ParticleMsg) -> None:
        if isinstance(msg, PoseArray) and len(msg.poses) == 0:
            self.latest_particle_msg = None
            return
        self.latest_particle_msg = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self.latest_scan_msg = msg

    def _on_camera(self, msg: Image) -> None:
        self.latest_camera_msg = msg

    # ------------------------------------------------------------------
    # キャプチャ・描画・配信
    # ------------------------------------------------------------------
    def _on_capture_timer(self) -> None:
        if not self.map_image.ready:
            self.get_logger().warn('マップ未受信のためキャプチャをスキップ')
            return
        if self.latest_particle_msg is None:
            self.get_logger().warn('パーティクル未受信のためキャプチャをスキップ')
            return

        snapshot = self._build_snapshot(self.latest_particle_msg)
        self.snapshot_history.append(snapshot)

        self.get_logger().info(
            f'キャプチャ #{len(self.snapshot_history)}/{self.params.snapshot_count} '
            f'(particles: {len(snapshot.particles)}個, laser: {len(snapshot.laser_points)}点)')

        self.latest_overlay = self.renderer.render(self.map_image.image, self.snapshot_history)
        self._publish_overlay()

    def _build_snapshot(self, particle_msg: ParticleMsg) -> Snapshot:
        pose_px = PoseEstimator.best_pose_pixel(particle_msg, self.map_image)

        particles: List[WeightedPixel] = []
        if self.params.show_particles:
            particles = PoseEstimator.particle_pixels(particle_msg, self.map_image)

        laser_points: List[Pixel] = []
        if (self.params.show_laser_scan
                and self.latest_scan_msg is not None
                and pose_px is not None):
            best_pose_world = PoseEstimator.best_pose_world(particle_msg)
            if best_pose_world is not None:
                laser_points = ScanProjector.project(
                    self.latest_scan_msg, best_pose_world, self.map_image)

        return Snapshot(pose=pose_px, particles=particles, laser_points=laser_points)

    def _publish_overlay(self) -> None:
        if self.latest_overlay is None:
            return

        out_msg = self.cv_bridge.cv2_to_imgmsg(self.latest_overlay, encoding='bgr8')
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = self.map_image.frame_id
        self.image_pub.publish(out_msg)
        self.get_logger().info('画像パブリッシュ')

    # ------------------------------------------------------------------
    # 保存サービス
    # ------------------------------------------------------------------
    def _on_save_overlay_request(self, request, response):
        num_images = request.num_images if request.num_images > 0 else 1
        interval = float(request.interval_sec)

        if num_images == 1 or interval <= 0.0:
            success, message = self._save_single_image()
            response.success = success
            response.message = message
            return response

        self._cancel_continuous_save_timer(log_cancel=True)
        self._images_to_save = num_images

        self._save_single_image()
        self._images_to_save -= 1

        if self._images_to_save > 0:
            self._continuous_save_timer = self.create_timer(
                interval, self._on_continuous_save_timer)

        response.success = True
        response.message = f'{interval}秒ごとに合計{num_images}枚の画像保存を開始しました。'
        return response

    def _on_continuous_save_timer(self) -> None:
        if self._images_to_save > 0:
            self._save_single_image()
            self._images_to_save -= 1

        if self._images_to_save <= 0:
            self._cancel_continuous_save_timer(log_cancel=False)
            self.get_logger().info('指定された全画像の連続保存が完了しました。')

    def _cancel_continuous_save_timer(self, log_cancel: bool) -> None:
        if self._continuous_save_timer is not None:
            self._continuous_save_timer.cancel()
            self._continuous_save_timer = None
            if log_cancel:
                self.get_logger().info('以前の保存処理をキャンセルして新しい保存を開始します。')

    def _save_single_image(self) -> Tuple[bool, str]:
        """1枚の画像をタイムスタンプ付きで保存し、成否とメッセージを返す"""
        if self.latest_overlay is None:
            self.get_logger().warn('オーバーレイ画像未生成のため保存をスキップ')

        camera_img = None
        if self.latest_camera_msg is not None:
            camera_img = self.cv_bridge.imgmsg_to_cv2(
                self.latest_camera_msg, desired_encoding='bgr8')
        else:
            self.get_logger().warn('カメラ画像未受信のため、オーバーレイ画像のみ保存しました')

        success, message = self.saver.save(self.latest_overlay, camera_img)
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)
        return success, message


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Superposition()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
