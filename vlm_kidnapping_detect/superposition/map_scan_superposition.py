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
from rclpy.signals import SignalHandlerOptions
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

# レーザースキャンはパーティクルと区別するため固定色で描画する。
# パーティクルの時系列カラーは 青->シアン->緑->黄->赤 の色相を使うため、
# そこに含まれないマゼンタを既定色にしている。
COLOR_LASER: BGRColor = (255, 0, 255)  # BGR: マゼンタ


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
    laser_color: BGRColor = COLOR_LASER


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
                self._draw_laser_points(overlay, snap.laser_points, self.config.laser_color)

            if self.config.show_best_pose and snap.pose is not None:
                self._draw_best_pose(overlay, snap.pose, t)

        return overlay

    def _draw_particles(self, img: np.ndarray, particles: Sequence[WeightedPixel], color: BGRColor) -> None:
        """particle_time_series_node.py方式:円の輪郭ではなく、
        パーティクル位置の画素をベクトル演算で直接上書きする。
        重み(weight)は座標抽出時に無視され、点の大きさには反映しない。"""
        if not particles:
            return

        points = np.array([(px, py) for px, py, _weight in particles], dtype=np.int32)
        px_arr = points[:, 0]
        py_arr = points[:, 1]

        valid = (
            (px_arr >= 0)
            & (py_arr >= 0)
            & (px_arr < img.shape[1])
            & (py_arr < img.shape[0])
        )
        if not np.any(valid):
            return

        img[py_arr[valid], px_arr[valid]] = color

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

        timestamp = datetime.now().strftime('%Y_%m%d_%H%M%S')  # 2026_0906_135042 の形式
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


class StateLogger:
    """key毎に直前の状態を覚え、状態が変化した時だけログを出す"""

    _UNSET = object()

    def __init__(self, logger) -> None:
        self._logger = logger
        self._states: dict = {}

    def get(self, key: str):
        return self._states.get(key)

    def update(self, key: str, state, message: str, level: str = 'info') -> bool:
        """状態が前回と異なればlevelでmessageを出力し、Trueを返す"""
        if self._states.get(key, self._UNSET) == state:
            return False
        self._states[key] = state
        # rclpyは同じ呼び出し行で重要度を変えると例外になるため、重要度毎に行を分ける
        if level == 'error':
            self._logger.error(message)
        elif level == 'warn':
            self._logger.warn(message)
        else:
            self._logger.info(message)
        return True


@dataclass
class NodeParams:
    capture_interval_sec: float = 1.0
    # この秒数以上受信が無ければ途絶とみなす(0以下で無効)
    topic_timeout_sec: float = 3.0
    snapshot_count: int = 5
    particle_topic: str = '/particle_cloud'
    particle_msg_type: str = 'ParticleCloud'
    show_particles: bool = True
    show_laser_scan: bool = True
    show_best_pose: bool = True
    particle_radius: int = 2
    best_pose_radius: int = 4
    laser_point_radius: int = 1
    laser_point_color: List[int] = field(default_factory=lambda: list(COLOR_LASER))


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
            laser_color=self._laser_color(),
        ))
        self.saver = ImageSaver('/tmp')
        self.cv_bridge = CvBridge()

        # --- 最新メッセージのキャッシュ ---
        self.latest_particle_msg: Optional[ParticleMsg] = None
        self.latest_scan_msg: Optional[LaserScan] = None
        self.latest_camera_msg: Optional[Image] = None
        self.latest_overlay: Optional[np.ndarray] = None

        # --- 状態変化時のみログを出すためのロガー ---
        self.state_log = StateLogger(self.get_logger())

        # --- 購読トピック毎の最終受信時刻(途絶検知用) ---
        self._last_received: dict = {}

        # --- 一度でもパブリッシュしたか(「開始」と「再開」のログを出し分ける) ---
        self._has_published = False

        # --- 監視する購読トピックと、ログに出す配信元の名前 ---
        self._topic_sources = {
            self.params.particle_topic: 'AMCL',
            '/scan': 'LiDAR',
            '/camera/color/image_raw': 'カメラサーバ',
        }

        # --- スナップショット履歴 ---
        self.snapshot_history: Deque[Snapshot] = deque(maxlen=self.params.snapshot_count)

        # --- 連続保存用の状態管理変数 ---
        self._continuous_save_timer = None
        self._images_to_save = 0

        self._init_subscriptions_and_services()

        self.capture_timer = self.create_timer(
            self.params.capture_interval_sec, self._on_capture_timer)
        self.monitor_timer = self.create_timer(1.0, self._on_monitor_timer)

        interval = self.params.capture_interval_sec
        self.get_logger().info(
            f'{interval}秒ごと、'
            f'直近{self.params.snapshot_count}個の重畳画像を、'
            f'{interval}秒ごとに /vlm_context_image へパブリッシュ')
        self.get_logger().info(
            f'起動 (particle_topic={self.params.particle_topic}, '
            f'particle_msg_type={self.params.particle_msg_type}, '
            f'capture_interval={self.params.capture_interval_sec}s, '
            f'topic_timeout={self.params.topic_timeout_sec}s, '
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

    def _laser_color(self) -> BGRColor:
        """laser_point_colorパラメータ(BGR)を検証してタプルへ変換する"""
        raw = self.params.laser_point_color
        try:
            channels = [int(c) for c in raw]
        except (TypeError, ValueError):
            channels = []

        if len(channels) != 3 or any(c < 0 or c > 255 for c in channels):
            self.get_logger().warn(
                f'laser_point_colorが不正です({raw})。既定値{COLOR_LASER}を使用します')
            return COLOR_LASER

        return channels[0], channels[1], channels[2]

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
        info = msg.info
        # 定期的に再配信されても、地図の中身(サイズ・解像度・原点)が変わった時だけログを出す
        self.state_log.update(
            'map',
            (info.width, info.height, info.resolution,
             info.origin.position.x, info.origin.position.y),
            f'マップ受信 (/map, {info.width}x{info.height}, '
            f'resolution={info.resolution:.3f})')
        self.map_image.update(msg)

    def _on_particles(self, msg: ParticleMsg) -> None:
        self._mark_received(self.params.particle_topic)
        empty = isinstance(msg, PoseArray) and len(msg.poses) == 0
        self.state_log.update(
            'particles_empty', empty,
            f'パーティクルが空です ({self.params.particle_topic})' if empty
            else f'パーティクル受信 ({self.params.particle_topic})',
            level='warn' if empty else 'info')
        self.latest_particle_msg = None if empty else msg

    def _on_scan(self, msg: LaserScan) -> None:
        self._mark_received('/scan')
        self.latest_scan_msg = msg

    def _on_camera(self, msg: Image) -> None:
        self._mark_received('/camera/color/image_raw')
        self.latest_camera_msg = msg

    # ------------------------------------------------------------------
    # 接続・受信状態の監視
    # ------------------------------------------------------------------
    def _mark_received(self, topic: str) -> None:
        """受信時刻を記録し、未受信/途絶からの復帰時だけログを出す"""
        self._last_received[topic] = self.get_clock().now()
        source = self._topic_sources[topic]
        prev = self.state_log.get(f'recv:{topic}')
        self.state_log.update(
            f'recv:{topic}', 'receiving',
            f'{source}からの受信再開 ({topic})' if prev == 'timeout'
            else f'{source}から受信開始 ({topic})')

    def _on_monitor_timer(self) -> None:
        # 購読トピック: publisherの有無で接続/切断を判定
        for topic, source in self._topic_sources.items():
            connected = self.count_publishers(topic) > 0
            prev = self.state_log.get(f'conn:{topic}')
            if connected:
                message = f'{source}と接続 ({topic} の購読開始)'
            elif prev:
                message = f'{source}との接続が切れました ({topic} のpublisherが消えました)'
            else:
                message = f'{source}と未接続 ({topic} のpublisherがいません)'
            self.state_log.update(
                f'conn:{topic}', connected, message,
                level='info' if connected else 'warn')
            self._check_timeout(topic)

        # 配信トピック: subscriberの有無
        has_sub = self.image_pub.get_subscription_count() > 0
        self.state_log.update(
            'conn:/vlm_context_image', has_sub,
            '/vlm_context_image の購読者が接続しました' if has_sub
            else '/vlm_context_image の購読者はいません')

    def _check_timeout(self, topic: str) -> None:
        # AMCLは静止中パーティクルを配信しないため、パーティクルは途絶判定の対象外
        if topic == self.params.particle_topic:
            return
        timeout = self.params.topic_timeout_sec
        last = self._last_received.get(topic)
        if timeout <= 0.0 or last is None:
            return
        elapsed = (self.get_clock().now() - last).nanoseconds * 1e-9
        if elapsed >= timeout:
            self.state_log.update(
                f'recv:{topic}', 'timeout',
                f'{self._topic_sources[topic]}からの受信途絶 '
                f'({topic}, {timeout:.1f}秒以上受信なし)', level='warn')

    # ------------------------------------------------------------------
    # キャプチャ・描画・配信
    # ------------------------------------------------------------------
    def _on_capture_timer(self) -> None:
        # キャプチャはcapture_interval_sec毎に走るため、状態が変わった時だけログを出す
        if not self.map_image.ready:
            self.state_log.update(
                'capture', 'no_map', 'マップ未受信のためキャプチャを待機中', level='warn')
            return
        if self.latest_particle_msg is None:
            self.state_log.update(
                'capture', 'no_particle', 'パーティクル未受信のためキャプチャを待機中', level='warn')
            return

        try:
            snapshot = self._build_snapshot(self.latest_particle_msg)
            self.snapshot_history.append(snapshot)

            # snapshot_count枚たまるまでは描画・パブリッシュしない
            if len(self.snapshot_history) < self.params.snapshot_count:
                self.state_log.update(
                    'capture', 'collecting',
                    f'スナップショット収集中 ({self.params.snapshot_count}枚たまったらパブリッシュ開始)')
                return

            self.latest_overlay = self.renderer.render(self.map_image.image, self.snapshot_history)
            self._publish_overlay()
        except Exception as e:  # 同じエラーが続いても1回だけ出す
            self.state_log.update(
                'capture', ('error', str(e)), f'重畳画像の生成・配信に失敗: {e}', level='error')
            return

        self.state_log.update(
            'capture', 'publishing',
            '重畳画像のパブリッシュ再開 (/vlm_context_image)' if self._has_published
            else '重畳画像のパブリッシュ開始 (/vlm_context_image)')
        self._has_published = True

        in_map = snapshot.pose is not None
        if in_map:
            in_map_message = ('推定姿勢は地図内です' if self.state_log.get('pose_in_map') is None
                              else '推定姿勢が地図内に戻りました')
        self.state_log.update(
            'pose_in_map', in_map,
            in_map_message if in_map else '推定姿勢が地図の範囲外です',
            level='info' if in_map else 'warn')

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

    # ------------------------------------------------------------------
    # 保存サービス
    # ------------------------------------------------------------------
    def _on_save_overlay_request(self, request, response):
        num_images = request.num_images if request.num_images > 0 else 1
        interval = float(request.interval_sec)

        self.get_logger().info(
            f'保存要求を受信 (num_images={num_images}, interval_sec={interval})')

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

        # 連続保存中に毎枚同じ警告が出ないよう、カメラ画像の有無は変化時のみログを出す
        camera_img = None
        if self.latest_camera_msg is not None:
            try:
                camera_img = self.cv_bridge.imgmsg_to_cv2(
                    self.latest_camera_msg, desired_encoding='bgr8')
                self.state_log.update(
                    'save_camera', 'ok', '保存: カメラ画像も保存します')
            except Exception as e:
                self.state_log.update(
                    'save_camera', ('error', str(e)),
                    f'カメラ画像の変換に失敗したため、オーバーレイ画像のみ保存します: {e}',
                    level='error')
        else:
            self.state_log.update(
                'save_camera', 'missing',
                'カメラ画像未受信のため、オーバーレイ画像のみ保存します', level='warn')

        success, message = self.saver.save(self.latest_overlay, camera_img)
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)
        return success, message


def main(args=None) -> None:
    # rclpyのSIGINTハンドラはcontextを先に閉じてしまい「終了」ログを出せないため無効化し、
    # KeyboardInterruptで抜けてからshutdownする
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Superposition()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('終了')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
