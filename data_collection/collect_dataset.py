#!/usr/bin/env python3
"""シミュレータで通常時・誘拐時の重畳画像とカメラ画像を収集する

sim_collection.launch.py を起動した状態で、1条件（localizer × mode）分のエピソードを回す。

  mode=normal      : 誘拐を起こさず走行するだけ
  mode=teleport    : 走行中に Gazebo 上のロボット本体を強制移動する（本当の誘拐）
  mode=initialpose : 本体はそのままで、/initialpose で推定側だけ誤った姿勢へ移す

/vlm_context_image を受信するたびに、時刻が最も近いカメラ画像・正解姿勢（Gazebo）・推定姿勢（TF）を
対応付けて保存し、正解と推定の誤差からラベル（normal / ambiguous / kidnapped）を付ける。

使い方:
  ros2 run vlm_kidnapping_detect collect_dataset --localizer emcl2 --mode teleport
"""

import argparse
import csv
import json
import math
import random
import subprocess
import threading
import time
from collections import Counter, deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Deque, Dict, List, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

LOCALIZER_NODES = {'amcl': '/amcl', 'emcl2': '/emcl2'}
MODES = ('normal', 'teleport', 'initialpose')
CSV_FIELDS = [
    'frame', 'stamp', 't_episode', 'phase', 't_since_event', 'label',
    'err_pos_m', 'err_yaw_deg',
    'gt_x', 'gt_y', 'gt_yaw_deg', 'est_x', 'est_y', 'est_yaw_deg',
    'gt_dt', 'camera_dt', 'overlay', 'perspective',
]


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float  # [rad]

    def dist(self, other: 'Pose2D') -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def as_dict(self) -> Dict[str, float]:
        return {'x': round(self.x, 4), 'y': round(self.y, 4),
                'yaw_deg': round(math.degrees(self.yaw), 2)}


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw: float) -> Quaternion:
    return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class Frame2D:
    """world→map の2次元剛体変換"""

    def __init__(self, x: float, y: float, yaw: float) -> None:
        self.x, self.y, self.yaw = x, y, yaw

    def world_to_map(self, p: Pose2D) -> Pose2D:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(c * p.x - s * p.y + self.x, s * p.x + c * p.y + self.y,
                      normalize_angle(p.yaw + self.yaw))

    def map_to_world(self, p: Pose2D) -> Pose2D:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = p.x - self.x, p.y - self.y
        return Pose2D(c * dx + s * dy, -s * dx + c * dy, normalize_angle(p.yaw - self.yaw))


class FreeSpace:
    """マップの自由空間のうち、壁から一定距離以上離れ、ロボットの開始位置とつながった領域"""

    def __init__(self, msg: OccupancyGrid, min_clearance_m: float, seed_pose: Pose2D) -> None:
        info = msg.info
        self.res = info.resolution
        self.ox, self.oy = info.origin.position.x, info.origin.position.y
        grid = np.array(msg.data, dtype=np.int16).reshape(info.height, info.width)
        free = (grid == 0).astype(np.uint8)
        clearance = cv2.distanceTransform(free, cv2.DIST_L2, 5) * self.res
        safe = (clearance >= min_clearance_m).astype(np.uint8)
        _, labels = cv2.connectedComponents(safe)
        seed = self._cell(seed_pose.x, seed_pose.y)
        seed_label = labels[seed[1], seed[0]] if self._in_grid(seed, grid.shape) else 0
        if seed_label == 0:  # 開始位置が安全領域外なら最大の連結成分を使う
            counts = np.bincount(labels.ravel())
            counts[0] = 0
            seed_label = int(np.argmax(counts))
        self.mask = labels == seed_label
        self.cells = np.argwhere(self.mask)  # (row, col)

    def _cell(self, x: float, y: float) -> Tuple[int, int]:
        return int((x - self.ox) / self.res), int((y - self.oy) / self.res)

    @staticmethod
    def _in_grid(cell: Tuple[int, int], shape) -> bool:
        return 0 <= cell[0] < shape[1] and 0 <= cell[1] < shape[0]

    def contains(self, x: float, y: float) -> bool:
        cell = self._cell(x, y)
        return self._in_grid(cell, self.mask.shape) and bool(self.mask[cell[1], cell[0]])

    def sample(self, rng: random.Random) -> Pose2D:
        row, col = self.cells[rng.randrange(len(self.cells))]
        return Pose2D(float(self.ox + (col + 0.5) * self.res),
                      float(self.oy + (row + 0.5) * self.res), rng.uniform(-math.pi, math.pi))

    def sample_at_distance(self, rng: random.Random, center: Pose2D,
                           d_min: float, d_max: float = math.inf) -> Optional[Pose2D]:
        for _ in range(500):
            p = self.sample(rng)
            if d_min <= p.dist(center) <= d_max:
                return p
        return None


class EpisodeDiscarded(Exception):
    pass


class DatasetCollector(Node):

    def __init__(self, args: argparse.Namespace, config: dict) -> None:
        super().__init__('collect_dataset',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.args = args
        self.cfg = config
        w2m = config['world_to_map']
        self.frame = Frame2D(w2m['x'], w2m['y'], math.radians(w2m['yaw_deg']))
        self.bridge = CvBridge()
        group = ReentrantCallbackGroup()

        # コールバックは購読を作った直後から呼ばれうるため、共有する状態を先に用意する
        # 記録状態（_on_overlay と本処理のスレッドで共有）
        self.lock = threading.Lock()
        self.recording = False
        self.episode_dir: Optional[Path] = None
        self.episode_start: Optional[Time] = None
        self.event_time: Optional[Time] = None
        self.rows: List[dict] = []
        self.skipped: Counter = Counter()

        # ノードは main() の MultiThreadedExecutor で回すので、TF 用の別スレッドは作らない
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_msg: Optional[OccupancyGrid] = None
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos,
                                 callback_group=group)
        self.camera_buffer: Deque[Image] = deque(maxlen=30)
        self.create_subscription(Image, '/camera/color/image_raw', self.camera_buffer.append,
                                 qos_profile_sensor_data, callback_group=group)
        self.create_subscription(Image, '/vlm_context_image', self._on_overlay, 10,
                                 callback_group=group)

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.get_state_cli = self.create_client(
            GetEntityState, '/gazebo/get_entity_state', callback_group=group)
        self.set_state_cli = self.create_client(
            SetEntityState, '/gazebo/set_entity_state', callback_group=group)
        self.motor_cli = self.create_client(SetBool, '/motor_power', callback_group=group)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                       callback_group=group)
        self.goal_handle = None
        self.goal_future = None
        self.goal_result = None

        # 正解姿勢は get_entity_state を周期的に呼んで (時刻, 姿勢) を貯める
        self.gt_buffer: Deque[Tuple[Time, Pose2D]] = deque(maxlen=200)
        self._gt_pending = False
        self.create_timer(0.05, self._poll_ground_truth, callback_group=group)

    # ------------------------------------------------------------------
    # 受信
    # ------------------------------------------------------------------
    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def _poll_ground_truth(self) -> None:
        if self._gt_pending or not self.get_state_cli.service_is_ready():
            return
        self._gt_pending = True
        req = GetEntityState.Request(name=self.cfg['robot_entity'], reference_frame='world')
        self.get_state_cli.call_async(req).add_done_callback(self._on_ground_truth)

    def _on_ground_truth(self, future) -> None:
        self._gt_pending = False
        res = future.result()
        if res is None or not res.success:
            return
        p = res.state.pose
        world = Pose2D(p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self.gt_buffer.append((Time.from_msg(res.header.stamp), self.frame.world_to_map(world)))

    def _nearest_ground_truth(self, stamp: Time) -> Tuple[Optional[Pose2D], Optional[float]]:
        if not self.gt_buffer:
            return None, None
        t, pose = min(self.gt_buffer, key=lambda e: abs((e[0] - stamp).nanoseconds))
        return pose, (t - stamp).nanoseconds * 1e-9

    def _nearest_camera(self, stamp: Time) -> Tuple[Optional[Image], Optional[float]]:
        """superposition の _find_camera_msg と同じ方式"""
        if not self.camera_buffer:
            return None, None
        msg = min(list(self.camera_buffer),
                  key=lambda m: abs((Time.from_msg(m.header.stamp) - stamp).nanoseconds))
        diff = (Time.from_msg(msg.header.stamp) - stamp).nanoseconds * 1e-9
        if abs(diff) > self.cfg['camera_sync_tolerance_sec']:
            return None, diff
        return msg, diff

    def _estimated_pose(self, stamp: Optional[Time] = None) -> Optional[Pose2D]:
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', stamp if stamp is not None else Time(),
                timeout=Duration(seconds=0.5))
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return Pose2D(t.x, t.y, yaw_from_quaternion(q))

    def label_of(self, err_pos: float, err_yaw_deg: float) -> str:
        k, n = self.cfg['label']['kidnapped'], self.cfg['label']['normal']
        if err_pos > k['pos_m'] or err_yaw_deg > k['yaw_deg']:
            return 'kidnapped'
        if err_pos <= n['pos_m'] and err_yaw_deg <= n['yaw_deg']:
            return 'normal'
        return 'ambiguous'

    def _on_overlay(self, msg: Image) -> None:
        with self.lock:
            if not self.recording:
                return
            episode_dir, episode_start, event_time = (
                self.episode_dir, self.episode_start, self.event_time)
        stamp = Time.from_msg(msg.header.stamp)
        gt, gt_dt = self._nearest_ground_truth(stamp)
        est = self._estimated_pose(stamp)
        if gt is None or est is None or abs(gt_dt) > 0.2:
            self.skipped['no_ground_truth' if gt is None or abs(gt_dt) > 0.2
                         else 'no_estimate'] += 1
            return

        # 1080p のカメラ画像は届くのが遅れるため、重畳画像の時刻より後の画像が届くまで少し待ってから対応付ける
        end = time.monotonic() + 0.5
        while time.monotonic() < end and not (
                self.camera_buffer
                and Time.from_msg(self.camera_buffer[-1].header.stamp) >= stamp):
            time.sleep(0.01)
        camera_msg, camera_dt = self._nearest_camera(stamp)
        t_episode = (stamp - episode_start).nanoseconds * 1e-9
        name = f'{t_episode:06.1f}'
        overlay_name = f'{name}_overlay.png'
        cv2.imwrite(str(episode_dir / overlay_name),
                    self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
        perspective_name = ''
        if camera_msg is not None:
            perspective_name = f'{name}_perspective.png'
            cv2.imwrite(str(episode_dir / perspective_name),
                        self.bridge.imgmsg_to_cv2(camera_msg, desired_encoding='bgr8'))
        else:
            self.skipped['no_camera'] += 1

        err_pos = gt.dist(est)
        err_yaw = abs(math.degrees(normalize_angle(gt.yaw - est.yaw)))
        t_since_event = ((stamp - event_time).nanoseconds * 1e-9
                         if event_time is not None else None)
        row = {
            'stamp': round(stamp.nanoseconds * 1e-9, 3),
            't_episode': round(t_episode, 3),
            'phase': 'pre' if t_since_event is None or t_since_event < 0 else 'post',
            't_since_event': '' if t_since_event is None else round(t_since_event, 3),
            'label': self.label_of(err_pos, err_yaw),
            'err_pos_m': round(err_pos, 4), 'err_yaw_deg': round(err_yaw, 2),
            'gt_x': round(gt.x, 4), 'gt_y': round(gt.y, 4),
            'gt_yaw_deg': round(math.degrees(gt.yaw), 2),
            'est_x': round(est.x, 4), 'est_y': round(est.y, 4),
            'est_yaw_deg': round(math.degrees(est.yaw), 2),
            'gt_dt': round(gt_dt, 3),
            'camera_dt': '' if camera_dt is None else round(camera_dt, 3),
            'overlay': overlay_name, 'perspective': perspective_name,
        }
        with self.lock:
            if self.recording and self.episode_dir == episode_dir:  # 記録終了後に届いた分は捨てる
                row['frame'] = len(self.rows)
                self.rows.append(row)

    # ------------------------------------------------------------------
    # シミュレーション操作
    # ------------------------------------------------------------------
    def now(self) -> Time:
        return self.get_clock().now()

    def sleep_sim(self, sec: float, during=None) -> None:
        """シミュレーション時間で sec 秒待つ。during があれば待機中に 0.5秒毎に呼ぶ"""
        end = self.now() + Duration(seconds=sec)
        while rclpy.ok() and self.now() < end:
            if during is not None:
                during()
            time.sleep(0.05 if during is None else 0.5)

    def call(self, client, request, timeout: float = 5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'サービス {client.srv_name} がありません')
        future = client.call_async(request)
        end = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > end:
                raise RuntimeError(f'サービス {client.srv_name} が応答しません')
            time.sleep(0.01)
        return future.result()

    def ground_truth(self) -> Pose2D:
        for _ in range(100):
            if self.gt_buffer:
                return self.gt_buffer[-1][1]
            time.sleep(0.05)
        raise RuntimeError('正解姿勢を取得できません（/gazebo/get_entity_state）')

    def teleport(self, pose_map: Pose2D) -> None:
        w = self.frame.map_to_world(pose_map)
        state = EntityState(name=self.cfg['robot_entity'], reference_frame='world')
        state.pose.position.x, state.pose.position.y = w.x, w.y
        state.pose.orientation = quaternion_from_yaw(w.yaw)
        res = self.call(self.set_state_cli, SetEntityState.Request(state=state))
        if not res.success:
            raise RuntimeError('set_entity_state に失敗')
        self.gt_buffer.clear()

    def publish_initialpose(self, pose: Pose2D) -> None:
        std = self.cfg['initialpose_std']
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = pose.x, pose.y
        msg.pose.pose.orientation = quaternion_from_yaw(pose.yaw)
        cov = [0.0] * 36
        cov[0] = cov[7] = std['xy_m'] ** 2
        cov[35] = math.radians(std['yaw_deg']) ** 2
        msg.pose.covariance = cov
        self.initialpose_pub.publish(msg)

    def motor_on(self) -> None:
        # raspicat_sim は cmd_vel が60秒途切れるとモータを切るため、毎エピソード入れ直す
        self.call(self.motor_cli, SetBool.Request(data=True))

    def send_goal(self, pose: Pose2D) -> None:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose.x, pose.y
        goal.pose.pose.orientation = quaternion_from_yaw(pose.yaw)
        self.goal_handle = None
        self.goal_future = self.nav_client.send_goal_async(goal)
        self.goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        handle = future.result()
        if handle is not None and handle.accepted:
            self.goal_handle = handle
            self.goal_result = handle.get_result_async()

    def goal_active(self) -> bool:
        if self.goal_future is not None and not self.goal_future.done():
            return True
        if self.goal_handle is None:
            return False
        return not self.goal_result.done()

    def cancel_goal(self) -> None:
        if self.goal_handle is not None and not self.goal_result.done():
            future = self.goal_handle.cancel_goal_async()
            end = time.monotonic() + 3.0
            while not future.done() and time.monotonic() < end:
                time.sleep(0.01)
        self.goal_handle = None
        self.goal_future = None


class ConditionRunner:
    """1条件（localizer × mode）分のエピソードを実行し、結果を保存する"""

    def __init__(self, node: DatasetCollector, args: argparse.Namespace, cfg: dict) -> None:
        self.node = node
        self.args = args
        self.cfg = cfg
        self.condition = f'{args.localizer}_{args.mode}'
        root = Path(cfg['output_root']).expanduser() / args.date
        self.out_dir = root / self.condition
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.free: Optional[FreeSpace] = None
        self.rng: Optional[random.Random] = None

    def log(self, msg: str) -> None:
        self.node.get_logger().info(f'[{self.condition}] {msg}')

    # ------------------------------------------------------------------
    def wait_ready(self) -> None:
        self.log('シミュレータと nav2 の準備を待機中')
        end = time.monotonic() + 120.0
        while self.node.map_msg is None or not self.node.gt_buffer:
            if time.monotonic() > end:
                raise RuntimeError('/map または /gazebo/get_entity_state が使えません')
            time.sleep(0.2)
        # amcl は初期姿勢を受け取るまで map フレームを出さず、nav2 の起動がそこで止まるため、正解姿勢を与える
        end = time.monotonic() + 60.0
        while self.node._estimated_pose() is None:
            if time.monotonic() > end:
                raise RuntimeError('map→base_footprint の TF が出ません')
            self.node.publish_initialpose(self.node.ground_truth())
            time.sleep(1.0)
        if not self.node.nav_client.wait_for_server(timeout_sec=120.0):
            raise RuntimeError('navigate_to_pose アクションサーバがありません')
        self.free = FreeSpace(self.node.map_msg, self.cfg['min_clearance_m'],
                              self.node.ground_truth())
        self.log(f'自由空間 {len(self.free.cells)} セル（壁から {self.cfg["min_clearance_m"]}m 以上）')

    def dump_params(self) -> List[str]:
        params_dir = self.out_dir / 'params'
        params_dir.mkdir(exist_ok=True)
        nodes = [LOCALIZER_NODES[self.args.localizer]] + list(self.cfg['param_dump_nodes'])
        failed = []
        for node in nodes:
            name = node.strip('/').replace('/', '__')
            for _ in range(3):  # 起動直後などに応答しないことがあるため再試行する
                try:
                    res = subprocess.run(['ros2', 'param', 'dump', node],
                                         capture_output=True, text=True, timeout=30)
                except subprocess.TimeoutExpired:
                    continue
                if res.returncode == 0:
                    (params_dir / f'{name}.yaml').write_text(res.stdout)
                    break
            else:
                failed.append(node)
                self.node.get_logger().warn(f'{node} のパラメータを取得できませんでした')
        return failed

    def next_episode_index(self) -> int:
        existing = [int(p.name[3:]) for p in self.out_dir.glob('ep_*') if p.name[3:].isdigit()]
        return max(existing) + 1 if existing else 0

    # ------------------------------------------------------------------
    def sample_kidnap(self, gt: Pose2D) -> Tuple[str, Pose2D]:
        types = self.cfg['kidnap_types']
        for _ in range(100):
            kind = self.rng.choice(sorted(types))
            spec = types[kind]
            if kind == 'small':
                d = self.rng.uniform(*spec['distance_m'])
                th = self.rng.uniform(-math.pi, math.pi)
                target = Pose2D(gt.x + d * math.cos(th), gt.y + d * math.sin(th),
                                normalize_angle(gt.yaw + math.radians(
                                    self.rng.uniform(*spec['yaw_deg']))))
                if not self.free.contains(target.x, target.y):
                    continue
            elif kind == 'large':
                target = self.free.sample_at_distance(self.rng, gt, spec['min_distance_m'])
                if target is None:
                    continue
            else:  # rotation
                dyaw = math.radians(self.rng.uniform(*spec['abs_yaw_deg']))
                target = Pose2D(gt.x, gt.y,
                                normalize_angle(gt.yaw + self.rng.choice((-1, 1)) * dyaw))
            return kind, target
        raise EpisodeDiscarded('kidnap_target_not_found')

    def keep_navigating(self) -> None:
        if self.node.goal_active():
            return
        gt = self.node.ground_truth()
        goal = self.free.sample_at_distance(self.rng, gt, *self.cfg['goal_distance_m'])
        if goal is not None:
            self.node.send_goal(goal)
            self.goals_sent += 1

    def run_episode(self, index: int) -> dict:
        node, cfg = self.node, self.cfg
        episode_seed = f'{cfg["seed"]}-{self.args.localizer}-{self.args.mode}-{index}'
        self.rng = random.Random(episode_seed)
        self.goals_sent = 0
        info = {'episode': index, 'condition': self.condition,
                'localizer': self.args.localizer, 'mode': self.args.mode,
                'seed': episode_seed, 'wall_start': datetime.now().isoformat(timespec='seconds')}

        # 1. リセット: 開始姿勢へ移動し、正しい初期姿勢を与える
        node.cancel_goal()
        node.motor_on()
        start = self.free.sample(self.rng)
        node.teleport(start)
        node.sleep_sim(cfg['reset_settle_sec'])
        node.publish_initialpose(start)
        node.sleep_sim(cfg['initialpose_settle_sec'])
        info['start_pose'] = start.as_dict()

        # 2. ウォームアップ: 走行しながら重畳画像の履歴からリセット前の軌跡を追い出し、推定の収束を確認
        self.keep_navigating()
        node.sleep_sim(cfg['warmup_sec'], during=self.keep_navigating)
        end = node.now() + Duration(seconds=cfg['reset_timeout_sec'])
        while True:
            gt, est = node.ground_truth(), node._estimated_pose()
            if est is not None:
                err_yaw = abs(math.degrees(normalize_angle(gt.yaw - est.yaw)))
                if node.label_of(gt.dist(est), err_yaw) == 'normal':
                    break
            if node.now() > end:
                node.cancel_goal()
                raise EpisodeDiscarded('reset_not_converged')
            node.sleep_sim(0.5, during=self.keep_navigating)

        # 3. 記録開始
        episode_dir = self.out_dir / f'ep_{index:03d}'
        episode_dir.mkdir(parents=True, exist_ok=False)
        with node.lock:
            node.rows = []
            node.skipped = Counter()
            node.episode_dir = episode_dir
            node.episode_start = node.now()
            node.event_time = None
            node.recording = True
        sim_start, wall_start = node.now(), time.monotonic()
        node.sleep_sim(cfg['pre_event_sec'], during=self.keep_navigating)

        # 4. イベント
        gt_before, est_before = node.ground_truth(), node._estimated_pose()
        event = {'t_episode': round((node.now() - node.episode_start).nanoseconds * 1e-9, 3),
                 'gt_before': gt_before.as_dict(),
                 'est_before': est_before.as_dict() if est_before else None}
        if self.args.mode != 'normal':
            kind, target = self.sample_kidnap(gt_before)
            if self.args.mode == 'teleport':
                node.teleport(target)
            else:
                node.publish_initialpose(target)
            event.update({'kidnap_type': kind, 'target': target.as_dict(),
                          'displacement_m': round(target.dist(gt_before), 3),
                          'rotation_deg': round(abs(math.degrees(
                              normalize_angle(target.yaw - gt_before.yaw))), 2)})
        with node.lock:
            node.event_time = node.now()
        info['event'] = event

        # 5. 誘拐後
        node.sleep_sim(cfg['post_event_sec'], during=self.keep_navigating)
        with node.lock:
            node.recording = False
            rows = list(node.rows)
            skipped = dict(node.skipped)
        node.cancel_goal()

        self.write_metadata(episode_dir, rows)
        sim_elapsed = (node.now() - sim_start).nanoseconds * 1e-9
        wall_elapsed = time.monotonic() - wall_start
        info.update({
            'frames': len(rows),
            'label_counts': dict(Counter(r['label'] for r in rows)),
            'skipped_frames': skipped,
            'goals_sent': self.goals_sent,
            'recorded_sim_sec': round(sim_elapsed, 2),
            'recorded_wall_sec': round(wall_elapsed, 2),
            'real_time_factor': round(sim_elapsed / wall_elapsed, 3) if wall_elapsed else None,
        })
        (episode_dir / 'episode.json').write_text(json.dumps(info, ensure_ascii=False, indent=2))
        return info

    @staticmethod
    def write_metadata(episode_dir: Path, rows: List[dict]) -> None:
        with open(episode_dir / 'metadata.csv', 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            writer.writeheader()
            writer.writerows(rows)

    # ------------------------------------------------------------------
    def run(self) -> None:
        self.wait_ready()
        param_dump_failed = self.dump_params()
        run_info = {
            'condition': self.condition, 'localizer': self.args.localizer, 'mode': self.args.mode,
            'wall_start': datetime.now().isoformat(timespec='seconds'),
            'config': self.cfg, 'episodes_requested': self.args.episodes,
            'free_space_cells': int(len(self.free.cells)),
            'param_dump_failed': param_dump_failed,
            'episodes': [], 'discarded': [],
        }
        index = self.next_episode_index()
        done = 0
        attempts = 0
        while done < self.args.episodes and rclpy.ok():
            attempts += 1
            if attempts > self.args.episodes * 3:
                self.node.get_logger().error('破棄が多すぎるため中断します')
                break
            try:
                info = self.run_episode(index)
            except EpisodeDiscarded as e:
                self.log(f'エピソード {index} を破棄 ({e})')
                run_info['discarded'].append({'episode': index, 'reason': str(e)})
                continue
            run_info['episodes'].append(info)
            self.log(f'エピソード {index} 完了: {info["frames"]}枚 {info["label_counts"]}')
            index += 1
            done += 1
            self.write_run_info(run_info)
        run_info['wall_end'] = datetime.now().isoformat(timespec='seconds')
        self.write_run_info(run_info)
        self.log(f'{done}エピソード収集完了 -> {self.out_dir}')

    def write_run_info(self, run_info: dict) -> None:
        # 再実行で追記した場合に前回分を消さないよう、実行毎に別ファイルにする
        path = self.out_dir / f'run_info_{run_info["wall_start"].replace(":", "")}.yaml'
        path.write_text(yaml.safe_dump(run_info, allow_unicode=True, sort_keys=False))


def load_config(path: Optional[str]) -> dict:
    if path is None:
        path = str(Path(get_package_share_directory('vlm_kidnapping_detect'))
                   / 'config' / 'collection.yaml')
    with open(path) as f:
        return yaml.safe_load(f)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--localizer', required=True, choices=sorted(LOCALIZER_NODES))
    parser.add_argument('--mode', required=True, choices=MODES)
    parser.add_argument('--episodes', type=int, default=None,
                        help='エピソード数（省略時は collection.yaml の episodes）')
    parser.add_argument('--date', default=datetime.now().strftime('%Y_%m%d'),
                        help='日付ディレクトリ名（既定: 今日 YYYY_MMDD）')
    parser.add_argument('--config', default=None, help='collection.yaml のパス')
    args, ros_args = parser.parse_known_args()
    cfg = load_config(args.config)
    if args.episodes is None:
        args.episodes = cfg['episodes']

    rclpy.init(args=ros_args)
    node = DatasetCollector(args, cfg)
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        ConditionRunner(node, args, cfg).run()
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_goal()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
