#!/usr/bin/env python3
"""論文に書きやすいよう、実行環境・実験条件・収集結果を YAML と Markdown で書き出す

  env         実行環境（OS・CPU/GPU・ROS 2/Gazebo・関連リポジトリのコミット）を日付ディレクトリへ追記
  conditions  条件ディレクトリの実験条件（collect_dataset の設定と、実際に動いていたノードのパラメータ）
  summary     条件ディレクトリの収集結果の集計
  summary-all 日付ディレクトリ内の全条件をまとめた表

使い方:
  ros2 run vlm_kidnapping_detect record_environment env --date-dir <日付ディレクトリ>
  ros2 run vlm_kidnapping_detect record_environment conditions --condition-dir <条件ディレクトリ>
  ros2 run vlm_kidnapping_detect record_environment summary --condition-dir <条件ディレクトリ>
  ros2 run vlm_kidnapping_detect record_environment summary-all --date-dir <日付ディレクトリ>
"""

import argparse
import csv
import json
import math
import os
import platform
import statistics
import subprocess
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import cv2
import yaml

# 実行環境として記録するリポジトリ（ワークスペースの src/ 以下）
REPOS = [
    'vlm_kidnapping_detect', 'emcl2_ros2', 'navigation2', 'raspicat_sim',
    'raspicat_description', 'raspicat_slam_navigation', 'raspicat_ros',
]

# 論文に載せる主なパラメータ（params/<ノード>.yaml 内のキー）
KEY_PARAMS = {
    'amcl': [
        'min_particles', 'max_particles', 'laser_model_type', 'max_beams',
        'recovery_alpha_fast', 'recovery_alpha_slow', 'update_min_d', 'update_min_a',
        'alpha1', 'alpha2', 'alpha3', 'alpha4', 'alpha5', 'sigma_hit', 'z_hit', 'z_rand',
    ],
    'emcl2': [
        'num_particles', 'alpha_threshold', 'expansion_radius_position',
        'expansion_radius_orientation', 'extraction_rate', 'range_threshold', 'sensor_reset',
        'laser_likelihood_max_dist', 'odom_fw_dev_per_fw', 'odom_fw_dev_per_rot',
        'odom_rot_dev_per_fw', 'odom_rot_dev_per_rot',
    ],
    'superposition_node': [
        'capture_interval_sec', 'snapshot_count', 'show_particles', 'show_laser_scan',
        'show_best_pose', 'best_pose_radius', 'laser_point_radius', 'laser_point_color',
        'particle_topic', 'particle_msg_type', 'camera_sync_tolerance_sec',
    ],
    'controller_server': [
        'controller_frequency', 'FollowPath.plugin', 'FollowPath.max_vel_x',
        'FollowPath.max_vel_theta', 'FollowPath.acc_lim_x',
    ],
    'velocity_smoother': ['max_velocity', 'max_accel'],
    'planner_server': ['GridBased.plugin'],
}


# ----------------------------------------------------------------------
# 共通
# ----------------------------------------------------------------------
def run(cmd: List[str]) -> str:
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=30).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return ''


def write_yaml(path: Path, data: Any) -> None:
    path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))


def md_table(rows: List[List[Any]], header: List[str]) -> str:
    lines = ['| ' + ' | '.join(header) + ' |', '| ' + ' | '.join('---' for _ in header) + ' |']
    lines += ['| ' + ' | '.join('' if v is None else str(v) for v in row) + ' |' for row in rows]
    return '\n'.join(lines)


def load_params(params_dir: Path, name: str) -> Dict[str, Any]:
    path = params_dir / f'{name}.yaml'
    if not path.exists():
        return {}
    data = yaml.safe_load(path.read_text()) or {}
    node = next(iter(data.values()), {})
    return node.get('ros__parameters', {})


def get_nested(params: Dict[str, Any], key: str) -> Any:
    value: Any = params
    for part in key.split('.'):
        if not isinstance(value, dict) or part not in value:
            return None
        value = value[part]
    return value


def latest_run_info(condition_dir: Path) -> Dict[str, Any]:
    runs = sorted(condition_dir.glob('run_info_*.yaml'))
    if not runs:
        raise SystemExit(f'{condition_dir} に run_info_*.yaml がありません（collect_dataset 未実行）')
    return yaml.safe_load(runs[-1].read_text())


def workspace_src() -> Path:
    # <ws>/install/vlm_kidnapping_detect → <ws>/src
    return Path(get_package_prefix('vlm_kidnapping_detect')).parent.parent / 'src'


# ----------------------------------------------------------------------
# env: 実行環境
# ----------------------------------------------------------------------
def git_info(path: Path) -> Dict[str, Any]:
    # src/ 以下はワークスペース全体で1つのリポジトリになっているものもあるため、上位のリポジトリも辿る
    toplevel = run(['git', '-C', str(path), 'rev-parse', '--show-toplevel'])
    if not toplevel:
        return {'path': str(path), 'git': False}
    return {
        'path': str(path),
        'repository_root': toplevel,
        'remote': run(['git', '-C', str(path), 'remote', 'get-url', 'origin']),
        'branch': run(['git', '-C', str(path), 'rev-parse', '--abbrev-ref', 'HEAD']),
        'commit': run(['git', '-C', str(path), 'rev-parse', 'HEAD']),
        'commit_date': run(['git', '-C', str(path), 'log', '-1', '--format=%ci']),
        # .pyc などのビルド生成物は差分扱いしない
        'uncommitted_changes': [
            line for line in run(['git', '-C', str(path), 'status', '--short', '--', '.'])
            .splitlines() if not line.endswith('.pyc')],
    }


def os_release() -> str:
    try:
        for line in Path('/etc/os-release').read_text().splitlines():
            if line.startswith('PRETTY_NAME='):
                return line.split('=', 1)[1].strip('"')
    except OSError:
        pass
    return platform.platform()


def cpu_model() -> str:
    for line in run(['lscpu']).splitlines():
        if line.startswith('Model name') or line.startswith('モデル名'):
            return line.split(':', 1)[1].strip()
    return platform.processor()


def mem_total_gb() -> Optional[float]:
    try:
        for line in Path('/proc/meminfo').read_text().splitlines():
            if line.startswith('MemTotal:'):
                return round(int(line.split()[1]) / 1024 / 1024, 1)
    except OSError:
        pass
    return None


def collect_environment(localizer: Optional[str]) -> Dict[str, Any]:
    src = workspace_src()
    gpus = [line.split(': ', 1)[-1] for line in run(['lspci']).splitlines()
            if 'VGA' in line or '3D controller' in line]
    return {
        'recorded_at': datetime.now().isoformat(timespec='seconds'),
        'localizer': localizer,
        'machine': {
            'hostname': platform.node(),
            'os': os_release(),
            'kernel': platform.release(),
            'cpu': cpu_model(),
            'cpu_threads': os.cpu_count(),
            'memory_gb': mem_total_gb(),
            'gpus': gpus,
            'nvidia_driver': run(['cat', '/proc/driver/nvidia/version']).split('\n')[0],
            # NVIDIA のカーネルモジュールとライブラリの版が食い違うと GLX が使えないため mesa で描画することがある
            'opengl_vendor_library': os.environ.get('__GLX_VENDOR_LIBRARY_NAME', '(default)'),
        },
        'software': {
            'ros_distro': os.environ.get('ROS_DISTRO'),
            'gazebo': run(['gzserver', '--version']).split('\n')[0],
            'python': sys.version.split()[0],
            'opencv': cv2.__version__,
        },
        'repositories': {name: git_info(src / name) for name in REPOS if (src / name).exists()},
    }


def environment_markdown(records: List[Dict[str, Any]]) -> str:
    out = ['# 実行環境', '']
    for rec in records:
        m, s = rec['machine'], rec['software']
        out += [f'## {rec["recorded_at"]}（localizer: {rec.get("localizer") or "-"}）', '',
                md_table([
                    ['OS', m['os']], ['カーネル', m['kernel']],
                    ['CPU', f'{m["cpu"]}（{m["cpu_threads"]} スレッド）'],
                    ['メモリ', f'{m["memory_gb"]} GB'], ['GPU', ' / '.join(m['gpus'])],
                    ['NVIDIA ドライバ', m['nvidia_driver']],
                    ['OpenGL ベンダライブラリ', m['opengl_vendor_library']],
                    ['ROS 2', s['ros_distro']], ['Gazebo', s['gazebo']],
                    ['Python', s['python']], ['OpenCV', s['opencv']],
                ], ['項目', '値']), '', '### リポジトリ', '',
                md_table([
                    [name, r.get('branch'), (r.get('commit') or '')[:10], r.get('commit_date'),
                     len(r.get('uncommitted_changes', [])) or 'なし']
                    for name, r in rec['repositories'].items()
                ], ['リポジトリ', 'ブランチ', 'コミット', 'コミット日時', '未コミットの変更']), '']
    return '\n'.join(out)


def cmd_env(args: argparse.Namespace) -> None:
    date_dir = Path(args.date_dir).expanduser()
    date_dir.mkdir(parents=True, exist_ok=True)
    path = date_dir / 'experiment_environment.yaml'
    data = yaml.safe_load(path.read_text()) if path.exists() else {'records': []}
    data['records'].append(collect_environment(args.localizer))
    write_yaml(path, data)
    (date_dir / 'experiment_environment.md').write_text(environment_markdown(data['records']))
    print(f'実行環境を記録: {path}')


# ----------------------------------------------------------------------
# conditions: 実験条件
# ----------------------------------------------------------------------
def robot_spec() -> Dict[str, Any]:
    """シミュレーションで使ったロボットモデルのセンサ・駆動の仕様を URDF から読む"""
    xacro_file = Path(get_package_share_directory('raspicat_description')) / 'urdf' / \
        'raspicat.urdf.xacro'
    root = ET.fromstring(run(['xacro', str(xacro_file), 'lidar_frame:=lidar_link']))

    def text(elem, path, cast=float):
        found = elem.find(path)
        return cast(found.text) if found is not None and found.text else None

    def joint_origin(child: str) -> Optional[Dict[str, Any]]:
        for joint in root.iter('joint'):
            c = joint.find('child')
            if c is not None and c.get('link') == child:
                origin = joint.find('origin')
                return {'parent': joint.find('parent').get('link'),
                        'xyz': origin.get('xyz') if origin is not None else None,
                        'rpy': origin.get('rpy') if origin is not None else None}
        return None

    spec: Dict[str, Any] = {}
    for sensor in root.iter('sensor'):
        if sensor.get('type') == 'ray' and sensor.get('name') == 'lidar_link_sensor':
            spec['lidar'] = {
                'update_rate_hz': text(sensor, 'update_rate'),
                'samples': text(sensor, 'ray/scan/horizontal/samples', int),
                'min_angle_rad': text(sensor, 'ray/scan/horizontal/min_angle'),
                'max_angle_rad': text(sensor, 'ray/scan/horizontal/max_angle'),
                'range_min_m': text(sensor, 'ray/range/min'),
                'range_max_m': text(sensor, 'ray/range/max'),
                'noise_stddev_m': text(sensor, 'ray/noise/stddev'),
                'mount': joint_origin('lidar_link'),
            }
        elif sensor.get('type') == 'camera' and sensor.get('name') == 'camera':
            spec['camera'] = {
                'update_rate_hz': text(sensor, 'update_rate'),
                'width': text(sensor, 'camera/image/width', int),
                'height': text(sensor, 'camera/image/height', int),
                'horizontal_fov_rad': text(sensor, 'camera/horizontal_fov'),
                'format': text(sensor, 'camera/image/format', str),
                'noise_stddev': text(sensor, 'camera/noise/stddev'),
                'mount': joint_origin('camera_link'),
            }
    for plugin in root.iter('plugin'):
        if plugin.get('filename') == 'libgazebo_ros_diff_drive.so':
            spec['drive'] = {
                'wheel_separation_m': text(plugin, 'wheel_separation'),
                'wheel_diameter_m': text(plugin, 'wheel_diameter'),
                'odometry_source': 'encoder（sim_collection.launch.py で world 基準から変更）',
            }
    return spec


def map_spec(params_dir: Path) -> Dict[str, Any]:
    yaml_file = load_params(params_dir, 'map_server').get('yaml_filename')
    if not yaml_file or not Path(yaml_file).exists():
        return {'yaml': yaml_file}
    info = yaml.safe_load(Path(yaml_file).read_text())
    image = Path(yaml_file).parent / info['image']
    img = cv2.imread(str(image), cv2.IMREAD_UNCHANGED)
    h, w = img.shape[:2] if img is not None else (None, None)
    res = info['resolution']
    return {'yaml': yaml_file, 'resolution_m': res, 'origin': info['origin'],
            'size_px': [w, h], 'size_m': [round(w * res, 2), round(h * res, 2)] if w else None}


def reset_setting(localizer: str, params: Dict[str, Any]) -> str:
    if localizer == 'emcl2':
        return (f'膨張リセット（alpha_threshold={params.get("alpha_threshold")}, '
                f'sensor_reset={params.get("sensor_reset")}）。既存のパラメータのまま')
    fast, slow = params.get('recovery_alpha_fast'), params.get('recovery_alpha_slow')
    injection = 'なし' if not fast and not slow else 'あり'
    return (f'recovery_alpha_fast={fast}, recovery_alpha_slow={slow}'
            f'（ランダムパーティクル注入{injection}）。既存のパラメータのまま')


def collect_conditions(condition_dir: Path) -> Dict[str, Any]:
    run_info = latest_run_info(condition_dir)
    cfg = run_info['config']
    params_dir = condition_dir / 'params'
    localizer = run_info['localizer']
    key_params = {
        node: {k: get_nested(load_params(params_dir, node), k) for k in keys}
        for node, keys in KEY_PARAMS.items()
        if node not in ('amcl', 'emcl2') or node == localizer
    }
    return {
        'condition': run_info['condition'],
        'localizer': localizer,
        'mode': run_info['mode'],
        'mode_description': {
            'normal': '誘拐を起こさず自律走行のみ',
            'teleport': '走行中に Gazebo 上のロボット本体を強制移動（/gazebo/set_entity_state）',
            'initialpose': '本体はそのままで、/initialpose に誤った姿勢を送り推定側だけを移動',
        }[run_info['mode']],
        'reset_setting': reset_setting(localizer, load_params(params_dir, localizer)),
        'simulation': {
            'simulator': 'Gazebo Classic',
            'world': 'iscas_museum.world（raspicat_gazebo）',
            'map': map_spec(params_dir),
            'world_to_map': cfg['world_to_map'],
            'robot': 'Raspberry Pi Cat（raspicat_description）',
            'robot_spec': robot_spec(),
            'navigation': 'Navigation2（raspicat_navigation の sim_nav2 / sim_nav2_emcl2 の設定）',
        },
        'procedure': {
            'episodes': run_info['episodes_requested'],
            'seed': cfg['seed'],
            'reset_settle_sec': cfg['reset_settle_sec'],
            'initialpose_settle_sec': cfg['initialpose_settle_sec'],
            'warmup_sec': cfg['warmup_sec'],
            'pre_event_sec': cfg['pre_event_sec'],
            'post_event_sec': cfg['post_event_sec'],
            'reset_timeout_sec': cfg['reset_timeout_sec'],
            'start_and_goal_selection':
                f'壁から {cfg["min_clearance_m"]} m 以上離れた自由空間から一様に選択。ゴールは現在地から '
                f'{cfg["goal_distance_m"][0]}〜{cfg["goal_distance_m"][1]} m、到達・失敗したら次を送る',
            'initialpose_std': cfg['initialpose_std'],
        },
        'kidnap': cfg['kidnap_types'] if run_info['mode'] != 'normal' else None,
        'label_thresholds': cfg['label'],
        'camera_sync_tolerance_sec': cfg['camera_sync_tolerance_sec'],
        'ground_truth': 'Gazebo の /gazebo/get_entity_state（20Hz で取得し、重畳画像の時刻に最も近い値）',
        'estimate': 'TF map→base_footprint（重畳画像の時刻で lookup）',
        'key_parameters': key_params,
        'param_dump_failed': run_info.get('param_dump_failed', []),
    }


def fov_deg(rad: Optional[float]) -> Optional[float]:
    return None if rad is None else round(math.degrees(rad), 1)


def conditions_markdown(c: Dict[str, Any]) -> str:
    sim, proc, spec = c['simulation'], c['procedure'], c['simulation']['robot_spec']
    lidar, cam, drive = spec.get('lidar', {}), spec.get('camera', {}), spec.get('drive', {})
    label = c['label_thresholds']
    out = [f'# 実験条件: {c["condition"]}', '',
           md_table([
               ['自己位置推定', c['localizer']], ['リセット', c['reset_setting']],
               ['条件', f'{c["mode"]}（{c["mode_description"]}）'],
               ['シミュレータ', sim['simulator']], ['環境', sim['world']],
               ['地図', f'{sim["map"].get("size_m")} m, 解像度 {sim["map"].get("resolution_m")} m/px'],
               ['ロボット', sim['robot']], ['ナビゲーション', sim['navigation']],
               ['正解姿勢', c['ground_truth']], ['推定姿勢', c['estimate']],
           ], ['項目', '内容']), '',
           '## センサ・駆動（シミュレーションモデル）', '',
           md_table([
               ['LiDAR', f'{lidar.get("update_rate_hz")} Hz, {lidar.get("samples")} 点, '
                         f'{lidar.get("min_angle_rad")}〜{lidar.get("max_angle_rad")} rad, '
                         f'{lidar.get("range_min_m")}〜{lidar.get("range_max_m")} m, '
                         f'ノイズ σ={lidar.get("noise_stddev_m")} m'],
               ['カメラ',
                f'{cam.get("width")}×{cam.get("height")}, {cam.get("update_rate_hz")} fps, '
                f'水平画角 {fov_deg(cam.get("horizontal_fov_rad"))}°'],
               ['駆動',
                f'差動二輪, トレッド {drive.get("wheel_separation_m")} m, '
                f'車輪径 {drive.get("wheel_diameter_m")} m, odom: {drive.get("odometry_source")}'],
           ], ['', '仕様']), '',
           '## 手順', '',
           md_table([
               ['エピソード数', proc['episodes']], ['乱数 seed', proc['seed']],
               ['ウォームアップ（保存しない）', f'{proc["warmup_sec"]} s'],
               ['誘拐前の記録', f'{proc["pre_event_sec"]} s'],
               ['誘拐後の記録', f'{proc["post_event_sec"]} s'],
               ['開始・ゴールの選び方', proc['start_and_goal_selection']],
               ['/initialpose の標準偏差',
                f'{proc["initialpose_std"]["xy_m"]} m, {proc["initialpose_std"]["yaw_deg"]}°'],
               ['カメラ画像の同期許容差', f'{c["camera_sync_tolerance_sec"]} s'],
           ], ['項目', '値']), '']
    if c['kidnap']:
        k = c['kidnap']
        out += ['## 誘拐（種類は一様に選択、量は誘拐直前の正解姿勢からの相対量）', '',
                md_table([
                    ['small', f'{k["small"]["distance_m"][0]}〜{k["small"]["distance_m"][1]} m, '
                              f'{k["small"]["yaw_deg"][0]}〜{k["small"]["yaw_deg"][1]}°'],
                    ['large', f'{k["large"]["min_distance_m"]} m 以上離れた自由空間, 向きはランダム'],
                    ['rotation', f'位置そのまま, {k["rotation"]["abs_yaw_deg"][0]}〜'
                                 f'{k["rotation"]["abs_yaw_deg"][1]}°'],
                ], ['種類', '量']), '']
    out += ['## ラベル（正解と推定の誤差から毎フレーム自動付与）', '',
            md_table([
                ['kidnapped', f'位置 > {label["kidnapped"]["pos_m"]} m または '
                              f'角度 > {label["kidnapped"]["yaw_deg"]}°'],
                ['normal', f'位置 ≤ {label["normal"]["pos_m"]} m かつ '
                           f'角度 ≤ {label["normal"]["yaw_deg"]}°'],
                ['ambiguous', '上記以外'],
            ], ['ラベル', '条件']), '', '## 主なパラメータ（実際に動いていたノードの値）', '']
    for node, params in c['key_parameters'].items():
        out += [f'### {node}', '',
                md_table([[k, v] for k, v in params.items()], ['パラメータ', '値']), '']
    if c['param_dump_failed']:
        out += [f'取得できなかったノード: {", ".join(c["param_dump_failed"])}', '']
    return '\n'.join(out)


def cmd_conditions(args: argparse.Namespace) -> None:
    condition_dir = Path(args.condition_dir).expanduser()
    data = collect_conditions(condition_dir)
    write_yaml(condition_dir / 'experiment_conditions.yaml', data)
    (condition_dir / 'experiment_conditions.md').write_text(conditions_markdown(data))
    print(f'実験条件を記録: {condition_dir / "experiment_conditions.yaml"}')


# ----------------------------------------------------------------------
# summary: 収集結果の集計
# ----------------------------------------------------------------------
def stats(values: List[float]) -> Optional[Dict[str, float]]:
    if not values:
        return None
    return {'mean': round(statistics.mean(values), 3), 'max': round(max(values), 3),
            'min': round(min(values), 3)}


def episode_outcome(rows: List[dict]) -> Dict[str, Any]:
    """誘拐後に kidnapped になったか、その後 normal に戻り続けたか（回復）とその時刻"""
    post = [r for r in rows if r['phase'] == 'post']
    kidnapped = [i for i, r in enumerate(post) if r['label'] == 'kidnapped']
    if not kidnapped:
        return {'kidnapped': False, 'recovered': None, 'recovery_sec': None}
    recovery_sec = None
    for i in range(kidnapped[0], len(post)):
        if all(r['label'] == 'normal' for r in post[i:]):
            recovery_sec = float(post[i]['t_since_event'])
            break
    return {'kidnapped': True, 'recovered': recovery_sec is not None, 'recovery_sec': recovery_sec}


def collect_summary(condition_dir: Path) -> Dict[str, Any]:
    run_infos = [yaml.safe_load(p.read_text())
                 for p in sorted(condition_dir.glob('run_info_*.yaml'))]
    labels_all, labels_pre, labels_post = Counter(), Counter(), Counter()
    kinds = Counter()
    displacement, rotation, err_after, err_pre, rtf, recovery = [], [], [], [], [], []
    outcomes = Counter()
    frames = missing_camera = 0
    episodes = sorted(condition_dir.glob('ep_*'))
    for ep in episodes:
        info = json.loads((ep / 'episode.json').read_text())
        with open(ep / 'metadata.csv') as f:
            rows = list(csv.DictReader(f))
        frames += len(rows)
        missing_camera += sum(1 for r in rows if not r['perspective'])
        for r in rows:
            labels_all[r['label']] += 1
            (labels_pre if r['phase'] == 'pre' else labels_post)[r['label']] += 1
            if r['phase'] == 'pre':
                err_pre.append(float(r['err_pos_m']))
        post = [r for r in rows if r['phase'] == 'post']
        if post:
            err_after.append(float(post[0]['err_pos_m']))
        event = info.get('event', {})
        if 'kidnap_type' in event:
            kinds[event['kidnap_type']] += 1
            displacement.append(event['displacement_m'])
            rotation.append(event['rotation_deg'])
        if info.get('real_time_factor'):
            rtf.append(info['real_time_factor'])
        outcome = episode_outcome(rows)
        if not outcome['kidnapped']:
            outcomes['not_kidnapped'] += 1
        elif outcome['recovered']:
            outcomes['recovered'] += 1
            recovery.append(outcome['recovery_sec'])
        else:
            outcomes['not_recovered'] += 1
    discarded = [d for r in run_infos for d in r.get('discarded', [])]
    walls = [(r.get('wall_start'), r.get('wall_end')) for r in run_infos]
    first = run_infos[0] if run_infos else {}
    return {
        'condition': first.get('condition', condition_dir.name),
        'episodes': len(episodes),
        'discarded': {'count': len(discarded),
                      'reasons': dict(Counter(d['reason'] for d in discarded))},
        'frames': frames,
        'labels': {'all': dict(labels_all), 'pre_event': dict(labels_pre),
                   'post_event': dict(labels_post)},
        'missing_perspective': missing_camera,
        'kidnap_types': dict(kinds),
        'displacement_m': stats(displacement),
        'rotation_deg': stats(rotation),
        'err_pos_first_post_frame_m': stats(err_after),
        'err_pos_pre_event_m': stats(err_pre),
        'episode_outcomes': dict(outcomes),
        'recovery_sec': stats(recovery),
        'real_time_factor': stats(rtf),
        'runs': [{'wall_start': s, 'wall_end': e} for s, e in walls],
    }


def fmt_stats(s: Optional[Dict[str, float]], unit: str = '') -> str:
    return '-' if not s else f'平均 {s["mean"]}{unit}（{s["min"]}〜{s["max"]}{unit}）'


def summary_markdown(s: Dict[str, Any]) -> str:
    lab = s['labels']

    def counts(c):
        return ', '.join(f'{k} {c.get(k, 0)}' for k in ('normal', 'ambiguous', 'kidnapped'))

    o = s['episode_outcomes']
    return '\n'.join([
        f'# 収集結果: {s["condition"]}', '',
        md_table([
            ['エピソード数', s['episodes']],
            ['破棄したエピソード', f'{s["discarded"]["count"]} {s["discarded"]["reasons"] or ""}'],
            ['フレーム（画像の組）数', s['frames']],
            ['ラベル（全体）', counts(lab['all'])],
            ['ラベル（誘拐前）', counts(lab['pre_event'])],
            ['ラベル（誘拐後）', counts(lab['post_event'])],
            ['カメラ画像が無いフレーム', s['missing_perspective']],
            ['誘拐の種類', s['kidnap_types'] or '-'],
            ['誘拐の移動距離', fmt_stats(s['displacement_m'], ' m')],
            ['誘拐の回転量', fmt_stats(s['rotation_deg'], '°')],
            ['誘拐前の位置誤差', fmt_stats(s['err_pos_pre_event_m'], ' m')],
            ['誘拐直後の位置誤差', fmt_stats(s['err_pos_first_post_frame_m'], ' m')],
            ['誘拐状態にならなかった', o.get('not_kidnapped', 0)],
            ['誘拐状態から回復', o.get('recovered', 0)],
            ['回復しなかった', o.get('not_recovered', 0)],
            ['回復までの時間', fmt_stats(s['recovery_sec'], ' s')],
            ['実時間係数', fmt_stats(s['real_time_factor'])],
        ], ['項目', '値']), ''])


def cmd_summary(args: argparse.Namespace) -> None:
    condition_dir = Path(args.condition_dir).expanduser()
    data = collect_summary(condition_dir)
    write_yaml(condition_dir / 'summary.yaml', data)
    (condition_dir / 'summary.md').write_text(summary_markdown(data))
    print(f'集計を記録: {condition_dir / "summary.yaml"}')


def cmd_summary_all(args: argparse.Namespace) -> None:
    date_dir = Path(args.date_dir).expanduser()
    rows, data = [], {}
    for condition_dir in sorted(p for p in date_dir.iterdir() if (p / 'summary.yaml').exists()):
        s = yaml.safe_load((condition_dir / 'summary.yaml').read_text())
        data[s['condition']] = s
        lab, o = s['labels']['all'], s['episode_outcomes']
        rows.append([
            s['condition'], s['episodes'], s['frames'], lab.get('normal', 0),
            lab.get('ambiguous', 0), lab.get('kidnapped', 0), s['missing_perspective'],
            o.get('not_kidnapped', 0), o.get('recovered', 0), o.get('not_recovered', 0),
            (s['recovery_sec'] or {}).get('mean', '-'),
        ])
    write_yaml(date_dir / 'summary_all.yaml', data)
    (date_dir / 'summary_all.md').write_text('\n'.join([
        f'# 収集結果一覧（{date_dir.name}）', '',
        md_table(rows, ['条件', 'エピソード', 'フレーム', 'normal', 'ambiguous', 'kidnapped',
                        'カメラ欠損', '誘拐にならず', '回復', '回復せず', '平均回復時間 [s]']),
        '', '各条件の詳細は <条件>/summary.md と <条件>/experiment_conditions.md、'
        '実行環境は experiment_environment.md を参照。', '']))
    print(f'全条件の集計を記録: {date_dir / "summary_all.md"}')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('env')
    p.add_argument('--date-dir', required=True)
    p.add_argument('--localizer', default=None)
    p.set_defaults(func=cmd_env)
    for name, func in (('conditions', cmd_conditions), ('summary', cmd_summary)):
        p = sub.add_parser(name)
        p.add_argument('--condition-dir', required=True)
        p.set_defaults(func=func)
    p = sub.add_parser('summary-all')
    p.add_argument('--date-dir', required=True)
    p.set_defaults(func=cmd_summary_all)
    args, _ = parser.parse_known_args()
    args.func(args)


if __name__ == '__main__':
    main()
