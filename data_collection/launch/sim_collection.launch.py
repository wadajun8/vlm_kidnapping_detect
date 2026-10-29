"""データセット収集用のシミュレーション一式を起動する

Gazebo(iscas_museum) + nav2(amcl または emcl2) + superposition を起動する。
既存の raspicat_with_iscas_museum.launch.py との違い:
  - world に libgazebo_ros_state.so（world プラグイン）を挿入し、
    /gazebo/set_entity_state で本体を強制移動、/gazebo/get_entity_state で正解姿勢を取得できるようにする
  - diff_drive の odom を world 基準から encoder 基準に変える
    （world 基準だと本体を強制移動したとき odom も一緒に跳び、自己位置推定が追従して誘拐にならないため）
  - GUI と rviz は起動しない。実機用の camera_server も起動しない（Gazebo のカメラを使う）

使い方:
  ros2 launch vlm_kidnapping_detect sim_collection.launch.py localizer:=emcl2
"""

import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# localizer 毎の nav2 launch と superposition のパーティクル設定
LOCALIZERS = {
    'amcl': {
        'nav2_launch': 'sim_nav2.launch.py',
        'particle_topic': '/particle_cloud',
        'particle_msg_type': 'ParticleCloud',
    },
    'emcl2': {
        'nav2_launch': 'sim_nav2_emcl2.launch.py',
        'particle_topic': '/particlecloud',
        'particle_msg_type': 'PoseArray',
    },
}

WORLD_TAG = "<world name='default'>"
STATE_PLUGIN = """
    <plugin name='gazebo_ros_state' filename='libgazebo_ros_state.so'>
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
      <update_rate>20.0</update_rate>
    </plugin>"""

# スポーン位置（Gazebo world 座標）。iscas_museum_map は world (0, -2) を map 原点として作られているため、
# ここにスポーンすると localizer の初期姿勢 map (0, 0, 0) と一致する（world→map は y に +2.0）
SPAWN_WORLD_XY = ('0.0', '-2.0')

# diff_drive プラグインの odometry_source: 0=ENCODER, 1=WORLD(既定)
ENCODER_ODOM_TAG = '<publish_odom>true</publish_odom>'


def encoder_odom_robot_description() -> str:
    """raspicat の URDF を xacro で生成し、diff_drive の odom を encoder 基準に書き換える"""
    xacro_file = os.path.join(
        get_package_share_directory('raspicat_description'), 'urdf', 'raspicat.urdf.xacro')
    urdf = subprocess.run(
        ['xacro', xacro_file, 'lidar_frame:=lidar_link'],
        check=True, capture_output=True, text=True).stdout
    if urdf.count(ENCODER_ODOM_TAG) != 1:
        raise RuntimeError(f'URDF 内に {ENCODER_ODOM_TAG} が1つだけ存在する前提が崩れています')
    return urdf.replace(
        ENCODER_ODOM_TAG, ENCODER_ODOM_TAG + '\n        <odometry_source>0</odometry_source>')


def world_with_state_plugin() -> str:
    """iscas_museum.world に gazebo_ros_state を挿入した world を一時ファイルに書き、そのパスを返す"""
    world = os.path.join(
        get_package_share_directory('raspicat_gazebo'), 'worlds', 'iscas_museum.world')
    with open(world) as f:
        sdf = f.read()
    if sdf.count(WORLD_TAG) != 1:
        raise RuntimeError(f'world 内に {WORLD_TAG} が1つだけ存在する前提が崩れています')
    out = tempfile.NamedTemporaryFile(
        'w', prefix='iscas_museum_state_', suffix='.world', delete=False)
    with out:
        out.write(sdf.replace(WORLD_TAG, WORLD_TAG + STATE_PLUGIN))
    return out.name


def launch_setup(context):
    localizer = LaunchConfiguration('localizer').perform(context)
    if localizer not in LOCALIZERS:
        raise RuntimeError(f'localizer は {list(LOCALIZERS)} のいずれか: {localizer}')
    spec = LOCALIZERS[localizer]

    gazebo_ros_launch = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    raspicat_gazebo_launch = os.path.join(
        get_package_share_directory('raspicat_gazebo'), 'launch')
    navigation_launch = os.path.join(
        get_package_share_directory('raspicat_navigation'), 'launch')
    superposition_yaml = os.path.join(
        get_package_share_directory('vlm_kidnapping_detect'), 'config', 'superposition.yaml')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_launch, 'gzserver.launch.py')),
        launch_arguments={'world': world_with_state_plugin()}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': encoder_odom_robot_description(),
            'frame_prefix': '/',
            'use_sim_time': True,
        }])

    spawn_raspicat = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(raspicat_gazebo_launch, 'spawn_raspicat.launch.py')),
        launch_arguments={'x_pose': SPAWN_WORLD_XY[0], 'y_pose': SPAWN_WORLD_XY[1]}.items())

    raspicat_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(raspicat_gazebo_launch, 'raspicat_simulation.launch.py')),
        launch_arguments={'rviz': 'false'}.items())

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_launch, spec['nav2_launch'])),
        launch_arguments={'use_sim_time': 'True', 'use_rviz': 'false'}.items())

    superposition = Node(
        package='vlm_kidnapping_detect',
        executable='superposition',
        name='superposition_node',
        output='screen',
        parameters=[superposition_yaml, {
            'use_sim_time': True,
            'particle_topic': spec['particle_topic'],
            'particle_msg_type': spec['particle_msg_type'],
        }])

    # include 先で宣言された launch 引数はスコープを切らないと他の include へ漏れる
    # （gzserver.launch.py の params_file='' が nav2 の params_file を上書きしてしまう）
    return [GroupAction([action]) for action in
            (gzserver, robot_state_publisher, spawn_raspicat, raspicat_sim, nav2, superposition)]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'localizer', default_value='emcl2', description='amcl または emcl2'),
        OpaqueFunction(function=launch_setup),
    ])
