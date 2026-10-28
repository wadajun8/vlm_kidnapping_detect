"""superposition（地図とスキャンの重畳）とカメラサーバを起動する

パラメータは config/ 以下の yaml で設定する（カメラの変更は config/camera_server.yaml）
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    def config(name: str) -> PathJoinSubstitution:
        return PathJoinSubstitution([FindPackageShare('vlm_kidnapping_detect'), 'config', name])

    return LaunchDescription([
        Node(
            package='vlm_kidnapping_detect',
            executable='superposition',
            name='superposition_node',
            output='screen',
            parameters=[config('superposition.yaml')],
        ),
        Node(
            package='vlm_kidnapping_detect',
            executable='camera_server',
            name='camera_server_node',
            output='screen',
            parameters=[config('camera_server.yaml')],
        ),
    ])
