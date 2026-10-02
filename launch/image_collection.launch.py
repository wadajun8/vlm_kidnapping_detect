import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # superpositionのパラメータファイル（config/superposition.yaml）
    superposition_params = os.path.join(
        get_package_share_directory('vlm_kidnapping_detect'), 'config', 'superposition.yaml')

    # 1つ目のノード：地図とスキャンの重畳処理
    superposition_node = Node(
        package='vlm_kidnapping_detect',
        executable='superposition',  # CMakeLists.txtのRENAMEで指定した名前
        name='superposition_node',   # 実行時のノード名
        output='screen',             # ログをターミナルに表示する設定
        parameters=[superposition_params],
    )

    # 2つ目のノード：カメラサーバー
    camera_server_node = Node(
        package='vlm_kidnapping_detect',
        executable='camera_server',  # CMakeLists.txtのRENAMEで指定した名前
        name='camera_server_node',
        output='screen',
    )

    # ※今後3つ目のノード（vlm_client）も同時起動したくなった場合は、
    # 同様にNode(...)を定義して、下のリストに追加するだけでOKです。

    # 起動するノードをリストにして返す
    return LaunchDescription([
        superposition_node,
        camera_server_node,
    ])
