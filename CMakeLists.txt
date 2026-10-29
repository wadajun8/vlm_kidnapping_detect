cmake_minimum_required(VERSION 3.8)
project(vlm_kidnapping_detect)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# ※ find_package(ament_cmake_python REQUIRED) は削除

# カスタムサービスのビルド
# （ここで自動的にPython用のサービスモジュールも生成されます）
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveOverlayImage.srv"
)

# ※ ament_python_install_package(${PROJECT_NAME}) は削除（これが衝突の原因）

# ノード（実行ファイル）のインストール
# RENAMEを使うと--symlink-installでもコピーになり、ノードの変更に再ビルドが必要になる。
# そのため元のファイル名でインストールし、実行ファイル名はそこへのシンボリックリンクにする
#   lib/<pkg>/camera_server -> camera_server_node.py -> (symlink-install時) src内のファイル
set(NODES
  "superposition:vlm_kidnapping_detect/superposition/map_scan_superposition.py"
  "camera_server:vlm_kidnapping_detect/camera_server/camera_server_node.py"
  "collect_dataset:data_collection/collect_dataset.py"
  "record_environment:data_collection/record_environment.py"
)
foreach(node IN LISTS NODES)
  string(REPLACE ":" ";" node "${node}")
  list(GET node 0 exec_name)
  list(GET node 1 script)
  get_filename_component(script_name ${script} NAME)
  install(PROGRAMS ${script} DESTINATION lib/${PROJECT_NAME})
  install(CODE "execute_process(COMMAND \${CMAKE_COMMAND} -E create_symlink
    ${script_name} \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/${exec_name})")
endforeach()

# launchファイル
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# パラメータファイル
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# データセット収集（data_collection/）の launch・パラメータ
install(DIRECTORY data_collection/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY data_collection/config/
  DESTINATION share/${PROJECT_NAME}/config
)

# リソースファイルのインストール
install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)
ament_index_register_resource(${PROJECT_NAME} CONTENT "")

ament_package()
