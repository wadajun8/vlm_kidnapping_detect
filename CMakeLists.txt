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
install(PROGRAMS
  vlm_kidnapping_detect/map_scan_superposition.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME superposition
)

# リソースファイルのインストール
install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)
ament_index_register_resource(${PROJECT_NAME} CONTENT "")

ament_package()
