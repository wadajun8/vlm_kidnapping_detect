# vlm_kidnapping_detect

VLMを用いて誘拐ロボット問題を検知するROS2パッケージ。
現時点では、マップ・自己位置・LiDARを時系列で重畳した画像のパブリッシュまで実装済み。

## クイックスタート

```bash
colcon build --packages-select vlm_kidnapping_detect
source install/setup.bash
ros2 run vlm_kidnapping_detect superposition
```

パラメータを指定して起動する場合:

```bash
ros2 run vlm_kidnapping_detect superposition \
  --ros-args \
  -p capture_interval_sec:=2.0 \
  -p snapshot_count:=3
```

### パーティクルトピックの指定（AMCL / EMCL 対応）

**AMCL（デフォルト）を使用する場合:**

```bash
ros2 run vlm_kidnapping_detect superposition \
  --ros-args \
  -p particle_topic:='/particle_cloud' \
  -p particle_msg_type:='ParticleCloud'
```

**[emcl2](https://github.com/ryuichiueda/emcl2)（または PoseArray 型のパーティクル）を使用する場合:**

```bash
ros2 run vlm_kidnapping_detect superposition \
  --ros-args \
  -p particle_topic:=/particlecloud \
  -p particle_msg_type:=PoseArray
```

### 表示内容のカスタマイズ

各要素の表示/非表示をパラメータで制御できます：

```bash
# パーティクルのみ表示
ros2 run vlm_kidnapping_detect superposition \
  --ros-args \
  -p show_particles:=true \
  -p show_laser_scan:=false \
  -p show_best_pose:=false

# パーティクル + センサデータ表示
ros2 run vlm_kidnapping_detect superposition \
  --ros-args \
  -p show_particles:=true \
  -p show_laser_scan:=true \
  -p show_best_pose:=false

# パーティクル + 代表位置表示
ros2 run vlm_kidnapping_detect superposition \
  --ros-args \
  -p show_particles:=true \
  -p show_laser_scan:=false \
  -p show_best_pose:=true

# 全て表示（デフォルト）
ros2 run vlm_kidnapping_detect superposition \
  --ros-args \
  -p show_particles:=true \
  -p show_laser_scan:=true \
  -p show_best_pose:=true
```

**EMCL + パーティクル非表示の例:**

```bash
ros2 run vlm_kidnapping_detect superposition \
  --ros-args \
  -p particle_topic:='/particles' \
  -p particle_msg_type:='PoseArray' \
  -p show_particles:=false \
  -p show_laser_scan:=true \
  -p show_best_pose:=true
```

画像を保存する場合:

```bash
ros2 service call /save_overlay_image std_srvs/srv/Trigger
```

保存先: `/tmp/overlay_YYYYMMDD_HHMMSS_mmm.png`

## サブスクライブ

| トピック | 型 | QoS | 説明 |
|---|---|---|---|
| `/map` | `nav_msgs/msg/OccupancyGrid` | RELIABLE / TRANSIENT_LOCAL | 背景マップ |
| `/particle_cloud` (デフォルト) | `nav2_msgs/msg/ParticleCloud` | BEST_EFFORT / VOLATILE | AMCLパーティクル群 |
| `/particles` (EMCL時) | `geometry_msgs/msg/PoseArray` | BEST_EFFORT / VOLATILE | EMCLパーティクル群 |
| `/scan` | `sensor_msgs/msg/LaserScan` | BEST_EFFORT (sensor_data) | LiDARスキャン |

※ パーティクルトピック名はパラメータで変更可能

## パブリッシュ

| トピック | 型 | 説明 |
|---|---|---|
| `/vlm_context_image` | `sensor_msgs/msg/Image` | 重畳画像(bgr8) |

## その他

### パラメータ

| パラメータ名 | 型 | デフォルト | 説明 |
|---|---|---|---|
| `capture_interval_sec` | double | `1.0` | スナップショット取得間隔 [秒] |
| `snapshot_count` | int | `5` | 重ねる世代数 |
| `particle_topic` | string | `/particle_cloud` | パーティクルトピック名 |
| `particle_msg_type` | string | `ParticleCloud` | パーティクルメッセージ型 (`ParticleCloud` または `PoseArray`) |
| `show_particles` | bool | `True` | パーティクルを描画するか |
| `show_laser_scan` | bool | `True` | LiDAR点群を描画するか |
| `show_best_pose` | bool | `True` | 自己位置マーカーを描画するか |
| `particle_radius` | int | `2` | パーティクルの描画半径 [px] |
| `best_pose_radius` | int | `4` | 自己位置マーカーの基準半径 [px] |
| `laser_point_radius` | int | `1` | LiDAR点群の描画半径 [px] |

### サービス

| サービス名 | 型 | 説明 |
|---|---|---|
| `/save_overlay_image` | `std_srvs/srv/Trigger` | 現在の重畳画像を `/tmp` に保存 |

### 描画仕様

- **Jetグラデーション**: 青(古) → シアン → 緑 → 黄 → 赤(新)で世代を色分け
- **描画順序**: 古い→新しい順で描画し、新しいものが最前面に表示される
- **パーティクル**: 円で描画、重みが大きいほど半径が大きくなる
- **LiDAR点群**: 小さな円で描画
- **自己位置マーカー**: LiDAR点群より前面に表示。新しいものほど半径が大きく、向きを矢印で表示

### 依存パッケージ

```xml
<depend>nav2_msgs</depend>
<depend>geometry_msgs</depend>
<depend>std_srvs</depend>
```

## ライセンス

BSD-3-Clause © 2026 Junya Wada
