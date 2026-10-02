# vlm_kidnapping_detect

VLMを用いて誘拐ロボット問題を検知するROS2パッケージ。
現時点では以下の機能が実装されています。

- マップ・自己位置・LiDARを重畳した画像のパブリッシュ（`superposition`）
- USBカメラ画像のパブリッシュ（`camera_server`）
- カスタムサービスを利用した重畳画像・カメラ画像の連続保存

## ノード一覧

| 実行ファイル名 | ノード名（`ros2 run` / launch） | 役割 |
| --- | --- | --- |
| `superposition` | `superposition` / `superposition_node` | 地図・パーティクル・LiDARを重畳した画像を配信し、保存サービスを提供 |
| `camera_server` | `camera_publisher` / `camera_server_node` | USBカメラの画像を `/camera/color/image_raw` へ配信 |

## クイックスタート

```bash
colcon build --packages-select vlm_kidnapping_detect
source install/setup.bash

# superposition と camera_server を同時に起動
ros2 launch vlm_kidnapping_detect image_collection.launch.py

```

launch 時の superposition のパラメータは `config/superposition.yaml` で設定します。
YAML を編集したら `colcon build` で install 先へ反映してください（`--symlink-install` でビルドしている場合は不要）。

```yaml
superposition_node:  # launch で付けているノード名と一致させる
  ros__parameters:
    capture_interval_sec: 1.0   # double は 1 ではなく 1.0 と書く
    snapshot_count: 5
    particle_topic: /particle_cloud
    particle_msg_type: ParticleCloud
    ...
```

ノードを個別に起動する場合:

```bash
ros2 run vlm_kidnapping_detect superposition
ros2 run vlm_kidnapping_detect camera_server

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

### LiDAR点群の色

LiDAR点群はパーティクルと区別するため、世代カラーではなく固定色で描画されます。
デフォルトはマゼンタ `(255, 0, 255)` です（パーティクルの世代カラーが使う色相
青→シアン→緑→黄→赤 に含まれないため）。

`laser_point_color` パラメータに **BGR順** の整数配列を渡すと変更できます:

```bash
# LiDAR点群を黄色にする
ros2 run vlm_kidnapping_detect superposition \
  --ros-args \
  -p laser_point_color:="[0, 255, 255]"

```

要素数が3でない、または0〜255の範囲外の値を含む場合は警告を出してデフォルト色に戻ります。

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

### 画像の保存（カスタムサービス）

サービスを呼び出すことで、現在の重畳画像を保存できます。引数により1枚のみの保存、または指定間隔での連続保存が可能です。

**1枚だけ保存する場合:**

```bash
ros2 service call /save_overlay_image vlm_kidnapping_detect/srv/SaveOverlayImage "{num_images: 1, interval_sec: 0.0}"

```

**指定間隔で連続保存する場合（例: 5秒ごとに合計5枚）:**

```bash
ros2 service call /save_overlay_image vlm_kidnapping_detect/srv/SaveOverlayImage "{num_images: 5, interval_sec: 5.0}"

```

保存先: `/tmp`

#### 保存される2種類の画像
 
1回の保存につき、同じタイムスタンプを持つ以下の2枚のPNG画像が `/tmp` に保存されます。
 
| ファイル名 | 内容 | 元トピック |
| --- | --- | --- |
| `{timestamp}_overlay.png` | マップに自己位置・パーティクル・LiDAR点群を重畳した俯瞰画像 | `/map`, `/particle_cloud`(または`/particles`), `/scan` から生成 |
| `{timestamp}_perspective.png` | 保存時点で最後に受信したロボット搭載カメラの画像（一人称視点） | `/camera/color/image_raw` |
 
- `{timestamp}` は `YYYY_MMDD_HHMMSS` 形式（例: `2026_0906_135042`）で、同じ保存タイミングの2枚には同一の値が使われます。
- 秒単位までしか持たないため、`interval_sec` を1秒未満にすると同じファイル名になり上書きされます。
- 保存時にカメラ画像を一度も受信していない場合は `overlay.png` のみが保存され、`perspective.png` は生成されません。

## ログ出力

両ノードとも、同じログを周期的に出し続けるのではなく、**状態が変化したタイミングで1回だけ**ログを出します。
同じエラーが続いても1回しか表示されないため、切断や復帰に気付きやすくなっています。

### superposition

起動時に動作の説明とパラメータを表示します:

```
[INFO] 1.0秒ごと、直近5個の重畳画像を、1.0秒ごとに /vlm_context_image へパブリッシュ
[INFO] 起動 (particle_topic=/particle_cloud, particle_msg_type=ParticleCloud, capture_interval=1.0s, topic_timeout=3.0s, snapshot_count=5, ...)
```

その後は以下のタイミングでログが出ます（`{配信元}` は AMCL / LiDAR / カメラサーバ）。

| タイミング | レベル | ログ例 |
| --- | --- | --- |
| 起動時にpublisherがいない | WARN | `カメラサーバと未接続 (/camera/color/image_raw のpublisherがいません)` |
| publisherを検出 | INFO | `カメラサーバと接続 (/camera/color/image_raw の購読開始)` |
| publisherが消えた | WARN | `カメラサーバとの接続が切れました (/camera/color/image_raw のpublisherが消えました)` |
| 初回受信 | INFO | `{配信元}から受信開始 ({トピック})` |
| `topic_timeout_sec` 以上受信なし | WARN | `{配信元}からの受信途絶 ({トピック}, 3.0秒以上受信なし)` |
| 途絶から復帰 | INFO | `{配信元}からの受信再開 ({トピック})` |
| マップ受信（サイズ・解像度・原点が変わった時のみ） | INFO | `マップ受信 (/map, 100x100, resolution=0.050)` |
| マップ／パーティクル待ち | WARN | `マップ未受信のためキャプチャを待機中` |
| `snapshot_count` 枚たまるまで | INFO | `スナップショット収集中 (5枚たまったらパブリッシュ開始)` |
| パブリッシュ開始・再開 | INFO | `重畳画像のパブリッシュ開始 (/vlm_context_image)` |
| 画像生成・配信で例外 | ERROR | `重畳画像の生成・配信に失敗: ...` |
| 推定姿勢が地図外へ出た／戻った | WARN / INFO | `推定姿勢が地図の範囲外です` / `推定姿勢が地図内に戻りました` |
| `/vlm_context_image` の購読者の有無が変化 | INFO | `/vlm_context_image の購読者が接続しました` |
| 保存サービスの受信 | INFO | `保存要求を受信 (num_images=3, interval_sec=0.5)` |
| 保存時のカメラ画像の有無が変化 | INFO / WARN | `カメラ画像未受信のため、オーバーレイ画像のみ保存します` |
| 終了 | INFO | `終了` |

- 接続状態（publisherの有無）は1秒ごとに確認します。
- AMCLはロボット静止中にパーティクルを配信しないため、パーティクルトピックは受信途絶の判定対象外です（切断はpublisherの有無で検知します）。
- 保存サービスでは、保存したファイルパスは1枚ごとに表示されます。

### camera_server

| タイミング | レベル | ログ |
| --- | --- | --- |
| 起動 | INFO | `カメラサーバを起動しました` |
| カメラと接続した（再接続を含む） | INFO | `カメラと接続しました` |
| カメラを開けない／途中で切断された | ERROR | `カメラと接続できません！接続を確認してください。` |

切断中は1秒ごとに再接続を試み、つながると自動で配信を再開します。
再接続を試すたびにOpenCVが出す警告は抑制しています（`OPENCV_LOG_LEVEL=ERROR`）。

## camera_server

USBカメラ（V4L2）の画像を 10Hz で `/camera/color/image_raw`（bgr8）へ配信します。
使用するデバイス番号はパラメータではなく、`camera_server_node.py` の `self.camera_index`（デフォルト `0` = `/dev/video0`）で指定します。
接続されているデバイスは `ls /dev/video*` で確認できます。

## サブスクライブ（superposition）

| トピック | 型 | QoS | 説明 |
| --- | --- | --- | --- |
| `/map` | `nav_msgs/msg/OccupancyGrid` | RELIABLE / TRANSIENT_LOCAL | 背景マップ |
| `/particle_cloud` (デフォルト) | `nav2_msgs/msg/ParticleCloud` | BEST_EFFORT / VOLATILE | AMCLパーティクル群 |
| `/particles` (EMCL時) | `geometry_msgs/msg/PoseArray` | BEST_EFFORT / VOLATILE | EMCLパーティクル群 |
| `/scan` | `sensor_msgs/msg/LaserScan` | BEST_EFFORT (sensor_data) | LiDARスキャン |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | BEST_EFFORT (sensor_data) | カメラ画像（保存時の `perspective.png` に使用） |

※ パーティクルトピック名はパラメータで変更可能

## パブリッシュ

| ノード | トピック | 型 | 説明 |
| --- | --- | --- | --- |
| `superposition` | `/vlm_context_image` | `sensor_msgs/msg/Image` | 重畳画像(bgr8) |
| `camera_server` | `/camera/color/image_raw` | `sensor_msgs/msg/Image` | カメラ画像(bgr8, 10Hz) |

## その他

### パラメータ（superposition）

| パラメータ名 | 型 | デフォルト | 説明 |
| --- | --- | --- | --- |
| `capture_interval_sec` | double | `1.0` | スナップショット取得間隔 [秒] |
| `topic_timeout_sec` | double | `3.0` | この秒数以上受信が無ければ受信途絶とみなす [秒]（0以下で無効） |
| `snapshot_count` | int | `5` | 重ねる世代数（この枚数たまるまでパブリッシュしない） |
| `particle_topic` | string | `/particle_cloud` | パーティクルトピック名 |
| `particle_msg_type` | string | `ParticleCloud` | パーティクルメッセージ型 (`ParticleCloud` または `PoseArray`) |
| `show_particles` | bool | `True` | パーティクルを描画するか |
| `show_laser_scan` | bool | `True` | LiDAR点群を描画するか |
| `show_best_pose` | bool | `True` | 自己位置マーカーを描画するか |
| `particle_radius` | int | `2` | （現在未使用。パーティクルは1画素で描画される） |
| `best_pose_radius` | int | `4` | 自己位置マーカーの基準半径 [px] |
| `laser_point_radius` | int | `1` | LiDAR点群の描画半径 [px] |
| `laser_point_color` | int[] | `[255, 0, 255]` | LiDAR点群の描画色（**BGR順**、各0〜255） |

### サービス

| サービス名 | 型 | 説明 |
| --- | --- | --- |
| `/save_overlay_image` | `vlm_kidnapping_detect/srv/SaveOverlayImage` | 重畳画像を `/tmp` に保存（枚数・間隔指定可） |

### 描画仕様

* **Jetグラデーション**: 青(古) → シアン → 緑 → 黄 → 赤(新)で世代を色分け
* **描画順序**: 古い→新しい順で描画し、新しいものが最前面に表示される
* **パーティクル**: 1画素ずつ直接描画。重み(weight)は点の大きさ・色には反映されない
* **LiDAR点群**: 最新の1世代分（現在時刻のもの）のみを小さな円で描画。
  パーティクルと区別するため世代カラーではなく `laser_point_color` の固定色を使う
* **自己位置マーカー**: LiDAR点群より前面に表示。新しいものほど半径が大きく、向きを矢印で表示

### 依存パッケージ（主なもの）

```xml
<depend>rclpy</depend>
<depend>nav2_msgs</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>nav_msgs</depend>
<depend>cv_bridge</depend>
<buildtool_depend>ament_cmake</buildtool_depend>
<buildtool_depend>ament_cmake_python</buildtool_depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>

```

## ライセンス

BSD-3-Clause © 2026 Junya Wada

