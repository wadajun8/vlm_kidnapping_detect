# vlm_kidnapping_detect

ROS 2パッケージ。AMCL(Adaptive Monte Carlo Localization)による自己位置推定とレーザースキャンデータを組み合わせ、マップ上にロボットの位置推定分布とセンサ観測データを時系列で重ねて可視化します。VLM(Vision Language Model)による異常検知を支援するコンテキスト画像を生成することを想定しています。

**ライセンス**: BSD-3-Clause  
**メンテナー**: Junya Wada (junya.wada.27@gmail.com)

---

## 概要

### 目的

誘拐ロボット問題(kidnapped robot problem)の検知を支援するため、以下の情報を時系列で可視化します：

- **自己位置推定の不確実性**: AMCLから得られるパーティクル群(粒子フィルタ)の加重平均による代表位置
- **ロボットの観測データ**: レーザースキャナから得られた距離データを、ロボットの推定位置に基づいてマップ座標系に変換

これらを赤(古い)→緑(新しい)のグラデーション色で描画することで、ロボットが誘拐された場合の位置推定の急激な変化を視覚的に検出できます。

### 主要な特徴

1. **重み付き平均による自己位置代表点の計算**
   - AMCLのパーティクルクラウドから、重み付き平均として代表位置を計算
   - 角度(yaw)は周期性を考慮し、単位円上のベクトル加重平均を使用

2. **バッチ処理によるセンサデータの集約**
   - 指定した時間窓内のレーザースキャンを1つのバッチとして集約
   - 複数バッチの履歴を保持し、時系列で描画

3. **座標変換と画像化**
   - ロボット座標系のセンサデータをmap座標系に変換
   - OccupancyGridを背景として、マップ座標系で全データを可視化

4. **タイマーによる周期パブリッシュ**
   - パーティクル受信頻度と独立した周期でROS Image messageをパブリッシュ
   - 受信間隔にばらつきがあっても安定した周期配信が可能

---

## ノードの詳細説明

### ノード名

`superposition` (ROS 2 Node)

実行時のコマンド:
```bash
ros2 run vlm_kidnapping_detect superposition
```

### 処理フロー

```
OccupancyGrid (/map) → マップ画像(背景)に変換・キャッシュ
                              ↓
ParticleCloud (/particle_cloud) → 重み付き平均で代表位置計算 → 歴史に追加
                              ↓
LaserScan (/scan) → 座標変換 → バッチに追加
                              ↓
                    時間窓チェック(0.1秒ごと)
                              ↓
                    render_overlay() → キャンバスに描画
                              ↓
               timer_callback(publish_rate周期)
                              ↓
                  Image (/vlm_context_image)
```

### 内部状態の管理

#### マップ関連
- `self.base_map_img`: OccupancyGridから生成されたBGR画像(背景)
- `self.map_info`: 座標変換に必要なマップ情報(解像度、原点など)

#### 自己位置推定
- `self.latest_best_pose_world`: 最新の代表位置(世界座標 x, y, yaw)
- `self.best_pose_history`: 直近N個の代表位置履歴(ピクセル座標 + yaw)

#### バッチ処理
- `self.current_batch`: 現在進行中のバッチ(時間窓内のスキャンデータ)
- `self.batch_history`: 過去N個のバッチ履歴

#### 描画結果
- `self.latest_overlay`: 最新の描画済み画像(タイマーでパブリッシュ用)

---

## パラメータ

ノード起動時に設定可能なパラメータ。ros2 param set コマンドまたはlaunchファイルで指定します。

| パラメータ名 | 型 | デフォルト | 説明 |
|:---|:---|:---:|:---|
| `history_length` | int | 5 | 自己位置代表点の履歴数(何世代分を同時に描画するか) |
| `publish_rate` | float | 2.0 | Image message のパブリッシュ周期[Hz] |
| `show_best_pose` | bool | True | 代表点(重み付き平均)を描画するか |
| `best_pose_radius` | int | 4 | 代表点の基準円半径[ピクセル]。新しいほど0.4〜1.0倍になる |
| `laser_point_radius` | int | 1 | レーザースキャン点の円半径[ピクセル] |
| `batch_window_seconds` | float | 1.0 | センサデータをまとめる時間窓[秒] |
| `batch_history_count` | int | 5 | 保持するバッチ数 |

### パラメータ設定例

```bash
# 高周期でパブリッシュ、短い時間窓
ros2 param set /superposition publish_rate 10.0
ros2 param set /superposition batch_window_seconds 0.5

# 長い履歴を保持
ros2 param set /superposition history_length 10
ros2 param set /superposition batch_history_count 10
```

---

## サブスクライブするトピック

### `/map` (nav_msgs/OccupancyGrid)

占有格子マップ。SLAM(gmapping, cartographerなど)またはamclから受信します。

- **QoS**: TRANSIENT_LOCAL + RELIABLE
- **用途**: 背景画像の生成、座標変換用のマップメタデータ取得

---

### `/particle_cloud` (nav2_msgs/ParticleCloud)

AMCLから発行されるパーティクルクラウド。各粒子は位置(x, y, z)、姿勢(quaternion)、重み(float64)を持ちます。

- **QoS**: BEST_EFFORT + VOLATILE
- **用途**: 重み付き平均により代表位置を計算、履歴を更新

---

### `/scan` (sensor_msgs/LaserScan)

LiDARまたは2D距離センサから得られたスキャンデータ。

- **QoS**: SENSOR_DATA
- **用途**: 各距離値をロボットの代表位置に基づいてmap座標系に変換、バッチに追加

---

## パブリッシュするトピック

### `/vlm_context_image` (sensor_msgs/Image)

現在の重畳画像(マップ+代表点+スキャン点群)をBGR画像で発行します。

- **QoS**: depth=10
- **メッセージ内容**:
  - `header.stamp`: メッセージ発行時刻
  - `header.frame_id`: マップフレーム名(通常: 'map')
  - `data`: BGR8 画像データ
  - 画像の解像度: OccupancyGridの幅×高さ

**画像の見方**:
- **白**: フリースペース
- **黒**: 障害物
- **グレー**: 未探索
- **赤〜緑の点/円**: センサデータと代表点。赤が古く、新しいほど緑に近づく
- **円+矢印**: 代表点。矢印がロボットの向き(yaw)を示す

---

## サービス

### `/save_overlay_image` (vlm_kidnapping_detect/SaveOverlayImage)

現在の重畳画像をファイルとして保存します。

**注**: サービス定義ファイル `srv/SaveOverlayImage.srv` が必要です。

**リクエスト**: (なし)

**レスポンス**:
- `success` (bool): 保存成功の可否
- `message` (str): 結果メッセージ(ファイルパスまたはエラー情報)

**保存先**: `/tmp/overlay_YYYYMMDD_HHMMSS_mmm.png`

**使用例**:
```bash
ros2 service call /save_overlay_image vlm_kidnapping_detect/SaveOverlayImage
```

---

## セットアップ方法

### 前提条件

- **OS**: Ubuntu 20.04 LTS 以上
- **ROS 2**: Foxy, Humble, Iron 等
- **Python**: 3.8 以上

### 依存パッケージのインストール

```bash
sudo apt update
sudo apt install ros-<distro>-nav2 ros-<distro>-nav2-msgs \
                 python3-opencv python3-numpy
```

`<distro>` を実際のディストリビューション名(humble, iron等)に置き換えてください。

### ワークスペースの構成

```
~/ros2_ws/
└── src/
    └── vlm_kidnapping_detect/
        ├── vlm_kidnapping_detect/
        │   └── map_scan_superposition.py
        ├── test/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── README.md
        └── LICENSE
```

### ビルド

```bash
cd ~/ros2_ws
colcon build --packages-select vlm_kidnapping_detect
source install/setup.bash
```

---

## 使い方

### 基本的な実行方法

#### 方法1: 直接実行

```bash
ros2 run vlm_kidnapping_detect superposition
```

#### 方法2: パラメータを指定して実行

```bash
ros2 run vlm_kidnapping_detect superposition \
    --ros-args \
    -p history_length:=10 \
    -p publish_rate:=5.0 \
    -p batch_window_seconds:=0.5
```

### ノードの動作確認

別ターミナルで以下を実行:

```bash
# Image トピックを確認
ros2 topic echo /vlm_context_image

# パラメータ一覧の表示
ros2 param list /superposition

# 特定のパラメータ値を確認
ros2 param get /superposition publish_rate

# トピック情報の表示
ros2 topic info /vlm_context_image
```

### 画像の保存方法

#### 方法1: サービスで保存

サービス定義が利用可能な場合:
```bash
ros2 service call /save_overlay_image vlm_kidnapping_detect/SaveOverlayImage
```

#### 方法2: ROS Image を rosbag で記録

```bash
# 記録開始
ros2 bag record /vlm_context_image

# (Ctrl+C で終了)

# 記録ファイルを確認
ls -la rosbag2_*
```

#### 方法3: 外部ツールで表示・保存

画像表示ツールで `/vlm_context_image` を可視化:

```bash
# rqt で表示
rqt --standalone rqt_image_view

# rviz2 で表示
rviz2
```

#### 方法4: Python スクリプトで保存

```python
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSaver:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('image_saver')
        self.bridge = CvBridge()
        self.sub = self.node.create_subscription(
            Image, '/vlm_context_image', self.callback, 10)

    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        filename = f'/tmp/overlay_{self.node.get_clock().now().nanoseconds}.png'
        cv2.imwrite(filename, cv_image)
        print(f'Image saved: {filename}')

if __name__ == '__main__':
    saver = ImageSaver()
    rclpy.spin(saver.node)
```

---

## ライセンス

このパッケージは **BSD-3-Clause ライセンス**の下で配布されています。詳細は[LICENSE](LICENSE)ファイルを参照してください。
