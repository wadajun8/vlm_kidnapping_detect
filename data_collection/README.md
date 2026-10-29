# data_collection: シミュレータでのデータセット収集

Gazebo（iscas_museum）で、通常時と誘拐時の **重畳画像（overlay）と一人称カメラ画像（perspective）の組** を、
正解姿勢との誤差から付けたラベル付きで集めます。オフラインで VLM の誘拐検知を評価するためのものです。

## 条件

自己位置推定 2種類 × 誘拐の起こし方 3種類 = 6条件です。リセットの設定はどちらも既存のパラメータのままです。

| | normal | teleport | initialpose |
| --- | --- | --- | --- |
| 内容 | 誘拐なしで走行 | 走行中にロボット本体を強制移動（`/gazebo/set_entity_state`） | 本体はそのまま、`/initialpose` で推定だけ誤った姿勢へ |
| amcl | `amcl_normal` | `amcl_teleport` | `amcl_initialpose` |
| emcl2 | `emcl2_normal` | `emcl2_teleport` | `emcl2_initialpose` |

誘拐の量は small（0.5〜1.5 m、±30°）、large（3 m 以上離れたランダムな位置）、rotation（位置そのまま 45〜180°）
から一様に選びます。値は `config/collection.yaml` で変えられます。

## 使い方

```bash
colcon build --symlink-install --packages-select vlm_kidnapping_detect
source install/setup.bash

# 全6条件（各 collection.yaml の episodes 数、既定20）
./src/vlm_kidnapping_detect/data_collection/run_all_conditions.bash

# 動作確認（各条件1エピソード）
./src/vlm_kidnapping_detect/data_collection/run_all_conditions.bash 1

# 一部の条件だけ
LOCALIZERS=emcl2 MODES="teleport initialpose" ./src/vlm_kidnapping_detect/data_collection/run_all_conditions.bash 5
```

手動で1条件だけ回す場合:

```bash
ros2 launch vlm_kidnapping_detect sim_collection.launch.py localizer:=emcl2   # 端末1
ros2 run vlm_kidnapping_detect collect_dataset --localizer emcl2 --mode teleport --episodes 3   # 端末2
```

同じ日付・条件のディレクトリがあれば `ep_` の番号は続きから振られます（上書きされません）。

## 1エピソードの流れ

1. ランダムな開始姿勢へロボットを移動し、正しい初期姿勢を `/initialpose` で与える
2. nav2 でランダムなゴールへ走行を始め、6秒は保存しない（重畳画像の履歴にリセット前の軌跡が残るため）。
   推定が「normal」に収まらなければそのエピソードは破棄する
3. 10秒記録 → 誘拐（normal では何もしない）→ 20秒記録。走行はゴールを送り直しながら続ける

`/vlm_context_image`（1秒ごと）を受信するたびに、時刻が最も近いカメラ画像・正解姿勢・推定姿勢を対応付けて保存します。

## ラベル

正解姿勢（Gazebo）と推定姿勢（TF `map→base_footprint`）の誤差から毎フレーム付けます。

| ラベル | 条件 |
| --- | --- |
| `kidnapped` | 位置 > 0.5 m または 角度 > 30° |
| `normal` | 位置 ≤ 0.2 m かつ 角度 ≤ 10° |
| `ambiguous` | その間（学習・評価から除外できるよう区別） |

## 出力

```
~/vlm_kidnapping_dataset/<YYYY_MMDD>/
  experiment_environment.yaml / .md    # 実行環境（OS・CPU/GPU・ROS 2・Gazebo・リポジトリのコミット）
  summary_all.yaml / .md               # 全条件の集計表（論文の表に使える形）
  logs/                                # シミュレーションと収集のログ
  <localizer>_<mode>/
    experiment_conditions.yaml / .md   # 実験条件（センサ仕様・手順・誘拐・ラベル・主なパラメータ）
    summary.yaml / .md                 # 収集結果（ラベル数・誘拐量・回復率・回復時間など）
    params/<ノード>.yaml               # 収集時に ros2 param dump した実際のパラメータ
    run_info_<開始時刻>.yaml           # 実行ごとの設定・エピソード一覧・破棄理由
    ep_000/
      0012.5_overlay.png               # ファイル名はエピソード開始からのシミュレーション時刻 [s]
      0012.5_perspective.png
      metadata.csv                     # フレームごとの正解・推定・誤差・ラベル・時刻差
      episode.json                     # 開始姿勢・誘拐の種類と量・誘拐前後の姿勢・seed など
```

`metadata.csv` の列:

| 列 | 内容 |
| --- | --- |
| `t_episode` / `t_since_event` | エピソード開始・誘拐からの時刻 [s]（誘拐前は `t_since_event` が空） |
| `phase` | `pre`（誘拐前）/ `post`（誘拐後） |
| `label`, `err_pos_m`, `err_yaw_deg` | ラベルと誤差 |
| `gt_*` / `est_*` | 正解・推定姿勢（map 座標、角度は度） |
| `gt_dt` / `camera_dt` | 重畳画像の時刻に対する正解姿勢・カメラ画像の時刻差 [s] |
| `overlay` / `perspective` | 画像ファイル名（カメラ画像が許容差内に無ければ `perspective` は空） |

`summary` の「回復」は、誘拐後に一度 `kidnapped` になり、その後最後まで `normal` が続いた場合を指し、
回復時間はその `normal` が始まった `t_since_event` です。

## 既存のシミュレーションとの違い（論文に書くべき点）

`sim_collection.launch.py` は既存のパッケージを編集せず、起動時に次の変更を加えます。

- **odom を encoder 基準にする**: Gazebo の diff_drive の odom は既定で world 基準（正解姿勢）です。
  そのままだと本体を強制移動したとき odom も一緒に跳び、自己位置推定が追従して誘拐になりません。
  URDF に `<odometry_source>0</odometry_source>` を挿入しています。
- **`libgazebo_ros_state.so` を world に挿入**: 強制移動と正解姿勢の取得に使います。
- **スポーン位置を world (0, −2) にする**: `iscas_museum_map` は world (0, −2) を map 原点として作られているためです
  （world→map は y に +2.0。`collection.yaml` の `world_to_map`）。
- GUI・rviz・実機用の `camera_server` は起動しません（カメラ画像は Gazebo の D435 モデルから）。

## 注意

- NVIDIA のカーネルモジュールとライブラリの版が食い違っていると（ドライバ更新後に再起動していないなど）、
  Gazebo のカメラ描画が `X Error ... GLX` で止まります。`run_all_conditions.bash` はこれを検出して
  `__GLX_VENDOR_LIBRARY_NAME=mesa` で描画します（使ったものは `experiment_environment.md` に記録）。
- amcl は初期姿勢を受け取るまで `map` フレームを出さず nav2 の起動が止まるため、`collect_dataset` が開始時に正解姿勢を与えます。
