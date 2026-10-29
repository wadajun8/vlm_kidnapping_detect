#!/bin/bash
# 6条件（{amcl, emcl2} × {normal, teleport, initialpose}）のデータセットを順に収集する
#
# 使い方（ワークスペースを source してから）:
#   ./run_all_conditions.bash                 # collection.yaml の episodes 数で全条件
#   ./run_all_conditions.bash 1               # 各条件1エピソードだけ（動作確認用）
#   LOCALIZERS=emcl2 MODES="teleport" ./run_all_conditions.bash 5
#
# localizer ごとにシミュレーション一式（sim_collection.launch.py）を起動し直す。
# 実行環境・実験条件・集計は record_environment で YAML / Markdown に書き出す。

set -u

EPISODES=${1:-}
LOCALIZERS=${LOCALIZERS:-"amcl emcl2"}
MODES=${MODES:-"normal teleport initialpose"}
DATE=${DATE:-$(date +%Y_%m%d)}

if ! ros2 pkg prefix vlm_kidnapping_detect > /dev/null 2>&1; then
    echo "vlm_kidnapping_detect が見つかりません。ワークスペースの install/setup.bash を source してください" >&2
    exit 1
fi

OUTPUT_ROOT=$(python3 -c "
import yaml, os
from ament_index_python.packages import get_package_share_directory as d
cfg = yaml.safe_load(open(os.path.join(d('vlm_kidnapping_detect'), 'config', 'collection.yaml')))
print(os.path.expanduser(cfg['output_root']))")
DATE_DIR="${OUTPUT_ROOT}/${DATE}"
LOG_DIR="${DATE_DIR}/logs"
mkdir -p "${LOG_DIR}"

# NVIDIA のカーネルモジュールとライブラリの版が食い違っていると Gazebo のカメラ描画（GLX）が失敗するため、
# その場合は mesa（内蔵 GPU / ソフトウェア描画）を使う。使ったものは実行環境に記録される
if [ -z "${__GLX_VENDOR_LIBRARY_NAME:-}" ] && nvidia-smi 2>&1 | grep -q "mismatch"; then
    echo "NVIDIA ドライバの版の不一致を検出したため mesa で描画します"
    export __GLX_VENDOR_LIBRARY_NAME=mesa
fi

SIM_PID=""
stop_sim() {
    if [ -n "${SIM_PID}" ]; then
        kill -INT -- "-${SIM_PID}" 2> /dev/null
        for _ in $(seq 1 30); do
            kill -0 "${SIM_PID}" 2> /dev/null || break
            sleep 1
        done
        kill -KILL -- "-${SIM_PID}" 2> /dev/null
        SIM_PID=""
    fi
    pkill -KILL -f "[g]zserver" 2> /dev/null
}
trap 'stop_sim; exit 130' INT TERM

episodes_arg=()
[ -n "${EPISODES}" ] && episodes_arg=(--episodes "${EPISODES}")

for localizer in ${LOCALIZERS}; do
    echo "=== ${localizer}: シミュレーションを起動 (ログ: ${LOG_DIR}/sim_${localizer}.log)"
    setsid ros2 launch vlm_kidnapping_detect sim_collection.launch.py localizer:="${localizer}" \
        > "${LOG_DIR}/sim_${localizer}.log" 2>&1 &
    SIM_PID=$!

    ready=0
    for _ in $(seq 1 120); do
        if grep -q "Successfully spawned entity" "${LOG_DIR}/sim_${localizer}.log"; then
            ready=1
            break
        fi
        sleep 1
    done
    if [ "${ready}" -ne 1 ]; then
        echo "シミュレーションが起動しませんでした。ログを確認してください" >&2
        stop_sim
        exit 1
    fi
    sleep 10  # nav2 の起動待ち（残りの準備待ちは collect_dataset が行う）

    ros2 run vlm_kidnapping_detect record_environment env \
        --date-dir "${DATE_DIR}" --localizer "${localizer}"

    for mode in ${MODES}; do
        condition_dir="${DATE_DIR}/${localizer}_${mode}"
        echo "=== ${localizer}_${mode}: 収集開始"
        ros2 run vlm_kidnapping_detect collect_dataset \
            --localizer "${localizer}" --mode "${mode}" --date "${DATE}" "${episodes_arg[@]}" \
            2>&1 | tee -a "${LOG_DIR}/collect_${localizer}_${mode}.log"
        ros2 run vlm_kidnapping_detect record_environment conditions --condition-dir "${condition_dir}"
        ros2 run vlm_kidnapping_detect record_environment summary --condition-dir "${condition_dir}"
    done

    stop_sim
done

ros2 run vlm_kidnapping_detect record_environment summary-all --date-dir "${DATE_DIR}"
echo "=== 完了: ${DATE_DIR}/summary_all.md"
