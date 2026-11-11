#!/bin/bash
# 連続判定用のデータセット（amcl × {normal, teleport, initialpose}、各 5 エピソード、1 エピソード 2 分）を
# コマンド 1 つで集める。中身は run_all_conditions.bash を config/collection_seq.yaml で呼ぶだけ。
#
# 使い方（ワークスペースを source してから）:
#   ./run_seq_collection.bash        # 本番（各条件 collection_seq.yaml の episodes 数、既定 5）
#   ./run_seq_collection.bash 1      # 試し（各条件 1 エピソード）
#   DATE=2026_1112_seq ./run_seq_collection.bash   # 保存先の日付ディレクトリ名を変える
#
# 終わったら、エピソードごとのフレーム数・誘拐の時刻・誘拐の種類と量を一覧で表示する。

set -u

HERE=$(cd "$(dirname "$0")" && pwd)
EPISODES=${1:-}

if ! ros2 pkg prefix vlm_kidnapping_detect > /dev/null 2>&1; then
    echo "vlm_kidnapping_detect が見つかりません。ワークスペースの install/setup.bash を source してください" >&2
    exit 1
fi
SHARE="$(ros2 pkg prefix vlm_kidnapping_detect)/share/vlm_kidnapping_detect"
export CONFIG="${SHARE}/config/collection_seq.yaml"
export LOCALIZERS=amcl
export MODES="normal teleport initialpose"
export DATE=${DATE:-$(date +%Y_%m%d)_seq}

# 重畳画像を半透明で描く設定になっているかを確かめる（superposition が実際に読む install 先の yaml）
python3 - "${SHARE}/config/superposition.yaml" <<'EOF' || exit 1
import sys, yaml
p = yaml.safe_load(open(sys.argv[1]))['superposition_node']['ros__parameters']
la, pa = p.get('laser_alpha', 1.0), p.get('particle_alpha', 1.0)
print(f'重畳画像の描き方: laser_alpha={la}, particle_alpha={pa}')
if la >= 1.0 or pa >= 1.0:
    print('半透明になっていません（1.0 は上書き）。config/superposition.yaml を直して colcon build してください',
          file=sys.stderr)
    sys.exit(1)
EOF

"${HERE}/run_all_conditions.bash" ${EPISODES}
status=$?

OUTPUT_ROOT=$(python3 -c "
import yaml, os, sys
print(os.path.expanduser(yaml.safe_load(open(sys.argv[1]))['output_root']))" "${CONFIG}")
DATE_DIR="${OUTPUT_ROOT}/${DATE}"
WANT=${EPISODES:-$(python3 -c "import yaml,sys; print(yaml.safe_load(open(sys.argv[1]))['episodes'])" "${CONFIG}")}

# エピソードの一覧
python3 - "${DATE_DIR}" "${WANT}" <<'EOF'
import json, sys
from pathlib import Path
root, want = Path(sys.argv[1]), int(sys.argv[2])
print(f'\n=== 集めたエピソード（{root}）')
print(f'{"条件":<18} {"エピソード":<8} {"フレーム":>6} {"誘拐の時刻[s]":>12}  誘拐')
short = []
for cond in sorted(p for p in root.iterdir() if p.is_dir() and p.name != 'logs'):
    eps = sorted(cond.glob('ep_*/episode.json'))
    for f in eps:
        e = json.loads(f.read_text())
        ev = e.get('event', {})
        kind = ev.get('kidnap_type')
        what = (f'{kind} {ev.get("displacement_m")} m, {ev.get("rotation_deg")}°' if kind else 'なし')
        print(f'{cond.name:<18} {f.parent.name:<8} {e.get("frames", "?"):>6} '
              f'{ev.get("t_episode", "?"):>12}  {what}')
    if len(eps) < want:
        short.append(f'{cond.name}: {len(eps)}/{want}')
if short:
    print('\nエピソードがそろわなかった条件: ' + '、'.join(short))
EOF

exit ${status}
