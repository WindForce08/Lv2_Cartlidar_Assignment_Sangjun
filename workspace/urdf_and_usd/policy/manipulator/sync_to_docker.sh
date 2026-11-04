#!/usr/bin/env bash
# 팀 공유본 isaac/ (로봇 설정 · USD) + 이 manipulator/ 폴더를 Docker 컨테이너 isaac-lab-base 가 보는 폴더로 복사 (rsync, 바뀐 것만).
#   ./sync_to_docker.sh          ->  ~/dockerWorkspace/IsaacLab/scripts/waffle_shared_261028/{isaac,manipulator}/
#                                    (컨테이너: /workspace/isaaclab/scripts/waffle_shared_261028/manipulator)
#   ./sync_to_docker.sh --pull   <-  학습 결과 (logs) 와 수집 데이터 (*.json) 를 되가져옴
# isaac/rl (주행 · 도킹) 은 isaac/rl/sync_to_docker.sh 가 맡으므로 여기서는 건드리지 않음.
# 폴더 이름의 공백 ("urdf_261028 (1)") 을 피하려고 컨테이너 쪽 이름은 waffle_shared_261028 로 고정.
set -eu
HERE="$(cd "$(dirname "$0")" && pwd)"
ISAAC="$(cd "$HERE/../../isaac" && pwd)"
ROOT="${ISAACLAB_HOST_DIR:-$HOME/dockerWorkspace/IsaacLab}/scripts/waffle_shared_261028"
DST="$ROOT/manipulator"
if [ "${1:-}" = "--pull" ]; then
  [ -d "$DST/logs" ] && rsync -a "$DST/logs/" "$HERE/logs/"
  for f in store_handover.json insert_starts.json; do [ -f "$DST/$f" ] && cp -f "$DST/$f" "$HERE/$f"; done
  echo "가져옴: $DST/{logs,*.json} -> $HERE/"
  exit 0
fi
mkdir -p "$ROOT/isaac" "$DST"
rsync -a --delete --exclude 'rl/' --exclude '*.log' --exclude '__pycache__/' "$ISAAC/" "$ROOT/isaac/"
rsync -a --delete --exclude 'logs/' --exclude '*.log' --exclude '__pycache__/' --exclude 'store_handover.json' --exclude 'insert_starts.json' "$HERE/" "$DST/"
for f in store_handover.json insert_starts.json; do [ -f "$HERE/$f" ] && [ ! -f "$DST/$f" ] && cp "$HERE/$f" "$DST/$f"; done
echo "복사함: $ISAAC -> $ROOT/isaac, $HERE -> $DST (컨테이너: /workspace/isaaclab/scripts/waffle_shared_261028/manipulator)"
