#!/usr/bin/env bash
# waffle_4wd 주행 강화 학습 실행기 (팀 공유본 isaac/ 의 로봇 설정을 씀). Docker 컨테이너 isaac-lab-base 안에서, 또는 Isaac Lab 이 있는 PC 에서.
# 창이 필요한 것 (*_play) 만 창으로, 나머지는 창 없이. 학습 중에는 창을 띄우지 말 것 (GPU 8 GB 부족).
# 팔 (위에서 집기 · 팔레트 궤적 · 옛 보관통 과제) 은 ../../policy/manipulator/run_rl.sh (2026-11-04 정리)
# 주행
#   ./run_rl.sh drive [환경 수] [반복]         주행 학습 -> logs/rsl_rl/waffle_drive/
#   ./run_rl.sh drive_play [환경 수]           주행 최근 체크포인트를 창으로
#   ./run_rl.sh dock [환경 수] [반복]          박스 앞 세우기 학습 (주사위 경기장) -> logs/rsl_rl/waffle_dock/
#   ./run_rl.sh dock_play [환경 수]            박스 앞 세우기 최근 체크포인트를 창으로
#   ./run_rl.sh dock_eval [환경 수] [--checkpoint 경로]  박스 앞 세우기 평가 (목표 11 곳별 성공률 · 끝 오차, 창 없이)
# Isaac Lab 위치: ISAACLAB_DIR (기본: 컨테이너 /workspace/isaaclab) 또는 ISAACLAB_PY
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
if [ -z "${ISAACLAB_DIR:-}" ]; then
  if [ -d /workspace/isaaclab ]; then ISAACLAB_DIR=/workspace/isaaclab; else ISAACLAB_DIR="$HOME/dockerWorkspace/IsaacLab"; fi
fi
export HEADLESS=1
if [ -n "${ISAACLAB_PY:-}" ]; then PY=("$ISAACLAB_PY")
elif [ -x "$ISAACLAB_DIR/.venv/bin/python" ]; then PY=("$ISAACLAB_DIR/.venv/bin/python")
elif [ -x "$ISAACLAB_DIR/isaaclab.sh" ]; then PY=("$ISAACLAB_DIR/isaaclab.sh" -p)
else echo "Isaac Lab 을 찾지 못했습니다: $ISAACLAB_DIR (ISAACLAB_DIR 또는 ISAACLAB_PY 로 지정)"; exit 1; fi
export_latest() {   # $1 = 과제, $2 = 실험 이름: 가장 최근 학습의 exported/policy.pt 가 없으면 play 를 창 없이 잠깐 돌려 만듦
  local last; last=$(ls -d "$HERE"/logs/rsl_rl/"$2"/*/ 2>/dev/null | sort | tail -1)
  if [ -n "$last" ] && [ ! -f "$last/exported/policy.pt" ]; then
    timeout 300 "${PY[@]}" "$HERE/play.py" --task "$1" --num_envs 1 --checkpoint latest --viz none > "$HERE/export_$2.log" 2>&1 &
    local pid=$!; until [ -f "$last/exported/policy.pt" ] || ! kill -0 $pid 2>/dev/null; do sleep 2; done; sleep 3
    pkill -f "[p]lay.py --task $1 --num_envs 1 "; wait $pid 2>/dev/null   # isaaclab.sh -p 는 자식 python 을 따로 띄우므로 이름으로 끔
  fi
}

pol() { [ -f "$HERE/policies/$1" ] && echo "--checkpoint $HERE/policies/$1" || echo "--checkpoint latest"; }
case "${1:-}" in
  drive) "${PY[@]}" "$HERE/train.py" --task Waffle-Drive-Direct --num_envs "${2:-64}" --max_iterations "${3:-300}" --viz none ;;
  drive_play) env -u HEADLESS "${PY[@]}" "$HERE/play.py" --task Waffle-Drive-Direct --num_envs "${2:-16}" --checkpoint latest --viz kit ;;
  dock) "${PY[@]}" "$HERE/train.py" --task Waffle-Dock-Direct --num_envs "${2:-128}" --max_iterations "${3:-1000}" --viz none ;;
  dock_play) env -u HEADLESS "${PY[@]}" "$HERE/play.py" --task Waffle-Dock-Direct --num_envs "${2:-8}" --checkpoint latest --viz kit ;;
  dock_eval) "${PY[@]}" "$HERE/dock_eval.py" --num_envs "${2:-220}" "${@:3}" ;;
  *) sed -n 2,11p "$0"; exit 2 ;;
esac
