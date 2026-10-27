#!/usr/bin/env bash
# arm_bench 팔 강화학습 실행기 - 이미 설치된 Isaac Sim 6.1 / Isaac Lab 3.0 을 그대로 씀 (새로 설치하지 않음).
# play 만 창으로, 나머지는 모두 창 없이 (헤드리스). 저사양 (8 GB GPU) 기본값.
#   ./run.sh convert                      팔 URDF (../urdf/arm_bench.urdf) -> USD (usd/arm_bench_sim/). URDF 를 바꿨을 때만
#   ./run.sh check [--envs 4]             물리 확인: 손으로 짠 순서 (IK) 로 퍽 집어 옮기기 -> results/check_pick.json
#   ./run.sh train [환경 수] [반복] [덮어쓰기...]  PPO 학습 (기본 256 환경, 1500 반복) -> logs/rsl_rl/arm_pick/<시각>/
#   ./run.sh eval [환경 수] [에피소드]      최근 학습 평가 (창 없이): 성공률 -> results/eval.json, exported/ 에 정책 파일
#   ./run.sh play [환경 수]                최근 학습을 창으로 재생 (화면 필요, Ctrl+C 로 끝)
#   ./run.sh board                         학습 곡선 (TensorBoard) http://localhost:6006
# Isaac Lab 위치: ISAACLAB_DIR=/경로/IsaacLab (기본 ~/pa-course/IsaacLab-6.1) 또는 ISAACLAB_PY=/경로/python
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
LAB="${ISAACLAB_DIR:-$HOME/pa-course/IsaacLab-6.1}"
export HEADLESS=1   # Isaac Lab 3.0 은 --headless 옵션이 없음. --viz 를 안 주거나 HEADLESS=1 이면 창 없이
if [ -n "${ISAACLAB_PY:-}" ]; then PY=("$ISAACLAB_PY")
elif [ -x "$LAB/.venv/bin/python" ]; then PY=("$LAB/.venv/bin/python")
else echo "Isaac Lab 을 찾지 못했습니다: $LAB (ISAACLAB_DIR 또는 ISAACLAB_PY 로 지정)"; exit 1; fi
cd "$HERE"
TASK=ArmBench-PickPlace
if [ "${1:-}" != "convert" ] && [ "${1:-}" != "board" ] && [ "${1:-}" != "" ] && [ ! -f usd/arm_bench_sim/arm_bench_sim.usda ]; then
  echo "USD 가 없어 먼저 변환합니다 (./run.sh convert)"; "${PY[@]}" scripts/convert_urdf.py || exit 1
fi
case "${1:-}" in
  convert) exec "${PY[@]}" scripts/convert_urdf.py ;;
  check)   shift; exec "${PY[@]}" scripts/check_pick.py "$@" ;;
  train)   N="${2:-256}"; IT="${3:-1500}"; shift; shift; shift
           exec "${PY[@]}" scripts/train.py --task "$TASK" --num_envs "$N" --max_iterations "$IT" --viz none "$@" ;;
  eval)    shift; exec "${PY[@]}" scripts/eval_policy.py "$@" ;;
  board)   exec "$(dirname "${PY[0]}")/tensorboard" --logdir logs/rsl_rl ;;
  play)    exec env -u HEADLESS "${PY[@]}" scripts/play.py --task "$TASK" --num_envs "${2:-4}" --checkpoint latest --viz kit ;;
  *) sed -n 2,10p "$0"; exit 2 ;;
esac
