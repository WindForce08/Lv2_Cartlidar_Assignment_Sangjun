#!/usr/bin/env bash
# waffle_4wd Isaac Sim 6.1 / Isaac Lab 3.0 실행기 (설치된 Isaac Lab 을 그대로 씀). window 만 창, 나머지는 창 없이.
#   ./run.sh convert [--only light|full]      URDF -> USD 2개 + robot_params.json 값을 USD 에 씀 (값 · URDF 를 바꿨을 때만)
#   ./run.sh check [--full] [--pose drive|stow|zero] [--cameras]   물리 · 센서 점검 -> results/check_<판>_<자세>.json, results/img/
#   ./run.sh window [--full] [--pose ...]      Isaac Sim 창으로 로봇 + 상자 장면 (화면 필요, 창을 닫으면 끝)
#   ./run.sh maze [--light] [--paused]         Isaac Sim 창으로 주사위 경기장 + 로봇 (scenes/, 출발 구역)
# Isaac Lab 위치: ISAACLAB_DIR=/경로/IsaacLab ./run.sh ...  (기본 ~/pa-course/IsaacLab-6.1)
#               또는 ISAACLAB_PY=/경로/python ./run.sh ...  (Isaac Lab 이 설치된 Python 을 직접)
# ROS 2 를 source 한 터미널이어도 됨 (ROS 환경 변수는 여기서 뺌: Isaac 의 Python 과 섞이지 않게)
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
LAB="${ISAACLAB_DIR:-$HOME/pa-course/IsaacLab-6.1}"
if [ -n "${ISAACLAB_PY:-}" ]; then PY=("$ISAACLAB_PY")
elif [ -x "$LAB/.venv/bin/python" ]; then PY=("$LAB/.venv/bin/python")
elif [ -x "$LAB/isaaclab.sh" ]; then PY=("$LAB/isaaclab.sh" -p)
else echo "Isaac Lab 을 찾지 못했습니다: $LAB (ISAACLAB_DIR 또는 ISAACLAB_PY 로 지정)"; exit 1; fi
cd "$HERE"
CLEAN=(env -u PYTHONPATH -u LD_LIBRARY_PATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION
       PYTHONPATH="$HERE")
need_usd() { [ -f usd/light/waffle_4wd_light/waffle_4wd_light.usda ] && [ -f usd/full/waffle_4wd_full/waffle_4wd_full.usda ] && [ -f usd/loaded/waffle_4wd_loaded/waffle_4wd_loaded.usda ] \
             || { echo "USD 가 없어 먼저 만듭니다 (convert)"; HEADLESS=1 "${CLEAN[@]}" "${PY[@]}" convert_urdf.py || exit 1; }; }
case "${1:-}" in
  convert) shift; HEADLESS=1 "${CLEAN[@]}" "${PY[@]}" convert_urdf.py "$@" ;;
  check)   shift; need_usd; HEADLESS=1 "${CLEAN[@]}" "${PY[@]}" sim_check.py "$@" ;;
  window)  shift; need_usd; "${CLEAN[@]}" "${PY[@]}" sim_check.py --window --viz kit "$@" ;;
  maze)    shift; need_usd; "${CLEAN[@]}" "${PY[@]}" maze_view.py --viz kit "$@" ;;
  *) sed -n 2,10p "$0"; exit 2 ;;
esac
