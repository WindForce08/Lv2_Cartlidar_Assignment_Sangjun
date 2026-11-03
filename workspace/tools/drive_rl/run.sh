#!/usr/bin/env bash
# 주행 강화학습 실행기 (Isaac Sim 6.1 / Isaac Lab 3.0). flat 학습 · 평가는 창 없이 (*_play 만 창). maze 는 창에 미로 몇 개만 보이며 학습 (GPU 8 GB 저사양).
#   ./run.sh flat [환경 수=64] [반복=300]   단계 A (평지 목표점) 학습 -> logs/rsl_rl/waffle_drive_flat/<시각>/
#   ./run.sh flat_play [환경 수=16]         최근 체크포인트를 창으로 (exported/policy.pt · policy.onnx 도 만듦)
#   ./run.sh flat_eval [환경 수=64]         최근 정책을 고정 목표 집합으로 평가 -> results/eval_flat_<회차>.json
#   ./run.sh flat_eval_pp [환경 수=64]      같은 목표 집합에서 기준선 (목표 쪽으로 돌고 직진) -> results/eval_flat_pp.json
#   ./run.sh course                         경로 (course.yaml) 검사 + course_map.png (Isaac 없이)
#   ./run.sh maze [환경=64] [반복=1000] [창에 보일 미로=4]   미로 경로 학습. 창으로 보면서 (보일 미로 0 = 창 없이, 약 2 배 빠름)
#                                           -> logs/rsl_rl/waffle_drive_maze/<시각>/, 창을 닫거나 Ctrl+C 로 멈춤 (50 반복마다 저장)
#   ./run.sh maze_start [환경=1024] [반복=300] [탐색 폭=그대로] [학습 폴더 이름=최근]
#                                           미로 학습을 이어서, 재설정의 25 % 를 출발점에서 (첫 구간 보강, 창 없이, k = 경로 끝까지)
#   ./run.sh maze_play [환경=4]             최근 미로 학습 결과를 창으로
#   ./run.sh maze_full [시작 점=0] [지날 점 수=끝까지]   최근 미로 학습 결과로 로봇 1 대가 달림 (창, 학습 안 함, 끝나면 같은 점에서 다시)
#   ./run.sh tb [실험=waffle_drive_maze]     학습 그래프 (TensorBoard) -> 브라우저 http://localhost:6006 (Ctrl+C 로 끔, 학습에는 영향 없음)
#   ./run.sh test                           base_cmd · 미로 형상 시험 (Isaac 없이)
# 작업 목록: workspace/docs/drive_rl_worklist_2026-11-03.md. 팀 로봇 설정 (urdf_and_usd/isaac) 은 읽기만 한다.
# Isaac Lab 위치: ISAACLAB_DIR=/경로/IsaacLab (기본 ~/pa-course/IsaacLab-6.1) 또는 ISAACLAB_PY=/경로/python
# ROS 2 를 source 한 터미널이어도 됨 (ROS 환경 변수는 여기서 뺌: Isaac 의 Python 과 섞이지 않게)
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
LAB="${ISAACLAB_DIR:-$HOME/pa-course/IsaacLab-6.1}"
if [ -n "${ISAACLAB_PY:-}" ]; then PY=("$ISAACLAB_PY")
elif [ -x "$LAB/.venv/bin/python" ]; then PY=("$LAB/.venv/bin/python")
elif [ -x "$LAB/isaaclab.sh" ]; then PY=("$LAB/isaaclab.sh" -p)
else echo "Isaac Lab 을 찾지 못했습니다: $LAB (ISAACLAB_DIR 또는 ISAACLAB_PY 로 지정)"; exit 1; fi
cd "$HERE"
mkdir -p logs results
CLEAN=(env -u PYTHONPATH -u LD_LIBRARY_PATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION
       -u HEADLESS PYTHONPATH="$HERE")
HL=("${CLEAN[@]}" HEADLESS=1 "${PY[@]}")   # 창 없이
GUI=("${CLEAN[@]}" "${PY[@]}")             # 창으로
TASK=Waffle-DriveFlat-Direct
EXP=waffle_drive_flat

export_latest() {   # 가장 최근 학습에 exported/policy.pt 가 없으면 play 를 창 없이 잠깐 돌려 만듦 (팀 run_rl.sh 와 같은 방식)
  local last; last=$(ls -d "$HERE"/logs/rsl_rl/"$EXP"/*/ 2>/dev/null | sort | tail -1)
  [ -n "$last" ] || { echo "학습 결과가 없습니다: logs/rsl_rl/$EXP (먼저 ./run.sh flat)"; exit 1; }
  if [ ! -f "$last/exported/policy.pt" ]; then
    echo "정책 내보내는 중: $last"
    "${HL[@]}" "$HERE/play.py" --task "$TASK" --num_envs 1 --checkpoint latest --viz none > "$HERE/logs/export_$EXP.log" 2>&1 &
    local pid=$!; until [ -f "$last/exported/policy.pt" ] || ! kill -0 $pid 2>/dev/null; do sleep 2; done; sleep 3
    kill $pid 2>/dev/null; pkill -f "[p]lay.py --task $TASK --num_envs 1 "; wait $pid 2>/dev/null
  fi
  [ -f "$last/exported/policy.pt" ] || { echo "policy.pt 를 만들지 못했습니다 (logs/export_$EXP.log)"; exit 1; }
}

case "${1:-}" in
  flat)         "${HL[@]}" train.py --task $TASK --num_envs "${2:-64}" --max_iterations "${3:-300}" --viz none ;;
  flat_play)    "${GUI[@]}" play.py --task $TASK --num_envs "${2:-16}" --checkpoint latest --viz kit ;;
  flat_eval)    export_latest; "${HL[@]}" eval_drive.py --num_envs "${2:-64}" ;;
  flat_eval_pp) "${HL[@]}" eval_drive.py --num_envs "${2:-64}" --controller pp ;;
  course)       python3 maze_map.py --course course.yaml ;;
  maze)         V="${4:-4}"
                if [ "$V" = 0 ]; then "${HL[@]}" train.py --task Waffle-DriveMaze-Direct --num_envs "${2:-64}" --max_iterations "${3:-1000}" --viz none
                else "${GUI[@]}" train.py --task Waffle-DriveMaze-Direct --num_envs "${2:-64}" --max_iterations "${3:-1000}" --viz kit --max_visible_envs "$V"; fi ;;
  maze_start)   RUN="${5:-}"
                if [ -n "$RUN" ]; then LAST="$HERE/logs/rsl_rl/waffle_drive_maze/$RUN/"
                else LAST=$(for d in $(ls -d "$HERE"/logs/rsl_rl/waffle_drive_maze/*/ 2>/dev/null | sort -r); do   # 체크포인트가 있는 가장 최근 폴더
                  ls "$d"model_*.pt >/dev/null 2>&1 && { echo "$d"; break; }; done); fi
                CK=$(ls "$LAST"model_*.pt 2>/dev/null | sort -V | tail -1)
                [ -n "$CK" ] || { echo "체크포인트가 없습니다: ${LAST:-logs/rsl_rl/waffle_drive_maze} (먼저 ./run.sh maze)"; exit 1; }
                echo "이어서 학습: $CK"
                if [ -n "${4:-}" ]; then   # 탐색 폭을 바꾼 사본으로 (cache/)
                  mkdir -p cache; NEW="$HERE/cache/$(basename "$(dirname "$CK")")_$(basename "$CK" .pt)_std$4.pt"
                  "${CLEAN[@]}" "${PY[@]}" set_std.py "$CK" "$NEW" "$4" || exit 1; CK="$NEW"
                fi
                "${HL[@]}" train.py --task Waffle-DriveMaze-Direct --num_envs "${2:-1024}" --max_iterations "${3:-300}" --viz none \
                  --checkpoint "$CK" env.curriculum_start_k=999 env.start0_prob=0.25 ;;
  maze_play)    "${GUI[@]}" play.py --task Waffle-DriveMaze-Direct --num_envs "${2:-4}" --checkpoint latest --viz kit ;;
  maze_full)    "${GUI[@]}" play.py --task Waffle-DriveMaze-Direct --num_envs 1 --checkpoint latest --viz kit env.full_course=true env.full_course_start="${2:-0}" env.full_course_k="${3:-0}" ;;
  tb)           "${CLEAN[@]}" "$(dirname "${PY[0]}")/tensorboard" --logdir "logs/rsl_rl/${2:-waffle_drive_maze}" --port 6006 ;;
  test)         "${CLEAN[@]}" "${PY[@]}" tests/test_base_cmd.py && "${CLEAN[@]}" "${PY[@]}" tests/test_maze_geom.py ;;
  *) sed -n 2,19p "$0"; exit 2 ;;
esac
