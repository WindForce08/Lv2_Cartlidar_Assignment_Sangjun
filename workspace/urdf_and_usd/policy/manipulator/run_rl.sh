#!/usr/bin/env bash
# waffle_4wd 매니퓰레이터 강화 학습 실행기 (팀 공유본 isaac/ 의 로봇 설정을 씀). Docker 컨테이너 isaac-lab-base 안에서, 또는 Isaac Lab 이 있는 PC 에서.
# 창이 필요한 것 (*_play) 만 창으로, 나머지는 창 없이. 학습 중에는 창을 띄우지 말 것 (GPU 8 GB 부족).
# B안 팔레트 (2026-11-02): 위에서 집기 + 고정 궤적 놓기
#   ./run_rl.sh toppick [환경 수] [반복]       위에서 집기 학습 -> logs/rsl_rl/waffle_toppick/
#   ./run_rl.sh toppick_play [환경 수]         위에서 집기 최근 체크포인트를 창으로
#   ./run_rl.sh toppick_eval [환경 수]         위에서 집기 실제 과제 평가 (policies/toppick_latest.pt, 기울임 · 수직 모드별)
#   ./run_rl.sh pallet_check                   팔레트 놓기 고정 궤적 확인 (칸 12 x 원기둥 · 상자, pallet_place_check.json). 궤적은 python3 pallet_traj.py
#   ./run_rl.sh pallet_check_play [대 수]      위 것을 창으로
#   ./run_rl.sh pallet_drop                    팔 없이 퍽만 칸에 떨어뜨려 팔레트 충돌 형상 확인
#   ./run_rl.sh pallet_unload                  팔레트 빼기 (놓기 궤적 거꾸로) 확인 (pallet_unload_check.json)
# 아래는 옛 퍽 보관통 (C · P · N 통) 기준 - B안 팔레트에서는 쓰지 않음
#   ./run_rl.sh sidepick [환경 수] [반복]      1 정렬·집기 학습 -> logs/rsl_rl/waffle_sidepick/
#   ./run_rl.sh sidepick_play [환경 수]        1 정렬·집기 (policies/sidepick_latest.pt) 창으로
#   ./run_rl.sh sidepick_diag [환경 수]        1 실제 과제 평가 (창 없이)
#   ./run_rl.sh store [환경 수] [반복]         2 옮기기·맞추기 학습 -> logs/rsl_rl/waffle_store/
#   ./run_rl.sh store_play [환경 수]           2 옮기기·맞추기 최근 체크포인트를 창으로
#   ./run_rl.sh store_diag [환경 수]           2 진단 (통까지 거리 · 기울기 분해 · 접촉)
#   ./run_rl.sh store_hybrid [환경 수]         2 + 손 IK 넣기 (10 mm 넣고 놓기) 평가
#   ./run_rl.sh store_hybrid_play [환경 수]    위 것을 창으로
#   ./run_rl.sh insert [환경 수] [반복]        3 넣기 학습 -> logs/rsl_rl/waffle_insert/
#   ./run_rl.sh insert_play [환경 수]          3 넣기 최근 체크포인트를 창으로
#   ./run_rl.sh insert_eval [환경 수]          3 넣기 평가 (시작 위치별 성공률 · 종료 이유)
#   ./run_rl.sh chain_play [환경 수]           1 -> 2 -> 3 (넣기 = 손 IK) 이어서 창으로 (policies/*_latest.pt)
#   ./run_rl.sh chain_eval [환경 수]           위 것을 창 없이 여러 대, 단계별 성공률
#   ./run_rl.sh chain_play_rl / chain_eval_rl  넣기를 강화 학습 모델로
#   ./run_rl.sh store_check                    통에 퍽 떨어뜨리기 + 팔-몸체 접촉 센서 확인
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
  toppick) "${PY[@]}" "$HERE/train.py" --task Waffle-TopPick-Direct --num_envs "${2:-256}" --max_iterations "${3:-300}" --viz none ;;
  toppick_play) env -u HEADLESS "${PY[@]}" "$HERE/play.py" --task Waffle-TopPick-Direct --num_envs "${2:-16}" --checkpoint latest --viz kit ;;
  toppick_eval) "${PY[@]}" "$HERE/toppick_eval.py" --num_envs "${2:-256}" ;;
  pallet_check) "${PY[@]}" "$HERE/pallet_place_check.py" ;;
  pallet_check_play) env -u HEADLESS "${PY[@]}" "$HERE/pallet_place_check.py" --gui --num_envs "${2:-4}" ;;
  pallet_drop) "${PY[@]}" "$HERE/pallet_place_check.py" --drop ;;
  pallet_unload) "${PY[@]}" "$HERE/pallet_place_check.py" --unload --row_speed "" ;;   # 빼기는 3열도 0.8 에서 24/24 (2026-11-03)
  sidepick) "${PY[@]}" "$HERE/train.py" --task Waffle-SidePick-Direct --num_envs "${2:-256}" --max_iterations "${3:-300}" --viz none ;;
  sidepick_play) env -u HEADLESS "${PY[@]}" "$HERE/play.py" --task Waffle-SidePick-Direct --num_envs "${2:-16}" --checkpoint latest --viz kit ;;
  sidepick_diag) "${PY[@]}" "$HERE/sidepick_diag.py" --num_envs "${2:-64}" --policy "$HERE/policies/sidepick_latest.pt" ;;
  store)   "${PY[@]}" "$HERE/train.py" --task Waffle-Store-Direct --num_envs "${2:-256}" --max_iterations "${3:-300}" --viz none ;;
  store_play) env -u HEADLESS "${PY[@]}" "$HERE/play.py" --task Waffle-Store-Direct --num_envs "${2:-16}" --checkpoint latest --viz kit ;;
  store_diag) export_latest Waffle-Store-Direct waffle_store; "${PY[@]}" "$HERE/store_diag.py" --num_envs "${2:-8}" ;;
  store_hybrid) "${PY[@]}" "$HERE/store_hybrid.py" --num_envs "${2:-16}" ;;
  store_hybrid_play) env -u HEADLESS "${PY[@]}" "$HERE/store_hybrid.py" --num_envs "${2:-8}" --gui --viz kit ;;
  insert)  "${PY[@]}" "$HERE/train.py" --task Waffle-Insert-Direct --num_envs "${2:-256}" --max_iterations "${3:-300}" --viz none ;;
  insert_play) env -u HEADLESS "${PY[@]}" "$HERE/play.py" --task Waffle-Insert-Direct --num_envs "${2:-16}" --checkpoint latest --viz kit ;;
  insert_eval) export_latest Waffle-Insert-Direct waffle_insert; "${PY[@]}" "$HERE/insert_eval.py" --num_envs "${2:-192}" ;;
  chain_play) env -u HEADLESS "${PY[@]}" "$HERE/chain_play.py" --num_envs "${2:-4}" --gui --viz kit --insert_ik ;;
  chain_eval) "${PY[@]}" "$HERE/chain_play.py" --num_envs "${2:-64}" --insert_ik ;;
  chain_play_rl) env -u HEADLESS "${PY[@]}" "$HERE/chain_play.py" --num_envs "${2:-4}" --gui --viz kit ;;
  chain_eval_rl) "${PY[@]}" "$HERE/chain_play.py" --num_envs "${2:-64}" ;;
  store_check) "${PY[@]}" "$HERE/store_check.py" ;;
  *) sed -n 2,27p "$0"; exit 2 ;;
esac
