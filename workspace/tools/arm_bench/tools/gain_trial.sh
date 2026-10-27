#!/usr/bin/env bash
# 게인 후보 하나를 실물로 기록 (장치 쓰기: 토크 ON · 이동 — 승인 뒤에만). 보드에 그 후보 펌웨어가 먼저 올라가 있어야 함.
#   tools/gain_trial.sh <이름> [포트]            예: tools/gain_trial.sh P800_I0
#   tools/gain_trial.sh <이름> [포트] --dry-run   현재 위치 · 계획만 (쓰기 없음)
# 순서: J2 ±0.1 · J3 ±0.1 (속도 0.1 rad/s, 머묾 4 s) → J2 HOLD 다시 잡기 3 번 (move --delta 0) → tools/gain_report.py 판정.
#   결과 logs/gain/<이름>/*.csv + report.txt. 같은 포트를 쓰는 슬라이더 · 모니터는 먼저 끔. 오류 · 추종 오차면 도구가 ESTOP 후 멈춤.
#   팔은 J2 · J3 를 ±0.1 rad (약 6°) 움직여도 닿는 곳이 없는 자세에 두고 시작 (도구가 config.h 한계도 확인).
set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
name="${1:?후보 이름 (예: P800_I0)}"
port="${2:-/dev/ttyACM0}"
mode="--yes"
[ "${3:-}" = "--dry-run" ] && mode="--dry-run"
out="$HERE/logs/gain/$name"
mkdir -p "$out"
tool=(python3 "$HERE/../../ros2_ws/src/opencr_hardware/scripts/opencr_arm_tool.py" move --port "$port" --speed 0.1 "$mode")
run() { local f="$1"; shift; echo "== $f"; "${tool[@]}" "$@" --log "$out/$f.csv"; }
run j2_up   --joint 2 --delta 0.1  --dwell 4
run j2_down --joint 2 --delta -0.1 --dwell 4
run j3_up   --joint 3 --delta 0.1  --dwell 4
run j3_down --joint 3 --delta -0.1 --dwell 4
for k in 1 2 3; do run "hold$k" --joint 2 --delta 0 --dwell 2; done
[ "$mode" = "--dry-run" ] && { rm -rf "$out"; exit 0; }
python3 "$HERE/tools/gain_report.py" "$out"/*.csv | tee "$out/report.txt"
