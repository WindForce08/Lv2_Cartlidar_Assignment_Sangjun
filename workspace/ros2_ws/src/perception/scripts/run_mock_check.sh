#!/usr/bin/env bash
# 인지 노드 모의 시험 M1~M5 실행 (PC, Isaac Sim 없이). 결과: <결과 폴더>/mock_check_result.json
#   scripts/run_mock_check.sh <URDF 파일> <결과 폴더> [perception install setup.bash]
# 다른 장비·세션 토픽과 섞이지 않게 ROS_DOMAIN_ID=87, 같은 PC 안 통신으로만 돈다.
# 끝나면(Ctrl+C 포함) 띄운 robot_state_publisher · perception 컨테이너를 모두 끈다.
set -u
URDF=${1:?사용: run_mock_check.sh <URDF> <결과 폴더> [setup.bash]}
OUT=${2:?결과 폴더}
SETUP=${3:-}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT"
[ -e "$OUT/mock_check_result.json" ] && { echo "이미 결과가 있음: $OUT (다른 폴더 사용)"; exit 1; }

export ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
set +u   # ROS setup 스크립트는 정의 안 된 변수를 읽는다
source /opt/ros/lyrical/setup.bash
[ -n "$SETUP" ] && source "$SETUP"
set -u

# URDF 는 140 KB 넘어 명령 인자로 못 넘긴다 → 파라미터 파일
python3 - "$URDF" "$OUT/rsp_params.yaml" <<'EOF'
import sys, yaml
yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {
    'robot_description': open(sys.argv[1]).read(), 'use_sim_time': True}}}, open(sys.argv[2], 'w'))
EOF

set -m   # 백그라운드 작업마다 자기 프로세스 그룹 → 그룹째 끈다
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file "$OUT/rsp_params.yaml" > "$OUT/rsp.log" 2>&1 &
RSP=$!
ros2 launch perception perception.launch.py use_sim_time:=true > "$OUT/perception_launch.log" 2>&1 &
LAUNCH=$!
cleanup() {
  kill -INT -"$LAUNCH" -"$RSP" 2>/dev/null
  for _ in $(seq 1 50); do
    kill -0 -"$LAUNCH" 2>/dev/null || kill -0 -"$RSP" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL -"$LAUNCH" -"$RSP" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM
sleep 4
python3 "$HERE/mock_check.py" --urdf "$URDF" --out "$OUT" 2>&1 | tee "$OUT/mock_check.out"
