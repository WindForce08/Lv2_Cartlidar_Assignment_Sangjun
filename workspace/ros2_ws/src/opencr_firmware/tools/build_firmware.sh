#!/usr/bin/env bash
# build_firmware.sh — OpenCR 펌웨어를 보드 역할별로 빌드 (· 업로드) 한다 (Firmware 팀). README.md 참고.
#
#   tools/build_firmware.sh wheel                 주행 보드 펌웨어 (OPENCR_BOARD_ROLE=1, 바퀴 ID 1~4)
#   tools/build_firmware.sh arm                   팔 보드 펌웨어   (OPENCR_BOARD_ROLE=2, 팔 ID 11~16)
#   tools/build_firmware.sh setup                 모터 사전 설정 도구 (tools/dxl_setup, 토크 ON 명령 없음)
#   tools/build_firmware.sh arm --upload PORT     빌드 후 업로드 (보드 역할을 한 번 더 확인함)
#
# 결과: $OPENCR_BUILD_DIR (기본 ~/opencr_builds) / opencr_wheel_v5.bin · opencr_arm_v5.bin · dxl_setup.bin
# arduino-cli: $ARDUINO_CLI → PATH → ~/.local/opt/arduino-cli-1.5.1/arduino-cli 순서로 찾는다.
# 업로드가 멈추면 (부트로더로 안 넘어감) OpenCR 을 Recovery Mode 로 넣고 다시 실행: PUSH SW2 를 누른 채 Reset → SW2 놓기.
set -euo pipefail

usage() { sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
[[ $# -ge 1 ]] || usage
target=$1; shift
port=""
while [[ $# -gt 0 ]]; do
  case $1 in
    --upload) [[ $# -ge 2 ]] || usage; port=$2; shift 2 ;;
    *) usage ;;
  esac
done

fw_dir=$(cd "$(dirname "$0")/.." && pwd)
version=$(sed -n 's/^static const uint8_t PROTOCOL_VERSION = \([0-9]*\);.*/\1/p' "$fw_dir/opencr_protocol.h")
case $target in
  wheel) sketch=$fw_dir; role=1; name=opencr_wheel_v$version; desc="주행 보드 (역할 1 · 바퀴 ID 1~4 · Velocity)" ;;
  arm)   sketch=$fw_dir; role=2; name=opencr_arm_v$version;   desc="팔 보드 (역할 2 · 팔 ID 11~16 · Position)" ;;
  setup) sketch=$fw_dir/tools/dxl_setup; role=""; name=dxl_setup; desc="모터 사전 설정 도구 (dxl_setup)" ;;
  *) usage ;;
esac

cli=${ARDUINO_CLI:-$(command -v arduino-cli || true)}
[[ -n $cli ]] || cli=$HOME/.local/opt/arduino-cli-1.5.1/arduino-cli
[[ -x $cli ]] || { echo "arduino-cli 를 찾지 못함 (ARDUINO_CLI 로 지정)" >&2; exit 1; }

out=${OPENCR_BUILD_DIR:-$HOME/opencr_builds}
build=$out/build_$target
mkdir -p "$out"
args=(compile --fqbn OpenCR:OpenCR:OpenCR --build-path "$build")
[[ -n $role ]] && args+=(--build-property "compiler.cpp.extra_flags=-DOPENCR_BOARD_ROLE=$role")
echo "== 빌드: $desc"
"$cli" "${args[@]}" "$sketch"
cp "$build/$(basename "$sketch").ino.bin" "$out/$name.bin"
echo "== 결과: $out/$name.bin  sha256 $(sha256sum "$out/$name.bin" | cut -c1-16)…"

[[ -n $port ]] || exit 0
echo
echo "== 업로드 대상: $desc → $port"
echo "   다른 역할의 보드에 올리면 모터가 엉뚱하게 움직일 수 있음. 보드 확인: opencr_arm_tool.py monitor 의 role (1 주행 · 2 팔)"
read -r -p "   이 보드가 맞으면 yes 입력: " answer
[[ $answer == yes ]] || { echo "업로드 취소"; exit 1; }
"$cli" upload -p "$port" -b OpenCR:OpenCR:OpenCR --input-dir "$build" "$sketch"
