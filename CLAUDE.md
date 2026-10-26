# CLAUDE.md — Claude Code 작업 메모

Claude Code 가 이 저장소에서 세션을 시작할 때 자동으로 읽는 파일이다.
다른 PC 에서도 작업을 이어갈 수 있도록 진행 상황 · 규칙 · 명령을 여기에 남긴다.
**이 파일 · AGENTS.md · codex/ 는 AI 도구 메모라 팀 저장소 (origin) 에 올리지 않는다** (.gitignore). 개인 저장소 backup 의 `ai-notes` 브랜치에만 둔다
(메모 파일만 든 별도 브랜치, 작업 브랜치와 이력 무관). 아래 명령은 작업 브랜치의 index · 파일을 건드리지 않는다:

```bash
git fetch backup ai-notes && git archive backup/ai-notes | tar -x          # 받기 (저장소 루트에서)
i=$(mktemp) && GIT_INDEX_FILE=$i git read-tree --empty && GIT_INDEX_FILE=$i git add -f CLAUDE.md AGENTS.md codex \
  && c=$(git commit-tree $(GIT_INDEX_FILE=$i git write-tree) -p backup/ai-notes -m "notes: update") && rm $i \
  && git push backup $c:refs/heads/ai-notes                                  # 고친 뒤 올리기 (먼저 받기의 fetch)
```

작업 단계가 바뀌면 아래 "현재 진행 상황" 을 갱신한다. 커밋 · 푸시는 사용자가 요청할 때만 한다.

## 프로젝트

- 퍽 운반 로봇: 주행 4륜 (2026-10-26 6륜에서 가운데 2개 제거) + 매니퓰레이터 · ROS 2 Lyrical · C++17. 메인 SBC Jetson Orin Nano, OpenCR 2대 (USB CDC).
- 펌웨어: `workspace/ros2_ws/src/opencr_firmware/` (Arduino 스케치, `COLCON_IGNORE`). 팀원용 설명: 그 폴더의 `README.md`.
  - 같은 소스를 역할별로 따로 빌드한다. **역할은 빌드 옵션으로만** (기본값 없음, 없으면 ino 의 `#error`): `tools/build_firmware.sh wheel|arm|setup`
  - `OPENCR_BOARD_ROLE=1` 주행 보드: 바퀴 Dynamixel ID 1~4, Velocity Mode, 바퀴 칸 0~3 (칸 4 · 5 는 STATE 0) · IMU · 배터리
  - `OPENCR_BOARD_ROLE=2` 팔 보드: Dynamixel ID 11~16, Position Mode, 팔 칸 6~11
- 호스트: `workspace/ros2_ws/src/opencr_hardware/` (ros2_control SystemInterface 플러그인). 후보 포트 2개를 열고 STATE.role 로 보드를 판별한다.
  `wheel_count` 기본 4 (xacro · launch), `config/controllers_4wd.yaml`.
- `opencr_firmware/opencr_protocol.h` 와 `opencr_hardware/include/opencr_hardware/opencr_protocol.hpp` 는 바이트 단위로 같아야 한다 (`test_protocol` 이 비교). 한쪽을 고치면 둘 다 복사한다.
- 프로토콜: `FF FD | TYPE | LEN | PAYLOAD | CRC16-LE`, CMD 0x01 55 B · STATE 0x81 192 B, CRC-16/BUYPASS (다항식 0x8005, 초기값 0, 헤더~페이로드 끝).
  - 코드는 **v5** (2026-10-26, 사용자 결정 "전체 v5 전환"): v4 와 길이 · 오프셋 같고 CMD `FLAG_REARM 0x08`, STATE `ERR_HOLD 0x40` · `ERR_FAULT 0x80` 추가.
  - 팀 규격 `workspace/docs/interfaces.md` 의 **승인본 (dev) 은 v3** (OpenCR 1대, CMD 54 B · STATE 191 B, 4륜 ID 1~4). 이 브랜치는 10/22 `a817e2e` 에서 승인 없이 v4 · 6륜으로 고쳤고,
    2026-10-26 사용자 결정으로 **v5 · 4륜 제안**으로 다시 고침 (PR 에서 Controller 팀장 승인 필요, 승인되지 않음). **승인 전 dev/main 머지 금지.**
    변경 요청서: `workspace/docs/opencr_v5_change_request.md` (v3 → v5 · 팔 6 번째 칸 = 그리퍼 개폐 · 부팅 점검 · dev manipulator.yaml 과 다른 점).
- 펌웨어 안전 판단은 `opencr_firmware/safety_core.h` (Arduino 의존 없음, ino 가 include). 팔 HOLD 는 REARM 상승 에지로만 풀린다.

## 절대 규칙 (사용자 지시, 2026-10-22)

- 실제 OpenCR 에 펌웨어를 업로드하지 않는다. 모터 토크 ON · 모터 구동 명령을 실행하지 않는다. 장치 쓰기 전에는 명시적 승인을 받는다.
  - 2026-10-26 사용자 지시로 변경: 팔 OpenCR 1대를 PC 에 직접 연결해 모터 사전 설정 → 펌웨어 업로드 → 저속 이동 시험을 진행한다.
    장치 쓰기 (업로드 · 모터 EEPROM · 토크 ON · 이동) 는 **단계마다** 명시적 승인을 받은 뒤에만 한다. 커밋은 여전히 지시 전 금지.
- 패키지 설치 · 시스템 설정 변경 전에는 필요한 패키지 · 이유 · 변경 사항을 먼저 보고하고 승인을 받는다.
- 저장소의 기존 사용자 변경을 덮어쓰거나 삭제하지 않는다.
- 프로토콜 필드 (v5) 를 근거 없이 바꾸지 않는다. 바꿔야 하면 이유와 호환성 영향을 먼저 보고한다.
- 실행하지 않은 테스트를 통과라고 보고하지 않는다. 하드웨어 없이 실제 구동 안전성이 검증되었다고 주장하지 않는다.
- 물리 E-stop 버튼 · 회로는 없다. 소프트웨어 정지 플래그를 물리 비상정지와 같다고 설명하지 않는다.
- 코드 수정 전에 분석 결과와 구현 계획을 먼저 제시하고 승인을 받는다. 문서에서 확인되지 않은 사실은 추측하지 말고 미확정으로 표시한다.
- 다른 팀 파일 (`control/config/manipulator.yaml` 등 Controller 팀) 은 고치지 않고 알릴 내용만 정리한다.

## 현재 진행 상황 — 듀얼 OpenCR 펌웨어

문서: 분석 · 결정 [workspace/docs/firmware_safety_analysis.md](workspace/docs/firmware_safety_analysis.md) (이어서 할 때 먼저 읽음) ·
팔 실기 기록 [workspace/docs/arm_bench_test_2026-10-26.md](workspace/docs/arm_bench_test_2026-10-26.md) ·
규격 변경 요청 [workspace/docs/opencr_v5_change_request.md](workspace/docs/opencr_v5_change_request.md) · 팀원용 [opencr_firmware/README.md](workspace/ros2_ws/src/opencr_firmware/README.md).

- 결정 (2026-10-26): D1 = B + v5 (REARM 상승 에지, ESTOP 해제에도 REARM). D5 구현 — D5-1 OpenCR 재부팅 때 팔 토크가 켜져 있으면 유지 ·
  D5-2 호스트 활성화 중 CMD 미송신 · D5-4 팔 토크 OFF 는 HOLD + TORQUE_ON 없는 CMD 연속 `ARM_TORQUE_OFF_CMDS` (50 ★). D3 · D4 · D6 **미결 — 구현 안 함**.
  코드 리뷰 R1 · R2 · R4 · R5 · R6 반영, 남음 R3 (REARM 점프 — D4 허용오차) · R7~R9 정리.
- 커밋 · 브랜치 (2026-10-26, 사용자 지시):
  - `feature/firmware` (origin · backup 같음, AI 메모 없는 이력): e71db83 · a57f48b (10/22 · 10/23 update) → 5b6c6c5 v5 안전 정책 · SIL · 팔 실기 →
    95eff5f 주행 4륜 · 역할별 빌드 · 문서 → 6b6dcd9 origin/dev 병합 (충돌 0) → 892e8f4 AI 메모 ignore. colcon test 77 실패 0 (병합 뒤).
    이 이력은 AI 메모를 뺀 채 다시 만든 것 — 예전 이력 (메모 포함) 은 `archive/feature-firmware-with-notes` (로컬 · backup).
  - **PR #6** (2026-10-26 사용자가 웹에서 올림): origin `feature/firmware` → `dev`. interfaces.md 변경이 있어 **Controller 팀장 승인 뒤에만 병합**. gh CLI 없음.
  - 팔 보드 바이너리는 커밋 전후 같음 (sha256 ab02c1cc…). xacro 미설치라 URDF 전개는 확인 못 함.
  - dev (80c3d3d) 의 mission · perception 은 이 PC 에 nav2_msgs · libzbar-dev 가 없어 빌드 안 됨 (dev 와 같은 파일).
- 팔 실기 (PC 직결, 상세는 실기 기록): 6대 W350 · ID {11, 12, 13, 16, 14, 15} (joint4 = 16 은 사용자 지시로 13 → 16) · ARM_DIR 모두 +1 ·
  모터 EEPROM 위치 한계 = config 범위 (부팅 점검 `checkArmLimits` 가 확인). J1~J5 ±0.1 rad 명령 추종 확인. 보드에는 ARM 펌웨어 (sha256 ab02c1cc…) 가 올라가 있음.
- 미결 (실기):
  - 중력 처짐 · 래칫 (P 게인만) → 게인 개선 계획 제시함 (P↑ + 작은 I, 펌웨어 부팅 때 RAM 쓰기, config ★, 그리퍼 칸 I = 0, 안전 (a) 추천) — **승인 대기**.
  - 시작 자세 (J1 +90° · J2 −2.083 · J3 +1.546 · J4 −1.865 rad) 가 범위 밖 → A안 (J2 최소 −2.14 · J3 최대 +1.60 · J4 최소 −1.92 ★) 선택, 그리퍼 범위 측정 후 승인 예정.
  - 그리퍼 결합이 바뀌어 0/4095 경계를 지남 (닫힘 611 · 열림 −1293 누적) → 열린 상태에서 "한바퀴안" ≈ 1100 으로 다시 결합 필요.
  - 주행 자세 (시뮬 `~/GIT/FreeCAD/TeamRobot/Sim3D/sim3d.html` `poseFor('drive')` = J2 −1.4356 · J3 +1.1497 · J4 +0.8094) — 경로 확인 후 도구 `--pose drive|start`.
- 주행 보드: 실물 미시험. 실물 작업 — 모터 ID 3 → 2 · 4 → 3 · 6 → 4 (한 대씩), `WHEEL_DIR` 실측 (바퀴 띄우고).
- 다음: PR #6 승인 대기 · 게인 · A안 · 그리퍼 → 주행 자세 → Jetson 세팅 (dialout · udev · aarch64 업로드 도구 없음 · 두 보드 ros2_control 시험).
  그 밖: v5 팀 승인 요청, D3 · D4 · D6 결정, 주행용 벤치 도구 (나중에).
- Jetson `cart11@cart11.local` (Orin Nano Super, Ubuntu 24.04 aarch64, JetPack 7.2.1). OpenCR by-id `usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00`
  (두 보드가 같음). cart11 dialout 아님 · ModemManager 동작 · OpenCR 업로드 도구가 aarch64 용으로 없음.
- 브랜치: `feature/firmware`

## 빌드 · 테스트

```bash
# 전체 (ros2_control 이 설치된 환경에서만 opencr_hardware 가 빌드된다)
cd workspace/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test && colcon test-result --verbose
```

opencr_hardware 시험 (2026-10-26 기준 colcon test 77 건 실패 0 — gtest 72 + ctest 묶음 5):

| 시험 | 대상 | ROS 필요 |
|---|---|---|
| test_protocol (+ test_estop_latch) | 프로토콜 헤더 · 호스트 잠금 · REARM 펄스 | 아니오 |
| test_safety_core | 실제 펌웨어 판단 코드 (safety_core.h) | 아니오 |
| test_firmware_sil | 실제 opencr_firmware.ino (주행 · 팔) + 테스트 더블 (test/sil: Dynamixel 버스 · USB · 가상 시계) | 아니오 |
| test_hardware_loopback | 실제 호스트 플러그인 + FakeOpenCR (테스트 더블) | 예 |
| test_host_firmware_sil | 실제 호스트 ↔ pty ↔ 실제 펌웨어 SIL 두 보드 | 예 |

ros2_control (`hardware_interface`) 이 없는 머신에서는 ROS 가 필요 없는 시험만 g++ 로 따로 돌린다 (출력은 저장소 밖):

```bash
cd workspace/ros2_ws/src/opencr_hardware
FW=$PWD/../opencr_firmware; OUT=/tmp/opencr_tests; mkdir -p $OUT
g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude \
  -DFIRMWARE_PROTOCOL_PATH="\"$FW/opencr_protocol.h\"" \
  -DHOST_PROTOCOL_PATH="\"$PWD/include/opencr_hardware/opencr_protocol.hpp\"" \
  test/test_protocol.cpp test/test_estop_latch.cpp -lgtest -lgtest_main -pthread -o $OUT/test_protocol && $OUT/test_protocol
g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude -I$FW test/test_safety_core.cpp -lgtest -lgtest_main -pthread -o $OUT/test_safety_core && $OUT/test_safety_core
for r in "wheel 1" "arm 2"; do set -- $r
  g++ -std=c++17 -c test/sil/sil_firmware.cpp -Itest/sil -Itest/sil/stubs -I$FW -DSIL_NS=sil_$1 -DSIL_ACCESSOR=sil_$1_board \
    -DOPENCR_BOARD_ROLE=$2 -DFIRMWARE_INO="\"$FW/opencr_firmware.ino\"" -o $OUT/sil_$1.o
done
g++ -std=c++17 test/test_firmware_sil.cpp $OUT/sil_wheel.o $OUT/sil_arm.o -Iinclude -Itest/sil -I$FW -lgtest -lgtest_main -pthread \
  -o $OUT/test_firmware_sil && $OUT/test_firmware_sil
```

펌웨어 실제 타깃 빌드 (업로드 아님). 결과는 `$OPENCR_BUILD_DIR` (기본 `~/opencr_builds`) 의 `opencr_wheel_v5.bin` · `opencr_arm_v5.bin` · `dxl_setup.bin`:

```bash
cd workspace/ros2_ws/src/opencr_firmware
tools/build_firmware.sh wheel && tools/build_firmware.sh arm && tools/build_firmware.sh setup
# 업로드: tools/build_firmware.sh arm --upload <포트>  (yes 확인, 멈추면 Recovery Mode: PUSH SW2 누른 채 Reset)
```

## 머신별 환경 메모

환경은 머신마다 다르므로 새 머신에서는 다시 확인한다 (`uname -m`, `ls /dev/ttyACM*`, `command -v arduino-cli`, `ls /opt/ros/lyrical/share | grep hardware_interface`).

- 회사 노트북 (2026-10-22): x86_64 Ubuntu 24.04.5, ROS 2 lyrical. OpenCR 미연결, Arduino 도구 없음, ros2_control · xacro 미설치 → `colcon build` 의 opencr_hardware 실패.
- 같은 노트북 (2026-10-26): 사용자가 `ros-lyrical-hardware-interface` 6.9.0 설치 → opencr_hardware 빌드 · 시험 가능. xacro · 컨트롤러 패키지는 여전히 없음.
  사용자 영역 설치 (승인): arduino-cli 1.5.1 (`~/.local/opt/arduino-cli-1.5.1`), OpenCR 보드 패키지 1.5.3 (`~/.arduino15`, 툴체인 gcc 5.4 는 32비트지만 그대로 실행됨),
  Dynamixel2Arduino 0.8.2 (`~/Arduino/libraries`). sudo 는 암호 필요 — 시스템 설치는 사용자가 직접 한다.
  PC 에 dialout · udev 규칙 적용 완료 (2026-10-26). OpenCR 은 업로드 때 부트로더로 자동 전환되지 않아 Recovery Mode 필요.
  scratchpad 읽기 도구 (저장소 밖): `watch_tick.py` (dxl_setup 용) · `watch_state.py` (ARM 펌웨어 STATE: 각도 · tick · 전류).
