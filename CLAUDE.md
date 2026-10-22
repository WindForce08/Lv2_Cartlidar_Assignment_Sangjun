# CLAUDE.md — Claude Code 작업 메모

Claude Code 가 이 저장소에서 세션을 시작할 때 자동으로 읽는 파일이다.
다른 PC 에서 클론해도 작업을 이어갈 수 있도록 진행 상황 · 규칙 · 명령을 여기에 남긴다.
작업 단계가 바뀌면 아래 "현재 진행 상황" 을 갱신한다. 커밋 · 푸시는 사용자가 요청할 때만 한다.

## 프로젝트

- 퍽 운반 로봇: 6륜 + 매니퓰레이터 · ROS 2 Lyrical · C++17. 메인 SBC Jetson Orin Nano, OpenCR 2대 (USB CDC).
- 펌웨어: `workspace/ros2_ws/src/opencr_firmware/` (Arduino 스케치, `COLCON_IGNORE`)
  - `OPENCR_BOARD_ROLE=1` 바퀴 보드: Dynamixel ID 1~6, Velocity Mode, 바퀴 슬롯 0~5 · IMU · 배터리
  - `OPENCR_BOARD_ROLE=2` 팔 보드: Dynamixel ID 11~16, Position Mode, 팔 슬롯 6~11
- 호스트: `workspace/ros2_ws/src/opencr_hardware/` (ros2_control SystemInterface 플러그인). 후보 포트 2개를 열고 STATE.role 로 보드를 판별한다.
- `opencr_firmware/opencr_protocol.h` 와 `opencr_hardware/include/opencr_hardware/opencr_protocol.hpp` 는 바이트 단위로 같아야 한다 (`test_protocol` 이 비교). 한쪽을 고치면 둘 다 복사한다.
- 프로토콜 v4: `FF FD | TYPE | LEN | PAYLOAD | CRC16-LE`, CMD 0x01 55 B · STATE 0x81 192 B, CRC-16/BUYPASS (다항식 0x8005, 초기값 0, 헤더~페이로드 끝).

## 절대 규칙 (사용자 지시, 2026-10-22)

- 실제 OpenCR 에 펌웨어를 업로드하지 않는다. 모터 토크 ON · 모터 구동 명령을 실행하지 않는다. 장치 쓰기 전에는 명시적 승인을 받는다.
- 패키지 설치 · 시스템 설정 변경 전에는 필요한 패키지 · 이유 · 변경 사항을 먼저 보고하고 승인을 받는다.
- 저장소의 기존 사용자 변경을 덮어쓰거나 삭제하지 않는다.
- 프로토콜 v4 필드를 근거 없이 바꾸지 않는다. 바꿔야 하면 이유와 호환성 영향을 먼저 보고한다.
- 실행하지 않은 테스트를 통과라고 보고하지 않는다. 하드웨어 없이 실제 구동 안전성이 검증되었다고 주장하지 않는다.
- 물리 E-stop 버튼 · 회로는 없다. 소프트웨어 정지 플래그를 물리 비상정지와 같다고 설명하지 않는다.
- 코드 수정 전에 분석 결과와 구현 계획을 먼저 제시하고 승인을 받는다. 문서에서 확인되지 않은 사실은 추측하지 말고 미확정으로 표시한다.

## 현재 진행 상황 — 듀얼 OpenCR 펌웨어 안전 정책

- 작업 지시 요약 · 1단계 분석 결과 · 구현 계획 · 미결 결정: [workspace/docs/firmware_safety_analysis.md](workspace/docs/firmware_safety_analysis.md)
  — 이 작업을 이어갈 때 먼저 끝까지 읽는다.
- 1단계 (읽기 전용 분석): 완료 (2026-10-22). 소스 코드 변경 없음.
- 다음: 사용자가 결정 사항 D1~D9 (분석 문서 §9) 에 답하면 2단계 계획 확정 → 3단계 구현. 답을 받기 전에는 구현하지 않는다.
- 브랜치: `feature/firmware`

## 빌드 · 테스트

```bash
# 전체 (ros2_control 이 설치된 환경에서만 opencr_hardware 가 빌드된다)
cd workspace/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test && colcon test-result --verbose
```

ros2_control (`hardware_interface`) 이 없는 머신에서는 프로토콜 · E-stop 래치 시험만 g++ 로 따로 돌릴 수 있다 (출력은 저장소 밖에 둔다):

```bash
cd workspace/ros2_ws/src/opencr_hardware
g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude \
  -DFIRMWARE_PROTOCOL_PATH="\"$PWD/../opencr_firmware/opencr_protocol.h\"" \
  -DHOST_PROTOCOL_PATH="\"$PWD/include/opencr_hardware/opencr_protocol.hpp\"" \
  test/test_protocol.cpp test/test_estop_latch.cpp -lgtest -lgtest_main -pthread -o /tmp/test_protocol
/tmp/test_protocol   # 2026-10-22 기준 9개 통과
```

펌웨어 컴파일 도구 (arduino-cli · OpenCR 보드 패키지 · Dynamixel2Arduino) 는 아직 정하지 않았다. 설치는 승인 사항이다 (분석 문서 D7).

## 머신별 환경 메모

환경은 머신마다 다르므로 새 머신에서는 다시 확인한다 (`uname -m`, `ls /dev/ttyACM*`, `command -v arduino-cli`, `ls /opt/ros/lyrical/share | grep hardware_interface`).

- 회사 노트북 (2026-10-22): x86_64 Ubuntu 24.04.5, ROS 2 lyrical. OpenCR 미연결, Arduino 도구 없음, ros2_control · xacro 미설치 → `colcon build` 의 opencr_hardware 실패.
