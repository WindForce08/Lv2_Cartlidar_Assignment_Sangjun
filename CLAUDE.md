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
- 프로토콜: `FF FD | TYPE | LEN | PAYLOAD | CRC16-LE`, CMD 0x01 55 B · STATE 0x81 192 B, CRC-16/BUYPASS (다항식 0x8005, 초기값 0, 헤더~페이로드 끝).
  - 코드는 **v5** (2026-10-26, 사용자 결정 "전체 v5 전환"): v4 와 길이 · 오프셋 같고 CMD `FLAG_REARM 0x08`, STATE `ERR_HOLD 0x40` · `ERR_FAULT 0x80` 추가.
  - 팀 규격 `workspace/docs/interfaces.md` 는 v4 그대로다 (Controller 팀장 승인 필요, 승인되지 않음). **승인 전 dev/main 머지 금지.** interfaces.md 는 승인 없이 고치지 않는다.
- 펌웨어 안전 판단은 `opencr_firmware/safety_core.h` (Arduino 의존 없음, ino 가 include). 팔 HOLD 는 REARM 상승 에지로만 풀린다.

## 절대 규칙 (사용자 지시, 2026-10-22)

- 실제 OpenCR 에 펌웨어를 업로드하지 않는다. 모터 토크 ON · 모터 구동 명령을 실행하지 않는다. 장치 쓰기 전에는 명시적 승인을 받는다.
  - 2026-10-26 사용자 지시로 변경: 팔 OpenCR 1대를 PC 에 직접 연결해 모터 사전 설정 → 펌웨어 업로드 → 저속 이동 시험을 진행한다.
    장치 쓰기 (업로드 · 모터 EEPROM · 토크 ON · 이동) 는 **단계마다** 명시적 승인을 받은 뒤에만 한다. 커밋은 여전히 지시 전 금지.
- 패키지 설치 · 시스템 설정 변경 전에는 필요한 패키지 · 이유 · 변경 사항을 먼저 보고하고 승인을 받는다.
- 저장소의 기존 사용자 변경을 덮어쓰거나 삭제하지 않는다.
- 프로토콜 v4 필드를 근거 없이 바꾸지 않는다. 바꿔야 하면 이유와 호환성 영향을 먼저 보고한다.
- 실행하지 않은 테스트를 통과라고 보고하지 않는다. 하드웨어 없이 실제 구동 안전성이 검증되었다고 주장하지 않는다.
- 물리 E-stop 버튼 · 회로는 없다. 소프트웨어 정지 플래그를 물리 비상정지와 같다고 설명하지 않는다.
- 코드 수정 전에 분석 결과와 구현 계획을 먼저 제시하고 승인을 받는다. 문서에서 확인되지 않은 사실은 추측하지 말고 미확정으로 표시한다.

## 현재 진행 상황 — 듀얼 OpenCR 펌웨어 안전 정책

- 작업 지시 요약 · 1단계 분석 결과 · 구현 계획 · 미결 결정: [workspace/docs/firmware_safety_analysis.md](workspace/docs/firmware_safety_analysis.md)
  — 이 작업을 이어갈 때 먼저 끝까지 읽는다.
- 1단계 (읽기 전용 분석): 완료 (2026-10-22).
- 2026-10-26 실기 없는 소프트웨어 검증 (미커밋, 사용자 승인 범위):
  - 결정: D1 = B + v5 (REARM 상승 에지, ESTOP 해제에도 REARM 필수). D3 · D4 · D6 은 **미결 — 구현하지 않음** (현재 동작 유지).
    D3 바퀴는 복구 후 자동 재개 · D4 HW 오류 · 연속 누락 FAULT 없음 (FAULT = 부팅 점검 실패만), REARM 연속성 허용오차 없음 · D6 바퀴 범위 초과 clamp.
  - D5 결정 · 구현 (2026-10-26, 미커밋): D5-1 OpenCR 재부팅 때 팔 토크가 켜져 있으면 유지 (EEPROM 은 읽어서 검사, 일부만 ON 이면 FAULT, 바퀴는 기존대로 OFF) ·
    D5-2 호스트 활성화 중 CMD 미송신 (토크 ON 은 첫 write(), 팔은 REARM 까지 HOLD) · D5-4 팔 토크 OFF 는 HOLD + TORQUE_ON 없는 CMD 연속
    `ARM_TORQUE_OFF_CMDS` (임시 50 ★) 일 때만, 바퀴는 즉시. 호스트에는 운용 중 팔 토크를 끄는 경로가 없음 (전원 차단만).
  - 코드 리뷰 (2026-10-26) 반영: R1 거부 프레임의 ESTOP 도 정지 · R2 호스트가 URDF min/max 로 팔 명령을 자름 · R4 리셋을 누른 채 풀려도 REARM ·
    R5 응답 없는 모터 HW 오류 비트 유지 · 폴링 생략 · R6 활성화 실패 원인 로그 구분. 남음: R3 REARM 점프 (D4 허용오차), R7~R9 정리.
  - 펌웨어: safety_core 연동 (역할 · NaN/Inf · 팔 범위 · seq 검증, 검증 통과 CMD 만 타이머 갱신, 팔 HOLD 래치 · REARM, 위치 모르면 토크 ON 거부),
    SyncRead 실패 칸 값 버림, HW 오류 마스크 버그 수정, 수신 바이트 상한.
  - 호스트: listen_only (CMD 미송신 통신 확인, 모터 없는 보드도 활성화), auto_rearm (벤치 전용), HOLD 상승 → 잠금 trip, 리셋 버튼 → REARM 펄스,
    활성화 전 두 보드 ERR_CMD_TIMEOUT 대기 (seq 기준 초기화), 운용 중 버전 불일치 = 링크 오류.
  - 시험: colcon test 76 건 실패 0 (2026-10-26, D5 · 리뷰 반영 후). 실제 타깃 컴파일 역할 1 · 2 성공. 실기 · 업로드 · 토크 · 모터 구동은 하지 않음.
- 실기 준비 (2026-10-26, 미커밋):
  - Jetson `cart11@cart11.local` (Orin Nano Super, Ubuntu 24.04 aarch64, JetPack 7.2.1, ROS lyrical). 팔 OpenCR 이 by-id
    `usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00` → ttyACM0 로 보였음. cart11 dialout 아님 · ModemManager 동작 ·
    업로드 도구 없음 (OpenCR 보드 패키지에 aarch64 도구가 없음). 지금은 Jetson 없이 OpenCR 을 PC 에 직접 연결해 진행.
  - 도구: `opencr_firmware/tools/dxl_setup/dxl_setup.ino` (모터 탐색 · ID · 통신 속도 설정, 토크 ON 명령 없음),
    `opencr_hardware/scripts/opencr_arm_tool.py` (monitor = 쓰기 없음 · move = v5 로 토크 ON → REARM → 0.1 rad/s 이하 저속 이동, --yes 필요).
    호스트 플러그인은 보드 2대가 있어야 활성화되므로 팔 보드 단독 시험은 이 도구로 한다.
  - 시험: 두 도구 모두 실제 OpenCR 툴체인으로 컴파일 성공. arm_tool 은 가상 OpenCR (실제 펌웨어 SIL + pty, 저장소 밖) 에서
    monitor · dry-run · 관절 이동 · 영점 · Ctrl-C 중단 확인. 실기에서는 아직 실행하지 않음.
- 실기 진행 (2026-10-26, PC 직결): PC dialout · udev 규칙 완료 (ModemManager 무시 확인). OpenCR 이 자동으로 부트로더에 들어가지 않아
  e-Manual Recovery Mode (PUSH SW2 누른 채 Reset) 로 dxl_setup 업로드 성공. 빌드 산출물은 `~/opencr_builds/` (dxl_setup · arm_v5).
  - joint1 모터: ID 11 · 1 Mbps · Position · Homing Offset 0 이미 설정됨. **모델 1020 = XM430-W350** (사용자: 팔 6대 모두 W350)
    → `ARM_MODEL` 1030 → 1020 으로 변경 (URDF 속도 · 토크 값은 이미 W350 기준). colcon test 76 건 실패 0 재확인.
  - joint1 정면 기준 자세 = tick ≈ 4100 (한 바퀴 안 4) — 펌웨어 영점 2048 과 약 180° 차이이고 Position 모드 0/4095 경계 위.
    사용자 결정: **혼 재조립 (정면 ≈ 2048)** → 재조립 후 정면 = tick 2043~2044 (−0.4°, Homing Offset 0 유지) 확인.
    정면 기준: URDF base_link +x (앞바퀴 · 전방 라이다 쪽), arm_base_link rpy 0 → 팔이 뻗는 방향이 로봇 앞일 때 tick 2048.
  - joint2 모터: ID 12 · 1020 · 1 Mbps · Position · Homing Offset 0 (설정 변경 불필요). 윗팔 수직 = tick 2035 (−1.1°).
    스캔 (7 통신 속도 × ID 0~252) 에 11 · 12 만 보임. 읽기 도구: scratchpad `watch_tick.py` (dxl_setup `r` 반복, 저장소 밖).
  - joint3 모터: ID 13 · 1020 · 1 Mbps · Position · Homing Offset 0. 아래팔 수평 = tick 2053 (단 그때 J2 2073 → 영점 오차 최대 ≈ +3.8° 가능, 대충 측정).
    사용자 결정: 지금은 대충 맞춘 영점으로 진행, **정밀 영점은 전체 범위 동작 · IK 전에 다시** (혼 재조립 또는 소프트웨어 보정 — 미결).
  - 회전 방향 (사용자 손 확인, 토크 OFF): J1 위에서 반시계 · J2 윗팔 앞으로 숙임 · J3 아래팔 끝 아래로 · J4 손목 끝 아래로 → 모두 tick 증가 = URDF + 방향 → ARM_DIR +1 맞음.
  - joint4 모터: 원래 ID 13 (J3 과 겹침) → J4 단독 연결로 스캔 (13 하나만) 후 **사용자 지시로 ID 16 으로 변경** (dxl_setup `i 13 16`, EEPROM, 승인).
    변경 후 스캔에 16 하나만 보임 · 1020 · 1 Mbps · Homing 0 · HW 0. 체인 재연결 후 스캔 11 · 12 · 13 · 16, 손목 일자 = tick 2020 (−2.5°, 대충).
    **config.h `ARM_IDS` {11..16} (joint4 = 14, gripper_roll = 16) 와 실물이 다름 → ARM 펌웨어 업로드 · 이동 전에 ARM_IDS 를 실물에 맞게 고쳐야 함
    (J5 · gripper_roll ID 확인 후 계획 · 승인).**
  - J5 · J6 연결 (조립성 때문에 함께): 스캔 11~16 6 대 모두 1020 · 1 Mbps, ID 겹침 없음. **J5 = ID 14 · J6 = ID 15** (사용자 손 확인).
    ID 14 · 15 는 Operating Mode 1 (Velocity) — ARM 펌웨어가 부팅 때 3 으로 바꿈. J5 tick 2055 · J6 tick 3485 (영점 기준 미확정).
    J5 회전 방향: 시계 방향 (뒤에서 앞을 보고) → tick 증가 = URDF x 축 + → ARM_DIR +1.
    J5 영점: 사용자 "거의 똑바른 자세, 시뮬레이션 상 모두 0" → tick 2055 (+0.6°).
    **J6 (ID 15) 는 gripper_roll 이 아니라 그리퍼 개폐** (사용자: 모터 시계 방향 = 열림 · 반시계 = 닫힘, 측정 때 완전히 닫힘 = tick 3485).
    → 펌웨어 · 프로토콜 · URDF 의 6 번째 팔 칸 (gripper_roll) 이 실물에 없음. interfaces.md v4 와 다름 → 팀 확인 필요 (승인 없이 규격 수정 금지).
    J6 개폐 범위 (토크 OFF 손 측정, 누적 tick): 닫힘 3485 → 열림 5051 (1566 tick ≈ 138°, 열림 = tick 증가). **열림 쪽 약 54° 지점에서 4095/0 경계를 지남**
    → Position 모드 (0~4095) 로는 열 수 없어 J6 기구 결합을 180° 재조립 (열린 상태에서만 결합 가능 → 열림 기준으로 맞춤).
    **재조립 후: 열림 2945 · 닫힘 1440** (이동 1505 tick ≈ 132°, 열림 = tick 증가, 0/4095 경계에서 충분히 떨어짐). rad = 닫힘 −0.93 · 열림 +1.38.
    처리 방식 (GRIPPER_ID 경로 vs 6 번째 칸) 미결 (팀 확인).
  - **벤치 임시 매핑 반영 (2026-10-26 사용자 승인, 미커밋)**: config.h `ARM_IDS = {11, 12, 13, 16, 14, 15}` · 6 번째 칸 범위 −0.83 ~ +1.27 rad.
    opencr_arm_tool.py: move 는 관절 1~5 만 (`--joint` 1~5, `--home` 기본 1,2,3,4,5, `--order` 에 6 거부), 6 번째 칸은 시작 위치 유지.
    → 시험 시작 때 그리퍼를 반쯤 열어 둬야 함 (완전히 닫힘 −0.93 은 범위 밖 → 도구가 시작 전에 중단).
    SIL 시험 Bench 가 슬롯 → 모터를 ARM_IDS 로 찾게 수정 (`arm_motor(k)`). colcon test 76 건 실패 0.
    ARM 펌웨어 (역할 2) 재빌드 `~/opencr_builds/arm_v5/opencr_firmware.ino.bin` 128008 B · sha256 8764590b6f318edc… (업로드 안 함).
  - **ARM 펌웨어 업로드 (2026-10-26 승인)**: Recovery Mode → arduino-cli upload `[OK] Download` CRC C823CC. dxl_setup 은 지워짐.
    monitor (쓰기 없음): 100 Hz · role 2 · fw 5 · gaps 0 · err 0x02 (CMD_TIMEOUT 만 — 부팅 점검 통과, SYNCREAD · CONFIG · FAULT 없음).
    팔 [deg] 2.7 · 3.3 · −1.2 · −1.8 · 1.3 · 10.2 (그리퍼 반쯤 열림, 모두 범위 안). monitor 로 열 순서 확인 (J4 · J5 · J6 손으로 → 해당 열만 변함, 사용자).
  - **첫 이동 (승인)**: `move --joint 1 --delta 0.1 --yes` → 목표 +0.125 도달 +0.123 · 제자리 +0.025 도달 +0.026, 다른 관절 변화 ≤ 0.002 rad, 오류 없음.
    끝난 뒤 err 0x42 (CMD_TIMEOUT|HOLD, 토크 유지) · 각도 ±0.2° 안에서 안정. 토크는 켜진 상태 (끄려면 팔 받치고 전원 차단).
  - **J2 이동 (승인) — 움직이지 않음**: 목표 +0.069 (현재 −0.031 에서 +0.1) → 도달 −0.029, 제자리 −0.031. 오류 비트 없음.
    도구는 추종 오차 0.098 < --max-track 0.2 라 중단하지 않음 (목표 미도달 검사 없음 — 도구 보완 필요).
    끝난 뒤 STATE 전류 6 관절 모두 0.000 A · 이전 HOLD 중 J2~J4 가 0.5° 흔들림. 원인 미확인 (받침에 막힘 · 토크 안 걸림 · J2 EEPROM 의
    PWM Limit · Min/Max Position Limit 등 — 펌웨어 · dxl_setup 은 이 항목을 읽지 않음). 사용자: 움직임 못 느낌.
    재부팅 (토크 풀림 확인) 후 첫 세션으로 J2 재시험 (승인): 목표 +0.126 → 도달 +0.026 (**또 안 움직임**), 오류 없음, 전류 6 관절 모두 0.000 A.
    → 세션 문제 아님 (부팅 직후 첫 시험도 실패), **J2 쪽 문제** (토크 미적용 · PWM Limit · 기구 막힘 등). J1 만 정상 이동 확인됨.
    J2 −0.1 (받침에서 드는 쪽, 승인): 목표 −0.074 → 도달 +0.026 (**역시 안 움직임**) → 받침 막힘 아님. 사용자: 모든 관절 토크 걸림 (손으로 밀어 딱딱함).
    정지 전류 0 A (전류 읽기는 정상: 손이 닿으면 J2 0.07 A). 의심: J2 EEPROM (PWM Limit 등). 그리퍼가 무거워 받침 필수 (사용자).
  - 진단 준비 (승인, 미커밋): dxl_setup `x <id>` (읽기 전용 Control Table 덤프: Drive Mode · PWM / 전류 / 속도 / 위치 한계 · 게인 · Goal / Present PWM · 전류 · 온도),
    opencr_arm_tool.py 구간별 최대 전류 출력 + 목표 미도달 (`--max-final` 0.02 ★) 이면 ESTOP 중단. 가짜 펌웨어 (scratchpad) 로 정상 / 막힘 확인.
    dxl_setup 빌드 `~/opencr_builds/dxl_setup` (sha256 69d9f2d35a327110…). 다음: 전원 OFF/ON → dxl_setup 업로드 → 6 대 x → ARM 펌웨어 재업로드.
  - **원인 확인 (dxl_setup 업로드 승인, CRC 7DA198 · `x` 로 6 대 읽기만)**: J2 (ID 12) **Min/Max Position Limit = 2230 / 3033** (+0.28 ~ +1.51 rad).
    현재 J2 위치 2107 · 시험 목표 2000~2130 이 모두 범위 밖 → 모터가 Goal Position 쓰기를 거부한 것으로 판단 (e-Manual Data Limit, 관찰과 일치 · 한계 수정 후 재시험으로 확인 필요), SyncWrite 는 응답이 없어 펌웨어가 모름.
    J1 (ID 11) 도 698 / 3502 (±2.07 rad) 로 config ±2.8 보다 좁음. J3~J6 은 0 / 4095. 그 밖 6 대 공통: PWM Limit 885 · Current Limit 1193 ·
    Velocity Limit 200 · Drive Mode 0 · Shutdown 52 · P 800 · I / D 0 · 온도 31~32 °C · HW 오류 0 (모두 정상값). 예전 위치 한계를 정한 이유는 미확인.
  - 사용자 결정 (2026-10-26): **6 대 모두 모터 위치 한계 = config 범위** (firmware radToTick 과 같은 계산: J1 223~3873 · J2 875~3091 · J3 1005~3026 ·
    J4 875~3352 · J5 223~3873 · 그리퍼 1507~2876) · **ARM 펌웨어 부팅 점검에 위치 한계 검사 추가** (구현 완료, 미커밋):
    `checkArmLimits` — Min/Max Position Limit 이 config 범위 목표를 덮지 않으면 MOTOR_CONFIG → FAULT (토크 거부). SIL 대역이 한계 밖 Goal Position 을 버림 ·
    시험 ArmPositionLimitNarrowerThanConfigFaults 추가 → colcon test **77 건 실패 0**. dxl_setup `m <id> <min> <max>` (EEPROM, 토크 OFF 확인 · 다시 읽어 확인).
    빌드: ARM `~/opencr_builds/arm_v5` sha256 ab02c1cc0511ed21… · dxl_setup sha256 e6f49a93a6c69460… · 역할 1 컴파일 성공 (업로드 안 함).
  - **위치 한계 쓰기 완료 (승인)**: dxl_setup 재업로드 (CRC 7F7390) → `m` 6 대 모두 OK → `x` 로 다시 읽어 6 대 한계 = 위 값 확인 (토크 0 · HW 오류 0).
    ARM 펌웨어 (위치 한계 점검 포함) 업로드 (승인, CRC C99F9C) → monitor: err 0x02 만 (부팅 점검 통과, MOTOR_CONFIG · FAULT 없음).
  - **J2 재시험 −0.1 (승인) 성공**: 목표 −0.005 → 도달 −0.003 · 제자리 +0.095 → +0.094, 최대 전류 J2 0.035 A, 오류 없음 → 원인 (위치 한계) 확인.
    단 사용자가 팔을 들고 시험 → 명령 추종은 확인, **팔 무게를 J2 가 버티는지 (부하) 는 미확인**.
  - **J3 −0.1 (승인, 받침 + 손은 아래에만)**: 목표 −0.143 → 도달 −0.137 (중력으로 6 mrad 정상상태 오차, P 게인만) · 제자리 −0.043 → −0.040.
    최대 전류 J3 0.110 A · J2 0.059 A (J2 가 하중을 받아 6 mrad 밀림 후 복귀). 오류 없음. 그리퍼를 드는 부하가 정격에 비해 작음 (W350 정지 전류 수 A 대비).
    사용자가 J3 시험 도중 받침을 치움 → 이후 팔이 스스로 무게를 버팀 (HOLD): J2 −0.33 A · J3 −0.30 A · J4 −0.07 A,
    목표 대비 처짐 J2 +0.026 rad (1.5°) · J3 +0.021 rad (1.2°) — P 게인만 (I = 0) 이라 중력만큼 처짐. 진동 없음. 받침 없으면 전원 차단 시 팔이 떨어짐.
  - **J4 −0.1 (승인, 받침 없음) — 도구가 중단 (목표 미도달 관절 [2])**: J4 −0.025 → 목표 −0.125, 도달 −0.107 (손목 처짐 0.018).
    J2 는 움직이지 않는데 +0.124 → +0.156 (0.032 > 0.02) · J3 −0.018 → +0.002. 최대 전류 J2 0.374 · J3 0.247 · J4 0.226 A. ESTOP 후 HOLD 에서 J2 +0.183 · J3 +0.024 · J4 −0.101.
    **발견: 중력 처짐 래칫** — P 게인만 (I = 0) 이라 J2 는 중력으로 목표보다 약 0.026~0.03 rad 처진 채 버팀. 목표를 '현재 위치' 로 다시 잡을 때마다
    (도구 시작 q0 = 현재 위치 · 펌웨어 HOLD 진입 · 토크 ON) 그 처짐만큼 팔이 더 내려감 (J2: 0.098 → 0.124 → 0.156 → 0.183). 해결 방법 미결 (I 게인 · P 게인 · HOLD 정책).
  - **영점 복귀 (승인, --order 2,3,4,1,5 --max-final 0.05) — J2 단계에서 중단**: J2 +0.184 → 목표 0, 도달 +0.069 (0.069 > 0.05), 최대 전류 J2 0.890 A.
    들어 올릴 때는 중력 + 기어 마찰을 P 게인만으로 이겨야 해 오차가 큼 (버틸 때 0.026 rad / 0.33 A, 들 때 0.069 rad / 0.89 A — 둘 다 약 13 A/rad).
    ESTOP 후 HOLD: J2 +0.094 (5.4°) · J3 +0.072 · J4 −0.091, 안정. → 위치 정확도는 게인 (I 게인 등) 결정 전까지 수 ° 오차. 결정 필요.
  - **J5 +0.1 (승인, --max-final 0.05)**: +0.103 → 도달 +0.101 · 제자리 +0.003 → +0.005, 오류 없음. 시작 때 래칫으로 J2 +0.104 → +0.133 · J3 +0.077 → +0.100.
    → **J1~J5 모두 명령 추종 확인** (J2 는 위치 한계 수정 뒤). 그리퍼 (6 번째 칸) 는 이동 시험 안 함.
- 주행 자세 (사용자 요청 2026-10-26, 검토 중 · 미구현): 시뮬레이션 `~/GIT/FreeCAD/TeamRobot/Sim3D/sim3d.html` (저장소 밖, assets/teamrobot.urdf)
  `poseFor('drive')` = IK (base_link 0.24, 0, 0.16, 아래 30°) → [J1 0 · J2 −1.4356 · J3 +1.1497 · J4 +0.8094 · 롤 0] rad (−82.3° · +65.9° · +46.4°),
  tick 2048 · 1112 · 2798 · 2576 (config · 모터 한계 안). 시뮬 URDF 축 · 치수 = 저장소 URDF (J2~J4 +y), 시뮬 팔 = 4 관절 + 손목 롤 + 평행 그리퍼 (실물과 같음).
  확인 필요: J2 를 뒤로 82° 젖힐 수 있는지 (예전 J2 한계 2230~3033 은 뒤로 못 가게 돼 있었음), 시뮬에서 주행 자세가 차체 충돌 상자와 겹침 (BASE_SKIP).
  **전원 켜기 전 기본 자세 (사용자가 항상 세팅, 2026-10-26 측정, 토크 OFF)**: J1 2028 (−1.8°) · J2 **690 (−2.083 rad, −119°)** · J3 **3056 (+1.546, +89°)** ·
  J4 **832 (−1.865, −107°)** · J5 2022 · 그리퍼 **43 (−3.08)**. → J2 · J3 · J4 · 그리퍼가 config · 모터 한계 · URDF 범위 **밖** (도구는 시작 거부,
  호스트는 URDF 로 자른 명령 → REARM 때 한계까지 튐 = R3). 그리퍼 43 은 앞서 잰 개폐 범위 (1440~2945) 와도 안 맞음 — 원인 확인 필요.
  그리퍼 다시 측정 (토크 OFF): 닫힘 609 · 완전 열림 **−818** (누적 tick, 이동 1427 ≈ 125°, **이번에는 열림 = tick 감소**) → 다시 0/4095 경계를 지남.
  재조립 직후 값 (닫힘 1440 · 열림 2945, 열림 = 증가) 과 방향 · 위치가 모두 달라짐 → 결합이 바뀐 것으로 보임. 다시 맞춰야 함 (열림 기준 목표 ≈ 1230).
  **시작 자세 확정 (사용자, 2026-10-26)**: 팔을 접어 **왼쪽 90°** — J1 3074 (+1.574 rad, +90.2°) · J2 690 (−2.083) · J3 3056 (+1.546) · J4 832 (−1.865) · J5 ≈ 2059 (0).
  A안 (사용자 선택): 시작 자세를 범위에 넣음 — 제안 J2 최소 −2.14 · J3 최대 +1.60 · J4 최소 −1.92 rad ★ (기본 자세 + 약 0.055 rad). 그리퍼 다시 결합 · 범위 측정 후 승인 예정.
- 다음 (장치 쓰기마다 승인): ① 팔 게인 결정 · 구현 (중력 처짐 · 래칫 — I 게인 / P 게인 / HOLD 정책) → SIL → 업로드 → J2 · J3 재시험 · 영점 복귀.
  ② 정밀 영점 (J3 · J4 대충 측정) · ③ 그리퍼 (6 번째 칸 = 개폐) 처리 방식 팀 확인 · 그리퍼 개폐 시험 · ④ Jetson 세팅 (dialout · udev · 업로드 도구 · 단일 보드 시험).
  그 밖: v5 팀 승인 요청, D3 · D4 · D6 결정, opencr_dual_board_v4.md · README 갱신, 이번 변경 커밋 (사용자 지시 시).
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
| test_firmware_sil | 실제 opencr_firmware.ino (바퀴 · 팔) + 테스트 더블 (test/sil: Dynamixel 버스 · USB · 가상 시계) | 아니오 |
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

펌웨어 실제 타깃 컴파일 (업로드 아님, 출력은 저장소 밖). 역할은 빌드 옵션으로 넘긴다 (3 을 넘기면 config.h `#error`):

```bash
CLI=~/.local/opt/arduino-cli-1.5.1/arduino-cli
for r in 1 2; do
  $CLI compile --fqbn OpenCR:OpenCR:OpenCR --build-path /tmp/opencr_fw_r$r \
    --build-property "compiler.cpp.extra_flags=-DOPENCR_BOARD_ROLE=$r" workspace/ros2_ws/src/opencr_firmware
done
```

## 머신별 환경 메모

환경은 머신마다 다르므로 새 머신에서는 다시 확인한다 (`uname -m`, `ls /dev/ttyACM*`, `command -v arduino-cli`, `ls /opt/ros/lyrical/share | grep hardware_interface`).

- 회사 노트북 (2026-10-22): x86_64 Ubuntu 24.04.5, ROS 2 lyrical. OpenCR 미연결, Arduino 도구 없음, ros2_control · xacro 미설치 → `colcon build` 의 opencr_hardware 실패.
- 같은 노트북 (2026-10-26): 사용자가 `ros-lyrical-hardware-interface` 6.9.0 설치 → opencr_hardware 빌드 · 시험 가능. xacro · 컨트롤러 패키지는 여전히 없음.
  사용자 영역 설치 (승인): arduino-cli 1.5.1 (`~/.local/opt/arduino-cli-1.5.1`), OpenCR 보드 패키지 1.5.3 (`~/.arduino15`, 툴체인 gcc 5.4 는 32비트지만 그대로 실행됨),
  Dynamixel2Arduino 0.8.2 (`~/Arduino/libraries`). sudo 는 암호 필요 — 시스템 설치는 사용자가 직접 한다.
