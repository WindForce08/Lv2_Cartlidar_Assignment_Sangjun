# 듀얼 OpenCR 펌웨어 안전 정책 — 작업 지시 · 1단계 분석 · 구현 계획

작성: 2026-10-22, Claude Code (1단계 읽기 전용 분석). 소스 코드는 아직 바꾸지 않았다.
코드 위치의 줄 번호는 2026-10-22 작업 트리 기준이며, 이후 바뀌면 함수 이름으로 찾는다.

---

## 0. 다음 세션에서 이어가는 방법

1. `CLAUDE.md` 의 절대 규칙을 지킨다 (업로드 · 토크 · 구동 금지, 설치 전 승인, v4 필드 임의 변경 금지).
2. 이 문서 §9 의 결정 사항 D1~D9 중 사용자가 답한 것을 확인한다. 답이 없으면 먼저 질문한다.
3. 답에 맞춰 §7 계획을 확정해 사용자에게 보여 주고, 승인 후 3단계 (구현) → 4단계 (테스트) → 5단계 (결과 보고) 로 진행한다.
4. 진행할 때마다 `CLAUDE.md` 의 "현재 진행 상황" 과 이 문서의 §9 상태를 갱신한다.

---

## 1. 작업 지시 요약 (사용자 원문을 압축, 2026-10-22)

### 1.1 진행 단계

1단계 읽기 전용 분석 → 2단계 구현 계획 (충돌 · 프로토콜 변경 필요 시 구현 전 보고) → 3단계 구현 (기존 기능 불필요한 제거 · 전체 재작성 금지) → 4단계 테스트 (통과 · 실패 · 미실행 구분) → 5단계 결과 보고.

5단계 보고 형식: ① 변경 파일 및 이유 ② 구현된 안전 정책 ③ 프로토콜 · 호스트 호환성 ④ 실행한 테스트와 결과 ⑤ 아직 검증되지 않은 위험 ⑥ 다음 단계 명령어 ⑦ 실제 OpenCR 업로드 전 확인 사항.

### 1.2 듀얼 OpenCR 구조

- 같은 펌웨어 소스를 역할별로 빌드. WHEEL `ROLE_WHEEL=1` (`OPENCR_BOARD_ROLE=1`, ID 1~6, Velocity, 100 Hz, 바퀴 슬롯 0~5 · IMU · 배터리). ARM `ROLE_ARM=2` (`OPENCR_BOARD_ROLE=2`, ID 11~16, Position, 100 Hz, 팔 슬롯 6~11).
- 호스트는 USB 장치 번호 · 물리 포트가 아니라 STATE 의 역할 값으로 보드를 구분.
- 역할별 빌드 설정이 실제 컴파일 결과에 반영되는지 검증. 두 보드가 서로의 모터를 제어하거나 다른 역할의 CMD 를 승인하지 않게.

### 1.3 프로토콜 v4

- 프레임 `0xFF 0xFD | TYPE | LEN | PAYLOAD | CRC16-LE`, CMD 0x01 55 B, STATE 0x81 192 B, LE, float32.
- 실제 헤더 · 호스트 파서를 읽고 필드 순서 · 오프셋 · 크기 · CRC 를 검증 (문서만 보고 추측 금지).
- v4 규격 임의 변경 금지. v3 프레임을 v4 로 승인하지 않기. 안전 정책에 v4 필드가 부족하면 패킷을 바꾸지 말고 이유 · 호환성 영향을 먼저 보고.
- CMD · STATE 역할 필드 존재 확인, 호스트 · 펌웨어 역할 상수 · 버전 대조.

### 1.4 확정된 안전 정책

- **CMD 타임아웃 200 ms**
  - ARM: 마지막 정상 승인 관절 목표 유지, 새 팔 목표 미승인, 위치 제어 루프 계속, 피드백 · STATE 계속, 일반 타임아웃만으로 토크 자동 차단 금지, 타임아웃을 상태 플래그로 보고 (기존 표현 먼저 확인).
  - WHEEL: 모든 바퀴 목표 속도 0, 상태 보고 계속, 팔 로직 실행 안 함.
  - 타이머는 정상 검증되고 역할이 맞는 명령으로만 갱신. CRC · 길이 · 역할 · 버전 불일치 · 비정상 데이터는 정상 명령이 아님.
- **ARM HOLD**: 타임아웃 시 진입. 마지막 승인 목표 보존 · 새 목표 미적용 · 위치 제어 계속 · STATE · 진단 계속. 통신이 복구되어도 자동으로 새 이동 금지. 명시적 재활성화 절차 전에는 새 목표 미승인. 복구 시 수신 버퍼의 오래된 CMD 를 실행하지 않도록 버퍼 · 시퀀스 처리 확인. 재활성화를 표현할 v4 필드가 실제로 있는지 먼저 검증, 없으면 최소 변경을 제안.
- **모터 · 버스 오류** (모터 응답 누락, 모델 불일치, 설정 오류, 모터 HW 오류, Dynamixel 버스 오류, 비정상 위치 · 센서, 펌웨어 내부 제어 오류) 는 일반 CMD 타임아웃과 구분.
  1. 오류 구분 · 진단 코드 기록 2. 오류 관절에 위험한 새 목표 미적용 3. 상태를 못 읽으면 실제 위치를 안다고 가정하지 않음 4. 타임아웃과 심각 HW 오류를 같은 경로로 합치지 않음 5. 모든 관절 토크 무조건 차단 금지 (팔이 중력 · 하중으로 처짐) 6. 고장 상태에서 기존 목표 유지가 안전하다고 무조건 가정하지 않음 7. 오류별 대응이 불명확하면 추측해서 코딩하지 말고 위험 · 가능한 대응을 보고 8. 오류 후 자동 정상 복귀 금지, 복구 조건 · 재활성화 절차를 명확히 정의.
- **물리 E-stop 없음**: 버튼 · 회로 추가 안 함. 소프트웨어 오류 플래그를 물리 비상정지와 같다고 설명하지 않음. 소프트웨어만으로 모든 고장에서 안전을 보장한다고 주장하지 않음. 기존 E-stop 명령 · 플래그의 실제 의미 확인, 존재하지 않는 물리 입력 가정 금지, 충돌하면 보고, 물리 회로가 없다는 사실을 문서에 명시.

### 1.5 상태 머신

- 상태: `STARTUP` (초기화 · 모터 설정 검사), `READY` (점검 완료, 명시적 활성화 대기), `ACTIVE`, `HOLD` (ARM 타임아웃), `FAULT` (심각 오류). E-stop 처리 상태를 따로 둘 수 있으나 물리 입력은 가정하지 않음.
- 전환: STARTUP 점검 실패 시 활성화 안 함 / READY 는 프로젝트의 명시적 활성화 절차를 통과해야 ACTIVE / ACTIVE 에서 ARM 타임아웃 → HOLD / ACTIVE 에서 WHEEL 타임아웃 → 바퀴 0 / HOLD 는 통신 복구로 자동 ACTIVE 안 됨 / 심각 오류 → FAULT + 원인 기록 / FAULT 해제는 원인 · 복구 조건 확인 후에만.
- 기존 활성화 · 토크 제어 · 타임아웃 · 오류 래치 기능을 먼저 확인하고 중복 구현하지 않음.

### 1.6 CMD 검증

헤더 · 길이, CRC, 프로토콜 버전, 보드 역할, 시퀀스 번호, 유한성, 관절별 허용 위치 범위, 단위, 제어 활성화 · 오류 상태. NaN · ±Inf · 범위 초과는 거부, 실패한 값은 모터에 전달하지 않음. 실패한 패킷이 타임아웃을 해제하는지 명시적으로 정의 · 테스트. 모터별 위치 제한 · 방향 반전은 실물 확인 전에 임의로 만들지 않음.

### 1.7 실시간 루프 (목표 100 Hz)

USB 수신이 루프를 무한 대기시키지 않게, 불완전 프레임 · 잘못된 CRC · 연속 쓰레기 처리, 버퍼 오버플로 없게, seq · 타임아웃 래핑 검토, 모터 통신 실패로 루프가 무기한 멈추지 않게. 측정 없이 100 Hz 보장 주장 금지. CMD 수신 주기 · STATE 송신 주기 · 루프 시간 · 누락 횟수를 측정할 진단 방법 제공.

### 1.8 호스트 일관성

`opencr_firmware.ino`, `config.h`, `opencr_protocol.h`, 호스트 `opencr_hardware.cpp` · `opencr_protocol.hpp`, URDF · ros2_control 설정, 테스트 · 루프백, README · 문서 검토. 호스트가 단일 OpenCR 만 지원하면 펌웨어만으로는 완성되지 않는다고 보고. 필요한 호스트 변경은 계약을 맞추되 대규모 구조 변경은 사전 보고.

### 1.9 테스트 계획 (하드웨어 없이)

- 정적: 필드 크기 · 오프셋, CRC 벡터, 호스트 · 펌웨어 상수 · 헤더 일치, Python/XML/C++ 문법 · 정적 검사, 기존 테스트 회귀.
- 단위 · 루프백: 올바른 v4 CMD 수신, 잘못된 CRC · 길이 · 역할 거부, v3 거부, NaN · Inf · 범위 초과 거부, 200 ms 타임아웃, ARM HOLD 목표 유지, HOLD 중 새 목표 차단, 복구 후 자동 이동 없음, WHEEL 타임아웃 시 전 바퀴 0, 잘못된 패킷으로 타임아웃 해제 안 됨, 호스트의 역할 중복 · 보드 누락 감지, STATE 역할 · 버전 · CRC · seq 검증.
- 한계: 토크 · 구동 · 회전 방향 · 하중 지지 · 물리 E-stop · Bus Watchdog 실제 효과는 검증 불가 — 검증했다고 주장하지 않음.

### 1.10 빌드 환경

빌드 도구 · 보드 패키지를 추측해서 고르지 말고, OpenCR 지원 방식과 프로젝트 문서를 확인해 제안. 설치 · 시스템 변경 전 승인. `OPENCR_BOARD_ROLE=1`, `=2` 두 구성을 각각 컴파일하고 각 역할 동작만 활성화되는지 확인.

---

## 2. 환경 · Git 상태 (2026-10-22, 회사 노트북)

| 항목 | 결과 |
|---|---|
| 저장소 | `~/GIT/Lv2_Cartlidar_Assignment`, 브랜치 `feature/firmware`, HEAD `0f00432` |
| 리모트 | `origin` = developlcy-oss/Lv2_Cartlidar_Assignment (팀), `backup` = WindForce08/Lv2_Cartlidar_Assignment_Sangjun |
| 머신 | x86_64 Ubuntu 24.04.5 노트북 (Jetson 아님) |
| OpenCR | `/dev/ttyACM*` 없음, `lsusb` 에 0483:5740 없음 → 이 머신에는 미연결 |
| 펌웨어 도구 | arduino-cli · Arduino IDE · PlatformIO · `~/.arduino15` · Dynamixel2Arduino · OpenCR 코어 모두 없음 → 펌웨어 컴파일 불가 |
| 호스트 도구 | ROS 2 lyrical, colcon, gtest, cppcheck, xmllint, check_urdf 있음. `hardware_interface` (ros2_control) · xacro 없음 → opencr_hardware colcon 빌드 실패 (rc=1) |

분석 시점의 미커밋 변경 (사용자의 이전 작업 — 단일 OpenCR v3 → 듀얼 OpenCR v4 전환): README, docs 2개, bringup launch, 펌웨어 3개, 호스트 헤더 · 소스 4개, 테스트 2개, base_bringup launch, udev, xacro 2개 (수정 17개) + 미추적 `workspace/docs/opencr_dual_board_v4.md`.

---

## 3. 관련 파일

| 구분 | 파일 |
|---|---|
| 펌웨어 | `opencr_firmware/opencr_firmware.ino`, `config.h`, `opencr_protocol.h` |
| 호스트 | `opencr_hardware/src/opencr_hardware.cpp`, `include/opencr_hardware/{opencr_hardware.hpp, opencr_protocol.hpp, serial_port.hpp, estop_latch.hpp}` |
| 테스트 | `opencr_hardware/test/{test_protocol.cpp, test_estop_latch.cpp, test_hardware_loopback.cpp}` (루프백은 pty 로 가짜 OpenCR 2대) |
| 설정 | `opencr_hardware/urdf/{opencr.ros2_control.xacro, robot.urdf.xacro}`, `config/controllers_6wd.yaml`, `control/config/manipulator.yaml`, `udev/99-opencr.rules` |
| 문서 | `README.md`, `workspace/docs/interfaces.md`, `workspace/docs/opencr_dual_board_v4.md` |
| /e_stop 소비자 | `control/src/manipulator_node.cpp`, `center/src/mission_node.cpp` |

---

## 4. 프로토콜 v4 구현 상태 (코드로 검증)

- CRC: CRC-16/BUYPASS, 범위 = 헤더 2바이트 ~ 페이로드 끝, LE 저장. Python 독립 계산으로 확인: Dynamixel ping 벡터 `0x4E19`, `"123456789"` → `0xFEE8`.
- CMD 55 B: role@0, wheel_vel[6]@1, arm_pos[6]@25, gripper_pos@49, flags@53, seq@54.
- STATE 192 B: role@0, pos[12]@1, vel[12]@49, effort[12]@97, quat[4]@145, gyro[3]@161, accel[3]@173, battery_v@185, error@189, fw_version@190, seq@191.
- `ROLE_WHEEL=1` · `ROLE_ARM=2` · `PROTOCOL_VERSION=4` 를 같은 헤더로 공유 (두 사본 바이트 동일, 테스트가 강제).
- CMD 에는 버전 필드가 없다 → 펌웨어는 길이로만 v3 를 거부 (v3 CMD 54 B ≠ 55). 호스트는 STATE 길이 (v3 191 B ≠ 192) 와 활성화 때 `fw_version == 4` 로 거부.
- CMD flags: bit0 TORQUE_ON · bit1 ESTOP · bit2 GRIPPER_VALID. bit3~7 미사용 (호스트는 0 송신).
- STATE error: 0x01 SYNCREAD · 0x02 CMD_TIMEOUT · 0x04 HW_ESTOP · 0x08 MOTOR_HW · 0x10 MOTOR_CONFIG · 0x20 ROLE_MISMATCH. **0x40 · 0x80 두 비트만 남음.**
- `FrameParser`: 오버플로 없음 (LEN ≤ 192 검사, 버퍼 198 B). 약점: 타입별 길이 미검사, CRC 실패 프레임 내부 바이트 재탐색 안 함 (쓰레기 `FF FD xx C0` 가 최대 196 B 를 삼킴), 길이 불일치는 `bad_frames` 미집계.
- seq: 펌웨어는 CMD seq, 호스트는 STATE seq 를 검사하지 않음. 호스트 `seq_` 가 두 역할 공용이라 각 보드에서는 2씩 증가.
- 기존 테스트: g++ 단독 빌드로 9개 통과 (Protocol 5, EstopLatch 4). 루프백은 ros2_control 미설치로 미실행.

---

## 5. 확정 안전 정책 vs 현재 코드

🔴 = 실물에서 위험.

| 정책 | 현재 구현 | 판정 |
|---|---|---|
| 역할별 빌드 | `config.h` 의 `OPENCR_BOARD_ROLE` (기본 1) + `#error`, ID · 모드 · SyncWrite 역할별 분기 | ✅ 코드상. ⚠️ 컴파일 미검증. 잘못 빌드하면 호스트가 역할 중복으로 활성화 거부 + 부팅 점검 (ID 미응답) 이 토크 거부 |
| 다른 역할 CMD 거부 | `receiveHost()` 폐기 + `ERR_ROLE_MISMATCH`, 타이머 미갱신 | ✅ (플래그는 다음 정상 CMD 에서 해제) |
| 자기 모터만 제어 | 토크 ON 을 `DXL_BROADCAST_ID` 로 | ⚠️ ID 별 지정 권장 |
| WHEEL 타임아웃 → 0 | `applySafety()` | ✅ (복구 시 자동 재개 — D3) |
| ARM 타임아웃 → 마지막 승인 목표 유지 · 토크 유지 | `g_safe_hold` 로 목표 동결 | ✅ |
| 🔴 HOLD 래치 · 자동 복귀 금지 | CMD 재수신 다음 주기에 `g_safe_hold = false` → 새 목표 즉시 적용 (`applySafety()` else 분기) | ❌ |
| 명시적 재활성화 | TORQUE_ON 0→1 엣지뿐 (토크 OFF 전제) | ❌ + 필드 부족 (§6) |
| 유효 CMD 만 타이머 갱신 | CRC · 길이 · 역할만 보고 값 검증 없이 갱신 | ❌ |
| 🔴 NaN · Inf · 범위 초과 거부 | 팔: `clampf` → 범위 초과 · ±Inf 는 관절 한계로 이동 명령, NaN 은 clamp 통과 후 `lroundf(NaN)` (결과 미정의 — x86 에서는 tick 2048, ARM 미확인). 바퀴: ±330 raw clamp, Inf · NaN 결과 플랫폼 의존. 현재 방어는 호스트 `isfinite` 필터뿐 | ❌ |
| 🔴 상태를 못 읽을 때 위치를 안다고 가정하지 않기 | SyncRead 누락 모터의 `g_present` 는 이전 값 (초기 0) 그대로 STATE 로 보고, 토크 ON 순간 `holdArmAtPresent()` 가 그 값을 목표로 사용 (0 tick = 중심에서 −π) | ❌ |
| 🔴 모터 HW 오류 | 50 ms 마다 1개씩 `readControlTableItem(HARDWARE_ERROR_STATUS)` 폴링. 마스크 버그: k==0 모터가 정상이면 전체 초기화 → 비트 깜빡임. 읽기 실패 = 정상 취급. 제어 계속 | ❌ |
| 비정상 센서값 · 내부 제어 오류 | 감지 없음 | ❌ |
| FAULT 래치 · 복구 절차 | `g_config_error` (재부팅으로만 해제) 외 없음 | ❌ |
| 🔴 전체 토크 무조건 차단 금지 | (a) 부팅 `setupMotor()` 가 모터마다 `torqueOff` → OpenCR 리셋 시 팔 낙하. (b) 호스트 `on_activate()` → `wait_first_state()` 가 `send_cmd(false)` (TORQUE_ON 없음) → 펌웨어가 BROADCAST 토크 OFF → deactivate→activate · 호스트 프로세스 재시작 시 HOLD 중인 팔 낙하 | ❌ |
| 🔴 모터 통신 실패로 루프가 멈추지 않기 | `syncRead` · `readControlTableItem` · `writeControlTableItem` 이 라이브러리 기본 타임아웃 사용 (라이브러리 미설치로 값 미확인). 미응답 모터가 있으면 10 ms 주기 초과 블로킹 가능. `receiveHost()` 의 `while(available)` 상한 없음 | ⚠️ 미검증 |
| 루프 시간 · 누락 · CMD 주기 진단 | 없음 (5주기 이상 밀리면 재동기만) | ❌ |
| 물리 E-stop 없음 | `config.h` `ESTOP_PIN_ENABLE 1` (핀 50 INPUT_PULLUP 읽음), `ERR_HW_ESTOP`, 호스트 래치 원인 "물리 E-stop 스위치", config 주석 "물리 E-stop 필수" | ❌ 설정 · 문서 불일치 |
| 소프트웨어 E-stop (FLAG_ESTOP) | 바퀴 0, 팔 현재 위치 정지 (위치 유효성 미확인), 해제 시 자동 재개 | ⚠️ 상태 머신에 통합 필요 |
| 호스트 듀얼 보드 | 후보 포트 2개, STATE.role 판별, WHEEL 1 · ARM 1 · 둘 다 v4 일 때만 활성화, 한쪽 끊기면 read() ERROR + 살아 있는 보드에 ESTOP | ✅ (단일 `/dev/opencr` 아님) |
| 호스트 HOLD · FAULT 대응 | 오류 비트는 로그 · 진단만, HOLD 중에도 JTC 가 새 목표 송신 | ❌ |
| 역할 중복 · 누락 감지 | `roles_ready()` · `read()` 에 로직 있음, 테스트 없음 | ⚠️ |
| STATE seq | 미검사 | ❌ |

기타:
- 바퀴 한계 충돌: 펌웨어 `WHEEL_VEL_LIMIT_RAW = 330` → 모터 7.91 rad/s × `WHEEL_GEAR` 1.5 = 바퀴 ≈ 11.9 rad/s. diff_drive 한계 (선속도 0.34 m/s + 각속도 2.0 rad/s × 0.317/2) 동시 사용 시 ≈ 19.9 rad/s. "범위 초과 거부" 로 바꾸면 회전하며 주행하는 명령이 거부된다 (D6).
- `wheel_count:=4` 설정 (`controllers_4wd.yaml`, xacro 4륜 슬롯) 은 6륜 고정 펌웨어의 슬롯 방향 (`WHEEL_DIR`) 과 맞지 않는다 — 범위 밖, 보고만.
  → 2026-10-26 해결: 주행 4륜으로 바뀌어 펌웨어도 `WHEEL_COUNT 4` (ID 1~4, xacro 4륜 칸 순서와 같음), 호스트 기본 `wheel_count` 4.
- e-Manual 기준 (실물 설정값 미확인): 모터 HW 오류 시 Shutdown 설정에 따라 해당 모터가 스스로 토크를 끈다. Bus Watchdog 만료 시 Position 모드 모터의 실제 동작은 실물로 확인해야 한다.

---

## 6. v4 필드 부족 — 변경이 필요한 이유

- 재활성화: TORQUE_ON 엣지는 먼저 토크를 꺼야 해서 팔이 떨어진다. ESTOP 1→0 엣지는 `joy_required=false` 이면 호스트 `EstopLatch::update()` 가 원인이 없을 때 리셋 버튼 없이 풀리므로 "명시적" 이 아니다.
- HOLD · FAULT 상태, 7가지 오류 분류, 관절별 원인, 루프 진단을 담기에 error 바이트의 남은 2비트가 부족하다.

| 선택지 | 내용 | 호환성 |
|---|---|---|
| A. 기존 필드만 | ESTOP 1→0 엣지 = 재활성화, 기존 오류 비트를 "상태 유지 중 래치" 의미로 변경 | 바이트 배치 불변. 기존 비트 의미 변경. 오류 분류 여전히 부족 |
| **B. 최소 추가 (권장)** | CMD flags `0x08 FLAG_REARM`, STATE error `0x40 HOLD` · `0x80 FAULT` | 길이 · 오프셋 불변. 구 호스트는 재활성화 불가 (안전 측 실패). 신 호스트 + 구 펌웨어는 자동 재개를 못 막으므로 **v5 상향 권장** |
| C. DIAG 프레임 추가 (B 와 함께 권장) | 새 타입 `0x82` (10 Hz 예상): 상태 · 원인 코드 · 관절별 상태 · 카운터 (CRC · 길이 · 역할 · 값 오류, seq 공백, 타임아웃, 루프 초과) · 루프 시간 / CMD 주기 최대값 | 현재 호스트 `poll_serial()` 은 STATE 외 타입을 무시 → 하위 호환. 없으면 §1.7 진단 요구를 만족할 채널이 없음 |

---

## 7. 구현 계획 (승인 후, §9 결정에 따라 조정)

### 7.1 변경 대상

- 펌웨어
  - `opencr_firmware/safety_core.h` 신설: Arduino 의존 없는 순수 C++ (상태 머신 · 값 검증 · 타임아웃 · 재활성화 · 연속성 검사 · 카운터). `estop_latch.hpp` 처럼 호스트 gtest 로 역할 1 · 2 각각 컴파일해 시험.
  - `opencr_firmware.ino`: safety_core 연동, 수신 바이트 상한, Dynamixel 호출에 명시적 짧은 타임아웃, 관절별 위치 유효성, HW 오류는 상태 패킷 alert 비트로 감지 (라이브러리 `XELInfoSyncRead_t.error` 필드 확인 필요), 부팅 시 토크가 이미 켜져 있으면 끄지 않는 방안 (D5), 토크 ON 을 ID 별로, 모델 불일치 모터에는 설정 쓰기 생략.
  - `config.h`: `ESTOP_PIN_ENABLE 0`, 임계값 (★) 정의.
- 프로토콜 헤더 2개 (항상 동일): D1 · D2 결정에 따른 변경만 + 파서 타입별 길이 검사 (와이어 포맷 불변).
- 호스트 (`opencr_hardware.cpp/.hpp`, 필요 시 `estop_latch.hpp`): `on_activate` 의 토크 OFF 프레임 제거, 펌웨어 HOLD · FAULT 를 E-stop 래치 트리거로 사용 (원인이 아니라 트리거 — 원인으로 두면 재활성화와 순환), 운용자 리셋 후 재활성화 송신 + 팔 명령을 현재 / 유지 목표로 재설정, seq 역할별 분리, STATE seq 진단, "물리 E-stop" 문구 정리.
- 테스트: `test_protocol.cpp` 보강 (골든 바이트 · v3 · 길이 · 타입), `test_firmware_safety.cpp` 신설 (CMakeLists 에 추가), `test_hardware_loopback.cpp` 의 FakeOpenCR 에 HOLD · 역할 중복 · 보드 누락 시나리오.
- 문서: `opencr_dual_board_v4.md`, `interfaces.md`, README — "별도 물리 E-stop 회로 없음, 소프트웨어 정지는 물리 비상정지가 아님", 상태 머신, 오류 표, 재활성화 절차.

### 7.2 상태 머신 (안)

```
STARTUP ─점검 실패→ FAULT(boot: 토크 거부, 재부팅으로만 해제 — 현재 동작 유지)
STARTUP ─통과→ READY (토크 OFF)
READY ─유효 CMD + TORQUE_ON + 전 관절 위치 유효→ ACTIVE (목표 = 현재 위치)
ACTIVE ─ARM 200 ms 유효 CMD 없음→ HOLD (마지막 승인 목표 유지 · 토크 유지 · STATE 지속 · 수신 CMD 폐기)
ACTIVE ─WHEEL 200 ms→ 바퀴 0 (복구 방식은 D3)
ACTIVE/HOLD ─심각 오류→ FAULT (원인 기록 · 새 목표 차단)
HOLD/FAULT ─원인 해소 + 재활성화 + 첫 목표가 유지 목표와 허용오차 이내→ ACTIVE
FLAG_ESTOP: 소프트웨어 정지 요청 (바퀴 0 · 팔 정지). 물리 비상정지 아님.
```

잘못된 패킷 규칙: CRC · 길이 · 역할 · 비유한값 · 범위 초과 중 하나라도 걸리면 값을 적용하지 않고 **타임아웃 타이머를 갱신하지 않는다**. 카운터만 증가. 테스트로 고정.

### 7.3 오류별 대응 초안 (★ = 값 결정 필요)

| 오류 | ARM | WHEEL |
|---|---|---|
| SyncRead K회 ★ 연속 누락 | FAULT, 해당 관절 위치 무효 처리, 마지막 목표 유지 | 즉시 0 → FAULT |
| 모터 HW 오류 | FAULT, 나머지 관절 목표 유지. 해당 모터는 Shutdown 설정에 따라 스스로 토크 OFF 가능. 복구에 모터 Reboot 필요 → 팔 처짐 위험, 운용 절차 결정 필요 | 0 + FAULT |
| 위치값 이상 (한계 + 여유 ★ 밖) | FAULT, 현재값이 아니라 마지막 목표 유지 | 0 |
| 루프 시간 초과 | 진단 카운트. FAULT 전환 여부는 결정 필요 | 같음 |

---

## 8. 테스트 계획과 실행 가능 여부 (2026-10-22 회사 노트북 기준)

| 시험 | 가능 여부 |
|---|---|
| 오프셋 골든 바이트, CRC 벡터, 헤더 동일성, v3 · 길이 · 역할 · NaN · Inf · 범위 거부, 200 ms 타임아웃, HOLD 유지 · 차단 · 자동 이동 없음, WHEEL 0, 잘못된 패킷이 타이머를 갱신하지 않음 (역할 1 · 2 각각 컴파일) | ✅ g++ + gtest |
| cppcheck, xmllint, launch `py_compile` | ✅ |
| ResourceManager 루프백 (역할 중복 · 누락, STATE seq, 호스트 재활성화) | ❌ ros2_control 설치 필요 |
| 역할별 펌웨어 실제 컴파일 | ❌ arduino-cli + OpenCR 코어 + Dynamixel2Arduino 설치 필요 |
| 토크 · 회전 방향 · 하중 · Bus Watchdog 효과 · 실측 100 Hz | ❌ 하드웨어 없이는 불가 — 검증했다고 주장하지 않음 |

---

## 9. 사용자 결정 사항 (상태: 모두 미결, 2026-10-22)

| ID | 질문 | Claude 권장 | 답 |
|---|---|---|---|
| D1 | 재활성화 · HOLD/FAULT 표현: 선택지 A 또는 B? B 라면 버전 v4 유지 / v5 상향? | B + v5 | 미결 |
| D2 | DIAG 프레임 (0x82) 추가? 없으면 루프 · CMD 주기 측정 채널 없음 | 추가 | 미결 |
| D3 | WHEEL 타임아웃 뒤 복구: 자동 재개 (현재) 또는 ARM 처럼 재활성화 필요? | — | 미결 |
| D4 | 오류별 대응 초안 (§7.3) 승인과 임계값: 연속 누락 K, 위치 여유, 재활성화 연속성 허용오차 (제안 0.05 rad = manipulator `verify_tolerance`), 모터 HW 오류 복구 절차 | — | 미결 |
| D5 | 토크 OFF 경로 수정: 부팅 시 토크가 켜져 있으면 끄지 않기, 호스트 `on_activate` 토크 OFF 제거 (호스트 변경 포함), 호스트의 명시적 토크 OFF 요청을 ARM 이 따를지 | 수정 | 미결 |
| D6 | 바퀴 범위 초과 거부 시 diff_drive 한계와 충돌 → `controllers_6wd.yaml` 한계 조정 또는 `WHEEL_VEL_LIMIT_RAW` (★) 재결정 | — | 미결 |
| D7 | 도구 설치 승인: 사용자 영역 (arduino-cli · OpenCR 보드 패키지 · Dynamixel2Arduino — OpenCR 툴체인의 32비트 라이브러리 요구 여부 확인 후), 시스템 apt (ros-lyrical ros2_control · 컨트롤러 · xacro — Lyrical 패키지 존재 먼저 확인) | — | 미결 |
| D8 | OpenCR 2대가 실제로 연결된 머신 (Jetson?) 확인 — 회사 노트북에는 미연결 | — | 미결 |
| D9 | 이전 v4 미커밋 작업을 구현 전에 커밋할지 | 커밋 | 미결 |
