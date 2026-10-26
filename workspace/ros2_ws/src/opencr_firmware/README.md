# opencr_firmware — OpenCR 보드 펌웨어 (Firmware 팀)

로봇에는 OpenCR 이 **2대** 있고, 같은 소스(`opencr_firmware.ino`)를 **역할별로 따로 빌드**해서 올린다.
두 보드의 펌웨어는 서로 바꿔 올리면 안 된다 — 빌드할 때 역할을 꼭 지정해야 하고(기본값 없음), 지정하지 않으면 빌드가 실패한다.

## 한눈에 — 주행 보드 vs 팔 보드

| | **주행 보드 (역할 1)** | **팔 보드 (역할 2)** |
|---|---|---|
| 빌드 | `tools/build_firmware.sh wheel` | `tools/build_firmware.sh arm` |
| 결과 파일 | `~/opencr_builds/opencr_wheel_v5.bin` | `~/opencr_builds/opencr_arm_v5.bin` |
| 빌드 옵션 | `OPENCR_BOARD_ROLE=1` | `OPENCR_BOARD_ROLE=2` |
| 모터 | 바퀴 XM430-W210 **4개, ID 1~4** | 팔 XM430-W350 6개, ID 11~16 |
| 모드 | Velocity (바퀴 각속도 rad/s) | Position (관절각 rad) |
| 그 밖 | IMU · 배터리 전압 | — |
| 패킷 칸 | 바퀴 칸 0~3 (칸 4 · 5 는 비움) | 팔 칸 6~11 |
| 설정 (`config.h`) | "주행 보드 전용" 구역 | "팔 보드 전용" 구역 |
| 명령 무수신 200 ms | 바퀴 0 | 그 자리에서 HOLD (토크 유지) |
| 호스트에서 명령하는 노드 | `drive_node` (`/cmd_vel` → diff_drive_controller) | `manipulator_node` (arm_controller) |

두 보드는 USB VID/PID · 시리얼 번호가 같아서 포트 이름으로는 구별되지 않는다.
**보드가 보내는 STATE 의 `role` 값(1 = 주행, 2 = 팔)으로 구별한다** — 호스트 플러그인(`opencr_hardware`)도 이 값으로 보드를 찾는다.

지금 보드에 무엇이 올라가 있는지 확인 (포트에 쓰지 않음):

```bash
python3 ../opencr_hardware/scripts/opencr_arm_tool.py monitor --port /dev/ttyACM0 --seconds 2
# 출력의 role=1 → 주행 펌웨어, role=2 → 팔 펌웨어. (주행 보드면 끝에 "팔 보드 v5 가 아님" 으로 끝나는 것이 정상)
```

## 빌드 · 업로드

필요: arduino-cli, OpenCR 보드 패키지 (1.5.3 에서 확인), Dynamixel2Arduino (0.8.2 에서 확인).

```bash
cd workspace/ros2_ws/src/opencr_firmware
tools/build_firmware.sh wheel                          # 주행 보드 펌웨어만 빌드
tools/build_firmware.sh arm                            # 팔 보드 펌웨어만 빌드
tools/build_firmware.sh wheel --upload /dev/ttyACM0    # 빌드 후 업로드 — 보드 역할을 yes 로 한 번 더 확인
```

- Arduino IDE 의 업로드 버튼은 쓰지 않는다 (역할 옵션을 넣을 수 없어 빌드가 실패함).
- 업로드가 `opencr_ld` 에서 멈추면 보드를 **Recovery Mode** 로 넣고 다시 실행: PUSH SW2 를 누른 채 Reset 을 눌렀다 떼고, SW2 를 뗀다 (STATUS LED 가 빠르게 깜빡임).
- PC 에서 포트 권한: `dialout` 그룹 + `../opencr_hardware/udev/99-opencr.rules` (ModemManager 가 포트를 잡지 않게 함).

## 주행 보드 (역할 1)

- 바퀴 4개 (2026-10-26 6륜에서 가운데 2개를 뺌). 칸 순서와 모터 ID:

  | 칸 | 바퀴 | ID | 방향 `WHEEL_DIR` |
  |---|---|---|---|
  | 0 | 왼쪽 앞 | 1 | +1 ★ |
  | 1 | 왼쪽 뒤 | 2 | +1 ★ |
  | 2 | 오른쪽 앞 | 3 | −1 ★ |
  | 3 | 오른쪽 뒤 | 4 | −1 ★ |

  호스트 쪽 이름 · 순서: `../opencr_hardware/urdf/opencr.ros2_control.xacro` (`wheel_fl/rl/fr/rr_joint` = 칸 0~3) · `config/controllers_4wd.yaml`.
- **모터 ID 다시 매기기가 필요하다**: 6륜 때 ID 는 왼쪽 1 · 2 · 3, 오른쪽 4 · 5 · 6 이었다. 가운데(2 · 5)를 뺐으므로 남은 모터를
  3 → 2, 4 → 3, 6 → 4 로 바꾼다 (`tools/dxl_setup`, 모터를 **한 대씩** 연결해서. 같은 ID 가 두 대 있으면 둘 다 바뀜).
  바꾸는 순서 주의: 4 → 3 을 하기 전에 3 → 2 를 먼저 (3 이 비어 있어야 함). 6 → 4 는 4 → 3 다음.
- `WHEEL_DIR` (★): 왼쪽 +1 · 오른쪽 −1 은 모터가 서로 마주 보게 달렸다는 가정이다. **바퀴를 띄운 상태에서 실제 회전 방향을 확인**하고 고친다.
- `WHEEL_GEAR = 1.5` (증속 벨트 36T → 24T), `WHEEL_VEL_LIMIT_RAW = 330` (★, 모터 Goal Velocity 상한) — 펌웨어는 넘는 값을 잘라 쓴다.
- 부팅 때 바퀴 토크를 끄고 시작한다. 토크는 호스트가 `TORQUE_ON` 을 보낼 때 켜진다.
- **아직 실물에서 돌려 보지 않았다** (주행 보드 없이 SIL 시험만 함). 첫 실기는 바퀴를 띄우고 저속으로.

## 팔 보드 (역할 2)

- 관절 칸 · 모터 ID (2026-10-26 실물 확인, **벤치 임시 매핑 ★** — 팀 규격과 다름, `../../../docs/opencr_v5_change_request.md`).
  칸 = STATE 의 관절 칸 (CMD 의 `arm_pos[0..5]` 와 같은 순서):

  | 칸 | 관절 | ID | 범위 (rad) |
  |---|---|---|---|
  | 6 | joint1 (허리) | 11 | −2.8 ~ 2.8 |
  | 7 | joint2 (어깨) | 12 | −1.8 ~ 1.6 |
  | 8 | joint3 (팔꿈치) | 13 | −1.6 ~ 1.5 |
  | 9 | joint4 (손목 굽힘) | 16 | −1.8 ~ 2.0 |
  | 10 | wrist_roll_joint (손목 롤) | 14 | −2.8 ~ 2.8 |
  | 11 | gripper_motor_joint = **그리퍼 개폐 모터** (각도 rad, 손가락 m 과의 변환 미정) | 15 | −0.83 ~ 1.27 ★ |

- 모터 EEPROM 의 Min/Max Position Limit 도 위 범위와 같게 맞춰 두었다. **부팅 점검이 이를 확인**한다 —
  모터 한계가 펌웨어 범위보다 좁으면 범위 밖 목표를 모터가 조용히 버리기 때문에 (실제로 joint2 가 그랬음) FAULT 로 막는다.
- 위치 게인 (P/I/D, 모터 RAM) 은 `config.h` 의 `ARM_POS_*_GAIN` 을 부팅 때마다 쓰고 다시 읽어 확인한다 (다르면 FAULT).
  지금은 공장값 (P 800 · I 0 · D 0) — 중력 처짐 때문에 실물 튜닝 예정 ★. 6 번째 칸 (그리퍼) 은 I = 0 고정.
- 팔은 명령이 끊기거나 ESTOP 이 와도 **토크를 끄지 않고 그 자리에서 HOLD** 한다 (무거운 그리퍼가 떨어지지 않게).
  HOLD 는 호스트의 REARM (리셋 버튼) 으로만 풀린다. 팔 토크를 끄는 방법은 사실상 전원 차단뿐 — 팔을 받친 뒤 끈다.
- 실기 시험 기록 · 알려진 문제 (중력 처짐, 시작 자세가 범위 밖, 그리퍼 결합): `../../../docs/arm_bench_test_2026-10-26.md`.
- ROS 없이 팔 보드만 시험: `../opencr_hardware/scripts/opencr_arm_tool.py` (monitor = 읽기만, move = 0.1 rad/s 이하 저속, `--yes` 필요).

## 공통 동작 (프로토콜 v5)

프레임 `FF FD | TYPE | LEN | PAYLOAD | CRC16-LE`, CMD 55 B · STATE 192 B (OpenCR 2대용 `role` 바이트 포함). 자세한 것은 `opencr_protocol.h`.

| 상황 | 주행 보드 | 팔 보드 |
|---|---|---|
| 정상 CMD (역할 · 유한값 · 범위 · seq 통과) | 바퀴 속도 적용 | HOLD 가 아니면 관절 목표 적용 |
| 200 ms 무수신 · 잘못된 CMD 만 옴 | 바퀴 0 (`ERR_CMD_TIMEOUT`) | HOLD (`ERR_HOLD`) |
| `FLAG_ESTOP` | 바퀴 0 | 현재 위치에서 HOLD |
| 부팅 점검 실패 (ID · 모델 · Homing 0 · 위치 한계) | `ERR_MOTOR_CONFIG` + `ERR_FAULT`, 토크 거부 | 같음 |

- `FLAG_REARM` (0x08) 상승 에지 = 팔 HOLD 해제. `ERR_HOLD` 0x40 · `ERR_FAULT` 0x80 은 v5 에서 추가.
- 물리 E-stop 회로는 아직 없다 (`ESTOP_PIN` 은 배선 전이라 풀업으로 "해제" 로 읽힘). 소프트웨어 ESTOP 은 물리 비상정지를 대신하지 않는다.
- 팀 규격 `../../../docs/interfaces.md` 의 승인본 (dev) 은 v3 (OpenCR 1대) 이다. 이 브랜치의 USB 패킷 절은 v5 제안이고,
  바뀐 점은 `../../../docs/opencr_v5_change_request.md` (Controller 팀장 승인 전 — dev/main 병합 금지).

## 파일

| 파일 | 내용 |
|---|---|
| `opencr_firmware.ino` | 펌웨어 본체 (두 역할 공용, `OPENCR_BOARD_ROLE` 로 나뉨) |
| `config.h` | ID · 방향 · 한계 · 주소 · 단위 (공통 / 주행 전용 / 팔 전용 구역) |
| `opencr_protocol.h` | 패킷 정의 — `../opencr_hardware/include/opencr_hardware/opencr_protocol.hpp` 와 **바이트 단위로 같아야 함** (시험이 비교) |
| `safety_core.h` | 안전 판단 (CMD 검증 · 타임아웃 · ESTOP · HOLD/REARM · FAULT), Arduino 의존 없음 |
| `tools/build_firmware.sh` | 역할별 빌드 · 업로드 |
| `tools/dxl_setup/` | 모터 사전 설정 도구 (탐색 `s` · 정보 `r` · 진단 덤프 `x` · ID `i` · 통신 속도 `b` · 위치 한계 `m`). 토크 ON 명령 없음 |

## 시험

ROS 2 (ros2_control) 가 있는 PC:

```bash
cd workspace/ros2_ws
colcon build --packages-up-to opencr_hardware && colcon test --packages-select opencr_hardware && colcon test-result --verbose
```

`test_firmware_sil` · `test_host_firmware_sil` 은 **실제 이 펌웨어 소스**를 PC 에서 컴파일해 가짜 모터 · 가짜 USB 와 돌린다 (두 역할 모두).
실물 모터 동작을 보장하지는 않는다. ROS 가 없는 PC 에서 돌리는 방법은 저장소 루트 `CLAUDE.md` 의 "빌드 · 테스트".

## 아직 정하지 않은 것 (★)

- 바퀴: 모터 ID 다시 매기기 (실물), `WHEEL_DIR` 실측, `WHEEL_VEL_LIMIT_RAW` 와 diff_drive 한계 (D6).
- 팔: 6 번째 칸 (그리퍼 개폐) 의 규격 반영 · 그리퍼 결합 다시 맞추기, 시작 자세를 범위에 넣기 (A안), 위치 게인 (중력 처짐), 정밀 영점.
- 공통: 물리 E-stop 배선, v5 · 4륜의 팀 규격 승인, D3 · D4 (`../../../docs/firmware_safety_analysis.md`).
