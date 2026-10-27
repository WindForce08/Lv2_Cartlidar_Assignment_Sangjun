# 슬라이더로 실물 팔 관절 구동 + RViz2 (arm_bench)

팔 OpenCR (역할 2, 펌웨어 v5) 한 대를 PC 에 USB 로 연결하고, 창의 슬라이더로 J1 ~ J5 · 그리퍼를 움직이면서 RViz2 에서 같은 자세를 본다.
**장치 쓰기**: [시작 (토크 ON)] 을 누르는 순간부터 토크 ON · 이동. 누르기 전에는 포트로 명령을 보내지 않는다.

## 1. 실행

```bash
cd ~/GIT/Lv2_Cartlidar_Assignment/workspace/tools/arm_bench
source /opt/ros/lyrical/setup.bash
ros2 launch slider/slider.launch.py                     # 실물 (팔 OpenCR = /dev/ttyACM0)
ros2 launch slider/slider.launch.py port:=/dev/ttyACM1  # 포트가 다를 때
ros2 launch slider/slider.launch.py fake:=true          # 보드 없이 연습 (가짜 OpenCR - 펌웨어 안전 규칙 · 그리퍼 막힘 보호 흉내)
ros2 launch slider/slider.launch.py rviz:=false         # RViz2 없이 슬라이더 창만
```

- 끝내기: 슬라이더 창을 닫거나 터미널에서 Ctrl+C (움직이는 중이면 ESTOP 후 송신 중단 → 펌웨어가 그 자리 HOLD, 토크 유지).
- 끝낸 뒤 남은 프로세스 확인: `ps -eo pid,cmd | grep -E "[r]obot_state_publisher|[r]viz2|[a]rm_slider|[f]ake_opencr"` → 남았으면 `kill <PID>`.
  (`pkill -f` 는 쓰지 말 것 - 같은 글자가 든 자기 셸까지 끔)
- 창 없이 손으로 움직여 RViz2 에서 보기만 (읽기 전용, 토크 안 켬): `ros2 launch monitor/view.launch.py`

### 실행 전 확인

- 팔 모터 12 V 전원 ON, OpenCR USB 연결. 사용자 계정이 `dialout` 그룹 (포트 권한).
- 같은 포트를 여는 다른 프로그램을 끔: 모니터 (`monitor/view.launch.py`), 벤치 도구 (`opencr_arm_tool.py`), `tools/grip_test.py`, `tools/gain_trial.sh`.
- 보드 상태를 읽기만 해서 확인 (선택): `python3 ../../ros2_ws/src/opencr_hardware/scripts/opencr_arm_tool.py monitor --port /dev/ttyACM0 --seconds 3`
  → `role=2 fw=5 err=0x02 CMD_TIMEOUT` 만이면 정상. `FAULT · MOTOR_CONFIG · SYNCREAD` 면 모터 전원이 꺼졌거나 부팅 점검 실패 → 전원 켜고 OpenCR 리셋 버튼.
- 팔 주변을 비우고, 팔이 움직여도 닿을 곳이 없는지 본다.

## 2. 창 사용법

| 화면 | 하는 일 |
|---|---|
| 상태 칸 (색) | 대기 (IDLE) → 토크 ON → REARM → 움직이는 중 (RUN) / 멈춤 (STOPPED) / 토크 끄는 중 (OFF) |
| **시작 (토크 ON)** | 토크 ON → 현재 자세에서 버팀 (HOLD) → REARM → 조작 가능. 슬라이더는 그 순간 실제 자세에서 시작 (팔이 튀지 않음) |
| J1 허리 ~ J5 손목 롤 [deg] | 관절 목표. 범위 = 펌웨어 `config.h` (J1 ±180°, J2 −122.6 ~ 91.7°, J3 ±91.7°, J4 −110 ~ 114.6°, J5 ±160.4°) |
| 그리퍼 [mm] | −11 (완전히 닫힘, 2650 tick) ~ +20 (끝까지 열림, 1550 tick) |
| 쥐는 힘 [tick] | 0 ~ 33. 물체에 막히면 "막힌 자리 + 이 값" 으로 약하게 쥠 (펌웨어 상한 33 = `GRIP_SQUEEZE_TICKS`) |
| 속도 [rad/s] | 관절 명령 변화 상한 (기본 0.2, 0.05 ~ 0.5). 그리퍼는 0.8 rad/s 따로 |
| **정지 (스페이스)** | ESTOP → 그 자리 HOLD (토크 유지, 쥐는 중인 그리퍼는 쥔 채). 다시 움직이려면 [시작] |
| 현재 자세로 | 슬라이더를 지금 실제 자세로 맞춤 |
| 팔을 받쳤음 + **토크 끄기** | 팔을 손으로 받친 뒤 체크해야 눌림 → 토크 OFF (받치지 않으면 팔이 떨어짐) |
| 아래 줄 | 관절마다 현재 · 명령 [deg, 그리퍼 mm] · 전류 [A] |

자동으로 멈추는 경우 (ESTOP → HOLD): 펌웨어 오류 비트, 관절 추종 오차 0.25 rad 초과 (막힘 · 힘 부족), 0.3 s 넘게 STATE 없음, 포트 끊김 (1 s 마다 다시 엶).
시작을 거부하는 경우: 팔 보드 (role 2) · v5 가 아님, 중단 오류 비트, 다른 프로그램이 명령 중 (펌웨어가 무수신이 아님), 현재 자세가 펌웨어 범위 밖.

## 3. 파일

| 파일 | 하는 일 |
|---|---|
| `slider/slider.launch.py` | robot_state_publisher (팔 URDF) + 슬라이더 창 + RViz2 를 한 번에. 인자 `port` · `fake` · `rviz`. ROS 통신은 이 PC 안에서만 (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`) |
| `slider/arm_slider.py` | Tk 슬라이더 창. `/joint_states` 발행 (팔 관절 + 손가락) → RViz2 의 팔이 따라옴 |
| `slider/arm_control.py` | 제어부 (100 Hz): TORQUE_ON → HOLD → REARM → 속도 한계로 목표 추종, 정지 · 오류 · 추종 오차 → ESTOP, 그리퍼 막힘 → 약하게 쥠 |
| `monitor/gripper_map.py` | 그리퍼 모터 tick ↔ 손가락 mm (닫힘 2650 → −11 mm, 열림 1550 → +20 mm), 펌웨어 그리퍼 보호 값 (`config.h` `GRIP_*`) 읽기 |
| `monitor/view.rviz` | RViz2 설정 (Fixed Frame `arm_base`) |
| `monitor/fake_opencr.py` | `fake:=true` 때 가상 포트 (/tmp/arm_bench_fake_opencr) 로 STATE 100 Hz, 펌웨어 안전 규칙 흉내 |
| `urdf/arm_bench.urdf` + `urdf/meshes/` | 팔만 떼어 낸 URDF (받침대 위). 관절 한계 = `config.h`. 폐기된 1안 CAD URDF 의 팔 부분에서 만듦 (생성 도구는 개인 폴더 `~/GIT/arm_bench/tools/make_arm_urdf.py`, 옮기지 않음) — 범위를 바꾸면 한계를 손으로 맞춤 ★ |
| `../../ros2_ws/src/opencr_hardware/scripts/opencr_arm_tool.py` | 프로토콜 v5 (CMD · STATE 프레임, CRC) · 범위 읽기 — 슬라이더 · 모니터가 같이 씀 |
| `../../ros2_ws/src/opencr_firmware/config.h` | 관절 범위 · 그리퍼 보호 값의 원천 (슬라이더 · 모니터가 직접 읽음 — 사본 없음) |

## 4. 문제가 생기면

| 증상 | 할 일 |
|---|---|
| RViz2 의 팔이 하얗거나 엉뚱한 자세 | 앞서 끈 robot_state_publisher 가 남음 → 위 `ps` 로 찾아 kill 후 다시 실행 |
| "포트를 열 수 없음" | 같은 포트를 연 프로그램을 끔, `dialout` 권한, USB 다시 꽂기 |
| "펌웨어가 무수신 상태가 되지 않음" | 다른 프로그램이 명령을 보내는 중 → 끄고 다시 |
| 시작 거부 "범위 밖" | 손으로 (토크 OFF 상태) 범위 안 자세로 옮긴 뒤 시작 |
| 오류 `MOTOR_HW` | 전원을 끄기 전에 어느 모터인지 읽기 (dxl_setup `r <id>`), 그리퍼면 막힌 채 오래 민 것 |
| 오류 `FAULT · MOTOR_CONFIG` | 부팅 점검 실패 (모터 전원 꺼짐 · EEPROM 범위 · 그리퍼 PWM 한계 > 400) → 원인 고치고 OpenCR 리셋 |
