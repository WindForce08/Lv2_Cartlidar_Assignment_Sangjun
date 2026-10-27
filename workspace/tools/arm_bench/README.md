# 팔 벤치 도구 (Firmware 팀, 2026-10-27 arm_bench 에서 옮김)

팔 OpenCR (역할 2, 펌웨어 v5) **한 대를 PC 에 USB 로 직접 연결**해 팔 (RM-X52 + 손목 롤 + 그리퍼) 만 보고 · 움직이고 · 시험하는 도구.
메인 bringup (`ros2 launch bringup bringup.launch.py`) 과는 **따로** 쓴다 — 같은 포트 · `/joint_states` 를 쓰므로 동시에 띄우지 않는다.

- 펌웨어 · 범위 · 그리퍼 보호 값은 저장소의 `../../ros2_ws/src/opencr_firmware/config.h` 를 직접 읽는다 (사본 없음).
- 프로토콜 · 범위 읽기는 `../../ros2_ws/src/opencr_hardware/scripts/opencr_arm_tool.py` (벤치 도구) 를 같이 쓴다.
- 팔 URDF (`urdf/arm_bench.urdf`) 는 이 도구들 전용이다. 관절 이름이 로봇 URDF 와 다르다 — `wrist_roll_joint` (= `joint5`) · `gripper_motor_joint` (= `gripper_roll` 칸, 실물 그리퍼 개폐).
- **장치 쓰기 (펌웨어 업로드 · 모터 EEPROM · 토크 ON · 이동) 는 단계마다 승인 뒤에만.** 물리 E-stop 회로는 없다 (창의 정지는 소프트웨어 ESTOP).
- 실기 기록: `../../docs/arm_bench_test_2026-10-26.md` · `../../docs/arm_bench_test_2026-10-27.md`.

```
arm_bench/
  monitor/   실물 팔 -> RViz2 (읽기 전용) · 가짜 OpenCR · 그리퍼 tick <-> 손가락 mm
  slider/    슬라이더로 직접 조작 + RViz2 (자세한 것: slider/README.md)
  tools/     gain_trial.sh · gain_report.py (게인 튜닝) · grip_test.py (그리퍼 막힘 보호 실물 시험) · live_tick.py (결합할 때 tick 표시)
  urdf/      팔만 있는 URDF + 겉모양 메시 23 개
  rl/        팔 집기 · 놓기 강화학습 (Isaac Lab 3.0, 코드만 — rl/README.md)
  logs/      시험 기록 (도구가 만듦, 저장소에 넣지 않음 - .gitignore)
```

## 준비

```bash
cd ~/GIT/Lv2_Cartlidar_Assignment/workspace/tools/arm_bench
source /opt/ros/lyrical/setup.bash
```

- 사용자 계정이 `dialout` 그룹 (포트 권한), `python3-serial` · `python3-tk` · `ros-lyrical-robot-state-publisher` · `ros-lyrical-rviz2`.
- **한 포트는 한 프로그램만** 연다: 모니터 · 슬라이더 · `opencr_arm_tool.py` · `grip_test.py` · `gain_trial.sh` 중 하나만.
- 보드 상태 읽기만: `python3 ../../ros2_ws/src/opencr_hardware/scripts/opencr_arm_tool.py monitor --port /dev/ttyACM0 --seconds 3`
  → `role=2 fw=5 err=0x02 CMD_TIMEOUT` 만이면 정상. `FAULT · MOTOR_CONFIG` 면 모터 전원이 꺼졌거나 부팅 점검 실패 (아래 장치 설정) → 고치고 OpenCR 리셋.

## 1. 보기만 (장치 쓰기 없음)

```bash
ros2 launch monitor/view.launch.py                    # 팔 OpenCR = /dev/ttyACM0
ros2 launch monitor/view.launch.py port:=/dev/ttyACM1
ros2 launch monitor/view.launch.py fake:=true         # 보드 없이 (가짜 OpenCR 가 관절을 천천히 흔듦)
```

1 초마다 `수신 Hz · role · fw · 오류 비트 · J1~J5 · 그리퍼 [deg]`. 토크가 꺼진 팔을 손으로 돌리면 RViz2 의 팔이 따라온다 (영점 · 방향 확인).

## 2. 슬라이더로 조작 (장치 쓰기: [시작] = 토크 ON)

```bash
ros2 launch slider/slider.launch.py                 # 실물
ros2 launch slider/slider.launch.py fake:=true      # 보드 없이 연습
```

[시작] → 토크 ON · 현재 자세에서 HOLD · REARM → 슬라이더 조작 (기본 0.2 rad/s). [정지] · 스페이스 · 오류 비트 · 추종 오차 0.25 rad · 끊김 → ESTOP (그 자리 HOLD, 쥔 그리퍼는 쥔 채).
그리퍼 [mm] −11 (닫힘 2650 tick) ~ +20 (열림 1550 tick). 사용법 · 문제 해결: `slider/README.md`.

## 3. 게인 튜닝 (장치 쓰기: 후보마다 업로드 · 토크 ON · 이동)

1. `config.h` 의 `ARM_POS_P_GAIN` · `ARM_POS_I_GAIN` 을 후보로 (그리퍼 칸 I = 0) → `../../ros2_ws/src/opencr_firmware/tools/build_firmware.sh arm --upload /dev/ttyACM0`.
2. 팔을 J2 · J3 를 ±0.1 rad 움직여도 닿는 곳이 없는 자세에 두고:

```bash
tools/gain_trial.sh P800_I0 /dev/ttyACM0 --dry-run    # 현재 위치 · 계획만
tools/gain_trial.sh P800_I0 /dev/ttyACM0              # J2 · J3 ±0.1 rad + J2 HOLD 다시 잡기 3 번 -> logs/gain/P800_I0/
python3 tools/gain_report.py logs/gain/P800_I0 logs/gain/P800_I100   # 후보끼리 비교
```

후보 순서 (★): P800/I0 (기준) → P800/I100 → P800/I300 → P1200/I300 (I 는 J2~J4 부터).
판정 (★ 제안): 1 s 안 오차 ≤ 0.01 · 넘침 ≤ 0.02 · 3 s 뒤 흔들림 ≤ 0.005 · HOLD 다시 잡기 3 번 J2 하강 ≤ 0.005 rad · 중단 오류 0. 5 분 HOLD 뒤 온도.

## 4. 그리퍼 막힘 보호 실물 시험 (장치 쓰기: 토크 ON · 그리퍼 이동)

`python3 tools/grip_test.py --help` — 팔 관절 5 개는 토크를 켠 자리에 고정, 그리퍼만 계단 명령 (빈 손 개폐 · 퍽 쥐기 · 쥔 채 ESTOP), 100 Hz 기록 → `logs/grip/`.
남은 확인 ★: 쥐는 힘 (`GRIP_SQUEEZE_TICKS` 33) 으로 퍽이 손으로 당겨도 안 빠지는지, 몇 분 쥘 때 모터 온도.

## 5. 그리퍼를 다시 결합할 때 (읽기만)

보드에 모터 설정 도구 (`build_firmware.sh setup --upload`) 를 올린 뒤 `python3 tools/live_tick.py --id 15 --target 1100` — tick 을 큰 글씨로.
열린 상태에서 결합해 열림 ≈ 1550 · 닫힘 ≈ 2650 이 되게. 값이 바뀌면 `config.h` 그리퍼 범위 · 모터 위치 한계 · URDF 한계를 같이 고침. 끝나면 팔 펌웨어를 다시 올림.

## 장치 설정 (2026-10-27, 팔 보드)

| 관절 (ID) | 펌웨어 범위 [rad] | 모터 EEPROM 위치 한계 [tick] | 비고 |
|---|---|---|---|
| J1 (11) | ±π | 0 ~ 4095 | 배선 꼬임 확인 필요 ★ |
| J2 (12) | −2.14 ~ 1.6 | 653 ~ 3091 | 대기 자세 포함 (A안) |
| J3 (13) | −1.6 ~ 1.6 | 1005 ~ 3091 | |
| J4 (16) | −1.92 ~ 2.0 | 796 ~ 3352 | |
| J5 (14) | ±2.8 | 223 ~ 3873 | |
| 그리퍼 (15) | −0.7946 ~ 0.9541 | 1530 ~ 2670, **PWM Limit 400** | 열림 1550 · 닫힘 2650, Homing Offset 0 |

- 펌웨어 부팅 점검이 위치 한계 · 게인 · 그리퍼 PWM Limit (≤ 400) 을 확인한다 — 다르면 FAULT (토크를 켜지 않음).
- 모터를 바꾸면 `tools/dxl_setup` (펌웨어 폴더) 으로 한 대씩: `i` ID · `b` 1 Mbps · `m` 위치 한계 · `w 15 400` (그리퍼) → `x` 로 확인 → 팔 펌웨어 다시 업로드.

## 옮기지 않은 것 (개인 폴더 `~/GIT/arm_bench` 에만)

- `tools/make_arm_urdf.py` — 폐기된 1안 CAD URDF 에서 팔 URDF 를 만드는 도구. 팔 URDF 는 이미 만들어져 있음.
- `tools/run_sil.sh` — 개인 폴더 펌웨어로 이 저장소의 시험을 돌리는 도구. 이 저장소에서는 `colcon test --packages-select opencr_hardware`.
- 시험 기록 (`logs/`), 빌드 결과 (`build/`), 강화학습의 학습 모델 · 결과 (`rl/logs` · `rl/results`, 개인 폴더에만).
