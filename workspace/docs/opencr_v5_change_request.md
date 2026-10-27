# interfaces.md "USB 패킷" 절 변경 요청 — v3 → v5 (OpenCR 2대 · 팔 HOLD/REARM) (Firmware 팀, 2026-10-26)

받는 사람: Controller 팀장 (interfaces.md 승인). 상태: **dev 에 병합됨** (PR #6, 2026-10-26 `c1ad4a4`). 그 뒤 고친 것 (1안 정리 · 팔 실기 이식 · PR #10 병합, 2026-10-27) 은
`feature/firmware` 에 있고 다음 PR 로 올린다 — interfaces.md 변경 (그리퍼 ESTOP 예외 · 부팅 점검) 이 있어 다시 승인 필요.
제안 내용은 이 브랜치의 `interfaces.md` "USB 패킷" 절에 반영해 두었다 (PR diff 로 검토). 구현 · 시험: `opencr_firmware/`, `opencr_hardware/`
(colcon test 77 건 실패 0, 실제 펌웨어 소스를 PC 에서 돌리는 SIL 포함), 팔 보드 실기: `arm_bench_test_2026-10-26.md`.

## 승인본 (dev, v3) 과 제안 (v5) 비교

| 항목 | 승인본 v3 | 제안 v5 | 바뀌는 이유 |
|---|---|---|---|
| OpenCR | 1대 | **2대** — 주행 보드 (역할 1) · 팔 보드 (역할 2) | 하드웨어 구성 결정: 주행 모터 전용 OpenCR + 팔 전용 OpenCR |
| CMD 0x01 | 54 B: `wheel_vel[6]` · `arm_pos[6]` · `gripper_pos` · `flags` · `seq` | **55 B**: 맨 앞에 `role` 추가 | 받은 보드가 자기 몫인지 확인 (아니면 버리고 0x20) |
| STATE 0x81 | 191 B | **192 B**: 맨 앞에 `role` 추가 | 같은 USB ID 인 두 보드를 호스트가 구별 |
| CMD flags | bit0 TORQUE_ON · bit1 ESTOP · bit2 GRIPPER_VALID | + **bit3 REARM** | 팔 HOLD 해제 (아래) |
| STATE error | 0x01 ~ 0x10 | + **0x20 역할 불일치** · **0x40 HOLD** · **0x80 FAULT** | |
| `PROTOCOL_VERSION` | 3 | **5** | 버전이 다르면 호스트가 활성화를 거부 |
| 바퀴 | 4륜, ID 1~4, 슬롯 0~3 (4 · 5 는 0) | **같음** (주행 보드에서) | — |
| 팔 | ID 11~16 | ID 11~16 (팔 보드에서), 슬롯 6~11 | — |
| 포트 | (1대) | `ports` 에 후보 2개, STATE.role 로 연결 | |
| 부팅 점검 | ping · 모델 · Homing 0, Return Delay 0 | + 팔 모터 위치 한계가 관절 범위를 덮는지 · 위치 게인 · 그리퍼 PWM 한계 ≤ 400 | 실기에서 모터 한계 때문에 목표가 응답 없이 무시됨, 그리퍼 과부하 |
| 그리퍼 막힘 보호 | 없음 | 6 번째 칸이 막히면 펌웨어가 "막힌 자리 + 33 tick" 으로 약하게 쥠, ESTOP 에도 쥔 채 (필드 변화 없음) | 2026-10-27 실기: 퍽을 PWM 100 % 로 쥐다 MOTOR_HW |

프레임 (`FF FD | TYPE | LEN | PAYLOAD | CRC16-LE`), 필드 순서 (role 뒤는 v3 와 같음), 다른 팀이 쓰는 ROS 토픽 · Action 은 바뀌지 않는다.
버전이 다른 조합 (v3/v4 호스트 + v5 펌웨어, 또는 반대) 은 활성화되지 않는다 — 안전한 쪽 실패. 호스트와 두 펌웨어를 함께 바꾼다.

### 팔 HOLD / REARM 이 필요한 이유

v3 에서는 명령이 끊기거나 ESTOP 뒤 다시 움직여도 되는지 표현할 방법이 없고, 다시 켜려면 토크를 껐다 켜야 해서 팔 (무거운 그리퍼) 이 떨어진다.
v5 에서 팔은 토크를 유지한 채 HOLD 하고, 운용자 리셋 버튼 → 호스트가 REARM 펄스를 보낼 때만 다시 움직인다. 바퀴는 지금처럼 0 으로 멈춘다.

> 이력: 이 브랜치는 10/22 에 interfaces.md 를 승인 없이 v4 · 6륜으로 고쳤었다 (`a817e2e`). 2026-10-26 주행이 다시 4륜이 되어 바퀴는 승인본과 같아졌고,
> 브랜치의 interfaces.md 는 위 v5 제안으로 바꿨다.

## 결정 요청 — 팔 6 번째 칸 · 모터 ID (실물 확인 결과)

실물 팔은 **허리 · 어깨 · 팔꿈치 · 손목 굽힘 · 손목 롤 + 평행 그리퍼** 이다 (팀 시뮬레이션 `TeamRobot/Sim3D` 의 `teamrobot.urdf` 와 같음).
`control/config/manipulator.yaml` 은 `joint5` + `gripper_roll` (둘 다 x 축 롤) 로 되어 있어 실물과 다르다.
로봇 URDF 는 2026-10-27 1안 차체 (`cartridar_01`) 폐기로 지웠다 — 차체 2안 대기 (아래 "로봇 URDF — 차체 2안 대기").

| 슬롯 | 지금 이름 | 실물 | 모터 ID |
|---|---|---|---|
| 6 · 7 · 8 | joint1 · 2 · 3 | 같음 | 11 · 12 · 13 |
| 9 | joint4 | 손목 굽힘 | **16** |
| 10 | joint5 → **wrist_roll_joint** | 손목 롤 | **14** |
| 11 | gripper_roll → **gripper_motor_joint** (모터 각 rad, 가상 관절) | **그리퍼 개폐 모터** (회전 → 손가락 벌림) | **15** |

- 그리퍼 개폐를 (a) 6 번째 팔 칸 (관절각 rad) 으로 둘지, (b) `gripper_pos` 필드 + `GRIPPER_VALID` 로 옮기고 6 번째 칸을 비울지.
  지금 펌웨어는 벤치용으로 (a) 를 쓴다. 범위는 2026-10-27 다시 결합한 실측 (열림 1550 · 닫힘 2650 tick) 의 양 끝 20 tick 여유 = −0.7946 ~ 0.9541 rad.
- 손가락 관절 (`gripper_left/right_joint`, m) 과 그리퍼 모터 각 (rad) 의 변환 · `gripper_controller` 연결 방식.

## Controller 팀 확인 요청 — dev 의 새 `manipulator.yaml` 과 펌웨어 · ros2_control

(값이 모두 TODO 인 것은 알고 있음. 실물에 맞출 때 같이 보면 좋은 것)

| 항목 | dev `manipulator.yaml` | 펌웨어 · ros2_control (`opencr.ros2_control.xacro`) | 영향 |
|---|---|---|---|
| 관절 한계 | J1 ±3.14 · J2 ±1.57 · J3 ±2.5 · J4 ±2.0 · J5 ±3.14 | 펌웨어 (2026-10-27 실물) J1 ±π · J2 −2.14~1.6 · J3 −1.6~1.6 · J4 −1.92~2.0 · J5 ±2.8. 호스트 자르기 (`opencr.ros2_control.xacro`) = 펌웨어 범위 (로봇 URDF 가 오면 그 한계와도 겹치는 구간) | 펌웨어는 범위 밖 CMD 를 통째로 거부, 호스트는 겹치는 구간으로 자름 (`test_robot_description` 이 확인) |
| STOW | [0, 1.2, **−2.3**, 1.1, 0, 0] | J3 최소 −1.6 (펌웨어 · 호스트 자르기) | 호스트가 J3 를 −1.5 로 잘라 다른 자세가 됨 |
| 부호 · 영점 | `joint_signs` · `joint_offsets` TODO | 실물에서 URDF + 방향 · 영점 확인 끝 (실기 기록 2장) | IK 의 부호 · 0점이 URDF 와 같아야 함 |
| 그리퍼 | `enabled: true`, `gripper_joint` 0~0.02 m (ParallelGripperCommand) | `gripper_controller` (ParallelGripperCommand, 늘 켬) → `gripper_motor_joint` = 6 번째 팔 칸, 목표는 **모터 각 rad** (−0.7946~0.9541, 열림 1550 tick = −0.764 · 닫힘 2650 tick = 0.924) | `gripper.joint_name` 을 `gripper_motor_joint` 로, 열림 · 닫힘 값을 rad 로. 물체에 막히면 펌웨어가 약하게 쥠 (닫힘 값을 끝까지 줘도 됨) |
| 시작 자세 | — | 전원 켜기 전 접어 둔 자세 (J2 691 · J3 3057 · J4 835 tick) 를 펌웨어 범위에 넣음 (A안, 2026-10-27) | 그 자세에서 바로 토크 ON 가능. 차체 2안이 오면 다시 봄 |
| **TFmini 위치** | `tfmini_frame: tfmini_link` · `/arm_range` 로 팔 끝 거리 센서처럼 씀 (pick ① 정밀 정렬) | 실물 TFmini 는 **차체 하판 앞쪽 아래** (Perception `lidar.yaml` 은 앞쪽 비상 정지로 씀). `tfmini_link` 위치는 차체 2안 URDF 에서 (1안 폐기, ★ 실측) | pick 정밀 정렬 (TFmini 거리) 설계를 실물에 맞게 다시 봐야 함 |

## 로봇 URDF — 차체 2안 대기 (2026-10-27, Firmware 팀)

1안 차체 (CAD 자동 생성 `cartridar_01.urdf`, 2026-10-26) 는 폐기되어 저장소에서 지웠다 (`urdf/cartridar_01.urdf` · `urdf/robot.urdf.xacro` · `scripts/import_cad_urdf.py`).
남긴 것: `urdf/opencr.ros2_control.xacro` (형상 없는 하드웨어 블록 — 칸 · 명령 한계 · 포트), `config/controllers_4wd.yaml` (차체 치수는 비움).
차체 2안 URDF 가 오면 `urdf/robot.urdf.xacro` 에서 그것을 include 하고 ros2_control 블록을 얹는다.
그때까지 `base_bringup.launch.py` 는 이유를 알리고 끝난다 (OpenCR 없이 나머지만: `bringup.launch.py use_hardware:=false`).

2안 URDF 가 지켜야 할 이름 (다른 노드가 씀 — URDF 가 생기면 `test_robot_description` 이 확인):

- 좌표: `base_link` · `base_footprint` · `arm_base_link` · `imu_link` · `tfmini_link` · `lds03_front/rear_link` · `top_camera_front/rear_link` ·
  `gripper_camera_optical_frame` · `end_effector_link`. realsense 드라이버가 만드는 `*_color_optical_frame` 은 URDF 에 두지 않는다.
- 관절: 팔 `joint1~4` · `wrist_roll_joint` · `gripper_motor_joint` (그리퍼 모터 각, 빈 링크 가상 관절), 바퀴 4개 (지금 이름 `wheel_fl/rl/fr/rr_joint` ★).
- 차체 치수 (`diff_drive` 바퀴 간격 · 반지름 · 속도 한계) 는 2안 확정 뒤. 펌웨어 `WHEEL_GEAR` 는 PR #10 에서 1.0 (직결, 실물 확인), `WHEEL_DIR` 은 실측 ★.

### dev PR #10 (`manipulatorFirmware`, 2026-10-27) 과 병합

- 펌웨어 · 시험은 그대로 합쳤다: 바퀴 칸별 모터 ID `{1, 3, 2, 4}` (실물 장착), `WHEEL_GEAR = 1.0`, 부팅 때 바퀴 Drive Mode 반전 끄기 ·
  Velocity Limit ≥ 330 (`prepareWheelConfig`), 한계를 넘는 바퀴 명령은 모든 바퀴를 같은 비율로 줄임, SIL 시험 6 개, `scripts/opencr_wheel_tool.py`.
- PR #10 이 고친 `urdf/robot.urdf.xacro` (예전 자리값 URDF 에 urdfv2 `teamrobot.urdf` 값을 넣은 것) 는 이 브랜치에서 지웠다
  (2안 대기 유지, 2026-10-27 결정 — 팀장 · PR #10 작성자 확인 ★). ros2_control 블록과 관절 이름 · 인자 (`wheel_count` · `use_gripper` ·
  `joint5` · `gripper_roll` · `wheel_left_front_joint` 등) 가 맞지 않아 그대로는 펼쳐지지 않는다.
  그 값은 2안 URDF 를 만들 때 쓰도록 남긴다 (urdfv2 기준, `base_link` = 하판 윗면 중심, 바닥 위 56.25 mm):
  - 바퀴: x ±0.100 · |y| 0.1372 (간격 0.2744) · z −0.02325, 반지름 0.033, 축 +y (4 개 모두, + 회전 = 전진), 한계 3.0 N·m · 8.063 rad/s
  - `base_footprint` = `base_link` 아래 0.05625. `imu_link` xyz (0, 0, −0.0192) · rpy (π, 0, π/2) — OpenCR 을 하판 아래에 뒤집어 닮
- `controllers_4wd.yaml` 의 바퀴 간격 · 반지름은 비운 그대로 둔다 (dev 는 0.317 · 0.033 이라 urdfv2 의 0.2744 와도 달랐음).
