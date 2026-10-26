# interfaces.md "USB 패킷" 절 변경 요청 — v3 → v5 (OpenCR 2대 · 팔 HOLD/REARM) (Firmware 팀, 2026-10-26)

받는 사람: Controller 팀장 (interfaces.md 승인). 상태: **승인 대기** — `feature/firmware` 브랜치에만 있고, 승인 전에는 dev/main 에 병합하지 않는다.
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
| 부팅 점검 | ping · 모델 · Homing 0, Return Delay 0 | + 팔 모터 위치 한계가 관절 범위를 덮는지 | 실기에서 모터 한계 때문에 목표가 응답 없이 무시됨 |

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
저장소 URDF 는 2026-10-26 CAD URDF (`cartridar_01.urdf`) 기준으로 바꿨다 (아래 "URDF 를 CAD 기준으로").

| 슬롯 | 지금 이름 | 실물 | 모터 ID |
|---|---|---|---|
| 6 · 7 · 8 | joint1 · 2 · 3 | 같음 | 11 · 12 · 13 |
| 9 | joint4 | 손목 굽힘 | **16** |
| 10 | joint5 → **wrist_roll_joint** | 손목 롤 | **14** |
| 11 | gripper_roll → **gripper_motor_joint** (모터 각 rad, 가상 관절) | **그리퍼 개폐 모터** (회전 → 손가락 벌림) | **15** |

- 그리퍼 개폐를 (a) 6 번째 팔 칸 (관절각 rad) 으로 둘지, (b) `gripper_pos` 필드 + `GRIPPER_VALID` 로 옮기고 6 번째 칸을 비울지.
  지금 펌웨어는 벤치용으로 (a) 를 쓰고, 범위를 그리퍼 끝에서 0.1 rad 안쪽으로 묶어 두었다.
- 손가락 관절 (`gripper_left/right_joint`, m) 과 그리퍼 모터 각 (rad) 의 변환 · `gripper_controller` 연결 방식.

## Controller 팀 확인 요청 — dev 의 새 `manipulator.yaml` 과 펌웨어 · URDF

(값이 모두 TODO 인 것은 알고 있음. 실물에 맞출 때 같이 보면 좋은 것)

| 항목 | dev `manipulator.yaml` | 펌웨어 · URDF (`opencr.ros2_control.xacro`) | 영향 |
|---|---|---|---|
| 관절 한계 | J1 ±3.14 · J2 ±1.57 · J3 ±2.5 · J4 ±2.0 · J5 ±3.14 | 펌웨어 J1 ±2.8 · J2 −1.8~1.6 · J3 −1.6~1.5 · J4 −1.8~2.0 · J5 ±2.8. URDF (cartridar_01) J1 ±π · J2 ±1.5 · J3 −1.5~1.4 · J4 −1.7~2.164 · 손목 롤 ±π. 호스트 자르기 = 둘이 겹치는 구간 (J1 ±2.8 · J4 −1.7~2.0 · 손목 ±2.8) | 펌웨어는 범위 밖 CMD 를 통째로 거부, 호스트는 겹치는 구간으로 자름 (`test_robot_description` 이 확인) |
| STOW | [0, 1.2, **−2.3**, 1.1, 0, 0] | J3 최소 −1.5 (URDF) | 호스트가 J3 를 −1.5 로 잘라 다른 자세가 됨 |
| 부호 · 영점 | `joint_signs` · `joint_offsets` TODO | 실물에서 URDF + 방향 · 영점 확인 끝 (실기 기록 2장) | IK 의 부호 · 0점이 URDF 와 같아야 함 |
| 그리퍼 | `enabled: true`, `gripper_joint` 0~0.02 m (ParallelGripperCommand) | `gripper_controller` (ParallelGripperCommand, 늘 켬) → `gripper_motor_joint` = 6 번째 팔 칸, 목표는 **모터 각 rad** (−0.83~1.27) | `gripper.joint_name` 을 `gripper_motor_joint` 로, 열림 · 닫힘 값을 rad 로. 범위는 그리퍼 결합을 다시 맞춘 뒤 다시 잼 ★ |
| 시작 자세 | — | 전원 켜기 전 접어 둔 자세 J2 −2.08 · J3 +1.55 · J4 −1.87 rad 가 범위 밖 | 범위를 넓히는 안 검토 중 (실기 기록 4-5) |
| **TFmini 위치** | `tfmini_frame: tfmini_link` · `/arm_range` 로 팔 끝 거리 센서처럼 씀 (pick ① 정밀 정렬) | 실물 TFmini 는 **차체 하판 앞쪽 아래** (Perception `lidar.yaml` 은 앞쪽 비상 정지로 씀). URDF `tfmini_link` 부모를 `base_link` 로 고침 (2026-10-26, 위치 ★ 실측) | pick 정밀 정렬 (TFmini 거리) 설계를 실물에 맞게 다시 봐야 함 |

## URDF 를 CAD 기준으로 (2026-10-26, Firmware 팀) — Controller · Perception 확인 요청

저장소 `opencr_hardware/urdf/robot.urdf.xacro` 는 손으로 쓴 자리값이라 실물 · CAD 와 달랐다 (TFmini 가 팔 끝, 라이다 뒤집힘 없음,
D435 위치 · 각도, IMU 방향, 바퀴 치수, 팔 구조). 이제 CAD 에서 자동 생성한 `cartridar_01.urdf` (FreeCAD TeamRobot/URDF,
`run_urdf.sh cartridar_01`, robot 이름 `cartridar_01`) 를 그대로 include 하고 ros2_control 만 얹는다. `test_robot_description` 이
ros2_control 파서로 이름 · 칸 · 컨트롤러 설정 · 명령 한계 (URDF ∩ 펌웨어 범위) 를 확인한다.

cartridar_01 (2026-10-26 사용자 1안) = 4륜 (가운데 바퀴 · 모터 뺌) · 바퀴 직결 (벨트 증속 없음) · **RealSense 브래킷 v2 (수평 아래 10°)** ·
팔 한계 = 보관함 계획 값 (J1 ±180°, J4 위 = 손목 실제 접촉 126° − 2° = 124°). 보관함 앞 줄 6칸은 J1 최대 2.98 · J4 최대 2.16 rad 이 필요해서,
실물에서 쓰려면 펌웨어 범위 · 모터 EEPROM 위치 한계를 넓혀야 한다 (장치 쓰기, 별도 승인 ★). 그 전까지 호스트는 펌웨어 범위로 자른다.
펌웨어 `WHEEL_GEAR = 1.5` (증속 벨트 가정) 는 직결과 다르다 — 고칠지 결정 필요 ★ (지금은 바퀴 속도 · 오도메트리가 1.5 배로 계산됨).

| 바뀐 이름 (이전 → 지금) | 쓰는 곳 |
|---|---|
| 바퀴 `wheel_left_front_joint` 등 → `wheel_fl/rl/fr/rr_joint` | `controllers_4wd.yaml` (바꿈) |
| 바퀴 간격 0.317 → **0.2744** (CAD y ±0.1372) | `diff_drive_controller` (바꿈) · drive 쪽 계산이 있으면 확인 |
| 팔 `joint5` · `gripper_roll` → `wrist_roll_joint` · `gripper_motor_joint` (가상), `arm_controller` 관절 = joint1~4 · wrist_roll_joint | **Controller `manipulator.yaml` · IK · 노드** |
| 팔 관절 한계 (URDF) = joint1 ±π · joint2 ±1.5 · joint3 −1.5~1.4 · joint4 −1.7~2.164 · 손목 롤 ±π. 호스트 자르기 = 펌웨어 범위와 겹치는 구간 | **Controller IK · 경로 계획**: 홈 자세 J1 > 137° 면 뒤 세로 프로파일, 롤 180° 근처 J4 > 112° 면 집게 카메라가 닿음 — 계획이 피해야 함 |
| `arm_base_link` = joint1 축 위 · link1 높이 (joint2 는 0.0595 m 위). 이전 자리값과 위치가 다름 | **Controller IK `base_height` 등** |
| 손끝 `tool_link` → `end_effector_link`, 팔 카메라 `arm_camera_link` → `gripper_camera_link` (+ `_optical_frame`) | Perception · Controller |
| D435 `camera_front/rear_link` → `top_camera_front/rear_link` (+ `_optical_frame`). CAD 이름 `d435_front/rear` 는 `import_cad_urdf.py` 가 가져올 때 바꿈. 원점 = 깊이 원점 (왼쪽 IR) = realsense `<camera_name>_link` | Perception: realsense 를 `camera_name` top_camera_front/rear 로 켜면 드라이버 TF 가 `top_camera_front_link` 아래에 `top_camera_front_color_optical_frame` 을 붙임 — 바꿀 것 없음 (실물 TF 확인 ★) |
| 라이다 `laser_front/rear` → `lds03_front/rear_link` (뒤집어 장착, roll π) | LDS-03 드라이버 frame_id 를 이 이름으로 |
| TFmini `tfmini_link` (차체 앞 아래), IMU `imu_link` (아래 OpenCR = 주행 보드), `base_footprint` 추가 | — |

- `use_gripper` 인자와 `gripper_joint` 는 지웠다. `gripper_controller` 는 늘 켜지고 `gripper_motor_joint` 를 명령한다 (ros2_controllers 소스상 켤 때 지금 위치를 명령 — 이 PC 에 패키지가 없어 실행 확인은 못 함).
  ⚠ 그리퍼 결합이 0/4095 경계를 지나는 동안은 호스트로 팔 보드를 켜지 않는다 (실기 기록 4-4).
- 겉모양 (메시) 은 저장소에 넣지 않았다 (RViz 에서는 충돌 · 좌표만 보임).
- **보관함 물리 시험 (Isaac Sim, 2026-10-26, 가정 ★: 퍽 30 g · 마찰 0.5 · 손가락 400 N/m)**: Sim3D 보관함 계획 자세로 퍽을 잡아 넣고 빼 봄
  (CAD 저장소 `TeamRobot/IsaacLab/cartridar_01/`). 처짐 없는 추종이면 원기둥 6/6 성공 · 차체 접촉 0 (단, 넣은 뒤 컵 위로 올라가 빠지고 위에서 내려가 꺼내는 경로로 바꿔야 함 —
  계획 그대로 컵 높이에서 J1 을 돌리면 손가락이 컵 속 퍽을 쓸고 지나감). 사각기둥은 놓을 때 기울어 끼는 경우가 있음.
  **지금 게인 (P 800 만) 으로는 처짐 (J2 약 2°) 때문에 넣기 1/10** → 팔 게인 개선이 먼저. Controller 의 집기 · 넣기 경로 설계에 참고.
- **D435 각도 35° → 10° (브래킷 v2, 2026-10-26)**: 깊이 원점이 앞 (−0.0441, 0.0175, 0.4618) m (차체 좌표, 이전보다 14.8 mm 높음), 광축 수평 아래 10°.
  Perception 의 바닥 · 거리 계산이 35° 를 가정했다면 다시 봐야 함 (URDF TF 를 쓰면 자동). 이전 v1 (35°) 차체 판은 CAD `archive_2026-10-22_v16`.
