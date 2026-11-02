# waffle_4wd_description

팀 4륜 차체 + RM-X52 팔 로봇 URDF 와 Gazebo Sim 센서 · 구동 설정. **B안 상판 팔레트 v3 판** (2026-11-02).
ROS 2 Lyrical · Gazebo Sim 10.5 (ros-lyrical-ros-gz) 에서 시험했다.

- 팀 261028 확정본 + 증속벨트 v4.2 · 바닥 브라켓 v4 · 하단 전장 (OpenCR 2층 · Jetson · TFmini 받침대) + **B안 상판 팔레트 v3** (칸 12, 조인트1 중심 3열).
  261028 에서 바뀐 것 · 질량 · 점검 결과 · 확인할 것은 **`CHANGES_B.md`**.
- 팔레트 칸 12곳 자리 (base_link 기준) 는 `config/pallet_slots.yaml`. 팔레트는 `base_link` 에 합쳐져 있어 따로 링크가 없다.
- xacro 인자 `pucks:=true` 면 칸마다 퍽 하나 (`puck_{R,G,B}_{1..4}`, 겉모양 · 질량만) 를 넣는다. 펼친 URDF 는 `waffle_4wd.urdf` (빈 팔레트) · `waffle_4wd_loaded.urdf` (퍽 12개).
- 센서 사양 (실물 값 · 출처) 과 Gazebo 값은 `config/sensors.yaml` 에 있다.
- `urdf/` · `meshes/` · `config/gz_bridge.yaml` · `config/camera_info_fix.yaml` 은 저장소 밖 생성기 결과다 (맨 아래 "다시 만들기"). 직접 고치지 말 것.

## 빠른 시작

```bash
# 작업 공간 src 에 이 폴더를 넣고
cd ~/<작업공간> && colcon build --symlink-install --packages-select waffle_4wd_description
source install/setup.bash

ros2 launch waffle_4wd_description display.launch.py      # RViz + 관절 슬라이더 (Gazebo 없이), pucks:=true 면 퍽 12개
ros2 launch waffle_4wd_description gazebo.launch.py       # Gazebo 창 + 주사위 경기장 출발 자리
ros2 launch waffle_4wd_description gazebo.launch.py rviz:=true          # + RViz (스캔 · 앞 · 뒤 D435 · TFmini)
ros2 launch waffle_4wd_description gazebo.launch.py gui:=false          # 창 없이 (센서는 렌더함)
```

Gazebo launch 인자: `world` (기본 `worlds/dice_mission.sdf`), `gui`, `rviz`, `collision` (`light` 기본 / `full`), `pucks` (`false` 기본), `x` `y` `yaw` (기본 0.734, 0.950, π = 신호등을 봄).

### 키보드로 움직이기

Gazebo 를 띄운 뒤 **다른 터미널**에서 (키 입력을 받으므로 터미널에서 직접 실행):

```bash
source ~/<작업공간>/install/setup.bash
ros2 run waffle_4wd_description keyboard_teleop.py
```

| 키 | 동작 |
|---|---|
| `w` / `x` | 앞 · 뒤 속도 +/− 0.02 m/s (최대 0.36) |
| `a` / `d` | 왼쪽 · 오른쪽 회전 +/− 0.2 rad/s (최대 2.0) |
| `s` · 스페이스 | 멈춤 |
| `1`~`5` | 팔 관절 고르기 (joint1~5) |
| `i` / `k` | 고른 관절 +/− 0.1 rad (URDF 한계 안) |
| `o` / `l` | 그리퍼 열기 · 닫기 (손가락 +/− 5 mm, −11 ~ +20 mm) |
| `h` | 팔 전부 0 (영 자세) |
| `q` · Ctrl-C | 멈춤 명령을 보내고 끝 |

영 자세에서 팔이 앞으로 0.41 m 나와 있어 출발 자리에서 0.2 m 넘게 앞으로 가면 팔이 신호등 · 벽에 닿는다 (닿으면 관절이 처지고 손가락이 안 움직임).
팔을 들거나 (`2` 고르고 `k`, `3` 고르고 `i` 등) 뒤로 가서 시험할 것.

토픽으로 직접:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.1}}}"   # 앞으로 0.1 m/s
ros2 topic pub --once /arm/joint2/cmd_pos std_msgs/msg/Float64 "{data: 0.5}"                  # 팔 관절 (rad) · 손가락 (m)
```

필요한 패키지: `ros-lyrical-ros-gz` (gz-sim 10.5, ros_gz_bridge, ros_gz_sim), `xacro`, `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`.

## 261028 에서 바뀐 것 (요약, 자세히는 `CHANGES_B.md`)

| 항목 | 이 패키지 |
|---|---|
| 퍽 보관 | B안 상판 팔레트 v3 (190.3 g), 원점 = 조인트1 축 · 상판 윗면 base_link (0.096, 0, 0.111) m. 칸 바닥 z 0.114 / 0.124 / 0.136 m (1 · 2 · 3열) |
| 구동 | 증속벨트 v4.2 + 바닥 브라켓 v4 + TO 캐리어. **바퀴 관절 x 앞 0.0925 · 뒤 −0.0675** (y ±0.1546, 바퀴 간격 0.317 그대로) → 회전 중심이 base_link 보다 x +12.5 mm 앞 |
| 프로파일 기둥 · 가로대 · D435 2대 | 캐리어를 따라 x +12.5 mm |
| 하판 전장 | Jetson (−79.5, 0), OpenCR 2층 `opencr_lower_link` · `opencr_upper_link`, TFmini 받침대 (렌즈 `tfmini_link` (0.130, 0, 0.0243) m), 배터리는 질량만 |
| IMU | **하나**: `imu_link` = 아래층 OpenCR 칩 → `/imu/data` (팀의 두 IMU 평균 노드는 시뮬에서 쓰지 않음) |
| 질량 | **4.264 kg** (퍽 12개 4.444 kg) |

관절 이름 (바퀴 4, joint1~5, gripper_roll, 손가락 2) 과 팀 코드 frame (`base_link`, `base_footprint`, `imu_link`, `laser_front/rear`, `tfmini_link`, `arm_base_link`, `tool_link`, `arm_camera_link`,
`top_camera_{front,rear}_color_optical_frame`) 은 261028 과 같다. `base_link` 원점도 같다 (판 가운데, 지면 위 6.75 mm). robot_state_publisher 가 루트 관성 KDL 경고를 내지만 동작에는 영향 없음.

## 센서 (실물 사양 · Gazebo 값)

| 센서 | 실물 사양 (출처는 `config/sensors.yaml`) | Gazebo | 토픽 · frame |
|---|---|---|---|
| LDS-03 x 2 (ROBOTIS, = COIN-D4) | 0.05~12 m, 360° · 0.9° (400 점), 10 Hz, ±10 mm (~1 m) · ±20 mm, UART 115200 bps 8N1 (USB2LDS), 5 V 240~320 mA, 42 g | gpu_lidar 400 점 10 Hz, 잡음 σ 10 mm | `/scan_front` · `/scan_rear` (LaserScan), `laser_front` · `laser_rear` (뒤집힘 roll π), 스캔면 바닥 위 82.2 mm |
| TFmini Plus (Benewake) | 0.1~12 m (흰 판), ±5 cm (~6 m) · ±1 %, 시야각 3.6°, 100 Hz (1~1000), UART 115200 bps 8N1, 5 V ≤110 mA, 12 g | 광선 1 개 gpu_lidar 100 Hz, 잡음 σ 25 mm → Range 변환 노드 | `/tfmini/range` (Range), `tfmini_link` (렌즈 가운데, +x 앞), 바닥 위 31 mm |
| D435 x 2 (Intel RealSense) | 깊이 화각 87° × 58°, 컬러 69° × 42°, 기준선 50 mm, 깊이 최소 0.28 m (최대 해상도) ~ 10 m | 640 × 480 30 Hz: 컬러 (fx 616.8) + 컬러에 맞춘 깊이 + IR 좌우 (fx 386.6) | `/top_camera_{front,rear}/` + `color/image_raw` · `color/camera_info` · `aligned_depth_to_color/image_raw` · `aligned_depth_to_color/camera_info` · `depth/color/points` · `infra1/image_rect_raw` · `infra1/camera_info` · `infra2/…` |
| 팔 카메라 (Arducam UC-684 B0201) | 대각 화각 120° (판매 정보) | 1280 × 720 30 Hz, 가로 113° 바늘구멍 (가정) | `/arm_camera/image_raw` · `/arm_camera/camera_info`, `arm_camera_optical_frame` |
| IMU (OpenCR 1.0 아래층, MPU9250) | 자이로 · 가속도 · 지자기 3 축 | 100 Hz, 잡음 σ 0.0017 rad/s · 0.03 m/s² (가정) | `/imu/data`, `imu_link` (아래층 OpenCR 칩 (9.5, 0, 20.2) mm). 위층 OpenCR IMU 는 끔 |

구동 · 기타 토픽 (Gazebo):

| 토픽 | 타입 | 주기 | 비고 |
|---|---|---|---|
| `/cmd_vel` | geometry_msgs/TwistStamped | — | DiffDrive (바퀴 4, 좌우 간격 0.317 m = 바퀴 폭 가운데 사이, 반지름 0.033 m, 최대 0.363 m/s) |
| `/odom` · `/tf` (odom → base_link) | nav_msgs/Odometry | 50 Hz | 바퀴 기준 (미끄럼 모름). 팀 EKF 를 같이 띄우면 odom → base_link TF 가 겹치니 둘 중 하나만 |
| `/ground_truth/odom` | nav_msgs/Odometry | 50 Hz | 시뮬레이터가 아는 실제 위치 (world 기준), SLAM 채점용 |
| `/joint_states` | sensor_msgs/JointState | 50 Hz | 바퀴 4 + 팔 8 |
| `/arm/<관절>/cmd_pos` | std_msgs/Float64 | — | joint1~5, gripper_roll, gripper_left/right_joint (속도 명령 방식, 최대 1 rad/s · 0.05 m/s) |
| `/clock` | rosgraph_msgs/Clock | | 모든 노드 `use_sim_time: true` |

카메라 토픽은 **구독할 때만 Gazebo 가 렌더**한다 (lazy). 안 쓰는 카메라는 시뮬 속도를 깎지 않는다.

## 시험 결과 (2026-10-28 261028 판 기준, RTX 5060 Laptop 8 GB, 창 없이)

B안 판 점검 (2026-11-02, 직진 · 회전 · 센서 주기 · Isaac 3판) 은 `CHANGES_B.md`. 아래는 261028 판 값이고 센서 · 팔 구성은 같다.


- 주기 (sim 시간): LiDAR 10.0 · IMU 100 · TFmini 100 · odom · joint_states 50 Hz.
- LiDAR 출발 자리 정면 0.506~0.514 m (신호등 몸체까지 계산 0.515), 후면 0.242~0.251 m (칸막이 벽까지 0.256). TFmini 0.46 m.
- 전진 0.1 m/s × 3 s → 실제 0.300 m, odom 0.300 m.
- 제자리 회전 0.5 rad/s × 3 s (86°) → **실제 70°, odom 86°** (바퀴 4 개 미끄럼). 회전은 IMU 를 같이 써야 함 (팀 EKF).
- 팔 joint1 0.8 · joint2 0.5 rad 명령 → 2 s 뒤 그 값, 그동안 차체 이동 0.3 mm. 손가락 −11 ↔ +20 mm (URDF 한계값) 왕복 5 회 정상.
- 키보드 조작 (가상 터미널로 키 입력): `w` x5 → 0.10 m/s 전진, `a` x3 → 회전, `2` + `i` x5 → joint2 0.5 rad, `o` x4 → 손가락 20 mm, `q` 정상 종료.
- 뒤 D435: 컬러 · 깊이 (100 % 유효, 중앙값 0.73 m) · IR · camera_info 모두 나옴.
- 실시간 비율: 카메라 구독 없음 1.0 (collision:=light) / 0.76 (full), 카메라 9 개 모두 구독 약 0.5.

## 알아 둘 점 (미확정 포함)

- **D435 컬러 · 깊이 Gazebo 센서는 광축으로 8 mm 앞**: URDF optical frame 이 D435 몸체 메시 앞면보다 0.2 ~ 0.6 mm 안쪽이라 (실물 렌즈가 유리 뒤에 있는 것과 같음, frame 은 맞음)
  Gazebo 깊이 센서가 막혀서 (전부 −inf). `scripts/camera_info_fix.py` 가 이를 고쳐 내므로 **받는 쪽은 더할 것이 없다** (2026-10-29):
  - 깊이 영상 `<D435>/aligned_depth_to_color/image_raw`: Gazebo 원본 (`/gz/...`) + 0.008 m = optical frame 기준 (같은 장 비교 평균 +8.000 mm 확인).
  - 점구름 `<D435>/depth/color/points`: Gazebo 원본은 센서 축 (x 앞 · y 왼 · z 위) 인데 frame_id 는 optical frame 이라 RViz · TF 에서 90° 돌아가 보였음 (10-28 공유본 버그)
    -> optical 축 (x 오른 · y 아래 · z 앞) 으로 바꾸고 z + 0.008 m. TF 로 base_link 에 옮기면 바닥 점이 바닥 높이에 옴.
  - 주기 (창 없이, reliable 로 받을 때): 원본 30 Hz -> 고친 출력 약 25 Hz. 수 MB 메시지를 best-effort 로 받으면 조각 손실로 원본 · 고친 출력 모두 크게 떨어지니 reliable 로 받을 것.
  - 실물 데이터에는 해당 없음 (실물 드라이버는 frame 기준으로 낸다).
- **camera_info**: Gazebo 가 같은 링크의 카메라마다 엉뚱한 기준선을 P 에 넣어서 `scripts/camera_info_fix.py` 가 실물 드라이버처럼 고친다 (infra2 만 Tx = −fx × 0.050).
- D435 640 × 480 내부 값 (fx 615 · 383 대표값), Arducam 해상도 · 화각, IMU 칩 자리 · 축 방향, 잡음 값은 가정 → 실물 camera_info · 실측으로 바꿀 것.
- LDS-03 앞뒤 자리는 임시 (귀 구멍이 판 구멍과 안 맞아 테이프 고정 검토 중), 테이프 두께 0 mm 로 둠. LDS-03 0° 방향 = 모델 +x 가정.
- 스캔면 (82.2 mm) 이 70 mm 박스보다 높아 LiDAR 로 박스는 안 보임 (사용자 결정).
- `collision:=light` 는 브라켓 · 블록 · 받침 · 모터 · 덮개 등을 상자로, 팔레트를 바닥판 상자 + 칸마다 칸 방향 상자 (칸 속 · 손가락 홈 무시) 로 둔다. **퍽 집기 · 팔레트 시험은 `collision:=full`** (팔레트 CoACD 볼록 조각, 전체 478 조각).
- Gazebo 는 팔 · 손가락 관절 물리 한계를 1 mm · 0.001 rad 넓혀 띄운다 (xacro `limit_margin:=0.001`): 한계 끝에 닿으면 Gazebo (DART) 에서
  속도 명령으로 빠져나오지 못하고 걸려서 (2026-10-28 시험). URDF 한계값을 그대로 명령해도 된다. `.urdf` 의 한계값은 그대로.
- 질량: 옮기거나 뺀 부품 (J1 묶음, 옛 브라켓, Jetson · OpenCR · 배터리 자리 상자, 뜬 기판) 이 받은 base_link 질량에 들어 있었다고 봄.
- 팀 저장소 값과 다른 것 (팀 저장소는 고치지 않음): `robot.urdf.xacro` 의 `arm_base_link` (0.087, 0, 0.111) → (0.096, 0, 0.131), `manipulator.yaml` `ik.base_height` → 0.0595,
  `tfmini_link` 는 집게 쪽 → 차체 앞, `laser_front/rear` (±0.15, 0, 0.08) → (±0.1089, 0, 0.0755) roll π, `perception/config/tfmini.yaml` 주석 "URDF 에 없음" → 이제 있음.
- **팀과 아직 맞춰야 할 값 (실물 기준 미정, 지금은 받은 2안 CAD 값)**:
  - 바퀴 좌우 간격 (2026-10-29 확인, 팀과 같음): 좌우 바퀴 폭 가운데 사이 0.317 m (바퀴 y ±0.1585) = 팀 `controllers_4wd.yaml` 0.317 m.
    URDF 바퀴 관절 원점 (y ±0.1546, 간격 0.3092) 은 바퀴 폭 가운데보다 3.9 mm 안쪽이라 diff_drive 값으로 쓰지 않는다. 팀 `robot.urdf.xacro` 는 아직 1안 값 (0.2744).
  - 바퀴 구동: 실물은 벨트 1:1.5 증속 (팀원 확인 2026-10-29, 이 URDF 관절 한계 10.99 rad/s · 1.8 N·m 와 같음) ↔ 팀 펌웨어 `config.h` `WHEEL_GEAR 1.0` (직결) 은 고쳐야 함.
  - D435 각도: URDF 35.2° 아래 ↔ 팀 문서 "35° → 10° (브라켓 v2)". 10° 로 바꾸면 frame 과 차체 메시 안 카메라 모양을 같이 돌려야 함.
  - 팔 관절 한계: 이 URDF = 펌웨어 config.h (A안, 팀 `feature/arm-from-dev` 와 같음) ↔ `origin/dev` 의 ros2_control 은 예전 값 (joint1 ±2.8 등).

## 파일

| 경로 | 내용 |
|---|---|
| `urdf/waffle_4wd.urdf` · `urdf/waffle_4wd_loaded.urdf` | 펼친 URDF (Gazebo 태그 포함): 빈 팔레트 · 퍽 12개 |
| `urdf/waffle_4wd.urdf.xacro` | 같은 내용 + 인자 `prefix`, `collision` (full / light), `pucks` (true / false), `limit_margin` |
| `config/pallet_slots.yaml` | 팔레트 칸 12곳 (base_link 기준 x · y · 방향 · 칸 바닥 · 벽 윗면, 칸마다 퍽 이름) |
| `config/sensors.yaml` | 센서 사양 · 출처 · Gazebo 값 |
| `config/gz_bridge.yaml` · `config/camera_info_fix.yaml` | Gazebo ↔ ROS 브리지 목록, camera_info · D435 깊이 · 점구름 고침 목록 (생성) |
| `launch/display.launch.py` · `launch/gazebo.launch.py` | RViz 보기 · Gazebo |
| `rviz/display.rviz` · `rviz/gazebo.rviz` | RViz 설정 |
| `worlds/dice_mission.sdf` | 주사위 기억 로봇 경기장 (발제 v5) |
| `scripts/` | `keyboard_teleop.py` (키보드 조작), `tfmini_scan_to_range.py`, `camera_info_fix.py` (Gazebo 보조 노드), `slider_to_gazebo.py` |
| `meshes/` | `b` (차체 · 증속벨트 · 브라켓 · 하판 · 팔레트, 충돌 조각 `b/collision` = 브라켓 · 팔레트 CoACD), `main` (바퀴 · OpenCR), `arm` (arm_bench), `parts` (LDS-03 · TFmini · Jetson), `collision` (TO 캐리어 볼록 조각, 261028 것 +12.5 mm) |
| `CHANGES_B.md` | 261028 → B안 바뀐 것 · 질량 · 충돌 · 점검 · 확인할 것 |

## 다시 만들기

이 패키지는 저장소 밖 생성기 `~/Downloads/Claude outputs/puck_storage_design/시뮬레이션_모델_B안/` 의 결과를 이름만 바꿔 넣은 것이다
(설계 SCAD `../B안_팔레트/puck_pallet.scad` · `../구동_증속벨트/` · `../하단_브라켓_전장/` + 팀 `~/Downloads/urdf_test/urdf_261028` 원본을 읽음).

```bash
cd "~/Downloads/Claude outputs/puck_storage_design/시뮬레이션_모델_B안"
PY=/경로/python ./build_all.sh --usd        # waffle_4wd_b_description/ · isaac/ 를 다시 만듦 (OpenSCAD · ROS 2 · trimesh · coacd · lxml · Isaac Lab)
```

저장소에 넣을 때 바꾸는 것:
- 패키지 폴더 `waffle_4wd_b_description` → `waffle_4wd_description`, URDF `waffle_4wd_b*.urdf(.xacro)` → `waffle_4wd*.urdf(.xacro)`
- 텍스트 파일 (메시 제외) 의 `waffle_4wd_b_description` → `waffle_4wd_description`, `waffle_4wd_b_loaded` → `waffle_4wd_loaded`, `waffle_4wd_b.urdf` → `waffle_4wd.urdf` (Isaac `urdf/*.urdf` · `tools/make_isaac_urdf.py` 도)
- `worlds/empty.sdf` 는 넣지 않음
- `config/pallet_slots.yaml` 은 생성기의 `build/stl/pallet_vals.json` (조인트1 · 상판 기준 mm) 에 (96, 0, 111) mm 를 더해 m 로 옮긴 것
