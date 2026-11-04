# 좌표 (TF) · 카메라 연결 규약

실물과 Isaac Sim 이 같은 프레임 이름 · 토픽 · 축 규약을 쓰게 하는 기준이다 (2026-10-28, tutor-s-robot `docs/CAMERA_AND_TF.md` 형식 참고).
URDF 는 `opencr_hardware/urdf/robot.urdf.xacro`. 시뮬레이션 참고본 (`~/Downloads/urdf_test/`):

| 참고본 | 쓰는 것 |
|---|---|
| `waffle_4wd_usd_261027_1711/waffle_4wd/waffle_4wd.usda` | Isaac Sim 6.2 시뮬레이션 USD (`workspace/tools/isaac_sim/waffle_bridge.py`) |
| `urdf_261027_1711/waffle_4wd_description_261027_1711/` | 위 USD 의 원본 URDF. 센서 프레임 근거는 `simulation_metadata.json` |
| `URDF_USD_IsaacSim62_v4/` | URDF → USD 변환 규칙 (`code/convert_urdf62.py`). `output/robot.urdf` (10-22) 는 LiDAR roll 0 인 옛 값이라 근거로 쓰지 않는다 |

## 축

- `base_link`: +X 앞 · +Y 왼쪽 · +Z 위 (ROS REP 103). 원점 = 기준 모델(waffle_4wd_description B안)과 같음 — 바닥 위 6.75 mm (바퀴 중심 z 0.02625 − 반지름 0.033). 2026-11-02 `add7244` 부터 (이전 저장소 값 56.25 mm).
- 광학 프레임 (`*_optical_frame`): +X 영상 오른쪽 · +Y 영상 아래 · +Z 앞 (보는 방향). 몸체 프레임 기준 rpy (-π/2, 0, -π/2).

## 프레임

| 프레임 | 부모 | 발행 | 쓰는 곳 | 상태 |
|---|---|---|---|---|
| `map` → `odom` | — | slam_toolbox | nav2 | |
| `odom` → `base_link` | — | **ekf 만** (diff_drive `enable_odom_tf: false`) | perception · manipulator (`fixed_frame`) | |
| `base_footprint` | base_link | robot_state_publisher (URDF) | perception 찾기 · 바코드 (`base_frame`) | |
| `imu_link` | base_link (0.0095, 0, 0.0202), rpy 0 | URDF | imu_sensor_broadcaster · ekf | 아래 OpenCR 보드를 바로 놓음 (2026-11-02 확인, 이전 roll π 는 틀림). Isaac 브리지 `IMU_RPY` 도 rpy 0 (2026-11-03) |
| `laser_front` · `laser_rear` | base_link (±0.1089, 0, 0.0755), roll π (뒤는 + yaw π) | URDF | LiDAR 드라이버 frame_id · slam_toolbox | 뒤집어 장착 반영 (2026-11-02). 스캔면 바닥 위 82 mm |
| `top_camera_front_link` · `top_camera_rear_link` | base_link (앞 (−0.0481, +0.0250, 0.4836) · 뒤 (−0.2079, −0.0250, 0.4836)), pitch 0.6144 rad | URDF | realsense2_camera TF 뿌리 (왼쪽 IR = 깊이 원점) | C 적용 (2026-11-02) |
| `top_camera_*_link` → `top_camera_*_color_optical_frame` | top_camera_*_link | realsense2_camera (실물) · Isaac 브리지 `/tf_static` (모의) | perception D435 · rear_obstacle | URDF 는 만들지 않음 (부모가 둘이 되지 않게) |
| `arm_base_link` → `link1` ~ `gripper_roll_link` | base_link (0.096, 0, 0.1135) — joint2 축이 기준 모델 자리 | URDF + /joint_states | manipulator (`arm_base_frame`) | |
| `arm_camera_link` → `arm_camera_optical_frame` | gripper_roll_link | URDF | Arducam UVC 드라이버 frame_id · perception 태그 | 그리퍼 장착 확인 (2026-10-28) |
| `tfmini_link` | base_link (0.130, 0, 0.0243) | URDF | tfmini_node · scan_to_range (`frame_id`) — 박스 앞면 거리 | 렌즈면 원점. 받침대 설계 · Isaac 값, ★ 실측 |
| `tool_link` | gripper_roll_link | URDF | perception `arm.range.frame` · `arm.plane.range_frame` · manipulator `range_frame` | ★ 이 프레임 기준 거리 토픽이 둘: `/arm_range` = 퍽 **표면** (manipulator 가 씀) · `/perception/arm_range` = 퍽 **중심** (예비 · 검증). 이름이 비슷하니 헷갈리지 말 것 |

## 저장소 ↔ USD (261027) 대응

| 저장소 | USD 261027 | 비고 |
|---|---|---|
| `base_link` (뿌리) → `base_footprint` (z -0.00675) | `base_footprint` (뿌리) → `base_link` (z +0.00675) | **부모 · 자식 방향이 반대.** 저장소 방향을 유지한다 (ekf 가 `odom → base_link` 를 발행하므로 base_link 가 뿌리여야 부모가 하나) |
| `top_camera_front_link` · `top_camera_rear_link` | 같은 이름. 아래에 `_optical_frame` → `_infra1/2_optical_frame` · `_color_optical_frame` | 원점 규약이 다름 (아래 D435 원점) |
| `arm_camera_link` → `arm_camera_optical_frame` | `camera_link` → `camera_optical_frame` (arm_link_5 아래) | Arducam |
| `laser_front` · `laser_rear` | `lidar_front` → `lidar_front_scan` (뒤도 같음) | 드라이버 frame_id 이름 추후 확정 |
| `imu_link` | 없음 | 브리지가 저장소 `imu_link` 축(rpy 0, `IMU_RPY`)으로 계산해 `/imu/data` 발행 |
| `tfmini_link` | 없음 | 시뮬레이션에 TFmini 센서 없음 |
| 관절 `wheel_{left,right}_{front,rear}_joint` · `joint1~5` · `gripper_roll` | `Wheel_Revolute_*` · `RMX52_*` (팔 부호 반대) · 손가락 `gripper_left/right_joint` | 대응 · 부호 · 그리퍼 변환은 `waffle_bridge.py` |

### D435 원점 (C 를 고칠 때)

- realsense2_camera 의 `<name>_link` 는 **깊이 원점 = 왼쪽 IR (infra1)** 이다. 드라이버가 컬러를 거기서 몸체 +y 약 0.015 m 에 발행한다.
- USD · 261027 의 `top_camera_*_link` 는 **두 IR 렌즈 가운데** (CAD 렌즈 원통 중간점) 다. infra1 은 몸체 +y 0.025 m (USD 확인: 앞 +0.0250, 뒤 -0.0250 — 뒤는 요 180°).
- 그래서 저장소 URDF 의 `top_camera_*_link` 는 261027 link 위치가 아니라 **infra1 위치** (앞 y +0.025, 뒤 y -0.025 만큼 옮긴 곳) 에 둔다. 그대로 쓰면 실물에서 컬러가 25 mm 어긋난다.
- 261027 의 컬러 외부보정 (`_optical_frame` 기준 (-0.04012, 0.00007, -0.00018), rpy (-0.0064, -0.0035, 0.0037)) 은 실물 D435 serial 261822072247 공장값 (rs-enumerate-devices -c, 2026-10-22). 둘째 D435 도 같다고 가정. 실물에서는 드라이버가 자기 공장값을 발행한다.

### 위치값 기준 (2026-11-02 부터)

저장소 URDF 센서 · 바퀴 위치는 **기준 모델 waffle_4wd_description B안** 값이다 (`add7244`). 아래 USD 261027 은 그 이전 기준이라 값이 다르다 (★ 실물 측정으로 최종 확인).

| 항목 | 저장소 (= B안) | USD 261027 (이전) |
|---|---|---|
| base_link 원점 높이 | 바닥 위 6.75 mm | 바닥 위 6.75 mm |
| 바퀴 | x 앞 0.0925 · 뒤 −0.0675, y ±0.1585 (belt, 간격 0.317) | x ±0.08, y ±0.1546 (간격 0.3092) |
| D435 기울기 | `top_camera_*_link` 0.6144 rad | 35.2° (0.6144 rad) |
| LiDAR 스캔 프레임 | (±0.1089, 0, 0.0755) roll π | (±0.1855, ±0.05, 0.0609) — 중심에서 옆으로 50 mm |
| TFmini | (0.130, 0, 0.0243) | 없음 |

## 적용할 것 (주석으로 남김)

| 항목 | 내용 | 위치 |
|---|---|---|
| C | **적용 (2026-11-02, `add7244`)**: URDF 에 realsense TF 뿌리 `top_camera_front_link` · `top_camera_rear_link` (infra1 = 깊이 원점) 를 두어 `base_link → top_camera_*_color_optical_frame` 이 이어짐. 시험이 이름을 확인 | robot.urdf.xacro, test_robot_description.cpp |
| LiDAR roll π | **적용 (2026-11-02)**: LDS-03 은 3층 아래에 뒤집어 매달림 → `laser_*` roll π. 드라이버 reverse 는 끔 (sensors.launch.py) | robot.urdf.xacro |
| IMU 축 | **적용 (2026-11-03)**: Isaac 브리지 `IMU_RPY` 를 URDF `imu_link` 와 같은 rpy 0 으로 (이전 (π, 0, π/2) 는 뒤집어 단 옛 가정 → 모의 ekf 회전 부호 반대) | tools/isaac_sim/waffle_bridge.py |
| TFmini | **적용 (2026-11-02)**: `tfmini_link` 를 base_link (0.130, 0, 0.0243) 로 옮김 (바닥 위 31.05 mm, base_link 는 바닥 위 6.75 mm). /arm_range 는 이제 perception 계산값(기준 tool_link)이라 manipulator `range_offset` = 퍽 반지름 0.015 | robot.urdf.xacro, manipulator.yaml, test_robot_description.cpp |
| A | manipulator STOW joint3 -2.3 이 펌웨어 범위 밖 → STOW 명령 거부 | manipulator.yaml |
| B | manipulator `lower/upper_limits` 가 URDF · config.h 보다 넓음 | manipulator.yaml |

## TF 발행 원칙

- 한 변환은 한 노드만 발행한다. `odom → base_link` 는 ekf, `map → odom` 은 slam_toolbox.
- realsense2_camera 는 `<camera_name>_link` 아래 (`_color_optical_frame` 등) 만 발행한다. URDF 는 그 아래 프레임을 만들지 않는다 (시험이 확인).
- 시뮬레이션의 정답 자세 (ground truth) 는 `/tf` 에 넣지 않는다. `sim_world` 프레임이나 별도 토픽으로 기록해 추정값과 비교한다.
- 로봇 TF 는 실물 · 시뮬레이션 모두 robot_state_publisher 가 저장소 URDF 로 만든다. 시뮬레이션은 `sim:=true` 로 TopicBasedSystem 이 `/isaac_joint_states` 를 받아 joint_state_broadcaster 가 `/joint_states` (저장소 이름) 를 내므로 팔 · `arm_camera_optical_frame` TF 도 같은 방식으로 생긴다.
- 시뮬레이션에는 realsense 드라이버가 없으므로 **브리지가 드라이버 대신 `top_camera_*_link → top_camera_*_color_optical_frame` 을 `/tf_static` 으로 발행**한다 (값은 위 D435 원점 · 공장 외부보정, 실물 드라이버와 같은 관계).
- C 는 적용됐다 (2026-11-02): URDF 에 `top_camera_*_link` 가 있어 예전의 `static_transform_publisher` 임시 연결은 필요 없다.

## Isaac Sim 발행 규약

`workspace/tools/isaac_sim/run_bridge.sh` (Isaac Sim 6.2 단독, 내장 jazzy rclpy) ↔ 호스트 `ros2 launch bringup bringup.launch.py sim:=true use_sim_time:=true use_teleop:=false`.

| 토픽 | 방향 | 타입 | frame_id · 내용 |
|---|---|---|---|
| `/isaac_joint_states` | Isaac → 호스트 | sensor_msgs/JointState | 저장소 관절 이름 (TopicBasedSystem 이 받음) |
| `/isaac_joint_commands` | 호스트 → Isaac | sensor_msgs/JointState | 바퀴 velocity · 팔 position. 0.5 s 끊기면 바퀴 정지 |
| `/clock` | Isaac → 호스트 | rosgraph_msgs/Clock | 시뮬레이션 시간 |
| `/imu/data` | Isaac → 호스트 | sensor_msgs/Imu | `imu_link` (URDF 와 같은 축, rpy 0) |
| `/e_stop` | Isaac → 호스트 | std_msgs/Bool | false 10 Hz (실물은 OpenCRHardware) |
| `/top_camera_front/color/image_raw` · `/color/camera_info` | Isaac → 호스트 | Image · CameraInfo | `top_camera_front_color_optical_frame` |
| `/top_camera_front/aligned_depth_to_color/image_raw` | Isaac → 호스트 | Image 32FC1 m (컬러와 같은 크기, 맞은 곳 없으면 0) | `top_camera_front_color_optical_frame` |
| `/top_camera_rear/...` | Isaac → 호스트 | 위와 같음 | `top_camera_rear_color_optical_frame` |
| `/arm_camera/image_raw` · `/arm_camera/camera_info` | Isaac → 호스트 | Image · CameraInfo | `arm_camera_optical_frame` (640×480) |
| `/tf_static` | Isaac → 호스트 | TFMessage | `top_camera_*_link` → `top_camera_*_color_optical_frame` (실물은 realsense2_camera 가 냄, USD 261027 외부보정 값) |
| `/scan_front` · `/scan_rear` | Isaac → 호스트 | sensor_msgs/LaserScan, 10 Hz · 360 점 | `laser_front` · `laser_rear` (roll π). PhysX 레이캐스트, 로봇 자신은 맞지 않음, 못 잼 = inf |

카메라 30 Hz (`--cams`, 기본 front,rear,arm) · 라이다 10 Hz (`--lidars`, 기본 front,rear). 카메라 · 라이다 prim 은 USD 의 광학 · 스캔 프레임 prim (`top_camera_*_color_optical_frame` · `camera_optical_frame` · `lidar_*_scan`) 을 찾아 쓴다. 고정 관절을 합친 USD (urdf_and_usd B안 등) 처럼 prim 이 없으면 경고하고 그 센서만 건너뛴다.
깊이는 32FC1 m (실물 realsense 는 16UC1 mm, perception 이 둘 다 받음). 시험 (2026-11-02, 벽 2개 장면, headless): 스캔 거리 기하 계산과 1 mm 안, 자기 맞음 0, 카메라 3대 30 Hz, publish 평균 5.3 ms · 최대 48 ms (라이다 프레임), 실시간비 0.91.

- 토픽 · frame_id 는 실물 드라이버와 같은 이름 (perception.yaml 을 바꾸지 않고 쓰기 위해).
- Isaac 카메라 prim 은 link 가 아니라 **컬러 광학 프레임 위치** (USD `top_camera_*_color_optical_frame`, link 에서 약 40 mm) 에 둔다. 그래야 영상과 TF 가 맞는다.
- CameraInfo 는 렌더러 값으로 계산한다: fx = 폭 × focalLength / horizontalAperture, fy = 높이 × focalLength / verticalAperture. D435 컬러 640×480 (실물 perception 설정) 은 HFOV 약 55° · VFOV 42°, fx ≈ 615 (Isaac Lab 3 시험 fx 614.7). 실물 이전 때는 realsense 의 CameraInfo 를 쓴다.
- 영상 · 깊이 · CameraInfo 는 같은 시뮬레이션 스텝의 stamp (perception `depth.sync_tol_s` 0.02 s). 노드는 `use_sim_time: true`.
- URDF → USD 변환은 `merge_fixed_joints=False` 로 한다 (v4 `convert_urdf62.py`). 고정 관절을 합치면 센서 프레임 prim 이 사라진다. `validate_usd.py` 가 그 prim 들을 확인한다.

Isaac Lab 3 (Isaac Sim 6.1) 에서 확인한 것 (2026-10-28, Sim 6.2 단독 실행은 확인 안 함):

- OmniGraph ROS 카메라 도우미 (ROS2CameraHelper) 는 못 쓴다. Camera 센서는 타일 렌더 프로덕트라 넘길 수 없고, 따로 만든 렌더 프로덕트는 그려지지 않아 빈 영상만 나간다.
- `OnPlaybackTick` 은 발화하지 않고 (Kit 타임라인을 재생하지 않음), 창이 없으면 `app.update()` 도 불리지 않는다. 확장을 실행 중에 켜면 USD 스키마 등록이 깨진다 (`--kit_args "--enable ..."` 로 시작할 때 켬).
- 해결: Camera 센서 값을 Isaac 내장 jazzy rclpy (`exts/isaacsim.ros2.core/jazzy/rclpy`) 로 직접 발행 — 30 Hz (시뮬레이션 시간), 영상 · info · 깊이 stamp 일치, 카메라 3대 VRAM 5.1 GB.
- Kit 뷰포트 창을 더 열지 않는다 (모든 뷰포트가 지오메트리를 못 그림).

## 점검

```bash
ros2 run tf2_ros tf2_echo base_link top_camera_front_color_optical_frame   # 드라이버(실물) · 브리지(모의)가 광학 프레임을 내야 이어짐
ros2 run tf2_ros tf2_echo base_link arm_camera_optical_frame
ros2 run tf2_tools view_frames                                            # 부모가 둘 · 끊긴 나무 확인
ros2 topic echo /top_camera_front/color/camera_info --once
ros2 topic hz /top_camera_front/color/image_raw
```

정적 검사:

- URDF: `colcon test --packages-select opencr_hardware` (test_robot_description — 나무 구조 · 프레임 · 광학 축 · 구동 설정).
- USD: `workspace/tools/validate_usd.py <usd>` (usd-core 만 필요, Isaac 없이). 결과 `PASS_STATIC_ONLY` 는 물리 · 렌더링 · ROS 연결을 보지 않았다는 뜻이다.
