# 통신 정의 (동결)

변경은 이 파일에 PR + Controller 팀장 승인. robot_interfaces 소유: Controller (Perception 리뷰).
공통 규칙:
- 명령 주체는 하나 (바퀴 = drive_node, 팔 · 그리퍼 = manipulator_node)
- /cmd_vel 계열은 TwistStamped
- 인식 · 거리 값은 header.stamp (촬영 시각, ROS 시계) 300 ms 지나면 쓰지 않음
- 200 ms (펌웨어 명령 무수신 정지) 와 300 ms (값의 최대 나이) 는 다른 규칙
- 이름 · QoS 는 코드에 직접 쓰지 않고 `robot_interfaces/names.hpp` 의 `names::` · `qos::` 상수를 쓴다 (이 표와 같이 고침)
- 300 ms 예외 하나: `/arm_range` 의 D435 보유 점은 "odom 에 고정된 정지 퍽 위치"를 현재 팔 자세로 다시 계산한 값이라 D435 가 마지막으로 본 뒤 `arm.range.hold_s`(3 s)까지 쓴다. 메시지 stamp 는 팔 영상 촬영 시각이라 받는 쪽 300 ms 규칙은 그대로
- 박스 접근: drive_node 가 FINE 정렬 동안 obstacle_stop 파라미터 `approach_stop_m` 을 "목표 − 0.03 m" 로 넣고 끝나면 0 으로 되돌린다 (되돌리지 못하면 60 s 뒤 스스로 복귀)
- perception 은 쓸 수 있는 프레임이 없으면 `/perception/result` 를 **발행하지 않는다** (카메라 정지). 받는 쪽은 300 ms 규칙으로 "모름" 처리

## 토픽

| 토픽 | 타입 | QoS | Publisher | Subscriber |
|---|---|---|---|---|
| /scan_front · /scan_rear | sensor_msgs/LaserScan | SensorData (intra) | coin_d4_driver (bringup `sensors.launch.py`, LDS-03) · Isaac 브리지 — frame_id `laser_front` · `laser_rear` | scan_merger |
| /scan | sensor_msgs/LaserScan | SensorData | scan_merger | slam_toolbox, nav2, drive_node |
| /top_camera_front · /top_camera_rear `/color/image_raw` · `/color/camera_info` · `/aligned_depth_to_color/image_raw` | sensor_msgs/Image · CameraInfo | SensorData | realsense2_camera (bringup `sensors.launch.py`) · Isaac 브리지 — frame_id `top_camera_*_color_optical_frame` | perception_node |
| /arm_camera/image_raw · /arm_camera/camera_info | sensor_msgs/Image · CameraInfo | SensorData | usb_cam (bringup `sensors.launch.py`) · Isaac 브리지 — frame_id `arm_camera_optical_frame`, 640×480 | perception_node |
| /perception/result | robot_interfaces/PerceptionResult | Reliable, depth 1 | perception_node | mission, drive, manipulator |
| /arm_range | sensor_msgs/Range | SensorData | perception_node (ARM_ALIGN, 팔 영상마다) — 팔 끝(`tool_link`) → 정렬 중인 퍽 **표면** 거리. ① Arducam 태그 퍽 중심(READ_TAG) ② D435 목표를 odom 에 붙잡아 둔 점(3 s 안) ③ 둘 다 없으면 팔 카메라 평면 교차 퍽 중심(`arm.range.plane_gate`). 못 구하면 +inf. **TFmini 아님** | manipulator (미세 보정 · 최종 전진 · pick ①) |
| /perception/arm_range | sensor_msgs/Range | SensorData | perception_node (ARM_ALIGN, `arm.plane`) — 팔 끝(`tool_link`) → 잠근 퍽 **중심** 거리 (팔 카메라 평면 교차). 함께 /perception/arm_target (PointStamped). ★ 위 `/arm_range`(퍽 **표면**)와 다른 토픽 — 이름이 비슷하니 헷갈리지 말 것, 두 값 차이 ≈ 퍽 반지름 | (없음 — 예비 · 검증용) |
| /tfmini/range | sensor_msgs/Range | SensorData, 50 Hz | tfmini_node (실물) · scan_to_range (Gazebo) — 1층 렌즈(`tfmini_link`) → **퍽이 놓인 박스 앞면** 거리, mm 출력 + 실측 보정 | drive (FINE 정렬 전후), obstacle_stop |
| /perception/pucks | robot_interfaces/PuckArray | Reliable, depth 1 (`qos::result()`, /perception/result 와 같음) | perception_node | perception_survey, manipulator (target_look 타겟체크자세에서 예상 자리 퍽, #82) |
| /perception/puck_survey | std_msgs/String (JSON) | Reliable + TransientLocal, depth 1 | perception_survey — B0 박스 6 · 칸 12. 필드 `slots[]`(box, side, x_mm, y_mm, color, shape, pose, id, status …) · `complete` · `errors` · `revisit` · `unconfirmed` · `frame`(발제 원점, mm). `complete` false 면 할당에 쓰지 않음. 자세한 것은 perception_survey/README.md | (mission 연동 미구현 — 계획) |
| /perception/signal | robot_interfaces/SignalSequence | Reliable, depth 1 | perception_node (READ_SIGNAL) | mission (plan 모드), mission_planner |
| /perception/barcode | robot_interfaces/Barcode | Reliable, depth 1 | perception_node (READ_BARCODE) | mission (plan 모드), mission_planner |
| /perception/task_marker | robot_interfaces/TaskMarker | Reliable, depth 1 | perception_node (READ_MARKER) | (없음) |
| /perception/rear_obstacle · `/reason` · `/distance` | std_msgs/Bool · String · Float32 | Reliable + TransientLocal (distance 기본) | rear_obstacle (뒤 D435 깊이, perception.launch `use_rear_obstacle`) | drive_node (후진 v < 0 확인, `scan.rear_camera`, 값 없음 = 막힘 · #84) |
| /localization/status | std_msgs/String | Reliable + TransientLocal | loc_monitor (localization 패키지, 10 Hz · 바뀔 때: OK · SUSPECT · LOST · RECOVERING · CONFIRM · FAILED) | perception_survey (OK 일 때만 관측을 받음, 1 s 넘게 못 받으면 그대로 받음) |
| /localization/lock | std_msgs/Bool | Reliable + TransientLocal | loc_monitor (OK · SUSPECT 가 아니면 true, 10 Hz) | twist_mux lock `localization` (priority 20 = nav 만 막음, #93) — base_bringup `loc_lock:=true` · bringup `use_loc_lock:=true` 일 때만 (끊겨도 잠김) |
| /perception/place_box · `/cells` · `/status` | geometry_msgs/PoseStamped · PoseArray · std_msgs/String (JSON) | Reliable + TransientLocal | perception_node (`/perception/place_box/measure` Trigger 뒤, 팔 영상, odom) | mission (measure_box, #73: place_at · verify 칸) |
| /perception/obstacle_stop | std_msgs/Bool | Reliable + TransientLocal, 10 Hz | obstacle_stop (`/tfmini/range` 판정) | twist_mux (lock priority 60, 0.5 s: drive · nav 막음, 조이스틱 통과) |
| /perception/obstacle_stop/reason | std_msgs/String | Reliable + TransientLocal | obstacle_stop (obstacle · too_close · stale · clear · no_return) | 모니터링 |
| /tfmini/scan | sensor_msgs/LaserScan | SensorData | Gazebo (gz_bridge) | scan_to_range → /tfmini/range |
| /map | nav_msgs/OccupancyGrid | Reliable + TransientLocal | slam_toolbox | nav2 |
| /odom | nav_msgs/Odometry | 기본 | diff_drive_controller | odom_icr |
| /odom_icr | nav_msgs/Odometry | 기본 | bringup odom_icr (/odom + vy = −icr_x·ω, IMU ω, #92) | ekf (odom0 vx · vy) |
| /imu/data | sensor_msgs/Imu | 기본 | imu_sensor_broadcaster (실물) · Isaac 브리지 (모의) | ekf |
| /odometry/filtered | nav_msgs/Odometry | 기본 | ekf | nav2, drive_node |
| /cmd_vel_teleop | TwistStamped | 기본 | teleop_twist_joy | twist_mux (100, 0.5 s) |
| /cmd_vel_drive | TwistStamped | 기본 | drive_node (align · skill) | twist_mux (50, 0.25 s) |
| /cmd_vel_nav | TwistStamped | 기본 | nav2 체인 마지막 (collision_monitor) | twist_mux (10, 0.5 s) |
| /cmd_vel | TwistStamped | 기본 | twist_mux | diff_drive_controller |
| /arm_controller/joint_trajectory | trajectory_msgs/JointTrajectory | 기본 | manipulator_node (RL 스트리밍) | arm_controller |
| /mission/status | robot_interfaces/MissionStatus | 기본, 2 Hz + 변경 시 | mission_node | 모니터링 |
| /joint_states | sensor_msgs/JointState | 기본, 100 Hz (controller_manager update_rate). | joint_state_broadcaster | robot_state_publisher, manipulator |
| /joy | sensor_msgs/Joy | 기본, 20 Hz autorepeat | joy_node | teleop_twist_joy, joy_teleop (시작 · 정지 버튼 → /mission/start · stop), OpenCRHardware (E-stop · 리셋 버튼, heartbeat) |
| /e_stop | std_msgs/Bool | Reliable + TransientLocal, 10 Hz | OpenCRHardware (잠금식) · Isaac 브리지 (모의, 항상 false) | twist_mux (lock 0.5 s), mission |
| /diagnostics | diagnostic_msgs/DiagnosticArray | 기본, 1 Hz + 오류 즉시 | OpenCRHardware (`opencr/link · motors · battery · estop`, hardware_id `opencr`) | mission (opencr/ 만) |
| /tf · /tf_static | tf2_msgs/TFMessage | 기본 · TransientLocal | slam_toolbox · ekf · robot_state_publisher · realsense2_camera — 규약 [frames_tf.md](frames_tf.md) | 전체 |
| /arm/skill_state · /arm/safety_state | robot_interfaces/SkillState · SafetyState | TransientLocal | **(발행 노드 없음 — 이름 · 메시지만 정의, manipulator 가 내지 않음)** | 모니터링 — **유지 또는 삭제 결정 필요** |
| /clock · /isaac_joint_states · /isaac_joint_commands | rosgraph_msgs/Clock · sensor_msgs/JointState | 기본 | Isaac 브리지 (`/clock` · 관절 상태) · TopicBasedSystem (관절 명령) | 모의 전용 (`sim:=true`) |

## Action · Service

| 이름 | 종류 | 타입 | Client | Server |
|---|---|---|---|---|
| /drive/navigate | Action | nav2_msgs/NavigateToPose | mission | drive → nav2 (같은 위치면 spin) |
| /drive/align | Action | robot_interfaces/AlignBase | mission | drive |
| /drive/execute_skill | Action | robot_interfaces/ExecuteSkill | mission | drive (강화학습 정책, 또는 정해진 회전 `turn_p<deg>` · `turn_m<deg>` · `turn_0` = 직전 FINE 정렬 방향 기준, #77) |
| /manipulator/pose | Action | robot_interfaces/ArmPose | mission | manipulator |
| /manipulator/pick | Action | robot_interfaces/Pick | mission | manipulator |
| /manipulator/execute_skill | Action | robot_interfaces/ExecuteSkill | mission | manipulator |
| navigate_to_pose | Action | nav2_msgs/NavigateToPose | drive | nav2 |
| spin | Action | nav2_msgs/Spin | drive | nav2 behavior_server |
| /arm_controller/follow_joint_trajectory | Action | control_msgs/FollowJointTrajectory | manipulator | arm_controller |
| /gripper_controller/gripper_cmd | Action | control_msgs/ParallelGripperCommand | manipulator | gripper_controller |
| /perception/set_mode | Service | robot_interfaces/SetPerceptionMode | mission | perception |
| /perception/survey/start · /reset · /finish · /revisit | Service | std_srvs/Trigger | (mission 연동 미구현 — CLI) | perception_survey — 순서: start → `complete` 토픽 → finish 성공. finish 실패면 revisit 로 문제 칸만 다시 |
| /mission/start · /mission/stop | Service | std_srvs/Trigger | joy_teleop · CLI | mission |
| /perception/check_dice | Service | robot_interfaces/CheckDice | mission (plan 모드) | perception |
| /mission_planner/plan | Service | std_srvs/Trigger | mission (plan 모드, `mission_plan.yaml` service) | mission_planner |
| /obstacle_stop `approach_stop_m` | Parameter | double (0 = 끔) | drive (FINE 정렬 시작 · 끝) | obstacle_stop |

## 메시지

정의 파일: `ros2_ws/src/robot_interfaces/{msg,srv,action}`. 아래 표의 메시지 정의는 robot_nodes.html 과 같다.

| 메시지 | 비고 |
|---|---|
| PerceptionResult | |
| SetPerceptionMode | 비트 |
| AlignBase | |
| ArmPose | result `success` · `message` (2026-10-22 message 추가) |
| Pick | `grasped` 포함 |
| ExecuteSkill | |
| MissionStatus | |
| SkillState · SafetyState | 모니터링 전용 |
| TaskMarker | /perception/task_marker (장소 태그) |
| SignalSequence | /perception/signal (신호등 점멸 순서, CONFIRMED 12개) |
| Barcode | /perception/barcode (LOCATE 위치 · READ 문자열) |
| CheckDice | /perception/check_dice (선반 3×3 주사위 판정) |
| Puck · PuckArray | /perception/pucks. Puck = tag_id · color · shape · pose (base_link) · standing · confidence · camera |

- **상자 퍽 면 요** (#81, 메시지는 그대로): 대상은 `shape = SHAPE_CUBOID` · `standing` · `orientation_valid`인 퍽이다.
  - 값: `pose.orientation`의 x축을 base_link xy 평면에 투영한 각을 90° 주기로 접은 것 [−45°, 45°)이다. 태그가 면 모서리와 나란하므로 이 x축이 옆면 방향이다.
  - 헬퍼: C++ `perception/face_yaw.hpp`, Python `perception_survey.face_yaw`(`face_yaw(qx, qy, qz, qw)`, `face_yaw_error(면 요, J1 → 퍽 방향각)`).
  - 팔 카메라 태그 퍽에만 있다. D435 색 검출 퍽은 방향이 없다(orientation_valid false).

## USB 패킷 (opencr_protocol.hpp = opencr_firmware/opencr_protocol.h)

> **v5 제안 (Firmware 팀, 2026-10-26) — Controller 팀장 승인 대기.** dev 의 승인본은 v3 (OpenCR 1대, CMD 54 B · STATE 191 B).
> 바뀐 점과 이유: [opencr_v5_change_request.md](opencr_v5_change_request.md). 승인 전에는 dev/main 에 병합하지 않는다.

- 공통 프레임: `0xFF 0xFD · TYPE · LEN · PAYLOAD · CRC16(Dynamixel 2.0, 리틀엔디안)`
- 구성: OpenCR 두 대 — 주행 보드 (역할 1) 바퀴 ID 1~4 (Velocity Mode), 팔 보드 (역할 2) 팔 ID 11~16 (Position Mode). 프레임의 역할 값으로 보드를 판별한다.
- 바퀴: **4륜** (왼쪽 앞 1 · 왼쪽 뒤 2 · 오른쪽 앞 3 · 오른쪽 뒤 4 = 슬롯 0~3). 패킷은 6자리를 유지하고 슬롯 4 · 5 는 0 (CMD 값은 무시).
- CMD 0x01 (Jetson → OpenCR, 100 Hz, 55 B):
  - `role` · `wheel_vel[6]` · `arm_pos[6]` · `gripper_pos` · `flags` · `seq`
  - flags: bit0 TORQUE_ON · bit1 ESTOP · bit2 GRIPPER_VALID · bit3 REARM (상승 에지로 팔 HOLD 해제)
- STATE 0x81 (OpenCR → Jetson, 100 Hz, 192 B):
  - `role` · `pos[12]` · `vel[12]` · `effort[12]` · `quat[4]` · `gyro[3]` · `accel[3]` · `battery_v` · `error` · `fw_version` · `seq`
  - 주행 보드는 바퀴 슬롯 0~3, 팔 보드는 팔 슬롯 6~11을 채운다. IMU·배터리는 주행 보드 값만 사용한다.
  - error 비트: 0x01 SyncRead 누락 · 0x02 명령 타임아웃 · 0x04 물리 E-stop · 0x08 모터 HW · 0x10 모터 설정 · 0x20 CMD 역할 불일치 ·
    0x40 HOLD (팔이 토크를 유지한 채 멈춤, REARM 으로만 풀림) · 0x80 FAULT (부팅 점검 실패, 토크 거부)
- `PROTOCOL_VERSION = 5`: 버전이 다르면 호스트가 활성화를 거부한다 (v3 은 길이도 다름). 호스트는 역할별 STATE를 하나씩 받고 두 보드 모두 v5인지 확인해야 활성화한다.
- 명령이 끊기거나 (200 ms) ESTOP: 바퀴 0, 팔은 토크를 유지한 채 HOLD (무수신은 마지막 승인 목표, ESTOP 은 현재 위치). 팔 HOLD 는 운용자 리셋 → 호스트 REARM 펄스로만 풀린다.
- OpenCR 빌드: `OPENCR_BOARD_ROLE=1` 주행, `=2` 팔 — 빌드 옵션으로만 정한다 (기본값 없음, `opencr_firmware/tools/build_firmware.sh wheel|arm`).
- 포트: `ports` 파라미터에 두 후보 경로를 쉼표로 전달한다. `/dev/ttyACM0`과 `/dev/ttyACM1`은 후보일 뿐이며, 실제 역할은 STATE.role로 연결한다.
- 부팅 점검 (펌웨어): ping · 모델 번호 · Homing Offset 0 확인, Return Delay 0 기록, (팔) 모터 Min/Max Position Limit 가 관절 범위를 덮는지 확인.
  하나라도 실패면 0x10 · 0x80 → 토크 켜기 거부
- Bus Watchdog 500 ms (25 단위): 토크 켤 때 모든 모터에 설정 — OpenCR 이 멈춰도 모터가 스스로 정지
- 시리얼은 `flock` + `TIOCEXCL` 로 한 프로세스만 연다
