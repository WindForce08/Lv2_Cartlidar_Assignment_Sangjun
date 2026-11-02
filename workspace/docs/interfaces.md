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
| /scan_front · /scan_rear | sensor_msgs/LaserScan | SensorData (intra) | LiDAR 드라이버 | scan_merger |
| /scan | sensor_msgs/LaserScan | SensorData | scan_merger | slam_toolbox, nav2, drive_node |
| /top_camera_front · /top_camera_rear `/color/image_raw` · `/color/camera_info` · `/aligned_depth_to_color/image_raw` | sensor_msgs/Image · CameraInfo | SensorData | realsense2_camera (bringup `sensors.launch.py`) · Isaac 브리지 — frame_id `top_camera_*_color_optical_frame` | perception_node |
| /arm_camera/image_raw · /arm_camera/camera_info | sensor_msgs/Image · CameraInfo | SensorData | usb_cam (bringup `sensors.launch.py`) · Isaac 브리지 — frame_id `arm_camera_optical_frame`, 640×480 | perception_node |
| /perception/result | robot_interfaces/PerceptionResult | Reliable, depth 1 | perception_node | mission, drive, manipulator |
| /arm_range | sensor_msgs/Range | SensorData | perception_node (ARM_ALIGN, 팔 영상마다) — 팔 끝(`tool_link`) → 정렬 중인 퍽 **표면** 거리. ① Arducam 태그 퍽 중심(READ_TAG) ② D435 목표를 odom 에 붙잡아 둔 점(3 s 안). 못 구하면 +inf. **TFmini 아님** | manipulator (미세 보정 · 최종 전진 · pick ①) |
| /perception/arm_range | sensor_msgs/Range | SensorData | perception_node (ARM_ALIGN, `arm.plane`) — 팔 끝(`tool_link`) → 잠근 퍽 **중심** 거리 (팔 카메라 평면 교차). 함께 /perception/arm_target (PointStamped). ★ 위 `/arm_range`(퍽 **표면**)와 다른 토픽 — 이름이 비슷하니 헷갈리지 말 것, 두 값 차이 ≈ 퍽 반지름 | (없음 — 예비 · 검증용) |
| /tfmini/range | sensor_msgs/Range | SensorData, 50 Hz | tfmini_node (실물) · scan_to_range (Gazebo) — 1층 렌즈(`tfmini_link`) → **퍽이 놓인 박스 앞면** 거리, mm 출력 + 실측 보정 | drive (FINE 정렬 전후), obstacle_stop |
| /perception/pucks | robot_interfaces/PuckArray | Reliable, depth 1 (`qos::result()`, /perception/result 와 같음) | perception_node | perception_survey (mission 은 필요하면) |
| /perception/puck_survey | std_msgs/String (JSON) | Reliable + TransientLocal, depth 1 | perception_survey — B0 박스 6 · 칸 12. 필드 `slots[]`(box, side, x_mm, y_mm, color, shape, pose, id, status …) · `complete` · `errors` · `revisit` · `unconfirmed` · `frame`(발제 원점, mm). `complete` false 면 할당에 쓰지 않음. 자세한 것은 perception_survey/README.md | mission |
| /perception/obstacle_stop | std_msgs/Bool | Reliable + TransientLocal, 10 Hz | obstacle_stop (`/tfmini/range` 판정) | twist_mux (lock priority 60, 0.5 s: drive · nav 막음, 조이스틱 통과) |
| /map | nav_msgs/OccupancyGrid | Reliable + TransientLocal | slam_toolbox | nav2 |
| /odom | nav_msgs/Odometry | 기본 | diff_drive_controller | ekf |
| /imu/data | sensor_msgs/Imu | 기본 | imu_sensor_broadcaster | ekf |
| /odometry/filtered | nav_msgs/Odometry | 기본 | ekf | nav2, drive_node |
| /cmd_vel_teleop | TwistStamped | 기본 | teleop_twist_joy | twist_mux (100, 0.5 s) |
| /cmd_vel_drive | TwistStamped | 기본 | drive_node (align · skill) | twist_mux (50, 0.25 s) |
| /cmd_vel_nav | TwistStamped | 기본 | nav2 체인 마지막 (collision_monitor) | twist_mux (10, 0.5 s) |
| /cmd_vel | TwistStamped | 기본 | twist_mux | diff_drive_controller |
| /arm_controller/joint_trajectory | trajectory_msgs/JointTrajectory | 기본 | manipulator_node (RL 스트리밍) | arm_controller |
| /mission/status | robot_interfaces/MissionStatus | 기본, 2 Hz + 변경 시 | mission_node | 모니터링 |
| /joint_states | sensor_msgs/JointState | 기본, 50 Hz | joint_state_broadcaster | robot_state_publisher, manipulator |
| /joy | sensor_msgs/Joy | 기본, 20 Hz autorepeat | joy_node | teleop_twist_joy, OpenCRHardware (heartbeat) |
| /e_stop | std_msgs/Bool | Reliable + TransientLocal, 10 Hz | OpenCRHardware (잠금식) | twist_mux (lock 0.5 s), mission, manipulator |
| /diagnostics | diagnostic_msgs/DiagnosticArray | 기본, 1 Hz + 오류 즉시 | OpenCRHardware (`opencr/link · motors · battery · estop`, hardware_id `opencr`) | mission (opencr/ 만) |
| /tf · /tf_static | tf2_msgs/TFMessage | 기본 · TransientLocal | slam_toolbox · ekf · robot_state_publisher · realsense2_camera — 규약 [frames_tf.md](frames_tf.md) | 전체 |
| /arm/skill_state · /arm/safety_state | robot_interfaces/SkillState · SafetyState | TransientLocal | manipulator_node | 모니터링 — **표 추가 여부 결정 필요** |

## Action · Service

| 이름 | 종류 | 타입 | Client | Server |
|---|---|---|---|---|
| /drive/navigate | Action | nav2_msgs/NavigateToPose | mission | drive → nav2 (같은 위치면 spin) |
| /drive/align | Action | robot_interfaces/AlignBase | mission | drive |
| /drive/execute_skill | Action | robot_interfaces/ExecuteSkill | mission | drive |
| /manipulator/pose | Action | robot_interfaces/ArmPose | mission | manipulator |
| /manipulator/pick | Action | robot_interfaces/Pick | mission | manipulator |
| /manipulator/execute_skill | Action | robot_interfaces/ExecuteSkill | mission | manipulator |
| navigate_to_pose | Action | nav2_msgs/NavigateToPose | drive | nav2 |
| spin | Action | nav2_msgs/Spin | drive | nav2 behavior_server |
| /arm_controller/follow_joint_trajectory | Action | control_msgs/FollowJointTrajectory | manipulator | arm_controller |
| /gripper_controller/gripper_cmd | Action | control_msgs/ParallelGripperCommand | manipulator | gripper_controller |
| /perception/set_mode | Service | robot_interfaces/SetPerceptionMode | mission | perception |
| /perception/survey/start · /reset · /finish · /revisit | Service | std_srvs/Trigger | mission | perception_survey — 순서: start → `complete` 토픽 → finish 성공. finish 실패면 revisit 로 문제 칸만 다시 |
| /mission/start · /mission/stop | Service | std_srvs/Trigger | joy_teleop · CLI | mission |

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
| Puck · PuckArray | /perception/pucks. Puck = tag_id · color · shape · pose (base_link) · standing · confidence · camera |

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
