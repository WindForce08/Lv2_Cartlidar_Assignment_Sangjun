# 통신 정의 (동결)

변경은 이 파일에 PR + Controller 팀장 승인. robot_interfaces 소유: Controller (Perception 리뷰).
공통 규칙:
- 명령 주체는 하나 (바퀴 = drive_node, 팔 · 그리퍼 = manipulator_node)
- /cmd_vel 계열은 TwistStamped
- 인식 · 거리 값은 header.stamp (촬영 시각, ROS 시계) 300 ms 지나면 쓰지 않음
- 200 ms (펌웨어 명령 무수신 정지) 와 300 ms (값의 최대 나이) 는 다른 규칙
- 이름 · QoS 는 코드에 직접 쓰지 않고 `robot_interfaces/names.hpp` 의 `names::` · `qos::` 상수를 쓴다 (이 표와 같이 고침)
- perception 은 쓸 수 있는 프레임이 없으면 `/perception/result` 를 **발행하지 않는다** (카메라 정지). 받는 쪽은 300 ms 규칙으로 "모름" 처리

## 토픽

| 토픽 | 타입 | QoS | Publisher | Subscriber |
|---|---|---|---|---|
| /scan_front · /scan_rear | sensor_msgs/LaserScan | SensorData (intra) | LiDAR 드라이버 | scan_merger |
| /scan | sensor_msgs/LaserScan | SensorData | scan_merger | slam_toolbox, nav2, drive_node |
| /perception/result | robot_interfaces/PerceptionResult | Reliable, depth 1 | perception_node | mission, drive, manipulator |
| /arm_range | sensor_msgs/Range | SensorData | tfmini_node | drive, manipulator |
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
| /tf · /tf_static | tf2_msgs/TFMessage | 기본 · TransientLocal | slam_toolbox · ekf · robot_state_publisher | 전체 |
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

## USB 패킷 (opencr_protocol.hpp = opencr_firmware/opencr_protocol.h)

- 공통 프레임: `0xFF 0xFD · TYPE · LEN · PAYLOAD · CRC16(Dynamixel 2.0, 리틀엔디안)`
- 바퀴: **4륜 확정** (ID 1~4, 슬롯 0~3). 패킷은 6자리를 유지하고 슬롯 4 · 5 는 0.
- CMD 0x01 (Jetson → OpenCR, 100 Hz, 54 B):
  - `wheel_vel[6]` · `arm_pos[6]` · `gripper_pos` · `flags` · `seq`
  - flags: bit0 TORQUE_ON · bit1 ESTOP · bit2 GRIPPER_VALID
- STATE 0x81 (OpenCR → Jetson, 100 Hz, 191 B):
  - `pos[12]` · `vel[12]` · `effort[12]` · `quat[4]` · `gyro[3]` · `accel[3]` · `battery_v` · `error` · `fw_version` · `seq`
  - error 비트: 0x01 SyncRead 누락 · 0x02 명령 타임아웃 · 0x04 물리 E-stop · 0x08 모터 HW · 0x10 모터 설정 (부팅 점검 실패)
- `PROTOCOL_VERSION = 3`: `fw_version` 이 다르면 opencr_hardware 가 활성화를 거부 (펌웨어 다시 올림)
- 부팅 점검 (펌웨어): ping · 모델 번호 · Homing Offset 0 확인, Return Delay 0 기록. 하나라도 실패면 0x10 → 토크 켜기 거부
- Bus Watchdog 500 ms (25 단위): 토크 켤 때 모든 모터에 설정 — OpenCR 이 멈춰도 모터가 스스로 정지
- 시리얼은 `flock` + `TIOCEXCL` 로 한 프로세스만 연다
