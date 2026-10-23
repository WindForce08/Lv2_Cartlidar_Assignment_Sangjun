// 통신 이름 · QoS 한 곳 정의 (Xmen tracking_common/interface.py 방식)
//   노드 코드에 토픽 이름 · QoS 를 직접 쓰지 않고 여기 값을 쓴다. 바꿀 때는 이 파일과 docs/interfaces.md 만 고친다.
#pragma once

#include "rclcpp/qos.hpp"

namespace robot_interfaces
{
namespace names
{
// 토픽
inline constexpr const char * PERCEPTION_RESULT = "/perception/result";
inline constexpr const char * PERCEPTION_PUCKS = "/perception/pucks";
inline constexpr const char * PERCEPTION_TASK_MARKER = "/perception/task_marker";
inline constexpr const char * PERCEPTION_SIGNAL = "/perception/signal";
inline constexpr const char * ARM_RANGE = "/arm_range";
inline constexpr const char * SCAN = "/scan";
inline constexpr const char * ODOM_FILTERED = "/odometry/filtered";
inline constexpr const char * CMD_VEL_DRIVE = "/cmd_vel_drive";
inline constexpr const char * ARM_STREAM = "/arm_controller/joint_trajectory";
inline constexpr const char * JOINT_STATES = "/joint_states";
inline constexpr const char * JOY = "/joy";
inline constexpr const char * E_STOP = "/e_stop";
inline constexpr const char * DIAGNOSTICS = "/diagnostics";
inline constexpr const char * MISSION_STATUS = "/mission/status";
inline constexpr const char * ARM_SKILL_STATE = "/arm/skill_state";
inline constexpr const char * ARM_SAFETY_STATE = "/arm/safety_state";
// Action
inline constexpr const char * DRIVE_NAVIGATE = "/drive/navigate";
inline constexpr const char * DRIVE_ALIGN = "/drive/align";
inline constexpr const char * DRIVE_SKILL = "/drive/execute_skill";
inline constexpr const char * MANIP_POSE = "/manipulator/pose";
inline constexpr const char * MANIP_PICK = "/manipulator/pick";
inline constexpr const char * MANIP_SKILL = "/manipulator/execute_skill";
inline constexpr const char * ARM_TRAJ_ACTION = "/arm_controller/follow_joint_trajectory";
inline constexpr const char * GRIPPER_ACTION = "/gripper_controller/gripper_cmd";
// Service
inline constexpr const char * PERCEPTION_SET_MODE = "/perception/set_mode";
inline constexpr const char * PERCEPTION_CHECK_DICE = "/perception/check_dice";
inline constexpr const char * MISSION_START = "/mission/start";
inline constexpr const char * MISSION_STOP = "/mission/stop";
}  // namespace names

namespace qos
{
// /perception/result: Reliable, depth 1 (최신 하나만, 받는 쪽은 stamp 300 ms 규칙)
inline rclcpp::QoS result() {return rclcpp::QoS(1).reliable();}
// /arm_range · /scan: SensorData (BestEffort)
inline rclcpp::QoS sensor() {return rclcpp::SensorDataQoS();}
// /e_stop: Reliable + TransientLocal, depth 1 (늦게 켠 노드도 현재 값을 받음)
inline rclcpp::QoS e_stop() {return rclcpp::QoS(1).reliable().transient_local();}
// 상태 모니터링 토픽 (/arm/*_state): 늦게 켠 모니터도 마지막 상태를 받음
inline rclcpp::QoS latched_state() {return rclcpp::QoS(10).transient_local();}
}  // namespace qos

// 300 ms 규칙 (인식 · 거리 값의 최대 나이) — 200 ms(펌웨어 명령 무수신 정지)와 다른 규칙
inline constexpr double MAX_VALUE_AGE_S = 0.3;
}  // namespace robot_interfaces
