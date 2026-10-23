cmake_minimum_required(VERSION 3.16)
project(robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PerceptionResult.msg"
  "msg/SkillState.msg"
  "msg/SafetyState.msg"
  "msg/MissionStatus.msg"
  "msg/Puck.msg"
  "msg/PuckArray.msg"
  "msg/TaskMarker.msg"
  "srv/SetPerceptionMode.srv"
  "action/AlignBase.action"
  "action/ArmPose.action"
  "action/Pick.action"
  "action/ExecuteSkill.action"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

# 통신 이름 · QoS 헤더 (names.hpp) — 생성 헤더와 같은 include 경로
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
