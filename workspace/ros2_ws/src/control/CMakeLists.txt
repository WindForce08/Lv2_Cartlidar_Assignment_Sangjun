cmake_minimum_required(VERSION 3.16)
project(control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(robot_policy REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# 순수 C++ (ROS 없음): IK
add_library(arm_ik STATIC src/arm_ik.cpp)
set_target_properties(arm_ik PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(arm_ik PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(arm_ik PUBLIC Eigen3::Eigen)

# 노드마다 공유 라이브러리 + component 등록 (⑧ control_container 에 로드, 단독 실행 파일도 생성)
macro(robot_component lib cls exe)
  add_library(${lib} SHARED src/${lib}.cpp)
  target_include_directories(${lib} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  target_link_libraries(${lib} PUBLIC
    rclcpp::rclcpp rclcpp_action::rclcpp_action rclcpp_components::component
    ${robot_interfaces_TARGETS} ${nav2_msgs_TARGETS} ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS} ${std_srvs_TARGETS} ${control_msgs_TARGETS} ${trajectory_msgs_TARGETS}
    ${diagnostic_msgs_TARGETS} ${nav_msgs_TARGETS} tf2::tf2 tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs)
  rclcpp_components_register_node(${lib} PLUGIN "${cls}" EXECUTABLE ${exe})
  install(TARGETS ${lib} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
endmacro()

robot_component(drive_node control::DriveNode drive)
robot_component(manipulator_node control::ManipulatorNode manipulator)
target_link_libraries(manipulator_node PUBLIC arm_ik robot_policy::robot_policy)
target_link_libraries(drive_node PUBLIC robot_policy::robot_policy)

install(TARGETS arm_ik ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control_logic test/test_fsm.cpp test/test_ik.cpp test/test_align.cpp)
  target_link_libraries(test_control_logic arm_ik)
endif()

ament_package()
