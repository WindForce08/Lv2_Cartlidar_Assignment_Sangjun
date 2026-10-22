cmake_minimum_required(VERSION 3.16)
project(center)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(control REQUIRED)            # control/fsm.hpp (상태머신 공용 헤더)
find_package(nav2_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 판단: MissionNode (⑧ control_container 에 로드, 단독 실행 파일 mission 도 생성)
add_library(mission_node SHARED src/mission_node.cpp)
target_include_directories(mission_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(mission_node PRIVATE ${control_INCLUDE_DIRS})
target_link_libraries(mission_node PUBLIC
  rclcpp::rclcpp rclcpp_action::rclcpp_action rclcpp_components::component
  ${robot_interfaces_TARGETS} ${nav2_msgs_TARGETS} ${diagnostic_msgs_TARGETS} ${std_msgs_TARGETS} ${std_srvs_TARGETS}
  tf2::tf2 tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs)
rclcpp_components_register_node(mission_node PLUGIN "center::MissionNode" EXECUTABLE mission)

install(TARGETS mission_node ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mission_table test/test_mission_table.cpp)
  target_include_directories(test_mission_table PRIVATE include)
endif()

ament_package()
