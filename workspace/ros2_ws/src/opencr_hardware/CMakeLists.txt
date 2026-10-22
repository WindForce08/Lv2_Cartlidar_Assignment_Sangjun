cmake_minimum_required(VERSION 3.16)
project(opencr_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)   # names.hpp (통신 이름 · QoS)

add_library(${PROJECT_NAME} SHARED src/opencr_hardware.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} PUBLIC
  hardware_interface::hardware_interface pluginlib::pluginlib rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle ${std_msgs_TARGETS} ${sensor_msgs_TARGETS} ${diagnostic_msgs_TARGETS} ${robot_interfaces_TARGETS})

pluginlib_export_plugin_description_file(hardware_interface opencr_hardware.xml)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config urdf launch udev DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp test/test_estop_latch.cpp)
  target_include_directories(test_protocol PRIVATE include)
  target_compile_definitions(test_protocol PRIVATE
    FIRMWARE_PROTOCOL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../opencr_firmware/opencr_protocol.h"
    HOST_PROTOCOL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/include/opencr_hardware/opencr_protocol.hpp")

  # 가상 시리얼로 펌웨어를 흉내 내고 ResourceManager 로 플러그인을 실제처럼 로드
  ament_add_gtest(test_hardware_loopback test/test_hardware_loopback.cpp SKIP_LINKING_MAIN_LIBRARIES TIMEOUT 60)
  target_include_directories(test_hardware_loopback PRIVATE include)
  target_link_libraries(test_hardware_loopback gtest hardware_interface::hardware_interface rclcpp::rclcpp)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle std_msgs sensor_msgs diagnostic_msgs robot_interfaces)
ament_package()
