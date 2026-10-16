cmake_minimum_required(VERSION 3.20)

project(center)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(
  center_node
  src/center_node.cpp
)

target_link_libraries(
  center_node
  rclcpp::rclcpp
  std_msgs::std_msgs__rosidl_typesupport_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
)

install(
  TARGETS center_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()