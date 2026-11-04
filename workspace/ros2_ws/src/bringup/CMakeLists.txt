cmake_minimum_required(VERSION 3.16)
project(bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/odom_icr.py DESTINATION lib/${PROJECT_NAME})   # #92 회전 중심 옆 이동 -> EKF

ament_package()
