cmake_minimum_required(VERSION 3.8)
project(waffle_4wd_description)

find_package(ament_cmake REQUIRED)

# Install package data
install(DIRECTORY urdf meshes launch rviz config worlds
  DESTINATION share/${PROJECT_NAME}
)
# Gazebo 보조 노드: TFmini 광선 -> sensor_msgs/Range, camera_info 기준선 고치기, 키보드 조작
install(PROGRAMS scripts/tfmini_scan_to_range.py scripts/camera_info_fix.py scripts/keyboard_teleop.py scripts/slider_to_gazebo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
