cmake_minimum_required(VERSION 3.20)

project(bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# lv2_module5/config 의 설정을 share/bringup/config 로 설치 (launch 가 이 경로를 읽음)
install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/center.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/fastdds_shm_big.xml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
