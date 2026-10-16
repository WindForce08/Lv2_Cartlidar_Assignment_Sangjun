cmake_minimum_required(VERSION 3.16)
project(perception LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)

# yaml-cpp 0.8 은 yaml-cpp::yaml-cpp, 이전 버전은 yaml-cpp 타깃 제공
if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(YAML_CPP_TARGET yaml-cpp)
endif()

# 검출 라이브러리: 설정·검출·오버레이 (ROS 의존 없음)
add_library(perception_core STATIC
  src/detector.cpp
)
set_target_properties(perception_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(perception_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(perception_core
  PUBLIC ${OpenCV_LIBS}
  PRIVATE ${YAML_CPP_TARGET}
)

# ROS2 노드: color 구독 -> /target 발행
add_executable(perception_node src/perception_node.cpp)
target_link_libraries(perception_node
  perception_core
  rclcpp::rclcpp
  cv_bridge::cv_bridge
  ament_index_cpp::ament_index_cpp
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)

install(TARGETS perception_node DESTINATION lib/${PROJECT_NAME})

# 로컬 전용 도구 (저장소에 올리지 않음): 파일이 있을 때만 빌드
#   batch_detect : 저장 이미지·영상 일괄 검출 -> 원본·마스크·오버레이·CSV
set(BATCH_DETECT_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/src/batch_detect.cpp)
if(EXISTS ${BATCH_DETECT_SOURCE})
  add_executable(batch_detect ${BATCH_DETECT_SOURCE})
  target_link_libraries(batch_detect perception_core)
  install(TARGETS batch_detect DESTINATION lib/${PROJECT_NAME})
endif()
install(TARGETS perception_core EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# 팀 설정 파일(lv2_module5/config/perception.yaml)을 노드 기본 설정으로 함께 설치
# colcon build --symlink-install 이면 원본 링크라서 yaml 수정이 바로 반영됨
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/perception.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# 로컬 전용 시험 (저장소에 올리지 않음): 시험 파일과 batch_detect 가 있을 때만 빌드
set(TEST_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/test/test_perception.cpp)
if(BUILD_TESTING AND EXISTS ${TEST_SOURCE} AND EXISTS ${BATCH_DETECT_SOURCE})
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_perception ${TEST_SOURCE} ${BATCH_DETECT_SOURCE})
  target_link_libraries(test_perception perception_core)
  target_compile_definitions(test_perception PRIVATE
    PERCEPTION_BATCH_NO_MAIN
    # 팀 설정 파일(lv2_module5/config/perception.yaml)로 시험
    PERCEPTION_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/perception.yaml"
  )
endif()

# 다른 패키지에서 find_package(perception) 후 perception::perception_core 로 사용 가능
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV yaml-cpp)
ament_package()
