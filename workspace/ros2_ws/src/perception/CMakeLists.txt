cmake_minimum_required(VERSION 3.16)
project(perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# AprilTag: OpenCV ≥ 4.7 은 objdetect, 4.6 은 contrib aruco
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio calib3d dnn OPTIONAL_COMPONENTS aruco objdetect)

# 선택: librealsense2 (D435 × 2) — Jetson 에서는 설치되어 있어야 SEARCH_OBJECT 사용 가능
option(WITH_REALSENSE "librealsense2 사용" ON)
if(WITH_REALSENSE)
  find_package(realsense2 QUIET)
endif()
# 선택: TensorRT + CUDA (검출 엔진)
option(WITH_TENSORRT "TensorRT 검출기 사용" ON)
if(WITH_TENSORRT)
  find_package(CUDAToolkit QUIET)
  find_path(TENSORRT_INCLUDE_DIR NvInfer.h PATHS /usr/include/aarch64-linux-gnu /usr/include/x86_64-linux-gnu /usr/local/TensorRT/include)
  find_library(TENSORRT_NVINFER nvinfer PATHS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/local/TensorRT/lib)
endif()

# 검출기 (perception_node 와 시험이 같이 씀)
add_library(perception_detector STATIC src/color_detector.cpp src/trt_detector.cpp src/tag_detector.cpp)
set_target_properties(perception_detector PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(perception_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(perception_detector PUBLIC ${OpenCV_LIBS})
if(WITH_TENSORRT AND CUDAToolkit_FOUND AND TENSORRT_INCLUDE_DIR AND TENSORRT_NVINFER)
  message(STATUS "perception: TensorRT 사용 (${TENSORRT_NVINFER})")
  target_compile_definitions(perception_detector PUBLIC USE_TENSORRT)
  target_include_directories(perception_detector PUBLIC ${TENSORRT_INCLUDE_DIR})
  target_link_libraries(perception_detector PUBLIC ${TENSORRT_NVINFER} CUDA::cudart)
else()
  message(WARNING "perception: TensorRT/CUDA 없음 — ColorDetector 만 빌드")
endif()

# perception_node (⑦ perception_container)
add_library(perception_node SHARED src/perception_node.cpp)
target_link_libraries(perception_node PUBLIC perception_detector
  rclcpp::rclcpp rclcpp_components::component ${robot_interfaces_TARGETS} ${geometry_msgs_TARGETS}
  tf2::tf2 tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs)
if(WITH_REALSENSE AND realsense2_FOUND)
  message(STATUS "perception: librealsense2 ${realsense2_VERSION} 사용")
  target_compile_definitions(perception_node PRIVATE USE_REALSENSE)
  target_link_libraries(perception_node PUBLIC realsense2::realsense2)
else()
  message(WARNING "perception: librealsense2 없음 — SEARCH_OBJECT(D435) 비활성")
endif()
rclcpp_components_register_node(perception_node PLUGIN "perception::PerceptionNode" EXECUTABLE perception)

# tfmini_node (⑧ control_container 에서 실행)
add_library(tfmini_node SHARED src/tfmini_node.cpp)
target_include_directories(tfmini_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(tfmini_node PUBLIC rclcpp::rclcpp rclcpp_components::component ${sensor_msgs_TARGETS} ${robot_interfaces_TARGETS})
rclcpp_components_register_node(tfmini_node PLUGIN "perception::TFminiNode" EXECUTABLE tfmini)

install(TARGETS perception_detector perception_node tfmini_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch udev DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_perception_logic test/test_tfmini.cpp test/test_vision.cpp test/test_tracker_tag.cpp)
  target_link_libraries(test_perception_logic perception_detector)
endif()

ament_package()
