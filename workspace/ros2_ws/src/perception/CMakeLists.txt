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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBAR REQUIRED zbar)
# LAPACK: 바코드 평면 맞춤의 고유값 분해를 numpy(np.linalg.eigh)와 같은 ssyevd 로
find_package(LAPACK REQUIRED)
# AprilTag: OpenCV ≥ 4.7 은 objdetect, 4.6 은 contrib aruco
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio calib3d dnn OPTIONAL_COMPONENTS aruco objdetect)

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

# 바코드 (R3): D435 라벨 찾기 · 팔 카메라 zbar 읽기 (perception_node · 시험 · 비교 도구가 같이 씀)
add_library(perception_barcode STATIC src/barcode_locator.cpp src/barcode_reader.cpp)
set_target_properties(perception_barcode PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(perception_barcode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(perception_barcode PRIVATE ${ZBAR_INCLUDE_DIRS})
target_link_libraries(perception_barcode PUBLIC ${OpenCV_LIBS} ${ZBAR_LIBRARIES} ${LAPACK_LIBRARIES})

# barcode_compare: 렌더 장면에서 찾기 · 읽기 결과를 JSON 으로 (scripts/barcode_compare.py 가 기준 Python 과 견줌)
add_executable(barcode_compare src/barcode_compare_main.cpp)
target_link_libraries(barcode_compare perception_barcode)

# perception_node: 카메라 3대(영상 토픽) → /perception/result
add_library(perception_node SHARED src/perception_node.cpp)
target_link_libraries(perception_node PUBLIC perception_detector perception_barcode ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp rclcpp_components::component ${robot_interfaces_TARGETS} ${sensor_msgs_TARGETS} ${geometry_msgs_TARGETS}
  cv_bridge::cv_bridge tf2::tf2 tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs)
rclcpp_components_register_node(perception_node PLUGIN "perception::PerceptionNode" EXECUTABLE perception)

# 라이다 노드들 (ROS 의존 없는 계산부는 include/perception/*.hpp, 단위 시험 대상)
add_library(perception_lidar SHARED src/scan_merger_node.cpp src/obstacle_stop_node.cpp src/tfmini_node.cpp)
target_include_directories(perception_lidar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(perception_lidar PUBLIC rclcpp::rclcpp rclcpp_components::component
  ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} tf2::tf2 tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs)
rclcpp_components_register_node(perception_lidar PLUGIN "perception::ScanMergerNode" EXECUTABLE scan_merger)
rclcpp_components_register_node(perception_lidar PLUGIN "perception::ObstacleStopNode" EXECUTABLE obstacle_stop)
rclcpp_components_register_node(perception_lidar PLUGIN "perception::TFminiNode" EXECUTABLE tfmini)

install(TARGETS perception_detector perception_barcode perception_node perception_lidar
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS barcode_compare RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch udev data DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_perception_logic test/test_tfmini.cpp test/test_vision.cpp test/test_tracker_tag.cpp
    test/test_inputs.cpp test/test_lidar.cpp test/test_puck.cpp test/test_signal.cpp test/test_dice.cpp
    test/test_signal_light.cpp test/test_depth_split.cpp
    test/test_tag_upright.cpp test/test_barcode.cpp test/test_puck_merge.cpp test/test_read_tag.cpp test/test_arm_lock.cpp)
  target_link_libraries(test_perception_logic perception_detector perception_barcode)
  target_compile_definitions(test_perception_logic PRIVATE
    PERCEPTION_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data" PERCEPTION_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
endif()

ament_package()
