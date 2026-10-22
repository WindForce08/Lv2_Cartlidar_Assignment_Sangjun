cmake_minimum_required(VERSION 3.16)
project(robot_policy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

# 선택: ONNX Runtime (CPU). Jetson: onnxruntime C/C++ 패키지 설치 후 ONNXRUNTIME_ROOT 지정 가능
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  PATHS ${ONNXRUNTIME_ROOT}/include /usr/include/onnxruntime /usr/local/include/onnxruntime /usr/local/include)
find_library(ONNXRUNTIME_LIB onnxruntime PATHS ${ONNXRUNTIME_ROOT}/lib /usr/lib /usr/local/lib)

add_library(robot_policy SHARED src/policy.cpp src/onnx_runner.cpp)
target_include_directories(robot_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(robot_policy PUBLIC yaml-cpp::yaml-cpp)
if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIB)
  message(STATUS "robot_policy: ONNX Runtime 사용 (${ONNXRUNTIME_LIB})")
  target_compile_definitions(robot_policy PUBLIC USE_ONNXRUNTIME)
  target_include_directories(robot_policy PUBLIC ${ONNXRUNTIME_INCLUDE_DIR})
  target_link_libraries(robot_policy PUBLIC ${ONNXRUNTIME_LIB})
else()
  message(WARNING "robot_policy: ONNX Runtime 없음 — 정책 로드 불가 (skill.enabled=false 로 운용, mission fallback)")
endif()

install(TARGETS robot_policy EXPORT export_robot_policy
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_policy test/test_policy.cpp)
  target_link_libraries(test_policy robot_policy)
  target_compile_definitions(test_policy PRIVATE POLICIES_YAML="${CMAKE_CURRENT_SOURCE_DIR}/config/policies.yaml")
endif()

ament_export_targets(export_robot_policy HAS_LIBRARY_TARGET)
ament_export_dependencies(yaml_cpp_vendor yaml-cpp)
ament_package()
