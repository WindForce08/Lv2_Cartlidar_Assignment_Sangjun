-- Cartographer 순수 위치 추정 설정(launch/localization.launch.py localization:=carto 가 씀).
-- 저장 지도(maps/carto_map.pbstream)를 -load_frozen_state 로 읽고, 새 궤적을 nodes carto_start 가 출발 자세에서 시작시킨다.
-- 바꾼 점(Cartographer 예제 turtlebot3 lds_2d.lua 기준):
--   map_frame carto_map — 팀 map(발제 좌표)과 이름이 겹치지 않게. map → carto_map 정적 TF 는 launch 가 냄(지도 맞춤 값).
--   tracking_frame · published_frame base_link, provide_odom_frame false, publish_to_tf false, publish_tracked_pose true,
--   use_pose_extrapolator false → Cartographer 는 TF 를 내지 않고 /tracked_pose(carto_map → base_link, 스캔 시각)만 낸다.
--   map → odom 은 map_odom_relay 가 같은 시각의 odom → base_link(팀 EKF)로 계산해 낸다.
--   num_laser_scans 2(앞 · 뒤 라이다, scan_body_mask 가 차체 가림을 지운 *_filtered), use_odometry true(EKF /odometry/filtered 를 사전값으로).
--   아래 '모의에서 고른 값' 은 Isaac 모의 비교로 정함.
-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- /* Author: Darby Lim */

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "carto_map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  use_pose_extrapolator = false,
  publish_to_tf = false,
  publish_tracked_pose = true,
  publish_frame_projected_to_2d = true,
  use_odometry = true,   -- EKF /odometry/filtered(launch 가 odom 을 remap)
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 2,   -- 앞 · 뒤 라이다
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)  -- 원본 math.rad(0.1)

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- POSE_GRAPH.optimize_every_n_nodes = 0

-- ---- 모의에서 고른 값(Cartographer 기본값은 주석) ----
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.025  -- 기본값 0.05
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0  -- 기본값 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0  -- 기본값 40.

-- ---- 순수 위치 추정 ----
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,   -- 새 궤적의 submap 은 최근 3 개만(공식 예제 backpack_2d_localization.lua 와 같음)
}
POSE_GRAPH.optimize_every_n_nodes = 20   -- 저장 지도에 대한 보정은 최적화 때만 반영되므로 공식 예제처럼 자주

-- 지도 전체 탐색 끔 · 지역 탐색 창 좁힘: 초기 자세를 줘도 새 궤적은 저장 궤적과 '연결'되지 않은 상태라 지도 전체 탐색만 해서
-- 미로의 비슷한 자리와 잘못 맞는 일이 있었다(모의에서 yaw π 뒤집힘). 아래 값이면 늘 현재 추정 자세 둘레 창만 탐색한다.
-- 노드 시각은 0001 년 기준 약 6.4e10 s 라 1e11 로 둔다. 위치를 잃었을 때 다시 찾는 일은 loc_monitor 가 맡는다.
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e11
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.0  -- 기본 7 m. 초기 자세가 cm 수준이라 넓은 창은 잘못 맞을 위험만 키움

return options
