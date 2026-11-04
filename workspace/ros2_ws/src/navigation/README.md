# navigation

주행(Nav2) 패키지다. 지도 위 목표 자세까지 경로를 계획하고(Smac 2D) 따라간다(RotationShim + MPPI). 최종 속도 명령은 `/cmd_vel_nav`(twist_mux `nav` 입력) 하나다.
위치 추정(AMCL)은 `localization` launch 를 `use_amcl_node:=true` 로 불러 띄운다. AMCL 설정은 `localization/config/amcl.yaml`.

## 실행

```bash
ros2 launch navigation navigation.launch.py                                     # 실물: AMCL 포함, 출력 /cmd_vel_nav
ros2 launch navigation navigation.launch.py use_localization:=false            # map → odom · /map 을 다른 곳에서 낼 때
ros2 launch navigation navigation.launch.py use_collision_monitor:=false       # smoother 가 바로 cmd_vel_out 으로
```

- 먼저 떠 있어야 하는 것: `bringup`(robot_state_publisher, EKF `odom → base_link` · `/odometry/filtered`, twist_mux), 라이다 `/scan_front` · `/scan_rear`.
- localization 인자(`start_x/y/yaw`, `map`, `use_loc_monitor` 등)는 이 launch 에 그대로 준다. 출발 자세 기본값 = base_link (0.69365, 0.95, π).
- 목표 보내기: `drive_node` 가 `navigate_to_pose` · `spin` action 을 부른다. 손으로는
  `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.734, y: 0.2915}, orientation: {w: 1.0}}}}"`

### Gazebo 시험 (waffle_4wd_description)

`waffle_4wd_description` 은 `ros2_ws/src` 밖에 있으므로 한 번 같이 빌드한다.

```bash
cd workspace/ros2_ws
colcon build --symlink-install --base-paths src ../urdf_and_usd/waffle_4wd_description --packages-select waffle_4wd_description robot_interfaces mission_planner localization navigation
source install/setup.bash
ros2 launch navigation gazebo_sim.launch.py               # 출발 자세로 Gazebo → 팔 주행 자세 → 5 s 뒤 Nav2 + AMCL
ros2 launch navigation gazebo_sim.launch.py gui:=false    # 창 없이. nav:=false 면 Gazebo · 팔 자세만, x:= y:= yaw:= 로 다른 시작 자세
```

- `lifecycle_manager_navigation ... Managed nodes are active` 가 나오면 목표를 보낸다.
- 지도 기본값은 Gazebo 경기장과 같은 SDF 에서 만든 `mission_planner/maps` (`/map` = `map_amcl`, `/map_objects` = `map_nav`, 0.025 m).
  localization 기본 Isaac SLAM 지도(`slam_walls_001`)는 동쪽으로 갈수록 SDF 와 어긋나(x 3 ~ 4.5 m 에서 8 ~ 9 cm, 북동 구석 15 cm 이상)
  AMCL 이 한쪽으로 치우치다 미로 칸을 착각해 1 m 넘어갔다. `map:=` · `objects_map:=` 로 바꿀 수 있다.
- Gazebo 팔 관절은 0 (영 자세) 으로 시작한다. 팀 기본 시작 자세는 주행 자세(`isaac/robot_params.json` `drive`)라서 launch 가 `/arm/<관절>/cmd_pos` 로 보낸다. 영 자세로 달리면 팔이 걸린다.
- Gazebo 는 twist_mux · ros2_control · EKF 를 거치지 않는다(diff_drive 플러그인이 `/cmd_vel` 을 받고 `odom → base_link` 를 냄). 그래서 `cmd_vel_out:=/cmd_vel` · `odom_topic:=/odom`.
- `use_loc_monitor:=false`: Gazebo 를 오래 켜 둔 뒤 띄우면 loc_monitor 가 시작하자마자 LOST(gap) → 전역 탐색으로 엉뚱한 자세를 후보로 잡는다(2026-11-04).
- Gazebo 는 카메라 3 대 렌더링으로 실시간의 약 0.5 ~ 0.66 배로 돈다. 모든 노드 `use_sim_time:=true`.

## 속도 명령 · TF

```
controller_server (RotationShim → MPPI) ─┐
behavior_server (spin · backup · wait) ──┴→ /nav2/cmd_vel_raw → velocity_smoother → /nav2/cmd_vel_smoothed
                                            → collision_monitor → cmd_vel_out (기본 /cmd_vel_nav → twist_mux)
```

- Nav2 기본 launch(nav2_bringup)는 중간 출력을 `cmd_vel_nav` 로 쓴다. 그대로 쓰면 twist_mux 가 smoother · collision_monitor 를 건너뛴 명령을 받으므로 이 launch 를 쓴다.
- 속도 명령은 TwistStamped(Nav2 1.5 기본 `enable_stamped_cmd_vel: true`, twist_mux `use_stamped: true`).

| TF | 내는 노드 |
|---|---|
| `map → odom` | `amcl` (localization launch, `use_amcl_node:=true`) |
| `odom → base_link` | 실물 = 팀 EKF, Gazebo = diff_drive 플러그인 |

## 노드 · 설정 (`config/nav2_params.yaml`, ★ = 임시값)

| 노드 | 설정 | 비고 |
|---|---|---|
| `planner_server` | Smac 2D (0.01 지도를 0.02 로 줄여 계산), `allow_unknown: false` | 시작 때 `Inflation layer ... not set sufficiently` ERROR 는 Smac 2D 가 늘 내는 알림 |
| `controller_server` | RotationShim (경로 방향과 45° 넘게 다르면 먼저 제자리 회전, 도착 뒤 목표 방향 회전) → MPPI(DiffDrive) | Nav2 1.4+ 는 MPPI 값을 `FollowPath.primary_controller` 아래에 둔다 |
| | vx −0.1 ~ 0.2 ★ · wz ±1.0 ★ · 가속 1.5, 도착 xy 0.05 ★ · yaw 0.1 ★ | drive.yaml `navigate.xy_goal_tolerance` 는 0.025 (Controller 확인 ★) |
| `local_costmap` · `global_costmap` | 정적 층 2 장(`/map` 벽, `/map_objects` 라이다에 안 보이는 물체) + `/scan_merged` + inflation 0.30 ★ | footprint = B안 외곽(앞 0.1328 · 뒤 −0.2135 · 반폭 0.1676) ★ |
| `behavior_server` | spin · backup · wait (기본 BT 와 drive_node 가 쓰는 것) | 최소 회전 0.3 rad/s ★ |
| `velocity_smoother` · `collision_monitor` | MPPI 와 같은 한계 / 차체가 1.2 s 안에 닿을 속도면 줄임 | |
| `bt_navigator` | 기본 BT `navigate_to_pose_w_replanning_and_recovery.xml` | 플러그인 이름(GridBased · FollowPath · general_goal_checker · progress_checker · PathHandler)을 바꾸지 말 것 |

Gazebo 시험(2026-11-04)에서 기본값에서 바꾼 것과 이유:

| 설정 | 값 | 기본값으로 생긴 일 |
|---|---|---|
| costmap `use_maximum` | true (costmap 단위) | Nav2 1.5 는 정적 층마다 받지 않는다. 덮어쓰기라 objects 지도의 "모름"이 벽 · 빈칸을 덮어 목표가 막힘으로 계산됨 |
| local `track_unknown_space` | false | 움직이는 창에서는 큰 값으로 합칠 때 "모름"(255)이 이김 |
| MPPI `CostCritic.near_collision_cost` | 254 | 미로 통로(약 0.53 m)는 차체(폭 0.355) 가장자리가 inscribed(장애물 0.14 m 안)에 닿을 수밖에 없어 멈춤 |
| MPPI Path · GoalAngle critic `threshold_to_consider` | 0.15 | 목표 0.5 m 안에서 경로 critic 이 꺼지고 방향 벌점이 켜져, 옆에 있는 목표 앞에서 멈춤 |
| goal checker `xy_goal_tolerance` | 0.05 | 마지막 옆 2 ~ 3 cm 를 차동 구동으로 못 메워 실패 |
| RotationShim | 추가 | 도착 근처에서 MPPI 가 돌다 밀려나(스키드 회전 중심 ≠ base_link) 다시 멈춤 |
| 계획기 `downsample_costmap` | false | 0.025 지도를 0.05 로 줄여 거칠게 계획 |

결과 (Gazebo, SDF 지도, `mission_planner/config/approach_poses.yaml` 의 stage 12 곳 차례로): **12/12 성공**, 실패 · 복구 0 회, sim 204 s.
AMCL 오차 주행 중 평균 1.9 cm · 95 % 3.4 cm · 최대 6.1 cm. 끝 위치 실제 오차 평균 7.5 cm · 최대 11.9 cm (도착 기준 5 cm + 도착 뒤 회전 밀림), 방향 평균 1.2°.
Isaac SLAM 지도로는 3 번째 목표에서 AMCL 이 위치를 잃어 2/12.

## 확인할 것 ★

- 차체 외곽(footprint): B안 값. control `front_offset` 0.263 · perception `self_box` 와 다름 → 실측으로 하나로.
- 회전 중심: 지금은 base_link. #49(−0.079) · #92(−0.0658) 실측 뒤 회전 중심 프레임을 쓸지 정함.
- 속도(vx 0.2 · wz 1.0) · inflation · 도착 기준은 실물에서 다시 맞춤.
- 지도: localization 기본 `/map` 은 Isaac 모의 미로 SLAM 지도이고 동쪽이 8 ~ 15 cm 어긋난다(위). 실물은 도면 지도(`mission_planner/maps`)로 먼저 시험하고, 다시 SLAM 하면 루프 닫힘 · 벽 위치를 도면과 비교한다.
