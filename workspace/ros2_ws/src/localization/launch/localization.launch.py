"""저장 지도 위 위치 추정: map → odom 과 위치 상태(/localization/status · /localization/lock), 주행용 지도(/map · /map_objects).

  ros2 launch localization localization.launch.py [use_sim_time:=true] [localization:=amcl|carto] [use_amcl_node:=true] [gate_in:=<토픽> gate_out:=<토픽>]

map = 발제 좌표(팀 map, mission_planner 문서 좌표 지도와 같은 원점). odom → base_link 는 팀 EKF(bringup ekf.yaml)가 낸다.
입력: /scan_front · /scan_rear(LaserScan), TF odom → base_link · base_link → laser_*, /odometry/filtered(EKF).
localization:=amcl(기본) — AMCL. AMCL 노드는 주행(Nav2) 쪽 launch 가 띄운다. 이 launch 는 그 입력(/scan_merged · /map)과 감시만 낸다.
  scan_body_mask → scan_merge(/scan_merged). use_amcl_node:=true 면 amcl 도 여기서 띄움(config/amcl.yaml, 단독 시험용).
localization:=carto — Cartographer 순수 위치 추정(기본으로는 안 뜸)
  scan_body_mask(차체 가림 지움 → /scan_*_filtered) → cartographer_node(maps/carto_map.pbstream 고정, map_frame carto_map)
  → carto_start(출발 자세에서 궤적 시작) → /tracked_pose → map_odom_relay(map → odom). 정적 TF map → carto_map(지도 맞춤 값).
loc_monitor(use_loc_monitor, 기본 true): 위치 잃음 감지 · 다시 찾기. 두 방식 모두 지원(다시 넣기만 다름).
  gate_in 을 주면 그 속도 명령을 받아 OK · SUSPECT 가 아닐 때 0 으로 바꿔 gate_out 으로 낸다(기본 none = 게이트 끔, /localization/lock 만 냄)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution, EqualsSubstitution, LaunchConfiguration as LC, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    me = FindPackageShare('localization')
    cfg = PathJoinSubstitution([me, 'config'])
    maps = lambda f: PathJoinSubstitution([me, 'maps', f])
    sim = {'use_sim_time': ParameterValue(LC('use_sim_time'), value_type=bool)}
    carto = IfCondition(EqualsSubstitution(LC('localization'), 'carto'))
    amcl = IfCondition(EqualsSubstitution(LC('localization'), 'amcl'))
    amcl_node = IfCondition(AndSubstitution(EqualsSubstitution(LC('localization'), 'amcl'), LC('use_amcl_node')))
    align = {'align_x': ParameterValue(LC('align_x'), value_type=float), 'align_y': ParameterValue(LC('align_y'), value_type=float),
             'align_yaw': ParameterValue(LC('align_yaw'), value_type=float)}
    start = {'start_x': ParameterValue(LC('start_x'), value_type=float), 'start_y': ParameterValue(LC('start_y'), value_type=float),
             'start_yaw': ParameterValue(LC('start_yaw'), value_type=float)}

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization', default_value='amcl', description='amcl(기본) | carto'),
        DeclareLaunchArgument('use_amcl_node', default_value='false',
                              description='true 면 amcl 노드도 여기서 띄움(단독 시험용). 기본은 주행(Nav2) 쪽 launch 가 띄움'),
        DeclareLaunchArgument('use_loc_monitor', default_value='true', description='위치 잃음 감지 · 다시 찾기'),
        DeclareLaunchArgument('use_map_server', default_value='true', description='/map · /map_objects 발행(주행 costmap 정적 층)'),
        DeclareLaunchArgument('map', default_value=maps('slam_walls_001.yaml'), description='/map — SLAM 벽 지도(발제 좌표, 0.01 m)'),
        DeclareLaunchArgument('objects_map', default_value=maps('objects.yaml'), description='/map_objects — 라이다에 안 보이는 고정 물체(발제 배치, 0.01 m)'),
        DeclareLaunchArgument('loc_map', default_value=maps('slam_walls.yaml'), description='loc_monitor 잔차 · 탐색용 벽 지도(0.025 m)'),
        DeclareLaunchArgument('carto_state', default_value=maps('carto_map.pbstream'), description='Cartographer 저장 지도'),
        DeclareLaunchArgument('carto_config_dir', default_value=cfg),
        DeclareLaunchArgument('carto_config', default_value='carto_localization.lua'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered', description='Cartographer odometry 입력(팀 EKF 출력)'),
        DeclareLaunchArgument('align_x', default_value='0.729', description='map(발제 좌표) ← carto_map 지도 맞춤 값(SLAM 지도 저장 때 잼)'),
        DeclareLaunchArgument('align_y', default_value='0.960'),
        DeclareLaunchArgument('align_yaw', default_value='3.143774'),
        DeclareLaunchArgument('start_x', default_value='0.69365', description='출발 자세(base_link, 발제 좌표) = 출발 구역 가운데 0.734 − 0.04035 (B안 외곽 가운데를 구역 가운데에, −X 봄). '
                                          'carto 궤적 시작 · use_amcl_node 때 AMCL 초기 자세'),
        DeclareLaunchArgument('start_y', default_value='0.950'),
        DeclareLaunchArgument('start_yaw', default_value='3.14159265'),
        DeclareLaunchArgument('future_dt', default_value='0.1', description='map_odom_relay 가 map → odom 을 앞당겨 낼 시간[s]'),
        DeclareLaunchArgument('mask_file', default_value='none', description='차체 가림 칸 yaml(none = 출발 전 0.5~1.9 s 정지 스캔에서 배움)'),
        DeclareLaunchArgument('amcl_params', default_value=PathJoinSubstitution([cfg, 'amcl.yaml'])),
        DeclareLaunchArgument('loc_res_mode', default_value='interp_p75'),
        DeclareLaunchArgument('loc_res_thr', default_value='0.034', description='잔차 문턱[m](interp_p75 기준, 모의에서 정함)'),
        DeclareLaunchArgument('loc_check_mode', default_value='pair'),
        DeclareLaunchArgument('loc_event_log', default_value='none', description='loc_monitor 사건 기록 jsonl(none = 안 씀)'),
        DeclareLaunchArgument('gate_in', default_value='none', description='loc_monitor 명령 게이트 입력(none = 끔)'),
        DeclareLaunchArgument('gate_out', default_value='/cmd_vel_nav'),
    ]
    map_servers = ['map_server', 'map_objects_server']
    return LaunchDescription(args + [
        Node(package='localization', executable='scan_body_mask', name='scan_body_mask', output='screen',
             parameters=[sim, {'mask_file': LC('mask_file')}]),
        # 주행용 지도
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen', condition=IfCondition(LC('use_map_server')),
             parameters=[sim, {'yaml_filename': LC('map'), 'topic_name': 'map', 'frame_id': 'map'}]),
        Node(package='nav2_map_server', executable='map_server', name='map_objects_server', output='screen', condition=IfCondition(LC('use_map_server')),
             parameters=[sim, {'yaml_filename': LC('objects_map'), 'topic_name': 'map_objects', 'frame_id': 'map'}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_maps', output='screen',
             condition=IfCondition(LC('use_map_server')), parameters=[sim, {'autostart': True, 'node_names': map_servers}]),
        # Cartographer
        Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node', output='screen', parameters=[sim], condition=carto,
             arguments=['-configuration_directory', LC('carto_config_dir'), '-configuration_basename', LC('carto_config'),
                        '-load_state_filename', LC('carto_state'), '-load_frozen_state=true', '-start_trajectory_with_default_topics=false'],
             remappings=[('scan_1', '/scan_front_filtered'), ('scan_2', '/scan_rear_filtered'), ('odom', LC('odom_topic'))]),
        Node(package='localization', executable='carto_start', name='carto_start', output='screen', condition=carto,
             parameters=[sim, align, start, {'carto_config_dir': LC('carto_config_dir'), 'carto_config': LC('carto_config')}]),
        Node(package='localization', executable='map_odom_relay', name='map_odom_relay', output='screen', condition=carto,
             parameters=[sim, align, {'map_frame': 'map', 'future_dt': ParameterValue(LC('future_dt'), value_type=float)}]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='carto_map_tf', output='screen', parameters=[sim], condition=carto,
             arguments=['--x', LC('align_x'), '--y', LC('align_y'), '--yaw', LC('align_yaw'), '--frame-id', 'map', '--child-frame-id', 'carto_map']),
        # AMCL
        Node(package='localization', executable='scan_merge', name='scan_merge', output='screen', condition=amcl, parameters=[sim]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', condition=amcl_node,
             parameters=[LC('amcl_params'), sim, {'initial_pose.x': ParameterValue(LC('start_x'), value_type=float),
                                                  'initial_pose.y': ParameterValue(LC('start_y'), value_type=float),
                                                  'initial_pose.yaw': ParameterValue(LC('start_yaw'), value_type=float)}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_amcl', output='screen', condition=amcl_node,
             parameters=[sim, {'autostart': True, 'node_names': ['amcl']}]),
        # 위치 잃음 감지 · 다시 찾기
        Node(package='localization', executable='loc_monitor', name='loc_monitor', output='screen', condition=IfCondition(LC('use_loc_monitor')),
             parameters=[sim, align, {'backend': LC('localization'), 'map': LC('loc_map'), 'mission_frame': 'map',
                                      'carto_config_dir': LC('carto_config_dir'), 'carto_config': LC('carto_config'),
                                      'res_mode': LC('loc_res_mode'), 'res_thr': ParameterValue(LC('loc_res_thr'), value_type=float),
                                      'check_mode': LC('loc_check_mode'), 'event_log': LC('loc_event_log'),
                                      'gate_in': LC('gate_in'), 'gate_out': LC('gate_out')}]),
    ])
