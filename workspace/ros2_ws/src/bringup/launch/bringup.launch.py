# 퍽 운반 로봇 전체 실행 (Jetson 프로세스 ① ② ⑥ ⑦ ⑧)
#
#   ros2 launch bringup bringup.launch.py                        # 실물: 하드웨어 + 인식 + 판단 · 제어
#   ros2 launch bringup bringup.launch.py use_hardware:=false    # OpenCR 없이 (판단 · 제어 · 인식만)
#   ros2 launch bringup bringup.launch.py use_perception:=false  # 카메라 없이
#   ros2 launch bringup bringup.launch.py skill_enabled:=true    # ONNX Runtime 빌드에서만 (정책 실행)
#   ros2 launch bringup bringup.launch.py drive:=direct          # 직결 구동 (기본 belt = 증속벨트)
#   ros2 launch bringup bringup.launch.py sim:=true use_sim_time:=true use_teleop:=false use_tfmini:=false   # Isaac Sim (workspace/tools/isaac_sim 브리지)
#   ros2 launch bringup bringup.launch.py front_serial:=<시리얼> rear_serial:=<시리얼>   # D435 두 대 구분 (rs-enumerate-devices -s)
#   use_sensors:=false — 카메라 · 라이다 드라이버를 띄우지 않는다 (sim:=true 면 기본 false: Isaac 브리지가 같은 토픽을 냄)
#   use_tfmini:=false — TFmini(박스 거리) · obstacle_stop 을 띄우지 않고 twist_mux obstacle lock 도 뺀다 (TFmini 가 없는데 lock 이 있으면 늘 잠김)
#
# ① ros2_control_node · ② base_container : opencr_hardware/base_bringup.launch.py
# ⑦ perception_container                  : perception/perception.launch.py
# ⑧ control_container (component_container_mt) — MissionNode(mission) · DriveNode · ManipulatorNode(control) ·
#    TFminiNode · ObstacleStopNode(perception, use_tfmini) — 1층 TFmini 박스 거리 → drive FINE 정렬 · 비상 정지(twist_mux lock).
#    카메라 인식(⑦)을 끄더라도 안전 정지가 남도록 TFmini 와 같은 프로세스에 둔다
#    같은 프로세스 · intra-process 통신 · 노드마다 MutuallyExclusive 콜백 그룹 → 바퀴 · 팔 · 판단 콜백이 병렬
# ⑥ ekf (use_ekf) — 바퀴 vx + 회전 중심 옆 이동 vy (odom_icr, #92) + IMU yaw rate
# 센서 드라이버 (use_sensors) : bringup/sensors.launch.py — D435 두 대 · 팔 Arducam · LDS-03 두 대 (TFmini 는 위 ⑧)
# nav2 · slam_toolbox 는 기존 패키지 launch 로 따로 (nav2 체인 마지막 출력만 /cmd_vel_nav)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hw = FindPackageShare('opencr_hardware')
    per = FindPackageShare('perception')
    ctl = FindPackageShare('control')
    mis = FindPackageShare('mission')
    pol = FindPackageShare('robot_policy')
    me = FindPackageShare('bringup')
    ipc = [{'use_intra_process_comms': True}]
    # 모든 노드가 같은 시계를 쓰게: 시뮬레이션이면 /clock (영상 · 라이다 stamp 가 시뮬레이션 시각이라 300 ms 규칙에 필요)
    sim = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    geometry = PathJoinSubstitution([ctl, 'config', 'robot_geometry.yaml'])   # drive · odom_icr 공용 (base.icr_x 한 곳)
    skill = {'skill.enabled':ParameterValue(LaunchConfiguration('skill_enabled'), value_type=bool),
             'skill.policies_file': LaunchConfiguration('policies_file')}
    # 카메라 없이 (use_perception:=false) 띄우면 뒤 D435 도 없음 → drive_node 후진 확인 끔 (켜 두면 값 없음 = 후진 막힘, #84)
    rear_camera = {'scan.rear_camera': ParameterValue(LaunchConfiguration('use_perception'), value_type=bool)}

    control_container = ComposableNodeContainer(
        name='control_container', namespace='', package='rclcpp_components',
        executable='component_container_mt', output='screen',
        composable_node_descriptions=[
            ComposableNode(package='mission', plugin='mission::MissionNode', name='mission_node',
                           parameters=[PathJoinSubstitution([mis, 'config', 'mission.yaml']),
                                       PathJoinSubstitution([mis, 'config', 'mission_table.yaml']), sim], extra_arguments=ipc),
            ComposableNode(package='control', plugin='control::DriveNode', name='drive_node',
                           parameters=[geometry, PathJoinSubstitution([ctl, 'config', 'drive.yaml']), skill, rear_camera, sim], extra_arguments=ipc),
            ComposableNode(package='control', plugin='control::ManipulatorNode', name='manipulator_node',
                           parameters=[PathJoinSubstitution([ctl, 'config', 'manipulator.yaml']), skill, sim], extra_arguments=ipc),
            ComposableNode(package='perception', plugin='perception::TFminiNode', name='tfmini_node',
                           parameters=[PathJoinSubstitution([per, 'config', 'tfmini.yaml']), sim], extra_arguments=ipc,
                           condition=IfCondition(LaunchConfiguration('use_tfmini'))),
            ComposableNode(package='perception', plugin='perception::ObstacleStopNode', name='obstacle_stop',
                           parameters=[PathJoinSubstitution([per, 'config', 'lidar.yaml']), sim], extra_arguments=ipc,
                           condition=IfCondition(LaunchConfiguration('use_tfmini'))),
        ])

    return LaunchDescription([
        DeclareLaunchArgument('use_hardware', default_value='true', description='① ② OpenCR · 컨트롤러 · twist_mux · 조이스틱'),
        DeclareLaunchArgument('sim', default_value='false',
                              description='① 을 Isaac Sim 브리지에 연결 (TopicBasedSystem). use_sim_time:=true 와 함께'),
        DeclareLaunchArgument('use_teleop', default_value='true', description='조이스틱 (끄면 E-stop heartbeat 검사도 끔)'),
        DeclareLaunchArgument('use_perception', default_value='true', description='⑦ 카메라 3대 · 검출'),
        DeclareLaunchArgument('use_tfmini', default_value='true',
                              description='1층 TFmini(박스 거리) · obstacle_stop · twist_mux obstacle lock. 시뮬레이션 · TFmini 없음은 false'),
        DeclareLaunchArgument('use_loc_lock', default_value='false',
                              description='twist_mux 위치 잃음 lock (/localization/lock, #93). localization 패키지를 같이 띄울 때만 true'),
        DeclareLaunchArgument('use_ekf', default_value='true', description='⑥ robot_localization ekf + odom_icr (#92)'),
        DeclareLaunchArgument('use_sensors',
                              default_value=PythonExpression(["'false' if '", LaunchConfiguration('sim'), "'.lower() == 'true' else 'true'"]),
                              description='카메라 · 라이다 드라이버 (sensors.launch.py). 기본: 실물 true · sim:=true 면 false'),
        DeclareLaunchArgument('front_serial', default_value='', description='★ 앞 D435 시리얼'),
        DeclareLaunchArgument('rear_serial', default_value='', description='★ 뒤 D435 시리얼'),
        DeclareLaunchArgument('ports', default_value='/dev/ttyACM0,/dev/ttyACM1',
                              description='두 OpenCR 후보 포트 (역할은 프로토콜 응답으로 판별)'),
        DeclareLaunchArgument('drive', default_value='belt', description='구동: belt (증속벨트) | direct (직결)'),
        DeclareLaunchArgument('skill_enabled', default_value='false',
                              description='정책 실행 — ONNX Runtime 빌드에서만 true (없으면 거절 → 미션 fallback)'),
        DeclareLaunchArgument('policies_file', default_value=PathJoinSubstitution([pol, 'config', 'policies.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='시뮬레이션 시계(/clock) 사용 — 모든 노드에 전달'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([hw, 'launch', 'base_bringup.launch.py'])),
            launch_arguments={'use_teleop': LaunchConfiguration('use_teleop'), 'ports': LaunchConfiguration('ports'),
                              'drive': LaunchConfiguration('drive'), 'sim': LaunchConfiguration('sim'),
                              'obstacle_lock': LaunchConfiguration('use_tfmini'),
                              'loc_lock': LaunchConfiguration('use_loc_lock')}.items(),
            condition=IfCondition(LaunchConfiguration('use_hardware'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([per, 'launch', 'perception.launch.py'])),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'use_obstacle_stop': 'false'}.items(),   # obstacle_stop 은 위 control_container
            condition=IfCondition(LaunchConfiguration('use_perception'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([me, 'launch', 'sensors.launch.py'])),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'front_serial': LaunchConfiguration('front_serial'),
                              'rear_serial': LaunchConfiguration('rear_serial')}.items(),
            condition=IfCondition(LaunchConfiguration('use_sensors'))),
        control_container,
        # ⑥ /odom + IMU ω → /odom_icr (vy = −icr_x·ω, 스키드 회전 중심 옆 이동, #92) → ekf odom0
        Node(package='bringup', executable='odom_icr.py', name='odom_icr', output='screen',
             parameters=[geometry, sim], condition=IfCondition(LaunchConfiguration('use_ekf'))),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
             parameters=[PathJoinSubstitution([me, 'config', 'ekf.yaml']), sim],
             condition=IfCondition(LaunchConfiguration('use_ekf'))),
    ])
