# 퍽 운반 로봇 전체 실행 (Jetson 프로세스 ① ② ⑥ ⑦ ⑧)
#
#   ros2 launch bringup bringup.launch.py                        # 실물: 하드웨어 + 인식 + 판단 · 제어
#   ros2 launch bringup bringup.launch.py use_hardware:=false    # OpenCR 없이 (판단 · 제어 · 인식만)
#   ros2 launch bringup bringup.launch.py use_perception:=false  # 카메라 없이
#   ros2 launch bringup bringup.launch.py skill_enabled:=true    # ONNX Runtime 빌드에서만 (정책 실행)
#
# ① ros2_control_node · ② base_container : opencr_hardware/base_bringup.launch.py
# ⑦ perception_container                  : perception/perception.launch.py
# ⑧ control_container (component_container_mt) — MissionNode(center) · DriveNode · ManipulatorNode(control) · TFminiNode(perception)
#    같은 프로세스 · intra-process 통신 · 노드마다 MutuallyExclusive 콜백 그룹 → 바퀴 · 팔 · 판단 콜백이 병렬
# ⑥ ekf (use_ekf) — 바퀴 vx + IMU yaw rate
# nav2 · slam_toolbox 는 기존 패키지 launch 로 따로 (nav2 체인 마지막 출력만 /cmd_vel_nav)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hw = FindPackageShare('opencr_hardware')
    per = FindPackageShare('perception')
    ctl = FindPackageShare('control')
    cen = FindPackageShare('center')
    pol = FindPackageShare('robot_policy')
    me = FindPackageShare('bringup')
    ipc = [{'use_intra_process_comms': True}]
    skill = {'skill.enabled': ParameterValue(LaunchConfiguration('skill_enabled'), value_type=bool),
             'skill.policies_file': LaunchConfiguration('policies_file')}

    control_container = ComposableNodeContainer(
        name='control_container', namespace='', package='rclcpp_components',
        executable='component_container_mt', output='screen',
        composable_node_descriptions=[
            ComposableNode(package='center', plugin='center::MissionNode', name='mission_node',
                           parameters=[PathJoinSubstitution([cen, 'config', 'mission.yaml'])], extra_arguments=ipc),
            ComposableNode(package='control', plugin='control::DriveNode', name='drive_node',
                           parameters=[PathJoinSubstitution([ctl, 'config', 'drive.yaml']), skill], extra_arguments=ipc),
            ComposableNode(package='control', plugin='control::ManipulatorNode', name='manipulator_node',
                           parameters=[PathJoinSubstitution([ctl, 'config', 'manipulator.yaml']), skill], extra_arguments=ipc),
            ComposableNode(package='perception', plugin='perception::TFminiNode', name='tfmini_node',
                           parameters=[PathJoinSubstitution([per, 'config', 'tfmini.yaml'])], extra_arguments=ipc),
        ])

    return LaunchDescription([
        DeclareLaunchArgument('use_hardware', default_value='true', description='① ② OpenCR · 컨트롤러 · twist_mux · 조이스틱'),
        DeclareLaunchArgument('use_teleop', default_value='true', description='조이스틱 (끄면 E-stop heartbeat 검사도 끔)'),
        DeclareLaunchArgument('use_perception', default_value='true', description='⑦ 카메라 3대 · 검출'),
        DeclareLaunchArgument('use_ekf', default_value='true', description='⑥ robot_localization ekf'),
        DeclareLaunchArgument('port', default_value='/dev/opencr'),
        DeclareLaunchArgument('skill_enabled', default_value='false',
                              description='정책 실행 — ONNX Runtime 빌드에서만 true (없으면 거절 → 미션 fallback)'),
        DeclareLaunchArgument('policies_file', default_value=PathJoinSubstitution([pol, 'config', 'policies.yaml'])),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([hw, 'launch', 'base_bringup.launch.py'])),
            launch_arguments={'use_teleop': LaunchConfiguration('use_teleop'), 'port': LaunchConfiguration('port')}.items(),
            condition=IfCondition(LaunchConfiguration('use_hardware'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([per, 'launch', 'perception.launch.py'])),
            condition=IfCondition(LaunchConfiguration('use_perception'))),
        control_container,
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
             parameters=[PathJoinSubstitution([me, 'config', 'ekf.yaml'])],
             condition=IfCondition(LaunchConfiguration('use_ekf'))),
    ])
