"""주행 (Nav2): 경로 계획 Smac 2D · 경로 추종 MPPI · behavior · velocity_smoother · collision_monitor + (선택) localization (AMCL).

  ros2 launch navigation navigation.launch.py                                     # 실물: AMCL 포함, 출력 /cmd_vel_nav (twist_mux nav 입력)
  ros2 launch navigation navigation.launch.py use_sim_time:=true odom_topic:=/odom cmd_vel_out:=/cmd_vel   # Gazebo (waffle_4wd_description)
  ros2 launch navigation navigation.launch.py use_localization:=false            # map → odom · /map 을 다른 곳에서 낼 때

설정: config/nav2_params.yaml (★ 임시값). AMCL 설정은 localization/config/amcl.yaml, 출발 자세 start_x/y/yaw 등 localization 인자는 그대로 넘어간다.
속도 명령: controller_server · behavior_server → /nav2/cmd_vel_raw → velocity_smoother → /nav2/cmd_vel_smoothed
  → collision_monitor → cmd_vel_out (기본 /cmd_vel_nav). use_collision_monitor:=false 면 smoother 가 바로 cmd_vel_out 으로 낸다.
  Nav2 기본 launch 는 중간 출력을 cmd_vel_nav 로 쓰므로, 그대로 쓰면 twist_mux 가 smoother · collision_monitor 를 건너뛴 명령을 받는다."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

RAW = '/nav2/cmd_vel_raw'            # controller_server · behavior_server 출력
SMOOTHED = '/nav2/cmd_vel_smoothed'  # velocity_smoother 출력 = collision_monitor 입력 (nav2_params.yaml 과 같게)


def launch_setup(context):
    use_cm = LC('use_collision_monitor').perform(context).lower() == 'true'
    out = LC('cmd_vel_out').perform(context)
    params = ParameterFile(LC('params_file'), allow_substs=True)   # bt_search_directories 의 $(find-pkg-share ...) 풀기
    sim = {'use_sim_time': ParameterValue(LC('use_sim_time'), value_type=bool)}
    odom = {'odom_topic': LC('odom_topic')}
    servers = ['controller_server', 'planner_server', 'behavior_server', 'velocity_smoother'] + \
        (['collision_monitor'] if use_cm else []) + ['bt_navigator']

    nodes = [
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen',
             parameters=[params, sim, odom], remappings=[('cmd_vel', RAW)]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=[params, sim]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
             parameters=[params, sim], remappings=[('cmd_vel', RAW)]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother', output='screen',
             parameters=[params, sim, odom], remappings=[('cmd_vel', RAW), ('cmd_vel_smoothed', SMOOTHED if use_cm else out)]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
             parameters=[params, sim, odom]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', output='screen',
             parameters=[sim, {'autostart': True, 'node_names': servers}]),
    ]
    if use_cm:
        nodes.append(Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor', output='screen',
                          parameters=[params, sim, {'cmd_vel_in_topic': SMOOTHED, 'cmd_vel_out_topic': out}]))
    return nodes


def generate_launch_description():
    me = FindPackageShare('navigation')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([me, 'config', 'nav2_params.yaml'])),
        DeclareLaunchArgument('use_localization', default_value='true',
                              description='localization launch 를 use_amcl_node:=true 로 함께 (AMCL · /map · /map_objects · /scan_merged · loc_monitor)'),
        DeclareLaunchArgument('use_collision_monitor', default_value='true'),
        DeclareLaunchArgument('cmd_vel_out', default_value='/cmd_vel_nav', description='최종 속도 명령 (실물 = twist_mux nav 입력, Gazebo = /cmd_vel)'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered', description='실물 = EKF, Gazebo = /odom'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('localization'), 'launch', 'localization.launch.py'])),
            launch_arguments={'use_amcl_node': 'true', 'use_sim_time': LC('use_sim_time')}.items(),
            condition=IfCondition(LC('use_localization'))),
        OpaqueFunction(function=launch_setup),
    ])
