# 프로세스 ① ros2_control_node (+ 컨트롤러 spawner) · ② base_container (robot_state_publisher · twist_mux · joy · teleop)
# ros2 launch opencr_hardware base_bringup.launch.py wheel_count:=4 use_gripper:=false
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _setup(context):
    pkg = FindPackageShare('opencr_hardware')
    wheel_count = LaunchConfiguration('wheel_count').perform(context)
    use_gripper = LaunchConfiguration('use_gripper').perform(context).lower() == 'true'
    use_teleop = LaunchConfiguration('use_teleop').perform(context).lower() == 'true'

    robot_description = ParameterValue(Command([
        'xacro ', PathJoinSubstitution([pkg, 'urdf', 'robot.urdf.xacro']),
        ' wheel_count:=', wheel_count,
        ' use_gripper:=', 'true' if use_gripper else 'false',
        ' ports:=', LaunchConfiguration('ports'),
        ' joy_required:=', LaunchConfiguration('use_teleop')]), value_type=str)
    controllers = PathJoinSubstitution([pkg, 'config', f'controllers_{wheel_count}wd.yaml'])

    def spawner(name, extra=()):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[name, '--controller-manager', '/controller_manager',
                               '--param-file', controllers, *extra])

    nodes = [
        # ① ros2_control_node — OpenCRHardware 플러그인과 컨트롤러가 같은 프로세스
        Node(package='controller_manager', executable='ros2_control_node', output='screen',
             parameters=[controllers], remappings=[('~/robot_description', '/robot_description')]),
        spawner('joint_state_broadcaster'),
        spawner('imu_sensor_broadcaster', ['--controller-ros-args', '-r imu_sensor_broadcaster/imu:=/imu/data']),
        spawner('diff_drive_controller', ['--controller-ros-args',
                '-r diff_drive_controller/cmd_vel:=/cmd_vel -r diff_drive_controller/odom:=/odom']),
        spawner('arm_controller'),
        # ② base_container 묶음 (component 미지원 노드는 단독 실행)
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}]),
        Node(package='twist_mux', executable='twist_mux', output='screen',
             parameters=[PathJoinSubstitution([pkg, 'config', 'twist_mux.yaml'])],
             remappings=[('cmd_vel_out', '/cmd_vel')]),
    ]
    if use_gripper:
        nodes.append(spawner('gripper_controller'))
    if use_teleop:
        nodes += [
            Node(package='joy', executable='joy_node', name='joy_node', output='screen',
                 parameters=[PathJoinSubstitution([pkg, 'config', 'teleop.yaml'])]),
            Node(package='joy_teleop', executable='joy_teleop', name='joy_teleop', output='screen',
                 parameters=[PathJoinSubstitution([pkg, 'config', 'teleop.yaml'])]),
            Node(package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
                 output='screen', parameters=[PathJoinSubstitution([pkg, 'config', 'teleop.yaml'])],
                 remappings=[('cmd_vel', '/cmd_vel_teleop')]),
        ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('wheel_count', default_value='4',
                              description='실물 바퀴 수 (기본 4 = 주행 펌웨어 WHEEL_COUNT, 6 은 이전 6륜 구성)'),
        DeclareLaunchArgument('use_gripper', default_value='false', description='그리퍼 개폐 액추에이터 사용'),
        DeclareLaunchArgument('use_teleop', default_value='true'),
        DeclareLaunchArgument('ports', default_value='/dev/ttyACM0,/dev/ttyACM1',
                              description='역할 자동 판별을 위한 두 OpenCR 후보 포트, 쉼표로 구분'),
        OpaqueFunction(function=_setup),
    ])
