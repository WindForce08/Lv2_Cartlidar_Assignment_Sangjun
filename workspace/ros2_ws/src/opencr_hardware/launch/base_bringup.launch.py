# 프로세스 ① ros2_control_node (+ 컨트롤러 spawner) · ② base_container (robot_state_publisher · twist_mux · joy · teleop)
# ros2 launch opencr_hardware base_bringup.launch.py   (주행 4륜)
# 로봇 URDF (urdf/robot.urdf.xacro = 차체 URDF + opencr.ros2_control.xacro) 가 있어야 한다.
#   2026-10-27 1안 차체 (cartridar_01) 폐기로 지움 → 차체 2안 URDF 가 올 때까지는 이 런치가 이유를 알리고 끝낸다.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


ROBOT_XACRO = os.path.join(get_package_share_directory('opencr_hardware'), 'urdf', 'robot.urdf.xacro')
NO_URDF = ('로봇 URDF 없음 (opencr_hardware/urdf/robot.urdf.xacro) — 1안 차체 폐기 (2026-10-27), 차체 2안 URDF 대기. '
           'OpenCR · 컨트롤러 없이 실행하려면 bringup 의 use_hardware:=false')


def _setup(context):
    if not os.path.exists(ROBOT_XACRO):
        return [LogInfo(msg='[opencr_hardware] ' + NO_URDF), Shutdown(reason=NO_URDF)]
    pkg = FindPackageShare('opencr_hardware')
    use_teleop = LaunchConfiguration('use_teleop').perform(context).lower() == 'true'

    robot_description = ParameterValue(Command([
        'xacro ', PathJoinSubstitution([pkg, 'urdf', 'robot.urdf.xacro']),
        ' ports:=', LaunchConfiguration('ports'),
        ' joy_required:=', LaunchConfiguration('use_teleop')]), value_type=str)
    controllers = PathJoinSubstitution([pkg, 'config', 'controllers_4wd.yaml'])

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
        spawner('gripper_controller'),
        # ② base_container 묶음 (component 미지원 노드는 단독 실행)
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}]),
        Node(package='twist_mux', executable='twist_mux', output='screen',
             parameters=[PathJoinSubstitution([pkg, 'config', 'twist_mux.yaml'])],
             remappings=[('cmd_vel_out', '/cmd_vel')]),
    ]
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
        DeclareLaunchArgument('use_teleop', default_value='true'),
        DeclareLaunchArgument('ports', default_value='/dev/ttyACM0,/dev/ttyACM1',
                              description='역할 자동 판별을 위한 두 OpenCR 후보 포트, 쉼표로 구분'),
        OpaqueFunction(function=_setup),
    ])
