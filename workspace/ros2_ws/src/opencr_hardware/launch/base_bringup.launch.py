# 프로세스 ① ros2_control_node (+ 컨트롤러 spawner) · ② base_container (robot_state_publisher · twist_mux · joy · teleop)
# ros2 launch opencr_hardware base_bringup.launch.py wheel_count:=4 use_gripper:=false drive:=belt
# Isaac Sim: ros2 launch opencr_hardware base_bringup.launch.py sim:=true use_teleop:=false  (브리지 workspace/tools/isaac_sim)
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
    drive = LaunchConfiguration('drive').perform(context)
    if drive not in ('direct', 'belt'):
        raise RuntimeError(f'drive 는 direct 또는 belt: {drive}')
    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    obstacle_lock = LaunchConfiguration('obstacle_lock').perform(context).lower() == 'true'
    clock = {'use_sim_time': sim}   # 시뮬레이션이면 /clock (Isaac 브리지 발행)

    robot_description = ParameterValue(Command([
        'xacro ', PathJoinSubstitution([pkg, 'urdf', 'robot.urdf.xacro']),
        ' wheel_count:=', wheel_count,
        ' use_gripper:=', 'true' if use_gripper else 'false',
        ' ports:=', LaunchConfiguration('ports'),
        ' joy_required:=', LaunchConfiguration('use_teleop'),
        ' drive:=', drive,
        ' sim:=', 'true' if sim else 'false']), value_type=str)
    controllers = PathJoinSubstitution([pkg, 'config', f'controllers_{wheel_count}wd.yaml'])
    drive_params = PathJoinSubstitution([pkg, 'config', f'drive_{drive}.yaml'])   # 바퀴 간격 · 속도 한계 (뒤 파일이 덮어씀)

    def spawner(name, extra=()):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[name, '--controller-manager', '/controller_manager',
                               '--param-file', controllers, *extra])

    nodes = [
        # ① ros2_control_node — OpenCRHardware 플러그인과 컨트롤러가 같은 프로세스
        Node(package='controller_manager', executable='ros2_control_node', output='screen',
             parameters=[controllers, clock], remappings=[('~/robot_description', '/robot_description')]),
        spawner('joint_state_broadcaster'),
        # 시뮬레이션은 IMU 센서가 ros2_control 에 없음 — 브리지가 /imu/data 를 직접 발행
        *([] if sim else [spawner('imu_sensor_broadcaster', ['--controller-ros-args', '-r imu_sensor_broadcaster/imu:=/imu/data'])]),
        spawner('diff_drive_controller', ['--param-file', drive_params, '--controller-ros-args',
                '-r diff_drive_controller/cmd_vel:=/cmd_vel -r diff_drive_controller/odom:=/odom']),
        spawner('arm_controller'),
        # ② base_container 묶음 (component 미지원 노드는 단독 실행)
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}, clock]),
        Node(package='twist_mux', executable='twist_mux', output='screen',
             parameters=[PathJoinSubstitution([pkg, 'config', 'twist_mux.yaml' if obstacle_lock else 'twist_mux_no_obstacle.yaml']),
                         clock],
             remappings=[('cmd_vel_out', '/cmd_vel')]),
    ]
    if use_gripper:
        nodes.append(spawner('gripper_controller'))
    if use_teleop:
        nodes += [
            Node(package='joy', executable='joy_node', name='joy_node', output='screen',
                 parameters=[PathJoinSubstitution([pkg, 'config', 'teleop.yaml']), clock]),
            Node(package='joy_teleop', executable='joy_teleop', name='joy_teleop', output='screen',
                 parameters=[PathJoinSubstitution([pkg, 'config', 'teleop.yaml']), clock]),
            Node(package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
                 output='screen', parameters=[PathJoinSubstitution([pkg, 'config', 'teleop.yaml']), clock],
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
        DeclareLaunchArgument('drive', default_value='belt',
                              description='구동: belt (증속벨트 1:1.5) | direct (직결) — URDF · 기어비 · diff_drive 가 같이 바뀜'),
        DeclareLaunchArgument('obstacle_lock', default_value='true',
                              description='twist_mux 에 TFmini 비상 정지 lock (/perception/obstacle_stop). TFmini 가 없으면 false — 안 그러면 drive · nav 가 늘 잠김'),
        DeclareLaunchArgument('sim', default_value='false',
                              description='Isaac Sim: TopicBasedSystem (/isaac_joint_states · /isaac_joint_commands) + /clock'),
        OpaqueFunction(function=_setup),
    ])
