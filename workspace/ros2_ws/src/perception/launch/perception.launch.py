# ⑦ perception_container — PerceptionNode 단독 (GPU 추론 · 카메라 3대 → 장애 격리)
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('perception')
    return LaunchDescription([
        ComposableNodeContainer(
            name='perception_container', namespace='', package='rclcpp_components',
            executable='component_container_mt', output='screen',
            composable_node_descriptions=[
                ComposableNode(package='perception', plugin='perception::PerceptionNode',
                               name='perception_node',
                               parameters=[PathJoinSubstitution([pkg, 'config', 'perception.yaml']),
                                           PathJoinSubstitution([pkg, 'config', 'arm_cam_calib.yaml'])],
                               extra_arguments=[{'use_intra_process_comms': True}]),
            ]),
    ])
