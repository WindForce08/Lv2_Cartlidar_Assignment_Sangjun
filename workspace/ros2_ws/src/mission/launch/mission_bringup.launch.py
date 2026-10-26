# ⑨ mission_container (component_container_mt) — MissionNode
# 사용: ros2 launch mission mission_bringup.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    config = os.path.join(get_package_share_directory('mission'), 'config')
    mission = ComposableNode(
        package='mission',
        plugin='mission::MissionNode',
        name='mission_node',
        parameters=[os.path.join(config, 'mission.yaml'),
                    os.path.join(config, 'mission_table.yaml')],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    container = ComposableNodeContainer(
        name='mission_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[mission],
        output='screen',
    )
    return LaunchDescription([container])
