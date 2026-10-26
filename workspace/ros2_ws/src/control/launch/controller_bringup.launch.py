# ⑧ control_container (component_container_mt) — DriveNode + ManipulatorNode + TFminiNode
# 사용: ros2 launch control controller_bringup.launch.py
# TODO(Controller): ④ nav2_container (nav2_params.yaml, use_composition) 추가
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    control = os.path.join(get_package_share_directory('control'), 'config')
    perception = os.path.join(get_package_share_directory('perception'), 'config')
    geometry = os.path.join(control, 'robot_geometry.yaml')   # drive · manipulator 공용
    intra = [{'use_intra_process_comms': True}]

    nodes = [
        ComposableNode(
            package='control', plugin='control::DriveNode', name='drive_node',
            parameters=[geometry, os.path.join(control, 'drive.yaml')], extra_arguments=intra),
        ComposableNode(
            package='control', plugin='control::ManipulatorNode', name='manipulator_node',
            parameters=[geometry, os.path.join(control, 'manipulator.yaml')], extra_arguments=intra),
        ComposableNode(
            package='perception', plugin='perception::TFminiNode', name='tfmini_node',
            parameters=[os.path.join(perception, 'tfmini.yaml')], extra_arguments=intra),
    ]
    container = ComposableNodeContainer(
        name='control_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=nodes,
        output='screen',
    )
    return LaunchDescription([container])
