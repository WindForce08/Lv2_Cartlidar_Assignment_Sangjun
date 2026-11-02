"""
Launch file for waffle_4wd.

Usage:
    ros2 launch waffle_4wd_description display.launch.py
    ros2 launch waffle_4wd_description display.launch.py namespace:=robot1
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg_dir = get_package_share_directory('waffle_4wd_description')

    xacro_file = os.path.join(pkg_dir, 'urdf', 'waffle_4wd.urdf.xacro')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'display.rviz')

    ns = LaunchConfiguration('namespace')
    prefix = LaunchConfiguration('prefix')

    robot_description = Command([
        'xacro ', xacro_file, ' prefix:=', prefix, ' pucks:=', LaunchConfiguration('pucks'),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='ROS 2 namespace for topics and nodes',
        ),
        DeclareLaunchArgument('pucks', default_value='false', description='true 면 팔레트에 퍽 12개 (주행용 질량)'),
        DeclareLaunchArgument(
            'prefix', default_value='',
            description='URDF link/joint name prefix (passed to xacro)',
        ),

        GroupAction([
            PushRosNamespace(ns),

            # Robot state publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{'robot_description': robot_description}],
                output='screen',
            ),

            # Joint state publisher GUI (for manual joint control)
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                output='screen',
            ),

            # RViz2
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_file],
                output='screen',
            ),
        ]),
    ])
