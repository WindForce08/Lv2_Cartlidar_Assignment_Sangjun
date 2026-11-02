"""위치 추정: 지도 2 장 (map_server x 2) + AMCL + lifecycle.

  ros2 launch mission_planner localization.launch.py [use_sim_time:=false]
  /map      = maps/map_nav (주행 계획 · costmap static layer 용: 박스 · 받침대 포함)
  /map_amcl = maps/map_amcl (AMCL 용: LiDAR 스캔면을 지나는 벽판 · 기둥 · 신호등만)
nav2 (planner · controller · bt_navigator) 는 따로 띄움 (bringup.launch.py 주석: 체인 마지막 출력 /cmd_vel_nav).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("mission_planner")
    params = os.path.join(share, "config", "amcl.yaml")
    sim = LaunchConfiguration("use_sim_time")
    common = {"use_sim_time": sim}
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
             parameters=[params, common, {"yaml_filename": os.path.join(share, "maps", "map_nav.yaml")}]),
        Node(package="nav2_map_server", executable="map_server", name="map_server_amcl", output="screen",
             parameters=[params, common, {"yaml_filename": os.path.join(share, "maps", "map_amcl.yaml")}]),
        Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen", parameters=[params, common]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_localization", output="screen",
             parameters=[common, {"autostart": True, "node_names": ["map_server", "map_server_amcl", "amcl"]}]),
    ])
