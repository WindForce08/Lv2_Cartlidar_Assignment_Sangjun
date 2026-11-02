"""mission_planner_node (계획기). mission_node 는 mission 패키지 launch 로 (source:=plan).

  ros2 launch mission_planner planner.launch.py pucks_file:=<퍽 등록표.yaml> [mode:=per_location|single_load] [log_dir:=/tmp/mission_log]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    L = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("pucks_file", default_value="", description="퍽 12 개 등록표 (placement.csv 를 옮긴 yaml, config/problem_q1_example.yaml 형식)"),
        DeclareLaunchArgument("mode", default_value="per_location", description="per_location | single_load"),
        DeclareLaunchArgument("log_dir", default_value="/tmp/mission_log", description="signal.json · allocation.json · plan.json · mission_lines.txt"),
        DeclareLaunchArgument("fine_distance", default_value="0.13", description="approach FINE (TFmini -> 박스 앞면) [m], >= 0.13"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="mission_planner", executable="mission_planner_node", name="mission_planner_node", output="screen",
             parameters=[{"pucks_file": L("pucks_file"), "mode": L("mode"), "log_dir": L("log_dir"),
                          "fine_distance": ParameterValue(L("fine_distance"), value_type=float),
                          "use_sim_time": ParameterValue(L("use_sim_time"), value_type=bool)}]),
    ])
