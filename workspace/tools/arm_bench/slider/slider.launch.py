"""슬라이더로 실물 팔 조작 + RViz2: 슬라이더 창 (arm_slider.py, 포트를 혼자 씀 · /joint_states 발행) + robot_state_publisher (팔 URDF) + RViz2.

    ros2 launch slider/slider.launch.py                     팔 OpenCR = /dev/ttyACM0
    ros2 launch slider/slider.launch.py port:=/dev/ttyACM1
    ros2 launch slider/slider.launch.py fake:=true          보드 없이 가짜 OpenCR (펌웨어 안전 규칙 흉내)
    ros2 launch slider/slider.launch.py rviz:=false

[시작] 을 누르기 전에는 토크를 켜지 않는다. 같은 포트를 여는 다른 프로그램 (monitor/view.launch.py, 벤치 도구) 은 먼저 끈다.
ROS 통신은 이 PC 안에서만 (ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST) - 다른 PC 의 /joint_states 와 섞이지 않게.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
URDF = os.path.join(ROOT, "urdf", "arm_bench.urdf")
FAKE_LINK = "/tmp/arm_bench_fake_opencr"


def _setup(context):
    with open(URDF, encoding="utf-8") as f:
        desc = f.read().replace('filename="meshes/', f'filename="file://{ROOT}/urdf/meshes/')
    fake = LaunchConfiguration("fake").perform(context).lower() == "true"
    port = FAKE_LINK if fake else LaunchConfiguration("port").perform(context)
    slider = ExecuteProcess(cmd=[sys.executable, os.path.join(HERE, "arm_slider.py"), "--port", port,
                                 "--title", f"arm_bench 슬라이더 - {'가짜 보드' if fake else port}"], output="screen")
    nodes = [Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
                  parameters=[{"robot_description": desc}])]
    if fake:
        nodes += [ExecuteProcess(cmd=[sys.executable, os.path.join(ROOT, "monitor", "fake_opencr.py"), "--link", FAKE_LINK,
                                      "--start", "대기"], output="screen"),
                  TimerAction(period=1.0, actions=[slider])]
    else:
        nodes.append(slider)
    if LaunchConfiguration("rviz").perform(context).lower() == "true":
        nodes.append(Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(ROOT, "monitor", "view.rviz")],
                          output="screen"))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0", description="팔 OpenCR 포트"),
        DeclareLaunchArgument("fake", default_value="false", description="true = 보드 없이 가짜 OpenCR"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
