"""실물 팔을 RViz2 로 보기: 팔 OpenCR STATE (읽기 전용) -> /joint_states -> robot_state_publisher (팔 URDF) -> RViz2.

    ros2 launch monitor/view.launch.py                       포트 /dev/ttyACM0
    ros2 launch monitor/view.launch.py port:=/dev/ttyACM1
    ros2 launch monitor/view.launch.py fake:=true            보드 없이 가짜 OpenCR (관절을 천천히 흔듦)
    ros2 launch monitor/view.launch.py rviz:=false           RViz2 없이 (모니터 출력 · TF 만)
    ros2 launch monitor/view.launch.py ticks:=1008,691,3057,835,2039,603   포트 없이 J1~J6 tick 으로 고정 자세 (영점 확인용)

포트에는 쓰지 않는다 (모니터는 읽기만). 팔을 움직이는 시험은 벤치 도구 (opencr_arm_tool.py move) 를 따로 실행하고,
이 화면은 같이 띄워 둔다. 단, 한 포트는 한 프로그램만 열 수 있어서 move 를 돌리는 동안은 이 모니터를 끄거나 (port 공유 불가) - README 참고.
"""
import math
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
URDF = os.path.join(ROOT, "urdf", "arm_bench.urdf")
FAKE_LINK = "/tmp/arm_bench_fake_opencr"


def _setup(context):
    with open(URDF, encoding="utf-8") as f:
        desc = f.read().replace('filename="meshes/', f'filename="file://{ROOT}/urdf/meshes/')
    ticks = LaunchConfiguration("ticks").perform(context).strip()
    fake = LaunchConfiguration("fake").perform(context).lower() == "true"
    port = FAKE_LINK if fake else LaunchConfiguration("port").perform(context)
    monitor = ExecuteProcess(cmd=[sys.executable, os.path.join(HERE, "arm_monitor.py"), "--port", port], output="screen")
    nodes = [Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
                  parameters=[{"robot_description": desc}])]
    if ticks:      # 고정 자세: tick -> rad = (tick - 2048) x 2π/4096 (펌웨어 tickToRad, ARM_DIR 모두 +1). 포트는 열지 않음
        t = [float(x) for x in ticks.split(",")]
        if len(t) != 6:
            raise RuntimeError("ticks 는 J1~J6 (joint1~4 · 손목 롤 · 그리퍼 모터) 6개")
        q = [(x - 2048) * 2 * math.pi / 4096 for x in t]
        names = ["joint1", "joint2", "joint3", "joint4", "wrist_roll_joint", "gripper_motor_joint", "gripper_left_joint", "gripper_right_joint"]
        sys.path.insert(0, HERE)
        import gripper_map

        f = gripper_map.finger_from_tick(t[5])            # 손가락 = 그리퍼 tick 변환 (gripper_map.py)
        msg = "{header: auto, name: [%s], position: [%s]}" % (", ".join(names), ", ".join(f"{v:.5f}" for v in q + [f, f]))
        nodes.append(ExecuteProcess(cmd=["ros2", "topic", "pub", "-r", "10", "/joint_states", "sensor_msgs/msg/JointState", msg], output="log"))
    elif fake:
        nodes += [ExecuteProcess(cmd=[sys.executable, os.path.join(HERE, "fake_opencr.py"), "--link", FAKE_LINK], output="screen"),
                  TimerAction(period=1.0, actions=[monitor])]
    else:
        nodes.append(monitor)
    if LaunchConfiguration("rviz").perform(context).lower() == "true":
        nodes.append(Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(HERE, "view.rviz")], output="screen"))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0", description="팔 OpenCR 포트"),
        DeclareLaunchArgument("fake", default_value="false", description="true = 보드 없이 가짜 OpenCR"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("ticks", default_value="", description="J1~J6 tick 6개 (쉼표) - 주면 포트 없이 그 자세로 고정"),
        OpaqueFunction(function=_setup),
    ])
