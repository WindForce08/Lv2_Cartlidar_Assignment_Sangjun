#!/usr/bin/env python3
"""관절 슬라이더 창 (joint_state_publisher_gui) 값을 Gazebo 팔 명령으로 넘긴다.

gazebo.launch.py sliders:=true 가 슬라이더 창의 출력을 /slider/joint_states 로 바꿔 띄우고 이 노드를 붙인다.
팔 관절 (joint1~5, 그리퍼 모터, 손가락 2) 만 /arm/<관절>/cmd_pos (Float64) 로 보낸다. 바퀴 슬라이더는 쓰지 않는다 (주행은 /cmd_vel).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

ARM = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper_roll', 'gripper_left_joint', 'gripper_right_joint']


class SliderToGazebo(Node):
    def __init__(self):
        super().__init__('slider_to_gazebo')
        self.pub = {j: self.create_publisher(Float64, f'/arm/{j}/cmd_pos', 10) for j in ARM}
        self.last = {}
        self.create_subscription(JointState, '/slider/joint_states', self.cb, 10)

    def cb(self, m):
        for name, pos in zip(m.name, m.position):
            if name in self.pub and self.last.get(name) != pos:      # 바뀐 관절만
                self.pub[name].publish(Float64(data=pos))
                self.last[name] = pos


def main():
    rclpy.init()
    rclpy.spin(SliderToGazebo())


if __name__ == '__main__':
    main()
