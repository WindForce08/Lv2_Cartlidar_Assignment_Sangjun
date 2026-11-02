#!/usr/bin/env python3
"""키보드로 Gazebo 의 waffle_4wd 를 움직인다 (주행 + 팔). gazebo.launch.py 를 띄운 뒤 **다른 터미널**에서:

    ros2 run waffle_4wd_description keyboard_teleop.py

주행은 TurtleBot3 teleop 처럼 누를 때마다 속도가 한 단계씩 바뀐다 (/cmd_vel, TwistStamped 10 Hz).
팔은 /arm/<관절>/cmd_pos (Float64) 로 보낸다. 관절 한계는 이 패키지 URDF 값.
"""
import math
import os
import select
import sys
import termios
import tty
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64

PKG = 'waffle_4wd_description'
LIN_STEP, LIN_MAX = 0.02, 0.36          # [m/s] 최대 = 바퀴 한계 10.996 rad/s x 0.033 m
ANG_STEP, ANG_MAX = 0.2, 2.0            # [rad/s]
JOINT_STEP, FINGER_STEP = 0.1, 0.005    # [rad], [m]
ARM = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
FINGERS = ['gripper_left_joint', 'gripper_right_joint']

HELP = """
waffle_4wd 키보드 조작 (Gazebo)
  주행   w / x : 앞 · 뒤 속도 +/- {ls} m/s      a / d : 왼쪽 · 오른쪽 회전 +/- {as_} rad/s
         s 또는 스페이스 : 멈춤
  팔     1~5 : 관절 고르기 (joint1~5)        i / k : 고른 관절 +/- {js} rad
         o / l : 그리퍼 열기 · 닫기 (손가락 +/- {fs} m)   h : 팔 전부 0 (영 자세)
  끝     q 또는 Ctrl-C (멈춤 명령을 보내고 끝)
"""


def joint_limits():
    urdf = os.path.join(get_package_share_directory(PKG), 'urdf', 'waffle_4wd.urdf')
    lim = {}
    for j in ET.parse(urdf).getroot().findall('joint'):
        lm = j.find('limit')
        if lm is not None and lm.get('lower') is not None:
            lim[j.get('name')] = (float(lm.get('lower')), float(lm.get('upper')))
    return lim


class Teleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.arm = {j: self.create_publisher(Float64, f'/arm/{j}/cmd_pos', 10) for j in ARM + FINGERS}
        self.lim = joint_limits()
        self.pos = {j: 0.0 for j in ARM + FINGERS}
        self.lin = self.ang = 0.0
        self.sel = 'joint1'
        self.create_timer(0.1, self.send_drive)

    def send_drive(self):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x, m.twist.angular.z = self.lin, self.ang
        self.cmd.publish(m)

    def set_joint(self, j, v):
        lo, hi = self.lim.get(j, (-math.inf, math.inf))
        self.pos[j] = min(max(v, lo), hi)
        self.arm[j].publish(Float64(data=self.pos[j]))

    def key(self, c):
        """키 하나 처리. 끝내려면 False."""
        if c in ('q', '\x03'):
            return False
        if c == 'w':
            self.lin = min(self.lin + LIN_STEP, LIN_MAX)
        elif c == 'x':
            self.lin = max(self.lin - LIN_STEP, -LIN_MAX)
        elif c == 'a':
            self.ang = min(self.ang + ANG_STEP, ANG_MAX)
        elif c == 'd':
            self.ang = max(self.ang - ANG_STEP, -ANG_MAX)
        elif c in ('s', ' '):
            self.lin = self.ang = 0.0
        elif c in '12345':
            self.sel = ARM[int(c) - 1]
        elif c in ('i', 'k'):
            self.set_joint(self.sel, self.pos[self.sel] + (JOINT_STEP if c == 'i' else -JOINT_STEP))
        elif c in ('o', 'l'):
            for f in FINGERS:
                self.set_joint(f, self.pos[f] + (FINGER_STEP if c == 'o' else -FINGER_STEP))
        elif c == 'h':
            for j in ARM + FINGERS:
                self.set_joint(j, 0.0)
        else:
            return True
        self.send_drive()
        return True

    def status(self):
        return (f'\r주행 {self.lin:+.2f} m/s {self.ang:+.2f} rad/s | {self.sel} {self.pos[self.sel]:+.2f} rad | '
                f'손가락 {self.pos[FINGERS[0]] * 1000:+.0f} mm      ')


def main():
    if not sys.stdin.isatty():
        sys.exit('키보드 입력이 필요합니다: 터미널에서 직접 실행하세요 (ros2 run waffle_4wd_description keyboard_teleop.py)')
    rclpy.init()
    node = Teleop()
    print(HELP.format(ls=LIN_STEP, as_=ANG_STEP, js=JOINT_STEP, fs=FINGER_STEP))
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        run = True
        while run and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if select.select([sys.stdin], [], [], 0.03)[0]:
                run = node.key(sys.stdin.read(1).lower())
                sys.stdout.write(node.status())
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.lin = node.ang = 0.0
        node.send_drive()                                   # 멈춤 명령
        print('\n멈춤 명령을 보내고 끝냅니다.')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
