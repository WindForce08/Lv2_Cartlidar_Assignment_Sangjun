#!/usr/bin/env python3
"""팔 OpenCR 의 STATE 를 읽어 /joint_states 로 보낸다 (읽기 전용 - 포트에 한 바이트도 쓰지 않음). RViz2 에서 실물 팔 보기용.

    python3 monitor/arm_monitor.py --port /dev/ttyACM0      (보통은 monitor/view.launch.py 가 실행)

- 파싱은 벤치 도구 (ros2_ws/src/opencr_hardware/scripts/opencr_arm_tool.py) 의 Link · decode_state 를 그대로 쓴다. 그 도구의 monitor 처럼 write 를 부르지 않는다.
- STATE 팔 칸 6~11 = joint1 · joint2 · joint3 · joint4 · wrist_roll_joint · gripper_motor_joint (펌웨어 ARM_IDS 순서, rad).
- 손가락 (gripper_left/right_joint, m) 은 그리퍼 모터 tick 을 직선으로 나눠 보냄 (monitor/gripper_map.py, 닫힘 2650 · 열림 1550 tick,
  --grip-closed · --grip-open 으로 바꿈).
- 1 초마다 수신 Hz · 보드 역할 · 펌웨어 버전 · 오류 비트 · 관절 각도 [deg] 를 출력. 역할이 팔 (2) 이 아니거나 버전이 다르면 경고.
"""
import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "..", "..", "ros2_ws", "src", "opencr_hardware", "scripts"))   # Lv2 벤치 도구
import opencr_arm_tool as tool  # noqa: E402

import gripper_map  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

ARM_NAMES = ["joint1", "joint2", "joint3", "joint4", "wrist_roll_joint", "gripper_motor_joint"]
FINGERS = ["gripper_left_joint", "gripper_right_joint"]


class ArmMonitor(Node):
    def __init__(self, port, rate, grip_closed, grip_open):
        super().__init__("arm_monitor")
        tool.self_check()
        self.port = port
        self.grip = (grip_closed, grip_open)
        self.link = None
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.n_rx, self.t_log, self.last_seq, self.last_rx = 0, time.monotonic(), None, None
        self.warned = set()
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f"읽기 전용 모니터: {port} (포트에 쓰지 않음)")

    def tick(self):
        if self.link is None:
            try:
                self.link = tool.Link(self.port)
                self.get_logger().info(f"포트 열림: {self.port}")
            except Exception as e:  # noqa: BLE001
                self.warn_once("open", f"포트를 열 수 없음 ({e}) - 연결·권한 (dialout) 확인, 1 초마다 다시 시도")
                time.sleep(1.0)
                return
        try:
            st = self.link.poll()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"포트 읽기 오류 ({e}) - 다시 엽니다")
            self.link = None
            return
        now = time.monotonic()
        if st is not None and st["seq"] != self.last_seq:
            self.last_seq, self.last_rx = st["seq"], now
            self.n_rx += 1
            if st["role"] != tool.ROLE_ARM:
                self.warn_once("role", f"이 보드는 팔 보드가 아님 (role={st['role']}, 팔 = {tool.ROLE_ARM})")
            if st["fw"] != tool.PROTOCOL_VERSION:
                self.warn_once("fw", f"펌웨어 프로토콜 v{st['fw']} (이 도구는 v{tool.PROTOCOL_VERSION})")
            q = tool.arm_pos(st)
            v = st["vel"][tool.ARM0:tool.ARM0 + 6]
            e = st["effort"][tool.ARM0:tool.ARM0 + 6]
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ARM_NAMES + FINGERS
            f = gripper_map.finger_from_rad(q[5], *self.grip)
            msg.position = [float(x) for x in q] + [f, f]
            msg.velocity = [float(x) for x in v] + [0.0, 0.0]
            msg.effort = [float(x) for x in e] + [0.0, 0.0]
            self.pub.publish(msg)
        if now - self.t_log >= 1.0:
            hz = self.n_rx / (now - self.t_log)
            self.n_rx, self.t_log = 0, now
            last = self.link.last
            if last is None or self.last_rx is None or now - self.last_rx > 0.5:
                self.get_logger().warning("STATE 없음 - 보드 전원·USB·펌웨어 확인")
            else:
                q = tool.arm_pos(last)
                deg = " ".join(f"{math.degrees(x):7.1f}" for x in q)
                gt = gripper_map.unwrap_tick(2048 + q[5] / gripper_map.RAD_PER_TICK, *self.grip)
                self.get_logger().info(f"{hz:5.1f} Hz role={last['role']} fw={last['fw']} err=0x{last['error']:02X} "
                                       f"{tool.err_text(last['error'])} | J1~J5·그리퍼 [deg] {deg} | 그리퍼 {gt:.0f} tick → 손가락 "
                                       f"{gripper_map.finger_from_rad(q[5], *self.grip) * 1000:+.1f} mm")

    def warn_once(self, key, text):
        if key not in self.warned:
            self.warned.add(key)
            self.get_logger().warning(text)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0", help="팔 OpenCR 포트")
    ap.add_argument("--rate", type=float, default=200.0, help="포트 확인 주기 [Hz] (STATE 는 보드가 100 Hz 로 보냄)")
    ap.add_argument("--grip-closed", type=float, default=gripper_map.CLOSED_TICK, help="그리퍼 완전히 닫힘 tick")
    ap.add_argument("--grip-open", type=float, default=gripper_map.OPEN_TICK, help="그리퍼 완전히 열림 tick (누적, 음수 가능)")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = ArmMonitor(args.port, args.rate, args.grip_closed, args.grip_open)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
