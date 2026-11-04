#!/usr/bin/env python3
"""odom_icr (#92): 바퀴 odom 에 스키드 회전 중심 (ICR) 옆 이동을 채워 EKF 에 넘긴다.

4 륜 스키드는 base_link 가 아니라 ICR (base_link x = icr_x, 모의 -0.0658 m) 둘레로 돈다.
그래서 돌 때 base_link 는 옆으로 vy = -icr_x * w 만큼 움직이는데, diff_drive /odom 은 vy 를 늘 0 으로 낸다.
  /odom (nav_msgs/Odometry) + /imu/data (w) -> /odom_icr (vy 채움, 나머지는 /odom 그대로)
  - w: IMU 회전 속도 (바퀴 회전 속도는 미끄러져 큼). IMU 가 imu_timeout 넘게 끊기면 /odom 의 w (경고)
  - vy 분산 = (icr_x * yaw_rate_sd)^2 + (w * icr_x_sd)^2 + vy_var_min
  icr_x = robot_geometry.yaml base.icr_x (drive_node turn 보정과 같은 값 한 곳). ekf.yaml odom0 = /odom_icr (vx · vy)
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class OdomIcr(Node):
    def __init__(self):
        super().__init__('odom_icr')
        self.icr_x = self.declare_parameter('base.icr_x', 0.0).value
        self.icr_x_sd = self.declare_parameter('icr_x_sd', 0.02).value        # [m] icr_x 불확실 (실측 전)
        self.yaw_rate_sd = self.declare_parameter('yaw_rate_sd', 0.05).value  # [rad/s] IMU w 오차
        self.vy_var_min = self.declare_parameter('vy_var_min', 1e-6).value    # [(m/s)^2] 0 분산 방지
        self.imu_timeout = self.declare_parameter('imu_timeout', 0.2).value   # [s]
        odom_in = self.declare_parameter('odom_in', '/odom').value
        odom_out = self.declare_parameter('odom_out', '/odom_icr').value
        imu_in = self.declare_parameter('imu_in', '/imu/data').value
        self.w_imu, self.t_imu = None, None
        self.pub = self.create_publisher(Odometry, odom_out, 10)
        self.create_subscription(Imu, imu_in, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_in, self.on_odom, 10)
        self.get_logger().info(f'{odom_in} + {imu_in} -> {odom_out}, icr_x {self.icr_x:+.4f} m')

    def on_imu(self, m):
        self.w_imu, self.t_imu = m.angular_velocity.z, self.get_clock().now()

    def on_odom(self, m):
        now = self.get_clock().now()
        if self.t_imu is not None and (now - self.t_imu).nanoseconds * 1e-9 <= self.imu_timeout:
            w = self.w_imu
        else:
            w = m.twist.twist.angular.z
            self.get_logger().warning('IMU 없음 -> 바퀴 w 로 vy', throttle_duration_sec=5.0)
        m.twist.twist.linear.y = -self.icr_x * w
        cov = list(m.twist.covariance)
        cov[7] = (self.icr_x * self.yaw_rate_sd) ** 2 + (w * self.icr_x_sd) ** 2 + self.vy_var_min
        m.twist.covariance = cov
        self.pub.publish(m)


def main():
    rclpy.init()
    node = OdomIcr()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
