#!/usr/bin/env python3
"""Gazebo TFmini (광선 1 개 LaserScan, /tfmini/scan) -> sensor_msgs/Range (/tfmini/range, 실물 tfmini_node 와 같은 토픽).

값은 config/sensors.yaml 의 tfmini_plus (시야각 3.6°, 0.1~12 m). 범위 밖은 +inf (최대 밖) / -inf (최소 안) 로 낸다.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range


class ScanToRange(Node):
    def __init__(self):
        super().__init__('tfmini_scan_to_range')
        self.fov = math.radians(self.declare_parameter('fov_deg', 3.6).value)
        self.frame = self.declare_parameter('frame_id', 'tfmini_link').value
        self.pub = self.create_publisher(Range, self.declare_parameter('range_topic', '/tfmini/range').value, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.declare_parameter('scan_topic', '/tfmini/scan').value, self.cb, qos_profile_sensor_data)

    def cb(self, scan):
        r = Range()
        r.header.stamp = scan.header.stamp
        r.header.frame_id = self.frame
        r.radiation_type = Range.INFRARED
        r.field_of_view = self.fov
        r.min_range, r.max_range = scan.range_min, scan.range_max
        d = scan.ranges[0] if scan.ranges else math.inf
        r.range = d if r.min_range <= d <= r.max_range else (-math.inf if d < r.min_range else math.inf)
        self.pub.publish(r)


def main():
    rclpy.init()
    rclpy.spin(ScanToRange())


if __name__ == '__main__':
    main()
