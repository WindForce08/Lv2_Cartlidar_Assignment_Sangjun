"""AMCL 입력용 스캔 합치기(AMCL 은 스캔 토픽 하나만 받음, localization:=amcl 일 때만 뜸). 앞 · 뒤 가림 제거 스캔(/scan_front_filtered · /scan_rear_filtered)을
base_link 원점 기준 한 스캔(/scan_merged, frame base_link, 720 칸 0.5°)으로 낸다. 각 점의 끝 위치는 그대로 두고 base_link 원점에서 본 각 · 거리로만 바꾼다.
앞 스캔마다 낸다(stamp = 앞 스캔). 같은 stamp 의 뒤 스캔이 pair_wait 초 안에 오면 그대로 붙이고, 안 오면 가장 최근 뒤 스캔(max_rear_age 안)을
odom TF 로 앞 스캔 시각의 차체 자세에 옮겨 넣는다(움직임 보정)."""
import math

import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


def tf_matrix(tf):
    q, t = tf.transform.rotation, tf.transform.translation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, t.x], [s, c, t.y], [0, 0, 1]])


class ScanMerge(Node):
    def __init__(self):
        super().__init__('scan_merge')
        self.declare_parameter('front', '/scan_front_filtered')
        self.declare_parameter('rear', '/scan_rear_filtered')
        self.declare_parameter('out', '/scan_merged')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('bins', 720)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('max_rear_age', 0.15)
        self.declare_parameter('pair_wait', 0.06)
        g = lambda n: self.get_parameter(n).value
        self.base, self.odom = g('base_frame'), g('odom_frame')
        self.bins, self.rmax, self.max_age, self.pair_wait = g('bins'), g('range_max'), g('max_rear_age'), g('pair_wait')
        self.buf = tf2_ros.Buffer()
        # 콜백 안에서 TF 를 timeout 으로 기다리므로 수신은 따로 도는 스레드에서(같은 executor 면 기다리는 동안 TF 가 안 들어와 늘 실패)
        self.listener = tf2_ros.TransformListener(self.buf, self, spin_thread=True)
        self.extr = {}
        self.rear = None
        self.pending = None      # 같은 stamp 뒤 스캔을 기다리는 앞 스캔
        self.pub = self.create_publisher(LaserScan, g('out'), 10)
        self.create_subscription(LaserScan, g('rear'), self.on_rear, qos_profile_sensor_data)
        self.create_subscription(LaserScan, g('front'), self.on_front_in, qos_profile_sensor_data)
        self.create_timer(0.02, self.flush)
        self.n_out = self.n_no_rear = self.n_paired = 0

    def points(self, msg):
        """스캔 → base_link 기준 점 (3, N) 동차좌표(유한 거리만)."""
        f = msg.header.frame_id
        if f not in self.extr:
            tf = self.buf.lookup_transform(self.base, f, Time())
            q = tf.transform.rotation
            x, y, z, w = q.x, q.y, q.z, q.w
            r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z)]])
            self.extr[f] = (r, np.array([tf.transform.translation.x, tf.transform.translation.y]))
        r, p = self.extr[f]
        rr = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + msg.angle_increment * np.arange(len(rr))
        ok = np.isfinite(rr) & (rr >= msg.range_min) & (rr <= msg.range_max)
        d = r @ np.stack([np.cos(a[ok]), np.sin(a[ok])])
        xy = p[:, None] + rr[ok] * d
        return np.vstack([xy, np.ones(xy.shape[1])])

    def on_front_in(self, msg):
        if self.pending is not None:
            self.on_front(self.pending)
        if self.rear is not None and self.rear.header.stamp == msg.header.stamp:
            self.on_front(msg)
        else:
            self.pending = msg

    def on_rear(self, msg):
        self.rear = msg
        if self.pending is not None and self.pending.header.stamp == msg.header.stamp:
            m, self.pending = self.pending, None
            self.on_front(m)

    def flush(self):
        if self.pending is not None and (self.get_clock().now() - Time.from_msg(self.pending.header.stamp)).nanoseconds * 1e-9 > self.pair_wait:
            m, self.pending = self.pending, None
            self.on_front(m)

    def on_front(self, msg):
        try:
            pts = [self.points(msg)]
        except tf2_ros.TransformException as e:
            self.get_logger().warning(f'정적 TF 없음: {e}', throttle_duration_sec=5.0)
            return
        tf_ = Time.from_msg(msg.header.stamp)
        rear = self.rear
        if rear is not None and abs((tf_ - Time.from_msg(rear.header.stamp)).nanoseconds) * 1e-9 <= self.max_age:
            try:
                if tf_ == Time.from_msg(rear.header.stamp):            # 같은 시각이면 차체 움직임 보정이 필요 없음
                    pts.append(self.points(rear))
                    self.n_paired += 1
                else:
                    to = tf_matrix(self.buf.lookup_transform(self.odom, self.base, tf_, timeout=Duration(seconds=0.05)))
                    tr = tf_matrix(self.buf.lookup_transform(self.odom, self.base, Time.from_msg(rear.header.stamp), timeout=Duration(seconds=0.05)))
                    pts.append(np.linalg.inv(to) @ tr @ self.points(rear))
            except tf2_ros.TransformException:
                self.n_no_rear += 1
        else:
            self.n_no_rear += 1
        xy = np.hstack(pts)
        ang = np.arctan2(xy[1], xy[0])
        rng = np.hypot(xy[0], xy[1])
        inc = 2 * math.pi / self.bins
        idx = np.clip(((ang + math.pi) / inc).astype(int), 0, self.bins - 1)
        out = np.full(self.bins, np.inf)
        np.minimum.at(out, idx, rng)
        m = LaserScan()
        m.header.stamp = msg.header.stamp
        m.header.frame_id = self.base
        m.angle_min, m.angle_increment = -math.pi + inc / 2, inc
        m.angle_max = m.angle_min + inc * (self.bins - 1)
        m.time_increment, m.scan_time = 0.0, msg.scan_time
        m.range_min, m.range_max = 0.0, self.rmax
        m.ranges = [float(v) for v in out]
        self.pub.publish(m)
        self.n_out += 1
        if self.n_out % 300 == 0:
            self.get_logger().info(f'합친 스캔 {self.n_out} (같은 시각 뒤 스캔 {self.n_paired}, 뒤 스캔 못 붙임 {self.n_no_rear})')


def main():
    rclpy.init()
    node = ScanMerge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        if rclpy.ok():                 # 종료 신호가 대기 집합을 만드는 중에 context 를 닫은 경우만 조용히(그 밖은 그대로 오류)
            raise


if __name__ == '__main__':
    main()
