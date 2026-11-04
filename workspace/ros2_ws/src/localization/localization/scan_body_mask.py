"""라이다 차체 가림 칸 제거(실시간). 원본 토픽은 그대로 두고 <입력 토픽><output_suffix> 로 낸다. 두 가지를 합쳐 inf 로 바꾼다:
 ① 고정 칸: 첫 스캔 뒤 learn_window 초(기본 0.5~1.9 s, 출발 전 정지 구간) 동안 점이 몸체 상자 안에 든 적 있는 칸.
    mask_file 이 있으면 그 칸을 쓰고(학습 안 함), 학습했으면 mask_save 에 저장.
 ② 매 스캔 상자 검사: 그 스캔에서 몸체 상자 안 점.
상자 기본값 x −0.175~0.285, |y| ≤ 0.19 m(차체 x −0.155~0.263 · y ±0.171 + 2 cm). 학습 끝나기 전 스캔은 ②만 적용해 낸다.
(①은 출발 전 정지 구간에 차체 · 기둥에 가린 각도를 배워 둔다. 주행 중에는 ②만으로는 가린 칸이 다 안 잡힐 수 있어 ①을 함께 쓴다.)"""
import math

import numpy as np
import rclpy
import tf2_ros
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


def quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class ScanBodyMask(Node):
    def __init__(self):
        super().__init__('scan_body_mask')
        self.declare_parameter('scan_topics', ['/scan_front', '/scan_rear'])
        self.declare_parameter('output_suffix', '_filtered')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('box', [-0.175, 0.285, 0.19])
        self.declare_parameter('learn_window', [0.5, 1.9])
        self.declare_parameter('mask_file', 'none')   # 'none' = 없음
        self.declare_parameter('mask_save', 'none')
        self.base = self.get_parameter('base_frame').value
        self.box = [float(v) for v in self.get_parameter('box').value]
        suffix = self.get_parameter('output_suffix').value
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.extr = {}                                   # 라이다 frame → (회전 3×3, 위치 3) base_link 기준(정적)
        self.stats = {}
        self.win = [float(v) for v in self.get_parameter('learn_window').value]
        self.save_path = '' if self.get_parameter('mask_save').value == 'none' else self.get_parameter('mask_save').value
        self.t0, self.fixed, self.learning = None, {}, {}   # 토픽 → bool 배열(고정 칸) · 학습 중 누적
        mf = self.get_parameter('mask_file').value
        if mf and mf != 'none':
            import yaml
            for topic, bins in yaml.safe_load(open(mf)).items():
                self.fixed[topic] = np.array(bins, dtype=int)
            self.get_logger().info(f'고정 칸 불러옴 {mf}: ' + ', '.join(f'{k} {len(v)}칸' for k, v in self.fixed.items()))
        for topic in self.get_parameter('scan_topics').value:
            pub = self.create_publisher(LaserScan, topic + suffix, 10)
            self.create_subscription(LaserScan, topic, lambda m, p=pub, t=topic: self.on_scan(m, p, t), qos_profile_sensor_data)
        self.create_timer(10.0, self.report)

    def on_scan(self, msg, pub, topic):
        f = msg.header.frame_id
        if f not in self.extr:
            try:
                tf = self.buf.lookup_transform(self.base, f, Time())
            except tf2_ros.TransformException as e:
                self.get_logger().warning(f'{f} → {self.base} 정적 TF 없음, 스캔 안 냄: {e}', throttle_duration_sec=5.0)
                return
            t = tf.transform.translation
            self.extr[f] = (quat_matrix(tf.transform.rotation), np.array([t.x, t.y, t.z]))
        rot, pos = self.extr[f]
        r = np.asarray(msg.ranges, dtype=np.float32)
        a = msg.angle_min + msg.angle_increment * np.arange(len(r))
        d = rot @ np.stack([np.cos(a), np.sin(a), np.zeros_like(a)])
        ok = np.isfinite(r)
        rr = np.where(ok, r, 0.0)
        x, y = pos[0] + rr * d[0], pos[1] + rr * d[1]
        inside = ok & (x >= self.box[0]) & (x <= self.box[1]) & (np.abs(y) <= self.box[2])
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = ts
        if topic not in self.fixed:
            if self.win[0] <= ts - self.t0 <= self.win[1]:
                self.learning[topic] = self.learning.get(topic, np.zeros(len(r), bool)) | inside
            elif ts - self.t0 > self.win[1] and topic in self.learning:
                self.fixed[topic] = np.nonzero(self.learning.pop(topic))[0]
                self.get_logger().info(f'{topic}: 고정 칸 {len(self.fixed[topic])}칸 학습({len(self.fixed[topic]) * math.degrees(msg.angle_increment):.1f}°)')
                self.save()
        if topic in self.fixed:
            idx = self.fixed[topic]
            inside[idx[idx < len(r)]] = True
        r[inside] = math.inf
        msg.ranges = r.tolist()
        pub.publish(msg)
        s = self.stats.setdefault(topic, [0, 0])
        s[0] += 1
        s[1] += int(inside.sum())

    def save(self):
        if self.save_path:
            import yaml
            with open(self.save_path, 'w') as f:
                yaml.safe_dump({k: [int(i) for i in v] for k, v in self.fixed.items()}, f)

    def report(self):
        for topic, (n, k) in self.stats.items():
            if n:
                self.get_logger().info(f'{topic}: 스캔 {n}, 평균 가린 칸 {k / n:.1f}')


def main():
    rclpy.init()
    try:
        rclpy.spin(ScanBodyMask())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        if rclpy.ok():                 # 종료 신호가 대기 집합을 만드는 중에 context 를 닫은 경우만 조용히(그 밖은 그대로 오류)
            raise


if __name__ == '__main__':
    main()
