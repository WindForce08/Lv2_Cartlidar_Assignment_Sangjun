#!/usr/bin/env python3
"""Gazebo 카메라 출력을 실물 D435 드라이버 (realsense2_camera) 와 같은 기준으로 고쳐 다시 낸다. 목록은 config/camera_info_fix.yaml
(tools/make_revision.py 가 config/sensors.yaml 로 만듦).

1. camera_info: P 행렬 Tx (스테레오 기준선). Gazebo (gz-sim 10) 는 같은 링크에 붙은 카메라마다 '첫 카메라와의 y 거리' 를 넣는다
   (앞 IR2 65 mm, 뒤 컬러 80 mm 등). 실물: 컬러 · 깊이 · infra1 Tx = 0, infra2 Tx = -fx x 0.050 m.
2. 깊이 영상 (aligned_depth_to_color/image_raw, 32FC1 [m]): 컬러 · 깊이 센서가 D435 몸체 메시 밖으로 광축 sensor_forward_m (8 mm) 앞에
   있으므로, 유한한 값마다 + add_m -> URDF optical frame 기준 깊이.
3. 점구름 (depth/color/points): Gazebo 는 센서 축 (x 앞 · y 왼 · z 위) 으로 내면서 frame_id 는 optical frame 이라 적는다.
   optical (x 오른 · y 아래 · z 앞) 로 바꾸고 z + add_m: (x, y, z) -> (-y, -z, x + add_m).

출력 토픽을 누가 구독할 때만 Gazebo 쪽 (/gz/...) 을 구독한다 -> 안 쓰는 카메라는 렌더하지 않음 (lazy 브리지와 같이).
"""
import array

import numpy as np
import rclpy
import yaml
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.event_handler import PublisherEventCallbacks
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

RELIABLE = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)   # best-effort · reliable 구독 모두 받음 (브리지 직접 출력과 같음)
LATEST = QoSProfile(depth=2, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)   # 큰 메시지: best-effort 는 조각 하나만 잃어도 통째로 버려져 주기가 반으로 떨어짐 (2026-10-29 측정)


class Relay:
    def __init__(self, node, msg_type, src, dst, fn, pub_qos=qos_profile_sensor_data, sub_qos=qos_profile_sensor_data):
        self.node, self.msg_type, self.src, self.fn, self.sub, self.sub_qos = node, msg_type, src, fn, None, sub_qos
        self.group = MutuallyExclusiveCallbackGroup()       # 중계마다 따로 (점구름 처리가 깊이 영상을 막지 않게)
        self.pub = node.create_publisher(msg_type, dst, pub_qos, event_callbacks=PublisherEventCallbacks(matched=self.matched))

    def matched(self, info):
        if info.current_count > 0 and self.sub is None:
            self.sub = self.node.create_subscription(self.msg_type, self.src, self.cb, self.sub_qos, callback_group=self.group)
        elif info.current_count == 0 and self.sub is not None:
            self.node.destroy_subscription(self.sub)
            self.sub = None

    def cb(self, m):
        self.pub.publish(self.fn(m))


def as_uint8(a):
    """uint8[] 필드에 넣을 값. bytes 를 넣으면 rclpy 가 값마다 Python 으로 검사해 매우 느림 (1.2 MB 깊이 영상 약 0.1 s) -> array.array('B')"""
    out = array.array('B')
    out.frombytes(a.tobytes())
    return out


def fix_info(baseline):
    def f(m):
        p = list(m.p)
        p[3] = -p[0] * baseline
        p[7] = 0.0
        m.p = p
        return m
    return f


def fix_depth(add_m):
    def f(m):
        if m.encoding != '32FC1':           # Gazebo 는 32FC1 [m]
            return m
        a = np.frombuffer(m.data, dtype='>f4' if m.is_bigendian else '<f4').copy()
        fin = np.isfinite(a)
        a[fin] += add_m
        m.data = as_uint8(a)
        return m
    return f


def fix_points(add_m):
    def f(m):
        off = {fl.name: fl.offset for fl in m.fields}
        rows = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.row_step)[:, :m.width * m.point_step]
        buf = rows.reshape(m.height * m.width, m.point_step).copy()
        dt = '>f4' if m.is_bigendian else '<f4'

        def col(n):
            return buf[:, off[n]:off[n] + 4].copy().view(dt).ravel()
        x, y, z = col('x'), col('y'), col('z')
        for n, v in (('x', -y), ('y', -z), ('z', x + add_m)):
            buf[:, off[n]:off[n] + 4] = v.astype(dt).view(np.uint8).reshape(-1, 4)
        m.data = as_uint8(buf)
        m.row_step = m.width * m.point_step
        return m
    return f


def main():
    rclpy.init()
    node = Node('camera_info_fix')
    cfg = yaml.safe_load(open(node.declare_parameter('config', '').value, encoding='utf-8'))
    relays = [Relay(node, CameraInfo, c['in'], c['out'], fix_info(c['baseline_m'])) for c in cfg['camera_info']]
    relays += [Relay(node, Image, c['in'], c['out'], fix_depth(c['add_m']), RELIABLE, LATEST) for c in cfg.get('depth_image', [])]
    relays += [Relay(node, PointCloud2, c['in'], c['out'], fix_points(c['add_m']), RELIABLE, LATEST) for c in cfg.get('points', [])]
    node.get_logger().info(f'camera_info {len(cfg["camera_info"])} · 깊이 영상 {len(cfg.get("depth_image", []))} · 점구름 {len(cfg.get("points", []))} 개 '
                           '(구독이 있을 때만 Gazebo 쪽 구독)')
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    ex.spin()


if __name__ == '__main__':
    main()
