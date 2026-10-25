#!/usr/bin/env python3
"""인지 노드 모의 시험 M1~M5 (Isaac Sim 없이 가짜 입력을 발행해 판정한다. 실물 시험 아님)

먼저 띄울 것 (같은 ROS_DOMAIN_ID, use_sim_time:=true):
  robot_state_publisher (URDF 261022_2119) · ros2 launch perception perception.launch.py use_sim_time:=true
이 스크립트가 /clock 을 직접 발행한다 (Isaac Sim 처럼 시뮬레이션 시계).

사용: python3 mock_check.py --urdf <waffle_4wd.urdf> --out <결과 폴더>   (보통은 run_mock_check.sh 로 실행)

판정 기준 (2026-10-22 사용자 확인)
  M1 앞 카메라에 파란 사각형 · 깊이 1.00 m(16UC1, 32FC1 각각) · camera_info 30 Hz, SEARCH_FRONT
     → object_found, 위치가 URDF 로 계산한 기대값과 각 축 0.01 m 이내, 발행 주기 14 Hz 이상
  M2 영상 발행 중단 → 마지막 결과 뒤 0.3 s 안에 발행이 멈춤
  M3 같은 stamp 재발행 → 새 결과 없음 / 60 s 뒤로 간 stamp → 재시작 경고, 결과가 다시 나옴
  M4 앞·뒤 스캔에 차체 밖 점 1개씩 + 차체 안 점 1개 → /scan 기대 칸 ±1, 거리 0.01 m 이내, 차체 안 점 없음
  M5 /tfmini/range 1.0 → 0.19 → 0.25 → 0.35×3 → 중단
     → 정지 켜짐(시작) → 꺼짐 → 켜짐 → 유지 → 꺼짐 → 켜짐(끊김)
"""
import argparse
import json
import math
import os
import threading
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from builtin_interfaces.msg import Time
from rcl_interfaces.msg import Log
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, LaserScan, Range
from std_msgs.msg import Bool, String
from robot_interfaces.msg import PerceptionResult
from robot_interfaces.srv import SetPerceptionMode

W, H, FX, FY, CX, CY = 640, 480, 600.0, 600.0, 320.0, 240.0
FRONT_FRAME = 'top_camera_front_color_optical_frame'


# ---------------- URDF 순방향 기구학 (tf2 와 별개로 기대값 계산) ----------------
def rpy_matrix(r, p, y):
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def link_in_base(urdf, link, base='base_link'):
    joints = {j.find('child').get('link'): j for j in ET.parse(urdf).getroot().findall('joint')}
    m = np.eye(4)
    while link != base:
        j = joints[link]
        o = j.find('origin')
        t = np.eye(4)
        t[:3, :3] = rpy_matrix(*[float(v) for v in o.get('rpy', '0 0 0').split()])
        t[:3, 3] = [float(v) for v in o.get('xyz', '0 0 0').split()]
        m = t @ m
        link = j.find('parent').get('link')
    return m


def stamp_of(t):
    s = int(math.floor(t))
    return Time(sec=s, nanosec=int(round((t - s) * 1e9)) % 1_000_000_000)


def sec_of(st):
    return st.sec + st.nanosec * 1e-9


class Mock(Node):
    def __init__(self):
        super().__init__('perception_mock_check')
        self.sim_t = 100.0
        self.last_wall = time.monotonic()
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.color_pub = self.create_publisher(Image, '/top_camera_front/color/image_raw', qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, '/top_camera_front/aligned_depth_to_color/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/top_camera_front/color/camera_info', qos_profile_sensor_data)
        self.scan_f = self.create_publisher(LaserScan, '/scan_front', qos_profile_sensor_data)
        self.scan_r = self.create_publisher(LaserScan, '/scan_rear', qos_profile_sensor_data)
        self.range_pub = self.create_publisher(Range, '/tfmini/range', qos_profile_sensor_data)
        self.results = []      # (wall, msg)
        self.scans = []
        self.stops = []        # (wall, sim, bool)
        self.reasons = []
        self.warns = []        # /rosout 경고 이상 (노드 이름, 내용)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        res_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PerceptionResult, '/perception/result', lambda m: self.results.append((time.monotonic(), m)), res_qos)
        self.create_subscription(LaserScan, '/scan', lambda m: self.scans.append((time.monotonic(), m)), qos_profile_sensor_data)
        self.create_subscription(Bool, '/perception/obstacle_stop', lambda m: self.stops.append((time.monotonic(), m.data)), latched)
        self.create_subscription(String, '/perception/obstacle_stop/reason', lambda m: self.reasons.append(m.data), latched)
        rosout_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Log, '/rosout', lambda m: m.level >= Log.WARN and self.warns.append((m.name, m.msg)), rosout_qos)
        self.cli = self.create_client(SetPerceptionMode, '/perception/set_mode')
        self.image_bgr = self.make_image()

    # 시뮬레이션 시계: 별도 스레드가 200 Hz 로 /clock 을 낸다 (Isaac Sim 처럼 일정 주기).
    # 실제 시간과 같은 속도로 흐르고, jump() 로 되감을 수 있다
    def start_clock(self):
        self.lock = threading.Lock()
        self.running = True

        def loop():
            while self.running:
                self.tick()
                time.sleep(0.005)
        self.clock_thread = threading.Thread(target=loop, daemon=True)
        self.clock_thread.start()

    def tick(self):
        with self.lock:
            now = time.monotonic()
            self.sim_t += now - self.last_wall
            self.last_wall = now
            t = self.sim_t
        self.clock_pub.publish(Clock(clock=stamp_of(t)))

    def now_sim(self):
        with self.lock:
            return self.sim_t + (time.monotonic() - self.last_wall)

    def jump(self, dt):
        with self.lock:
            self.sim_t += dt

    def spin_for(self, seconds, every=None, period=1 / 100):
        end = time.monotonic() + seconds
        next_every = 0.0
        while time.monotonic() < end:
            if every and time.monotonic() >= next_every:
                every[1]()
                next_every = time.monotonic() + every[0]
            rclpy.spin_once(self, timeout_sec=period)

    @staticmethod
    def make_image():
        img = np.full((H, W, 3), 128, np.uint8)
        blue = cv2.cvtColor(np.uint8([[[108, 230, 200]]]), cv2.COLOR_HSV2BGR)[0, 0]   # HSV 파랑 범위 [98~118] 안
        img[220:260, 300:340] = blue                                                  # 40×40 px, 중심 (320, 240)
        return img

    def camera_msgs(self, t, depth_kind):
        st = stamp_of(t)
        rgb = cv2.cvtColor(self.image_bgr, cv2.COLOR_BGR2RGB)
        color = Image(height=H, width=W, encoding='rgb8', step=W * 3, data=rgb.tobytes())
        color.header.stamp, color.header.frame_id = st, FRONT_FRAME
        if depth_kind == '16UC1':
            d = np.full((H, W), 1000, np.uint16)
            depth = Image(height=H, width=W, encoding='16UC1', step=W * 2, data=d.tobytes())
        else:
            d = np.full((H, W), 1.0, np.float32)
            depth = Image(height=H, width=W, encoding='32FC1', step=W * 4, data=d.tobytes())
        depth.header.stamp, depth.header.frame_id = st, FRONT_FRAME
        info = CameraInfo(height=H, width=W, distortion_model='plumb_bob', d=[0.0] * 5,
                          k=[FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0],
                          p=[FX, 0.0, CX, 0.0, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0])
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.header.stamp, info.header.frame_id = st, FRONT_FRAME
        return color, depth, info

    def publish_camera(self, depth_kind, stamp=None):
        c, d, i = self.camera_msgs(self.now_sim() if stamp is None else stamp, depth_kind)
        self.info_pub.publish(i)
        self.depth_pub.publish(d)
        self.color_pub.publish(c)

    def set_mode(self, mode):
        req = SetPerceptionMode.Request()
        req.mode = mode
        fut = self.cli.call_async(req)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.01)
        return fut.result()


def scan_msg(frame, t, points):
    """points: [(angle, range)] → 360칸 LaserScan (-π 부터 1°)"""
    n = 360
    s = LaserScan()
    s.header.stamp, s.header.frame_id = stamp_of(t), frame
    s.angle_min, s.angle_increment = -math.pi, 2 * math.pi / n
    s.angle_max = s.angle_min + (n - 1) * s.angle_increment
    s.range_min, s.range_max, s.scan_time = 0.05, 12.0, 0.1
    r = [float('inf')] * n
    for a, d in points:
        r[int(round((a - s.angle_min) / s.angle_increment)) % n] = d
    s.ranges = r
    return s


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--urdf', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rclpy.init()
    n = Mock()
    n.start_clock()
    report = {}

    n.spin_for(3.0)   # 시계·TF·발견 대기
    if not n.cli.wait_for_service(timeout_sec=10.0):
        raise SystemExit('/perception/set_mode 없음')

    # ---------------- M1 ----------------
    cam = link_in_base(args.urdf, FRONT_FRAME)
    expected = (cam @ np.array([0.0, 0.0, 1.0, 1.0]))[:3]   # 화면 중심, 깊이 1.00 m → 광축 위 1 m
    res = n.set_mode(1)
    m1 = {'set_mode': [res.success, res.message], 'expected_base_m': expected.round(5).tolist()}
    for kind in ('16UC1', '32FC1'):
        n.results.clear()
        t0 = time.monotonic()
        n.spin_for(3.0, every=(1 / 30, lambda k=kind: n.publish_camera(k)))
        win = [m for w, m in n.results if w - t0 > 0.5]    # 앞 0.5 s 는 시작 구간으로 뺀다
        found = [m for m in win if m.object_found]
        hz = (len(win) - 1) / (sec_of(win[-1].header.stamp) - sec_of(win[0].header.stamp)) if len(win) > 1 else 0.0
        errs = [max(abs(m.object_pose.pose.position.x - expected[0]), abs(m.object_pose.pose.position.y - expected[1]),
                    abs(m.object_pose.pose.position.z - expected[2])) for m in found]
        last = found[-1].object_pose.pose.position if found else None
        m1[kind] = {
            'results': len(win), 'found': len(found), 'rate_hz': round(hz, 2),
            'max_axis_err_m': round(max(errs), 6) if errs else None,
            'last_pos': [round(last.x, 5), round(last.y, 5), round(last.z, 5)] if last else None,
            'frame_id': found[-1].object_pose.header.frame_id if found else None,
            'pass': bool(found) and len(found) == len(win) and max(errs) <= 0.01 and hz >= 14.0}
    m1['pass'] = m1['16UC1']['pass'] and m1['32FC1']['pass']
    report['M1'] = m1

    # ---------------- M2 ----------------
    n.results.clear()
    n.spin_for(1.0, every=(1 / 30, lambda: n.publish_camera('16UC1')))
    stop_wall = time.monotonic()
    n.spin_for(1.0)
    after = [w - stop_wall for w, _ in n.results if w > stop_wall]
    report['M2'] = {'results_after_stop_s': [round(a, 3) for a in after],
                    'last_after_stop_s': round(max(after), 3) if after else 0.0,
                    'pass': (max(after) if after else 0.0) <= 0.3}

    # ---------------- M3 ----------------
    n.results.clear()
    fixed = n.now_sim()
    n.publish_camera('16UC1', stamp=fixed)
    n.spin_for(0.2)
    first = len(n.results)
    n.spin_for(0.5, every=(1 / 30, lambda: n.publish_camera('16UC1', stamp=fixed)))   # 같은 stamp 재전송
    dup_new = len(n.results) - first
    log_before = sum('시각 재시작' in w for _, w in n.warns)
    n.jump(-60.0)   # 시계와 영상 stamp 를 함께 60 s 되감는다 (시뮬레이션 리셋)
    n.results.clear()
    n.spin_for(2.0, every=(1 / 30, lambda: n.publish_camera('16UC1')))
    restart_warn = sum('시각 재시작' in w for _, w in n.warns) - log_before
    resumed = [m for _, m in n.results]
    report['M3'] = {'first_result': first, 'new_results_on_duplicate': dup_new,
                    'restart_warnings': restart_warn, 'results_after_restart': len(resumed),
                    'found_after_restart': sum(m.object_found for m in resumed),
                    'pass': first >= 1 and dup_new == 0 and restart_warn >= 1 and len(resumed) > 0}
    n.set_mode(0)

    # ---------------- M4 ----------------
    def to_base(frame, a, r):
        p = link_in_base(args.urdf, frame) @ np.array([r * math.cos(a), r * math.sin(a), 0.0, 1.0])
        return p[:3]
    pts = {'front_out': ('lidar_front_scan', 0.5, 1.0), 'rear_out': ('lidar_rear_scan', 0.0, 0.5),
           'front_in': ('lidar_front_scan', math.pi, 0.1)}
    exp = {}
    for k, (fr, a, r) in pts.items():
        b = to_base(fr, a, r)
        ang = math.atan2(b[1], b[0])
        exp[k] = {'base_xy': [round(b[0], 5), round(b[1], 5)], 'range': round(math.hypot(b[0], b[1]), 5),
                  'bin': int(math.floor((ang + math.pi) / (2 * math.pi / 360) + 0.5)) % 360}
    n.scans.clear()

    def pub_scans():
        t = n.now_sim()
        n.scan_f.publish(scan_msg('lidar_front_scan', t, [(0.5, 1.0), (math.pi, 0.1)]))
        n.scan_r.publish(scan_msg('lidar_rear_scan', t, [(0.0, 0.5)]))
    n.spin_for(1.5, every=(0.1, pub_scans))
    ok4 = False
    detail = {}
    if n.scans:
        r = list(n.scans[-1][1].ranges)
        finite = {i: v for i, v in enumerate(r) if math.isfinite(v)}
        detail['finite_bins'] = {str(i): round(v, 5) for i, v in finite.items()}
        detail['frame_id'] = n.scans[-1][1].header.frame_id

        def hit(e):
            for i, v in finite.items():
                d = min((i - e['bin']) % 360, (e['bin'] - i) % 360)
                if d <= 1 and abs(v - e['range']) <= 0.01:
                    return True
            return False
        ok4 = hit(exp['front_out']) and hit(exp['rear_out']) and len(finite) == 2
    report['M4'] = {'expected': exp, 'scans_received': len(n.scans), **detail, 'pass': ok4}

    # ---------------- M5 ----------------
    n.stops.clear()
    n.spin_for(0.3)   # 시작 상태(정지, 끊김)
    seq = [1.0] * 3 + [0.19] + [0.25] + [0.35] * 3
    for v in seq:
        m = Range()
        m.header.stamp, m.header.frame_id = stamp_of(n.now_sim()), 'tfmini_link'
        m.radiation_type, m.field_of_view, m.min_range, m.max_range, m.range = Range.INFRARED, 0.063, 0.1, 12.0, v
        n.range_pub.publish(m)
        n.spin_for(0.05)
    n.spin_for(0.6)   # 중단 → 0.2 s 뒤 끊김 정지
    changes = []
    for _, s in n.stops:
        if not changes or changes[-1] != s:
            changes.append(s)
    report['M5'] = {'state_changes': changes, 'reasons_seen': list(dict.fromkeys(n.reasons)),
                    'pass': changes == [True, False, True, False, True]}

    report['warnings'] = [f'{a}: {b}' for a, b in n.warns]
    report['all_pass'] = all(report[k]['pass'] for k in ('M1', 'M2', 'M3', 'M4', 'M5'))
    with open(os.path.join(args.out, 'mock_check_result.json'), 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=1, ensure_ascii=False)
    print(json.dumps(report, indent=1, ensure_ascii=False))
    n.running = False
    n.clock_thread.join()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
