#!/usr/bin/env python3
"""퍽 · 과제 마커 모의 시험 P1~P5 (Isaac Sim 없이 합성 영상을 발행해 판정한다. 실물 시험 아님)

먼저 띄울 것 (같은 ROS_DOMAIN_ID, use_sim_time:=true): robot_state_publisher · perception.launch.py
보통은 run_mock_check.sh <URDF> <결과 폴더> <setup.bash> mock_check_pucks.py <설정 폴더> 로 실행한다.
설정 폴더는 config/ 복사본에서 perception.yaml 의 image_qos 를 reliable 로 바꾼 것(1080p 영상이 프로세스
사이 best effort 로는 버려짐 — 아래 Mock 주석).

합성 영상: 직육면체 · 원기둥 퍽(30 × 30 × 60 mm, 원기둥 지름 30)을 회색 선반(0.20 × 0.20 × 0.08 m) 위에 두고
광선 추적으로 그린다. 태그는 tag36h11 검은 테두리 바깥 0.018 m + 흰 여백 1칸, 직육면체는 6면 가운데,
원기둥은 옆 4곳(휘어진 스티커) + 위 · 아래(면 전체 스티커) — 가상 과제 월드와 같은 가정.
조명: 밝음 1.0, 어두움 0.625 (현장 사진 낮조명off / 낮조명on 평균 밝기 85 / 136).
팔 카메라: Arducam B0201 1920 × 1080, 대각 120° 의 핀홀 근사(f ≈ 636 px, 왜곡 없음). 카메라 자세는 이 스크립트가
/tf 로 base_link → mock_arm_optical 을 영상 stamp 마다 발행한다(팔 관절과 무관하게 자세를 정하려고).
앞 D435: 컬러 640 × 480, 내부값은 실물(시리얼 261822072247) 값, 자세는 URDF.

판정 기준 (2026-10-23 사용자 확인)
  P1 번호: 거리 0.15 · 0.25 · 0.40 m × 기울기 0 · 20 · 40° × 모양 2 × 옆면 · 끝면 × 조명 2 (× 서 있음 · 누움)
     → 번호 정답률 100 %, 없는 번호 0
  P2 위치 · 자세: 같은 조건 → 중심 오차 ≤ 5 mm, 긴 축 방향 오차 ≤ 10°, 서 있음 / 누움 100 %
  P3 색: 같은 조건 → 정답률 ≥ 95 % (원기둥 끝면은 "모름"도 정답)
  P4 마커: 앞 D435, 거리 0.5 · 1.0 · 1.5 m × 각도 0 · 30° (× 조명 2) → 번호 정답, 위치 오차 ≤ 2 cm
  P5 시각: 출력 stamp = 입력 영상 stamp, 영상이 끊기면 0.3 s 안에 발행 멈춤, 같은 stamp 재전송에 새 결과 없음
"""
import argparse
import itertools
import json
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster
from robot_interfaces.msg import Puck, PuckArray, TaskMarker
from robot_interfaces.srv import SetPerceptionMode

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mock_check import link_in_base, sec_of, stamp_of  # noqa: E402

# Arducam B0201(IMX291): 1920 × 1080, 대각 화각 120° → 핀홀 근사 f = (대각 2203 px / 2) / tan 60° ≈ 636 px.
#   실제 광각 렌즈 왜곡은 넣지 않음(실물은 camera_info 로 왜곡 보정한 뒤 같은 계산)
ARM_F = math.hypot(1920, 1080) / 2 / math.tan(math.radians(60))
ARM = dict(w=1920, h=1080, fx=ARM_F, fy=ARM_F, cx=960.0, cy=540.0, frame='mock_arm_optical')
FRONT = dict(w=640, h=480, fx=605.850769042969, fy=605.679870605469, cx=324.365814208984, cy=245.139175415039,
             frame='top_camera_front_color_optical_frame')
TAG = 0.018                       # 검은 테두리 바깥
TAG_HALF = TAG * 10 / 8 / 2       # 흰 여백 포함 반
MARKER = 0.10
MARKER_HALF = MARKER * 8 / 6 / 2
SHELF_TOP = 0.08
COLORS = {'red': (0.75, 0.08, 0.07), 'green': (0.10, 0.55, 0.18), 'blue': (0.08, 0.22, 0.78)}   # 가상 월드와 같음
COLOR_CODE = {'red': Puck.COLOR_RED, 'green': Puck.COLOR_GREEN, 'blue': Puck.COLOR_BLUE}
LIGHT = {'bright': 1.0, 'dark': 85 / 136}
L_DIR = np.array([0.3, 0.2, 1.0]) / np.linalg.norm([0.3, 0.2, 1.0])
AMB, DIF = 0.45, 0.55


def marker_texture(dict_id, marker_id, black_cells):
    cell = 32
    dic = cv2.aruco.getPredefinedDictionary(dict_id)
    if hasattr(cv2.aruco, 'generateImageMarker'):   # OpenCV ≥ 4.7
        m = cv2.aruco.generateImageMarker(dic, marker_id, black_cells * cell, borderBits=1)
    else:                                            # 4.6 (Ubuntu 24.04 python3-opencv)
        m = cv2.aruco.drawMarker(dic, marker_id, black_cells * cell, borderBits=1)
    canvas = np.full(((black_cells + 2) * cell,) * 2, 255, np.uint8)
    canvas[cell:-cell, cell:-cell] = m
    return canvas.astype(np.float32) / 255.0 * 0.9   # 흰 종이 반사율 0.9


def sample(tex, a, b, clamp=False):
    """a: 오른쪽, b: 위쪽 (0~1). 범위 밖이면 nan (clamp 면 가장자리 값)"""
    n = tex.shape[0]
    if clamp:
        a, b = np.clip(a, 0, 1 - 1e-9), np.clip(b, 0, 1 - 1e-9)
    inside = (a >= 0) & (a < 1) & (b >= 0) & (b < 1)
    out = np.full(a.shape, np.nan, np.float32)
    out[inside] = tex[np.minimum(((1 - b[inside]) * n).astype(int), n - 1), np.minimum((a[inside] * n).astype(int), n - 1)]
    return out


# ---------------- 광선 추적 물체 (base_link 좌표) ----------------
BOX_FACE_UV = {(0, 1): ((0, 1, 0), (0, 0, 1)), (0, -1): ((0, -1, 0), (0, 0, 1)), (1, 1): ((-1, 0, 0), (0, 0, 1)),
               (1, -1): ((1, 0, 0), (0, 0, 1)), (2, 1): ((1, 0, 0), (0, 1, 0)), (2, -1): ((1, 0, 0), (0, -1, 0))}


class Box:
    def __init__(self, center, R, size, albedo, tags=None):
        """tags: {(축, 부호): (texture, 반폭, (u0, v0))} — 면 위 (u0, v0) 를 중심으로 붙인 그림"""
        self.c, self.R, self.h = np.asarray(center, float), np.asarray(R, float), np.asarray(size, float) / 2
        self.albedo, self.tags = np.asarray(albedo, np.float32), tags or {}

    def hit(self, o, d):
        dl = d @ self.R
        ol = np.broadcast_to((o - self.c) @ self.R, dl.shape)
        with np.errstate(divide='ignore', invalid='ignore'):
            inv = 1.0 / dl
            t1, t2 = (-self.h - ol) * inv, (self.h - ol) * inv
        tn, tf = np.minimum(t1, t2), np.maximum(t1, t2)
        tmin, tmax, axis = np.nanmax(tn, 1), np.nanmin(tf, 1), np.nanargmax(tn, 1)
        ok = (tmax >= tmin) & (tmin > 1e-6)
        t = np.where(ok, tmin, np.inf)
        idx = np.nonzero(ok)[0]
        ax = axis[idx]
        sgn = -np.sign(dl[idx, ax])
        nl = np.zeros((len(idx), 3))
        nl[np.arange(len(idx)), ax] = sgn
        normal = np.zeros_like(d)
        normal[idx] = nl @ self.R.T
        alb = np.zeros_like(d, dtype=np.float32)
        alb[idx] = self.albedo
        p = ol[idx] + t[idx, None] * dl[idx]
        for (k, s), (tex, half, (u0, v0)) in self.tags.items():
            sel = (ax == k) & (sgn == s)
            if not sel.any():
                continue
            u, v = (np.asarray(x, float) for x in BOX_FACE_UV[(k, s)])
            g = sample(tex, (p[sel] @ u - u0) / (2 * half) + 0.5, (p[sel] @ v - v0) / (2 * half) + 0.5)
            rows = idx[sel]
            m = ~np.isnan(g)
            alb[rows[m]] = g[m, None]
        return t, normal, alb


class Cylinder:
    """축 = 지역 z. 옆 스티커 4곳(지역 x 에서 90° 간격), 위 · 아래 면 전체 스티커(흰 여백 바깥은 가장자리 값)"""

    def __init__(self, center, R, radius, height, albedo, tex):
        self.c, self.R, self.r, self.hh = np.asarray(center, float), np.asarray(R, float), radius, height / 2
        self.albedo, self.tex = np.asarray(albedo, np.float32), tex

    def hit(self, o, d):
        dl = d @ self.R
        ol = np.broadcast_to((o - self.c) @ self.R, dl.shape)
        a = dl[:, 0] ** 2 + dl[:, 1] ** 2
        b = 2 * (ol[:, 0] * dl[:, 0] + ol[:, 1] * dl[:, 1])
        cc = ol[:, 0] ** 2 + ol[:, 1] ** 2 - self.r ** 2
        disc = b * b - 4 * a * cc
        with np.errstate(divide='ignore', invalid='ignore'):
            ts = (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a)
            zs = ol[:, 2] + ts * dl[:, 2]
            side_ok = (disc >= 0) & (a > 1e-12) & (ts > 1e-6) & (np.abs(zs) <= self.hh)
            t_side = np.where(side_ok, ts, np.inf)
            caps = []
            for zc in (self.hh, -self.hh):
                tc = (zc - ol[:, 2]) / dl[:, 2]
                px, py = ol[:, 0] + tc * dl[:, 0], ol[:, 1] + tc * dl[:, 1]
                caps.append(np.where((tc > 1e-6) & (px * px + py * py <= self.r ** 2), tc, np.inf))
        allt = np.stack([t_side, caps[0], caps[1]], 1)
        which = np.argmin(allt, 1)
        t = allt[np.arange(len(t_side)), which]
        idx = np.nonzero(np.isfinite(t))[0]
        p = ol[idx] + t[idx, None] * dl[idx]
        w = which[idx]
        nl = np.zeros((len(idx), 3))
        sm = w == 0
        nl[sm, :2] = p[sm, :2] / self.r
        nl[w == 1, 2], nl[w == 2, 2] = 1, -1
        normal = np.zeros_like(d)
        normal[idx] = nl @ self.R.T
        alb = np.zeros_like(d, dtype=np.float32)
        alb[idx] = self.albedo
        g = np.full(len(idx), np.nan, np.float32)
        ang = np.arctan2(p[:, 1], p[:, 0])
        for k in range(4):
            da = (ang - k * math.pi / 2 + math.pi) % (2 * math.pi) - math.pi
            arc = da * self.r
            sel = sm & (np.abs(arc) <= TAG_HALF) & (np.abs(p[:, 2]) <= TAG_HALF)
            g[sel] = sample(self.tex, arc[sel] / (2 * TAG_HALF) + 0.5, p[sel, 2] / (2 * TAG_HALF) + 0.5)
        for wi, sg in ((1, 1), (2, -1)):
            sel = w == wi
            g[sel] = sample(self.tex, p[sel, 0] / (2 * TAG_HALF) + 0.5, sg * p[sel, 1] / (2 * TAG_HALF) + 0.5, clamp=True)
        m = ~np.isnan(g)
        alb[idx[m]] = g[m, None]
        return t, normal, alb


class Floor:
    def hit(self, o, d):
        with np.errstate(divide='ignore', invalid='ignore'):
            t = -o[2] / d[:, 2]
        t = np.where((d[:, 2] < 0) & (t > 0), t, np.inf)
        normal = np.zeros_like(d)
        normal[:, 2] = 1
        return t, normal, np.full_like(d, 0.30, dtype=np.float32)


def render(objs, cam, T, level, rng, ss=2):
    """T: base_link ← 카메라 광학 프레임 4 × 4. 반환 RGB uint8"""
    W, H = cam['w'] * ss, cam['h'] * ss
    u = (np.arange(W) + 0.5) / ss
    v = (np.arange(H) + 0.5) / ss
    uu, vv = np.meshgrid(u, v)
    dc = np.stack([(uu - cam['cx']) / cam['fx'], (vv - cam['cy']) / cam['fy'], np.ones_like(uu)], -1).reshape(-1, 3)
    dc /= np.linalg.norm(dc, axis=1, keepdims=True)
    d = dc @ T[:3, :3].T
    o = T[:3, 3]
    best = np.full(len(d), np.inf)
    normal = np.zeros_like(d)
    alb = np.full_like(d, 0.6, dtype=np.float32)   # 아무것도 없으면 먼 배경
    for ob in objs:
        t, n, a = ob.hit(o, d)
        m = t < best
        best[m], normal[m], alb[m] = t[m], n[m], a[m]
    shade = np.where(np.isfinite(best), AMB + DIF * np.clip(normal @ L_DIR, 0, None), 1.0)
    img = (alb * shade[:, None] * level * 255.0).reshape(H, W, 3)
    img = cv2.resize(img, (cam['w'], cam['h']), interpolation=cv2.INTER_AREA)
    img += rng.normal(0, 2.0, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def look_at(C, target, up=(0, 0, 1)):
    z = np.asarray(target, float) - C
    z /= np.linalg.norm(z)
    x = np.cross(z, up)
    if np.linalg.norm(x) < 1e-6:
        x = np.cross(z, (1, 0, 0))
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, :3] = np.stack([x, np.cross(z, x), z], 1)
    T[:3, 3] = C
    return T


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


# ---------------- 시험 조건 ----------------
def puck_cases():
    tex = {i: marker_texture(cv2.aruco.DICT_APRILTAG_36h11, i, 8) for i in range(12)}
    cases = []
    grid = itertools.product(('standing', 'lying'), ('cuboid', 'cylinder'), ('side', 'end'),
                             (0.15, 0.25, 0.40), (0, 20, 40), ('bright', 'dark'))
    for k, (post, shape, face, dist, tilt, light) in enumerate(grid):
        tag_id, color = k % 12, ('red', 'green', 'blue')[k % 3]
        psi = math.radians(15 + 7 * (k % 5))     # 수평 방향을 조금씩 바꿈
        h = np.array([-math.cos(psi), -math.sin(psi), 0.0])   # 로봇 쪽(−x 근처) 수평
        up = np.array([0, 0, 1.0])
        if post == 'standing':
            half_z = 0.03
            if face == 'side':
                n = h
                R = np.stack([n, np.cross(up, n), up], 1)
            else:
                n = up
                R = Rz(psi)
        else:
            half_z = 0.015
            ax = np.array([math.sin(psi), -math.cos(psi), 0.0])   # 긴 축 수평
            if face == 'side':
                n = up
                R = np.stack([up, np.cross(ax, up), ax], 1)
            else:
                n = h                                              # 끝면이 로봇 쪽 수평
                R = np.stack([up, np.cross(n, up), n], 1)
        P = np.array([0.45, 0.0, SHELF_TOP + half_z])
        depth = 0.015 if face == 'side' else 0.03
        Ptag = P + n * depth
        th = math.radians(tilt)
        if face == 'side' and post == 'lying' or (face == 'end' and post == 'standing'):
            view = math.cos(th) * up + math.sin(th) * h      # 위를 향한 면: 로봇 쪽으로 기울임
        else:
            view = math.cos(th) * n + math.sin(th) * up      # 옆을 향한 면: 위로 기울임
        C = Ptag + dist * view
        T = look_at(C, Ptag)
        alb = COLORS[color]
        if shape == 'cuboid':
            tags = {key: (tex[tag_id], TAG_HALF, (0.0, 0.0)) for key in BOX_FACE_UV}
            puck = Box(P, R, (0.03, 0.03, 0.06), alb, tags)
        else:
            puck = Cylinder(P, R, 0.015, 0.06, alb, tex[tag_id])
        shelf = Box((0.45, 0.0, SHELF_TOP / 2), np.eye(3), (0.20, 0.20, SHELF_TOP), (0.55, 0.55, 0.55))
        cases.append(dict(name=f'{post}-{shape}-{face}-{dist:.2f}m-{tilt}deg-{light}', posture=post, shape=shape,
                          face=face, dist=dist, tilt=tilt, light=light, tag_id=tag_id, color=color,
                          center=P.tolist(), axis=R[:, 2].tolist(), T=T, objs=[Floor(), shelf, puck]))
    return cases


def marker_cases(urdf):
    T = link_in_base(urdf, FRONT['frame'])
    C = T[:3, 3]
    tex = marker_texture(cv2.aruco.DICT_4X4_50, 10, 6)
    cases = []
    for k, (dist, ang, light) in enumerate(itertools.product((0.5, 1.0, 1.5), (0, 30), ('bright', 'dark'))):
        M = np.array([C[0] + dist, C[1], 0.15])
        a = math.radians(ang)
        nrm = np.array([-math.cos(a), -math.sin(a), 0.0])
        up = np.array([0, 0, 1.0])
        R = np.stack([nrm, np.cross(up, nrm), up], 1)
        wall_c = M - nrm * 0.0015 + np.array([0, 0, 0.04])     # 벽판 0.38 m 높이 가운데 0.19
        tags = {(0, 1): (tex, MARKER_HALF, (0.0, -0.04))}
        wall = Box(wall_c, R, (0.003, 1.6, 0.38), (0.30, 0.35, 0.45), tags)
        # 화면 안에 마커 네 모서리가 다 들어오는지 (계산)
        corners = [M + R[:, 1] * su * MARKER / 2 + up * sv * MARKER / 2 for su in (-1, 1) for sv in (-1, 1)]
        inv = np.linalg.inv(T)
        in_view = True
        for p in corners:
            q = inv[:3, :3] @ p + inv[:3, 3]
            u = FRONT['fx'] * q[0] / q[2] + FRONT['cx']
            v = FRONT['fy'] * q[1] / q[2] + FRONT['cy']
            in_view = in_view and q[2] > 0 and 0 <= u < FRONT['w'] and 0 <= v < FRONT['h']
        cases.append(dict(name=f'marker-{dist:.1f}m-{ang}deg-{light}', dist=dist, angle=ang, light=light, marker_id=10,
                          center=M.tolist(), in_view=bool(in_view), T=T, objs=[Floor(), wall]))
    return cases


# ---------------- ROS ----------------
class Mock(Node):
    def __init__(self):
        super().__init__('perception_puck_check')
        self.sim_t = 200.0
        self.last_wall = time.monotonic()
        self.lock = threading.Lock()
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        # 영상 발행은 reliable (best effort · reliable 구독 모두와 연결). 1080p rgb8 한 장(6.2 MB)이 UDP 수신 버퍼
        # 상한(이 PC net.core.rmem_max 4 MB)보다 커서 best effort 로는 프로세스 사이에서 거의 다 버려진다
        img_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.arm_img = self.create_publisher(Image, '/arm_camera/image_raw', img_qos)
        self.arm_info = self.create_publisher(CameraInfo, '/arm_camera/camera_info', img_qos)
        self.front_img = self.create_publisher(Image, '/top_camera_front/color/image_raw', img_qos)
        self.front_info = self.create_publisher(CameraInfo, '/top_camera_front/color/camera_info', img_qos)
        self.tfb = TransformBroadcaster(self)
        q = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.pucks, self.markers = [], []
        self.create_subscription(PuckArray, '/perception/pucks', lambda m: self.pucks.append((time.monotonic(), m)), q)
        self.create_subscription(TaskMarker, '/perception/task_marker', lambda m: self.markers.append((time.monotonic(), m)), q)
        self.cli = self.create_client(SetPerceptionMode, '/perception/set_mode')
        self.running = True
        threading.Thread(target=self._clock_loop, daemon=True).start()

    def _clock_loop(self):   # 이름이 _clock 이면 rclpy Node 내부 속성과 겹친다
        while self.running:
            self.clock_pub.publish(Clock(clock=stamp_of(self.now_sim())))
            time.sleep(0.005)

    def now_sim(self):
        with self.lock:
            return self.sim_t + (time.monotonic() - self.last_wall)

    def spin_for(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def set_mode(self, mode):
        req = SetPerceptionMode.Request()
        req.mode = mode
        fut = self.cli.call_async(req)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.01)
        return fut.result()

    @staticmethod
    def info(cam, st):
        m = CameraInfo(height=cam['h'], width=cam['w'], distortion_model='plumb_bob', d=[0.0] * 5,
                       k=[cam['fx'], 0.0, cam['cx'], 0.0, cam['fy'], cam['cy'], 0.0, 0.0, 1.0],
                       p=[cam['fx'], 0.0, cam['cx'], 0.0, 0.0, cam['fy'], cam['cy'], 0.0, 0.0, 0.0, 1.0, 0.0])
        m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        m.header.stamp, m.header.frame_id = st, cam['frame']
        return m

    @staticmethod
    def image(cam, rgb, st):
        m = Image(height=cam['h'], width=cam['w'], encoding='rgb8', step=cam['w'] * 3, data=rgb.tobytes())
        m.header.stamp, m.header.frame_id = st, cam['frame']
        return m

    def publish_arm(self, rgb, T, t):
        st = stamp_of(t)
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = st, 'base_link', ARM['frame']
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = (float(v) for v in T[:3, 3])
        qw, qx, qy, qz = quat_from_R(T[:3, :3])
        tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z = qw, qx, qy, qz
        self.tfb.sendTransform(tf)
        self.arm_info.publish(self.info(ARM, st))
        self.arm_img.publish(self.image(ARM, rgb, st))

    def publish_front(self, rgb, t):
        st = stamp_of(t)
        self.front_info.publish(self.info(FRONT, st))
        self.front_img.publish(self.image(FRONT, rgb, st))


def quat_from_R(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return w, x, y, z


def rotate_z(q):
    """quaternion (geometry_msgs) 의 z 축"""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])


def run_frames(n, publish, frames=4, period=0.1):
    """새 stamp 영상을 frames 번 발행. 발행한 stamp 목록"""
    stamps = []
    for _ in range(frames):
        t = n.now_sim()
        publish(t)
        stamps.append(round(t, 6))
        n.spin_for(period)
    return stamps


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--urdf', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rng = np.random.default_rng(1)

    t0 = time.monotonic()
    pcs = puck_cases()
    for c in pcs:
        c['rgb'] = render(c['objs'], ARM, c['T'], LIGHT[c['light']], rng)
    mcs = marker_cases(args.urdf)
    for c in mcs:
        c['rgb'] = render(c['objs'], FRONT, c['T'], LIGHT[c['light']], rng)
    print(f'합성 영상 {len(pcs) + len(mcs)}장 {time.monotonic() - t0:.1f} s', flush=True)
    img_dir = os.path.join(args.out, 'images')
    os.makedirs(img_dir, exist_ok=True)
    for c in pcs + mcs:
        small = cv2.resize(c['rgb'], (c['rgb'].shape[1] // 2, c['rgb'].shape[0] // 2), interpolation=cv2.INTER_AREA)
        cv2.imwrite(os.path.join(img_dir, c['name'] + '.jpg'), cv2.cvtColor(small, cv2.COLOR_RGB2BGR))   # 확인용 반 크기

    rclpy.init()
    n = Mock()
    n.spin_for(3.0)
    if not n.cli.wait_for_service(timeout_sec=10.0):
        raise SystemExit('/perception/set_mode 없음')
    report = {'conditions': {'arm_camera': {k: ARM[k] for k in ('w', 'h', 'fx', 'fy')}, 'light': LIGHT,
                             'note': '합성 영상(광선 추적) 모의 시험. Arducam 내부값은 가정'}}

    # ---------------- P1 · P2 · P3 (팔 카메라, READ_TAG) ----------------
    res = n.set_mode(4)   # READ_TAG
    report['set_mode_read_tag'] = [res.success, res.message]
    rows = []
    all_stamps = set()
    n.pucks.clear()
    for c in pcs:   # 결과가 늦게 와도 놓치지 않게, 모두 보낸 뒤 stamp 로 나눠 판정한다
        c['stamps'] = run_frames(n, lambda t, c=c: n.publish_arm(c['rgb'], c['T'], t), period=0.25)   # 1080p 전송 시간
        all_stamps.update(c['stamps'])
    n.spin_for(2.0)
    for c in pcs:
        stamps = c['stamps']
        msgs = [m for _, m in n.pucks if round(sec_of(m.header.stamp), 6) in stamps]
        ids_ok, wrong_ids, pos_err, ang_err, stand_ok, color_ok = [], [], [], [], [], []
        exp_c, exp_ax = np.array(c['center']), np.array(c['axis'])
        for m in msgs:
            got = [p for p in m.pucks if p.tag_id >= 0]
            wrong_ids += [p.tag_id for p in got if p.tag_id != c['tag_id']]
            hit = [p for p in got if p.tag_id == c['tag_id']]
            ids_ok.append(len(hit) == 1)
            if not hit:
                continue
            p = hit[0]
            pos = np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z])
            pos_err.append(float(np.linalg.norm(pos - exp_c)))
            if p.orientation_valid:
                ang_err.append(math.degrees(math.acos(min(1.0, abs(float(rotate_z(p.pose.orientation) @ exp_ax))))))
                stand_ok.append(p.standing == (c['posture'] == 'standing'))
            else:
                ang_err.append(None)
                stand_ok.append(False)
            unknown_ok = c['shape'] == 'cylinder' and c['face'] == 'end' and p.color == Puck.COLOR_UNKNOWN
            color_ok.append(p.color == COLOR_CODE[c['color']] or unknown_ok)
        rows.append({k: c[k] for k in ('name', 'posture', 'shape', 'face', 'dist', 'tilt', 'light', 'tag_id', 'color')} | {
            'msgs': len(msgs), 'id_ok': sum(ids_ok), 'wrong_ids': wrong_ids,
            'pos_err_mm_max': round(1000 * max(pos_err), 2) if pos_err else None,
            'axis_err_deg_max': (round(max(a for a in ang_err if a is not None), 2)
                                 if any(a is not None for a in ang_err) else None),
            'orientation_invalid': sum(a is None for a in ang_err),
            'standing_ok': sum(stand_ok), 'color_ok': sum(color_ok), 'hits': len(pos_err)})
        print(rows[-1]['name'], rows[-1]['msgs'], rows[-1]['id_ok'], rows[-1]['pos_err_mm_max'],
              rows[-1]['axis_err_deg_max'], rows[-1]['color_ok'], flush=True)

    tot = sum(r['msgs'] for r in rows)
    id_rate = sum(r['id_ok'] for r in rows) / tot if tot else 0.0
    wrong = sum(len(r['wrong_ids']) for r in rows)
    hits = sum(r['hits'] for r in rows)
    pos_fail = [r['name'] for r in rows if r['pos_err_mm_max'] is None or r['pos_err_mm_max'] > 5.0]
    ang_fail = [r['name'] for r in rows if r['orientation_invalid'] or r['axis_err_deg_max'] is None or r['axis_err_deg_max'] > 10.0]
    stand_fail = [r['name'] for r in rows if r['standing_ok'] != r['hits'] or r['hits'] == 0]
    color_rate = sum(r['color_ok'] for r in rows) / hits if hits else 0.0
    report['P1'] = {'messages': tot, 'id_rate': round(id_rate, 4), 'wrong_ids': wrong, 'pass': id_rate == 1.0 and wrong == 0 and tot > 0}
    report['P2'] = {'pos_fail_cases': pos_fail, 'axis_fail_cases': ang_fail, 'standing_fail_cases': stand_fail,
                    'pass': not pos_fail and not ang_fail and not stand_fail}
    report['P3'] = {'color_rate': round(color_rate, 4), 'color_fail_cases': [r['name'] for r in rows if r['color_ok'] != r['hits']],
                    'pass': color_rate >= 0.95}

    # P5 (퍽): stamp 일치 · 끊김 · 재전송
    stamp_bad = [round(sec_of(m.header.stamp), 6) for _, m in n.pucks if round(sec_of(m.header.stamp), 6) not in all_stamps]
    c = pcs[0]
    n.pucks.clear()
    run_frames(n, lambda t: n.publish_arm(c['rgb'], c['T'], t), frames=10)
    stop = time.monotonic()
    n.spin_for(1.0)
    after_p = [round(w - stop, 3) for w, _ in n.pucks if w > stop]
    fixed = n.now_sim()
    n.publish_arm(c['rgb'], c['T'], fixed)
    n.spin_for(1.0)
    first = len(n.pucks)
    for _ in range(10):
        n.publish_arm(c['rgb'], c['T'], fixed)
        n.spin_for(0.05)
    dup_p = len(n.pucks) - first
    n.set_mode(0)

    # ---------------- P4 (앞 D435, READ_MARKER) ----------------
    res = n.set_mode(16)   # READ_MARKER
    report['set_mode_read_marker'] = [res.success, res.message]
    mrows = []
    mstamps = set()
    n.markers.clear()
    for c in mcs:
        c['stamps'] = run_frames(n, lambda t, c=c: n.publish_front(c['rgb'], t))
        mstamps.update(c['stamps'])
    n.spin_for(2.0)
    for c in mcs:
        stamps = c['stamps']
        msgs = [m for _, m in n.markers if round(sec_of(m.header.stamp), 6) in stamps]
        errs, ok = [], []
        for m in msgs:
            if c['in_view']:
                good = m.found and m.marker_id == c['marker_id']
                if m.found:
                    e = np.linalg.norm(np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]) - np.array(c['center']))
                    errs.append(float(e))
                    good = good and e <= 0.02
            else:
                good = not m.found
            ok.append(good)
        mrows.append({k: c[k] for k in ('name', 'dist', 'angle', 'light', 'in_view')} | {
            'msgs': len(msgs), 'ok': sum(ok), 'found': sum(m.found for m in msgs),
            'pos_err_mm_max': round(1000 * max(errs), 2) if errs else None})
        print(mrows[-1], flush=True)
    report['P4'] = {'cases': mrows, 'out_of_view_cases': [r['name'] for r in mrows if not r['in_view']],
                    'pass': all(r['msgs'] > 0 and r['ok'] == r['msgs'] for r in mrows)}
    mstamp_bad = [round(sec_of(m.header.stamp), 6) for _, m in n.markers if round(sec_of(m.header.stamp), 6) not in mstamps]
    c = mcs[0]
    n.markers.clear()
    run_frames(n, lambda t: n.publish_front(c['rgb'], t), frames=10)
    stop = time.monotonic()
    n.spin_for(1.0)
    after_m = [round(w - stop, 3) for w, _ in n.markers if w > stop]
    fixed = n.now_sim()
    n.publish_front(c['rgb'], fixed)
    n.spin_for(1.0)
    first = len(n.markers)
    for _ in range(10):
        n.publish_front(c['rgb'], fixed)
        n.spin_for(0.05)
    dup_m = len(n.markers) - first
    n.set_mode(0)

    report['P5'] = {'puck_stamp_mismatch': stamp_bad, 'marker_stamp_mismatch': mstamp_bad,
                    'puck_last_after_stop_s': max(after_p) if after_p else 0.0,
                    'marker_last_after_stop_s': max(after_m) if after_m else 0.0,
                    'puck_new_on_duplicate': dup_p, 'marker_new_on_duplicate': dup_m}
    p5 = report['P5']
    p5['pass'] = (not stamp_bad and not mstamp_bad and p5['puck_last_after_stop_s'] <= 0.3 and
                  p5['marker_last_after_stop_s'] <= 0.3 and dup_p == 0 and dup_m == 0)
    report['puck_cases'] = rows
    report['all_pass'] = all(report[k]['pass'] for k in ('P1', 'P2', 'P3', 'P4', 'P5'))
    with open(os.path.join(args.out, 'puck_check_result.json'), 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=1, ensure_ascii=False, default=lambda o: o.item())   # numpy 수
    print(json.dumps({k: v for k, v in report.items() if k != 'puck_cases'}, indent=1, ensure_ascii=False, default=lambda o: o.item()))
    n.running = False
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
