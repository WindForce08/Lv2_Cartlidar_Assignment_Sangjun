"""로봇 외곽 · 접근 자세표.

로봇 (waffle_4wd B안, base_link 기준, 2026-11-02 URDF):
  앞 끝 (상판) x +0.1328, 뒤 끝 (뒤 카메라 기둥) x -0.2135, 폭 (바퀴 바깥) y +-0.1676, J1 축 x +0.096.
  로봇 중심 (외곽 가운데) = base_link + (-0.04035, 0). 발제 "로봇 중심 (734, 950)" 은 이 점으로 본다.

접근 자세 (모두 지도 = 발제 좌표 m, yaw = 로봇 앞 방향):
  - 시작 박스: 두 퍽 (박스 중심 X +-40 mm) 을 잇는 선 (X 축) 에 수직인 쪽 = 박스 남쪽 (yaw +90°) 또는 북쪽 (yaw -90°).
    로봇이 박스 중심 X 에 서면 두 퍽이 J1 앞 같은 거리, 옆 +-40 mm -> 한 번 서서 둘 다 집음.
  - L1 · L2 · L3: 장소 태그가 붙은 면 (접근 방향 수직 면) 쪽. L1 = 서쪽 면 (yaw 0), L2 = 받침대 북쪽 면 (yaw -90°), L3 = 남쪽 면 (yaw +90°).
  - final: 앞 끝 - 면 = gap (집기 · 놓기 자세), stage: 앞 끝 - 면 = standoff (nav2 목표, 그다음 drive align 이 곧게 들어감).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

FRONT, REAR, HALF_W, J1_X = 0.1328, -0.2135, 0.1676, 0.096
CENTER_X = (FRONT + REAR) / 2  # -0.04035

START_BOXES = {  # 발제 07 쪽 시작 박스 중심 (m), 205 x 205 x 70
    1: (1.468, 0.950), 2: (3.670, 3.152), 3: (2.936, 5.354),
    4: (0.734, 5.354), 5: (2.202, 5.354), 6: (3.670, 3.886),
}
BOX_HALF = 0.1025
PUCK_DX = 0.040  # 박스 위 퍽 2 개 = 중심 X +-40 mm, Y 0
PLACES = {  # 장소 격자 중심, 접근할 면 (바깥 법선 방향), 면까지 거리
    "L1": dict(center=(3.670, 0.950), face_dir=(-1.0, 0.0), face_dist=0.1025, tag=12),
    "L2": dict(center=(2.202, 3.700), face_dir=(0.0, 1.0), face_dist=3.910 - 3.700, tag=13),  # 820 받침대 (중심 y 3.5) 북쪽 면 y 3.91
    "L3": dict(center=(3.670, 5.354), face_dir=(0.0, -1.0), face_dist=0.1025, tag=14),
}
OBSERVE = (0.734, 0.950)  # 출발 · 관측 · 복귀 (로봇 중심), -X 를 봄
START_ZONE_HALF = 0.30
GRID_STEP = 0.060
DICE = {  # 주사위 눈 -> 격자 칸 (dx, dy) 단위 칸, 지도 +Y 가 위 (발제 03 쪽 그림)
    1: [(0, 0)],
    2: [(-1, 1), (1, -1)],
    3: [(-1, 1), (0, 0), (1, -1)],
    4: [(-1, 1), (1, 1), (-1, -1), (1, -1)],
    5: [(-1, 1), (1, 1), (0, 0), (-1, -1), (1, -1)],
    6: [(-1, 1), (-1, 0), (-1, -1), (1, 1), (1, 0), (1, -1)],
}
REACH_VERTICAL = 0.23  # [m] J1 축 - 퍽 수평 거리: 수직 집기 · 놓기 (TopPick reach_max, pallet_traj IK)
REACH_TILT = 0.35      # 45° 기울여 (팀 URDF IK: J1 거리 0.345 m · 옆 0.06 까지 오차 0, 2026-11-02). Isaac 놓기 미검증
GAP_INLINE = 0.04     # [m] ±X 대안 접근 (두 퍽이 앞뒤) 의 앞 끝 - 박스 면


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Pose:
    x: float
    y: float
    yaw: float

    def tf(self, px, py):
        """base_link 좌표 (px, py) -> 지도."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + px * c - py * s, self.y + px * s + py * c

    def inv(self, wx, wy):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = wx - self.x, wy - self.y
        return dx * c + dy * s, -dx * s + dy * c

    def as_list(self):
        return [round(self.x, 4), round(self.y, 4), round(self.yaw, 4)]


def footprint_points(pose: Pose, margin=0.0, step=0.02):
    """외곽 사각형 (margin 만큼 넓힘) 의 둘레 + 안쪽 격자 점 (지도)."""
    xs = np.arange(REAR - margin, FRONT + margin + 1e-9, step)
    ys = np.arange(-HALF_W - margin, HALF_W + margin + 1e-9, step)
    xs = np.r_[xs, FRONT + margin]
    ys = np.r_[ys, HALF_W + margin]
    gx, gy = np.meshgrid(xs, ys)
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return np.c_[pose.x + gx.ravel() * c - gy.ravel() * s, pose.y + gx.ravel() * s + gy.ravel() * c]


def footprint_poly(pose: Pose, margin=0.0):
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    pts = [(REAR - margin, -HALF_W - margin), (FRONT + margin, -HALF_W - margin), (FRONT + margin, HALF_W + margin), (REAR - margin, HALF_W + margin)]
    return np.array([(pose.x + px * c - py * s, pose.y + px * s + py * c) for px, py in pts])


def _sat_overlap(a, b):
    """볼록 다각형 a, b 가 겹치는지 (분리축)."""
    for poly in (a, b):
        n = len(poly)
        for k in range(n):
            ex, ey = poly[(k + 1) % n] - poly[k]
            ax = np.array([-ey, ex])
            pa, pb = a @ ax, b @ ax
            if pa.max() < pb.min() or pb.max() < pa.min():
                return False
    return True


def footprint_clear(solids, pose: Pose, margin=0.0):
    """외곽 (margin [m] 넓힘) 이 장애물 (SDF 충돌 형상의 바닥 투영) 과 겹치지 않음. 격자가 아니라 원래 형상으로 판정."""
    fp = footprint_poly(pose, margin)
    lo, hi = fp.min(0), fp.max(0)
    for s in solids:
        q = s.footprint()
        if (q.max(0) < lo).any() or (q.min(0) > hi).any():
            continue
        if _sat_overlap(fp, q):
            return False
    return True


def clearance(solids, pose: Pose, step=0.005, limit=0.2):
    """외곽을 몇 m 까지 넓혀도 안 닿는지 (step 단위, 0 = 이미 닿음)."""
    m = 0.0
    while m < limit and footprint_clear(solids, pose, m + step):
        m += step
    return m if footprint_clear(solids, pose, 0.0) else 0.0


def pose_facing(target, face_dir, face_dist, bumper_gap, lateral=0.0):
    """물체 면 (target 에서 face_dir 쪽 face_dist) 앞에서 그 면을 보고 선 base_link 자세. 앞 끝 - 면 = bumper_gap."""
    fx, fy = face_dir
    yaw = math.atan2(-fy, -fx)  # 로봇 앞 = 면 안쪽
    d = face_dist + bumper_gap + FRONT
    lx, ly = -fy, fx  # 옆 (왼쪽 +)
    return Pose(target[0] + fx * d + lx * lateral, target[1] + fy * d + ly * lateral, yaw)


@dataclass
class Approach:
    key: str            # 예 "B1:S", "L2"
    kind: str           # box | place | home
    final: Pose
    stage: Pose
    standoff: float
    ok: bool
    note: str = ""
    extra: dict = field(default_factory=dict)


def j1_dist(pose: Pose, wx, wy):
    px, py = pose.inv(wx, wy)
    return math.hypot(px - J1_X, py), px - J1_X, py


def make_approaches(solids, gap=0.02, standoffs=(0.45, 0.35, 0.30), margin=0.05):
    """접근 자세표 (solids = 주행 높이 장애물). stage 는 외곽 + margin 이 비어야 함, 막히면 standoff 를 줄여 다시.
    final 은 대상 면에 gap 만큼 붙으므로 대상 물체는 빼고 margin 0.01 로 판정 (final 이 막히면 ok False)."""
    out = {}

    def best(key, kind, target, face_dir, face_dist, extra, gap=gap):
        fin = pose_facing(target, face_dir, face_dist, gap)
        fin_ok = footprint_clear(solids, fin, margin=min(0.01, gap / 2))
        for so in standoffs:
            st = pose_facing(target, face_dir, face_dist, so)
            if footprint_clear(solids, st, margin=margin):
                extra = dict(extra, gap=gap, final_clearance=round(clearance(solids, fin), 3), stage_clearance=round(clearance(solids, st), 3))
                return Approach(key, kind, fin, st, so, fin_ok, "" if fin_ok else "final 외곽이 막힘", extra)
        return Approach(key, kind, fin, pose_facing(target, face_dir, face_dist, standoffs[-1]), standoffs[-1], False,
                        "stage 외곽이 막힘 (standoff 를 줄여도)", extra)

    for b, (cx, cy) in START_BOXES.items():
        # ±Y 면 (두 퍽 잇는 선에 수직) 이 기본. 둘 다 안 되는 박스만 ±X 면 (두 퍽이 앞뒤로 줄 섬 = inline) 을 대안으로
        sides = {side: best(f"B{b}:{side}", "box", (cx, cy), fdir, BOX_HALF, {}) for side, fdir in (("S", (0.0, -1.0)), ("N", (0.0, 1.0)))}
        if not any(a.ok for a in sides.values()):
            # inline 은 가까운 퍽이 J1 0.12 m 아래로 오지 않게 틈 GAP_INLINE (TopPick 학습 gap 0.005 ~ 0.04 안)
            sides.update({side: best(f"B{b}:{side}", "box", (cx, cy), fdir, BOX_HALF, {}, gap=GAP_INLINE) for side, fdir in (("W", (-1.0, 0.0)), ("E", (1.0, 0.0)))})
        for side, a in sides.items():
            pk = {sx: dict(zip(("dist", "fwd", "lat"), (round(v, 4) for v in j1_dist(a.final, cx + sx * PUCK_DX, cy)))) for sx in (-1, 1)}
            far = max(v["dist"] for v in pk.values())
            a.extra = {**a.extra, "box": b, "side": side, "inline": side in ("W", "E"), "pucks": pk}
            if a.ok and far > REACH_VERTICAL:
                a.ok, a.note = False, f"먼 퍽 J1 거리 {far:.3f} > 수직 집기 {REACH_VERTICAL}"
            out[a.key] = a
    for name, p in PLACES.items():
        cx, cy = p["center"]
        a = best(name, "place", (cx, cy), p["face_dir"], p["face_dist"], {})
        cells = {}
        for gx in (-1, 0, 1):
            for gy in (-1, 0, 1):
                wx, wy = cx + gx * GRID_STEP, cy + gy * GRID_STEP
                d, fwd, lat = j1_dist(a.final, wx, wy)
                mode = "vertical" if d <= REACH_VERTICAL else ("tilt" if d <= REACH_TILT else "unreachable")
                cells[f"{gx},{gy}"] = dict(x=round(wx, 4), y=round(wy, 4), dist=round(d, 4), fwd=round(fwd, 4), lat=round(lat, 4), mode=mode)
        a.extra = {**a.extra, "tag": p["tag"], "cells": cells}
        out[name] = a
    # 출발 · 관측 · 복귀: 로봇 중심 = (734, 950), -X 를 봄 -> base_link = 중심 - CENTER_X 방향
    home = Pose(*_center_to_base(OBSERVE, math.pi), math.pi)
    corners = [home.tf(px, py) for px in (REAR, FRONT) for py in (-HALF_W, HALF_W)]
    inside = min(START_ZONE_HALF - abs(x - OBSERVE[0]) for x, _ in corners), min(START_ZONE_HALF - abs(y - OBSERVE[1]) for _, y in corners)
    out["HOME"] = Approach("HOME", "home", home, home, 0.0, footprint_clear(solids, home, margin=0.0),
                           extra={"zone_margin_x": round(inside[0], 4), "zone_margin_y": round(inside[1], 4)})
    return out


def _center_to_base(center, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return center[0] - CENTER_X * c, center[1] - CENTER_X * s


def place_order(approach: Approach, die: int):
    """눈 칸들을 놓는 순서: J1 에서 먼 칸부터 (쥔 퍽이 이미 놓은 퍽 위를 지나지 않게). -> [(칸 키, 칸 정보)]"""
    cells = approach.extra["cells"]
    keys = [f"{dx},{dy}" for dx, dy in DICE[die]]
    return sorted(((k, cells[k]) for k in keys), key=lambda kv: -kv[1]["dist"])
