"""경기장 (발제 v5 주사위 맵) -> 정적 점유 지도 2 장.

원본: urdf_and_usd/waffle_4wd_description/worlds/dice_mission.sdf (generate_world.py 가 발제 좌표 mm / 1000 으로 만든 것,
원점 = 왼쪽 아래 기둥 중심, +X 오른쪽, +Y 위, 바닥 Z = 0). 충돌 상자 (box · cylinder) 를 그대로 바닥에 투영한다.

  map_nav   주행 계획용: 바닥 위 높이 [0, nav_z_max] 와 겹치는 모든 것 (벽판 · 기둥 · 박스 70 mm · 중앙 받침대 · 신호등 · 바코드 받침)
  map_amcl  AMCL 용: LiDAR 스캔면 (바닥 위 lidar_z, LDS-03 base_link z 0.0755 + 0.00675 = 0.082 m) 을 지나는 것만.
            박스 · 받침대 (높이 0.07) 는 스캔면 아래라 LiDAR 에 안 보임 -> AMCL 지도에 넣으면 없는 벽을 찾다가 위치가 틀어진다.
"""
from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np

LIDAR_Z = 0.0755 + 0.00675  # [m] 바닥 위 스캔면 (sensors.json lds03 pos z + root_height)
ARENA_W, ARENA_H = 4.464, 6.364  # 발제 경기장 (mm 4464 x 6364), 원점 = 왼쪽 아래 기둥 중심


@dataclass
class Solid:
    """바닥에 선 볼록 물체 (회전 없는 상자 또는 원기둥, 월드 좌표)."""
    name: str
    x: float
    y: float
    z0: float
    z1: float
    sx: float = 0.0  # 상자 크기 (원기둥이면 0)
    sy: float = 0.0
    r: float = 0.0   # 원기둥 반지름
    yaw: float = 0.0

    def footprint(self, n=16):
        if self.r > 0:
            a = np.linspace(0, 2 * math.pi, n, endpoint=False)
            return np.c_[self.x + self.r * np.cos(a), self.y + self.r * np.sin(a)]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        p = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * [self.sx / 2, self.sy / 2]
        return np.c_[self.x + p[:, 0] * c - p[:, 1] * s, self.y + p[:, 0] * s + p[:, 1] * c]


def _pose(s):
    v = [float(t) for t in (s or "0 0 0 0 0 0").split()]
    return v + [0.0] * (6 - len(v))


def load_sdf(path):
    """SDF 의 static 모델 충돌 형상 -> Solid 목록 (모델 pose + 충돌 pose, 요만 고려)."""
    root = ET.parse(path).getroot()
    out = []
    for m in root.iter("model"):
        if m.get("name") == "ground_plane":
            continue
        mp = _pose(m.findtext("pose"))
        for c in m.iter("collision"):
            cp = _pose(c.findtext("pose"))
            g = c.find("geometry")[0]
            cy, sy_ = math.cos(mp[5]), math.sin(mp[5])
            x = mp[0] + cp[0] * cy - cp[1] * sy_
            y = mp[1] + cp[0] * sy_ + cp[1] * cy
            z = mp[2] + cp[2]
            name = f"{m.get('name')}/{c.get('name')}"
            if g.tag == "box":
                sx, sy, sz = (float(t) for t in g.findtext("size").split())
                out.append(Solid(name, x, y, z - sz / 2, z + sz / 2, sx=sx, sy=sy, yaw=mp[5] + cp[5]))
            elif g.tag == "cylinder":
                r, l = float(g.findtext("radius")), float(g.findtext("length"))
                out.append(Solid(name, x, y, z - l / 2, z + l / 2, r=r))
    return out


@dataclass
class Grid:
    """점유 격자: occ[iy, ix] (True = 막힘). 셀 (ix, iy) 의 가운데 = origin + (ix + 0.5, iy + 0.5) * res."""
    occ: np.ndarray
    res: float
    ox: float
    oy: float

    @property
    def shape(self):
        return self.occ.shape

    def to_cell(self, x, y):
        return int(math.floor((x - self.ox) / self.res)), int(math.floor((y - self.oy) / self.res))

    def to_world(self, ix, iy):
        return self.ox + (ix + 0.5) * self.res, self.oy + (iy + 0.5) * self.res

    def inside(self, ix, iy):
        return 0 <= iy < self.occ.shape[0] and 0 <= ix < self.occ.shape[1]

    def blocked(self, x, y):
        ix, iy = self.to_cell(x, y)
        return (not self.inside(ix, iy)) or bool(self.occ[iy, ix])


def _fill_polygon(occ, poly, res, ox, oy):
    """볼록 다각형 안 (셀 가운데 기준, 반 셀 넓혀 얇은 벽판 3 mm 도 한 셀은 막힘)."""
    xs, ys = poly[:, 0], poly[:, 1]
    i0, i1 = int(math.floor((xs.min() - ox) / res)) - 1, int(math.ceil((xs.max() - ox) / res)) + 1
    j0, j1 = int(math.floor((ys.min() - oy) / res)) - 1, int(math.ceil((ys.max() - oy) / res)) + 1
    i0, j0 = max(i0, 0), max(j0, 0)
    i1, j1 = min(i1, occ.shape[1] - 1), min(j1, occ.shape[0] - 1)
    if i1 < i0 or j1 < j0:
        return
    gx, gy = np.meshgrid(ox + (np.arange(i0, i1 + 1) + 0.5) * res, oy + (np.arange(j0, j1 + 1) + 0.5) * res)
    inside = np.ones_like(gx, dtype=bool)
    n = len(poly)
    area = sum(poly[k, 0] * poly[(k + 1) % n, 1] - poly[(k + 1) % n, 0] * poly[k, 1] for k in range(n))
    sgn = 1.0 if area > 0 else -1.0
    for k in range(n):
        ax, ay = poly[k]
        bx, by = poly[(k + 1) % n]
        ex, ey = bx - ax, by - ay
        ln = math.hypot(ex, ey) or 1.0
        # 변 바깥쪽으로 반 셀 (res / 2) 넓힘
        d = sgn * (ex * (gy - ay) - ey * (gx - ax)) / ln
        inside &= d >= -res / 2
    occ[j0:j1 + 1, i0:i1 + 1] |= inside


def rasterize(solids, res=0.025, margin=0.15, zmin=0.0, zmax=10.0, at_z=None):
    """solids -> Grid. at_z 를 주면 그 높이를 지나는 것만, 아니면 [zmin, zmax] 와 겹치는 것."""
    ox, oy = -margin, -margin
    nx = int(math.ceil((ARENA_W + 2 * margin) / res))
    ny = int(math.ceil((ARENA_H + 2 * margin) / res))
    occ = np.zeros((ny, nx), dtype=bool)
    for s in solids:
        if at_z is not None:
            if not (s.z0 <= at_z <= s.z1):
                continue
        elif s.z1 < zmin or s.z0 > zmax:
            continue
        _fill_polygon(occ, s.footprint(), res, ox, oy)
    return Grid(occ, res, ox, oy)


def write_map(grid: Grid, path_noext, comment=""):
    """Nav2 map_server 형식: PGM (0 = 막힘, 254 = 빈 곳) + YAML. PGM 첫 줄이 위 (지도 +Y 끝)."""
    img = np.where(grid.occ, 0, 254).astype(np.uint8)[::-1]
    with open(path_noext + ".pgm", "wb") as f:
        f.write(f"P5\n# {comment}\n{img.shape[1]} {img.shape[0]}\n255\n".encode())
        f.write(img.tobytes())
    with open(path_noext + ".yaml", "w", encoding="utf-8") as f:
        f.write(f"# {comment}\n")
        f.write(f"image: {os.path.basename(path_noext)}.pgm\nmode: trinary\nresolution: {grid.res}\n")
        f.write(f"origin: [{grid.ox:.4f}, {grid.oy:.4f}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")


def build_maps(sdf_path, res=0.025, nav_z_max=0.45):
    solids = load_sdf(sdf_path)
    nav = rasterize(solids, res=res, zmin=0.002, zmax=nav_z_max)  # 출발 구역 표시 (두께 1 mm) 는 뺌
    amcl = rasterize(solids, res=res, at_z=LIDAR_Z)
    return solids, nav, amcl
