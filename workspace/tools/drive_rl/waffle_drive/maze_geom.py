"""미로 형상 계산 (torch, Isaac 없이 돎): 차체 둘레 → 장애물까지 거리 (부딪힘 · 벽 가까움), 2D 라이다, 보기용 미로 USD.

장애물 = maze_map.load_shapes() (worlds/dice_mission.sdf 충돌체 287 개, 모두 yaw 만 있는 상자). 좌표 = 미로 좌표
(경기장 왼쪽 아래 기둥 중심 = 환경 원점, x 오른쪽 · y 위 [m]). 미로는 Isaac 에 보기용으로만 놓고 (충돌 없음)
부딪힘은 여기서 지도로 판단한다: 물리 충돌체 287 개 × 환경 수를 PhysX 에 올리지 않아 가볍고, 닿는 순간 에피소드를 끝내므로 결과가 같다.

- 차체 = base_link 기준 직사각형 x −0.2135 ~ 0.1328, y ±0.1676 (바퀴 바깥 포함). 둘레를 1 cm 간격 점으로 보고
  점마다 가장 가까운 상자까지 거리 (안이면 음수) 를 잰다. 3 mm 벽판이 둘레를 가로지르면 어떤 점이든 0.35 cm 안에 들어온다.
- 라이다 = 로봇 가운데에서 360° 를 n 개 광선으로 (실물 LDS-03 두 대를 합친 것으로 단순화 ★). 윗면이 스캔면 (8.2 cm) 보다
  낮은 물체 (퍽 상자 · 받침대 7 cm, 바닥 턱) 는 안 보인다 = 실물과 같음.
"""

from __future__ import annotations

import math
import os

import torch

from maze_map import H, W, load_shapes

FOOT_X = (-0.2135, 0.1328)  # 차체 뒤 · 앞 끝 (base_link 기준, B안 URDF 충돌체) [m]
FOOT_Y = 0.1676  # 차체 반폭 (바퀴 바깥) [m]
SCAN_Z = 0.082  # 라이다 스캔면 높이 [m]
OBJECT_PREFIX = ("start_box", "place_")  # 퍽 상자 · 놓는 자리 (부딪힘을 벽과 따로 셈)


def footprint_points(spacing: float = 0.01) -> torch.Tensor:
    """차체 직사각형 둘레 점 (P, 2), base_link 기준"""
    (x0, x1), y1 = FOOT_X, FOOT_Y
    nx, ny = math.ceil((x1 - x0) / spacing), math.ceil(2 * y1 / spacing)
    xs, ys = torch.linspace(x0, x1, nx + 1), torch.linspace(-y1, y1, ny + 1)
    return torch.cat((
        torch.stack((xs, torch.full_like(xs, -y1)), 1), torch.stack((xs, torch.full_like(xs, y1)), 1),
        torch.stack((torch.full_like(ys[1:-1], x0), ys[1:-1]), 1), torch.stack((torch.full_like(ys[1:-1], x1), ys[1:-1]), 1),
    ))


class MazeGeom:
    """장애물 상자 묶음 (K 개) 과 차체 · 라이다 계산. 모든 입력 · 출력은 미로 좌표 torch 텐서 (환경 수 N 이 앞 차원)."""

    def __init__(self, device: str = "cpu", shapes: list[dict] | None = None, foot_spacing: float = 0.01):
        shapes = [s for s in (shapes if shapes is not None else load_shapes()) if not s["marking"]]
        if any(s["kind"] != "box" for s in shapes):
            raise ValueError("원기둥 충돌체는 아직 지원하지 않음 (지금 경기장은 모두 상자)")
        f = lambda v: torch.tensor(v, dtype=torch.float32, device=device)  # noqa: E731
        self.shapes = shapes
        self.center = f([[s["x"], s["y"]] for s in shapes])  # (K, 2)
        self.half = f([[s["a"] / 2, s["b"] / 2] for s in shapes])  # (K, 2)
        yaw = f([s["yaw"] for s in shapes])
        self.cos, self.sin = torch.cos(yaw), torch.sin(yaw)
        self.is_object = torch.tensor([s["model"].startswith(OBJECT_PREFIX) for s in shapes], device=device)
        self.scan = torch.tensor([s["top"] > SCAN_Z for s in shapes], device=device)
        self.foot = footprint_points(foot_spacing).to(device)  # (P, 2)

    def _local(self, p: torch.Tensor, idx: torch.Tensor | None = None) -> tuple[torch.Tensor, torch.Tensor]:
        """점 p (..., 2) → 상자마다의 상자 좌표 (u, v), 모양 (..., K)"""
        c, cs, sn = (self.center, self.cos, self.sin) if idx is None else (self.center[idx], self.cos[idx], self.sin[idx])
        dx, dy = p[..., 0:1] - c[:, 0], p[..., 1:2] - c[:, 1]
        return cs * dx + sn * dy, -sn * dx + cs * dy

    def distance(self, p: torch.Tensor) -> torch.Tensor:
        """점 p (..., 2) → 상자마다 부호 거리 (..., K) [m] (상자 안이면 음수)"""
        u, v = self._local(p)
        qx, qy = u.abs() - self.half[:, 0], v.abs() - self.half[:, 1]
        return torch.hypot(qx.clamp(min=0), qy.clamp(min=0)) + torch.maximum(qx, qy).clamp(max=0)

    def footprint_world(self, pos: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
        """로봇 위치 (N, 2) · 방향 (N,) → 차체 둘레 점 (N, P, 2)"""
        c, s = torch.cos(yaw)[:, None], torch.sin(yaw)[:, None]
        fx, fy = self.foot[:, 0], self.foot[:, 1]
        return torch.stack((pos[:, 0:1] + c * fx - s * fy, pos[:, 1:2] + s * fx + c * fy), dim=-1)

    def footprint_clearance(self, pos: torch.Tensor, yaw: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """차체 둘레에서 (벽 · 신호등 · 바코드, 퍽 상자 · 놓는 자리) 까지 가장 짧은 거리 (N,), (N,) [m]. 경기장 밖으로 나가도 벽으로 침"""
        pts = self.footprint_world(pos, yaw)
        d = self.distance(pts).amin(dim=1)  # (N, K)
        big = torch.full_like(d[:, :1], 9.0)
        wall = torch.where(self.is_object, big, d).amin(dim=1)
        obj = torch.where(self.is_object, d, big).amin(dim=1)
        out = torch.minimum(torch.minimum(pts[..., 0], W - pts[..., 0]), torch.minimum(pts[..., 1], H - pts[..., 1])).amin(dim=1)
        return torch.minimum(wall, out), obj

    def lidar(self, pos: torch.Tensor, yaw: torch.Tensor, beams: int, max_range: float) -> torch.Tensor:
        """로봇 가운데에서 몸체 앞 (0) 부터 반시계로 beams 개 광선 → 거리 (N, beams) [m], 최대 max_range"""
        idx = self.scan.nonzero().squeeze(1)
        ang = yaw[:, None] + torch.arange(beams, device=pos.device) * (2 * math.pi / beams)  # (N, B)
        dx, dy = torch.cos(ang)[..., None], torch.sin(ang)[..., None]  # (N, B, 1)
        ou, ov = self._local(pos, idx)  # (N, K)
        cs, sn, h = self.cos[idx], self.sin[idx], self.half[idx]
        du, dv = cs * dx + sn * dy, -sn * dx + cs * dy  # (N, B, K)
        eps = 1e-9
        iu = 1.0 / torch.where(du.abs() < eps, torch.full_like(du, eps), du)
        iv = 1.0 / torch.where(dv.abs() < eps, torch.full_like(dv, eps), dv)
        tu1, tu2 = (-h[:, 0] - ou[:, None]) * iu, (h[:, 0] - ou[:, None]) * iu
        tv1, tv2 = (-h[:, 1] - ov[:, None]) * iv, (h[:, 1] - ov[:, None]) * iv
        tmin = torch.maximum(torch.minimum(tu1, tu2), torch.minimum(tv1, tv2))
        tmax = torch.minimum(torch.maximum(tu1, tu2), torch.maximum(tv1, tv2))
        hit = tmax >= tmin.clamp(min=0)
        t = torch.where(hit, tmin.clamp(min=0), torch.full_like(tmin, max_range))
        return t.amin(dim=-1).clamp(max=max_range)


# ---------------- 보기용 미로 USD (충돌 · 물리 없음) ----------------
_COLORS = {"arena": (0.36, 0.37, 0.40), "start_box": (0.90, 0.52, 0.15), "place": (0.17, 0.48, 0.85),
           "signal_light": (0.80, 0.18, 0.18), "barcode_label": (0.05, 0.05, 0.05)}
_LOW = (0.62, 0.63, 0.66)  # 바닥 턱 (윗면 < 5 cm 인 경기장 부품)
_FLOOR = (0.93, 0.92, 0.88)
_ROUTE, _STOP, _FACE = (0.48, 0.25, 0.75), (0.75, 0.08, 0.35), (1.0, 0.80, 0.10)


def _cube(name: str, x: float, y: float, z0: float, z1: float, a: float, b: float, yaw: float, rgb) -> str:
    h = max(z1 - z0, 0.002)
    return (f'    def Cube "{name}"\n    {{\n        double size = 1\n'
            f"        double3 xformOp:translate = ({x:.5f}, {y:.5f}, {z0 + h / 2:.5f})\n"
            f"        float xformOp:rotateZ = {math.degrees(yaw):.4f}\n"
            f"        float3 xformOp:scale = ({a:.5f}, {b:.5f}, {h:.5f})\n"
            '        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateZ", "xformOp:scale"]\n'
            f"        color3f[] primvars:displayColor = [({rgb[0]}, {rgb[1]}, {rgb[2]})]\n    }}\n")


def maze_usda(course: list[dict] | None = None, shapes: list[dict] | None = None) -> str:
    """미로 (바닥 · 벽 · 기둥 · 상자 · 받침대 · 신호등) + 경로 점 (보라 = 지나가는 점, 자홍 = 정지점, 노랑 막대 = 바라볼 방향)"""
    shapes = [s for s in (shapes if shapes is not None else load_shapes()) if not s["marking"]]
    parts = ['#usda 1.0\n(\n    defaultPrim = "Maze"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\ndef Xform "Maze"\n{\n',
             _cube("floor", W / 2, H / 2, -0.001, 0.001, W, H, 0.0, _FLOOR)]
    for i, s in enumerate(shapes):
        key = next((k for k in _COLORS if s["model"].startswith(k)), "arena")
        rgb = _LOW if key == "arena" and s["top"] < 0.05 else _COLORS[key]
        parts.append(_cube(f"s{i:03d}", s["x"], s["y"], max(s.get("bot", 0.0), 0.0), s["top"], s["a"], s["b"], s["yaw"], rgb))
    for i, p in enumerate(course or []):
        if p["stop"]:
            parts.append(_cube(f"wp{i:02d}", p["x"], p["y"], 0.001, 0.004, 0.08, 0.08, 0.0, _STOP))
            h = p["heading"]
            parts.append(_cube(f"wp{i:02d}_face", p["x"] + 0.09 * math.cos(h), p["y"] + 0.09 * math.sin(h), 0.001, 0.005,
                               0.12, 0.025, h, _FACE))
        else:
            parts.append(_cube(f"wp{i:02d}", p["x"], p["y"], 0.001, 0.003, 0.05, 0.05, 0.0, _ROUTE))
    parts.append("}\n")
    return "".join(parts)


def write_maze_usda(path: str, course: list[dict] | None = None) -> str:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(maze_usda(course))
    return path
