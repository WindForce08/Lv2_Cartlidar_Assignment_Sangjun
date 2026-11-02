"""주행 비용: 부풀린 점유 격자 위 A* (8 방향) + 시야선 당기기 (꺾인 선) -> 길이 + 회전 비용.

실제 주행은 nav2 가 하고, 여기 비용은 방문 순서 · 접근 면 고르기용 (도전 C: 직선거리 대신 통과 가능한 경로).
"""
from __future__ import annotations

import heapq
import math

import numpy as np

from .arena import Grid
from .poses import Pose, wrap


def inflate(grid: Grid, radius: float) -> Grid:
    """막힌 셀을 radius 만큼 넓힘 (원판 팽창)."""
    r = int(math.ceil(radius / grid.res))
    occ = grid.occ
    out = occ.copy()
    ys, xs = np.nonzero(occ)
    offs = [(dx, dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1) if (dx * dx + dy * dy) * grid.res ** 2 <= radius ** 2]
    h, w = occ.shape
    for dx, dy in offs:
        yy, xx = ys + dy, xs + dx
        m = (yy >= 0) & (yy < h) & (xx >= 0) & (xx < w)
        out[yy[m], xx[m]] = True
    return Grid(out, grid.res, grid.ox, grid.oy)


def _nearest_free(g: Grid, ix, iy, rmax=12):
    if g.inside(ix, iy) and not g.occ[iy, ix]:
        return ix, iy
    for r in range(1, rmax + 1):
        for dx in range(-r, r + 1):
            for dy in (-r, r) if abs(dx) != r else range(-r, r + 1):
                jx, jy = ix + dx, iy + dy
                if g.inside(jx, jy) and not g.occ[jy, jx]:
                    return jx, jy
    return None


def astar(g: Grid, a, b):
    """셀 a -> b 경로 (셀 목록) 또는 None."""
    if a == b:
        return [a]
    SQ2 = math.sqrt(2)
    nb = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, SQ2), (1, -1, SQ2), (-1, 1, SQ2), (-1, -1, SQ2)]
    openh = [(0.0, 0.0, a)]
    came = {a: None}
    cost = {a: 0.0}
    while openh:
        _, c, cur = heapq.heappop(openh)
        if cur == b:
            path = [cur]
            while came[path[-1]] is not None:
                path.append(came[path[-1]])
            return path[::-1]
        if c > cost[cur] + 1e-12:
            continue
        for dx, dy, w in nb:
            n = (cur[0] + dx, cur[1] + dy)
            if not g.inside(*n) or g.occ[n[1], n[0]]:
                continue
            if dx and dy and (g.occ[cur[1], n[0]] or g.occ[n[1], cur[0]]):  # 모서리 빠져나가기 금지
                continue
            nc = c + w
            if nc < cost.get(n, 1e18):
                cost[n] = nc
                came[n] = cur
                heapq.heappush(openh, (nc + math.hypot(b[0] - n[0], b[1] - n[1]), nc, n))
    return None


def _los(g: Grid, a, b):
    """셀 a, b 사이 직선이 막힌 셀을 안 지나는지 (반 셀 간격 표본)."""
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2) + 1
    for k in range(n + 1):
        t = k / n
        x, y = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
        for ix, iy in {(math.floor(x + 0.5), math.floor(y + 0.5)), (math.floor(x), math.floor(y)), (math.ceil(x), math.ceil(y))}:
            if not g.inside(ix, iy) or g.occ[iy, ix]:
                return False
    return True


def smooth(g: Grid, cells):
    """시야선 당기기: 직선으로 갈 수 있는 점은 건너뜀."""
    if len(cells) <= 2:
        return cells
    out = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        j = len(cells) - 1
        while j > i + 1 and not _los(g, cells[i], cells[j]):
            j -= 1
        out.append(cells[j])
        i = j
    return out


class CostModel:
    """지도 + 부풀림 -> 자세 사이 비용 (m 환산). turn_m_per_rad: 회전 1 rad 를 몇 m 로 칠지 (제자리 회전 시간)."""

    def __init__(self, nav: Grid, inflate_radius=0.19, turn_m_per_rad=0.10):
        self.g = inflate(nav, inflate_radius)
        self.turn = turn_m_per_rad
        self._cache = {}

    def path(self, p: Pose, q: Pose):
        a = _nearest_free(self.g, *self.g.to_cell(p.x, p.y))
        b = _nearest_free(self.g, *self.g.to_cell(q.x, q.y))
        if a is None or b is None:
            return None
        cells = astar(self.g, a, b)
        if cells is None:
            return None
        pts = [self.g.to_world(*c) for c in smooth(self.g, cells)]
        pts[0], pts[-1] = (p.x, p.y), (q.x, q.y)
        return pts

    def cost(self, p: Pose, q: Pose):
        key = (round(p.x, 3), round(p.y, 3), round(p.yaw, 3), round(q.x, 3), round(q.y, 3), round(q.yaw, 3))
        if key in self._cache:
            return self._cache[key]
        pts = self.path(p, q)
        if pts is None:
            res = (math.inf, None)
        else:
            length, turn, h = 0.0, 0.0, p.yaw
            for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
                d = math.hypot(x1 - x0, y1 - y0)
                if d < 1e-6:
                    continue
                a = math.atan2(y1 - y0, x1 - x0)
                turn += abs(wrap(a - h))
                h = a
                length += d
            turn += abs(wrap(q.yaw - h))
            res = (length + self.turn * turn, dict(length=length, turn_rad=turn, points=[[round(x, 3), round(y, 3)] for x, y in pts]))
        self._cache[key] = res
        return res
