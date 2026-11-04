"""위치 잃음 감지 · 다시 찾기 계산(ROS 없음, numpy · cv2). loc_monitor 가 씀.
- 지도: 발제 좌표 점유 지도(maps/slam_walls.yaml) → 가장 가까운 점유 칸까지 거리장(clip m 에서 자름).
- residual(pose, pts): 라이다 끝점(base_link 기준)을 자세에 놓았을 때 거리장 값의 중앙값 — 지도에 없는 물체(퍽 · 사람 다리)가 일부 가려도 덜 흔들림.
- search(pts, center, ...): 자세 후보를 격자로 훑어 거리장 평균(clip)이 가장 작은 자세를 찾고(거친 → 고운 2단), 1등과 '다른 자리' 2등 점수를 함께 돌려줌(모호성).
  다른 자리 = 1등에서 0.3 m 넘게 또는 20° 넘게 떨어진 후보. 지도 전체 탐색은 center=None(빈 칸 위 격자)."""
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


class DistField:
    def __init__(self, map_yaml, clip=0.10):
        meta = yaml.safe_load(Path(map_yaml).read_text())
        img = cv2.imread(str(Path(map_yaml).parent / meta['image']), cv2.IMREAD_GRAYSCALE)
        occ = ((255 - img.astype(float)) / 255.0) > meta['occupied_thresh']
        free = ((255 - img.astype(float)) / 255.0) < meta['free_thresh']
        self.res, (self.ox, self.oy) = float(meta['resolution']), meta['origin'][:2]
        self.H, self.W = img.shape
        self.clip = clip
        self.dist = np.minimum(cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5) * self.res, clip).astype(np.float32)
        # 차체가 설 수 있는 칸(빈 칸 · 벽에서 0.15 m 넘게) — 지도 전체 탐색 후보
        wall_far = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5) * self.res > 0.15
        r, c = np.nonzero(free & wall_far)
        self.free_rc = (r, c)
        self.free_xy = np.stack([self.ox + (c + 0.5) * self.res, self.oy + (self.H - 1 - r + 0.5) * self.res], 1)

    def free_grid(self, step_m):
        """빈 칸을 x · y 모두 step_m 간격으로 솎음(행 순서 목록을 [::k] 로 뽑으면 한 축만 솎임)"""
        k = max(1, int(round(step_m / self.res)))
        r, c = self.free_rc
        keep = (r % k == 0) & (c % k == 0)
        return self.free_xy[keep]

    def lookup(self, xy):
        """xy (..., 2) → 거리(지도 밖은 clip)"""
        c = np.floor((xy[..., 0] - self.ox) / self.res).astype(np.int64)
        r = self.H - 1 - np.floor((xy[..., 1] - self.oy) / self.res).astype(np.int64)
        ok = (c >= 0) & (c < self.W) & (r >= 0) & (r < self.H)
        out = np.full(xy.shape[:-1], self.clip, np.float32)
        out[ok] = self.dist[r[ok], c[ok]]
        return out

    def lookup_interp(self, xy):
        """xy (..., 2) → 칸 중심 기준 쌍선형 보간 거리(지도 밖은 clip).  칸 값(0.025 m 계단)보다 작은 어긋남에 민감"""
        fx = (xy[..., 0] - self.ox) / self.res - 0.5
        fy = (self.H - 1) - ((xy[..., 1] - self.oy) / self.res - 0.5)
        c0, r0 = np.floor(fx).astype(np.int64), np.floor(fy).astype(np.int64)
        ax, ay = fx - c0, fy - r0
        ok = (c0 >= 0) & (c0 + 1 < self.W) & (r0 >= 0) & (r0 + 1 < self.H)
        out = np.full(xy.shape[:-1], self.clip, np.float32)
        c, r, x, y, g = c0[ok], r0[ok], ax[ok], ay[ok], self.dist
        out[ok] = g[r, c] * (1 - x) * (1 - y) + g[r, c + 1] * x * (1 - y) + g[r + 1, c] * (1 - x) * y + g[r + 1, c + 1] * x * y
        return out


def place(poses, pts):
    """poses (N, 3) · pts (M, 2) base_link 기준 → (N, M, 2) 지도 좌표"""
    c, s = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
    return np.stack([poses[:, 0:1] + c * pts[None, :, 0] - s * pts[None, :, 1],
                     poses[:, 1:2] + s * pts[None, :, 0] + c * pts[None, :, 1]], -1)


def residual(field, pose, pts):
    return float(np.median(field.lookup(place(np.asarray([pose], float), pts))[0]))


def residual_multi(field, pose_pts, mode='cell_median'):
    """[(자세, 점), …] — 스캔마다 자기 stamp 의 자세로 놓은 끝점을 모아 중앙값(회전 중 한 자세로 놓으면 끝점이 밀림).
    mode: 'cell_median'(칸 값 · 중앙값) · 'interp_p75'(쌍선형 보간 · 75 백분위, 모의 녹화 비교에서 가장 잘 가름)"""
    look = field.lookup_interp if mode == 'interp_p75' else field.lookup
    d = np.concatenate([look(place(np.asarray([p], float), q))[0] for p, q in pose_pts if len(q)])
    return float(np.percentile(d, 75)) if mode == 'interp_p75' else float(np.median(d))


def scores(field, poses, pts, chunk=4000):
    out = np.empty(len(poses), np.float32)
    for i in range(0, len(poses), chunk):
        out[i:i + chunk] = field.lookup(place(poses[i:i + chunk], pts)).mean(1)
    return out


def _grid(center, lin, lin_step, ang, ang_step):
    xs = np.arange(-lin, lin + 1e-9, lin_step)
    ys = np.arange(-lin, lin + 1e-9, lin_step)
    yaws = np.arange(-ang, ang + 1e-9, ang_step)
    X, Y, A = np.meshgrid(xs, ys, yaws, indexing='ij')
    return np.stack([center[0] + X.ravel(), center[1] + Y.ravel(), center[2] + A.ravel()], 1)


def _refine(field, pts, pose):
    best = np.asarray(pose, float)
    for lin, ls, ang, as_ in ((0.06, 0.01, math.radians(4), math.radians(0.5)), (0.012, 0.003, math.radians(0.6), math.radians(0.15))):
        cand = _grid(best, lin, ls, ang, as_)
        sc = scores(field, cand, pts)
        best = cand[int(np.argmin(sc))]
        bs = float(sc.min())
    return best, bs


def search(field, pts, center=None, lin=0.5, ang=math.radians(30), n_refine=8, sep_xy=0.3, sep_yaw=math.radians(20)):
    """돌려줌: dict(best=(x, y, yaw), score, second_score, second=(…) | None, n_cand)"""
    if center is None:                                      # 지도 전체: 빈 칸 0.1 m 격자 × 5°
        xy = field.free_grid(0.10)
        yaws = np.arange(-math.pi, math.pi, math.radians(5))
        cand = np.concatenate([np.column_stack([xy, np.full(len(xy), a)]) for a in yaws])
    else:
        cand = _grid(center, lin, 0.04, ang, math.radians(3))
    sc = scores(field, cand, pts)
    order = np.argsort(sc)
    picked = []                                             # 서로 다른 자리 상위 n_refine 개
    for i in order:
        p = cand[i]
        if all(math.hypot(p[0] - q[0], p[1] - q[1]) > sep_xy or abs(math.atan2(math.sin(p[2] - q[2]), math.cos(p[2] - q[2]))) > sep_yaw for q in picked):
            picked.append(p)
        if len(picked) >= n_refine:
            break
    refined = sorted((_refine(field, pts, p) for p in picked), key=lambda r: r[1])
    best, bs = refined[0]
    second = next(((p, s) for p, s in refined[1:]
                   if math.hypot(p[0] - best[0], p[1] - best[1]) > sep_xy or abs(math.atan2(math.sin(p[2] - best[2]), math.cos(p[2] - best[2]))) > sep_yaw), None)
    return {'best': tuple(float(v) for v in best), 'score': bs, 'second': None if second is None else tuple(float(v) for v in second[0]),
            'second_score': None if second is None else second[1], 'n_cand': len(cand)}


def T2d_from_tf(tx, ty, qx, qy, qz, qw):
    """3차원 변환(위치 · 쿼터니언) → 바닥 평면 2×3 [A | t]. 라이다가 거꾸로(roll π) 달린 경우 A 는 반사(det −1)가 됨 — yaw 만 뽑으면 좌우가 뒤집힘."""
    R = np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw)],
                  [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz)]])
    return np.column_stack([R, [tx, ty]])


def T2d_compose(a, b):
    """2×3 끼리 a ∘ b"""
    return np.column_stack([a[:, :2] @ b[:, :2], a[:, :2] @ b[:, 2] + a[:, 2]])


def scan_points(ranges, angle_min, angle_inc, T_base_laser, r_min=0.15, r_max=3.5, n_max=180):
    """LaserScan 값 → base_link 기준 점 (M, 2). T_base_laser = 2×3 [A | t](T2d_from_tf). 고르게 n_max 개로 줄임."""
    r = np.asarray(ranges, float)
    a = angle_min + angle_inc * np.arange(len(r))
    ok = np.isfinite(r) & (r > r_min) & (r < r_max)
    if not ok.any():
        return np.zeros((0, 2))
    xy = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], 1)
    T = np.asarray(T_base_laser, float)
    p = xy @ T[:, :2].T + T[:, 2]
    if len(p) > n_max:
        p = p[np.linspace(0, len(p) - 1, n_max).astype(int)]
    return p
