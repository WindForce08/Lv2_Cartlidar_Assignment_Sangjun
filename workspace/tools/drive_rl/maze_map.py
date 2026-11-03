"""주사위 경기장 (미로) 지도: worlds/dice_mission.sdf 의 충돌체를 위에서 본 그림 + 칸 이름 (Isaac 없이, 시스템 python3).

    python3 maze_map.py                 # -> maze_map.png (칸 A1 ~ E8, 벽 · 기둥 · 상자 · 받침대, 출발 구역)
    python3 maze_map.py --out 다른.png
    python3 maze_map.py --course course.yaml   # 경로 검사 (점 · 구간마다 벽까지 여유) + course_map.png (./run.sh course)

칸 = 미로 안쪽 기둥 격자 (x = 0.367 + 0.734k, y = 0.583 + 0.734k) 사이의 0.734 m 정사각형. 열 A ~ E (x 가운데 0.734 ~ 3.670),
행 0 ~ 8 (y 가운데: 0 행 0.2915 = 아래 바깥벽 통로, 1 ~ 7 행 0.950 ~ 5.354, 8 행 6.0125 = 위 바깥벽 통로). A1 = 출발 구역 (0.734, 0.95). 팀 tools/make_maze_scene.py 의 --cell i j (원점부터 0.734 격자) 와는
반 칸 어긋나 있어 다르다 (그쪽 칸 가운데는 안쪽 벽 줄 위에 놓이기도 함).
좌표 원점 = 경기장 왼쪽 아래 기둥 중심, x 오른쪽 · y 위 (m). 칸 이름 색: 초록 = 로봇이 들어감 (가운데에서 장애물까지 ≥ 반폭 0.1676 + 0.03),
회색 = 막힘 (물체가 놓인 칸 등). 0 행 · 8 행 모서리에서 제자리 회전은 차체 직사각형으로 따로 확인함 (2026-11-02, 모두 가능).
"""

from __future__ import annotations

import argparse
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
from scipy.ndimage import distance_transform_edt

HERE = os.path.dirname(os.path.abspath(__file__))
SDF = os.path.join(HERE, "..", "..", "urdf_and_usd", "waffle_4wd_description", "worlds", "dice_mission.sdf")
CELL = 0.734  # 칸 간격 [m]
W, H = 4.404, 6.304  # 경기장 크기 [m]
COLS, ROWS = "ABCDE", 8  # 칸 열 · 행 수
CELL_X0, CELL_Y0 = 0.734, 0.95  # A1 가운데 [m]
RES = 0.02  # 점유 격자 [m]
HALF_W = 0.1676  # 차체 반폭 (바퀴 바깥)
ROBOT_R = math.hypot(0.226, 0.1676)  # 제자리 회전 외접 반경 0.2814
START = (0.734, 0.95)  # 출발 구역 가운데 (start_zone)


def _pose(el) -> list[float]:
    t = el.findtext("pose")
    return [float(v) for v in t.split()] if t else [0.0] * 6


def _rot(yaw: float, x: float, y: float) -> tuple[float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y, s * x + c * y


def load_shapes(sdf: str = SDF) -> list[dict]:
    """충돌체 (yaw 만 있는 상자 · 원기둥) 목록. 바닥 표시 (윗면 z < 0.01) 는 marking=True"""
    world = ET.parse(sdf).getroot().find("world")
    out = []
    for m in world.iter("model"):
        name = m.get("name")
        if name == "ground_plane":
            continue
        mp = _pose(m)
        for link in m.iter("link"):
            lp = _pose(link)
            for c in link.findall("collision"):
                cp = _pose(c)
                bx, by = _rot(mp[5], lp[0], lp[1])
                cx, cy = _rot(mp[5] + lp[5], cp[0], cp[1])
                x, y, z = mp[0] + bx + cx, mp[1] + by + cy, mp[2] + lp[2] + cp[2]
                yaw = mp[5] + lp[5] + cp[5]
                box, cyl = c.find(".//box/size"), c.find(".//cylinder")
                if box is not None:
                    sx, sy, sz = (float(v) for v in box.text.split())
                    out.append(dict(kind="box", x=x, y=y, yaw=yaw, a=sx, b=sy, top=z + sz / 2, bot=z - sz / 2, model=name,
                                    marking=z + sz / 2 < 0.01))
                elif cyl is not None:
                    r = float(cyl.findtext("radius"))
                    hl = float(cyl.findtext("length")) / 2
                    out.append(dict(kind="cyl", x=x, y=y, yaw=0.0, a=2 * r, b=2 * r, top=z + hl, bot=z - hl, model=name, marking=False))
    return out


def clearance(shapes: list[dict]) -> tuple[np.ndarray, float, float]:
    """장애물까지 거리 [m] 격자 (ny, nx), 원점 (ox, oy). 경기장 밖은 막힘"""
    ox, oy = -0.2, -0.2
    nx, ny = int(round((W + 0.4) / RES)), int(round((H + 0.4) / RES))
    gx, gy = ox + (np.arange(nx) + 0.5) * RES, oy + (np.arange(ny) + 0.5) * RES
    X, Y = np.meshgrid(gx, gy)
    occ = (X < 0) | (X > W) | (Y < 0) | (Y > H)
    for s in shapes:
        if s["marking"]:
            continue
        dx, dy = X - s["x"], Y - s["y"]
        pad = RES / 2  # 벽판은 3 mm 라 격자 점 사이에 끼어 빠지지 않게 반 칸씩 넓혀 칠함 (안전 쪽)
        if s["kind"] == "box":
            c, sn = math.cos(-s["yaw"]), math.sin(-s["yaw"])
            u, v = c * dx - sn * dy, sn * dx + c * dy
            occ |= (np.abs(u) <= s["a"] / 2 + pad) & (np.abs(v) <= s["b"] / 2 + pad)
        else:
            occ |= dx * dx + dy * dy <= (s["a"] / 2 + pad) ** 2
    return distance_transform_edt(~occ) * RES, ox, oy


ROW_LINES = [0.0] + [0.583 + k * CELL for k in range(ROWS)] + [H]
"""행 경계 y [m]: 아래 바깥벽 + 안쪽 기둥 줄 8 개 + 위 바깥벽. 0 행 · 8 행은 바깥벽을 따라가는 좁은 통로 (0.583 m)"""


def cell_center(name: str) -> tuple[float, float]:
    """칸 이름 (예: "A1", "c4", "B0") → 가운데 (x, y) [m]. 행 가운데 = 그 행 경계 둘의 가운데 (0 행 0.2915, 8 행 6.0125)"""
    col, row = COLS.index(name[0].upper()), int(name[1:])
    if not 0 <= row <= ROWS:
        raise ValueError(f"칸 이름은 A0 ~ {COLS[-1]}{ROWS}: {name}")
    return CELL_X0 + col * CELL, (ROW_LINES[row] + ROW_LINES[row + 1]) / 2


def clearance_at(dist: np.ndarray, ox: float, oy: float, x: float, y: float) -> float:
    r, c = int((y - oy) / RES), int((x - ox) / RES)
    return float(dist[r, c]) if 0 <= r < dist.shape[0] and 0 <= c < dist.shape[1] else 0.0


def load_course(path: str) -> list[dict]:
    """course.yaml → [{x, y, cell, stop: 물체 이름 | None, heading: rad | None}]. 칸 이름 줄 = 지나가는 점 (칸 가운데),
    {stop, x, y, face [deg]} 줄 = 정지점"""
    import yaml

    out = []
    for item in yaml.safe_load(open(path, encoding="utf-8"))["route"]:
        if isinstance(item, str):
            x, y = cell_center(item)
            out.append(dict(x=x, y=y, cell=item.upper(), stop=None, heading=None))
        else:
            out.append(dict(x=float(item["x"]), y=float(item["y"]), cell=None, stop=str(item["stop"]),
                            heading=math.radians(float(item["face"]))))
    return out


def check_course(course: list[dict], margin: float = 0.03, step: float = 0.02) -> bool:
    """점마다 · 이웃 점 사이 구간 (2 cm 간격) 마다 로봇 가운데에서 장애물까지 ≥ 반폭 + margin 인지 출력. 모두 통과면 True"""
    dist, ox, oy = clearance(load_shapes())
    need, ok, total = HALF_W + margin, True, 0.0
    for i, p in enumerate(course):
        c = clearance_at(dist, ox, oy, p["x"], p["y"])
        seg = ""
        if i + 1 < len(course):
            q = course[i + 1]
            n = max(1, int(math.hypot(q["x"] - p["x"], q["y"] - p["y"]) / step))
            seg_c = min(clearance_at(dist, ox, oy, p["x"] + (q["x"] - p["x"]) * k / n, p["y"] + (q["y"] - p["y"]) * k / n)
                        for k in range(n + 1))
            total += math.hypot(q["x"] - p["x"], q["y"] - p["y"])
            seg = f"  → 다음 점까지 최소 {seg_c:.3f}" + ("" if seg_c >= need else "  ✗ 좁음")
            ok &= seg_c >= need
        ok &= c >= need
        name = f"정지 {p['stop']} {math.degrees(p['heading']):.0f}°" if p["stop"] else p["cell"]
        print(f"{i:2d} {name:16s} ({p['x']:.3f}, {p['y']:.3f})  여유 {c:.3f}{'' if c >= need else '  ✗ 좁음'}{seg}")
    print(f"{len(course)} 점, 정지 {sum(1 for p in course if p['stop'])}, 길이 {total:.1f} m, 기준 = 가운데에서 장애물까지 ≥ {need:.3f} m → "
          + ("모두 통과" if ok else "좁은 곳 있음 (✗)"))
    return ok


def draw(out: str, course: list[dict] | None = None) -> None:
    """지도 그림. course = [{x, y, stop: 물체 이름 | None, heading: rad | None}] 이면 경로 · 정지점 (번호 · 바라보는 방향) 을 겹쳐 그림"""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    from matplotlib.transforms import Affine2D

    shapes = load_shapes()
    dist, ox, oy = clearance(shapes)
    fig, ax = plt.subplots(figsize=(7.6, 10.2), dpi=110)
    colors = {"arena": "#555555", "start_box": "#d9822b", "place": "#2b7bd9", "signal_light": "#c23b3b"}
    for s in shapes:
        key = next((k for k in colors if s["model"].startswith(k)), None)
        col = colors.get(key, "#888888")
        if s["marking"]:
            ax.add_patch(Rectangle((s["x"] - s["a"] / 2, s["y"] - s["b"] / 2), s["a"], s["b"], fill=False, ls="--", ec="#2a9d4b", lw=1.5))
        elif s["kind"] == "box":
            t = Affine2D().rotate_around(s["x"], s["y"], s["yaw"]) + ax.transData
            ax.add_patch(Rectangle((s["x"] - s["a"] / 2, s["y"] - s["b"] / 2), s["a"], s["b"], color=col, transform=t, alpha=0.85))
        else:
            ax.add_patch(Circle((s["x"], s["y"]), s["a"] / 2, color=col, alpha=0.85))
    # 물체 이름 (상자 · 받침대 · 신호등)
    named = {}
    for s in shapes:
        if s["model"] != "arena" and not s["marking"]:
            named.setdefault(s["model"], []).append((s["x"], s["y"]))
    short = lambda n: n.replace("start_box_", "box").replace("place_L2_pedestal", "L2").replace("place_", "").replace("signal_light", "signal")  # noqa: E731
    for n, pts in named.items():
        if n.startswith("place_tag") or n == "barcode_label":
            continue
        x, y = np.mean(pts, axis=0)
        ax.text(x, y, short(n), ha="center", va="center", fontsize=8, color="white", weight="bold")
    # 칸 이름 (가운데 점 + 이름)
    for c, letter in enumerate(COLS):
        for r in range(ROWS + 1):
            name = f"{letter}{r}"
            x, y = cell_center(name)
            col = "#1a8a3a" if clearance_at(dist, ox, oy, x, y) >= HALF_W + 0.03 else "#9a9a9a"
            ax.plot(x, y, marker="o", ms=3, color=col, alpha=0.5 if course else 1.0)
            ax.text(x, y + 0.13, name, ha="center", va="center", fontsize=11 if not course else 8, color=col, weight="bold",
                    alpha=0.6 if course else 1.0)
    if course:
        ax.plot([p["x"] for p in course], [p["y"] for p in course], "-", color="#7b3fbf", lw=2.0, alpha=0.75, zorder=4)
        k = 0
        for p in course:
            if p.get("stop"):
                k += 1
                h = p["heading"]
                ax.annotate("", xy=(p["x"] + 0.22 * math.cos(h), p["y"] + 0.22 * math.sin(h)), xytext=(p["x"], p["y"]),
                            arrowprops=dict(arrowstyle="-|>", color="#c0145a", lw=2.2), zorder=6)
                ax.plot(p["x"], p["y"], marker="o", ms=13, color="#c0145a", zorder=5)
                ax.text(p["x"], p["y"], str(k), ha="center", va="center", fontsize=8, color="white", weight="bold", zorder=7)
            else:
                ax.plot(p["x"], p["y"], marker="o", ms=4, color="#7b3fbf", zorder=5)
    ax.plot(*START, marker="*", ms=14, color="#2a9d4b", alpha=0.6)
    ax.text(START[0], START[1] - 0.16, "START", ha="center", fontsize=8, color="#2a9d4b", weight="bold")
    ax.set_xlim(-0.2, W + 0.2)
    ax.set_ylim(-0.2, H + 0.2)
    ax.set_aspect("equal")
    ax.set_xticks([0.0] + [0.367 + k * CELL for k in range(6)] + [W])
    ax.set_yticks([0.0] + [0.583 + k * CELL for k in range(8)] + [H])
    ax.tick_params(labelsize=7)
    ax.grid(True, color="#dddddd", lw=0.6)
    ax.set_title("dice_mission maze (top view)  cell name: green = robot fits, gray = blocked\n"
                 + ("purple = route, numbered red = stop (arrow = facing)" if course else
                    "grid lines = inner post lattice, dot = cell center"), fontsize=9)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    fig.tight_layout()
    fig.savefig(out)
    print(f"저장: {out}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=None, help="그림 파일 (기본 maze_map.png, --course 면 course_map.png)")
    ap.add_argument("--course", default=None, help="경로 파일 (course.yaml): 검사 + 경로 그림")
    a = ap.parse_args()
    if a.course:
        course = load_course(a.course)
        ok = check_course(course)
        draw(a.out or os.path.join(HERE, "course_map.png"), course)
        raise SystemExit(0 if ok else 1)
    draw(a.out or os.path.join(HERE, "maze_map.png"))
