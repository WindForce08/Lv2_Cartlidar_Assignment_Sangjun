"""미로 형상 (maze_geom.py) 시험 (Isaac 없이, Isaac Lab venv 의 python): ./run.sh test"""

import math
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)

import torch  # noqa: E402

from maze_map import load_course  # noqa: E402
from waffle_drive.maze_geom import FOOT_X, FOOT_Y, MazeGeom, maze_usda  # noqa: E402

G = MazeGeom("cpu")
COURSE = load_course(os.path.join(HERE, "course.yaml"))
CHECKS = []


def check(name):
    def wrap(f):
        CHECKS.append((name, f))
        return f
    return wrap


def t(*v):
    return torch.tensor(v, dtype=torch.float32)


def start_poses():
    """경로 점마다 학습 시작 자세 (흔들지 않은 것): 정지점은 바라볼 방향, 나머지는 다음 점 쪽"""
    pos, yaw = [], []
    for i, p in enumerate(COURSE[:-1]):
        q = COURSE[i + 1]
        pos.append((p["x"], p["y"]))
        yaw.append(p["heading"] if p["stop"] else math.atan2(q["y"] - p["y"], q["x"] - p["x"]))
    return torch.tensor(pos), torch.tensor(yaw)


@check("경로 시작 자세 58 개 모두 벽 · 물체에서 2 cm 넘게 떨어짐 (재설정의 대체 자세)")
def _():
    pos, yaw = start_poses()
    w, o = G.footprint_clearance(pos, yaw)
    c = torch.minimum(w, o)
    assert bool((c > 0.02).all()), [(i, round(float(v), 3)) for i, v in enumerate(c) if v <= 0.02]


@check("정지점은 물체 면에서 10.75 cm 앞 (차체 앞 끝 기준. 경로는 예전 앞끝 0.1403 기준 10 cm 로 정함)")
def _():
    st = [p for p in COURSE if p["stop"] and p["stop"] != "signal"]
    _, o = G.footprint_clearance(torch.tensor([(p["x"], p["y"]) for p in st]), torch.tensor([p["heading"] for p in st]))
    assert bool(((o - 0.1075).abs() < 0.002).all()), o


@check("벽판 (3 mm) 을 차체 둘레가 가로지르면 닿음으로 잡힘")
def _():
    # A1 과 B1 사이 벽판 (x 1.101, y 0.598 ~ 1.302) 에 차체 앞을 1 mm 씩 밀어 넣음
    hits = []
    for dx in (0.0, 0.001, 0.002, 0.004, 0.008):
        x = 1.101 - 0.0015 - FOOT_X[1] + dx  # 앞 끝이 벽 면에 닿는 자리 + dx
        w, _ = G.footprint_clearance(t([x, 0.95]), t(0.0))
        hits.append(float(w))
    assert hits[0] < 0.005 and all(h < 0.005 for h in hits[1:]), hits
    w, _ = G.footprint_clearance(t([1.101 - 0.0015 - FOOT_X[1] - 0.02, 0.95]), t(0.0))
    assert 0.015 < float(w) < 0.025, float(w)


@check("경기장 밖으로 나가면 벽으로 침")
def _():
    w, _ = G.footprint_clearance(t([0.1, 3.0]), t(0.0))
    assert float(w) < 0.0, float(w)


@check("라이다: A1 에서 왼쪽 (신호등) 0.58 m, 뒤 (벽판) 0.41 m, 퍽 상자 (7 cm) 는 안 보임")
def _():
    L = G.lidar(t([0.691, 0.95]), t(math.pi), 36, 2.0)[0]
    assert abs(float(L[0]) - 0.581) < 0.005 and abs(float(L[18]) - 0.4085) < 0.005, L
    # box1 앞 정지점에서 앞을 보면 상자 (1.468, 높이 7 cm) 너머 벽까지
    L = G.lidar(t([1.8108, 0.95]), t(math.pi), 36, 2.0)[0]
    assert float(L[0]) > 0.5, float(L[0])


@check("라이다는 max_range 로 잘림 · 몸체 앞이 0 번 광선")
def _():
    L = G.lidar(t([2.202, 0.95], [2.202, 0.95]), t(0.0, math.pi / 2), 8, 0.3)
    assert float(L.max()) <= 0.3 + 1e-6
    full = G.lidar(t([2.202, 0.95], [2.202, 0.95]), t(0.0, math.pi / 2), 8, 9.0)
    assert torch.allclose(full[0, 2], full[1, 0]), full  # 0 rad 의 왼쪽 (+y) = 90° 로 돈 로봇의 앞


@check("미로 USD 글: 상자 287 + 바닥 + 경로 점")
def _():
    s = maze_usda(COURSE)
    n_stop = sum(1 for p in COURSE if p["stop"])
    assert s.count('def Cube "') == 287 + 1 + len(COURSE) + n_stop, s.count('def Cube "')
    assert "Physics" not in s  # 보기용 (충돌 없음)


@check("차체 직사각형 크기")
def _():
    f = G.foot
    assert abs(float(f[:, 0].min()) - FOOT_X[0]) < 1e-6 and abs(float(f[:, 0].max()) - FOOT_X[1]) < 1e-6
    assert abs(float(f[:, 1].abs().max()) - FOOT_Y) < 1e-6
    gaps = torch.cdist(f, f).fill_diagonal_(9).amin(dim=1)
    assert float(gaps.max()) <= 0.0101, float(gaps.max())


if __name__ == "__main__":
    bad = 0
    for name, f in CHECKS:
        try:
            f()
            print(f"통과  {name}")
        except AssertionError as e:
            bad += 1
            print(f"실패  {name}: {e}")
    print(f"{len(CHECKS) - bad}/{len(CHECKS)} 통과")
    raise SystemExit(1 if bad else 0)
