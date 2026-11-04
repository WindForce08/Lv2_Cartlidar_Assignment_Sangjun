import math
import os

import yaml

from mission_planner.mission_lines import expect_cells, to_lines
from mission_planner.poses import CENTER_X, J1_X, puck_after_turn
from mission_planner.planner import plan

PROB = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), "..", "config", "problem_q1_example.yaml"), encoding="utf-8"))
KNOWN = {"goto", "approach", "skill", "verify", "wait"}


def test_puck_after_turn():
    # 회전 없으면 그대로. ±Y 접근 퍽 (J1 앞 0.2665 · 옆 +0.04) 을 +17.06° 돌면 J1 은 퍽 줄 위 -> J1→퍽 = 처음 앞 방향 (-17.06°),
    # 거리는 J1 이 로봇 중심 둘레로 뒤로 (J1_X - CENTER_X)(1 - cos) = 6 mm 물러난 만큼 늘어남
    x, y = puck_after_turn(0.2, 0.0, 0.0)
    assert abs(x - (J1_X + 0.2)) < 1e-12 and abs(y) < 1e-12
    x, y = puck_after_turn(0.2665, 0.04, 17.06)
    assert abs(math.degrees(math.atan2(y, x - J1_X)) + 17.06) < 0.1
    assert abs(math.hypot(x - J1_X, y) - (0.2665 + (J1_X - CENTER_X) * (1 - math.cos(math.radians(17.06))))) < 0.001


def test_expect_cells():
    assert expect_cells(1, "R") == "000010000"
    assert expect_cells(2, "Y") == "300000003"        # 위 왼쪽 · 아래 오른쪽 (지도 +Y 위)
    assert expect_cells(3, "G") == "200020002"
    assert expect_cells(6, "R") == "101101101"        # 왼쪽 · 오른쪽 열


def test_lines_q1(world):
    for mode in ("per_location", "single_load"):
        p = plan(world, PROB, mode)
        lines, notes = to_lines(world, p)
        types = [ln.split()[0] for ln in lines]
        assert set(types) <= KNOWN
        assert types[-1] == "wait" and lines[-2].startswith("goto")
        loads = [ln.split()[2] for ln in lines if ln.startswith("skill manipulator pallet_load")]
        unloads = [ln.split()[2] for ln in lines if ln.startswith("skill manipulator pallet_unload")]
        places = [ln.split()[2] for ln in lines if ln.startswith("skill manipulator place_")]
        assert len(loads) == len(unloads) == len(places) == 12
        assert all(x.endswith("_tilt") for x in loads + unloads + places)          # 모두 45°
        assert sorted(x.replace("pallet_load_", "") for x in loads) == sorted(x.replace("pallet_unload_", "") for x in unloads)
        assert len([t for t in types if t == "verify"]) == 3
        assert sum("toppick_tilt" in ln for ln in lines) == 12
        # 집기 전 타겟체크자세 (#82): toppick_tilt 바로 앞마다 target_look:base_footprint:x:y:z, 예상 퍽은 J1 45° 범위 안
        for k, ln in enumerate(lines):
            if "toppick_tilt" in ln:
                f = lines[k - 1].split()[2].split(":")
                assert f[0] == "target_look" and f[1] == "base_footprint" and len(f) == 5
                x, y, z = (float(v) for v in f[2:])
                assert 0.12 <= math.hypot(x - J1_X, y) <= 0.35 and abs(z - 0.10) < 1e-9
        turns = [ln.split()[2] for ln in lines if ln.startswith("skill drive")]
        n_box = sum(s["shape"] == "box" and s["turn_deg"] != 0.0 for s in p["steps"] if s["type"] == "pick")
        assert len(turns) == 2 * n_box and turns.count("turn_0") == n_box            # 돌고 -> 집고 -> 되돌림
        assert set(turns) <= {"turn_0", "turn_p17", "turn_m17"}
        for ln in lines:                                   # mission_node checkStep 과 같은 인자 수
            a = ln.split()
            if a[0] == "goto":
                assert len(a) == 4 and all(float(v) == float(v) for v in a[1:])
            if a[0] == "approach":
                assert float(a[1]) >= 0.13
            if a[0] == "verify":
                assert len(a) == 7 and len(a[5]) == 9 and set(a[5]) <= set("0123") and a[6] in "012"
