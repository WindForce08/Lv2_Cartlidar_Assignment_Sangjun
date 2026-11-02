import os

import yaml

from mission_planner.mission_lines import expect_cells, to_lines
from mission_planner.planner import plan

PROB = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), "..", "config", "problem_q1_example.yaml"), encoding="utf-8"))
KNOWN = {"goto", "approach", "pick_target", "skill", "verify", "wait"}


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
        # 기울여 놓을 칸 (L2 가운데 · 먼 줄) 의 퍽은 팔레트에서 꺼낼 때부터 45° (_tilt) -> 칸 번호만 비교
        slot = lambda x, pre: x.replace(pre, "").replace("_tilt", "")  # noqa: E731
        assert sorted(slot(x, "pallet_load_") for x in loads) == sorted(slot(x, "pallet_unload_") for x in unloads)
        assert len([t for t in types if t == "verify"]) == 3
        picks = sum(t == "pick_target" for t in types) + sum("toppick_tilt" in ln for ln in lines)
        assert picks == 12
        if mode == "per_location":
            assert not any("toppick_tilt" in ln or "pallet_load" in ln and "_tilt" in ln for ln in lines)
        for ln in lines:                                   # mission_node checkStep 과 같은 인자 수
            a = ln.split()
            if a[0] == "goto" or a[0] == "pick_target":
                assert len(a) == 4 and all(float(v) == float(v) for v in a[1:])
            if a[0] == "approach":
                assert float(a[1]) >= 0.13
            if a[0] == "verify":
                assert len(a) == 7 and len(a[5]) == 9 and set(a[5]) <= set("0123") and a[6] in "012"
