import os

import pytest
import yaml

from mission_planner.planner import TILT_SLOTS, plan
from mission_planner.poses import DICE

PROB = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), "..", "config", "problem_q1_example.yaml"), encoding="utf-8"))


@pytest.mark.parametrize("mode", ["per_location", "single_load"])
def test_plan_q1(world, mode):
    p = plan(world, PROB, mode)
    picks = [s for s in p["steps"] if s["type"] == "pick"]
    places = [s for s in p["steps"] if s["type"] == "place"]
    assert sorted(s["puck"] for s in picks) == list(range(12)) and sorted(s["puck"] for s in places) == list(range(12))
    assert [p["groups"][L]["group"] for L in ("L1", "L2", "L3")] == ["Y", "R", "G"]  # P2-01 오름차순, R4 Y2 G6
    for L, a in p["allocation"].items():
        assert len(a["ids"]) == p["groups"][L]["count"]
        cells = sorted(s["cell"] for s in places if s["at"] == L)
        assert cells == sorted(f"{dx},{dy}" for dx, dy in DICE[len(a["ids"])])
    assert all(s["tilt"] and tuple(s["slot"]) in TILT_SLOTS for s in picks)          # 모두 45°
    assert all(s["mode"] == "tilt" for s in places)
    for s in picks:                                                                  # 상자 퍽만 제자리 회전 (inline 은 0)
        a = world.approaches[s["at"]]
        if s["shape"] == "box" and not a.extra["inline"]:
            assert 16.5 < abs(s["turn_deg"]) < 17.5
        else:
            assert s["turn_deg"] == 0.0
    if mode == "per_location":
        assert all(s["slot"][0] in (1, 2) for s in picks)            # 느린 3 열 안 씀
    else:
        rows = [s["slot"][0] for s in picks]
        assert rows == sorted(rows, reverse=True)                      # 3 열 -> 2 열 -> 1 열로 싣기
        out = [s["from_slot"][0] for s in places]
        assert out == sorted(out)                                     # 1 열 -> 2 열 -> 3 열로 꺼내기
    assert p["steps"][0]["type"] == "observe" and p["steps"][-1]["type"] == "return"
    assert {s["at"] for s in p["steps"] if s["type"] == "verify"} == {"L1", "L2", "L3"}
