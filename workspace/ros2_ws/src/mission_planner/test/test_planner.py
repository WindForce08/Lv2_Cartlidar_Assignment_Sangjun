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
    if mode == "per_location":
        assert not any(s["tilt"] for s in picks)
    else:
        rows = [s["slot"][0] for s in picks]
        assert rows == sorted(rows, reverse=True)                      # 3 열 -> 2 열 -> 1 열로 싣기
        assert sum(s["tilt"] for s in picks) == 4 and all((tuple(s["slot"]) in TILT_SLOTS) == s["tilt"] for s in picks)
        out = [s["from_slot"][0] for s in places]
        assert out == sorted(out)                                     # 1 열 -> 2 열 -> 3 열로 꺼내기
    assert p["steps"][0]["type"] == "observe" and p["steps"][-1]["type"] == "return"
    assert {s["at"] for s in p["steps"] if s["type"] == "verify"} == {"L1", "L2", "L3"}
