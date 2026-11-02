import os

import pytest
import yaml

from mission_planner.allocation import (MissionError, allocate_greedy, count_signal, enumerate_allocations, order_groups,
                                        parse_pucks, y_options)

CASES = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), "cases_b.yaml"), encoding="utf-8"))["cases"]
COL = {"r": "red", "g": "green"}
SHP = {"c": "cylinder", "b": "box"}


def pucks_of(codes):
    return parse_pucks([dict(id=i, color=COL[c[0]], shape=SHP[c[1]], box=i // 2 + 1, side=-1 if i % 2 == 0 else 1) for i, c in enumerate(codes)])


def check_alloc(alloc, counts):
    ids = [p.id for g in alloc.values() for p in g]
    assert sorted(ids) == list(range(12))  # 중복 · 누락 없음
    assert all(p.color == "red" for p in alloc["R"]) and len(alloc["R"]) == counts["R"]
    assert all(p.color == "green" for p in alloc["G"]) and len(alloc["G"]) == counts["G"]
    assert len(alloc["Y"]) == counts["Y"] and len({p.shape for p in alloc["Y"]}) == 1


@pytest.mark.parametrize("case", CASES, ids=[c["name"] for c in CASES])
def test_case_b(case):
    pucks = pucks_of(case["pucks"])
    counts = count_signal(case["signal"])
    if case.get("impossible"):
        with pytest.raises(MissionError):
            y_options(pucks, counts)
        with pytest.raises(MissionError):
            list(enumerate_allocations(pucks, counts))
        return
    groups = order_groups(counts, case["barcode"])
    e = case["expect"]
    assert [g for g, _ in groups] == [e["L1"], e["L2"], e["L3"]]
    allocs = list(enumerate_allocations(pucks, counts))
    assert allocs
    for alloc, _ in allocs:
        check_alloc(alloc, counts)
    if case.get("greedy_fails"):
        with pytest.raises(MissionError):
            allocate_greedy(pucks, counts)


def test_signal_errors():
    with pytest.raises(MissionError):
        count_signal("RGRRGRGGYGY")      # 11 회
    with pytest.raises(MissionError):
        count_signal("RGRRGRGGYGY?")     # 불명 색
    with pytest.raises(MissionError):
        count_signal("RRRRRRRYYYYG")     # R 7 > 6
    with pytest.raises(MissionError):
        order_groups({"R": 4, "Y": 4, "G": 4}, "P9-99")


def test_tie_order_asc():
    assert [g for g, _ in order_groups({"R": 4, "Y": 4, "G": 4}, "P2-01")] == ["R", "Y", "G"]
    assert [g for g, _ in order_groups({"R": 5, "Y": 2, "G": 5}, "P2-02")] == ["R", "G", "Y"]
    assert [g for g, _ in order_groups({"R": 5, "Y": 2, "G": 5}, "P2-01")] == ["Y", "R", "G"]
