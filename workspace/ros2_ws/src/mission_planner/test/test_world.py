import math

from mission_planner.poses import HALF_W, REACH_TILT, REACH_TILT_MIN, START_BOXES, turned


def test_maps(world):
    nav, amcl = world.nav, world.amcl
    assert not nav.blocked(0.734, 0.95) and not amcl.blocked(0.734, 0.95)        # 출발 구역
    for b, (x, y) in START_BOXES.items():                                       # 박스 70 mm: 주행 지도엔 있음, AMCL 지도엔 없음 (스캔면 82 mm 아래)
        assert nav.blocked(x, y) and not amcl.blocked(x, y), b
    assert nav.blocked(2.202, 3.5) and not amcl.blocked(2.202, 3.5)            # 중앙 받침대
    assert nav.blocked(0.08, 0.95) and amcl.blocked(0.08, 0.95)                # 신호등 몸체 (스캔면 지남)
    assert amcl.blocked(1.101, 0.0) and nav.blocked(1.101, 0.0)                 # 남쪽 벽판


def test_approaches(world):
    A = world.approaches
    for b in range(1, 7):
        opts = [a for a in A.values() if a.kind == "box" and a.extra["box"] == b and a.ok]
        assert opts, f"박스 {b} 접근 자세 없음"
        sides = {a.extra["side"] for a in opts}
        if sides & {"W", "E"}:          # ±X 는 ±Y 가 모두 막힌 박스만
            assert not A.get(f"B{b}:S").ok and not A.get(f"B{b}:N").ok
        for a in opts:
            assert all(REACH_TILT_MIN <= v["dist"] <= REACH_TILT for v in a.extra["pucks"].values())
            if not a.extra["inline"]:   # ±Y 면: 두 퍽이 J1 앞 같은 거리, 옆 ±40 mm
                d = [v["dist"] for v in a.extra["pucks"].values()]
                assert abs(d[0] - d[1]) < 1e-3
                for sx, v in a.extra["pucks"].items():   # 돈 뒤 J1→퍽 방향 = 처음 앞 방향 (상자 퍽 면)
                    t = turned(a.final, v["turn_deg"])
                    jx, jy = t.tf(0.096, 0.0)
                    px, py = START_BOXES[b][0] + sx * 0.04, START_BOXES[b][1]
                    err = math.atan2(py - jy, px - jx) - a.final.yaw
                    assert abs(math.atan2(math.sin(err), math.cos(err))) < math.radians(0.5)
                assert a.extra["turn_clearance"] >= 0.01
    h = A["HOME"]
    assert h.ok and h.extra["zone_margin_x"] > 0.05 and h.extra["zone_margin_y"] > 0.05
    assert abs(h.final.yaw - math.pi) < 1e-6
    for L in ("L1", "L2", "L3"):
        assert A[L].ok and all(c["mode"] == "tilt" for c in A[L].extra["cells"].values())
    assert HALF_W > 0.16
