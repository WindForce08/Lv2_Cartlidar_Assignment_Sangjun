import math
import random

from mission_planner import gtsp


def test_matches_brute_force():
    rnd = random.Random(3)
    for trial in range(40):
        n = rnd.randint(1, 5)
        pts = {"s": (0, 0), "e": (rnd.random() * 5, rnd.random() * 5)}
        clusters = []
        for i in range(n):
            opts = []
            for k in range(rnd.randint(1, 3)):
                key = f"c{i}o{k}"
                pts[key] = (rnd.random() * 5, rnd.random() * 5)
                opts.append(key)
            clusters.append(opts)
        cost = lambda a, b: math.dist(pts[a], pts[b]) + (0.3 if a != b else 0)  # noqa: E731
        end = "e" if trial % 2 else None
        prec = {1: {0}} if n >= 2 and trial % 3 == 0 else None
        c1, s1 = gtsp.solve("s", clusters, end, cost, prec)
        c2, s2 = gtsp.brute_force("s", clusters, end, cost, prec)
        assert abs(c1 - c2) < 1e-9
        assert sorted(ci for ci, _ in s1) == list(range(n))
        assert all(o in clusters[ci] for ci, o in s1)
