"""할당 · 적재 · 경로 계획기.

입력 (problem): signal (12 점멸 'RGRR…'), barcode ('P2-01' …), pucks [{id, color, shape, box, side}] (placement.csv 와 같은 뜻).
출력 (plan): 그룹 · 할당 · 팔레트 칸 · 방문 순서 · 단계 목록 (+ signal.json · allocation.json 용 내용).

운영 방식 (mode):
  per_location (기본)  L 마다: 그 그룹 퍽이 있는 박스들을 돌며 싣고 -> L 에 놓기. 한 번에 6 개 이하라 팔레트의 수직 칸 8 개만 씀
                       (기둥 옆 기울임 칸 4 개 안 씀). 박스는 여러 번 들를 수 있음.
  single_load          12 개를 한 번에 실은 뒤 L 3 곳 배달. 팔레트는 3 열 -> 2 열 -> 1 열로 싣고 (쥔 퍽이 찬 칸 위로 못 지나감)
                       1 열 -> 2 열 -> 3 열로 꺼냄 -> 처음 배달할 L 의 퍽을 1 열 (마지막에 실음) 에. 열마다 3 · 4 번째 칸은 기울임 칸.
  둘 다: 할당 (Y 모양 · ID) x L 방문 순서 x (single_load 는 열 나누기) 를 모두 따져 경로 비용이 가장 작은 것.
경로 비용 = 대기 자세 (stage) 사이 A* 길이 + 회전 (pathcost) + 들어가고 나오는 직선 (2 x (standoff − gap)).
"""
from __future__ import annotations

import math
from itertools import combinations, permutations

from . import gtsp
from .allocation import GROUPS, MissionError, count_signal, enumerate_allocations, order_groups, parse_pucks, y_options
from .arena import build_maps, load_sdf
from .pathcost import CostModel
from .poses import PLACES, REACH_VERTICAL, make_approaches, place_order

ROW_ORDER = [1, 2, 0, 3]  # 같은 열 안 싣는 순서 (pallet_traj.py: J1 각이 큰 칸부터 = 1, 2 다음 0, 3)
TILT_SLOTS = {(2, 0), (2, 3), (3, 0), (3, 3)}  # 기둥 옆 4칸 (집게 45° 기울여 놓음 -> 집을 때부터 45°)
VERTICAL_SEQ = [(3, 1), (3, 2), (2, 1), (2, 2), (1, 1), (1, 2), (1, 0), (1, 3)]  # per_location 싣는 칸 (기울임 칸 뺌)
FULL_SEQ = [(r, i) for r in (3, 2, 1) for i in ROW_ORDER]  # single_load 싣는 칸 (12)


class World:
    """경기장 + 접근 자세표 + 비용 (계산해 두고 여러 문제에 씀)."""

    def __init__(self, sdf_path, res=0.025, inflate_radius=0.19, turn_m_per_rad=0.10):
        self.solids = [s for s in load_sdf(sdf_path) if s.z1 > 0.002 and s.z0 < 0.45]
        _, self.nav, self.amcl = build_maps(sdf_path, res=res)
        self.approaches = make_approaches(self.solids)
        self.cm = CostModel(self.nav, inflate_radius, turn_m_per_rad)
        self.box_opts = {}
        for a in self.approaches.values():
            if a.kind == "box" and a.ok:
                self.box_opts.setdefault(a.extra["box"], []).append(a.key)
        missing = [b for b in range(1, 7) if b not in self.box_opts]
        if missing:
            raise MissionError(f"접근 자세가 없는 박스 {missing}")

    def inout(self, key):
        a = self.approaches[key]
        gap = a.extra.get("gap", 0.02)
        return 2 * max(0.0, a.standoff - gap)

    def cost(self, a, b):
        if a == b:
            return 0.0
        c, _ = self.cm.cost(self.approaches[a].stage, self.approaches[b].stage)
        return c + (self.inout(b) if self.approaches[b].kind != "home" else 0.0)

    def path(self, a, b):
        return self.cm.cost(self.approaches[a].stage, self.approaches[b].stage)[1]


def _pick_tour(world, start, pucks, end, precedence_groups=None):
    """pucks 를 모두 집는 박스 방문 (GTSP). -> (비용, [(접근 키, [퍽, …]), …])."""
    by_box = {}
    for p in pucks:
        by_box.setdefault(p.box, []).append(p)
    boxes = sorted(by_box)
    clusters = [world.box_opts[b] for b in boxes]
    c, seq = gtsp.solve(start, clusters, end, world.cost)
    if seq is None:
        return math.inf, None
    stops = []
    for ci, key in seq:
        ps = by_box[boxes[ci]]
        a = world.approaches[key]
        ps = sorted(ps, key=lambda p: a.extra["pucks"][p.side]["dist"])  # inline 이면 가까운 퍽부터
        stops.append((key, ps))
    return c, stops


def _verify_and_return(world, last_L, places):
    """마지막 L 에서: 나머지 L 을 다시 들러 새로 관측 (마지막 L 은 그 자리에서) -> 출발 구역. -> (비용, 순서)."""
    rest = [p for p in places if p != last_L]
    best = (math.inf, None)
    for order in permutations(rest):
        c, prev = 0.0, last_L
        for k in order:
            c += world.cost(prev, k)
            prev = k
        c += world.cost(prev, "HOME")
        best = min(best, (c, [last_L, *order]), key=lambda t: t[0])
    return best


def _plan_per_location(world, alloc, group_of_L):
    best = None
    places = list(group_of_L)
    for perm in permutations(places):
        total, cur, trips = 0.0, "HOME", []
        for L in perm:
            ps = alloc[group_of_L[L]]
            c, stops = _pick_tour(world, cur, ps, L)
            if stops is None:
                total = math.inf
                break
            total += c
            trips.append((L, stops))
            cur = L
        if total == math.inf:
            continue
        cv, vorder = _verify_and_return(world, cur, places)
        total += cv
        if best is None or total < best[0]:
            best = (total, trips, vorder)
    return best


def _row_splits(group_members, buckets):
    """그룹 퍽들을 [(열, 개수), …] 에 나눠 담는 모든 방법."""
    if not buckets:
        yield {}
        return
    (row, k), rest = buckets[0], buckets[1:]
    for sel in combinations(group_members, k):
        others = [p for p in group_members if p not in sel]
        for tail in _row_splits(others, rest):
            yield {**tail, row: list(sel) + tail.get(row, [])}


def _plan_single_load(world, alloc, group_of_L, split_limit=400):
    best = None
    places = list(group_of_L)
    for perm in permutations(places):
        # 꺼내는 자리 1 ~ 12 (1 열 = 1 ~ 4, 2 열 = 5 ~ 8, 3 열 = 9 ~ 12), 먼저 배달할 L 부터
        pos, buckets_of = 0, {}
        for L in perm:
            n = len(alloc[group_of_L[L]])
            b = {}
            for k in range(pos, pos + n):
                row = k // 4 + 1
                b[row] = b.get(row, 0) + 1
            buckets_of[L] = sorted(b.items())
            pos += n
        per_group = [list(_row_splits(alloc[group_of_L[L]], buckets_of[L])) for L in perm]
        n_comb = math.prod(len(x) for x in per_group)
        combos = _product_limited(per_group, split_limit)
        for combo in combos:
            rows = {1: [], 2: [], 3: []}
            for d in combo:
                for r, ps in d.items():
                    rows[r] += ps
            total, cur, phases = 0.0, "HOME", []
            for r in (3, 2, 1):  # 싣기: 3 열부터
                c, stops = _pick_tour(world, cur, rows[r], None)
                if stops is None:
                    total = math.inf
                    break
                total += c
                phases.append((r, stops))
                cur = stops[-1][0] if stops else cur
            if total == math.inf:
                continue
            for L in perm:
                total += world.cost(cur, L)
                cur = L
            cv, vorder = _verify_and_return(world, cur, places)
            total += cv
            if best is None or total < best[0]:
                best = (total, phases, list(perm), combo, vorder, n_comb)
    return best


def _product_limited(lists, limit):
    out = [[]]
    for lst in lists:
        out = [o + [x] for o in out for x in lst]
        if len(out) > limit:
            out = out[:limit]
    return out


def plan(world: World, problem: dict, mode="per_location", alloc_limit=2000):
    counts = count_signal(problem["signal"])
    groups = order_groups(counts, problem["barcode"])
    pucks = parse_pucks(problem["pucks"])
    y_options(pucks, counts)  # 불가능하면 여기서 MissionError
    group_of_L = {f"L{k + 1}": g for k, (g, _) in enumerate(groups)}
    best = None
    n_alloc = 0
    for alloc, yshape in enumerate_allocations(pucks, counts, limit=alloc_limit):
        n_alloc += 1
        r = _plan_per_location(world, alloc, group_of_L) if mode == "per_location" else _plan_single_load(world, alloc, group_of_L)
        if r is not None and (best is None or r[0] < best[1][0]):
            best = (alloc, r, yshape)
    if best is None:
        raise MissionError("경로를 찾지 못함")
    alloc, r, yshape = best
    steps, warnings = _steps(world, mode, alloc, group_of_L, r)
    return dict(
        mode=mode,
        signal=dict(sequence=list(problem["signal"]), counts=counts),
        barcode=dict(code=problem["barcode"], order="desc" if problem["barcode"] in ("P2-02", "P2-04") else "asc"),
        groups={L: dict(group=g, count=counts[g]) for L, g in group_of_L.items()},
        allocation={L: dict(group=g, ids=[p.id for p in alloc[g]], y_shape=yshape if g == "Y" else None,
                            colors=[p.color for p in alloc[g]], shapes=[p.shape for p in alloc[g]]) for L, g in group_of_L.items()},
        cost_m=round(r[0], 3), allocations_checked=n_alloc,
        steps=steps, warnings=warnings,
    )


def _steps(world, mode, alloc, group_of_L, r):
    """실행 단계 목록 (navigate · pick · place · verify · return) + 경고."""
    steps, warn = [dict(type="observe", at="HOME", note="바코드 -> 12 회 점멸 -> 3 초 소등 확인 뒤 출발")], []
    cur = "HOME"
    pallet = {}  # 칸 -> 퍽

    def nav(to):
        nonlocal cur
        if to != cur:
            info = world.path(cur, to)
            a = world.approaches[to]
            steps.append(dict(type="navigate", to=to, stage=a.stage.as_list(), final=a.final.as_list(), standoff=a.standoff,
                              length_m=round(info["length"], 3) if info else None, waypoints=info["points"] if info else None))
            cur = to

    def pick(key, p, slot):
        a = world.approaches[key]
        tilt = slot in TILT_SLOTS
        d = a.extra["pucks"][p.side]["dist"]
        steps.append(dict(type="pick", at=key, puck=p.id, color=p.color, shape=p.shape, box=p.box, side=p.side,
                          j1_dist=d, tilt=tilt, slot=list(slot)))
        pallet[slot] = p
        if a.extra.get("inline"):
            warn.append(f"퍽 {p.id}: 박스 {p.box} 는 ±Y 면이 막혀 {a.extra['side']} 면 (두 퍽 앞뒤) 에서 집음, J1 거리 {d:.3f}")
        if tilt and p.shape == "box":
            warn.append(f"퍽 {p.id}: 상자 퍽을 45° 기울여 집음 -> 퍽 면이 J1 반경 방향에 맞게 서야 함 (±3°)")

    def place(L, unload):
        a = world.approaches[L]
        die = len(unload)
        for (cell, info), (slot, p) in zip(place_order(a, die), unload):
            steps.append(dict(type="place", at=L, puck=p.id, from_slot=list(slot), cell=cell, x=info["x"], y=info["y"],
                              j1_dist=info["dist"], mode=info["mode"]))
            if info["mode"] != "vertical":
                warn.append(f"{L} 칸 {cell}: J1 거리 {info['dist']:.3f} m -> {info['mode']} (수직 한계 {REACH_VERTICAL}) - 퍽 {p.id} 를 기울여 놓아야 함, 미검증")
            del pallet[slot]

    if mode == "per_location":
        total, trips, vorder = r
        for L, stops in trips:
            seq = iter(VERTICAL_SEQ)
            loaded = []
            for key, ps in stops:
                nav(key)
                for p in ps:
                    s = next(seq)
                    pick(key, p, s)
                    loaded.append((s, p))
            nav(L)
            place(L, loaded[::-1])  # 나중에 실은 것부터 꺼냄
    else:
        total, phases, perm, combo, vorder, n_comb = r
        slots_of_row = {row: [(row, i) for i in ROW_ORDER] for row in (1, 2, 3)}
        loaded = {}
        for row, stops in phases:
            it = iter(slots_of_row[row])
            for key, ps in stops:
                nav(key)
                for p in ps:
                    s = next(it)
                    pick(key, p, s)
                    loaded[p.id] = s
        for L in perm:
            nav(L)
            ps = alloc[group_of_L[L]]
            unload = sorted(((loaded[p.id], p) for p in ps), key=lambda t: (t[0][0], -ROW_ORDER.index(t[0][1])))  # 1 열부터, 열 안은 실은 반대
            place(L, unload)
        if n_comb > 400:
            warn.append(f"열 나누기 경우 {n_comb} 중 400 개만 봄")
    for L in vorder:
        nav(L)
        steps.append(dict(type="verify", at=L, note="새 카메라 관측으로 칸별 퍽 ID · 위치 기록 (목표 복사 금지)"))
    nav("HOME")
    steps.append(dict(type="return", at="HOME", note="차체 전체가 출발 구역 안에서 3 초 정지"))
    return steps, sorted(set(warn))
