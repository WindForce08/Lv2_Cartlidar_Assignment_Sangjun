"""신호 · 바코드 -> L1 ~ L3 그룹, 퍽 12 개 -> 그룹 할당 (발제 v5 02 · 04 쪽).

규칙: R = 빨강만, G = 초록만, Y = 색 섞어도 되지만 모양 하나 (원기둥 또는 직육면체). 12 개 모두 정확히 한 번.
  -> R · G 가 빨강 · 초록을 Y 에 안 쓴 나머지 전부라서 Y 에 들어갈 빨강 수 = 빨강 − R, 초록 수 = 초록 − G 로 정해진다.
     남는 자유는 Y 모양 (최대 2 가지) 과 같은 색 · 모양 안에서 어느 ID 를 고를지뿐 -> 전부 나열해 경로 비용으로 고른다.
바코드: P2-01 · P2-03 = 오름차순 (적은 횟수부터), P2-02 · P2-04 = 내림차순. 횟수가 같으면 R -> Y -> G 순서 유지 (안정 정렬).
"""
from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations

GROUPS = ("R", "Y", "G")
BARCODES = {"P2-01": "asc", "P2-03": "asc", "P2-02": "desc", "P2-04": "desc"}
COLORS = ("red", "green")
SHAPES = ("cylinder", "box")


class MissionError(ValueError):
    """규칙 위반 -> 오류 기록 후 안전 정지 (발제 05 쪽)."""


@dataclass(frozen=True)
class Puck:
    id: int
    color: str   # red | green
    shape: str   # cylinder | box
    box: int     # 시작 박스 1 ~ 6
    side: int    # 박스 중심 X 기준 -1 (X − 40 mm) | +1 (X + 40 mm)


def parse_pucks(rows):
    pucks = [Puck(int(r["id"]), str(r["color"]), str(r["shape"]), int(r["box"]), int(r["side"])) for r in rows]
    ids = [p.id for p in pucks]
    if len(pucks) != 12 or sorted(ids) != list(range(12)):
        raise MissionError(f"퍽 ID 는 0 ~ 11 이 한 번씩이어야 함: {sorted(ids)}")
    for p in pucks:
        if p.color not in COLORS or p.shape not in SHAPES or not 1 <= p.box <= 6 or p.side not in (-1, 1):
            raise MissionError(f"퍽 {p.id} 속성 오류: {p}")
    slots = [(p.box, p.side) for p in pucks]
    if len(set(slots)) != 12:
        raise MissionError("같은 박스 · 같은 자리에 퍽이 둘")
    return pucks


def count_signal(seq):
    """점멸 12 개 (문자열 'RGRR…' 또는 목록) -> 횟수. 누락 · 불명 색은 추정하지 않고 오류."""
    s = list(seq) if not isinstance(seq, str) else list(seq.strip())
    if len(s) != 12:
        raise MissionError(f"점멸 {len(s)} 회 (12 회여야 함)")
    bad = [c for c in s if c not in GROUPS]
    if bad:
        raise MissionError(f"불명 색 {bad}")
    n = {g: s.count(g) for g in GROUPS}
    if not all(1 <= v <= 6 for v in n.values()):
        raise MissionError(f"각 횟수는 1 ~ 6: {n}")
    return n


def order_groups(counts, barcode):
    """-> [(L1 그룹, 횟수), (L2 …), (L3 …)]."""
    if barcode not in BARCODES:
        raise MissionError(f"미등록 바코드 {barcode}")
    desc = BARCODES[barcode] == "desc"
    order = sorted(GROUPS, key=lambda g: -counts[g] if desc else counts[g])  # sorted 는 안정 -> 동률이면 R, Y, G
    return [(g, counts[g]) for g in order]


def y_options(pucks, counts):
    """가능한 Y 모양과 그때 Y 에 넣을 빨강 · 초록 수. 없으면 MissionError."""
    nr = sum(p.color == "red" for p in pucks)
    ng = 12 - nr
    yr, yg = nr - counts["R"], ng - counts["G"]
    if yr < 0 or yg < 0:
        raise MissionError(f"빨강 {nr} · 초록 {ng} 로는 R {counts['R']} · G {counts['G']} 를 못 채움")
    opts = []
    for s in SHAPES:
        rs = [p for p in pucks if p.color == "red" and p.shape == s]
        gs = [p for p in pucks if p.color == "green" and p.shape == s]
        if len(rs) >= yr and len(gs) >= yg:
            opts.append(dict(shape=s, y_red=yr, y_green=yg, red_pool=rs, green_pool=gs))
    if not opts:
        raise MissionError(f"Y {counts['Y']} 개를 한 모양으로 못 만듦 (Y 에 빨강 {yr} · 초록 {yg} 필요)")
    return opts


def enumerate_allocations(pucks, counts, limit=None):
    """가능한 할당 전부 (dict 그룹 -> 퍽 목록, y_shape). limit 개에서 멈춤."""
    k = 0
    for o in y_options(pucks, counts):
        for rsel in combinations(o["red_pool"], o["y_red"]):
            for gsel in combinations(o["green_pool"], o["y_green"]):
                y = set(rsel) | set(gsel)
                alloc = {"R": [p for p in pucks if p.color == "red" and p not in y],
                         "Y": sorted(y, key=lambda p: p.id),
                         "G": [p for p in pucks if p.color == "green" and p not in y]}
                yield alloc, o["shape"]
                k += 1
                if limit and k >= limit:
                    return


def allocate_greedy(pucks, counts):
    """비교용 순차 선택 (도전 B 기본 방식): R 은 빨강을 ID 순으로, G 는 초록을 ID 순으로 먼저 고르고 Y 는 남은 것에서.
    남은 것의 모양이 섞이면 실패 -> MissionError. (탐색 방식 enumerate_allocations 는 Y 를 먼저 정해 이 실패가 없음)"""
    reds = sorted((p for p in pucks if p.color == "red"), key=lambda p: p.id)
    greens = sorted((p for p in pucks if p.color == "green"), key=lambda p: p.id)
    if len(reds) < counts["R"] or len(greens) < counts["G"]:
        raise MissionError("빨강 · 초록 수 부족")
    r, g = reds[:counts["R"]], greens[:counts["G"]]
    y = reds[counts["R"]:] + greens[counts["G"]:]
    if len(y) != counts["Y"] or len({p.shape for p in y}) > 1:
        raise MissionError(f"순차 선택 실패: 남은 Y 후보 모양 {sorted({p.shape for p in y})}")
    return {"R": r, "Y": y, "G": g}, (y[0].shape if y else None)
