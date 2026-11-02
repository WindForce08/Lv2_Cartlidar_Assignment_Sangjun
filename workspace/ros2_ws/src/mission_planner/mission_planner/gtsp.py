"""일반화 TSP (GTSP): 묶음 (박스) 마다 선택지 (접근 면) 하나를 골라 모든 묶음을 한 번씩 방문하는 최소 비용 순서.

Held-Karp 동적 계획: dp[방문한 묶음 집합][마지막 선택지] = 최소 비용. 묶음 n 개, 선택지 합 m 개면 O(2^n · m^2).
이 미션은 한 번에 묶음 6 개 이하 (박스 6) 라 바로 정확해(최적) 를 낸다. brute_force 는 시험용 참값.
"""
from __future__ import annotations

import math
from itertools import permutations, product


def solve(start, clusters, end, cost, precedence=None):
    """start: 출발 노드, clusters: [[노드, …], …] (노드 = 비교 가능한 키), end: 도착 노드 또는 None (열린 경로),
    cost(a, b) -> float, precedence: {묶음 i: {먼저 가야 할 묶음 j, …}} (없으면 순서 자유).
    -> (비용, [(묶음 번호, 고른 노드), …]) ; 길이 없으면 (inf, None)."""
    n = len(clusters)
    if n == 0:
        return (cost(start, end) if end is not None else 0.0), []
    pre = [0] * n
    for i, js in (precedence or {}).items():
        for j in js:
            pre[i] |= 1 << j
    INF = math.inf
    dp = {}
    for i, opts in enumerate(clusters):
        if pre[i]:
            continue
        for o in opts:
            c = cost(start, o)
            if c < dp.get((1 << i, i, o), (INF,))[0]:
                dp[(1 << i, i, o)] = (c, None)
    for mask in range(1, 1 << n):
        for i in range(n):
            if not mask >> i & 1:
                continue
            for o in clusters[i]:
                cur = dp.get((mask, i, o))
                if cur is None:
                    continue
                for j in range(n):
                    if mask >> j & 1 or (pre[j] & mask) != pre[j]:
                        continue
                    nm = mask | 1 << j
                    for p in clusters[j]:
                        c = cur[0] + cost(o, p)
                        if c < dp.get((nm, j, p), (INF,))[0]:
                            dp[(nm, j, p)] = (c, (mask, i, o))
    full = (1 << n) - 1
    best, bkey = INF, None
    for i in range(n):
        for o in clusters[i]:
            v = dp.get((full, i, o))
            if v is None:
                continue
            c = v[0] + (cost(o, end) if end is not None else 0.0)
            if c < best:
                best, bkey = c, (full, i, o)
    if bkey is None:
        return INF, None
    seq, k = [], bkey
    while k is not None:
        seq.append((k[1], k[2]))
        k = dp[k][1]
    return best, seq[::-1]


def brute_force(start, clusters, end, cost, precedence=None):
    """참값 (시험용): 모든 순서 x 모든 선택지."""
    n = len(clusters)
    best, arg = math.inf, None
    for order in permutations(range(n)):
        pos = {c: k for k, c in enumerate(order)}
        if precedence and any(pos[j] > pos[i] for i, js in precedence.items() for j in js):
            continue
        for choice in product(*(clusters[i] for i in order)):
            c, prev = 0.0, start
            for o in choice:
                c += cost(prev, o)
                prev = o
            if end is not None:
                c += cost(prev, end)
            if c < best:
                best, arg = c, list(zip(order, choice))
    return best, arg
