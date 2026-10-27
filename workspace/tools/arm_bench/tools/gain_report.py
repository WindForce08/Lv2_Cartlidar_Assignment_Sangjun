#!/usr/bin/env python3
"""게인 튜닝 기록 판정: 벤치 도구 move --log 의 CSV 를 읽어 관절마다 정착 · 넘침 · 흔들림 · 처짐 · 전류를 낸다 (읽기만, 장치 쓰기 없음).

    python3 tools/gain_report.py logs/gain/<후보>/*.csv          (보통은 tools/gain_trial.sh 가 부름)
    python3 tools/gain_report.py logs/gain/P800_I0 logs/gain/P800_I100   폴더를 주면 후보끼리 나란히 비교

- CSV 열: t_s · phase · error · cmd1~6 · pos1~6 · cur1~6 (opencr_arm_tool.py move --log). 머묾 단계 = phase 이름 끝 "(머묾)".
- 움직인 관절 = 그 머묾 단계 직전 이동 단계에서 명령이 바뀐 관절.
- 판정 기준 (Lv2 workspace/docs/arm_bench_test_2026-10-26.md 6장 제안 ★):
    1 초 안 오차 ≤ 0.01 rad · 넘침 ≤ 0.02 rad · 3 초 뒤 흔들림 (최대 - 최소) ≤ 0.005 rad · HOLD 다시 잡기 3 번 J2 하강 ≤ 0.005 rad · 오류 비트 (중단) 0
- hold*.csv (move --delta 0 를 여러 번) 는 파일 첫 줄 위치끼리 비교해 HOLD 를 다시 잡을 때마다 처지는 양 (래칫) 을 본다.
"""
import csv
import glob
import os
import sys

LIM_ERR_1S, LIM_OVER, LIM_RIPPLE, LIM_RATCHET = 0.01, 0.02, 0.005, 0.005
ERR_ABORT = 0x01 | 0x04 | 0x08 | 0x10 | 0x20 | 0x80   # SYNCREAD · HW_ESTOP · MOTOR_HW · MOTOR_CONFIG · ROLE · FAULT (opencr_arm_tool.py 와 같음)
NAMES = ["J1", "J2", "J3", "J4", "J5", "그리퍼"]


def load(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append(dict(t=float(r["t_s"]), phase=r["phase"], err=int(r["error"]),
                             cmd=[float(r[f"cmd{j}"]) for j in range(1, 7)], pos=[float(r[f"pos{j}"]) for j in range(1, 7)],
                             cur=[float(r[f"cur{j}"]) for j in range(1, 7)]))
    return rows


def phases(rows):
    out = []
    for r in rows:
        if not out or out[-1][0] != r["phase"]:
            out.append((r["phase"], []))
        out[-1][1].append(r)
    return out


def dwell_metrics(move_rows, dwell_rows):
    """이동 단계 + 머묾 단계 → 움직인 관절마다 지표"""
    out = []
    first, last = move_rows[0]["cmd"], dwell_rows[0]["cmd"]
    for j in range(6):
        d = last[j] - first[j]
        if abs(d) < 1e-4:
            continue
        sign = 1.0 if d > 0 else -1.0
        tgt = last[j]
        t0 = dwell_rows[0]["t"]
        e = [(r["t"] - t0, r["pos"][j] - tgt) for r in dwell_rows]
        at_1s = min(e, key=lambda x: abs(x[0] - 1.0))
        after3 = [x for t, x in e if t >= 3.0] or [x for _, x in e[-20:]]
        out.append(dict(joint=j, delta=d, err_1s=abs(at_1s[1]) if at_1s[0] >= 0.9 else None,
                        over=max(0.0, max(sign * x for _, x in e)), ripple=max(after3) - min(after3),
                        final=sum(x for _, x in e[-20:]) / len(e[-20:]),
                        cur_peak=max(abs(r["cur"][j]) for r in move_rows + dwell_rows),
                        cur_hold=sum(abs(r["cur"][j]) for r in dwell_rows[-20:]) / len(dwell_rows[-20:]),
                        dwell_s=e[-1][0]))
    return out


def report(files):
    lines, ok_all, starts = [], True, []
    for path in files:
        rows = load(path)
        if not rows:
            lines.append(f"{os.path.basename(path)}: 빈 기록")
            ok_all = False
            continue
        bad = sorted({r["err"] for r in rows if r["err"] & ERR_ABORT})
        aborted = any(r["phase"] == "abort" for r in rows)
        if os.path.basename(path).startswith("hold"):
            starts.append((os.path.basename(path), rows[0]["pos"]))
        ph = phases(rows)
        for k, (name, prow) in enumerate(ph):
            if not name.endswith("(머묾)") or k == 0:
                continue
            for m in dwell_metrics(ph[k - 1][1], prow):
                checks = [m["err_1s"] is not None and m["err_1s"] <= LIM_ERR_1S, m["over"] <= LIM_OVER,
                          m["dwell_s"] < 3.0 or m["ripple"] <= LIM_RIPPLE]
                ok = all(checks)
                ok_all &= ok
                e1 = "  -  " if m["err_1s"] is None else f"{m['err_1s']:.4f}"
                lines.append(f"{os.path.basename(path):14s} {name[:-5].strip():22s} {NAMES[m['joint']]:3s} {m['delta']:+.3f} | "
                             f"1 s 오차 {e1} 넘침 {m['over']:.4f} 흔들림 {m['ripple']:.4f} 남은 오차 {m['final']:+.4f} | "
                             f"전류 최대 {m['cur_peak']:.2f} 유지 {m['cur_hold']:.2f} A | {'합격' if ok else '불합격'}")
        if bad or aborted:
            ok_all = False
            lines.append(f"{os.path.basename(path):14s} 중단됨" if aborted else f"{os.path.basename(path):14s} 오류 비트 {bad}")
    if len(starts) >= 2:
        drop = starts[-1][1][1] - starts[0][1][1]
        ok = abs(drop) <= LIM_RATCHET
        ok_all &= ok
        seq = " → ".join(f"{p[1]:+.4f}" for _, p in starts)
        lines.append(f"HOLD 다시 잡기 {len(starts)} 번: J2 시작 위치 {seq} (변화 {drop:+.4f} rad) | {'합격' if ok else '불합격'}")
    lines.append(f"기준 (★): 1 s 오차 ≤ {LIM_ERR_1S} · 넘침 ≤ {LIM_OVER} · 3 s 뒤 흔들림 ≤ {LIM_RIPPLE} · HOLD 래칫 ≤ {LIM_RATCHET} rad · 중단 오류 0"
                 f"  → {'모두 합격' if ok_all else '불합격 있음'}")
    return lines, ok_all


def main():
    args = sys.argv[1:]
    if not args:
        sys.exit(__doc__)
    groups = []
    for a in args:
        if os.path.isdir(a):
            groups.append((a, sorted(glob.glob(os.path.join(a, "*.csv")))))
        else:
            groups.append((None, [a]))
    if all(g is None for g, _ in groups):
        groups = [(None, [f for _, fs in groups for f in fs])]
    for name, files in groups:
        if name:
            print(f"== {name}")
        lines, _ = report(files)
        print("\n".join(lines))


if __name__ == "__main__":
    main()
