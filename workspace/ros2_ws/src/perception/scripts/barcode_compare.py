"""바코드 찾기 C++(barcode_compare) 출력을 기준 Python(locate_v2.py) 출력과 장면마다 견준다 (모의 렌더 R3).

사용(PC):
  python3 -I barcode_compare.py --ref <locate_v2.py> --tool <barcode_compare 실행 파일> --tex <라벨 그림 png>
      --render <render_r3 폴더> [--render ...] [--cond clean n0.0018c3s0 ...] --out <결과 폴더> [--jobs 6] [--arm]

조건: clean = 렌더 깊이 그대로, n<σ>c<상관>s<씨앗> = 깊이 잡음 σ·z² (화소 상관 가우시안 상관 px, 씨앗) —
  ev2.py 와 같은 식 · 같은 씨앗(crc32(장면 이름) + 7919·씨앗). '5장 평균' 모의는 n0.0018c3s0 · s1 · s2.
견주는 것: 장면마다 검출 목록(같은 라벨 자리인지 5 mm 안 짝짓기) · 위치 차 · 점수 차, 그리고 ev2.py 방식 판정 값
  (라벨 보임 ≥ 0.95 · base_footprint 수평 ≤ 1.0 m 장면, 찾음 30 mm 안, 헛검출 50 mm 넘음)을 두 출력에서 따로 센다.
--arm: 팔 장면을 C++(barcode_compare --read)와 기준 zbar_check.py(44차, ba6a17bac600cb5c)의 감마 LUT · read · decide
  (같은 zbar, ctypes, quality ≥ QMIN 기호만)로 읽어 장면마다 기호 목록 · 결정이 같은지,
  판정 값(--arm-dist m · 20° 이하 · 보임 ≥ 0.95 읽음, 전 장면 틀린 문자열)을 견준다.
  --arm-scale 1 2 3: 확대 배율(1 = 원본). 1 이 아니면 두 쪽 모두 INTER_CUBIC 으로 키움(zbar_check.py 와 같음).
  --gamma 2 --qmin 4: 44차 기본값.
"""
import argparse
import collections
import importlib.util
import json
import os
import shutil
import subprocess
import sys
import zlib
from multiprocessing import Pool
from pathlib import Path

import cv2
import numpy as np

PAIR_MM = 5.0   # 같은 검출로 보는 C++ · Python 가운데 거리 [mm]

_REF = None
_CFG = None


def _init(ref_path, tex):
    global _REF, _CFG
    spec = importlib.util.spec_from_file_location("locate_ref", ref_path)
    _REF = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(_REF)
    _CFG = dict(_REF.CFG2)
    _CFG["tex"] = tex


def _depth(root, p, cond):
    dep = np.load(root / p["images"]["depth"])
    if cond["noise"] > 0:   # ev2.py 와 같은 식
        rng = np.random.default_rng(zlib.crc32(p["name"].encode()) + 7919 * cond["seed"])
        e = rng.normal(0, 1, dep.shape).astype(np.float32)
        if cond["corr"] > 0:
            e = cv2.GaussianBlur(e, (0, 0), cond["corr"]); e /= e.std()
        dep = (dep + e * cond["noise"] * dep ** 2).astype(np.float32)
    return dep


def _ref_run(args):
    root, p, depth_path, T, K = args
    bgr = cv2.imread(str(root / p["images"]["color"]))
    dep = np.load(depth_path)
    found = _REF.locate(bgr, dep, K, np.array(T), _CFG)
    return p["name"], [dict(kind="top" if f["kind"] == "top" else "wall", center=[float(x) for x in f["center"]],
                            score=float(f["score"]), theta=float(f["theta"])) for f in found]


def parse_cond(s):
    if s == "clean":
        return dict(name=s, noise=0.0, corr=0.0, seed=0)
    a = s[1:].replace("c", " ").replace("s", " ").split()
    return dict(name=s, noise=float(a[0]), corr=float(a[1]), seed=int(a[2]))


def judge(rep, p, dets, Twc):
    """ev2.py run() 과 같은 판정 줄."""
    labs = {lb["id"]: lb for lb in rep["labels"]}
    cur = labs[p["label"]]
    present = [np.array(cur["center"])] + [np.array(lb["center"]) for lb in rep["labels"]
                                           if lb["world"] == cur["world"] and lb["source"] == "world" and lb["id"] != cur["id"]
                                           and lb["prim"] != (cur["prim"] if cur["source"] == "moved" else None)]
    judged = p["label_visible"] >= 0.95 and p["base_hdist_m"] <= 1.0
    rows = []
    for d in dets:
        w = (Twc @ np.append(d["center"], 1))[:3]
        rows.append(dict(score=d["score"], dcur=float(np.linalg.norm(w - present[0])),
                         dmin=float(min(np.linalg.norm(w - c) for c in present))))
    return judged, cur["where"], rows


def summarize(res, thr):
    c = collections.Counter(); errs = []
    for judged, where, rows in res:
        c["false"] += sum(r["dmin"] > 0.05 and r["score"] >= thr for r in rows)
        if judged:
            c[where + "_n"] += 1
            hit = [r["dcur"] for r in rows if r["dcur"] <= 0.03 and r["score"] >= thr]
            if hit:
                c[where + "_hit"] += 1; errs.append(min(hit))
    return dict(wall=f"{c['wall_hit']}/{c['wall_n']}", top=f"{c['shelf_top_hit']}/{c['shelf_top_n']}", false=c["false"],
                err_max_mm=round(max(errs) * 1000, 2) if errs else None)


def compare(cpp, py):
    """장면 하나: 5 mm 안 짝짓기 (점수 높은 Python 검출부터)."""
    used = set(); pairs = []; only_py = []
    for d in py:
        best, bi = None, None
        for i, e in enumerate(cpp):
            if i in used:
                continue
            dist = 1000 * np.linalg.norm(np.array(d["center"]) - np.array(e["center"]))
            if dist <= PAIR_MM and (best is None or dist < best):
                best, bi = dist, i
        if bi is None:
            only_py.append(d)
        else:
            used.add(bi); pairs.append((d, cpp[bi], best))
    only_cpp = [e for i, e in enumerate(cpp) if i not in used]
    return pairs, only_py, only_cpp


def _zbar(qmin):
    """zbar_check.py 의 read() 와 같은 설정 (CODE128 만, X · Y density 1, quality ≥ qmin)."""
    import ctypes as C
    Z = C.CDLL("libzbar.so.0")
    Z.zbar_image_scanner_create.restype = C.c_void_p
    Z.zbar_image_create.restype = C.c_void_p
    Z.zbar_image_first_symbol.restype = C.c_void_p
    Z.zbar_image_first_symbol.argtypes = [C.c_void_p]
    Z.zbar_symbol_next.restype = C.c_void_p
    Z.zbar_symbol_next.argtypes = [C.c_void_p]
    Z.zbar_symbol_get_data.restype = C.c_char_p
    Z.zbar_symbol_get_data.argtypes = [C.c_void_p]
    Z.zbar_symbol_get_quality.argtypes = [C.c_void_p]
    Z.zbar_image_scanner_set_config.argtypes = [C.c_void_p, C.c_int, C.c_int, C.c_int]
    Z.zbar_image_set_format.argtypes = [C.c_void_p, C.c_ulong]
    Z.zbar_image_set_size.argtypes = [C.c_void_p, C.c_uint, C.c_uint]
    Z.zbar_image_set_data.argtypes = [C.c_void_p, C.c_void_p, C.c_ulong, C.c_void_p]
    Z.zbar_scan_image.argtypes = [C.c_void_p, C.c_void_p]
    Z.zbar_image_destroy.argtypes = [C.c_void_p]
    sc = Z.zbar_image_scanner_create()
    Z.zbar_image_scanner_set_config(sc, 0, 0, 0)
    Z.zbar_image_scanner_set_config(sc, 128, 0, 1)
    Z.zbar_image_scanner_set_config(sc, 0, 0x100, 1)
    Z.zbar_image_scanner_set_config(sc, 0, 0x101, 1)
    y800 = int.from_bytes(b"Y800", "little")

    def read(gray):
        g = np.ascontiguousarray(gray)
        img = Z.zbar_image_create()
        Z.zbar_image_set_format(img, y800)
        Z.zbar_image_set_size(img, g.shape[1], g.shape[0])
        Z.zbar_image_set_data(img, g.ctypes.data, g.size, None)
        Z.zbar_scan_image(sc, img)
        out, s = [], Z.zbar_image_first_symbol(img)
        while s:
            if Z.zbar_symbol_get_quality(s) >= qmin:
                out.append(Z.zbar_symbol_get_data(s).decode())
            s = Z.zbar_symbol_next(s)
        Z.zbar_image_destroy(img)
        return out
    return read


def compare_arm(a, root, rep, out, scale):
    poses = [p for p in rep["poses"] if p["kind"] == "arm"]
    lst = out / f"arm_{root.name}.txt"
    lst.write_text("".join(f"{p['name']} {root / p['images']['color']}\n" for p in poses))
    r = subprocess.run([a.tool, "--read", str(lst), repr(float(scale)), repr(float(a.gamma)), str(a.qmin)],
                       capture_output=True, text=True, check=True)
    lst.unlink()
    cpp = {d["name"]: d for d in map(json.loads, r.stdout.splitlines())}
    read = _zbar(a.qmin)
    lut = (255 * (np.arange(256) / 255.0) ** a.gamma).round().astype(np.uint8)
    same = 0; diffs = []; c = collections.Counter(); ms = []
    for p in poses:
        g = lut[cv2.cvtColor(cv2.imread(str(root / p["images"]["color"])), cv2.COLOR_BGR2GRAY)]
        syms = read(g if scale == 1 else cv2.resize(g, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC))
        u = set(syms); py_txt = u.pop() if len(u) == 1 else None
        d = cpp[p["name"]]; ms.append(d["ms"])
        cpp_txt = d["text"] if d["ok"] else None
        if syms == d["symbols"] and py_txt == cpp_txt:
            same += 1
        else:
            diffs.append(dict(scene=p["name"], py=syms, cpp=d["symbols"]))
        target = p["dist_nominal"] <= a.arm_dist + 1e-9 and p["tilt_deg"] <= 20 and p["label_visible"] >= 0.95
        for k, t in (("py", py_txt), ("cpp", cpp_txt)):
            if target:
                c[k + "_n"] += 1; c[k + "_ok"] += t == p["text"]
            c[k + "_wrong"] += t is not None and t != p["text"]
    row = dict(render=root.name, cond=f"arm x{scale:g}", scenes=len(poses), same_list=same,
               judge_py=f"{c['py_ok']}/{c['py_n']} 틀림 {c['py_wrong']}", judge_cpp=f"{c['cpp_ok']}/{c['cpp_n']} 틀림 {c['cpp_wrong']}",
               ms_median=float(np.median(ms)), ms_max=float(max(ms)), diffs=diffs)
    print(json.dumps({k: v for k, v in row.items() if k != "diffs"}, ensure_ascii=False), flush=True)
    for d in diffs:
        print("  다름", d, flush=True)
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", required=True)
    ap.add_argument("--tool", required=True)
    ap.add_argument("--tex", required=True)
    ap.add_argument("--render", action="append", required=True)
    ap.add_argument("--cond", nargs="*", default=["clean"])
    ap.add_argument("--out", required=True)
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--arm", action="store_true")
    ap.add_argument("--arm-scale", nargs="+", type=float, default=[1.0])
    ap.add_argument("--arm-dist", type=float, default=0.05)
    ap.add_argument("--gamma", type=float, default=2.0)
    ap.add_argument("--qmin", type=int, default=4)
    a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    table = []
    for rdir in a.render:
        root = Path(rdir)
        rep = json.loads((root / "render_r3.json").read_text())
        poses = [p for p in rep["poses"] if p["kind"] == "d435"]
        if a.limit:
            poses = poses[: a.limit]
        K = rep["d435"]["K"]
        if a.arm:
            for sc in a.arm_scale:
                table.append(compare_arm(a, root, rep, out, sc))
            (out / "compare.json").write_text(json.dumps(table, ensure_ascii=False, indent=1))
        for cs in a.cond:
            cond = parse_cond(cs)
            tmp = out / f"tmp_{root.name}_{cs}"
            tmp.mkdir(exist_ok=True)
            jobs, refs = [], []
            for p in poses:
                Twc = np.array(p["T_world_cam"]); Twb = np.array(p["T_world_base_footprint"])
                T = np.linalg.inv(Twb) @ Twc
                if cond["noise"] > 0:
                    dp = tmp / (p["name"] + ".npy"); np.save(dp, _depth(root, p, cond))
                else:
                    dp = root / p["images"]["depth"]
                jobs.append(" ".join([p["name"], str(root / p["images"]["color"]), str(dp)]
                                     + [repr(float(K[k])) for k in ("fx", "fy", "cx", "cy")] + [repr(float(x)) for x in T.ravel()]))
                refs.append((root, p, str(dp), T.tolist(), K))
            procs = []   # C++ 도구는 한 장면씩 차례로 돌므로 목록을 jobs 개로 나눠 함께 돌림
            for j in range(a.jobs):
                part = jobs[j::a.jobs]
                if not part:
                    continue
                (tmp / f"jobs{j}.txt").write_text("\n".join(part) + "\n")
                procs.append(subprocess.Popen([a.tool, str(tmp / f"jobs{j}.txt"), a.tex], stdout=subprocess.PIPE, text=True))
            cpp = {}; ms = []
            for pr in procs:
                so, _ = pr.communicate()
                if pr.returncode:
                    raise SystemExit(f"barcode_compare 실패 {pr.returncode}")
                for ln in so.splitlines():
                    d = json.loads(ln); cpp[d["name"]] = d["dets"]; ms.append(d["ms"])
            with Pool(a.jobs, initializer=_init, initargs=(a.ref, a.tex)) as pool:
                py = dict(pool.map(_ref_run, refs, chunksize=2))
            shutil.rmtree(tmp)
            (out / f"raw_{root.name}_{cs}.json").write_text(json.dumps(dict(cpp=cpp, py=py)))   # 장면별 검출 원본
            res_c, res_p = [], []
            n_same = 0; dpos = []; dscore = []; diffs = []
            for p in poses:
                Twc = np.array(p["T_world_cam"])
                pairs, op, oc = compare(cpp[p["name"]], py[p["name"]])
                n_same += not op and not oc and all(x["kind"] == y["kind"] for x, y, _ in pairs)
                dpos += [d for _, _, d in pairs]; dscore += [abs(x["score"] - y["score"]) for x, y, _ in pairs]
                if op or oc:
                    diffs.append(dict(scene=p["name"], only_py=[(round(d["score"], 4), d["kind"]) for d in op],
                                      only_cpp=[(round(d["score"], 4), d["kind"]) for d in oc]))
                res_c.append(judge(rep, p, cpp[p["name"]], Twc)); res_p.append(judge(rep, p, py[p["name"]], Twc))
            row = dict(render=root.name, cond=cs, scenes=len(poses), same_list=n_same,
                       dets_py=sum(len(v) for v in py.values()), dets_cpp=sum(len(v) for v in cpp.values()),
                       cpp_ms_median=round(float(np.median(ms)), 1), cpp_ms_max=round(float(max(ms)), 1),
                       pos_max_mm=round(max(dpos), 4) if dpos else None, score_max_diff=float(max(dscore)) if dscore else None,
                       judge_py=summarize(res_p, 0.65), judge_cpp=summarize(res_c, 0.65), diffs=diffs)
            table.append(row)
            print(json.dumps({k: v for k, v in row.items() if k != "diffs"}, ensure_ascii=False), flush=True)
            for d in diffs:
                print("  다름", d, flush=True)
            (out / "compare.json").write_text(json.dumps(table, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    sys.exit(main())
