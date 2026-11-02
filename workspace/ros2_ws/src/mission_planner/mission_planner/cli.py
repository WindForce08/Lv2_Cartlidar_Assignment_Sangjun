"""명령 (ROS 없이 돌아감).

  mission_make_maps [--sdf …] [--out <패키지 폴더>]     지도 2 장 (maps/) + 접근 자세표 (config/approach_poses.yaml)
  mission_plan --problem p.yaml [--mode per_location|single_load|compare] [--out 폴더]
                                                       -> plan.json · signal.json · allocation.json
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import yaml

from .allocation import MissionError

HERE = os.path.dirname(os.path.abspath(__file__))


def default_sdf():
    """패키지 worlds/dice_mission.sdf (설치본은 share, 소스는 패키지 폴더)."""
    for p in (os.path.join(HERE, "..", "worlds", "dice_mission.sdf"),):
        if os.path.exists(p):
            return os.path.normpath(p)
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("mission_planner"), "worlds", "dice_mission.sdf")
    except Exception:  # noqa: BLE001
        return "worlds/dice_mission.sdf"


def make_maps(argv=None):
    ap = argparse.ArgumentParser(description="경기장 SDF -> 지도 2 장 + 접근 자세표")
    ap.add_argument("--sdf", default=default_sdf())
    ap.add_argument("--out", default=os.path.normpath(os.path.join(HERE, "..")))
    ap.add_argument("--res", type=float, default=0.025)
    a = ap.parse_args(argv)
    from .arena import LIDAR_Z, build_maps, write_map
    from .planner import World

    solids, nav, amcl = build_maps(a.sdf, res=a.res)
    os.makedirs(os.path.join(a.out, "maps"), exist_ok=True)
    os.makedirs(os.path.join(a.out, "config"), exist_ok=True)
    write_map(nav, os.path.join(a.out, "maps", "map_nav"), "주행 계획용 (벽 · 기둥 · 박스 · 받침대 · 신호등, 바닥 위 0 ~ 0.45 m), mission_make_maps 가 만듦")
    write_map(amcl, os.path.join(a.out, "maps", "map_amcl"), f"AMCL 용 (LiDAR 스캔면 바닥 위 {LIDAR_Z:.4f} m 를 지나는 것만), mission_make_maps 가 만듦")
    w = World(a.sdf, res=a.res)
    table = {}
    for k, ap_ in w.approaches.items():
        e = {kk: v for kk, v in ap_.extra.items()}
        table[k] = dict(kind=ap_.kind, ok=ap_.ok, note=ap_.note, final=ap_.final.as_list(), stage=ap_.stage.as_list(),
                        standoff=ap_.standoff, **e)
    with open(os.path.join(a.out, "config", "approach_poses.yaml"), "w", encoding="utf-8") as f:
        f.write("# 접근 자세표 (mission_make_maps 가 만듦, 직접 고치지 말 것). 지도 = 발제 좌표 [m], yaw [rad] = 로봇 앞 방향.\n"
                "# final = 집기 · 놓기 자세 (앞 끝 - 면 = gap), stage = nav2 목표 (앞 끝 - 면 = standoff), ok = 외곽이 장애물과 안 겹침.\n"
                "# 시작 박스: S = 남쪽 (yaw +90°) · N = 북쪽 (yaw -90°) 에서 두 퍽 잇는 선에 수직으로. ±Y 둘 다 안 되는 박스만 W · E (inline).\n"
                "# pucks: 박스 퍽 (side -1 = X − 40, +1 = X + 40) 의 J1 축 거리 [m]. cells: 장소 3 x 3 칸 (dx,dy 칸 단위, +Y 위) 의 J1 거리 · 집게 방식.\n")
        yaml.safe_dump(table, f, allow_unicode=True, sort_keys=False, width=200)
    print(f"지도: {a.out}/maps/map_nav · map_amcl (.pgm .yaml), 칸 {a.res} m, 크기 {nav.shape[1]} x {nav.shape[0]}")
    bad = [k for k, v in w.approaches.items() if not v.ok]
    print(f"접근 자세표: {a.out}/config/approach_poses.yaml ({len(table)} 개, 막힘 {bad})")
    return 0


def plan_cmd(argv=None):
    ap = argparse.ArgumentParser(description="할당 · 적재 · 경로 계획")
    ap.add_argument("--problem", required=True)
    ap.add_argument("--mode", default="per_location", choices=["per_location", "single_load", "compare"])
    ap.add_argument("--sdf", default=default_sdf())
    ap.add_argument("--out", default=None, help="이 폴더에 plan.json · signal.json · allocation.json")
    a = ap.parse_args(argv)
    from .planner import World, plan

    prob = yaml.safe_load(open(a.problem, encoding="utf-8"))
    w = World(a.sdf)
    try:
        modes = ["per_location", "single_load"] if a.mode == "compare" else [a.mode]
        plans = {m: plan(w, prob, m) for m in modes}
    except MissionError as e:
        print(f"[오류] {e} -> 안전 정지", file=sys.stderr)
        return 2
    for m, p in plans.items():
        n_nav = sum(s["type"] == "navigate" for s in p["steps"])
        length = sum(s["length_m"] or 0 for s in p["steps"] if s["type"] == "navigate")
        print(f"[{m}] 비용 {p['cost_m']:.2f} (주행 {length:.2f} m, 이동 {n_nav} 번) · 할당 {', '.join(f'{L}={v['group']}:{v['ids']}' for L, v in p['allocation'].items())}")
        for wmsg in p["warnings"]:
            print(f"  경고: {wmsg}")
    if a.out:
        os.makedirs(a.out, exist_ok=True)
        p = plans[modes[0]]
        json.dump(p, open(os.path.join(a.out, "plan.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        json.dump(dict(signal=p["signal"], barcode=p["barcode"]), open(os.path.join(a.out, "signal.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        json.dump(dict(groups=p["groups"], allocation=p["allocation"]), open(os.path.join(a.out, "allocation.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        print(f"-> {a.out}/plan.json · signal.json · allocation.json")
    return 0


if __name__ == "__main__":
    sys.exit(plan_cmd())
