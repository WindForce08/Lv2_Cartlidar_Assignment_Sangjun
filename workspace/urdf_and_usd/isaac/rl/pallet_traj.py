"""B안 팔레트 놓기 고정 궤적 만들기 (학습 없음): 위에서 쥔 퍽을 팔레트 칸 12곳에 놓는 관절 경유점 -> pallet_place_traj.json.

팔레트는 몸체에 고정이라 칸 자리가 늘 같다 (정책_학습기록 4.11 절 "스토리지는 몸체에 고정이라 고정 궤적으로 될 수 있음").
IK 는 팀 URDF (`../../waffle_4wd_description/urdf/waffle_4wd.urdf`) 를 그대로 읽어 푼다 (numpy · scipy · pyyaml, Isaac 필요 없음).

약속 (위에서 집기):
  - 잡는 점 TCP = gripper_roll_link + (0.14, 0, 0) (robot_cfg.GRASP_OFFSET_EE) = 퍽 중심 + 15 mm 위 (팔레트 설계 grip_depth 15 = 퍽 윗면 − 15 mm 와 같은 점).
  - 집게 방향 (gripper_roll_link +x) = 아래, 손목 롤 joint5 = 0 -> 손가락 닫는 축 = 팔 평면에 수직 = 칸 접선 방향 (팔레트 손가락 홈 방향).
  - 집게를 아래로 둘 수 있는 TCP 높이는 약 0.20 m 까지 (퍽 바닥 약 0.16) 라, 칸에 든 퍽 (윗면 0.174 ~ 0.196) 위로는 쥔 퍽을 옮길 수 없다
    (팔레트 설계 transit_tcp_z = 상판 + 145 mm 는 OM-X 기준, 팀 팔은 그 높이에서 집게가 23° 기욺).
    -> **칸은 바깥 열부터 채움 (3 -> 2 -> 1열), 같은 열은 J1 각이 큰 칸부터**: 목표 칸으로 가는 길 (J1 회전 · 반경 이동) 위의 칸은 늘 비어 있다.
  - 경로 (반경 r · TCP 높이 z, J1 축 기준): 앞 운반 자세 (J1 0, r = PIVOT_R, z = PIVOT_Z, 집게 수직)
          -> J1 을 칸 방향으로 (rotate, 1열 칸 벽 위 · 2열 칸 안쪽) -> 지나는 열의 칸 벽 위로 계단식으로 올리며 칸 반경까지 (carry)
          -> 퍽 바닥 = 칸 바닥 + PLACE_GAP (place)
          -> 집게 벌림 (release) -> hover 높이로 (retract) -> 거꾸로 (return · lower · rotate_back).
    rise · reach · retract · return 은 높이가 높아 집게가 기울 수 있음 (기울기를 가장 작게 푼 값, 출력의 max_tilt_deg). hover · place 는 2° 안을 목표.
  - **기둥 옆 4칸 (2열 · 3열 0 · 3, J1 ±147.1° · ±155.6°) 은 집게를 45° 기울여 놓음** (TILT_SLOTS, 2026-11-02): 수직이면 집게 몸이 뒤 프로파일 기둥
    (static_433/434) 에 닿음. 퍽은 세운 채, 집게 윗부분을 J1 쪽으로 기울임 (손끝이 칸 반경 바깥, 닫는 축 = 접선 그대로, joint5 0).
    -> 퍽을 **집을 때부터 45° 로 쥐어야** 함 (쥔 뒤 손목만 기울이면 퍽도 기욺). 앞 운반 자세도 기울인 판 (transit.q_front_tilt, 칸마다 q_front).

    python3 pallet_traj.py            # -> pallet_place_traj.json, 칸마다 자세 오차 · 기울기 · 관절 한계 여유를 출력
"""

from __future__ import annotations

import argparse
import json
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, "..", "..", "waffle_4wd_description")
URDF = os.path.join(PKG, "urdf", "waffle_4wd.urdf")
SLOTS = os.path.join(PKG, "config", "pallet_slots.yaml")
OUT = os.path.join(HERE, "pallet_place_traj.json")

ARM = ["joint1", "joint2", "joint3", "joint4", "joint5"]
EE = "gripper_roll_link"
TCP = np.array([0.14, 0.0, 0.0, 1.0])  # gripper_roll_link 좌표
GRASP_DZ = 0.015  # TCP = 퍽 중심 + 15 mm
PUCK_H = 0.060
BOTTOM = TCP[0] + GRASP_DZ + PUCK_H / 2  # 쥔 퍽 바닥 가운데 = gripper_roll_link x 0.185 (경로는 이 점을 맞춤: 집게가 기울어도 퍽 바닥이 칸 축 위)
PLATE_TOP = 0.111  # 상판 윗면 (base_link z) = 팔레트 원점
PIVOT_R = 0.10  # [m] J1 을 돌리는 퍽 바닥 가운데 반경: J1 서보 브라켓 (J1 축에서 약 47 mm) 밖, 2열 칸 (안쪽 끝 0.120) 안
#   (0.05 는 Isaac 확인에서 J1 ±60° 쯤 퍽이 브라켓에 걸림, 2026-11-02)
PIVOT_Z = 0.156  # [m] 그때 퍽 바닥 높이 = 1열 칸 벽 윗면 0.144 + HOVER_GAP (회전 길이 1열 칸 위를 지남)
HOVER_GAP = 0.012  # 퍽 바닥 = 지나가는 칸 벽 윗면 + 12 mm (팀 팔 강성 18.88 이면 Isaac 에서 약 6 mm 처짐 + 여유 6 mm, 2026-11-02 확인)
CUP_HALF = 0.0222  # 칸 맨 위 바깥 반폭 (팔레트 설계 cup_out 44.3 / 2)
PUCK_R = 0.015
PLACE_GAP = 0.001  # 퍽 바닥 = 칸 바닥 + 1 mm 에서 놓음
LIMIT_MARGIN = 0.02  # [rad] 관절 한계 안쪽 여유
MAX_TILT = math.radians(2.0)
TILT_DEG = 45.0  # [°] 기둥 옆 4칸: 퍽은 세운 채 집게 윗부분을 J1 쪽으로 기울여 쥠 (손끝이 바깥, 닫는 축 = 접선 그대로)
TILT_SLOTS = "2:0,2:3,3:0,3:3"  # 열:칸 (J1 ±147.1° · ±155.6°: 수직이면 집게 몸이 뒤 프로파일 기둥 static_433/434 에 닿음)
STEP = 0.005  # [m] 직교 경로 경유점 간격 (rotate 는 J1 0.05 rad)


# ---------------------------------------------------------------------------------------------
class Arm:
    """URDF base_link -> gripper_roll_link 사슬 (팔 5 관절, 나머지는 0)."""

    def __init__(self, urdf):
        r = ET.parse(urdf).getroot()
        by_child = {j.find("child").get("link"): j for j in r.findall("joint")}
        chain, link = [], EE
        while link in by_child:
            j = by_child[link]
            chain.append(j)
            link = j.find("parent").get("link")
        assert link == "base_link", link
        self.chain = []
        for j in reversed(chain):
            o = j.find("origin")
            xyz = [float(v) for v in o.get("xyz", "0 0 0").split()]
            rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
            ax = j.find("axis")
            self.chain.append((j.get("name"), j.get("type"), self._T(xyz, rpy),
                               np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else None))
        lim = {j.get("name"): (float(j.find("limit").get("lower")), float(j.find("limit").get("upper")))
               for j in r.findall("joint") if j.get("name") in ARM}
        self.lo = np.array([lim[n][0] for n in ARM]) + LIMIT_MARGIN
        self.hi = np.array([lim[n][1] for n in ARM]) - LIMIT_MARGIN
        self.j1 = self.link_origin("arm_base_link")

    @staticmethod
    def _T(xyz, rpy):
        a, b, c = rpy
        ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
        M = np.eye(4)
        M[:3, :3] = [[cc * cb, cc * sb * sa - sc * ca, cc * sb * ca + sc * sa], [sc * cb, sc * sb * sa + cc * ca, sc * sb * ca - cc * sa], [-sb, cb * sa, cb * ca]]
        M[:3, 3] = xyz
        return M

    def fk(self, q, until=None):
        M = np.eye(4)
        for name, typ, T, ax in self.chain:
            M = M @ T
            if typ == "revolute" and name in ARM:
                th = q[ARM.index(name)]
                K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
                R = np.eye(4)
                R[:3, :3] = np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K
                M = M @ R
            if until is not None and name == until:
                return M
        return M

    def link_origin(self, link):
        M = np.eye(4)
        for name, typ, T, ax in self.chain:
            M = M @ T
            if name.replace("_joint", "_link") == link or (name == "arm_base_joint" and link == "arm_base_link"):
                return M[:3, 3].copy()
        # arm_base_link 는 joint1 의 부모: joint1 앞까지의 변환
        M = np.eye(4)
        for name, typ, T, ax in self.chain:
            if name == "joint1":
                return M[:3, 3].copy()
            M = M @ T
        raise KeyError(link)

    def tcp(self, q, off=None):
        M = self.fk(q)
        return (M @ (TCP if off is None else np.array([off, 0.0, 0.0, 1.0])))[:3], M[:3, :3]

    def ik_down(self, p, q1, q0, wide=False, off=None, tilt=0.0):
        """J1 = q1 고정, joint5 = 0, TCP = p, 집게 방향 = 아래 (안 되면 가장 덜 기울게). q0 = 이어 붙일 시작값 (J2 ~ J4),
        None 또는 wide 면 여러 시작값으로 전체 탐색. off = 맞출 점 (gripper_roll_link x, 기본 TCP 0.14). -> (q, 위치 오차 m, 기울기 rad)."""

        # tilt [rad] > 0: 집게 축을 칸 반경 바깥으로 기울임 (윗부분이 J1 쪽). 퍽은 세운 채라 맞출 점 = TCP, 목표 = 퍽 바닥 + (GRASP_DZ + PUCK_H / 2) 위
        a_dir = np.array([math.sin(tilt) * math.cos(q1), math.sin(tilt) * math.sin(q1), -math.cos(tilt)])
        lift = np.array([0.0, 0.0, GRASP_DZ + PUCK_H / 2]) if (tilt and off is not None) else np.zeros(3)

        def res(x):
            q = [q1, x[0], x[1], x[2], 0.0]
            t, R = self.tcp(q, None if tilt else off)
            return np.r_[(t - p - lift) * 1000.0, (R[:, 0] - a_dir)]

        best = None
        starts = [q0] if q0 is not None else []
        starts += [[a, b, c] for a in ((-1.2, -0.8, -0.4, 0.0, 0.4) if tilt else (-0.8, -0.4, 0.0, 0.4)) for b in (-1.2, -0.8, 0.0, 0.8) for c in ((0.7, 1.2, 1.6, 1.95) if tilt else (1.2, 1.6, 1.95))] if (q0 is None or wide) else []
        for x0 in starts:
            x0 = np.clip(x0, self.lo[1:4] + 1e-4, self.hi[1:4] - 1e-4)
            s = least_squares(res, x0, bounds=(self.lo[1:4], self.hi[1:4]), xtol=1e-12, ftol=1e-12)
            r = res(s.x)
            e, a = np.linalg.norm(r[:3]) / 1000.0, math.asin(min(1.0, np.linalg.norm(r[3:]) / 2.0)) * 2.0
            if best is None or (e, a) < (best[1], best[2]):
                best = ([q1, *s.x, 0.0], e, a)
        return best


def lin(a, b, step):
    n = max(1, int(math.ceil(abs(b - a) / step)))
    return [a + (b - a) * k / n for k in range(1, n + 1)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=OUT)
    ap.add_argument("--tilt_deg", type=float, default=TILT_DEG)
    ap.add_argument("--tilt_slots", default=TILT_SLOTS, help="집게를 기울여 놓을 칸 '열:칸,...' ('' = 모두 수직)")
    a = ap.parse_args()
    tilt_set = {tuple(int(v) for v in x.split(":")) for x in a.tilt_slots.split(",") if x}
    arm = Arm(URDF)
    S = yaml.safe_load(open(SLOTS, encoding="utf-8"))
    j1 = arm.j1
    print(f"J1 축 base_link {np.round(j1, 4).tolist()}, 관절 한계 (여유 {LIMIT_MARGIN}) lo {np.round(arm.lo, 3).tolist()} hi {np.round(arm.hi, 3).tolist()}")

    def at(r, yaw, z):
        return np.array([j1[0] + r * math.cos(yaw), j1[1] + r * math.sin(yaw), z])

    # 앞 운반 자세
    q_front, e, tl = arm.ik_down(at(PIVOT_R, 0.0, PIVOT_Z), 0.0, None, off=BOTTOM)
    assert e < 1e-3 and tl < MAX_TILT, (e, tl)
    q_front_t, e, tl = arm.ik_down(at(PIVOT_R, 0.0, PIVOT_Z), 0.0, None, off=BOTTOM, tilt=math.radians(a.tilt_deg))
    assert e < 1e-3 and tl < MAX_TILT, ("기울인 앞 운반 자세", e, tl)
    q_front_v = q_front
    worst = dict(err=0.0, tilt=0.0)
    out_slots = []
    for s in sorted(S["slots"], key=lambda s: (-s["row"], -abs(math.atan2(s["y"] - j1[1], s["x"] - j1[0])), -s["y"])):  # 채우는 순서: 바깥 열, J1 각이 큰 칸부터
        yaw = math.atan2(s["y"] - j1[1], s["x"] - j1[0])
        r = math.hypot(s["x"] - j1[0], s["y"] - j1[1])
        tdeg = a.tilt_deg if (s["row"], s["index"]) in tilt_set else 0.0
        tilt = math.radians(tdeg)
        q_front = q_front_t if tdeg else q_front_v
        segs = []

        def track(name, pts, q_start, grip):
            q, qs, emax, tmax = q_start, [], 0.0, 0.0
            for p in pts:
                q, e, t = arm.ik_down(p, yaw, q[1:4])
                qs.append(q)
                emax, tmax = max(emax, e), max(tmax, t)
            segs.append(dict(name=name, gripper=grip, q=[[round(v, 5) for v in x] for x in qs], max_err_mm=round(emax * 1000, 2), max_tilt_deg=round(math.degrees(tmax), 2)))
            return q

        def joint_seg(name, grip, th_from, th_to, q_rest):
            qs = [[th, *q_rest, 0.0] for th in lin(th_from, th_to, 0.05)]
            segs.append(dict(name=name, gripper=grip, q=[[round(v, 5) for v in x] for x in qs], max_err_mm=0.0, max_tilt_deg=0.0))
            return qs[-1]

        # 반경 길 (r, z): 지나는 열의 칸 벽 + HOVER_GAP 위로 퍽 바닥이 가게 계단식으로 올림 (안쪽 열 벽이 더 낮음)
        rows = sorted({(x["row"], math.hypot(x["x"] - j1[0], x["y"] - j1[1]), x["wall_top_z"]) for x in S["slots"]})
        path, z = [(PIVOT_R, PIVOT_Z)], PIVOT_Z
        for _, rr, wz in rows:
            if rr > r + 1e-6:
                break
            z_need = wz + HOVER_GAP
            r_in = max(PIVOT_R, rr - CUP_HALF - PUCK_R - 0.003)  # 그 열 칸 바깥 + 퍽 반지름 + 3 mm 앞에서 올림
            if z_need > z:
                path += [(r_in, z), (r_in, z_need)]
                z = z_need
        path.append((r, z))
        z_h = z
        pts = []
        for (r0, z0), (r1, z1) in zip(path[:-1], path[1:]):
            n = max(1, int(math.ceil(max(abs(r1 - r0), abs(z1 - z0)) / STEP)))
            pts += [at(r0 + (r1 - r0) * k / n, yaw, z0 + (z1 - z0) * k / n) for k in range(1, n + 1)]
        z_p = s["floor_z"] + PLACE_GAP

        # 놓는 자세를 전체 탐색으로 먼저 풀고, 거기서 거꾸로 이어 풀어 같은 해 가지를 따라감
        q_pl, e, tl = arm.ik_down(at(r, yaw, z_p), yaw, None, off=BOTTOM, tilt=tilt)
        def back(points, q_end):
            q, out = q_end, []
            for p_ in reversed(points):
                q, e_, t_ = arm.ik_down(p_, yaw, q[1:4], off=BOTTOM, tilt=tilt)
                out.append((q, e_, t_))
            return out[::-1]
        down = back([at(r, yaw, zz) for zz in lin(z_h, z_p, 0.002)][:-1], q_pl) + [(q_pl, e, tl)]
        carry = back(pts, down[0][0])
        def seg(name, grip, sol):
            segs.append(dict(name=name, gripper=grip, q=[[round(v, 5) for v in x[0]] for x in sol],
                             max_err_mm=round(max(x[1] for x in sol) * 1000, 2), max_tilt_deg=round(math.degrees(max(x[2] for x in sol)), 2)))
        # 앞 운반 자세 (J1 = yaw) 와 거꾸로 푼 길 첫 점이 다르면 그 사이를 관절 공간으로 이음 (같은 TCP, 자세만)
        q_piv = [yaw, *q_front[1:4], 0.0]
        q = joint_seg("rotate", "hold", 0.0, yaw, q_front[1:4])
        seg("carry", "hold", [(q_piv, 0.0, 0.0)] + carry if np.abs(np.array(carry[0][0]) - q_piv).max() > 0.05 else carry)
        seg("place", "hold", down)
        segs.append(dict(name="release", gripper="open", q=[[round(v, 5) for v in q_pl]], max_err_mm=0.0, max_tilt_deg=0.0))
        up = [x for x in down[::-1]][1:] + [(carry[-1][0], 0.0, 0.0)]
        seg("retract", "open", up)
        seg("return", "open", carry[::-1][1:] + [(q_piv, 0.0, 0.0)])
        joint_seg("rotate_back", "open", yaw, 0.0, q_front[1:4])
        hg = HOVER_GAP

        allq = np.array([x for sg in segs for x in sg["q"]])
        margin = float(min((allq - arm.lo).min(), (arm.hi - allq).min()))
        em = max(sg["max_err_mm"] for sg in segs)
        tm = max(sg["max_tilt_deg"] for sg in segs)
        worst["err"], worst["tilt"] = max(worst["err"], em), max(worst["tilt"], tm)
        pl = next(sg for sg in segs if sg["name"] == "place")
        print(f"{s['row']}열 {s['index']} ({s['puck']}, {s['shape']:8s}) 반경 {r * 1000:5.1f} mm J1 {math.degrees(yaw):+7.1f}° | hover 간격 {hg * 1000:.0f} mm"
              f" | 오차 최대 {em:.2f} mm 기울기 최대 {tm:.2f}° ({', '.join(f"{sg['name']} {sg['max_tilt_deg']:.1f}" for sg in segs if sg['max_tilt_deg'] > 0)}) 운반 높이 퍽 바닥 {z_h:.3f} ({len(path)} 꺾임) | 관절 한계 여유 {margin:+.3f} rad"
              f" | 집게 기울기 {tdeg:.0f}° | 놓는 자세 {[round(v, 2) for v in pl['q'][-1]]}")
        out_slots.append(dict(row=s["row"], index=s["index"], puck=s["puck"], shape=s["shape"], x=s["x"], y=s["y"], theta_deg=s["theta_deg"],
                              floor_z=s["floor_z"], wall_top_z=s["wall_top_z"], j1_yaw=round(yaw, 5), radius=round(r, 5),
                              tilt_deg=tdeg, q_front=[round(v, 5) for v in q_front],
                              joint_margin_rad=round(margin, 4), segments=segs))
    doc = dict(
        note=("B안 팔레트 놓기 고정 궤적 (pallet_traj.py 가 만듦, 직접 고치지 말 것). base_link 좌표 [m] · 관절 [rad]. "
              "TCP = gripper_roll_link + (0.14, 0, 0) = 퍽 중심 + 0.015, 집게 아래 (운반 중 기울 수 있음) · joint5 0. 경로는 쥔 퍽 바닥 가운데 (gripper_roll_link x 0.185) 를 맞춤. slots 순서 = 채우는 순서 (바깥 열부터). 칸마다 segments 를 차례로 따라감: "
              "gripper hold = 쥔 채, open = 벌림. 경유점 사이는 관절 공간 직선 (경유점 간격 5 mm · 놓기 2 mm · J1 0.05 rad)."),
        urdf="waffle_4wd_description/urdf/waffle_4wd.urdf", slots_file="waffle_4wd_description/config/pallet_slots.yaml",
        arm_joints=ARM, tcp_offset_ee=[0.14, 0.0, 0.0], grasp_dz=GRASP_DZ, puck_h=PUCK_H,
        transit=dict(r=PIVOT_R, puck_bottom_z=PIVOT_Z, q_front=[round(v, 5) for v in q_front_v], q_front_tilt=[round(v, 5) for v in q_front_t], tilt_deg=a.tilt_deg), hover_gap=HOVER_GAP, path_point_ee=[BOTTOM, 0.0, 0.0],
        place_gap=PLACE_GAP, limit_margin=LIMIT_MARGIN, max_err_mm=round(worst["err"], 3), max_tilt_deg=round(worst["tilt"], 3),
        slots=out_slots,
    )
    json.dump(doc, open(a.out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"앞 운반 자세 q_front {[round(v, 3) for v in q_front_v]} 기울임 {[round(v, 3) for v in q_front_t]} (퍽 바닥 반경 {PIVOT_R} m, 높이 {PIVOT_Z:.3f} m)")
    print(f"-> {a.out} (칸 {len(out_slots)}, 전체 오차 최대 {worst['err']:.2f} mm, 기울기 최대 {worst['tilt']:.2f}°)")


if __name__ == "__main__":
    main()
