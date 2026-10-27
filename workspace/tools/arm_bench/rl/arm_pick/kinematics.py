"""팔 (RM-X52 + 손목 롤) 집게 기울기를 정한 역기구학 · 순기구학. 치수는 ../urdf/arm_bench.urdf 의 관절 원점에서 읽는다 (Isaac 없이 numpy · torch).

좌표 = arm_base (받침대 윗면, J1 서보 바닥). 집게 끝 (tip) = end_effector_link (gripper_link 원점 + x 0.139 m, 손가락 끝).
- 집게 기울기 pitch = 집게 축이 수평에서 아래로 기운 각 (0 = 수평 앞, π/2 = 바로 아래) = joint2 + joint3 + joint4. 손목 롤 (joint5) 은 따로.
- joint1 = 수평 방향, joint2 · joint3 = 2 링크 평면 IK (팔꿈치 위), joint4 = pitch - joint2 - joint3.
- 손가락 실제 끝 (충돌 상자) 은 end_effector_link 보다 FINGER_BEYOND_TIP (21 mm) 더 나감. 손가락 패드 = gripper_link x 0.12 ~ 0.16.
- y 축 양의 회전은 x 를 -z 쪽으로 돌림 (URDF 축 0 1 0): 방향각 (x 축에서 +z 쪽으로) θ 가 θ - q 가 된다.
- 닿는 범위: J4 와 어깨 사이 거리가 |A - B| ~ A + B (A = 0.130, B = 0.124 m) 안이어야 함 → workspace_z_max(d).
"""

from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET

URDF = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "urdf", "arm_bench.urdf")
ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "wrist_roll_joint"]
FINGER_JOINTS = ["gripper_left_joint", "gripper_right_joint"]


def _read(path=URDF):
    r = ET.parse(path).getroot()
    o = {j.get("name"): [float(v) for v in j.find("origin").get("xyz").split()] for j in r.findall("joint")}
    lim = {}
    for j in r.findall("joint"):
        lm = j.find("limit")
        if lm is not None and lm.get("lower") is not None:
            lim[j.get("name")] = (float(lm.get("lower")), float(lm.get("upper")))
    return o, lim


_O, LIMITS = _read()
J1_X = _O["arm_mount_joint"][0] + _O["joint1"][0]                 # J1 축의 x (0.012)
SHOULDER_Z = _O["arm_mount_joint"][2] + _O["joint1"][2] + _O["joint2"][2]   # J2 높이 (0.0765)
A_X, A_Z = _O["joint3"][0], _O["joint3"][2]                       # J2 -> J3 (0.024, 0.128)
LINK_A = math.hypot(A_X, A_Z)
ALPHA0 = math.atan2(A_Z, A_X)                                     # 0 자세에서 위팔 방향각 (79.4°)
LINK_B = _O["joint4"][0]                                          # J3 -> J4 (0.124)
TOOL = _O["wrist_roll_joint"][0] + _O["end_effector_joint"][0]    # J4 -> 집게 끝 (0.028 + 0.139)
TIP_IN_GRIPPER = _O["end_effector_joint"][0]                      # gripper_link 원점 -> 집게 끝 (0.139)
REACH_MARGIN = 0.004                                              # [m] 팔을 다 펴지 않게
FINGER_BEYOND_TIP = 0.0596 + 0.0947 + 0.0053 - TIP_IN_GRIPPER - 0.0   # 손가락 충돌 상자 끝 (gripper_link x 0.1596) - 끝 (0.139)


def workspace_z_max(d, margin=REACH_MARGIN):
    """수평 거리 d (J1 축에서) 에서 위에서 내려 잡기 (pitch π/2) 로 갈 수 있는 집게 끝 최고 높이 (numpy · torch · float 모두)"""
    rmax = LINK_A + LINK_B - margin
    sq = rmax * rmax - d * d
    sq = sq.clamp(min=0.0) if hasattr(sq, "clamp") else max(sq, 0.0)
    return sq ** 0.5 + SHOULDER_Z - TOOL


def ik(x, y, z, pitch, roll=0.0):
    """집게 끝 (x, y, z) [m, arm_base 좌표] · 집게 기울기 pitch [rad] -> 관절각 (joint1~4, 손목 롤). torch 텐서 (배치) 또는 numpy.

    닿지 않는 점은 가장 가까운 닿는 점으로 (팔을 다 펴거나 접은 자세). 관절 한계 (URDF = 펌웨어 config.h) 로 자름."""
    T = _backend(x)
    dx = x - J1_X
    q1 = T.atan2(y, dx)
    d = T.sqrt(dx * dx + y * y)
    vx = d - TOOL * T.cos(pitch + 0.0 * d)
    vz = z + TOOL * T.sin(pitch + 0.0 * d) - SHOULDER_Z
    r = T.sqrt(vx * vx + vz * vz)
    r_c = T.clip(r, abs(LINK_A - LINK_B) + 1e-3, LINK_A + LINK_B - 1e-4)
    vx, vz = vx * r_c / r, vz * r_c / r
    phi = T.atan2(vz, vx)
    cb = T.clip((LINK_A * LINK_A + r_c * r_c - LINK_B * LINK_B) / (2 * LINK_A * r_c), -1.0, 1.0)
    th_a = phi + T.arccos(cb)                                    # 팔꿈치 위
    th_b = T.atan2(vz - LINK_A * T.sin(th_a), vx - LINK_A * T.cos(th_a))
    q2 = ALPHA0 - th_a
    q3 = -th_b - q2
    q4 = pitch - q2 - q3
    q5 = roll + 0.0 * q1
    return [T.clip(q, *LIMITS[n]) for q, n in zip([q1, q2, q3, q4, q5], ARM_JOINTS)]


def ik_top_down(x, y, z, roll=0.0):
    """위에서 내려 잡기 (pitch π/2)"""
    return ik(x, y, z, math.pi / 2, roll)


def fk_tip(q1, q2, q3, q4):
    """joint1~4 -> 집게 끝 (x, y, z) 와 집게 기울기 pitch (0 = 수평, π/2 = 바로 아래). 손목 롤은 끝 위치에 영향 없음"""
    T = _backend(q1)
    th_a = ALPHA0 - q2
    th_b = -(q2 + q3)
    th_t = -(q2 + q3 + q4)
    d = LINK_A * T.cos(th_a) + LINK_B * T.cos(th_b) + TOOL * T.cos(th_t)
    z = SHOULDER_Z + LINK_A * T.sin(th_a) + LINK_B * T.sin(th_b) + TOOL * T.sin(th_t)
    return J1_X + d * T.cos(q1), d * T.sin(q1), z, -th_t


class _Np:
    import numpy as _np
    atan2, sqrt, sin, cos, arccos = _np.arctan2, _np.sqrt, _np.sin, _np.cos, _np.arccos

    @staticmethod
    def clip(v, lo, hi):
        return _Np._np.clip(v, lo, hi)


class _Torch:
    @staticmethod
    def _t():
        import torch
        return torch

    atan2 = staticmethod(lambda a, b: _Torch._t().atan2(a, b))
    sqrt = staticmethod(lambda a: _Torch._t().sqrt(a))
    sin = staticmethod(lambda a: _Torch._t().sin(a))
    cos = staticmethod(lambda a: _Torch._t().cos(a))
    arccos = staticmethod(lambda a: _Torch._t().arccos(a))
    clip = staticmethod(lambda v, lo, hi: _Torch._t().clamp(v, lo, hi))


def _backend(v):
    return _Torch if type(v).__module__.startswith("torch") else _Np


if __name__ == "__main__":
    import numpy as np

    print(f"치수: J1 x {J1_X:.4f} · 어깨 z {SHOULDER_Z:.4f} · A {LINK_A:.4f} ({math.degrees(ALPHA0):.1f}°) · B {LINK_B:.4f} · 집게 {TOOL:.4f} m")
    print("한계 [rad]:", {n: tuple(round(v, 3) for v in LIMITS[n]) for n in ARM_JOINTS})
    for d in (0.08, 0.12, 0.16, 0.20, 0.22):
        print(f"  d {d:.2f} m: 위에서 내려 잡기 최고 z {workspace_z_max(d):.3f} m")
    worst = 0.0
    for x, y, z in [(0.20, 0.0, 0.02), (0.12, 0.05, 0.02), (0.16, -0.08, 0.05), (0.18, 0.06, 0.04), (0.10, 0.0, 0.08)]:
        q = ik_top_down(np.float64(x), np.float64(y), np.float64(z))
        p = fk_tip(*q[:4])
        err = math.dist((x, y, z), p[:3])
        worst = max(worst, err)
        print(f"  끝 ({x:.2f}, {y:+.2f}, {z:.2f}) -> q [deg] {' '.join(f'{math.degrees(v):7.1f}' for v in q)} | FK 오차 {err * 1000:.2f} mm, 아래 {p[3]:.3f}")
    print("FK·IK 최대 오차", f"{worst * 1000:.3f} mm")
