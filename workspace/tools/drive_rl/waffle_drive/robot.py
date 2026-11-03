"""주행 과제용 로봇 설정 = 팀 공유본 ``workspace/urdf_and_usd/isaac/waffle_4wd_isaac`` 를 그대로 불러 씀 (바꾸지 않고 읽기만).

팀 light USD (차체 충돌 = 상자): 축간 0.16 · 좌우 0.317 m, 바퀴 1.8 N·m · 10.996 rad/s, 마찰 1.0, 시작 자세 drive (팔 접음).
물리값은 팀 ``robot_params.json``. Isaac 앱이 뜬 뒤에만 import 한다 (isaaclab.sim 을 씀). 명령 한계 상수는 base_cmd.py.
팀 폴더를 다른 곳에서 쓰려면 WAFFLE_ISAAC_DIR=/경로/isaac.
"""

from __future__ import annotations

import os
import sys

_WORKSPACE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
ISAAC_DIR = os.environ.get("WAFFLE_ISAAC_DIR", os.path.join(_WORKSPACE, "urdf_and_usd", "isaac"))
if ISAAC_DIR not in sys.path:
    sys.path.insert(0, ISAAC_DIR)

from waffle_4wd_isaac.robot_cfg import (  # noqa: E402  (팀 설정)
    ARM_JOINTS,
    CONTACT_MATERIAL,
    FINGER_JOINTS,
    WAFFLE_4WD_CFG,
    WHEEL_JOINTS_LEFT,
    WHEEL_JOINTS_RIGHT,
    WHEEL_RADIUS,
    WHEEL_SEPARATION,
    WHEEL_SPEED_MAX,
)

__all__ = [
    "ARM_JOINTS",
    "CONTACT_MATERIAL",
    "FINGER_JOINTS",
    "WAFFLE_4WD_CFG",
    "WHEEL_JOINTS_LEFT",
    "WHEEL_JOINTS_RIGHT",
    "WHEEL_RADIUS",
    "WHEEL_SEPARATION",
    "WHEEL_SPEED_MAX",
]
