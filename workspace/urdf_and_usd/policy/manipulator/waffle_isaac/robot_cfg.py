"""강화 학습 과제용 로봇 설정 = 팀 공유본 ``isaac/waffle_4wd_isaac`` 를 그대로 불러 씀 (2026-10-29 부터).

팀 설정 (`../../waffle_4wd_isaac/robot_cfg.py`, 물리값 `../../robot_params.json`) 을 바꾸지 않고 읽기만 한다:
  - 정밀 충돌 USD (`WAFFLE_4WD_FULL_CFG`, 차체 볼록 메시 423 개) = 퍽 · 상자 · 보관함 가까이 과제용
  - 팔 강성 18.88 N·m/rad (펌웨어 P 800 · I 0 환산 = 실물처럼 처짐), 손가락 13 N · 6500 N/m, 마찰 1.0,
    자기 충돌 = 팔 ↔ 차체 · 바퀴 켬, 팔 몸체끼리 (손가락끼리 포함) 뺌
이 파일에는 과제 쪽 약속 (집게 몸 · 잡는 점 · 축 · 손가락 과제 값 · 접촉 센서 이름) 만 둔다.
URDF 가 바뀌면 여기와 MuJoCo 쪽 ``arm_def.py`` (이전 패키지 sim/tools) 를 먼저 볼 것 (정책_학습기록 4.8 · 4.13 절).
"""

from __future__ import annotations

import os
import sys

# 팀 공유본 isaac/ (waffle_4wd_isaac 가 있는 곳) 찾기: 저장소 = urdf_and_usd/policy/manipulator -> ../../isaac,
#   docker (sync_to_docker.sh) = waffle_shared_261028/manipulator -> ../isaac, 예전 배치 isaac/rl -> ..
_UP3 = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_ISAAC = next((p for p in (_UP3, os.path.join(_UP3, "isaac"), os.path.join(os.path.dirname(_UP3), "isaac"))
               if os.path.isdir(os.path.join(p, "waffle_4wd_isaac"))), _UP3)
if _ISAAC not in sys.path:
    sys.path.insert(0, _ISAAC)

from waffle_4wd_isaac.robot_cfg import (  # noqa: E402  (팀 설정)
    CONTACT_MATERIAL,
    PARAMS,
    ROOT_HEIGHT,
    WAFFLE_4WD_CFG,
    WAFFLE_4WD_FULL_CFG,
    WHEEL_JOINTS_LEFT,
    WHEEL_JOINTS_RIGHT,
)

WAFFLE_CFG = WAFFLE_4WD_FULL_CFG
"""과제에서 쓰는 관절체 = 팀 정밀 판 (스토리지 통 · 상자 · 퍽 접촉이 정확해야 함)."""
WAFFLE_LIGHT_CFG = WAFFLE_4WD_CFG
FLOOR_MATERIAL = CONTACT_MATERIAL
"""바닥 재질 (팀 값 1.0, PhysX 결합 = 평균이라 로봇과 같게)."""

WHEEL_REGEX = "wheel_.*_joint"
ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5"]
"""과제 행동이 움직이는 팔 5 관절 (팀 ARM_JOINTS 의 gripper_roll 은 팀 설정 그대로 0 에 둠)."""
GRIPPER_JOINT, GRIPPER_MIMIC = "gripper_left_joint", "gripper_right_joint"  # 두 손가락에 같은 값 (팀 AGENTS.md 4)
GRIPPER_ROLL = "gripper_roll"
BASE_HEIGHT = ROOT_HEIGHT  # base_link 원점 지면 위 높이 (0.00675)
ROOT_TO_BASE_LINK = 0.0  # 루트 = base_link

# 집게 약속 (MuJoCo 는 arm_def.py 에 같은 값)
EE_BODY = "gripper_roll_link"
GRASP_OFFSET_EE = (0.14, 0.0, 0.0)  # 손가락 패드 가운데 (gripper_roll_link 좌표)
ARM_LINKS_REGEX = "(link[2-5]|gripper_roll_link)"  # 팔 링크 (손가락 뺌) - 몸체 접촉 센서
FINGER_LINKS_REGEX = "(gripper_left_link|gripper_right_link)"
BODY_LINK = "base_link"  # 스토리지 · 카메라 가로대 · 센서 · 보드가 모두 base_link 에 합쳐짐
FINGER_OPEN_TASK = 0.004  # 벌림 49.6 mm (벌림 [mm] = 41.6 + 2000 x 손가락)
FINGER_CLOSE = float(PARAMS["fingers"]["closed"])  # −0.011 (팀 값)
FINGER_HOLD_30 = -0.0058  # 30 mm 퍽 쥠
FINGER_RELEASED = -0.002
FINGER_GRASP_RANGE = (-0.0100, -0.0030)


def ee_axes(quat_apply, q, ex, ey, ez):
    """집게 몸 자세 q -> (닫는 축, 아래쪽, −집게 방향) = 이전 팔 arm_link_5 코드의 (x, y, z) 축 뜻에 맞춘 순서.
    261028 팔: 집게 방향 +x, 닫는 축 y, 카메라 위 +z."""
    return quat_apply(q, ey), -quat_apply(q, ez), -quat_apply(q, ex)
