"""waffle_4wd B안 (4륜 스키드 + 증속벨트 v4.2 + RM-X52 팔 + B안 상판 팔레트 v3) Isaac Lab 3.0 관절체 설정.

USD 는 ``convert_urdf.py`` 가 ``urdf/waffle_4wd_{light,full}.urdf`` 에서 만든다 (``usd/{light,full}/waffle_4wd_{light,full}/waffle_4wd_{light,full}.usda``).
관절 이름은 팀 Lv2 (origin/dev) 와 같다: 바퀴 ``wheel_{left,right}_{front,rear}_joint``, 팔 ``joint1~5`` (joint5 = 손목 롤),
그리퍼 모터 ``gripper_roll`` (팀 펌웨어 칸 5, 개폐 모터 각), 손가락 ``gripper_left/right_joint`` (직선, mimic 없음 - 같은 명령을 줄 것).
물리값은 ``robot_params.json`` 에서 읽는다 (값 · 근거 · 상태). 바꾸면 이 설정은 바로 반영되고, USD 안 값 (Isaac Sim 창에서 쓸 때) 은 convert 를 다시.

과제에서 쓰기::

    from waffle_4wd_isaac.robot_cfg import WAFFLE_4WD_CFG, pose
    robot: ArticulationCfg = WAFFLE_4WD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    robot_full = WAFFLE_4WD_FULL_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")      # 정밀 충돌
    robot.init_state.joint_pos = pose("stow")                                          # 다른 시작 자세

바퀴 관절 축은 4개 모두 +y (왼쪽) 이라 양수 = 앞으로 굴러감 (2026-10-29 시험: 4개 같은 +3.03 rad/s 로 0.299 m 전진).
"""

from __future__ import annotations

import json
import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PARAMS_PATH = os.path.join(ROOT, "robot_params.json")
SENSORS_PATH = os.path.join(ROOT, "urdf", "sensors.json")
KINDS = ("light", "full", "loaded")
URDF_PATHS = {k: os.path.join(ROOT, "urdf", f"waffle_4wd_{k}.urdf") for k in KINDS}
USD_PATHS = {k: os.path.join(ROOT, "usd", k, f"waffle_4wd_{k}", f"waffle_4wd_{k}.usda") for k in KINDS}

WHEEL_JOINTS_LEFT = ["wheel_left_front_joint", "wheel_left_rear_joint"]
WHEEL_JOINTS_RIGHT = ["wheel_right_front_joint", "wheel_right_rear_joint"]
WHEEL_JOINTS = WHEEL_JOINTS_LEFT + WHEEL_JOINTS_RIGHT
ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper_roll"]
FINGER_JOINTS = ["gripper_left_joint", "gripper_right_joint"]
ARM_BODIES = ["link2", "link3", "link4", "link5", "gripper_roll_link", "gripper_motor_link", "gripper_left_link", "gripper_right_link"]
"""USD 의 팔 몸체 (link1 = J1 모터는 고정이라 base_link 에 합쳐짐). 이 몸체끼리는 충돌을 뺌 (convert_urdf.py)."""


def _values(path: str) -> dict:
    """{묶음: {이름: value}} (status · source 는 사람용)"""
    with open(path, encoding="utf-8") as f:
        raw = json.load(f)
    return {g: {k: v["value"] for k, v in d.items() if isinstance(v, dict) and "value" in v} for g, d in raw.items() if not g.startswith("_")}


PARAMS = _values(PARAMS_PATH)
with open(SENSORS_PATH, encoding="utf-8") as _f:
    SENSORS = json.load(_f)
ROOT_HEIGHT = float(SENSORS["root_height_m"])
"""base_link 원점 (하판 기준) 의 지면 위 높이 [m] = 바퀴 반지름 - 바퀴 관절 높이 (0.00675)."""
WHEEL_RADIUS = float(PARAMS["wheels"]["radius"])
WHEEL_SEPARATION = float(PARAMS["wheels"]["separation"])
WHEEL_SPEED_MAX = float(PARAMS["wheels"]["velocity_limit"])
FINGER_CLOSED, FINGER_OPEN = float(PARAMS["fingers"]["closed"]), float(PARAMS["fingers"]["open"])

CONTACT_MATERIAL = sim_utils.RigidBodyMaterialCfg(
    static_friction=PARAMS["contact"]["static_friction"], dynamic_friction=PARAMS["contact"]["dynamic_friction"]
)
"""바닥에 쓸 재질. 로봇 쪽 재질은 USD 에 같은 값으로 들어 있음 (PhysX 기본 결합 = 평균이라 바닥도 같은 값이어야 이 마찰이 됨)."""


def pose(name: str | None = None) -> dict[str, float]:
    """시작 자세 (관절 이름 -> rad 또는 m). 'drive' (팔 접음, 기본) | 'stow' (실물 전원 켤 때) | 'zero'. 바퀴는 0"""
    p = PARAMS["poses"]
    return {"wheel_.*_joint": 0.0, **p[name or p["default"]]}


def _cfg(kind: str) -> ArticulationCfg:
    W, A, F = PARAMS["wheels"], PARAMS["arm"], PARAMS["fingers"]
    return ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=USD_PATHS[kind],
            activate_contact_sensors=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False, max_depenetration_velocity=1.0),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=bool(PARAMS["contact"]["self_collision"]),   # 팔 ↔ 차체 (팔끼리는 USD 에서 뺌)
                solver_position_iteration_count=16,   # 손가락 ↔ 퍽 접촉, 자기 충돌
                solver_velocity_iteration_count=1,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, ROOT_HEIGHT + 0.003), joint_pos=pose(), joint_vel={".*": 0.0}),
        actuators={
            # 바퀴 4개: 속도 제어 (강성 0, 감쇠 = 속도 오차당 토크)
            "wheels": ImplicitActuatorCfg(
                joint_names_expr=["wheel_.*_joint"],
                effort_limit_sim=W["effort_limit"],
                velocity_limit_sim=W["velocity_limit"],
                stiffness=0.0,
                damping=W["damping"],
                armature=W["armature"],
            ),
            # 팔 5관절 + 그리퍼 모터: 위치 제어 (펌웨어 P 800 환산, I 게인은 시뮬에 없음 -> 실물처럼 처짐)
            "arm": ImplicitActuatorCfg(
                joint_names_expr=ARM_JOINTS,
                effort_limit_sim=A["effort_limit"],
                velocity_limit_sim=A["velocity_limit"],
                stiffness=A["stiffness"],
                damping=A["damping"],
                armature=A["armature"],
            ),
            # 손가락 2개 (직선, 같은 명령): 막히면 2 mm 안에서 한계 힘으로 미는 펌웨어 쥐기 흉내
            "fingers": ImplicitActuatorCfg(
                joint_names_expr=FINGER_JOINTS,
                effort_limit_sim=F["force_limit"],
                velocity_limit_sim=F["velocity_limit"],
                stiffness=F["stiffness"],
                damping=F["damping"],
            ),
        },
    )


WAFFLE_4WD_CFG = _cfg("light")
"""가벼운 판 (차체 충돌 = 상자). 주행 · 미로 학습 기본."""
WAFFLE_4WD_FULL_CFG = _cfg("full")
"""정밀 판 (차체 충돌 = 볼록 메시). 퍽 집기 · 팔레트 · 벽 가까이 시험."""

WAFFLE_4WD_LOADED_CFG = _cfg("loaded")
"""가벼운 판 + 팔레트에 퍽 12개 (원기둥 12 g · 사각 18 g) 질량. 주행용."""


def describe() -> str:
    W, A, F, C = PARAMS["wheels"], PARAMS["arm"], PARAMS["fingers"], PARAMS["contact"]
    return (f"바퀴: 한계 {W['effort_limit']} N·m · {W['velocity_limit']:.2f} rad/s, 감쇠 {W['damping']}, armature {W['armature']} | "
            f"팔: 강성 {A['stiffness']:.2f} · 감쇠 {A['damping']:.3f} · {A['effort_limit']} N·m | "
            f"손가락: {F['force_limit']} N · 강성 {F['stiffness']:.0f} | 마찰 {C['static_friction']}/{C['dynamic_friction']} · 자기 충돌 {C['self_collision']} | "
            f"시작 자세 {PARAMS['poses']['default']}")
