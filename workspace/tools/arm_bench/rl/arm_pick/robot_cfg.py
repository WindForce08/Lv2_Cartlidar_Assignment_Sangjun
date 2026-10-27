"""받침대에 고정한 팔 (RM-X52 + 손목 롤 + 집게) Isaac Lab 3.0 관절체 설정 - 값은 실물 펌웨어 (../../ros2_ws/src/opencr_firmware/config.h) 에서.

USD 는 ``scripts/convert_urdf.py`` 가 ``../urdf/arm_bench.urdf`` 에서 만든다 (그리퍼 모터 가상 관절만 뺌) -> ``usd/arm_bench_sim/arm_bench_sim.usda``.
관절 한계 = URDF = 펌웨어 ARM_MIN/MAX_RAD (tools/make_arm_urdf.py 가 맞춤).

구동기 (XM430-W350, 12 V: 정지 토크 4.1 N·m, 무부하 4.82 rad/s, PWM 885 = 100 %):
- 팔 5관절 위치 제어: 강성 = 펌웨어 위치 P 게인 환산 = P/128 [PWM/tick] x 4096/2π [tick/rad] x 4.1/885 [N·m/PWM] (P 800 -> 18.9 N·m/rad).
  감쇠 = 역기전력 4.1/4.82. 펌웨어 I 게인은 시뮬 구동기에 없음 (I > 0 으로 튜닝하면 실물이 덜 처짐 - 시뮬은 처짐 그대로).
- 손가락 2개 (직선, 같은 명령): 힘 한계 = 펌웨어 쥐는 힘 GRIP_SQUEEZE_TICKS 환산 x 효율 GRIP_EFFICIENCY ★.
  모터 1100 tick (1.69 rad) = 손가락 31 mm 이동 (실측 열림 1550 · 닫힘 2650), 모터 하나가 손가락 둘을 움직임.
"""

from __future__ import annotations

import math
import os
import re

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

HERE = os.path.dirname(os.path.abspath(__file__))
RL = os.path.dirname(HERE)
BENCH = os.path.dirname(RL)
CONFIG_H = os.path.join(BENCH, "..", "..", "ros2_ws", "src", "opencr_firmware", "config.h")   # Lv2 펌웨어 config.h
USD_PATH = os.path.join(RL, "usd", "arm_bench_sim", "arm_bench_sim.usda")

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "wrist_roll_joint"]
FINGER_JOINTS = ["gripper_left_joint", "gripper_right_joint"]
FINGER_CLOSED, FINGER_OPEN = -0.011, 0.020   # [m] 손가락 관절 (monitor/gripper_map.py 와 같음: 닫힘 2650 · 열림 1550 tick)

STALL_TORQUE, NO_LOAD_SPEED, PWM_FULL = 4.1, 4.82, 885.0
GRIP_EFFICIENCY = 0.5                         # 그리퍼 기구 효율 (추정 ★ - 실측하면 고침)
FINGER_M_PER_MOTOR_RAD = 0.031 / (1100 * 2 * math.pi / 4096)   # 손가락 하나 이동 / 모터 각 (0.0184 m/rad)


def _config_array(name: str) -> list[float]:
    text = open(CONFIG_H, encoding="utf-8").read()
    m = re.search(name + r"\[ARM_COUNT\]\s*=\s*\{([^}]*)\}", text)
    return [float(v.strip().rstrip("f")) for v in m.group(1).split(",")]


def _config_int(name: str) -> int:
    return int(re.search(name + r"\s*=\s*(\d+)", open(CONFIG_H, encoding="utf-8").read()).group(1))


P_GAIN = _config_array("ARM_POS_P_GAIN")      # 펌웨어 위치 P 게인 (칸 0~5)
I_GAIN = _config_array("ARM_POS_I_GAIN")
GRIP_SQUEEZE_TICKS = _config_int("GRIP_SQUEEZE_TICKS")


def stiffness_from_p(p: float) -> float:
    """XM430 위치 P 게인 -> 관절 강성 [N·m/rad]"""
    return p / 128.0 * 4096.0 / (2 * math.pi) * STALL_TORQUE / PWM_FULL


ARM_STIFFNESS = {n: stiffness_from_p(P_GAIN[k]) for k, n in enumerate(ARM_JOINTS)}
ARM_DAMPING = STALL_TORQUE / NO_LOAD_SPEED     # 0.85 N·m·s/rad (위치 D 0 -> 역기전력만)
ARM_ARMATURE = 0.01                            # 감속기 반사 관성 [kg·m²] (추정, cartridar 와 같은 값)
GRIP_MOTOR_TORQUE = GRIP_SQUEEZE_TICKS * P_GAIN[5] / 128.0 * STALL_TORQUE / PWM_FULL       # 쥘 때 모터 토크 [N·m]
FINGER_FORCE = GRIP_EFFICIENCY * GRIP_MOTOR_TORQUE / (2 * FINGER_M_PER_MOTOR_RAD)          # 손가락 하나 미는 힘 [N]
FINGER_SPEED = 0.031 / 0.76                    # [m/s] 빈 손 끝에서 끝까지 약 0.76 s (PWM 한계 45 % 추정 ★)

ARM_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=USD_PATH,
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False, max_depenetration_velocity=1.0),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,          # 팔 링크끼리 충돌 상자가 실물보다 큼 (cartridar 시험) -> 관절 한계로 다룸
            solver_position_iteration_count=16,     # 손가락 ↔ 퍽 접촉
            solver_velocity_iteration_count=1,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        # 시작 자세 = 집게 끝 (0.232, 0, 0.11) 수평 IK (과제가 리셋 때 다시 계산)
        joint_pos={"joint1": 0.0, "joint2": -0.4045, "joint3": 1.2601, "joint4": -0.8556, "wrist_roll_joint": 0.0, "gripper_.*_joint": FINGER_OPEN},
        joint_vel={".*": 0.0},
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=ARM_JOINTS,
            joint_effort_limit=STALL_TORQUE,
            joint_velocity_limit=NO_LOAD_SPEED,
            stiffness=ARM_STIFFNESS,
            damping=ARM_DAMPING,
            armature=ARM_ARMATURE,
        ),
        "fingers": ImplicitActuatorCfg(
            joint_names_expr=FINGER_JOINTS,
            joint_effort_limit=FINGER_FORCE,
            joint_velocity_limit=FINGER_SPEED,
            stiffness=FINGER_FORCE / 0.002,        # 2 mm 덜 닫히면 힘 한계 (막히면 한계만큼 미는 펌웨어 쥐기 흉내)
            damping=FINGER_FORCE / 0.002 * 0.02,
        ),
    },
)
"""받침대 위 팔 (arm_base 를 환경 원점에 고정). 과제에서는 ``ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")``."""


def describe() -> str:
    k = ", ".join(f"{n} {ARM_STIFFNESS[n]:.1f}" for n in ARM_JOINTS)
    return (f"팔 강성 [N·m/rad] (config.h P {P_GAIN[:5]}, I {I_GAIN[:5]} - I 는 시뮬에 없음): {k} · 감쇠 {ARM_DAMPING:.2f}\n"
            f"손가락: 쥐는 힘 {GRIP_SQUEEZE_TICKS} tick -> 모터 {GRIP_MOTOR_TORQUE:.2f} N·m -> 손가락 하나 {FINGER_FORCE:.1f} N "
            f"(효율 {GRIP_EFFICIENCY} ★) · 속도 {FINGER_SPEED * 1000:.0f} mm/s")
