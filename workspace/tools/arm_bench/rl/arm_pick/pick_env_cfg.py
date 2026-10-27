"""ArmBench-PickPlace 과제 설정 (Direct RL): 받침대 위 팔로 테이블 위 퍽을 옆에서 (집게 수평) 집어 목표 자리에 세워 놓기.

행동 4개 [-1, 1]: 집게 끝 목표 이동 dx · dy · dz (x tip_step [m], arm_base 좌표) · 쥐기 (> 0 닫기, 아니면 열기).
  집게 끝 목표 + 집게 기울기 tool_pitch -> IK (kinematics.py) -> 관절 목표 (펌웨어 범위 안). 손목 롤 0 (원기둥 퍽).
  위에서 내려 잡기 (pitch π/2) 는 안 됨: J4 상한 2.0 rad 때문에 손가락 끝이 최고 약 5 cm (퍽 60 mm 위를 못 지나감).
  수평 잡기는 퍽이 J1 축에서 24 cm 넘게 떨어져야 테이블 높이에 닿음 (가까우면 팔을 그만큼 못 접음) → 퍽 25 ~ 33 cm.
  실물: 같은 IK 로 관절 목표 -> 펌웨어 팔 칸 0~4, 쥐기 -> 6 번째 칸 닫힘 2650 / 열림 1550 tick (펌웨어 GripGuard 가 막히면 약하게 쥠).
관측 25개 (pick_env.py _get_observations): 집게 끝 (3) · 손가락 벌림 (1) · 퍽 (3) · 퍽 - 끝 (3) · 퍽 세움 (1) · 목표 (2) · 목표 - 퍽 (2) ·
  팔 관절 (5) · 이전 행동 (4) · 들었었음 (1). 위치는 arm_base 좌표 [m]. 실물에서 퍽 · 목표 위치는 카메라 · 사람이 줌.
보상 (걸음마다): 다가가기 → 쥐기 → 들기 → 목표 위로 옮기기 → 내려놓기 → 놓고 빠지기. 마지막 상태 (목표에 세워 놓고 빠짐) 가 가장 큼.
  성공해도 끝내지 않음 (끝내면 들고만 있는 보상이 성공 보너스보다 커서 놓지 않음 - 2026-10-27 첫 학습). 넘어뜨림 · 밖으로 밀림 = 벌점 · 끝.
  성공 = 에피소드가 끝날 때 목표에 세워 놓고 빠져 있음.

값을 바꾸는 법: 이 파일을 고치거나 실행할 때 덮어쓰기 (예: ./run.sh train 256 1500 env.episode_length_s=12).
"""

from __future__ import annotations

from isaaclab_physx.physics import PhysxCfg

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, RigidObjectCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

from .robot_cfg import ARM_CFG

PUCK_RADIUS, PUCK_HEIGHT = 0.015, 0.060        # [m] 원기둥 퍽 Ø30 x 60 (Sim3D README)
MATERIAL = sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=0.8)   # 테이블 · 손가락 (추정 ★)


@configclass
class ArmPickEnvCfg(DirectRLEnvCfg):
    # ---- 시간 (물리 120 Hz, 제어 30 Hz)
    decimation = 4
    episode_length_s = 10.0
    action_space = 4
    observation_space = 25
    state_space = 0

    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=decimation, physics=PhysxCfg(), physics_material=MATERIAL)
    robot_cfg: ArticulationCfg = ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    puck_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Puck",
        spawn=sim_utils.CylinderCfg(
            radius=PUCK_RADIUS,
            height=PUCK_HEIGHT,
            axis="Z",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(solver_position_iteration_count=16, solver_velocity_iteration_count=1,
                                                         max_depenetration_velocity=1.0),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.03),   # PLA 출력 추정 ★
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.8, dynamic_friction=0.6),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.85, 0.12, 0.10)),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.16, 0.0, PUCK_HEIGHT / 2)),
    )
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=256, env_spacing=0.8, replicate_physics=True, clone_in_fabric=True)

    # ---- 집게 끝 명령 (arm_base 좌표, J1 축에서 수평 거리 d)
    tool_pitch = 0.0                 # [rad] 집게 기울기 (0 = 수평 앞, π/2 = 바로 아래) - 고정
    tip_step = 0.006                 # [m] 행동 1 = 제어 주기마다 이만큼 (0.18 m/s)
    tip_d_range = (0.20, 0.38)       # [m] 수평 거리 (24 cm 보다 가까우면 낮은 곳은 IK 가 관절 한계에서 자름)
    tip_z_range = (0.040, 0.13)      # [m] 높이 (끝 아래 손가락 뿌리 · 몸체가 테이블에 닿지 않게)
    start_tip = (0.232, 0.0, 0.11)   # [m] 시작 집게 끝: 퍽 영역보다 안쪽 · 퍽 위 (손가락 아래 끝 0.08 > 퍽 0.06)
    start_tip_noise = 0.01           # [m]

    # ---- 퍽 · 목표 (J1 축 기준 극좌표)
    puck_d_range = (0.25, 0.33)
    puck_angle_range = (-0.7, 0.7)   # [rad] 앞 (+x) 기준
    target_min_sep = 0.06            # [m] 퍽 ~ 목표 최소 거리
    fixed_task: tuple | None = None  # ((퍽 x, y), (목표 x, y)) 주면 늘 그 자리 (check 용)

    # ---- 판정
    grasp_tip_above_center = 0.015   # [m] 잡는 점 = 퍽 중심 위 이만큼 (패드 ±14.5 mm 가 퍽 위쪽 절반, 손가락 뿌리가 테이블에서 1.6 cm)
    lift_height = 0.015              # [m] 퍽 바닥이 이만큼 뜨면 들었음
    place_radius = 0.015             # [m] 퍽 중심 ~ 목표
    upright_min = 0.97               # 퍽 축 z 성분 (약 14° 안)
    tipped_max = 0.7                 # 이보다 기울면 넘어짐 (약 45°)
    lost_d = 0.42                    # [m] J1 축에서 이보다 멀면 밀려남 (퍽 영역 0.33 + 여유)

    # ---- 보상 크기 (걸음마다). 들고 있기 최대 ≈ 1 + 2 + 3 + 2 = 8, 목표에 내려 쥔 채 ≈ 13, 놓고 빠짐 ≈ 5 + 8 + 10 = 23
    rew_reach = 1.0
    rew_grasp = 1.0
    rew_lift = 2.0
    rew_carry = 3.0                  # 쥔 채 (들었던 뒤) 목표 가까이
    rew_carry_fine = 2.0
    rew_lower = 2.0                  # 쥔 채 목표 위에서 내려놓기
    rew_on_target = 5.0              # 들었던 퍽이 목표에 서 있음
    rew_release = 4.0                # 그 상태에서 손가락 열기 · 빠지기 (각각)
    rew_placed = 10.0                # 놓고 빠짐 (성공 상태)
    rew_action_rate = -0.01
    rew_fail = -5.0                  # 넘어뜨림 · 밀려남 (끝)

    show_target: bool = True         # 창 (play) 에서 목표 자리 표시 (창 없을 때는 안 만듦)
