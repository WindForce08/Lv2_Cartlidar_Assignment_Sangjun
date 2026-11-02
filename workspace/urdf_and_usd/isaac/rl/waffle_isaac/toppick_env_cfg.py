"""위에서 집기 `Waffle-TopPick-Direct` (Direct RL): 상자 위 퍽을 **위에서** (집게 아래 방향) 집어 세운 채 들기. B안 팔레트 (2026-11-02).

B안 상판 팔레트 v3 는 위에서 쥔 퍽을 칸에 내려 놓는 설계 (손가락 홈 = 칸 접선 방향). 팀 팔로 옆에서 쥐면 3열 4칸만 들어가고,
위에서 쥐면 12칸 모두 들어감 (`pallet_traj.py` IK). 놓기는 고정 궤적 (`pallet_place_traj.json`, `pallet_place_check.py`) 이고 이 과제는 집기만.

배치: 로봇 제자리. **위에서 수직으로 집을 수 있는 거리는 J1 축에서 약 0.23 m 까지** -> 상자 205 x 205 x 70 을 로봇 앞 끝 (상판 x 0.1328) 에서
gap 0.5 ~ 4 cm (주행이 상자 앞에 바짝 세움), 가로 ±4 cm, 요 무작위. 퍽 2 개 = 상자 중심 ± 0.04 (상자 x 축). 집을 퍽 = J1 축에 가까운 쪽, 다른 퍽은 옆 장애물.

행동 4 = 잡는 점 (TCP) 변위 x · y · z (base 좌표, x 5 mm) + 집게 (> 0 벌림 61.6 mm, ≤ 0 닫기). 변위 -> 감쇠 최소제곱 IK 로 관절 목표.
  IK 의 집게 자세 목표 = 집게 방향 아래, 손가락 닫는 축 = J1 축에서 본 접선 방향 (상자 퍽이면 인식한 면 방향 중 접선에 가까운 쪽) -> 손목 롤 (joint5) 로 맞춤.
  쥔 뒤 놓기 궤적은 joint5 = 0 (닫는 축 = 칸 접선) 이라, 상자 퍽은 면이 칸 방향과 맞음.
관측 35 = 팔 위치 (시작 자세 기준) 5 · 속도 5, 왼손가락 1, 인식 퍽 좌표 3 (참값 + 에피소드마다 ±4 mm 치우침 + 스텝마다 ±1 mm),
          잡는 점 3, 퍽 − 잡는 점 3, 집게 방향 3, 손가락 닫는 축 3, 옆 퍽 − 퍽 (x, y) 2, 상자 모양 퍽 여부 1,
          닫는 축 − 목표 닫는 방향 (cos, sin) 2, 이전 행동 4. 위치 · 방향은 base_link 좌표.
상자 접촉: 팔 링크 · 손가락 vs 상자 (벌점), 차체 · 바퀴 vs 상자 (종료 + 큰 벌점).

**기울여 집기 (2026-11-02)**: 팔레트 기둥 옆 4칸 (2 · 3열 0 · 3) 에 넣을 퍽은 집게를 45° 기울여 쥐어야 함 (놓기 궤적 pallet_traj.py TILT_SLOTS,
수직이면 집게 몸이 뒤 프로파일 기둥에 닿음). 에피소드마다 tilt_prob 로 기울임 모드 -> IK 집게 방향 목표 = 아래에서 손가락 닫는 축 둘레로 45°
(집게 윗부분이 J1 쪽, 손끝이 J1 에서 먼 쪽), 퍽은 세운 채. 5 축 팔이라 기울인 집게 방향은 팔 평면 안 -> 닫는 축 = J1 접선 그대로.
  상자 퍽은 면이 접선과 맞아야 함 -> 기울임 모드 상자 퍽은 상자 요를 접선 ±tilt_box_align 로 둠 (주행이 상자 퍽 면에 맞춰 차체를 돌려 세운다고 가정).
  관측 36 = 위 35 + 기울임 모드 (0 / 1). 시작 자세도 기울인 판 (READY_TILT, 잡는 점 자리는 READY_POSE 와 같음).
"""

from __future__ import annotations

from isaaclab_physx.physics import PhysxCfg

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import DirectRLEnvCfg, ViewerCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

from .dice2_cfg import box_cfg, puck_cfg
from .robot_cfg import BODY_LINK, FINGER_CLOSE, FINGER_GRASP_RANGE, FLOOR_MATERIAL, WAFFLE_CFG

READY_POSE = {"joint1": 0.0, "joint2": 0.48, "joint3": -0.76, "joint4": 1.84, "joint5": 0.0}
"""시작 자세 = TCP 가 J1 축 앞 0.20 m (base x 0.296), 상자 위 퍽의 잡는 높이보다 5 cm 위, 집게 아래 (pallet_traj.Arm IK, 2026-11-02). 학습 때 잡음."""
READY_TILT = {"joint1": 0.0, "joint2": -0.462, "joint3": 0.771, "joint4": 0.476, "joint5": 0.0}
"""기울임 모드 시작 자세: 잡는 점은 READY_POSE 와 같은 자리, 집게 45° (pallet_traj.Arm.ik_down(tilt=45°), 오차 0)."""

_robot = WAFFLE_CFG.replace(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=WAFFLE_CFG.spawn.replace(
        activate_contact_sensors=True,
        articulation_props=WAFFLE_CFG.spawn.articulation_props.replace(enabled_self_collisions=True),
    ),
    init_state=WAFFLE_CFG.init_state.replace(joint_pos={**WAFFLE_CFG.init_state.joint_pos, **READY_POSE, "gripper_left_joint": 0.010,
                                                        "gripper_right_joint": 0.010}),
)
RED, GREEN = (0.9, 0.1, 0.1), (0.1, 0.7, 0.3)


@configclass
class TopPickSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg(physics_material=FLOOR_MATERIAL))
    robot: ArticulationCfg = _robot
    boxA: RigidObjectCfg = box_cfg("A", (0.30, 0.0))
    puckT_cyl: RigidObjectCfg = puck_cfg("T_cyl", "cyl", RED)  # 집을 퍽: 원기둥 · 상자 중 하나만 상자 위
    puckT_box: RigidObjectCfg = puck_cfg("T_box", "box", RED)
    puckN: RigidObjectCfg = puck_cfg("N", "cyl", GREEN, (0.34, 0.0))
    arm_box_contact = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/" + "(link[2-5]|gripper_roll_link|gripper_left_link|gripper_right_link)",
        filter_prim_paths_expr=["{ENV_REGEX_NS}/BoxA"], update_period=0.0, history_length=1,
    )
    body_box_contact = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/(" + BODY_LINK + "|wheel_.*)",
        filter_prim_paths_expr=["{ENV_REGEX_NS}/BoxA"], update_period=0.0, history_length=1,
    )
    light = AssetBaseCfg(prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.85, 0.85, 0.85)))


@configclass
class WaffleTopPickEnvCfg(DirectRLEnvCfg):
    decimation = 4  # 30 Hz
    episode_length_s = 6.0
    action_space = 4  # 직교: 잡는 점 x, y, z 변위 + 집게
    observation_space = 36
    state_space = 0
    viewer: ViewerCfg = ViewerCfg(eye=(0.75, -0.6, 0.55), lookat=(0.28, 0.0, 0.12), origin_type="env", env_index=0)
    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=decimation, physics=PhysxCfg(), physics_material=FLOOR_MATERIAL)
    scene: TopPickSceneCfg = TopPickSceneCfg(num_envs=256, env_spacing=1.5, replicate_physics=True, clone_in_fabric=True)

    cart_scale = 0.005  # [m / 스텝] 잡는 점 변위 최대 (30 Hz -> 15 cm/s)
    rot_hold_weight = 0.3
    ik_damping = 1e-4
    target_lead_max = 0.25
    gripper_open = 0.010  # 61.6 mm (퍽 양옆 15.8 mm 씩). 옆 퍽 (중심 8 cm) 과는 안 닿음
    gripper_closed = -0.0075  # 팀 FINGER_CLOSE −0.011 이면 한쪽 손가락만 힘 한계 (13 N) 라 퍽이 닫는 축으로 3 ~ 5 mm 치우침 -> 30 mm 퍽에서 2 mm 더 (약 12 N, 두 손가락 같음). 팔레트 놓기 확인과 같음 (2026-11-02)

    # 배치 무작위
    robot_front = 0.1328  # B안 판 차체 앞 끝 (상판, isaac/tools/make_maze_scene.py BODY_X)
    gap_range = (0.005, 0.04)  # 위에서 집기는 J1 축 0.23 m 안 -> 상자를 바짝
    lat_range = (-0.04, 0.04)
    joint1_x = 0.096
    reach_max = 0.23  # [m] 참고: 집게 수직으로 잡는 높이에 닿는 J1 축 거리 한계 (pallet_traj IK)
    ready_noise = 0.12
    near_start_prob = 0.3  # 역방향 커리큘럼: 퍽을 시작 자세 잡는 점 바로 아래에 둠
    near_dxy_max = 0.02
    near_noise = 0.03
    near_grasp_x = 0.296  # [m] 시작 자세 잡는 점 x (base_link, J1 축 0.20 m)
    perc_bias = 0.004  # [m] 인식 좌표 에피소드 치우침
    perc_jitter = 0.001  # [m] 스텝마다
    perc_yaw_bias = 0.05  # [rad] 상자 퍽 면 방향 인식 치우침 (약 3°)
    box_shape_prob = 0.5
    tilt_prob = 0.5  # 기울임 모드 비율 (팔레트 12칸 중 4칸이지만 학습은 반반)
    tilt_deg = 45.0
    tilt_box_align = 0.05  # [rad] 기울임 모드 상자 퍽 면 - 접선 (약 3°, 주행 정렬 오차)
    grasp_dz = 0.015  # 잡는 점 = 퍽 중심 + 15 mm (= 퍽 윗면 − 15 mm, 팔레트 설계 grip_depth). 손가락 끝은 TCP 아래 20 mm -> 상자 위 25 mm
    approach_height = 0.04  # [m] 수평으로 멀면 목표를 이만큼 위로 (위에서 내려오게)
    approach_lat = 0.02  # [m] 이 수평 거리 안에서 목표가 잡는 높이로 내려옴

    # 판정
    near_dist = 0.03
    grasp_dist = 0.02
    grasp_finger_range = FINGER_GRASP_RANGE
    grasp_tilt_max = 0.05
    lift_min = 0.02
    lift_goal = 0.05
    success_lift = 0.03
    success_hold_s = 1.0
    tilt_ok_cos = 0.95  # 기울임 모드 성공: 집게 방향 · 목표 방향 (약 18° 안)
    upright_min = 0.95
    topple_cos = 0.7
    fall_drop = 0.02
    contact_force = 1.0  # [N]
    base_move_max = 0.01

    # 보상
    rew_reach = 1.0
    rew_reach_fine = 1.0
    rew_down = 0.5  # 집게 아래 (12 cm 안)
    rew_roll = 0.3  # 닫는 축 = 목표 방향 (12 cm 안)
    rew_grasp = 3.0
    rew_lifted = 6.0
    rew_lift_height = 10.0
    rew_puck_tilt = -2.0
    rew_arm_box = -2.0
    rew_body_box = -20.0
    rew_base_move = -10.0
    rew_action_rate = -0.02
    rew_joint_vel = -1e-3
    rew_neighbor_moved = -20.0
    rew_toppled = -2.0
    rew_base_tilt = -8.0
    rew_fall = -5.0
