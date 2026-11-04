"""박스 앞 세우기 `Waffle-Dock-Direct` (Direct RL): 주사위 경기장 (dice_mission) 에서 시작 박스 · 놓기 자리 앞 접근 자세로 바짝 세우기.

전체 경로는 mission_planner (GTSP) + Nav2 / AMCL 이 stage 자세 (앞 끝 - 면 = standoff 0.28 ~ 0.45 m) 까지 데려오고,
이 정책은 그 근처에서 final 자세 (앞 끝 - 면 = gap 2 cm, B6:W 만 4 cm) 로 들어가 멈춘다 = 위에서 집기 (TopPick) 가 요구하는 자리.

장면: 환경마다 경기장 USD (isaac/scenes/dice_mission.usda, 벽 · 기둥 · 박스 정적 충돌) + 로봇 (가벼운 판). 좌표 = 발제 좌표 + 환경 원점.
목표: mission_planner/config/approach_poses.yaml 의 ok 인 box · place 자세 11 곳 중 무작위 하나 (자세 = base_link, yaw = 로봇 앞 방향).
시작: 목표 final 에서 뒤로 back_range, 옆 ±lat_range, 요 ±yaw_range (stage 근처). 팔 = 주행 자세 고정.
행동 2 = 왼쪽 · 오른쪽 바퀴 속도 [-1, 1] x WHEEL_SPEED_MAX x wheel_scale (앞뒤 같은 값).
관측 9 = 목표 final 의 몸체 좌표 (x, y) · 요 오차 (cos, sin) [AMCL 수준 잡음: 에피소드 치우침 + 스텝 잡음], 몸체 속도 (x, y), 요 각속도, 이전 행동 2.
성공: 앞뒤 |e| < tol_along, 옆 |e| < tol_lat, 요 |e| < tol_yaw, 속도 < stop_speed (참값 기준) -> 종료 + 보상.
충돌: 앞 끝이 박스 면을 넘음 (평면 외곽 계산) 또는 차체 (base_link) 수평 접촉 힘 > contact_force -> 종료 + 벌점.
"""

from __future__ import annotations

import os

from isaaclab_physx.physics import PhysxCfg

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import DirectRLEnvCfg, ViewerCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

from .robot_cfg import BODY_LINK, FLOOR_MATERIAL, WAFFLE_LIGHT_CFG

_HERE = os.path.dirname(os.path.abspath(__file__))
_ISAAC = os.path.dirname(os.path.dirname(_HERE))  # .../urdf_and_usd/isaac
ARENA_USD = os.environ.get("WAFFLE_ARENA_USD", os.path.join(_ISAAC, "scenes", "dice_mission.usda"))
APPROACH_POSES = os.environ.get(
    "WAFFLE_APPROACH_POSES",
    os.path.join(_ISAAC, "..", "..", "ros2_ws", "src", "mission_planner", "config", "approach_poses.yaml"),
)

_robot = WAFFLE_LIGHT_CFG.replace(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=WAFFLE_LIGHT_CFG.spawn.replace(activate_contact_sensors=True),
)


@configclass
class DockSceneCfg(InteractiveSceneCfg):
    arena = AssetBaseCfg(prim_path="{ENV_REGEX_NS}/Arena", spawn=sim_utils.UsdFileCfg(usd_path=ARENA_USD))
    robot: ArticulationCfg = _robot
    body_contact = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/" + BODY_LINK, update_period=0.0, history_length=1)


@configclass
class WaffleDockEnvCfg(DirectRLEnvCfg):
    decimation = 4  # 제어 30 Hz = 물리 120 Hz / 4
    episode_length_s = 15.0
    action_space = 2
    observation_space = 9
    state_space = 0
    viewer: ViewerCfg = ViewerCfg(eye=(2.2, -2.0, 6.0), lookat=(2.2, 3.2, 0.0), origin_type="env", env_index=0)

    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=decimation, physics=PhysxCfg(), physics_material=FLOOR_MATERIAL)
    # 경기장 약 4.4 x 6.3 m (원점 = 왼쪽 아래 기둥) -> 간격 8 m 면 이웃 경기장과 안 겹침
    scene: DockSceneCfg = DockSceneCfg(num_envs=64, env_spacing=8.0, replicate_physics=True, clone_in_fabric=True)

    approach_poses_path = APPROACH_POSES
    # 시작 (목표 final 기준, final 좌표: +x = 로봇 앞)
    back_range = (0.10, 0.50)  # [m] final 뒤로 (stage 는 0.28 ~ 0.43 m 뒤)
    lat_range = 0.06  # [m] 옆 ± (AMCL 95 % 24 mm + Nav2 목표 허용 오차)
    yaw_range = 0.30  # [rad] ±17°
    # 관측 잡음 (AMCL 시뮬 시험: 평균 16 / 95 % 24 mm)
    obs_bias_pos = 0.01  # [m] 에피소드마다 고정 치우침 ±
    obs_bias_yaw = 0.02  # [rad]
    obs_noise_pos = 0.005  # [m] 스텝마다 ±
    obs_noise_yaw = 0.01  # [rad]
    wheel_scale = 0.6  # 행동 1 = 바퀴 최대 속도의 60 % (약 0.22 m/s)

    # 성공 · 종료
    tol_along = 0.012  # [m] gap 2 cm ± 1.2 (TopPick 은 gap 0.5 ~ 4 cm 로 학습)
    tol_lat = 0.025  # [m] (TopPick 가로 ±4 cm)
    tol_yaw = 0.06  # [rad] 3.4°
    stop_speed = 0.02  # [m/s] 이 속도 아래 + 요 각속도 0.1 rad/s 아래에서만 성공
    max_dist = 1.0  # [m] 목표에서 이만큼 멀어지면 종료 (벌점)
    contact_force = 1.0  # [N] 차체 (base_link) 수평 접촉 힘 (벽 · 박스)

    # 보상
    lat_weight = 3.0  # 진행 오차 = sqrt(앞뒤² + (lat_weight x 옆)²) + yaw_weight x |요 오차|
    yaw_weight = 0.2  # [m / rad]
    rew_progress = 20.0
    rew_success = 20.0
    rew_near = 0.05  # 허용 범위 안에 있는 스텝마다 (멈추도록)
    rew_action_rate = -0.05
    rew_time = -0.02
    rew_collision = -10.0
    rew_lost = -5.0
