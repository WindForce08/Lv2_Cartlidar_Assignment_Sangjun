"""주행 `Waffle-Drive-Direct` (Direct RL): 무작위 목표점까지 4륜 스키드 조향으로 가기.

이전 URDF (261027) 의 D1 과제를 팀 공유본 로봇 설정 (waffle_4wd_isaac, 가벼운 충돌 판) 으로 옮김 (정책_학습기록 2 절).
행동 2 = 왼쪽·오른쪽 바퀴 속도 [-1, 1] x WHEEL_SPEED_MAX (앞뒤 같은 값). 팔·집게는 주행 자세 (robot_params.json drive) 위치 명령으로 고정.
관측 11 = 몸체 속도 (x, y), 요 각속도, 중력 방향 (3), 몸체 좌표 목표 (x, y), 목표 거리, 이전 행동 (2).
"""

from __future__ import annotations

from isaaclab_physx.physics import PhysxCfg

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

from .robot_cfg import FLOOR_MATERIAL, WAFFLE_LIGHT_CFG


@configclass
class DriveSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg(physics_material=FLOOR_MATERIAL))
    robot: ArticulationCfg = WAFFLE_LIGHT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    light = AssetBaseCfg(prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.8, 0.8, 0.8)))


@configclass
class WaffleDriveEnvCfg(DirectRLEnvCfg):
    decimation = 4  # 제어 30 Hz = 물리 120 Hz / 4
    episode_length_s = 12.0
    action_space = 2
    observation_space = 11
    state_space = 0

    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=decimation, physics=PhysxCfg(), physics_material=FLOOR_MATERIAL)
    scene: DriveSceneCfg = DriveSceneCfg(num_envs=64, env_spacing=6.0, replicate_physics=True, clone_in_fabric=True)

    goal_distance_range = (1.0, 2.5)  # [m]
    goal_radius = 0.15  # [m]
    start_yaw_range = (-3.14159, 3.14159)  # [rad]
    tilt_limit = 0.6  # 중력 z 성분 > -0.6 (약 53° 이상 기울면) 넘어짐 종료

    rew_progress = 20.0
    rew_reached = 10.0
    rew_action_rate = -0.05
    rew_time = -0.02
    rew_tipped = -10.0
