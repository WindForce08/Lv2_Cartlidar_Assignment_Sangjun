"""단계 A — 평지에서 무작위 목표점까지 (Direct RL). 구현은 flat_env.py, 행동 → 바퀴 속도는 base_cmd.py.

행동 2 개 [-1, 1]:
  action_mode "twist" (기본) = (vx, wz) × drive_node skill 한계 (0.3 m/s, 1.5 rad/s). base_cmd 가 diff_drive 가속 한계와
                                펌웨어 비례 축소를 흉내 내 바퀴 속도로 바꾼다 (실물: 정책 → drive_node → twist_mux → diff_drive → 펌웨어)
  action_mode "wheels"        = 왼쪽 · 오른쪽 바퀴 속도 × 시뮬 바퀴 한계 (예전 D1 방식, 비교용)
관측 11 개 (실물에서 얻을 수 있는 값만): 몸체 vx, 요 각속도, 중력 방향 (3), 몸체 좌표 목표 (x, y), 목표 거리,
  한계 적용 뒤 vx 명령, 이전 행동 (2). 옆 속도 vy 는 실물에서 못 얻으므로 뺐다.
팔 · 손가락은 drive 자세 위치 명령으로 고정.
"""

from __future__ import annotations

import math

from isaaclab_physx.physics import PhysxCfg

from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

from .robot import CONTACT_MATERIAL, WAFFLE_4WD_CFG


@configclass
class WaffleDriveFlatEnvCfg(DirectRLEnvCfg):
    # env (제어 20 Hz = 물리 120 Hz / 6, 실물 정책 주기 = robot_policy policies.yaml rate_hz 20)
    decimation = 6
    episode_length_s = 15.0  # 2.5 m 를 0.3 m/s 로 8.3 s + 뒤돌기 약 3 s (시뮬 제자리 회전은 이상값의 약 70 %)
    action_space = 2
    observation_space = 11
    state_space = 0

    # simulation (PhysX GPU, 바닥 재질 = 팀 값 μ 1.0)
    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=decimation, physics=PhysxCfg(), physics_material=CONTACT_MATERIAL)

    # robot (팀 light USD)
    robot_cfg: ArticulationCfg = WAFFLE_4WD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # scene (8 GB GPU: 64 환경부터 시작해 메모리를 보며 늘림)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=64, env_spacing=6.0, replicate_physics=True, clone_in_fabric=True)

    # 행동
    action_mode = "twist"  # "twist" | "wheels"

    # task
    goal_distance_range = (1.0, 2.5)  # [m]
    goal_radius = 0.15  # [m]
    start_yaw_range = (-math.pi, math.pi)  # [rad]
    tilt_limit = 0.6  # 중력 방향 z 성분이 -tilt_limit 보다 크면 (약 53° 이상 기울면) 넘어짐으로 종료
    fixed_tasks_seed = -1  # 0 이상이면 (평가) 환경마다 k 번째 에피소드의 시작 방향 · 목표가 늘 같음
    fixed_tasks_per_env = 8

    # reward scales (예전 로봇 D1 과 같음 — 첫 회차는 비교를 위해 바꾸지 않음)
    rew_progress = 20.0
    rew_reached = 10.0
    rew_action_rate = -0.05
    rew_time = -0.02
    rew_tipped = -10.0
