"""미로 경로 따라가기 (Direct RL). 구현은 maze_env.py, 형상 (부딪힘 · 라이다) 은 maze_geom.py, 경로는 ../course.yaml.

과제: 경기장 (dice_mission) 여러 벌에서 경로의 웨이포인트를 차례로 지난다.
- 구간 학습: 에피소드마다 경로의 무작위 점 s 에서 시작해 다음 k 점 (s+1 ~ s+k) 을 지나면 성공. k 는 성공률이 오르면 1 씩 는다
  (처음 1, 지금 k 로 시작한 에피소드 성공률 ≥ curriculum_success 이면 +1, 끝까지 가면 경로 끝 = 출발점으로 돌아오기).
- 지나가는 점: 반경 pass_radius 안에 들어오면 도착 (0.35 m = 모서리를 잘라도 됨, 0.15 m = 칸 가운데를 지나야 함).
- 정지점: stop_pos_tol · stop_yaw_tol 안에서 거의 멈춘 채 (|vx| < stop_still_v, |wz| < stop_still_w) stop_hold_s 유지하면 도착.
- 벽 (경기장 · 신호등 · 바코드) 이나 물체 (퍽 상자 · 놓는 자리) 에 차체가 닿거나, 넘어지면 실패로 끝.
  한 웨이포인트에 wp_timeout_s 넘게 못 가면 시간 초과로 끝.
행동 2 개 = 단계 A 와 같음 (vx, wz) × drive_node skill 한계 → base_cmd 가 diff_drive · 펌웨어 한계를 흉내 냄.
관측 17 + 라이다 (몸체 좌표, 실물에서 얻을 수 있거나 경로에서 정해지는 값):
  vx, wz, 중력 방향 (3), 다음 점 (x, y), 거리, 정지점인지, 바라볼 방향 오차 (sin, cos, 정지점일 때만), 그다음 점 (x, y),
  멈춤 유지 비율, 한계 적용 뒤 vx 명령, 이전 행동 (2), 라이다 lidar_beams 개 (/ lidar_max_range).
팔 · 손가락은 drive 자세로 고정. 미로는 보기용 (충돌 없음), 부딪힘은 지도로 판단 (maze_geom.py).
"""

from __future__ import annotations

import math
import os

from isaaclab_physx.physics import PhysxCfg
from isaaclab_visualizers.kit import KitVisualizerCfg

from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

from .robot import CONTACT_MATERIAL, WAFFLE_4WD_CFG

DRIVE_RL_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAZE_W, MAZE_H = 4.404, 6.304  # 경기장 크기 [m] (maze_map.W, H)


@configclass
class WaffleDriveMazeEnvCfg(DirectRLEnvCfg):
    # env (제어 20 Hz = 물리 120 Hz / 6, 실물 정책 주기)
    decimation = 6
    episode_length_s = 300.0  # 경로 전체 (35 m) 도 들어가게. 실제로는 웨이포인트마다 wp_timeout_s 로 끊김
    action_space = 2
    lidar_beams = 36  # 10° 마다
    observation_space = 17 + lidar_beams
    state_space = 0

    # simulation (PhysX GPU, 바닥 재질 = 팀 값 μ 1.0). 창: 환경 0 의 미로를 비스듬히 위에서 봄, 보이는 환경 = 앞에서부터
    sim: SimulationCfg = SimulationCfg(
        dt=1 / 120, render_interval=decimation, physics=PhysxCfg(), physics_material=CONTACT_MATERIAL,
        visualizer_cfgs=[KitVisualizerCfg(eye=(MAZE_W / 2, -3.2, 6.5), lookat=(MAZE_W / 2, MAZE_H / 2 - 0.4, 0.0),
                                          origin_type="env", origin_env_index=0, randomly_sample_visible_envs=False)],
    )

    # robot (팀 light USD)
    robot_cfg: ArticulationCfg = WAFFLE_4WD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # scene: 미로 4.4 × 6.3 m 가 겹치지 않게 7.5 m 간격 (8 GB GPU: 64 환경부터)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=64, env_spacing=7.5, replicate_physics=True, clone_in_fabric=True)

    # 행동
    action_mode = "twist"  # "twist" | "wheels" (단계 A 와 같음)

    # 경로 · 도착 조건
    course_file = os.path.join(DRIVE_RL_DIR, "course.yaml")
    pass_radius = 0.35  # [m] 지나가는 점 (B안). A안 = 0.15
    stop_pos_tol = 0.05  # [m]
    stop_yaw_tol = math.radians(10.0)  # [rad]
    stop_still_v = 0.03  # [m/s]
    stop_still_w = 0.10  # [rad/s]
    stop_hold_s = 0.5  # [s]
    wp_timeout_s = 15.0  # [s] 한 웨이포인트에 이보다 오래 걸리면 시간 초과

    # 시작 자세: 경로 점 s 에서 위치 ± spawn_pos_noise, 방향 = (정지점이면 바라볼 방향, 아니면 다음 점 쪽) ± spawn_yaw_noise.
    # 흔든 자세가 벽에 붙으면 (여유 < 2 cm) 흔들지 않은 자세로
    spawn_pos_noise = 0.03  # [m]
    spawn_yaw_noise = math.radians(20.0)  # [rad]

    # 커리큘럼 (지날 점 수 k)
    curriculum_start_k = 1  # 경로 점 수보다 크면 경로 끝까지 (이어서 학습할 때: run.sh maze_start)
    curriculum_max_k = 0  # 0 = 경로 끝까지
    curriculum_success = 0.8  # 지금 k 로 시작한 에피소드의 성공률이 이 이상이면 k + 1
    curriculum_min_episodes = 128  # 성공률을 볼 최소 에피소드 수 (환경 수가 더 크면 환경 수)
    full_course = False  # True = 늘 full_course_start 번 점에서 흔들지 않고 경로 끝까지 (k 고정, 보기용: run.sh maze_full)
    full_course_start = 0  # 0 = 출발점 (A1 신호등)
    full_course_k = 0  # 지날 점 수 (0 = 경로 끝까지)
    start0_prob = 0.0  # 재설정 때 이 확률로 출발점 (0 번) 에서 시작 (첫 구간 보강, 무작위로만 뽑으면 1/58)

    # 라이다 · 벽
    lidar_max_range = 2.0  # [m] (관측은 / 이 값)
    hit_margin = 0.005  # [m] 차체 둘레에서 장애물까지 이보다 가까우면 닿음
    near_wall_dist = 0.05  # [m] 이보다 가까우면 가까움 벌점 (0 ~ 1 비례)
    tilt_limit = 0.6  # 중력 방향 z 성분이 -tilt_limit 보다 크면 (약 53° 이상 기울면) 넘어짐

    # 보상 (기본값: 사용자 결정 2026-11-02 "보상은 기본으로")
    rew_progress = 20.0  # 다음 점까지 거리 줄어든 만큼 [/m]
    rew_heading = 3.0  # 정지점 heading_zone 안에서 방향 오차 줄어든 만큼 [/rad] (정면 보고 서기 돕기)
    heading_zone = 0.25  # [m]
    rew_reached = 10.0  # 웨이포인트 도착 (지나가는 점 · 정지점 같음)
    rew_time = -0.02  # 스텝마다
    rew_action_rate = -0.05  # 행동 변화²
    rew_near_wall = -1.0  # 벽 · 물체 near_wall_dist 안 (가까울수록 1 에 가깝게)
    rew_hit_wall = -10.0  # 벽에 닿음 → 끝
    rew_hit_object = -10.0  # 퍽 상자 · 놓는 자리에 닿음 → 끝
    rew_tipped = -10.0  # 넘어짐 → 끝
