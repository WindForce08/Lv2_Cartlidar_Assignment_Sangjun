"""단계 A 구현 (TeamRobot drive_env.py 를 waffle_4wd 로 옮김). 설정은 flat_env_cfg.py."""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

import isaaclab.sim as sim_utils
from isaaclab import cloner
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.utils.math import quat_from_euler_xyz, sample_uniform

from .base_cmd import SKILL_VX, SKILL_WZ, BaseCmd
from .robot import (
    ARM_JOINTS,
    CONTACT_MATERIAL,
    FINGER_JOINTS,
    WHEEL_JOINTS_LEFT,
    WHEEL_JOINTS_RIGHT,
    WHEEL_RADIUS,
    WHEEL_SEPARATION,
    WHEEL_SPEED_MAX,
)

if TYPE_CHECKING:
    from .flat_env_cfg import WaffleDriveFlatEnvCfg

EPISODE_KEYS = ("success", "time", "ideal_time", "tipped", "max_tilt", "max_accel")
"""끝난 에피소드 기록 (last_episode) 항목. 평가 eval_drive.py 가 읽는다."""


def _wrap(a: torch.Tensor) -> torch.Tensor:
    return torch.atan2(torch.sin(a), torch.cos(a))


class WaffleDriveFlatEnv(DirectRLEnv):
    """4륜 스키드 조향으로 무작위 목표점까지 주행."""

    cfg: WaffleDriveFlatEnvCfg

    def __init__(self, cfg: WaffleDriveFlatEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self._left_ids, _ = self.robot.find_joints(WHEEL_JOINTS_LEFT, preserve_order=True)
        self._right_ids, _ = self.robot.find_joints(WHEEL_JOINTS_RIGHT, preserve_order=True)
        self._hold_ids, _ = self.robot.find_joints(ARM_JOINTS + FINGER_JOINTS, preserve_order=True)
        self._wheel_ids = list(self._left_ids) + list(self._right_ids)
        n, dev = self.num_envs, self.device
        self.base_cmd = BaseCmd(n, dev, WHEEL_RADIUS, WHEEL_SEPARATION, WHEEL_SPEED_MAX, cfg.sim.dt, cfg.action_mode)
        self.actions = torch.zeros(n, 2, device=dev)
        self.prev_actions = torch.zeros_like(self.actions)
        self.goal_w = torch.zeros(n, 2, device=dev)
        self.prev_dist = torch.zeros(n, device=dev)
        self.reached = torch.zeros(n, dtype=torch.bool, device=dev)
        self.tipped = torch.zeros(n, dtype=torch.bool, device=dev)
        # 에피소드 중 기록 → 끝나면 last_episode 로 (평가용)
        self.ideal_time = torch.zeros(n, device=dev)  # |처음 방향 오차| / 1.5 + 거리 / 0.3 [s]
        self.max_tilt = torch.zeros(n, device=dev)  # [deg]
        self.max_accel = torch.zeros(n, device=dev)  # 몸체 전진 속도 변화 / 제어 주기 [m/s²]
        self.prev_vx = torch.zeros(n, device=dev)
        self.last_episode = {k: torch.zeros(n, device=dev) for k in EPISODE_KEYS}
        # 고정 과제 (평가): 환경마다 k 번째 에피소드의 (시작 yaw, 목표 거리, 목표 방향) 을 seed 로 미리 정함
        self._tasks = None
        if cfg.fixed_tasks_seed >= 0:
            u = torch.rand(n, cfg.fixed_tasks_per_env, 3, generator=torch.Generator().manual_seed(cfg.fixed_tasks_seed))
            (y0, y1), (r0, r1) = cfg.start_yaw_range, cfg.goal_distance_range
            self._tasks = torch.stack(
                (y0 + (y1 - y0) * u[..., 0], r0 + (r1 - r0) * u[..., 1], -math.pi + 2 * math.pi * u[..., 2]), dim=-1
            ).to(dev)
            self._task_k = torch.zeros(n, dtype=torch.long, device=dev)

    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot_cfg)
        spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg(physics_material=CONTACT_MATERIAL))
        src, dest = "/World/envs/env_0", "/World/envs/env_{}"
        pos = cloner.grid_transforms(self.scene.num_envs, self.scene.cfg.env_spacing, device=self.device)[0]
        plan = cloner.clone_plan_from_env_0(src, dest, self.scene.num_envs, self.device, pos, global_paths=("/World/ground",))
        cloner.replicate(plan, stage=self.scene.stage)
        if "physx" in self.scene.physics_backend:
            self.scene.filter_collisions(global_prim_paths=[])
        self.scene.articulations["robot"] = self.robot
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.8, 0.8, 0.8))
        light_cfg.func("/World/Light", light_cfg)

    # ---- 행동 ----
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self.prev_actions = self.actions.clone()
        self.actions = actions.clamp(-1.0, 1.0)

    def _apply_action(self) -> None:
        # 물리 스텝마다: diff_drive 가속 한계가 물리 dt 로 쌓이고, 펌웨어처럼 좌우를 같은 비율로 줄인 바퀴 속도
        wl, wr = self.base_cmd.step(self.actions)
        vel = torch.cat(
            (wl.unsqueeze(1).expand(-1, len(self._left_ids)), wr.unsqueeze(1).expand(-1, len(self._right_ids))), dim=1
        )
        cmd = self.robot.actuators.target_command
        cmd.set_velocity_index(value=vel.contiguous(), joint_ids=self._wheel_ids)
        cmd.set_position_index(
            value=self.robot.data.default_joint_pos.torch[:, self._hold_ids].contiguous(), joint_ids=self._hold_ids
        )

    # ---- 관측·보상·종료 ----
    def _goal_body(self) -> tuple[torch.Tensor, torch.Tensor]:
        pos = self.robot.data.root_pos_w.torch[:, :2]
        heading = self.robot.data.heading_w.torch
        d = self.goal_w - pos
        c, s = torch.cos(heading), torch.sin(heading)
        gx, gy = c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]
        return torch.stack((gx, gy), dim=1), torch.linalg.norm(d, dim=1)

    def _get_observations(self) -> dict:
        g_b, dist = self._goal_body()
        data = self.robot.data
        obs = torch.cat(
            (
                data.root_lin_vel_b.torch[:, 0:1],
                data.root_ang_vel_b.torch[:, 2:3],
                data.projected_gravity_b.torch,
                g_b,
                dist.unsqueeze(1),
                self.base_cmd.vx.unsqueeze(1),
                self.actions,
            ),
            dim=-1,
        )
        return {"policy": obs}

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        grav = self.robot.data.projected_gravity_b.torch
        self.tipped = grav[:, 2] > -self.cfg.tilt_limit
        _, dist = self._goal_body()
        self.reached = dist < self.cfg.goal_radius
        tilt = torch.rad2deg(torch.acos((-grav[:, 2]).clamp(-1.0, 1.0)))
        self.max_tilt = torch.maximum(self.max_tilt, tilt)
        vx = self.robot.data.root_lin_vel_b.torch[:, 0]
        self.max_accel = torch.maximum(self.max_accel, (vx - self.prev_vx).abs() / self.step_dt)
        self.prev_vx = vx.clone()
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        return self.tipped | self.reached, time_out

    def _get_rewards(self) -> torch.Tensor:
        _, dist = self._goal_body()
        progress = self.prev_dist - dist
        self.prev_dist = dist
        rew = (
            self.cfg.rew_progress * progress
            + self.cfg.rew_reached * self.reached.float()
            + self.cfg.rew_action_rate * torch.sum(torch.square(self.actions - self.prev_actions), dim=1)
            + self.cfg.rew_time
            + self.cfg.rew_tipped * self.tipped.float()
        )
        log = self.extras.setdefault("log", {})
        log["Metrics/goal_distance"] = dist.mean().item()
        log["Metrics/vx_mean"] = self.robot.data.root_lin_vel_b.torch[:, 0].mean().item()
        return rew

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        if not hasattr(self, "base_cmd"):  # 부모 __init__ 중
            super()._reset_idx(env_ids)
            return
        ids = env_ids if isinstance(env_ids, torch.Tensor) else torch.as_tensor(env_ids, device=self.device)
        # 끝난 에피소드 기록 (super 가 episode_length_buf 를 0 으로 만들기 전에)
        t = self.episode_length_buf[ids].float() * self.step_dt
        rec = self.last_episode
        rec["success"][ids] = self.reached[ids].float()
        rec["time"][ids] = t
        rec["ideal_time"][ids] = self.ideal_time[ids]
        rec["tipped"][ids] = self.tipped[ids].float()
        rec["max_tilt"][ids] = self.max_tilt[ids]
        rec["max_accel"][ids] = self.max_accel[ids]
        if len(ids) > 0:
            log = self.extras.setdefault("log", {})
            ok = self.reached[ids]
            log["Metrics/success_rate"] = ok.float().mean().item()
            if ok.any():
                log["Metrics/time_to_goal"] = t[ok].mean().item()
        super()._reset_idx(ids)

        n = len(ids)
        if self._tasks is not None:
            task = self._tasks[ids, self._task_k[ids] % self.cfg.fixed_tasks_per_env]
            self._task_k[ids] += 1
            yaw, r, th = task[:, 0], task[:, 1], task[:, 2]
        else:
            yaw = sample_uniform(self.cfg.start_yaw_range[0], self.cfg.start_yaw_range[1], (n,), self.device)
            r = sample_uniform(self.cfg.goal_distance_range[0], self.cfg.goal_distance_range[1], (n,), self.device)
            th = sample_uniform(-math.pi, math.pi, (n,), self.device)
        root = self.robot.data.default_root_pose.torch[ids].clone()
        root[:, :3] += self.scene.env_origins[ids]
        zeros = torch.zeros_like(yaw)
        root[:, 3:7] = quat_from_euler_xyz(zeros, zeros, yaw)
        self.robot.write_root_pose_to_sim_index(root_pose=root, env_ids=ids)
        self.robot.write_root_velocity_to_sim_index(root_velocity=self.robot.data.default_root_vel.torch[ids].clone(), env_ids=ids)
        self.robot.write_joint_position_to_sim_index(position=self.robot.data.default_joint_pos.torch[ids].clone(), env_ids=ids)
        self.robot.write_joint_velocity_to_sim_index(velocity=self.robot.data.default_joint_vel.torch[ids].clone(), env_ids=ids)
        self.goal_w[ids] = self.scene.env_origins[ids, :2] + torch.stack((r * torch.cos(th), r * torch.sin(th)), dim=1)
        self.prev_dist[ids] = r
        self.ideal_time[ids] = _wrap(th - yaw).abs() / SKILL_WZ + r / SKILL_VX
        self.actions[ids] = 0.0
        self.prev_actions[ids] = 0.0
        self.reached[ids] = False
        self.tipped[ids] = False
        self.max_tilt[ids] = 0.0
        self.max_accel[ids] = 0.0
        self.prev_vx[ids] = 0.0
        self.base_cmd.reset(ids)
