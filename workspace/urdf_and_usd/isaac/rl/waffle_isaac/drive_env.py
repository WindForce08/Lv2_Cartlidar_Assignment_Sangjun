"""Waffle 주행 과제 `Waffle-Drive-Direct` 구현. 설정은 drive_env_cfg.py."""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.utils.math import quat_from_euler_xyz, sample_uniform

from .robot_cfg import WHEEL_JOINTS_LEFT, WHEEL_JOINTS_RIGHT
from waffle_4wd_isaac.robot_cfg import ARM_JOINTS, FINGER_JOINTS, WHEEL_SPEED_MAX  # 팀 설정 (.robot_cfg 가 sys.path 에 넣음)

if TYPE_CHECKING:
    from .drive_env_cfg import WaffleDriveEnvCfg


class WaffleDriveEnv(DirectRLEnv):
    """4륜 스키드 조향으로 무작위 목표점까지 주행."""

    cfg: WaffleDriveEnvCfg

    def __init__(self, cfg: WaffleDriveEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self._left_ids, _ = self.robot.find_joints(WHEEL_JOINTS_LEFT, preserve_order=True)
        self._right_ids, _ = self.robot.find_joints(WHEEL_JOINTS_RIGHT, preserve_order=True)
        self._hold_ids, _ = self.robot.find_joints(ARM_JOINTS + FINGER_JOINTS, preserve_order=True)
        self._wheel_ids = list(self._left_ids) + list(self._right_ids)
        self.actions = torch.zeros(self.num_envs, 2, device=self.device)
        self.prev_actions = torch.zeros_like(self.actions)
        self.goal_w = torch.zeros(self.num_envs, 2, device=self.device)
        self.prev_dist = torch.zeros(self.num_envs, device=self.device)
        self.reached = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

    @property
    def robot(self) -> Articulation:
        """장면 설정 (drive_env_cfg.DriveSceneCfg) 이 만든 로봇"""
        return self.scene["robot"]

    # ---- 행동 ----
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self.prev_actions = self.actions.clone()
        self.actions = actions.clamp(-1.0, 1.0)

    def _apply_action(self) -> None:
        vel = torch.cat(
            (self.actions[:, 0:1].expand(-1, len(self._left_ids)), self.actions[:, 1:2].expand(-1, len(self._right_ids))), dim=1
        ) * WHEEL_SPEED_MAX
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
        obs = torch.cat(
            (
                self.robot.data.root_lin_vel_b.torch[:, :2],
                self.robot.data.root_ang_vel_b.torch[:, 2:3],
                self.robot.data.projected_gravity_b.torch,
                g_b,
                dist.unsqueeze(1),
                self.actions,
            ),
            dim=-1,
        )
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        _, dist = self._goal_body()
        progress = self.prev_dist - dist
        self.prev_dist = dist
        self.reached = dist < self.cfg.goal_radius
        tipped = self.robot.data.projected_gravity_b.torch[:, 2] > -self.cfg.tilt_limit
        rew = (
            self.cfg.rew_progress * progress
            + self.cfg.rew_reached * self.reached.float()
            + self.cfg.rew_action_rate * torch.sum(torch.square(self.actions - self.prev_actions), dim=1)
            + self.cfg.rew_time
            + self.cfg.rew_tipped * tipped.float()
        )
        self.extras.setdefault("log", {})["Metrics/goal_distance"] = dist.mean().item()
        return rew

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        tipped = self.robot.data.projected_gravity_b.torch[:, 2] > -self.cfg.tilt_limit
        _, dist = self._goal_body()
        terminated = tipped | (dist < self.cfg.goal_radius)
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        return terminated, time_out

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        if len(env_ids) > 0 and hasattr(self, "reached"):
            self.extras.setdefault("log", {})["Metrics/success_rate"] = self.reached[env_ids].float().mean().item()
        super()._reset_idx(env_ids)
        n = len(env_ids)
        root = self.robot.data.default_root_pose.torch[env_ids].clone()
        root[:, :3] += self.scene.env_origins[env_ids]
        yaw = sample_uniform(self.cfg.start_yaw_range[0], self.cfg.start_yaw_range[1], (n,), self.device)
        zeros = torch.zeros_like(yaw)
        root[:, 3:7] = quat_from_euler_xyz(zeros, zeros, yaw)
        self.robot.write_root_pose_to_sim_index(root_pose=root, env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(root_velocity=self.robot.data.default_root_vel.torch[env_ids].clone(), env_ids=env_ids)
        self.robot.write_joint_position_to_sim_index(position=self.robot.data.default_joint_pos.torch[env_ids].clone(), env_ids=env_ids)
        self.robot.write_joint_velocity_to_sim_index(velocity=self.robot.data.default_joint_vel.torch[env_ids].clone(), env_ids=env_ids)
        r = sample_uniform(self.cfg.goal_distance_range[0], self.cfg.goal_distance_range[1], (n,), self.device)
        th = sample_uniform(-3.14159, 3.14159, (n,), self.device)
        self.goal_w[env_ids] = self.scene.env_origins[env_ids, :2] + torch.stack((r * torch.cos(th), r * torch.sin(th)), dim=1)
        self.prev_dist[env_ids] = r
        self.actions[env_ids] = 0.0
        self.prev_actions[env_ids] = 0.0
