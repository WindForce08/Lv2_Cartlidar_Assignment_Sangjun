"""ArmBench-PickPlace 환경 (Direct RL). 설정 · 관측 · 보상 설명은 pick_env_cfg.py."""

from __future__ import annotations

import math
import os
from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

import isaaclab.sim as sim_utils
from isaaclab import cloner
from isaaclab.assets import Articulation, RigidObject
from isaaclab.envs import DirectRLEnv
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.utils.math import quat_apply, sample_uniform

from . import kinematics as K
from .pick_env_cfg import MATERIAL, PUCK_HEIGHT
from .robot_cfg import ARM_JOINTS, FINGER_CLOSED, FINGER_JOINTS, FINGER_OPEN

if TYPE_CHECKING:
    from .pick_env_cfg import ArmPickEnvCfg

ENV0 = "/World/envs/env_0"
REST_Z = PUCK_HEIGHT / 2


class ArmPickEnv(DirectRLEnv):
    """테이블 위 퍽을 옆에서 (집게 수평) 집어 목표 자리에 세워 놓기."""

    cfg: ArmPickEnvCfg

    def __init__(self, cfg: ArmPickEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self._arm_ids, _ = self.robot.find_joints(ARM_JOINTS, preserve_order=True)
        self._finger_ids, _ = self.robot.find_joints(FINGER_JOINTS, preserve_order=True)
        self._grip_body = self.robot.find_bodies("gripper_link")[0][0]
        n, dev = self.num_envs, self.device
        self.actions = torch.zeros(n, 4, device=dev)
        self.prev_actions = torch.zeros_like(self.actions)
        self.cmd_tip = torch.zeros(n, 3, device=dev)
        self.q_target = torch.zeros(n, len(ARM_JOINTS), device=dev)
        self.finger_target = torch.full((n,), FINGER_OPEN, device=dev)
        self.target = torch.zeros(n, 2, device=dev)
        self._tip_offset = torch.tensor([K.TIP_IN_GRIPPER, 0.0, 0.0], device=dev).expand(n, 3)
        # 판정 (리셋 때 지우지 않음 - 끝난 이유를 eval 이 셈)
        z = torch.zeros(n, dtype=torch.bool, device=dev)
        self.grasped, self.lifted, self.was_lifted, self.placed, self.failed = z.clone(), z.clone(), z.clone(), z.clone(), z.clone()
        self._was_lifted_ep = z.clone()
        self._placed_ep = z.clone()          # 에피소드 안에서 한 번이라도 성공 상태
        self.placed_ever = z.clone()
        print("[arm_bench] " + __import__("arm_pick.robot_cfg", fromlist=["describe"]).describe())

    # ---------------------------------------------------------------- 장면
    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot_cfg)
        self.puck = RigidObject(self.cfg.puck_cfg)
        spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg(physics_material=MATERIAL))
        pos = cloner.grid_transforms(self.scene.num_envs, self.scene.cfg.env_spacing, device=self.device)[0]
        plan = cloner.clone_plan_from_env_0(ENV0, "/World/envs/env_{}", self.scene.num_envs, self.device, pos,
                                            global_paths=("/World/ground",))
        cloner.replicate(plan, stage=self.scene.stage)
        if "physx" in self.scene.physics_backend:
            self.scene.filter_collisions(global_prim_paths=["/World/ground"])
        self.scene.articulations["robot"] = self.robot
        self.scene.rigid_objects["puck"] = self.puck
        light = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.8, 0.8, 0.8))
        light.func("/World/Light", light)
        self.markers = None
        if self.cfg.show_target and os.environ.get("HEADLESS") != "1":
            from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

            self.markers = VisualizationMarkers(VisualizationMarkersCfg(prim_path="/Visuals/Target", markers={
                "target": sim_utils.CylinderCfg(radius=0.018, height=0.002,
                                                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.8, 0.2)))}))

    # ---------------------------------------------------------------- 행동
    def _clamp_tip(self, p: torch.Tensor) -> torch.Tensor:
        c = self.cfg
        dx, y = p[:, 0] - K.J1_X, p[:, 1]
        d = torch.sqrt(dx * dx + y * y).clamp(min=1e-6)
        dc = d.clamp(*c.tip_d_range)
        return torch.stack((K.J1_X + dx * dc / d, y * dc / d, p[:, 2].clamp(*c.tip_z_range)), 1)

    def _ik(self, tip: torch.Tensor) -> torch.Tensor:
        pitch = torch.full_like(tip[:, 0], self.cfg.tool_pitch)
        return torch.stack(K.ik(tip[:, 0], tip[:, 1], tip[:, 2], pitch), 1)

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self.prev_actions = self.actions.clone()
        self.actions = actions.clamp(-1.0, 1.0)
        self.cmd_tip = self._clamp_tip(self.cmd_tip + self.actions[:, :3] * self.cfg.tip_step)
        self.q_target = self._ik(self.cmd_tip)
        self.finger_target = torch.where(self.actions[:, 3] > 0, FINGER_CLOSED, FINGER_OPEN)

    def _apply_action(self) -> None:
        cmd = self.robot.actuators.target_command
        cmd.set_position_index(value=self.q_target.contiguous(), joint_ids=self._arm_ids)
        cmd.set_position_index(value=self.finger_target.unsqueeze(1).expand(-1, len(self._finger_ids)).contiguous(),
                               joint_ids=self._finger_ids)

    # ---------------------------------------------------------------- 상태
    def tip_pos(self) -> torch.Tensor:
        """집게 끝 (end_effector_link) 위치 - 환경 좌표 = arm_base 좌표"""
        d = self.robot.data
        p = d.body_pos_w.torch[:, self._grip_body] + quat_apply(d.body_quat_w.torch[:, self._grip_body], self._tip_offset)
        return p - self.scene.env_origins

    def finger(self) -> torch.Tensor:
        return self.robot.data.joint_pos.torch[:, self._finger_ids].mean(1)

    def puck_state(self):
        d = self.puck.data
        pos = d.root_pos_w.torch - self.scene.env_origins
        qx, qy = d.root_quat_w.torch[:, 0], d.root_quat_w.torch[:, 1]   # (x, y, z, w)
        up_z = 1.0 - 2.0 * (qx * qx + qy * qy)                           # 퍽 축 z 성분
        return pos, up_z

    def _update_state(self):
        c = self.cfg
        self.tip = self.tip_pos()
        self.f = self.finger()
        self.puck_pos, self.up_z = self.puck_state()
        grasp_pt = self.puck_pos + torch.tensor([0.0, 0.0, c.grasp_tip_above_center], device=self.device)
        self.d_reach = torch.linalg.norm(self.tip - grasp_pt, dim=1)
        horiz = torch.linalg.norm(self.tip[:, :2] - self.puck_pos[:, :2], dim=1)
        closing = self.finger_target < 0
        blocked = (self.f > FINGER_CLOSED + 0.0015) & (self.f < 0.0)     # 손가락이 퍽에 막힘 (빈 손이면 -0.011 까지 닫힘)
        self.horiz = horiz
        inside = (self.tip[:, 2] < self.puck_pos[:, 2] + PUCK_HEIGHT / 2) & (self.tip[:, 2] > self.puck_pos[:, 2] - PUCK_HEIGHT / 2)
        self.grasped = closing & blocked & (horiz < 0.015) & inside
        self.held = self.grasped & (self.puck_pos[:, 2] - REST_Z > 0.005)
        self.lifted = self.grasped & (self.puck_pos[:, 2] - REST_Z > c.lift_height)
        self._was_lifted_ep |= self.lifted
        self.d_place = torch.linalg.norm(self.puck_pos[:, :2] - self.target, dim=1)
        upright = self.up_z > c.upright_min
        self.on_target = self._was_lifted_ep & (self.d_place < c.place_radius) & ((self.puck_pos[:, 2] - REST_Z).abs() < 0.004) & upright
        self.retreat = torch.maximum((horiz / 0.04).clamp(0.0, 1.0),
                                     ((self.tip[:, 2] - self.puck_pos[:, 2] - PUCK_HEIGHT / 2) / 0.035).clamp(0.0, 1.0))
        self.clear = (self.finger_target > 0) & (self.f > 0.0) & (self.retreat >= 1.0)   # 놓고 옆이나 위로 빠짐
        self.placed = self.on_target & self.clear
        self._placed_ep |= self.placed
        d_axis = torch.linalg.norm(self.puck_pos[:, :2] - torch.tensor([K.J1_X, 0.0], device=self.device), dim=1)
        self.failed = (self.up_z < c.tipped_max) | (d_axis > c.lost_d) | (self.puck_pos[:, 2] < -0.05)

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        self._update_state()
        self.was_lifted = self._was_lifted_ep.clone()
        self.placed_ever = self._placed_ep.clone()
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        return self.failed, time_out

    def _get_rewards(self) -> torch.Tensor:
        c = self.cfg
        carrying = (self.grasped & self._was_lifted_ep).float()          # 들었던 퍽을 쥐고 있음 (목표에 내려놓아도)
        on_t = self.on_target.float()
        open_frac = ((self.f - FINGER_CLOSED) / (FINGER_OPEN - FINGER_CLOSED)).clamp(0.0, 1.0)
        near_t = carrying * (self.d_place < 2 * c.place_radius).float()
        rew = (
            c.rew_reach * (1.0 - torch.tanh(self.d_reach / 0.04)) * (1.0 - on_t)
            + c.rew_grasp * self.grasped.float()
            + c.rew_lift * self.lifted.float()
            + c.rew_carry * carrying * (1.0 - torch.tanh(self.d_place / 0.05))
            + c.rew_carry_fine * carrying * (1.0 - torch.tanh(self.d_place / 0.01))
            + c.rew_lower * near_t * (1.0 - torch.tanh((self.puck_pos[:, 2] - REST_Z).clamp(min=0.0) / 0.01))
            + c.rew_on_target * on_t
            + c.rew_release * on_t * (open_frac + self.retreat)
            + c.rew_placed * self.placed.float()
            + c.rew_action_rate * torch.sum(torch.square(self.actions - self.prev_actions), dim=1)
            + c.rew_fail * self.failed.float()
        )
        log = self.extras.setdefault("log", {})
        log["Metrics/reach_m"] = self.d_reach.mean().item()
        log["Metrics/grasped_frac"] = self.grasped.float().mean().item()
        log["Metrics/lifted_frac"] = self.lifted.float().mean().item()
        return rew

    # ---------------------------------------------------------------- 관측
    def _get_observations(self) -> dict:
        tip, f = self.tip_pos(), self.finger()
        puck, up_z = self.puck_state()
        f_n = (f - FINGER_CLOSED) / (FINGER_OPEN - FINGER_CLOSED) * 2.0 - 1.0
        q = self.robot.data.joint_pos.torch[:, self._arm_ids]
        obs = torch.cat((tip, f_n.unsqueeze(1), puck, puck - tip, up_z.unsqueeze(1), self.target, self.target - puck[:, :2], q,
                         self.actions, self._was_lifted_ep.float().unsqueeze(1)), dim=-1)
        return {"policy": obs}

    # ---------------------------------------------------------------- 리셋
    def _sample_xy(self, n: int) -> torch.Tensor:
        c = self.cfg
        d = sample_uniform(c.puck_d_range[0], c.puck_d_range[1], (n,), self.device)
        a = sample_uniform(c.puck_angle_range[0], c.puck_angle_range[1], (n,), self.device)
        return torch.stack((K.J1_X + d * torch.cos(a), d * torch.sin(a)), 1)

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        if len(env_ids) > 0 and self.common_step_counter > 0:
            log = self.extras.setdefault("log", {})
            log["Metrics/success_rate"] = self.placed[env_ids].float().mean().item()          # 끝날 때 성공 상태
            log["Metrics/placed_ever_rate"] = self._placed_ep[env_ids].float().mean().item()
            log["Metrics/failed_rate"] = self.failed[env_ids].float().mean().item()
            log["Metrics/was_lifted_rate"] = self._was_lifted_ep[env_ids].float().mean().item()
        super()._reset_idx(env_ids)
        c, n, dev = self.cfg, len(env_ids), self.device
        # 팔: 시작 집게 끝 (+ 흔들기) 의 IK 자세, 손가락 열림
        tip0 = torch.tensor(c.start_tip, device=dev).repeat(n, 1) + sample_uniform(-c.start_tip_noise, c.start_tip_noise, (n, 3), dev)
        tip0 = self._clamp_tip(tip0)
        q0 = self._ik(tip0)
        jp = self.robot.data.default_joint_pos.torch[env_ids].clone()
        jp[:, self._arm_ids] = q0
        jp[:, self._finger_ids] = FINGER_OPEN
        self.robot.write_joint_position_to_sim_index(position=jp, env_ids=env_ids)
        self.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(jp), env_ids=env_ids)
        self.cmd_tip[env_ids] = tip0
        self.q_target[env_ids] = q0
        self.finger_target[env_ids] = FINGER_OPEN
        # 퍽 · 목표
        if c.fixed_task is not None:
            puck_xy = torch.tensor(c.fixed_task[0], device=dev).repeat(n, 1)
            target = torch.tensor(c.fixed_task[1], device=dev).repeat(n, 1)
        else:
            puck_xy = self._sample_xy(n)
            target = self._sample_xy(n)
            for _ in range(8):                                   # 퍽과 너무 가까운 목표는 다시 뽑음
                close = torch.linalg.norm(target - puck_xy, dim=1) < c.target_min_sep
                if not close.any():
                    break
                target[close] = self._sample_xy(int(close.sum()))
        root = self.puck.data.default_root_pose.torch[env_ids].clone()
        root[:, :2] = puck_xy
        root[:, 2] = REST_Z + 0.001
        root[:, :3] += self.scene.env_origins[env_ids]
        root[:, 3:7] = torch.tensor([0.0, 0.0, 0.0, 1.0], device=dev)
        self.puck.write_root_pose_to_sim_index(root_pose=root, env_ids=env_ids)
        self.puck.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=env_ids)
        self.target[env_ids] = target
        self.actions[env_ids] = 0.0
        self.prev_actions[env_ids] = 0.0
        self._was_lifted_ep[env_ids] = False
        self._placed_ep[env_ids] = False
        if self.markers is not None:
            pos = torch.cat((self.target, torch.full((self.num_envs, 1), 0.001, device=dev)), 1) + self.scene.env_origins
            self.markers.visualize(translations=pos)
