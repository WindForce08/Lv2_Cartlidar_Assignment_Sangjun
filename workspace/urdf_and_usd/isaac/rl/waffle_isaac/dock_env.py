"""박스 앞 세우기 `Waffle-Dock-Direct` 구현. 설정 · 설명은 dock_env_cfg.py."""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch
import yaml

from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.utils.math import quat_from_euler_xyz, sample_uniform

from .robot_cfg import WHEEL_JOINTS_LEFT, WHEEL_JOINTS_RIGHT
from waffle_4wd_isaac.robot_cfg import ARM_JOINTS, FINGER_JOINTS, WHEEL_SPEED_MAX  # 팀 설정 (.robot_cfg 가 sys.path 에 넣음)

if TYPE_CHECKING:
    from .dock_env_cfg import WaffleDockEnvCfg

RESULTS = {1: "성공", 2: "면 넘음", 3: "차체 접촉", 4: "벗어남", 5: "시간 초과"}
FRONT = 0.1328  # base_link 에서 앞 끝 (상판) x, mission_planner/poses.py 와 같음
HALF_W = 0.1676  # 바퀴 바깥 y


def _wrap(a: torch.Tensor) -> torch.Tensor:
    return torch.atan2(torch.sin(a), torch.cos(a))


def load_targets(path: str) -> tuple[list[str], list[tuple[float, float, float, float]]]:
    """approach_poses.yaml -> ok 인 box · place 의 (이름, (x, y, yaw, gap))"""
    with open(path) as f:
        d = yaml.safe_load(f)
    names, vals = [], []
    for k, v in d.items():
        if v.get("ok") and v.get("kind") in ("box", "place") and v.get("gap") is not None:
            x, y, yaw = (float(a) for a in v["final"])
            names.append(k)
            vals.append((x, y, yaw, float(v["gap"])))
    if not vals:
        raise RuntimeError(f"접근 자세가 없음: {path}")
    return names, vals


class WaffleDockEnv(DirectRLEnv):
    """4륜 스키드 조향으로 박스 · 놓기 자리 앞 접근 자세에 바짝 세우기."""

    cfg: WaffleDockEnvCfg

    def __init__(self, cfg: WaffleDockEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self._left_ids, _ = self.robot.find_joints(WHEEL_JOINTS_LEFT, preserve_order=True)
        self._right_ids, _ = self.robot.find_joints(WHEEL_JOINTS_RIGHT, preserve_order=True)
        self._hold_ids, _ = self.robot.find_joints(ARM_JOINTS + FINGER_JOINTS, preserve_order=True)
        self._wheel_ids = list(self._left_ids) + list(self._right_ids)
        self.target_names, vals = load_targets(self.cfg.approach_poses_path)
        self.targets = torch.tensor(vals, device=self.device)  # (T, 4) = x, y, yaw, gap (경기장 좌표)
        n = self.num_envs
        self.actions = torch.zeros(n, 2, device=self.device)
        self.prev_actions = torch.zeros_like(self.actions)
        self.tgt_idx = torch.zeros(n, dtype=torch.long, device=self.device)
        self.goal = torch.zeros(n, 3, device=self.device)  # 세계 좌표 x, y, yaw
        self.bias = torch.zeros(n, 3, device=self.device)  # 관측 치우침 (몸체 x, y, 요)
        self.prev_err = torch.zeros(n, device=self.device)
        self.success = torch.zeros(n, dtype=torch.bool, device=self.device)
        self.collided = torch.zeros(n, dtype=torch.bool, device=self.device)
        self.lost = torch.zeros(n, dtype=torch.bool, device=self.device)
        self.hit_body = torch.zeros(n, dtype=torch.bool, device=self.device)
        # 평가용 (dock_eval.py): 목표 고정, 끝난 에피소드 결과 (RESULTS 번호) · 마지막 오차 (앞뒤, 옆, 요) · 길이 [스텝]
        self.eval_target: torch.Tensor | None = None
        self.last_err = torch.zeros(n, 3, device=self.device)
        self.ep_result = torch.zeros(n, dtype=torch.long, device=self.device)
        self.ep_err = torch.zeros(n, 3, device=self.device)
        self.ep_len = torch.zeros(n, dtype=torch.long, device=self.device)
        print(f"[Dock] 목표 {len(self.target_names)} 곳: {', '.join(self.target_names)}", flush=True)

    @property
    def robot(self) -> Articulation:
        return self.scene["robot"]

    # ---- 행동 ----
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self.prev_actions = self.actions.clone()
        self.actions = actions.clamp(-1.0, 1.0)

    def _apply_action(self) -> None:
        vel = torch.cat(
            (self.actions[:, 0:1].expand(-1, len(self._left_ids)), self.actions[:, 1:2].expand(-1, len(self._right_ids))), dim=1
        ) * (WHEEL_SPEED_MAX * self.cfg.wheel_scale)
        cmd = self.robot.actuators.target_command
        cmd.set_velocity_index(value=vel.contiguous(), joint_ids=self._wheel_ids)
        cmd.set_position_index(
            value=self.robot.data.default_joint_pos.torch[:, self._hold_ids].contiguous(), joint_ids=self._hold_ids
        )

    # ---- 오차 ----
    def _errors(self) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """참값 오차. 반환: 목표의 몸체 좌표 (gx, gy), 요 오차 (목표 - 로봇), final 좌표의 로봇 위치 (along, lat)."""
        pos = self.robot.data.root_pos_w.torch[:, :2]
        yaw = self.robot.data.heading_w.torch
        d = self.goal[:, :2] - pos
        c, s = torch.cos(yaw), torch.sin(yaw)
        gx, gy = c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]
        eyaw = _wrap(self.goal[:, 2] - yaw)
        cg, sg = torch.cos(self.goal[:, 2]), torch.sin(self.goal[:, 2])
        along = -(cg * d[:, 0] + sg * d[:, 1])  # 로봇이 final 보다 앞 (박스 쪽) 이면 +
        lat = -(-sg * d[:, 0] + cg * d[:, 1])
        return gx, gy, eyaw, along, lat

    def _front_over(self, along, lat, eyaw) -> torch.Tensor:
        """앞 끝 두 모서리 (FRONT, ±HALF_W) 가 박스 면 (final 좌표 along = FRONT + gap) 을 넘었는지 (평면 외곽, 옆은 따지지 않음)"""
        gap = self.targets[self.tgt_idx, 3]
        a = -eyaw  # final 좌표에서 로봇 요
        ca, sa = torch.cos(a), torch.sin(a)
        corner = along + FRONT * ca + HALF_W * sa.abs()
        return corner > FRONT + gap

    # ---- 관측 · 보상 · 종료 ----
    def _get_observations(self) -> dict:
        gx, gy, eyaw, _, _ = self._errors()
        n = self.num_envs
        gx = gx + self.bias[:, 0] + sample_uniform(-1.0, 1.0, (n,), self.device) * self.cfg.obs_noise_pos
        gy = gy + self.bias[:, 1] + sample_uniform(-1.0, 1.0, (n,), self.device) * self.cfg.obs_noise_pos
        ey = eyaw + self.bias[:, 2] + sample_uniform(-1.0, 1.0, (n,), self.device) * self.cfg.obs_noise_yaw
        obs = torch.cat(
            (
                torch.stack((gx, gy, torch.cos(ey), torch.sin(ey)), dim=1),
                self.robot.data.root_lin_vel_b.torch[:, :2],
                self.robot.data.root_ang_vel_b.torch[:, 2:3],
                self.actions,
            ),
            dim=-1,
        )
        return {"policy": obs}

    def _status(self):
        gx, gy, eyaw, along, lat = self._errors()
        dist = torch.sqrt(gx * gx + gy * gy)
        # 진행 오차: 옆 오차에 가중 (스키드 조향은 옆으로 못 가서, 거리만 보면 옆 오차를 남긴 채 앞으로만 감)
        err = torch.sqrt(along * along + (self.cfg.lat_weight * lat) ** 2) + self.cfg.yaw_weight * eyaw.abs()
        inside = (along.abs() < self.cfg.tol_along) & (lat.abs() < self.cfg.tol_lat) & (eyaw.abs() < self.cfg.tol_yaw)
        speed = torch.linalg.norm(self.robot.data.root_lin_vel_b.torch[:, :2], dim=1)
        wz = self.robot.data.root_ang_vel_b.torch[:, 2].abs()
        stopped = (speed < self.cfg.stop_speed) & (wz < 0.1)
        f = self.scene["body_contact"].data.net_forces_w.torch  # (N, 1, 3)
        contact = torch.linalg.norm(f[..., :2], dim=-1).amax(dim=1) > self.cfg.contact_force  # 수평 힘만 (바닥 긁힘의 수직 힘 뺌)
        over = self._front_over(along, lat, eyaw)
        return dist, err, inside, stopped, contact, over, along, lat, eyaw

    def _get_rewards(self) -> torch.Tensor:
        dist, err, inside, stopped, contact, over, along, lat, eyaw = self._status()
        progress = self.prev_err - err
        self.prev_err = err
        self.success = inside & stopped
        self.collided = contact | over
        self.hit_body = contact
        self.lost = dist > self.cfg.max_dist
        self.last_err = torch.stack((along, lat, eyaw), dim=1)
        rew = (
            self.cfg.rew_progress * progress
            + self.cfg.rew_success * self.success.float()
            + self.cfg.rew_near * inside.float()
            + self.cfg.rew_action_rate * torch.sum(torch.square(self.actions - self.prev_actions), dim=1)
            + self.cfg.rew_time
            + self.cfg.rew_collision * self.collided.float()
            + self.cfg.rew_lost * self.lost.float()
        )
        log = self.extras.setdefault("log", {})
        log["Metrics/err_along_mm"] = along.abs().mean().item() * 1000
        log["Metrics/err_lat_mm"] = lat.abs().mean().item() * 1000
        log["Metrics/err_yaw_deg"] = eyaw.abs().mean().item() * 57.2958
        log["Metrics/inside_ratio"] = inside.float().mean().item()
        log["Metrics/body_contact_ratio"] = contact.float().mean().item()
        return rew

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        dist, _, inside, stopped, contact, over, _, _, _ = self._status()
        terminated = (inside & stopped) | contact | over | (dist > self.cfg.max_dist)
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        return terminated, time_out

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        if not hasattr(self, "targets"):  # DirectRLEnv.__init__ 중 호출
            super()._reset_idx(env_ids)
            return
        if len(env_ids) > 0:
            log = self.extras.setdefault("log", {})
            log["Episode/success_rate"] = self.success[env_ids].float().mean().item()
            log["Episode/collision_rate"] = self.collided[env_ids].float().mean().item()
            log["Episode/body_contact_rate"] = self.hit_body[env_ids].float().mean().item()
            log["Episode/lost_rate"] = self.lost[env_ids].float().mean().item()
            code = torch.full((len(env_ids),), 5, dtype=torch.long, device=self.device)
            code[self.lost[env_ids]] = 4
            code[self.collided[env_ids]] = 2
            code[self.hit_body[env_ids]] = 3
            code[self.success[env_ids]] = 1
            self.ep_result[env_ids] = code
            self.ep_err[env_ids] = self.last_err[env_ids]
            self.ep_len[env_ids] = self.episode_length_buf[env_ids]
        super()._reset_idx(env_ids)
        n = len(env_ids)
        dev = self.device
        idx = torch.randint(0, self.targets.shape[0], (n,), device=dev) if self.eval_target is None else self.eval_target[env_ids]
        self.tgt_idx[env_ids] = idx
        t = self.targets[idx]
        org = self.scene.env_origins[env_ids, :2]
        self.goal[env_ids, :2] = org + t[:, :2]
        self.goal[env_ids, 2] = t[:, 2]
        # 시작: final 에서 뒤로 back, 옆 lat, 요 오차
        back = sample_uniform(self.cfg.back_range[0], self.cfg.back_range[1], (n,), dev)
        lat = sample_uniform(-self.cfg.lat_range, self.cfg.lat_range, (n,), dev)
        dyaw = sample_uniform(-self.cfg.yaw_range, self.cfg.yaw_range, (n,), dev)
        cg, sg = torch.cos(t[:, 2]), torch.sin(t[:, 2])
        x = self.goal[env_ids, 0] - back * cg - lat * sg
        y = self.goal[env_ids, 1] - back * sg + lat * cg
        yaw = t[:, 2] + dyaw
        root = self.robot.data.default_root_pose.torch[env_ids].clone()
        root[:, 0], root[:, 1] = x, y
        root[:, 2] += self.scene.env_origins[env_ids, 2]
        zeros = torch.zeros_like(yaw)
        root[:, 3:7] = quat_from_euler_xyz(zeros, zeros, yaw)
        self.robot.write_root_pose_to_sim_index(root_pose=root, env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(root_velocity=self.robot.data.default_root_vel.torch[env_ids].clone(), env_ids=env_ids)
        self.robot.write_joint_position_to_sim_index(position=self.robot.data.default_joint_pos.torch[env_ids].clone(), env_ids=env_ids)
        self.robot.write_joint_velocity_to_sim_index(velocity=self.robot.data.default_joint_vel.torch[env_ids].clone(), env_ids=env_ids)
        # 관측 치우침 (에피소드 고정)
        self.bias[env_ids, :2] = sample_uniform(-1.0, 1.0, (n, 2), dev) * self.cfg.obs_bias_pos
        self.bias[env_ids, 2] = sample_uniform(-1.0, 1.0, (n,), dev) * self.cfg.obs_bias_yaw
        self.prev_err[env_ids] = torch.sqrt(back * back + (self.cfg.lat_weight * lat) ** 2) + self.cfg.yaw_weight * dyaw.abs()
        self.actions[env_ids] = 0.0
        self.prev_actions[env_ids] = 0.0
        self.success[env_ids] = False
        self.collided[env_ids] = False
        self.lost[env_ids] = False
        self.hit_body[env_ids] = False
