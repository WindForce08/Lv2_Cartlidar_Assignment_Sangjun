"""미로 경로 따라가기 구현. 설정 · 과제 설명은 maze_env_cfg.py."""

from __future__ import annotations

import math
import os
from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

import isaaclab.sim as sim_utils
from isaaclab import cloner
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.utils.math import quat_from_euler_xyz

from maze_map import load_course

from .base_cmd import BaseCmd
from .maze_geom import MazeGeom, write_maze_usda
from .maze_env_cfg import DRIVE_RL_DIR
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
    from .maze_env_cfg import WaffleDriveMazeEnvCfg

REWARD_TERMS = ("progress", "heading", "reached", "time", "action_rate", "near_wall", "hit_wall", "hit_object", "tipped")
END_KEYS = ("success", "hit_wall", "hit_object", "tipped", "timeout")
"""에피소드가 끝난 이유 (로그 Episode_End/<이유> = 끝난 에피소드 중 비율)"""


def _wrap(a: torch.Tensor) -> torch.Tensor:
    return torch.atan2(torch.sin(a), torch.cos(a))


class WaffleDriveMazeEnv(DirectRLEnv):
    """경기장 여러 벌에서 경로의 웨이포인트를 차례로 지나기 (구간 학습 + 지날 점 수 커리큘럼)."""

    cfg: WaffleDriveMazeEnvCfg

    def __init__(self, cfg: WaffleDriveMazeEnvCfg, render_mode: str | None = None, **kwargs):
        self._course = load_course(cfg.course_file)
        super().__init__(cfg, render_mode, **kwargs)
        n, dev = self.num_envs, self.device
        self._left_ids, _ = self.robot.find_joints(WHEEL_JOINTS_LEFT, preserve_order=True)
        self._right_ids, _ = self.robot.find_joints(WHEEL_JOINTS_RIGHT, preserve_order=True)
        self._hold_ids, _ = self.robot.find_joints(ARM_JOINTS + FINGER_JOINTS, preserve_order=True)
        self._wheel_ids = list(self._left_ids) + list(self._right_ids)
        self.base_cmd = BaseCmd(n, dev, WHEEL_RADIUS, WHEEL_SEPARATION, WHEEL_SPEED_MAX, cfg.sim.dt, cfg.action_mode)
        self.geom = MazeGeom(dev)
        # 경로 (M 점)
        c = self._course
        self.wp = torch.tensor([[p["x"], p["y"]] for p in c], device=dev)
        self.wp_stop = torch.tensor([p["stop"] is not None for p in c], device=dev)
        self.wp_head = torch.tensor([p["heading"] if p["stop"] else 0.0 for p in c], device=dev)
        self.M = len(c)
        nxt = torch.cat((self.wp[1:], self.wp[-1:])) - self.wp
        self.wp_out_yaw = torch.where(self.wp_stop, self.wp_head, torch.atan2(nxt[:, 1], nxt[:, 0]))  # 시작 방향
        # 환경마다 상태
        self.actions = torch.zeros(n, 2, device=dev)
        self.prev_actions = torch.zeros_like(self.actions)
        self.wp_idx = torch.ones(n, dtype=torch.long, device=dev)  # 지금 가는 점
        self.end_idx = torch.ones(n, dtype=torch.long, device=dev)  # 이 점까지 가면 성공
        self.start_idx = torch.zeros(n, dtype=torch.long, device=dev)
        self.wp_steps = torch.zeros(n, dtype=torch.long, device=dev)  # 지금 점으로 간 스텝 수
        self.hold = torch.zeros(n, dtype=torch.long, device=dev)  # 정지 조건 유지 스텝 수
        self.prev_dist = torch.zeros(n, device=dev)
        self.prev_yaw_err = torch.zeros(n, device=dev)
        self.n_reached = torch.zeros(n, dtype=torch.long, device=dev)
        self.ep_steps = torch.zeros(n, dtype=torch.long, device=dev)  # 이 에피소드 스텝 수 (episode_length_buf 는 학습 시작 때 무작위로 채워짐)
        self.end = {k: torch.zeros(n, dtype=torch.bool, device=dev) for k in END_KEYS}
        self.reached_now = torch.zeros(n, dtype=torch.bool, device=dev)
        self.clear = torch.ones(n, device=dev)
        self._terms = {k: torch.zeros(n, device=dev) for k in REWARD_TERMS}
        self._ep_sums = {k: torch.zeros(n, device=dev) for k in REWARD_TERMS}
        self._hold_steps = max(1, round(cfg.stop_hold_s / self.step_dt))
        self._timeout_steps = round(cfg.wp_timeout_s / self.step_dt)
        # 커리큘럼
        self.k_max = self.M - 1 if cfg.curriculum_max_k <= 0 else min(cfg.curriculum_max_k, self.M - 1)
        if cfg.full_course:
            self.k = self.k_max if cfg.full_course_k <= 0 else min(cfg.full_course_k, self.k_max)
        else:
            self.k = min(cfg.curriculum_start_k, self.k_max)
        self.ep_k = torch.zeros(n, dtype=torch.long, device=dev)  # 에피소드를 시작할 때의 k
        self._min_eps = max(cfg.curriculum_min_episodes, n)  # 환경이 많으면 빨리 끝난 에피소드만으로 판단하지 않게
        self._cur_eps = 0
        self._cur_ok = 0
        if self.sim.has_gui:
            self.set_debug_vis(True)

    # ---- 장면 ----
    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot_cfg)
        spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg(physics_material=CONTACT_MATERIAL))
        # 보기용 미로 (충돌 없음) 를 env_0 에 놓으면 복제 때 모든 환경으로 같이 복사된다
        usd = write_maze_usda(os.path.join(DRIVE_RL_DIR, "cache", "maze_visual.usda"), self._course)
        maze_cfg = sim_utils.UsdFileCfg(usd_path=usd)
        maze_cfg.func("/World/envs/env_0/Maze", maze_cfg)
        src, dest = "/World/envs/env_0", "/World/envs/env_{}"
        pos = cloner.grid_transforms(self.scene.num_envs, self.scene.cfg.env_spacing, device=self.device)[0]
        plan = cloner.clone_plan_from_env_0(src, dest, self.scene.num_envs, self.device, pos, global_paths=("/World/ground",))
        cloner.replicate(plan, stage=self.scene.stage)
        if "physx" in self.scene.physics_backend:
            self.scene.filter_collisions(global_prim_paths=[])
        self.scene.articulations["robot"] = self.robot
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.8, 0.8, 0.8))
        light_cfg.func("/World/Light", light_cfg)

    # ---- 행동 (단계 A 와 같음) ----
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self.prev_actions = self.actions.clone()
        self.actions = actions.clamp(-1.0, 1.0)

    def _apply_action(self) -> None:
        wl, wr = self.base_cmd.step(self.actions)
        vel = torch.cat(
            (wl.unsqueeze(1).expand(-1, len(self._left_ids)), wr.unsqueeze(1).expand(-1, len(self._right_ids))), dim=1
        )
        cmd = self.robot.actuators.target_command
        cmd.set_velocity_index(value=vel.contiguous(), joint_ids=self._wheel_ids)
        cmd.set_position_index(
            value=self.robot.data.default_joint_pos.torch[:, self._hold_ids].contiguous(), joint_ids=self._hold_ids
        )

    # ---- 상태 ----
    def _pose(self) -> tuple[torch.Tensor, torch.Tensor]:
        """미로 좌표 위치 (N, 2) 와 방향 (N,)"""
        pos = self.robot.data.root_pos_w.torch[:, :2] - self.scene.env_origins[:, :2]
        return pos, self.robot.data.heading_w.torch

    def _to_body(self, p: torch.Tensor, pos: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
        d = p - pos
        c, s = torch.cos(yaw), torch.sin(yaw)
        return torch.stack((c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]), dim=1)

    def _target_err(self, pos: torch.Tensor, yaw: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """지금 점까지 거리 (N,) 와 바라볼 방향 오차 (N,) (지나가는 점이면 0)"""
        dist = torch.linalg.norm(self.wp[self.wp_idx] - pos, dim=1)
        yaw_err = torch.where(self.wp_stop[self.wp_idx], _wrap(self.wp_head[self.wp_idx] - yaw), torch.zeros_like(yaw))
        return dist, yaw_err

    # ---- 종료 · 보상 (step 안에서 _get_dones → _get_rewards → 재설정 → _get_observations 순) ----
    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        cfg = self.cfg
        pos, yaw = self._pose()
        vx = self.robot.data.root_lin_vel_b.torch[:, 0]
        wz = self.robot.data.root_ang_vel_b.torch[:, 2]
        wall_c, obj_c = self.geom.footprint_clearance(pos, yaw)
        hit_wall, hit_obj = wall_c < cfg.hit_margin, obj_c < cfg.hit_margin
        self.clear = torch.minimum(wall_c, obj_c)
        tipped = self.robot.data.projected_gravity_b.torch[:, 2] > -cfg.tilt_limit

        dist, yaw_err = self._target_err(pos, yaw)
        is_stop = self.wp_stop[self.wp_idx]
        still = (vx.abs() < cfg.stop_still_v) & (wz.abs() < cfg.stop_still_w)
        in_stop = is_stop & (dist < cfg.stop_pos_tol) & (yaw_err.abs() < cfg.stop_yaw_tol) & still
        self.hold = torch.where(in_stop, self.hold + 1, torch.zeros_like(self.hold))
        failed = hit_wall | hit_obj | tipped
        reached = torch.where(is_stop, self.hold >= self._hold_steps, dist < cfg.pass_radius) & ~failed

        # 보상 항 (점을 바꾸기 전 기준)
        t = self._terms
        t["progress"] = cfg.rew_progress * (self.prev_dist - dist)
        zone = (is_stop & (dist < cfg.heading_zone)).float()
        t["heading"] = cfg.rew_heading * (self.prev_yaw_err - yaw_err.abs()) * zone
        t["reached"] = cfg.rew_reached * reached.float()
        t["time"] = torch.full_like(dist, cfg.rew_time)
        t["action_rate"] = cfg.rew_action_rate * torch.sum(torch.square(self.actions - self.prev_actions), dim=1)
        t["near_wall"] = cfg.rew_near_wall * ((cfg.near_wall_dist - self.clear) / cfg.near_wall_dist).clamp(0.0, 1.0)
        t["hit_wall"] = cfg.rew_hit_wall * hit_wall.float()
        t["hit_object"] = cfg.rew_hit_object * hit_obj.float()
        t["tipped"] = cfg.rew_tipped * tipped.float()

        # 다음 점으로
        success = reached & (self.wp_idx >= self.end_idx)
        advance = reached & ~success
        self.wp_idx = torch.where(advance, (self.wp_idx + 1).clamp(max=self.M - 1), self.wp_idx)
        self.n_reached += reached.long()
        self.hold = torch.where(reached, torch.zeros_like(self.hold), self.hold)
        self.wp_steps = torch.where(reached, torch.zeros_like(self.wp_steps), self.wp_steps + 1)
        self.ep_steps += 1
        dist, yaw_err = self._target_err(pos, yaw)
        self.prev_dist, self.prev_yaw_err = dist, yaw_err.abs()
        self.reached_now = reached

        timeout = (self.wp_steps >= self._timeout_steps) | (self.episode_length_buf >= self.max_episode_length - 1)
        e = self.end
        e["success"], e["hit_wall"], e["hit_object"], e["tipped"] = success, hit_wall, hit_obj & ~hit_wall, tipped & ~hit_wall & ~hit_obj
        e["timeout"] = timeout & ~(success | failed)
        return success | failed, e["timeout"]

    def _get_rewards(self) -> torch.Tensor:
        rew = torch.zeros(self.num_envs, device=self.device)
        for k, v in self._terms.items():
            rew += v
            self._ep_sums[k] += v
        return rew

    # ---- 관측 ----
    def _get_observations(self) -> dict:
        cfg = self.cfg
        pos, yaw = self._pose()
        data = self.robot.data
        tgt = self._to_body(self.wp[self.wp_idx], pos, yaw)
        nxt = self._to_body(self.wp[(self.wp_idx + 1).clamp(max=self.M - 1)], pos, yaw)
        dist, yaw_err = self._target_err(pos, yaw)
        is_stop = self.wp_stop[self.wp_idx].float()
        lidar = self.geom.lidar(pos, yaw, cfg.lidar_beams, cfg.lidar_max_range) / cfg.lidar_max_range
        obs = torch.cat(
            (
                data.root_lin_vel_b.torch[:, 0:1],
                data.root_ang_vel_b.torch[:, 2:3],
                data.projected_gravity_b.torch,
                tgt,
                dist.unsqueeze(1),
                is_stop.unsqueeze(1),
                (torch.sin(yaw_err) * is_stop).unsqueeze(1),
                (torch.cos(yaw_err) * is_stop).unsqueeze(1),
                nxt,
                (self.hold.float() / self._hold_steps).unsqueeze(1),
                self.base_cmd.vx.unsqueeze(1),
                self.actions,
                lidar,
            ),
            dim=-1,
        )
        return {"policy": obs}

    # ---- 재설정 ----
    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        if not hasattr(self, "geom"):  # 부모 __init__ 중
            super()._reset_idx(env_ids)
            return
        ids = env_ids if isinstance(env_ids, torch.Tensor) else torch.as_tensor(env_ids, device=self.device)
        self._log_finished(ids)
        super()._reset_idx(ids)
        cfg, n, dev = self.cfg, len(ids), self.device

        # 시작 점 s (마지막 점 제외), 목표 = s + k 까지
        s = torch.randint(0, self.M - 1, (n,), device=dev)
        s = torch.where(torch.rand(n, device=dev) < cfg.start0_prob, torch.zeros_like(s), s)
        noise = 0.0 if cfg.full_course else 1.0
        if cfg.full_course:
            s.fill_(cfg.full_course_start)
        pos = self.wp[s] + (torch.rand(n, 2, device=dev) * 2 - 1) * cfg.spawn_pos_noise * noise
        yaw = self.wp_out_yaw[s] + (torch.rand(n, device=dev) * 2 - 1) * cfg.spawn_yaw_noise * noise
        wall_c, obj_c = self.geom.footprint_clearance(pos, yaw)
        bad = torch.minimum(wall_c, obj_c) < 0.02
        pos = torch.where(bad[:, None], self.wp[s], pos)
        yaw = torch.where(bad, self.wp_out_yaw[s], yaw)

        root = self.robot.data.default_root_pose.torch[ids].clone()
        root[:, :2] = pos + self.scene.env_origins[ids, :2]
        root[:, 2] += self.scene.env_origins[ids, 2]
        zeros = torch.zeros_like(yaw)
        root[:, 3:7] = quat_from_euler_xyz(zeros, zeros, yaw)
        self.robot.write_root_pose_to_sim_index(root_pose=root, env_ids=ids)
        self.robot.write_root_velocity_to_sim_index(root_velocity=self.robot.data.default_root_vel.torch[ids].clone(), env_ids=ids)
        self.robot.write_joint_position_to_sim_index(position=self.robot.data.default_joint_pos.torch[ids].clone(), env_ids=ids)
        self.robot.write_joint_velocity_to_sim_index(velocity=self.robot.data.default_joint_vel.torch[ids].clone(), env_ids=ids)

        self.start_idx[ids] = s
        self.wp_idx[ids] = s + 1
        self.end_idx[ids] = (s + self.k).clamp(max=self.M - 1)
        self.ep_k[ids] = self.k
        nxt = self.wp[s + 1]
        self.prev_dist[ids] = torch.linalg.norm(nxt - pos, dim=1)
        self.prev_yaw_err[ids] = torch.where(self.wp_stop[s + 1], _wrap(self.wp_head[s + 1] - yaw).abs(), zeros)
        self.wp_steps[ids] = 0
        self.hold[ids] = 0
        self.n_reached[ids] = 0
        self.ep_steps[ids] = 0
        self.actions[ids] = 0.0
        self.prev_actions[ids] = 0.0
        for v in self._ep_sums.values():
            v[ids] = 0.0
        self.base_cmd.reset(ids)

    def _log_finished(self, ids: torch.Tensor) -> None:
        """끝난 에피소드 통계 → extras["log"] (텐서보드), 커리큘럼 k 올리기"""
        ids = ids[self.ep_steps[ids] > 0]  # 처음 재설정 (아직 안 달린 환경) 은 뺌
        if len(ids) == 0:
            return
        log = self.extras.setdefault("log", {})
        for k in END_KEYS:
            log[f"Episode_End/{k}"] = self.end[k][ids].float().mean().item()
        for k, v in self._ep_sums.items():
            log[f"Episode_Reward/{k}"] = v[ids].mean().item()
        log["Metrics/waypoints_reached"] = self.n_reached[ids].float().mean().item()
        log["Metrics/episode_time"] = (self.ep_steps[ids].float() * self.step_dt).mean().item()
        s0 = ids[self.start_idx[ids] == 0]
        if len(s0):  # 출발점에서 시작한 에피소드: 2 점 (A0 · B0) 넘게 지났으면 첫 모서리 통과
            log["Metrics/start0_passed_b0"] = (self.n_reached[s0] >= 2).float().mean().item()
        if self.cfg.full_course:  # 회차마다 결과를 터미널에
            pos, yaw = self._pose()
            for i in ids.tolist():
                why = next((k for k in END_KEYS if bool(self.end[k][i])), "?")
                p = self._course[int(self.wp_idx[i])]
                print(f"[maze] 환경 {i}: {why} · 지난 점 {int(self.n_reached[i])}/{int(self.end_idx[i] - self.start_idx[i])} · {float(self.ep_steps[i]) * self.step_dt:.1f} s"
                      f" · 가던 점 {int(self.wp_idx[i])} ({p['stop'] or p['cell']}) · 위치 ({float(pos[i, 0]):.3f}, {float(pos[i, 1]):.3f})"
                      f" 방향 {math.degrees(float(yaw[i])):.0f}°", flush=True)
        # 성공률은 지금 k 로 시작한 에피소드만 (k 가 오른 직후 이전 k 로 달리던 에피소드가 끝나며 섞이지 않게)
        cur = ids[self.ep_k[ids] == self.k]
        self._cur_eps += len(cur)
        self._cur_ok += int(self.end["success"][cur].sum().item())
        if self._cur_eps >= self._min_eps:
            rate = self._cur_ok / self._cur_eps
            log["Curriculum/success_rate"] = rate
            if rate >= self.cfg.curriculum_success and self.k < self.k_max:
                self.k += 1
                print(f"[maze] 커리큘럼: 성공률 {rate:.2f} → 지날 점 수 k = {self.k}", flush=True)
            self._cur_eps = self._cur_ok = 0
        log["Curriculum/k"] = float(self.k)

    # ---- 창에 다음 점 표시 (창이 있을 때만) ----
    def _set_debug_vis_impl(self, debug_vis: bool):
        if debug_vis and not hasattr(self, "_markers"):
            mk = lambda r, rgb: sim_utils.SphereCfg(radius=r, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=rgb))  # noqa: E731
            self._markers = VisualizationMarkers(VisualizationMarkersCfg(
                prim_path="/Visuals/drive_target",
                markers={"pass": mk(0.05, (0.1, 0.8, 0.2)), "stop": mk(0.06, (0.9, 0.1, 0.3)), "face": mk(0.03, (1.0, 0.8, 0.1))},
            ))
        if hasattr(self, "_markers"):
            self._markers.set_visibility(debug_vis)

    def _debug_vis_callback(self, event):
        if not hasattr(self, "wp_idx"):
            return
        n = self.num_envs
        tgt = self.wp[self.wp_idx] + self.scene.env_origins[:, :2]
        h = self.wp_head[self.wp_idx]
        stop = self.wp_stop[self.wp_idx]
        face = tgt + torch.where(stop[:, None], 0.12 * torch.stack((torch.cos(h), torch.sin(h)), 1), torch.zeros_like(tgt))
        z = torch.full((n, 1), 0.12, device=self.device)
        trans = torch.cat((torch.cat((tgt, z), 1), torch.cat((face, z), 1)))
        idx = torch.cat((stop.long(), torch.where(stop, 2, 0)))
        envs = torch.arange(n, device=self.device).repeat(2)
        self._markers.visualize(translations=trans, marker_indices=idx, environment_ids=envs)
