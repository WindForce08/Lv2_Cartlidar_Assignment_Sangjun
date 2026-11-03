"""넣기 전용 `Waffle-Insert-Direct` 구현. 설정과 설명은 insert_env_cfg.py. 장면·판정 도구는 store_env 를 이어 씀."""

from __future__ import annotations

import json
import os
from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.envs import DirectRLEnv
from isaaclab.utils.math import quat_apply, quat_box_minus, sample_uniform

from .dice2_cfg import PUCK_H
from .insert_env_cfg import INSERT_POSE
from .robot_cfg import ARM_JOINTS, EE_BODY, FINGER_HOLD_30, ROOT_TO_BASE_LINK, WHEEL_REGEX
from .store_env import WaffleStoreEnv
from .train_log import TrainLog

if TYPE_CHECKING:
    from .insert_env_cfg import WaffleInsertEnvCfg

STARTS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "insert_starts.json")


class WaffleInsertEnv(WaffleStoreEnv):
    cfg: WaffleInsertEnvCfg

    def __init__(self, cfg: WaffleInsertEnvCfg, render_mode: str | None = None, **kwargs):
        # store_env 의 시작 상태 (store_starts.json) 대신 insert_starts.json
        DirectRLEnv.__init__(self, cfg, render_mode, **kwargs)
        r = self.robot
        self._arm, _ = r.find_joints(ARM_JOINTS, preserve_order=True)
        self._gl, _ = r.find_joints(["gripper_left_joint"])
        self._gr, _ = r.find_joints(["gripper_right_joint"])
        self._wheels, _ = r.find_joints([WHEEL_REGEX])
        self._a5 = r.find_bodies(EE_BODY)[0][0]
        lim = r.data.soft_joint_pos_limits.torch[0, self._arm].clone()
        self._lo, self._hi = lim[:, 0], lim[:, 1]
        n, dev = self.num_envs, self.device
        from .dice2_cfg import GRASP_OFFSET_A5
        S = json.load(open(os.path.join(os.path.dirname(STARTS), "store_starts.json"), encoding="utf-8"))
        self._tubes = torch.tensor([[t["axis"][0], t["axis"][1], t["rim_z"]] for t in S["tubes"]], device=dev)
        R = [x for x in json.load(open(STARTS, encoding="utf-8"))["rows"] if x["tube"] in cfg.tubes_used]
        self._iq = torch.tensor([x["q"] for x in R], device=dev)
        self._ifg = torch.tensor([x["finger"] for x in R], device=dev)
        self._ipk = torch.tensor([x["puck_b"] for x in R], device=dev)
        self._ipq = torch.tensor([x["puck_quat"] for x in R], device=dev)
        self._itube = torch.tensor([x["tube"] for x in R], device=dev)
        dep = torch.tensor([x["depth"] for x in R], device=dev)
        bucket = torch.where(dep < -0.002, 0, torch.where(dep < 0.006, 1, 2))
        w = torch.zeros(len(R), device=dev)
        for b in range(3):
            m = bucket == b
            if m.any():
                w[m] = cfg.start_weight[b] / m.sum()
        self._sw = w / w.sum()
        self._bucket_n = [int((bucket == b).sum()) for b in range(3)]
        self._bl = torch.tensor([[0.0, 0.0, ROOT_TO_BASE_LINK]], device=dev)
        self._off = torch.tensor([GRASP_OFFSET_A5], device=dev).repeat(n, 1)
        self._ex = torch.tensor([[1.0, 0.0, 0.0]], device=dev).repeat(n, 1)
        self._ey = torch.tensor([[0.0, 1.0, 0.0]], device=dev).repeat(n, 1)
        self._ez = torch.tensor([[0.0, 0.0, 1.0]], device=dev).repeat(n, 1)
        self._q0 = torch.tensor([[INSERT_POSE[j] for j in ARM_JOINTS]], device=dev)
        nb = getattr(r, "num_base_dofs", 6)
        self._cols = torch.tensor([nb + j for j in self._arm], device=dev)
        self.actions = torch.zeros(n, 4, device=dev)
        self.prev_actions = torch.zeros_like(self.actions)
        self.arm_target = r.data.default_joint_pos.torch[:, self._arm].clone()
        self.tube = torch.ones(n, dtype=torch.long, device=dev)
        self.calib = torch.zeros(n, 3, device=dev)
        self.q5_hold = torch.zeros(n, 4, device=dev)
        self.root0 = torch.zeros(n, 3, device=dev)
        self.ever_inserted = torch.zeros(n, dtype=torch.bool, device=dev)
        self.hold = torch.zeros(n, dtype=torch.long, device=dev)
        self.succeeded = torch.zeros(n, dtype=torch.bool, device=dev)
        self.contact_steps = torch.zeros(n, device=dev)
        self.start_bucket = torch.zeros(n, dtype=torch.long, device=dev)
        self.best_xy = torch.ones(n, device=dev)
        self.max_bottom = torch.zeros(n, device=dev)
        self.tlog = TrainLog(self, ("tipped", "dropped", "crash", "base_move"))  # 보상 항목별 · 종료 원인 · 행동 포화 기록 (종료는 store_env._get_dones)
        print(f"[Insert] 시작 상태 통 위 / 모따기 / 통 안 = {self._bucket_n}", flush=True)

    # ---- 행동: 잡는 점 위치 변화 -> IK ----
    def _pre_physics_step(self, actions):
        c, r, n = self.cfg, self.robot, self.num_envs
        self.prev_actions = self.actions.clone()
        self.actions = actions.clamp(-1.0, 1.0)
        dp_b = self.actions[:, :3] * c.cart_scale
        dp_w = quat_apply(r.data.root_link_quat_w.torch, dp_b)
        J = r.data.body_link_jacobian_w.torch[:, self._a5][:, :, self._cols]  # (N, 6, 5)
        q5 = r.data.body_link_quat_w.torch[:, self._a5]
        fresh = self.episode_length_buf < 2  # 리셋 직후엔 링크 자세가 아직 안 바뀜 -> 처음 두 스텝 동안 지금 자세를 유지 목표로
        self.q5_hold[fresh] = q5[fresh]
        rr = quat_apply(q5, self._off)
        Jl = J[:, 0:3] - torch.cross(rr.unsqueeze(-1).expand(-1, -1, 5), J[:, 3:6], dim=1)
        Jf = torch.cat((Jl, J[:, 3:6] * c.rot_hold_weight), dim=1)
        rot_err = quat_box_minus(self.q5_hold, q5) * c.rot_hold_weight
        err = torch.cat((dp_w, rot_err), dim=1)
        JJt = Jf @ Jf.transpose(1, 2) + c.ik_damping * torch.eye(6, device=self.device)
        dq = (Jf.transpose(1, 2) @ torch.linalg.solve(JJt, err.unsqueeze(-1))).squeeze(-1)
        q = r.data.joint_pos.torch[:, self._arm]
        lead = c.target_lead_max
        t = torch.clamp(self.arm_target + dq, q - lead, q + lead)
        self.tlog.actions(actions, self.arm_target + dq - q, lead, t, self._lo, self._hi)
        self.arm_target = torch.clamp(t, self._lo, self._hi)

    def _apply_action(self):
        cmd = self.robot.actuators.target_command
        g = torch.where(self.actions[:, 3:4] > 0.0, self.cfg.gripper_open, self.cfg.gripper_closed)
        cmd.set_position_index(value=self.arm_target.contiguous(), joint_ids=self._arm)
        cmd.set_position_index(value=g.expand(-1, 2).contiguous(), joint_ids=self._gl + self._gr)  # 두 손가락 같은 값
        cmd.set_velocity_index(value=torch.zeros(self.num_envs, len(self._wheels), device=self.device), joint_ids=self._wheels)

    # ---- 관측 ----
    def _get_observations(self):
        c, d = self.cfg, self.robot.data
        s = self._state()
        _, az = self.a5_axes_w()
        t_obs = s["t"] + self.calib
        est_bottom = s["gb"] - torch.tensor([[0.0, 0.0, c.grasp_dz + PUCK_H / 2]], device=self.device)
        obs = torch.cat((
            d.joint_pos.torch[:, self._arm] - self._q0, d.joint_vel.torch[:, self._arm], d.applied_torque.torch[:, self._arm],
            d.joint_pos.torch[:, self._gl], s["gb"], t_obs, est_bottom - t_obs, self.vec_bl(-az),
            torch.nn.functional.one_hot(self.tube, 3).float(), self.actions,
        ), dim=-1)
        return {"policy": obs, "critic": self._privileged()}

    def _privileged(self):
        s = self._state()
        f = torch.stack((self._force("finger_body_contact"), self._force("arm_body_contact"), self._force("puck_body_contact")), dim=1)
        return torch.cat((s["pc"] - s["gb"], s["u"], s["pb"] - s["t"], torch.clamp(f, max=50.0) / 10.0), dim=-1)

    # ---- 보상 ----
    def _get_rewards(self):
        c = self.cfg
        s = self._state()
        dxy, depth, held, u, fing = s["dxy"], s["depth"], s["held"], s["u"], s["fing"]
        up = u[:, 2]
        self.ever_inserted |= s["inside"]
        clear = -depth
        xy = 1 - torch.tanh(dxy / 0.002)
        depth_r = torch.clamp((depth + 0.005) / (c.depth_goal + 0.005), 0.0, 1.0) * (1 - torch.tanh(dxy / 0.004))
        released = s["inside"] & (fing > c.released_finger)
        retract = (self.ever_inserted & ~held).float() * torch.clamp((s["gb"][:, 2] - s["t"][:, 2] - 0.03) / 0.05, 0.0, 1.0)
        release_ok = (depth > c.release_depth) & (dxy < 0.003) & (up > 0.99) & (fing > c.released_finger)
        f_arm, f_fing, f_puck = self._force("arm_body_contact"), self._force("finger_body_contact"), self._force("puck_body_contact")
        zone = (dxy < c.zone_xy) & (clear < c.zone_clear)
        fing_c = f_fing > c.contact_force
        body = (f_arm > c.contact_force) | (fing_c & ~zone)
        fing_zone = fing_c & zone
        crash = (f_arm > c.crash_force) | ((f_fing > c.crash_force) & ~zone)
        jam = (f_fing > c.crash_force) & zone
        puck_body = (f_puck > c.contact_force) & ~((dxy < c.inside_xy) & (depth > -0.003))
        dropped = ~held & ~self.ever_inserted
        pre = held & ~self.ever_inserted
        rew = self.tlog.reward({  # 이름 = Episode_Reward/<이름>, 합은 이전 식과 같음
            "xy": pre.float() * c.rew_xy * xy,
            "depth": pre.float() * c.rew_depth * depth_r,
            "inserted": c.rew_inserted * s["inside"].float() * held.float(),
            "released": c.rew_released * released.float(),
            "release_ok": c.rew_release_ok * release_ok.float(),
            "stored": c.rew_stored * s["stored"].float(),
            "retract": c.rew_retract * retract,
            "puck_tilt": c.rew_puck_tilt * (1 - up) * held.float(),
            "body_contact": c.rew_body_contact * body.float(),
            "finger_zone": c.rew_finger_zone * fing_zone.float(),
            "body_crash": c.rew_body_crash * crash.float(),
            "jam": c.rew_jam * jam.float(),
            "puck_body": c.rew_puck_body * puck_body.float(),
            "drop": c.rew_drop * dropped.float(),
            "time": c.rew_time,  # 상수 (스텝마다)
            "action_rate": c.rew_action_rate * torch.sum(torch.square(self.actions - self.prev_actions), dim=1),
            "base_tilt": c.rew_base_tilt * self.base_tilt(),
        })
        self.hold = torch.where(s["stored"], self.hold + 1, torch.zeros_like(self.hold))
        self.succeeded |= self.hold >= int(c.success_hold_s / (c.sim.dt * c.decimation))
        self.contact_steps += body.float()
        self.best_xy = torch.minimum(self.best_xy, torch.where(held, dxy, self.best_xy))
        self.max_bottom = torch.maximum(self.max_bottom, depth)
        log = self.extras.setdefault("log", {})
        log["Metrics/xy_to_tube_mm"] = dxy.mean().item() * 1000
        log["Metrics/depth_mm"] = depth.mean().item() * 1000
        log["Metrics/held_ratio"] = held.float().mean().item()
        log["Metrics/inserted_ratio"] = s["inside"].float().mean().item()
        log["Metrics/released_ratio"] = released.float().mean().item()
        log["Metrics/stored_ratio"] = s["stored"].float().mean().item()
        log["Metrics/body_contact_ratio"] = body.float().mean().item()
        log["Metrics/finger_zone_contact_ratio"] = fing_zone.float().mean().item()
        log["Metrics/jam_ratio"] = jam.float().mean().item()
        return rew

    # ---- 초기화 ----
    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        if len(env_ids) > 0:
            log = self.extras.setdefault("log", {})
            log["Metrics/success_rate"] = self.succeeded[env_ids].float().mean().item()
            for b, nm in enumerate(("from_above", "from_chamfer", "from_inside")):
                m = self.start_bucket[env_ids] == b
                if m.any():
                    log[f"Metrics/success_{nm}"] = self.succeeded[env_ids][m].float().mean().item()
            log["Metrics/ever_inserted"] = self.ever_inserted[env_ids].float().mean().item()
            log["Metrics/contact_steps"] = self.contact_steps[env_ids].mean().item()
            log["Metrics/max_depth_mm"] = self.max_bottom[env_ids].mean().item() * 1000
            self.tlog.reset(env_ids)
        DirectRLEnv._reset_idx(self, env_ids)
        c, r, n, dev = self.cfg, self.robot, len(env_ids), self.device
        org = self.scene.env_origins[env_ids]
        root = r.data.default_root_pose.torch[env_ids].clone()
        root[:, :3] += org
        r.write_root_pose_to_sim_index(root_pose=root, env_ids=env_ids)
        r.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=env_ids)
        self.root0[env_ids] = root[:, :3]
        k = torch.multinomial(self._sw, n, replacement=True)
        q = r.data.default_joint_pos.torch[env_ids].clone()
        q[:, self._arm] = self._iq[k]
        q[:, self._gl[0]] = self._ifg[k, 0]
        q[:, self._gr[0]] = self._ifg[k, 1]
        r.write_joint_position_to_sim_index(position=q, env_ids=env_ids)
        r.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), env_ids=env_ids)
        self.arm_target[env_ids] = q[:, self._arm]
        self.tube[env_ids] = self._itube[k]
        pose = torch.cat((root[:, :3] + self._bl + self._ipk[k], self._ipq[k]), dim=1)
        self.puck.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
        self.puck.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=env_ids)
        self.calib[env_ids] = sample_uniform(-c.tube_calib_noise, c.tube_calib_noise, (n, 3), dev)
        dep = (self._ipk[k] - self._tubes[self._itube[k]])[:, 2] - PUCK_H / 2
        self.start_bucket[env_ids] = torch.where(-dep < -0.002, 0, torch.where(-dep < 0.006, 1, 2))
        self.actions[env_ids] = 0.0
        self.actions[env_ids, 3] = -1.0
        self.prev_actions[env_ids] = self.actions[env_ids]
        self.ever_inserted[env_ids] = False
        self.hold[env_ids] = 0
        self.succeeded[env_ids] = False
        self.contact_steps[env_ids] = 0.0
        self.best_xy[env_ids] = 1.0
        self.max_bottom[env_ids] = -1.0
