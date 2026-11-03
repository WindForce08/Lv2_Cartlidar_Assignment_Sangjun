"""스토리지 넣기 `Waffle-Store-Direct` 구현. 설정과 설명은 store_env_cfg.py."""

from __future__ import annotations

import json
import math
import os
from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation, RigidObject
from isaaclab.envs import DirectRLEnv
from isaaclab.utils.math import quat_apply, quat_apply_inverse

from .dice2_cfg import GRASP_OFFSET_A5, PUCK_H
from .robot_cfg import ARM_JOINTS, EE_BODY, FINGER_HOLD_30, GRIPPER_JOINT, GRIPPER_MIMIC, ROOT_TO_BASE_LINK, WHEEL_REGEX, ee_axes
from .train_log import TrainLog

if TYPE_CHECKING:
    from .store_env_cfg import WaffleStoreEnvCfg

STARTS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "store_starts.json")


class WaffleStoreEnv(DirectRLEnv):
    cfg: WaffleStoreEnvCfg

    def __init__(self, cfg: WaffleStoreEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        r = self.robot
        self._arm, _ = r.find_joints(ARM_JOINTS, preserve_order=True)
        self._gl, _ = r.find_joints([GRIPPER_JOINT])
        self._gr, _ = r.find_joints([GRIPPER_MIMIC])
        self._wheels, _ = r.find_joints([WHEEL_REGEX])
        self._a5 = r.find_bodies(EE_BODY)[0][0]
        lim = r.data.soft_joint_pos_limits.torch[0, self._arm].clone()
        self._lo, self._hi = lim[:, 0], lim[:, 1]
        n, dev = self.num_envs, self.device
        S = json.load(open(STARTS, encoding="utf-8"))
        assert S["arm_joints"] == list(ARM_JOINTS)
        allr = list(S["samples"])
        hand = os.path.join(os.path.dirname(STARTS), "store_handover.json")  # 집기 모델이 실제로 끝난 상태 (chain_play --collect_handover)
        if os.path.exists(hand):
            allr += json.load(open(hand, encoding="utf-8"))["samples"]
        smp = [s for s in allr if s["cat"] in cfg.start_cats and (s["tube"] < 0 or s["tube"] in cfg.tubes_used)]
        self._sq = torch.tensor([s["q"] for s in smp], device=dev)
        self._spk = torch.tensor([s["puck_b"] for s in smp], device=dev)
        self._spq = torch.tensor([s["puck_quat_b"] for s in smp], device=dev)
        self._stube = torch.tensor([s["tube"] for s in smp], device=dev)
        cats = [s["cat"] for s in smp]
        cw = dict(zip(cfg.start_cats, cfg.cat_weight))
        cw = {k: v for k, v in cw.items() if k in cats}  # 없는 종류 (예: H 파일 없음) 는 빼고 나머지 비율로
        w = torch.tensor([cw[c] / cats.count(c) for c in cats], device=dev)
        print(f"[Store] 시작 상태 { {k: cats.count(k) for k in cw} } 비율 {cw}", flush=True)
        self._sw = w / w.sum()
        self._tubes = torch.tensor([[t["axis"][0], t["axis"][1], t["rim_z"]] for t in S["tubes"]], device=dev)  # base_link 좌표
        self._bl = torch.tensor([[0.0, 0.0, ROOT_TO_BASE_LINK]], device=dev)
        self._off = torch.tensor([GRASP_OFFSET_A5], device=dev).repeat(n, 1)
        self._ex = torch.tensor([[1.0, 0.0, 0.0]], device=dev).repeat(n, 1)
        self._ey = torch.tensor([[0.0, 1.0, 0.0]], device=dev).repeat(n, 1)
        self._ez = torch.tensor([[0.0, 0.0, 1.0]], device=dev).repeat(n, 1)
        self.actions = torch.zeros(n, 6, device=dev)
        self.prev_actions = torch.zeros_like(self.actions)
        self.arm_target = r.data.default_joint_pos.torch[:, self._arm].clone()
        self.tube = torch.zeros(n, dtype=torch.long, device=dev)
        self.root0 = torch.zeros(n, 3, device=dev)
        self.ever_inserted = torch.zeros(n, dtype=torch.bool, device=dev)
        self.hold = torch.zeros(n, dtype=torch.long, device=dev)
        self.succeeded = torch.zeros(n, dtype=torch.bool, device=dev)
        self.contact_steps = torch.zeros(n, device=dev)
        self.best_xy = torch.ones(n, device=dev)
        self.max_bottom = torch.zeros(n, device=dev)
        self.tlog = TrainLog(self, ("tipped", "dropped", "crash", "base_move"))  # 보상 항목별 · 종료 원인 · 행동 포화 기록

    @property
    def robot(self) -> Articulation:
        return self.scene["robot"]

    @property
    def puck(self) -> RigidObject:
        return self.scene["puck"]

    # ---- 좌표 (base_link 좌표 = 루트 + ROOT_TO_BASE_LINK, 261028 부터 루트 = base_link) ----
    def to_bl(self, p_w):
        d = self.robot.data
        return quat_apply_inverse(d.root_link_quat_w.torch, p_w - d.root_link_pos_w.torch) - self._bl

    def vec_bl(self, v_w):
        return quat_apply_inverse(self.robot.data.root_link_quat_w.torch, v_w)

    def grasp_w(self):
        d = self.robot.data
        return d.body_link_pos_w.torch[:, self._a5] + quat_apply(d.body_link_quat_w.torch[:, self._a5], self._off)

    def a5_axes_w(self):
        """(닫는 축, −집게 방향) = 이전 arm_link_5 의 (x, z) 축 뜻 (robot_cfg.ee_axes)."""
        close, _, neg_appr = ee_axes(quat_apply, self.robot.data.body_link_quat_w.torch[:, self._a5], self._ex, self._ey, self._ez)
        return close, neg_appr

    def puck_axis_bl(self):
        """퍽 축 (위쪽 부호)."""
        u = self.vec_bl(quat_apply(self.puck.data.root_quat_w.torch, self._ez))
        return u * torch.where(u[:, 2:3] < 0, -1.0, 1.0)

    def _yaw_err(self):
        """관절 1 이 목표 통 방향 각 (joint1 축에서 통 축으로, 새 팔은 joint1 각 = 팔 방향) 에서 얼마나 먼지 [rad].
        joint1 한계가 ±π 라 P 통 (+2.80) 은 + 쪽으로, N 통 (−2.80) 은 − 쪽으로 돌아야 닿음 (반대로 돌면 ±π 한계에 막힘)."""
        t = self._tubes[self.tube]
        th = torch.atan2(t[:, 1], t[:, 0] - self.cfg.joint1_x)
        return (self.robot.data.joint_pos.torch[:, self._arm[0]] - th).abs()

    def base_tilt(self):
        g = self.robot.data.projected_gravity_b.torch
        return torch.acos((-g[:, 2]).clamp(-1.0, 1.0))

    def _force(self, k):
        f = self.scene[k].data.normal_force_matrix_w.torch  # (N, 몸, 상대, 3)
        return torch.linalg.norm(f, dim=-1).amax(dim=(1, 2))

    def _state(self):
        c = self.cfg
        pc = self.to_bl(self.puck.data.root_pos_w.torch)
        u = self.puck_axis_bl()
        pb = pc - u * (PUCK_H / 2)  # 퍽 바닥 중심
        t = self._tubes[self.tube]
        dxy = torch.linalg.norm(pb[:, :2] - t[:, :2], dim=1)
        depth = t[:, 2] - pb[:, 2]  # + = 입구 아래
        gb = self.to_bl(self.grasp_w())
        held = torch.linalg.norm(pc - gb, dim=1) < c.held_dist
        fing = self.robot.data.joint_pos.torch[:, self._gl[0]]
        inside = (dxy < c.inside_xy) & (depth > c.inserted_depth)
        stored = (dxy < c.inside_xy) & (depth > c.stored_depth) & (u[:, 2] > 0.95) & (fing > c.released_finger) & ~held
        return dict(pc=pc, u=u, pb=pb, t=t, dxy=dxy, depth=depth, gb=gb, held=held, fing=fing, inside=inside, stored=stored)

    # ---- 행동 ----
    def _pre_physics_step(self, actions):
        self.prev_actions = self.actions.clone()
        self.actions = actions.clamp(-1.0, 1.0)
        q = self.robot.data.joint_pos.torch[:, self._arm]
        lead = self.cfg.target_lead_max
        s = self._state()
        fine = (s["dxy"] < self.cfg.fine_xy) & (-s["depth"] < self.cfg.zone_clear)
        scale = self.cfg.action_scale * torch.where(fine, self.cfg.fine_scale, 1.0).unsqueeze(1)
        t = torch.clamp(self.arm_target + scale * self.actions[:, :5], q - lead, q + lead)
        self.tlog.actions(actions, self.arm_target + scale * self.actions[:, :5] - q, lead, t, self._lo, self._hi)
        self.arm_target = torch.clamp(t, self._lo, self._hi)
        if self.cfg.lock_roll:  # 손목 롤 (관절 5) 0 고정: 집게 닫는 축 수평 = 퍽 세움 (261028 ST-N5)
            self.arm_target[:, 4] = 0.0

    def _apply_action(self):
        cmd = self.robot.actuators.target_command
        g = torch.where(self.actions[:, 5:6] > 0.0, self.cfg.gripper_open, self.cfg.gripper_closed)
        cmd.set_position_index(value=self.arm_target.contiguous(), joint_ids=self._arm)
        cmd.set_position_index(value=g.expand(-1, 2).contiguous(), joint_ids=self._gl + self._gr)  # 두 손가락 같은 값
        cmd.set_velocity_index(value=torch.zeros(self.num_envs, len(self._wheels), device=self.device), joint_ids=self._wheels)

    # ---- 관측·보상·종료 ----
    def _get_observations(self):
        d = self.robot.data
        s = self._state()
        ax, az = self.a5_axes_w()
        obs = torch.cat((
            d.joint_pos.torch[:, self._arm] - d.default_joint_pos.torch[:, self._arm], d.joint_vel.torch[:, self._arm],
            d.joint_pos.torch[:, self._gl], s["gb"], s["pc"] - s["gb"], s["u"], s["t"], s["pb"] - s["t"],
            self.vec_bl(-az), self.vec_bl(ax), torch.nn.functional.one_hot(self.tube, 3).float(), self.actions,
        ), dim=-1)
        return {"policy": obs}

    def _get_rewards(self):
        c = self.cfg
        s = self._state()
        dxy, depth, held, u, fing = s["dxy"], s["depth"], s["held"], s["u"], s["fing"]
        up = u[:, 2]
        self.ever_inserted |= s["inside"]
        pre = held & ~self.ever_inserted  # 넣기 전 쥐고 옮기는 중
        near = dxy < c.inside_xy
        # 높이 들기: 퍽 바닥이 (입구 − 30 cm) -> 입구 + 12 mm, 축에 맞으면 다 받은 것으로 (내려 넣을 때 깎이지 않게)
        clear = -depth  # 퍽 바닥 − 입구
        lift = torch.where(near, torch.ones_like(dxy), torch.clamp((clear + 0.30) / (0.30 + c.clear_margin), 0.0, 1.0))
        # 다가가기: 높이 들었을 때만 (입구 아래 10 cm 부터 켜짐)
        high = torch.where(near, torch.ones_like(dxy), torch.clamp((clear + c.high_band) / c.high_band, 0.0, 1.0))
        approach = high * (c.rew_approach * (1 - torch.tanh(dxy / 0.15)) + c.rew_approach_mid * (1 - torch.tanh(dxy / 0.03))
                           + c.rew_approach_fine * (1 - torch.tanh(dxy / 0.008)) + c.rew_precise * (1 - torch.tanh(dxy / 0.002)))
        # 통 5 cm 안: 퍽 바닥을 입구 + 8 mm 로 (넣기 직전 높이), 축에 맞으면 다 받은 것으로
        hover = torch.where(near, torch.ones_like(dxy), (1 - torch.tanh((clear - c.hover_clear).abs() / 0.01)) * (dxy < 0.05).float())
        aligned = ((dxy < c.align_xy) & (up > c.upright_min)).float()
        insert = (1 - torch.tanh(dxy / 0.003)) * (up > c.upright_min).float() * torch.clamp((depth + 0.01) / 0.04, 0.0, 1.0)
        released = s["inside"] & (fing > c.released_finger)
        retract = (self.ever_inserted & ~held).float() * torch.clamp(torch.linalg.norm(s["gb"] - s["t"], dim=1) / 0.06, max=1.0)
        f_arm = self._force("arm_body_contact")
        f_fing = self._force("finger_body_contact")
        f_puck = self._force("puck_body_contact")
        zone = (dxy < c.zone_xy) & (clear < c.zone_clear)  # 넣는 자리: 손가락이 통 위에 닿는 건 약하게
        fing_c = f_fing > c.contact_force
        body = (f_arm > c.contact_force) | (fing_c & ~zone)
        fing_zone = fing_c & zone
        crash = (f_arm > c.crash_force) | ((f_fing > c.crash_force) & ~zone)
        puck_body = (f_puck > c.contact_force) & ~(near & (depth > -0.003))
        dropped = ~held & ~self.ever_inserted
        base_move = torch.linalg.norm(self.robot.data.root_link_pos_w.torch[:, :2] - self.root0[:, :2], dim=1) > c.base_move_max
        p = pre.float()
        rew = self.tlog.reward({  # 이름 = Episode_Reward/<이름>, 합은 이전 식과 같음 (pre * (...) 를 항목별로 나눔)
            "hold": p * c.rew_hold,
            "lift": p * c.rew_lift * lift,
            "approach": p * approach,
            "hover": p * c.rew_hover * hover,
            "upright": p * c.rew_upright * (1 - torch.tanh((1 - up) / c.upright_scale)),
            "level": p * c.rew_level * (1 - torch.tanh(self.vec_bl(-self.a5_axes_w()[1])[:, 2].abs() / 0.03)),
            "yaw": p * c.rew_yaw * (1 - self._yaw_err() / math.pi),
            "insert": c.rew_insert * insert * held.float(),
            "inserted": c.rew_inserted * s["inside"].float(),
            "released": c.rew_released * released.float(),
            "stored": c.rew_stored * s["stored"].float(),
            "retract": c.rew_retract * retract,
            "puck_tilt": c.rew_puck_tilt * (1 - up) * held.float(),
            "body_contact": c.rew_body_contact * body.float(),
            "finger_zone": c.rew_finger_zone * fing_zone.float(),
            "body_crash": c.rew_body_crash * crash.float(),
            "puck_body": c.rew_puck_body * puck_body.float(),
            "drop": c.rew_drop * dropped.float(),
            "base_move": c.rew_base_move * base_move.float(),
            "action_rate": c.rew_action_rate * torch.sum(torch.square(self.actions - self.prev_actions), dim=1),
            "joint_vel": c.rew_joint_vel * torch.sum(torch.square(self.robot.data.joint_vel.torch[:, self._arm]), dim=1),
            "base_tilt": c.rew_base_tilt * self.base_tilt(),
        })
        self.hold = torch.where(s["stored"], self.hold + 1, torch.zeros_like(self.hold))
        self.succeeded |= self.hold >= int(c.success_hold_s / (c.sim.dt * c.decimation))
        self.contact_steps += body.float()
        self.best_xy = torch.minimum(self.best_xy, torch.where(held, dxy, self.best_xy))
        self.max_bottom = torch.maximum(self.max_bottom, torch.where(held, clear, self.max_bottom))
        log = self.extras.setdefault("log", {})
        log["Metrics/xy_to_tube_mm"] = dxy.mean().item() * 1000
        log["Metrics/bottom_above_rim_mm"] = clear.mean().item() * 1000
        log["Metrics/held_ratio"] = held.float().mean().item()
        log["Metrics/aligned_ratio"] = aligned.mean().item()
        log["Metrics/inserted_ratio"] = s["inside"].float().mean().item()
        log["Metrics/released_ratio"] = released.float().mean().item()
        log["Metrics/stored_ratio"] = s["stored"].float().mean().item()
        log["Metrics/body_contact_ratio"] = body.float().mean().item()
        log["Metrics/puck_body_contact_ratio"] = puck_body.float().mean().item()
        log["Metrics/finger_zone_contact_ratio"] = fing_zone.float().mean().item()
        log["Metrics/puck_upright"] = up.mean().item()
        return rew

    def _get_dones(self):
        s = self._state()
        tipped = self.robot.data.projected_gravity_b.torch[:, 2] > -0.6
        dropped = ~s["held"] & ~self.ever_inserted & ~s["inside"]
        zone = (s["dxy"] < self.cfg.zone_xy) & (-s["depth"] < self.cfg.zone_clear)
        crash = (self._force("arm_body_contact") > self.cfg.crash_force) | ((self._force("finger_body_contact") > self.cfg.crash_force) & ~zone)
        base_move = torch.linalg.norm(self.robot.data.root_link_pos_w.torch[:, :2] - self.root0[:, :2], dim=1) > self.cfg.base_move_max
        z = torch.zeros_like(self.tube)
        self.term_reason = torch.where(dropped, 1, torch.where(crash, 2, torch.where(base_move, 3, torch.where(tipped, 4, z))))  # 진단용
        self.term_arm_f = self._force("arm_body_contact")
        self.term_fing_f = self._force("finger_body_contact")
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        self.tlog.dones(tipped=tipped, dropped=dropped, crash=crash, base_move=base_move, time_out=time_out)
        return tipped | dropped | crash | base_move, time_out

    # ---- 초기화 ----
    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        if len(env_ids) > 0:
            log = self.extras.setdefault("log", {})
            log["Metrics/success_rate"] = self.succeeded[env_ids].float().mean().item()
            for i, nm in enumerate("CPN"):
                m = self.tube[env_ids] == i
                if m.any():
                    log[f"Metrics/success_{nm}"] = self.succeeded[env_ids][m].float().mean().item()
            log["Metrics/ever_inserted"] = self.ever_inserted[env_ids].float().mean().item()
            log["Metrics/contact_steps"] = self.contact_steps[env_ids].mean().item()
            log["Metrics/best_xy_mm"] = self.best_xy[env_ids].mean().item() * 1000
            log["Metrics/max_bottom_above_rim_mm"] = self.max_bottom[env_ids].mean().item() * 1000
            self.tlog.reset(env_ids)
        super()._reset_idx(env_ids)
        c, r, n, dev = self.cfg, self.robot, len(env_ids), self.device
        org = self.scene.env_origins[env_ids]
        root = r.data.default_root_pose.torch[env_ids].clone()
        root[:, :3] += org
        r.write_root_pose_to_sim_index(root_pose=root, env_ids=env_ids)
        r.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=env_ids)
        self.root0[env_ids] = root[:, :3]
        k = torch.multinomial(self._sw, n, replacement=True)
        q = r.data.default_joint_pos.torch[env_ids].clone()
        q[:, self._arm] = torch.clamp(self._sq[k], self._lo, self._hi)
        q[:, self._gl[0]] = FINGER_HOLD_30
        q[:, self._gr[0]] = FINGER_HOLD_30
        r.write_joint_position_to_sim_index(position=q, env_ids=env_ids)
        r.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), env_ids=env_ids)
        self.arm_target[env_ids] = q[:, self._arm]
        st = self._stube[k]
        used = torch.tensor(c.tubes_used, device=dev)
        self.tube[env_ids] = torch.where(st < 0, used[torch.randint(0, len(c.tubes_used), (n,), device=dev)], st)
        # 쥔 퍽 (루트 요 0 -> base_link 좌표 = 월드 방향)
        pose = torch.cat((root[:, :3] + self._bl + self._spk[k], self._spq[k]), dim=1)
        self.puck.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
        self.puck.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=env_ids)
        self.actions[env_ids] = 0.0
        self.actions[env_ids, 5] = -1.0  # 쥔 채로 시작
        self.prev_actions[env_ids] = self.actions[env_ids]
        self.ever_inserted[env_ids] = False
        self.hold[env_ids] = 0
        self.succeeded[env_ids] = False
        self.contact_steps[env_ids] = 0.0
        self.best_xy[env_ids] = 1.0
        self.max_bottom[env_ids] = -1.0
