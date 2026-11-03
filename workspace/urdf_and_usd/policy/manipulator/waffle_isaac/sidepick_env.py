"""정렬·집기 `Waffle-SidePick-Direct` 구현. 설정과 설명은 sidepick_env_cfg.py."""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation, RigidObject
from isaaclab.envs import DirectRLEnv
from isaaclab.utils.math import quat_apply, quat_apply_inverse, quat_box_minus, quat_from_euler_xyz, quat_from_matrix, sample_uniform

from .dice2_cfg import BOX_H, BOX_SIZE, GRASP_OFFSET_A5, PUCK_Z
from .robot_cfg import ARM_JOINTS, EE_BODY, GRIPPER_JOINT, GRIPPER_MIMIC, WHEEL_REGEX, ee_axes
from .train_log import TrainLog

if TYPE_CHECKING:
    from .sidepick_env_cfg import WaffleSidePickEnvCfg


def wrap90(a):
    return a - (math.pi / 2) * torch.round(a / (math.pi / 2))


class WaffleSidePickEnv(DirectRLEnv):
    cfg: WaffleSidePickEnvCfg

    def __init__(self, cfg: WaffleSidePickEnvCfg, render_mode: str | None = None, **kwargs):
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
        self._off = torch.tensor([GRASP_OFFSET_A5], device=dev).repeat(n, 1)
        self._ex = torch.tensor([[1.0, 0.0, 0.0]], device=dev).repeat(n, 1)
        self._ey = torch.tensor([[0.0, 1.0, 0.0]], device=dev).repeat(n, 1)
        self._ez = torch.tensor([[0.0, 0.0, 1.0]], device=dev).repeat(n, 1)
        self.actions = torch.zeros(n, cfg.action_space, device=dev)  # 관절 공간 6 (팔 5 + 집게) 또는 직교 4 (잡는 점 xyz + 집게)
        self._cols = torch.tensor([getattr(r, "num_base_dofs", 6) + j for j in self._arm], device=dev)
        self.prev_actions = torch.zeros_like(self.actions)
        self.arm_target = r.data.default_joint_pos.torch[:, self._arm].clone()
        self.is_box = torch.zeros(n, dtype=torch.bool, device=dev)
        self.puck_z0 = torch.full((n,), PUCK_Z, device=dev)
        self.perc_bias = torch.zeros(n, 3, device=dev)
        self.nb_prev = torch.zeros(n, 2, device=dev)
        self.root0 = torch.zeros(n, 3, device=dev)
        self.hold = torch.zeros(n, dtype=torch.long, device=dev)
        self.succeeded = torch.zeros(n, dtype=torch.bool, device=dev)
        self.max_lift = torch.zeros(n, device=dev)
        self.arm_box_steps = torch.zeros(n, device=dev)
        self.tlog = TrainLog(self, ("tipped", "fell", "body_box", "base_move"))  # 보상 항목별 · 종료 원인 · 행동 포화 기록

    @property
    def robot(self) -> Articulation:
        return self.scene["robot"]

    def _obj(self, k) -> RigidObject:
        return self.scene[k]

    def puck_pos_w(self):
        return torch.where(self.is_box[:, None], self._obj("puckT_box").data.root_pos_w.torch, self._obj("puckT_cyl").data.root_pos_w.torch)

    def puck_quat_w(self):
        return torch.where(self.is_box[:, None], self._obj("puckT_box").data.root_quat_w.torch, self._obj("puckT_cyl").data.root_quat_w.torch)

    def to_base(self, p_w):
        d = self.robot.data
        return quat_apply_inverse(d.root_link_quat_w.torch, p_w - d.root_link_pos_w.torch)

    def vec_base(self, v_w):
        return quat_apply_inverse(self.robot.data.root_link_quat_w.torch, v_w)

    def grasp_w(self):
        d = self.robot.data
        return d.body_link_pos_w.torch[:, self._a5] + quat_apply(d.body_link_quat_w.torch[:, self._a5], self._off)

    def a5_axes_w(self):
        """(닫는 축, 아래쪽, −집게 방향) = 이전 팔 arm_link_5 의 (x, y, z) 축 뜻 (robot_cfg.ee_axes)."""
        return ee_axes(quat_apply, self.robot.data.body_link_quat_w.torch[:, self._a5], self._ex, self._ey, self._ez)

    def grasp_target_w(self):
        p = self.puck_pos_w().clone()
        p[:, 2] += self.cfg.grasp_dz
        return p

    def face_delta(self):
        px = quat_apply(self.puck_quat_w(), self._ex)
        ax, _, _ = self.a5_axes_w()
        return wrap90(torch.atan2(px[:, 1], px[:, 0]) - torch.atan2(ax[:, 1], ax[:, 0]))

    def lift(self):
        return self.puck_pos_w()[:, 2] - self.puck_z0

    def base_tilt(self):
        g = self.robot.data.projected_gravity_b.torch
        return torch.acos((-g[:, 2]).clamp(-1.0, 1.0))

    def puck_up(self):
        return quat_apply(self.puck_quat_w(), self._ez)[:, 2].abs()

    def _contact(self, k):
        f = self.scene[k].data.normal_force_matrix_w.torch  # (N, 몸, 상대, 3)
        return torch.linalg.norm(f, dim=-1).amax(dim=(1, 2)) > self.cfg.contact_force

    # ---- 행동 ----
    def _pre_physics_step(self, actions):
        self.prev_actions = self.actions.clone()
        self.actions = actions.clamp(-1.0, 1.0)
        q = self.robot.data.joint_pos.torch[:, self._arm]
        lead = self.cfg.target_lead_max
        if self.cfg.cartesian:
            dq = self._ik_step(self.actions[:, :3] * self.cfg.cart_scale)
        else:
            dq = self.cfg.action_scale * self.actions[:, :5]
        t = torch.clamp(self.arm_target + dq, q - lead, q + lead)
        self.tlog.actions(actions, self.arm_target + dq - q, lead, t, self._lo, self._hi)
        self.arm_target = torch.clamp(t, self._lo, self._hi)

    def _ik_step(self, dp_b):
        """직교 행동 (261028 N7): 잡는 점 변위 (base 좌표) + 집게 자세 = 수평·joint1 축에서 바깥 방향·카메라 위 -> 감쇠 최소제곱 IK (넣기 전용 환경과 같은 방식)."""
        c, d = self.cfg, self.robot.data
        rq = d.root_link_quat_w.torch
        J = d.body_link_jacobian_w.torch[:, self._a5][:, :, self._cols]
        q5 = d.body_link_quat_w.torch[:, self._a5]
        rr = quat_apply(q5, self._off)
        Jl = J[:, 0:3] - torch.cross(rr.unsqueeze(-1).expand(-1, -1, 5), J[:, 3:6], dim=1)
        w = c.rot_hold_weight
        Jf = torch.cat((Jl, J[:, 3:6] * w), dim=1)
        j1 = d.root_link_pos_w.torch + quat_apply(rq, torch.tensor([[c.joint1_x, 0.0, 0.0]], device=self.device).expand(self.num_envs, 3))
        up = quat_apply(rq, self._ez)
        a = self.grasp_w() - j1
        a = a - (a * up).sum(1, keepdim=True) * up
        a = a / a.norm(dim=1, keepdim=True).clamp(min=1e-6)
        R = torch.stack((a, torch.cross(up, a, dim=1), up), dim=2)
        err = torch.cat((quat_apply(rq, dp_b), quat_box_minus(quat_from_matrix(R), q5) * w), dim=1)
        JJt = Jf @ Jf.transpose(1, 2) + c.ik_damping * torch.eye(6, device=self.device)
        return (Jf.transpose(1, 2) @ torch.linalg.solve(JJt, err.unsqueeze(-1))).squeeze(-1)

    def _apply_action(self):
        cmd = self.robot.actuators.target_command
        g = torch.where(self.actions[:, -1:] > 0.0, self.cfg.gripper_open, self.cfg.gripper_closed)
        cmd.set_position_index(value=self.arm_target.contiguous(), joint_ids=self._arm)
        cmd.set_position_index(value=g.expand(-1, 2).contiguous(), joint_ids=self._gl + self._gr)  # 두 손가락 같은 값
        cmd.set_velocity_index(value=torch.zeros(self.num_envs, len(self._wheels), device=self.device), joint_ids=self._wheels)

    # ---- 관측·보상·종료 ----
    def _get_observations(self):
        d = self.robot.data
        perc = self.puck_pos_w() + self.perc_bias + (torch.rand(self.num_envs, 3, device=self.device) * 2 - 1) * self.cfg.perc_jitter
        pb = self.to_base(perc)
        gb = self.to_base(self.grasp_w())
        ax, _, az = self.a5_axes_w()
        nb = self.to_base(self._obj("puckN").data.root_pos_w.torch)[:, :2] - pb[:, :2]
        obs = torch.cat((
            d.joint_pos.torch[:, self._arm] - d.default_joint_pos.torch[:, self._arm], d.joint_vel.torch[:, self._arm],
            d.joint_pos.torch[:, self._gl], pb, gb, pb - gb, self.vec_base(-az), self.vec_base(ax), nb,
            self.is_box.float().unsqueeze(1), self.actions,
        ), dim=-1)
        return {"policy": obs}

    def _get_rewards(self):
        c = self.cfg
        g = self.grasp_w()
        to_t = self.grasp_target_w() - g
        dist = torch.linalg.norm(to_t, dim=1)
        lift = self.lift()
        near = dist < c.near_dist
        close = (dist < 0.12).float()
        ax, ay, az = self.a5_axes_w()
        appr = -az
        horiz = 1 - appr[:, 2].abs()
        cam_up = (-ay[:, 2]).clamp(min=0.0)
        dir_h = to_t.clone()
        dir_h[:, 2] = 0
        dir_h = dir_h / dir_h.norm(dim=1, keepdim=True).clamp(min=1e-6)
        point = (appr * dir_h).sum(dim=1).clamp(min=0.0) * ((dist > 0.02) & (dist < 0.12)).float()
        align = self.is_box.float() * (1 + torch.cos(4 * self.face_delta())) / 2 * close
        up = self.puck_up()
        tilt = self.base_tilt()
        fing = self.robot.data.joint_pos.torch[:, self._gl[0]]
        grasped = (dist < c.grasp_dist) & (fing > c.grasp_finger_range[0]) & (fing < c.grasp_finger_range[1]) & (tilt < c.grasp_tilt_max) & (up > 0.9)
        lifted = (lift > c.lift_min) & near & (up > c.upright_min)
        height = (torch.clamp(lift, 0.0, c.lift_goal) - torch.clamp(lift - c.lift_goal, min=0.0)) / c.lift_goal * near.float()
        xy = self._obj("puckN").data.root_pos_w.torch[:, :2]
        moved = torch.linalg.norm(xy - self.nb_prev, dim=1)
        self.nb_prev = xy.clone()
        toppled = (up < c.topple_cos) & (lift < c.lift_min)
        arm_box = self._contact("arm_box_contact")
        body_box = self._contact("body_box_contact")
        base_move = torch.linalg.norm(self.robot.data.root_link_pos_w.torch[:, :2] - self.root0[:, :2], dim=1)
        rew = self.tlog.reward({  # 이름 = Episode_Reward/<이름>, 합은 이전 한 줄 식과 같음
            "reach": c.rew_reach * (1 - torch.tanh(dist / 0.1)),
            "reach_fine": c.rew_reach_fine * (1 - torch.tanh(dist / 0.02)),
            "horizontal": close * c.rew_horizontal * horiz,
            "camera_up": close * c.rew_camera_up * cam_up,
            "point": c.rew_point * point,
            "align": c.rew_align * align,
            "grasp": c.rew_grasp * grasped.float(),
            "lifted": c.rew_lifted * lifted.float(),
            "lift_height": c.rew_lift_height * height,
            "puck_tilt": c.rew_puck_tilt * (1 - up) * close,
            "arm_box": c.rew_arm_box * arm_box.float(),
            "body_box": c.rew_body_box * body_box.float(),
            "base_move": c.rew_base_move * (base_move > c.base_move_max).float(),
            "action_rate": c.rew_action_rate * torch.sum(torch.square(self.actions - self.prev_actions), dim=1),
            "joint_vel": c.rew_joint_vel * torch.sum(torch.square(self.robot.data.joint_vel.torch[:, self._arm]), dim=1),
            "neighbor_moved": c.rew_neighbor_moved * moved,
            "toppled": c.rew_toppled * toppled.float(),
            "base_tilt": c.rew_base_tilt * tilt,
            "fall": c.rew_fall * self._fell().float(),
        })
        held = (lift > c.success_lift) & near & (up > c.upright_min)
        self.hold = torch.where(held, self.hold + 1, torch.zeros_like(self.hold))
        self.succeeded |= self.hold >= int(c.success_hold_s / (c.sim.dt * c.decimation))
        self.max_lift = torch.maximum(self.max_lift, lift * near.float())
        self.arm_box_steps += arm_box.float()
        log = self.extras.setdefault("log", {})
        log["Metrics/grasp_distance"] = dist.mean().item()
        log["Metrics/horizontal"] = horiz.mean().item()
        log["Metrics/grasped_ratio"] = grasped.float().mean().item()
        log["Metrics/lifted_ratio"] = lifted.float().mean().item()
        log["Metrics/toppled_ratio"] = toppled.float().mean().item()
        log["Metrics/arm_box_contact_ratio"] = arm_box.float().mean().item()
        log["Metrics/body_box_contact_ratio"] = body_box.float().mean().item()
        return rew

    def _fell(self):
        return (self.puck_pos_w()[:, 2] < self.puck_z0 - self.cfg.fall_drop) & (self.lift() < self.cfg.lift_min)

    def _get_dones(self):
        tipped = self.robot.data.projected_gravity_b.torch[:, 2] > -0.6
        body_box = self._contact("body_box_contact")
        base_move = torch.linalg.norm(self.robot.data.root_link_pos_w.torch[:, :2] - self.root0[:, :2], dim=1) > self.cfg.base_move_max
        fell = self._fell()
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        self.tlog.dones(tipped=tipped, fell=fell, body_box=body_box, base_move=base_move, time_out=time_out)
        return tipped | fell | body_box | base_move, time_out

    # ---- 초기화 ----
    def _place(self, k, env_ids, xy, yaw, z):
        o = self._obj(k)
        pose = o.data.default_root_pose.torch[env_ids].clone()
        pose[:, :2] = xy + self.scene.env_origins[env_ids, :2]
        pose[:, 2] = z + self.scene.env_origins[env_ids, 2]
        zero = torch.zeros_like(yaw)
        pose[:, 3:7] = quat_from_euler_xyz(zero, zero, yaw)
        o.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
        o.write_root_velocity_to_sim_index(root_velocity=torch.zeros(len(env_ids), 6, device=self.device), env_ids=env_ids)

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        if len(env_ids) > 0:
            log = self.extras.setdefault("log", {})
            ok, bx = self.succeeded[env_ids].float(), self.is_box[env_ids]
            log["Metrics/success_rate"] = ok.mean().item()
            if bx.any():
                log["Metrics/success_box"] = ok[bx].mean().item()
            if (~bx).any():
                log["Metrics/success_cyl"] = ok[~bx].mean().item()
            log["Metrics/max_lift"] = self.max_lift[env_ids].mean().item()
            log["Metrics/arm_box_steps"] = self.arm_box_steps[env_ids].mean().item()
            self.tlog.reset(env_ids)
        super()._reset_idx(env_ids)
        c, r, n, dev = self.cfg, self.robot, len(env_ids), self.device
        org = self.scene.env_origins[env_ids]

        def U(lo, hi, shape):
            return sample_uniform(lo, hi, shape, dev)

        root = r.data.default_root_pose.torch[env_ids].clone()
        root[:, :3] += org
        r.write_root_pose_to_sim_index(root_pose=root, env_ids=env_ids)
        r.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=env_ids)
        self.root0[env_ids] = root[:, :3]
        q = r.data.default_joint_pos.torch[env_ids].clone()
        near = torch.rand(n, device=dev) < c.near_start_prob  # 역방향 커리큘럼: 퍽을 벌린 손가락 사이 (시작 자세 앞 0 ~ 3.5 cm) 에 둠
        nz = torch.where(near, c.near_noise, c.ready_noise).unsqueeze(1)
        q[:, self._arm] = torch.clamp(q[:, self._arm] + U(-1.0, 1.0, (n, 5)) * nz, self._lo, self._hi)
        r.write_joint_position_to_sim_index(position=q, env_ids=env_ids)
        r.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), env_ids=env_ids)
        self.arm_target[env_ids] = q[:, self._arm]

        # 상자: 로봇 앞 끝에서 gap, 가로 이동, 요 (가장 가까운 면·모서리 기준으로 gap)
        yaw = U(0.0, math.pi, (n,))
        half = BOX_SIZE / 2
        half_proj = half * (torch.cos(yaw).abs() + torch.sin(yaw).abs())
        bc = torch.stack((c.robot_front + U(*c.gap_range, (n,)) + half_proj, U(*c.lat_range, (n,))), dim=1)
        self._place("boxA", env_ids, bc, yaw, torch.full((n,), BOX_H / 2, device=dev))
        off = torch.stack((torch.cos(yaw), torch.sin(yaw)), dim=1) * 0.04
        p1, p2 = bc - off, bc + off
        j1 = torch.tensor([c.joint1_x, 0.0], device=dev)
        d1 = (torch.linalg.norm(p1 - j1, dim=1) - c.sweet_radial).abs()
        d2 = (torch.linalg.norm(p2 - j1, dim=1) - c.sweet_radial).abs()
        pick1 = (d1 <= d2)[:, None]
        if near.any():  # 집을 퍽 = 시작 자세의 잡는 점 바로 앞, 상자는 그 아래로 (p1 = 집을 퍽)
            want = torch.stack((c.near_grasp_x + U(0.0, c.near_dx_max, (n,)), U(-c.near_dy_max, c.near_dy_max, (n,))), dim=1)
            bc = torch.where(near[:, None], want + off, bc)
            self._place("boxA", env_ids, bc, yaw, torch.full((n,), BOX_H / 2, device=dev))
            p1, p2 = bc - off, bc + off
            pick1 = pick1 | near[:, None]
        tgt, nbr = torch.where(pick1, p1, p2), torch.where(pick1, p2, p1)
        is_box = torch.rand(n, device=dev) < c.box_shape_prob
        self.is_box[env_ids] = is_box
        park = torch.tensor([0.0, 0.7], device=dev).expand(n, 2)
        z = torch.full((n,), PUCK_Z, device=dev)
        self._place("puckT_box", env_ids, torch.where(is_box[:, None], tgt, park), yaw, z)
        self._place("puckT_cyl", env_ids, torch.where(is_box[:, None], park + torch.tensor([0.1, 0.0], device=dev), tgt), yaw, z)
        self._place("puckN", env_ids, nbr, yaw, z)
        self.nb_prev[env_ids] = nbr + org[:, :2]
        self.puck_z0[env_ids] = PUCK_Z + org[:, 2]
        self.perc_bias[env_ids] = U(-c.perc_bias, c.perc_bias, (n, 3))
        self.actions[env_ids] = 0.0
        self.actions[env_ids, -1] = 1.0
        self.prev_actions[env_ids] = self.actions[env_ids]
        self.hold[env_ids] = 0
        self.succeeded[env_ids] = False
        self.max_lift[env_ids] = 0.0
        self.arm_box_steps[env_ids] = 0.0
