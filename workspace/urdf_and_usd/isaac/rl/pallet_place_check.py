"""B안 팔레트 놓기 고정 궤적 확인 (Isaac PhysX, 학습 없음): pallet_place_traj.json 을 그대로 따라 쥔 퍽을 칸에 놓아 봄.

대마다 칸 하나 x 퍽 모양 하나 (칸 12 x 원기둥 · 상자 = 24 대, 다른 칸은 비어 있음).
  1. 앞 운반 자세 (q_front) 에서 집게를 벌리고 퍽을 손가락 사이 (TCP = 퍽 중심 + 15 mm, 집게 아래) 에 둔 뒤 쥠
  2. 궤적 (rotate -> carry -> place) 을 관절 속도 한도 (--speed) 로 따라감, 놓는 높이에서 잠깐 멈춤
  3. 집게 벌림 (FINGER_RELEASED, 37.6 mm = 팔레트 설계 grip_open 38) -> retract -> return -> rotate_back -> 1 s 기다림
  4. 결과: 퍽 바닥 가운데 - 칸 축 (수평), 퍽 바닥 - 칸 바닥 (높이), 기울기, 상자 퍽 면 방향, 옮기는 중 퍽이 손가락에서 벗어난 최대 거리,
     팔 링크 · 손가락 · 퍽이 몸체 (base_link = 차체 · 팔레트 · 기둥) 에 닿은 최대 힘 (단계별), 처음 닿은 구간 · 자세
  처짐 보정 (--ki): 팀 팔 구동은 펌웨어 위치 P 800 · I 0 (강성 18.88 N·m/rad) 이라 퍽을 쥔 팔이 6 ~ 13 mm 처진다 (ki 0 으로 확인).
     호스트 (ROS 노드) 가 측정 관절값으로 목표를 고쳐 주는 적분 보정을 흉내 냄: 목표 = 계획 + bias, bias += ki · dt · (계획 − 측정).

    ./run_rl.sh pallet_check            # -> pallet_place_check.json
    ./run_rl.sh pallet_check_play       # 창으로 (대 수를 줄여 보기: --num_envs 4)
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--traj", default=None, help="궤적 파일 (기본 pallet_place_traj.json)")
parser.add_argument("--speed", type=float, default=0.8, help="[rad/s] 관절 목표 속도 한도")
parser.add_argument("--num_envs", type=int, default=24, help="대 수 (칸 12 x 원기둥 · 상자 순서로 앞에서부터)")
parser.add_argument("--ki", type=float, default=4.0, help="[1/s] 호스트 쪽 처짐 보정: 관절 목표 = 계획 + 적분 (계획 − 측정) x ki. 0 = 끔 (펌웨어 I 0 그대로)")
parser.add_argument("--bias_max", type=float, default=0.15, help="[rad] 처짐 보정 한도")
parser.add_argument("--gui", action="store_true")
parser.add_argument("--debug", action="store_true", help="0 번 대의 쥐기 전후 값을 자세히 출력하고 끝냄")
parser.add_argument("--drop", action="store_true", help="팔 없이 퍽만: 칸 축 · 칸 방향에 맞춰 퍽 바닥 = 칸 벽 윗면 + 5 mm 에서 놓아 앉는지 (충돌 형상 확인)")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = not args_cli.gui
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import torch  # noqa: E402
from isaaclab_physx.physics import PhysxCfg  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg, AssetBaseCfg, RigidObjectCfg  # noqa: E402
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg, ViewerCfg  # noqa: E402
from isaaclab.scene import InteractiveSceneCfg  # noqa: E402
from isaaclab.sensors import ContactSensorCfg  # noqa: E402
from isaaclab.sim import SimulationCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from isaaclab.utils.math import quat_apply, quat_apply_inverse, quat_from_matrix  # noqa: E402

from waffle_isaac.dice2_cfg import PUCK_H, puck_cfg  # noqa: E402
from waffle_isaac.robot_cfg import (  # noqa: E402
    ARM_JOINTS, ARM_LINKS_REGEX, BODY_LINK, EE_BODY, FINGER_CLOSE, FINGER_LINKS_REGEX, FINGER_RELEASED,
    FLOOR_MATERIAL, GRIPPER_JOINT, GRIPPER_MIMIC, WAFFLE_CFG, WHEEL_REGEX,
)

TRAJ = json.load(open(args_cli.traj or os.path.join(HERE, "pallet_place_traj.json"), encoding="utf-8"))
Q_FRONT = TRAJ["transit"]["q_front"]
BODY = "{ENV_REGEX_NS}/Robot/" + BODY_LINK
CENTER_EE = TRAJ["tcp_offset_ee"][0] + TRAJ["grasp_dz"]  # 쥔 퍽 중심 = gripper_roll_link x 0.155 (TCP 아래 15 mm)
OPEN_PLACE = 0.010  # 자리 잡을 때 벌림 61.6 mm
FINGER_AT_PUCK = -0.0056  # 퍽을 둘 때 손가락 관절을 바로 이 값 (30.4 mm, 퍽 30 mm 와 0.2 mm 틈) 으로 써 넣음: 닫는 데 0.4 s 걸려 그동안 퍽이 떨어짐

_robot = WAFFLE_CFG.replace(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=WAFFLE_CFG.spawn.replace(
        activate_contact_sensors=True,
        collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),
        articulation_props=WAFFLE_CFG.spawn.articulation_props.replace(enabled_self_collisions=True),
    ),
    init_state=WAFFLE_CFG.init_state.replace(joint_pos={**WAFFLE_CFG.init_state.joint_pos, **dict(zip(ARM_JOINTS, Q_FRONT)),
                                                        GRIPPER_JOINT: OPEN_PLACE, GRIPPER_MIMIC: OPEN_PLACE}),
)


def _puck(name, shape, rgb, xy):
    p = puck_cfg(name, shape, rgb, xy)
    return p.replace(spawn=p.spawn.replace(activate_contact_sensors=True,
                                           collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0)))


def _touch(prim):
    return ContactSensorCfg(prim_path=prim, filter_prim_paths_expr=[BODY], update_period=0.0, history_length=1)


@configclass
class SceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg(physics_material=FLOOR_MATERIAL))
    robot: ArticulationCfg = _robot
    puckC: RigidObjectCfg = _puck("C", "cyl", (0.9, 0.1, 0.1), (0.0, 0.6))
    puckB: RigidObjectCfg = _puck("B", "box", (0.1, 0.3, 0.9), (0.0, 0.7))
    arm_body = _touch("{ENV_REGEX_NS}/Robot/" + ARM_LINKS_REGEX)
    finger_body = _touch("{ENV_REGEX_NS}/Robot/" + FINGER_LINKS_REGEX)
    puckC_body = _touch("{ENV_REGEX_NS}/PuckC")
    puckB_body = _touch("{ENV_REGEX_NS}/PuckB")
    light = AssetBaseCfg(prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.85, 0.85, 0.85)))


@configclass
class PalletCheckEnvCfg(DirectRLEnvCfg):
    decimation = 1
    episode_length_s = 1000.0
    action_space = 1
    observation_space = 1
    state_space = 0
    viewer: ViewerCfg = ViewerCfg(eye=(0.25, -0.55, 0.55), lookat=(-0.02, 0.0, 0.15), origin_type="env", env_index=0)
    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=4, physics=PhysxCfg(), physics_material=FLOOR_MATERIAL)
    scene: SceneCfg = SceneCfg(num_envs=24, env_spacing=1.5, replicate_physics=True, clone_in_fabric=True)


class PalletCheckEnv(DirectRLEnv):
    """관절 목표 (arm_target) · 손가락 목표 (grip) 를 밖에서 정해 주는 껍데기 환경 (관측 · 보상 없음)."""

    cfg: PalletCheckEnvCfg

    def __init__(self, cfg, render_mode=None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        r = self.robot
        self._arm, _ = r.find_joints(ARM_JOINTS, preserve_order=True)
        self._gl, _ = r.find_joints([GRIPPER_JOINT])
        self._gr, _ = r.find_joints([GRIPPER_MIMIC])
        self._wheels, _ = r.find_joints([WHEEL_REGEX])
        self._ee = r.find_bodies(EE_BODY)[0][0]
        self.arm_target = r.data.default_joint_pos.torch[:, self._arm].clone()
        self.grip = torch.full((self.num_envs, 1), OPEN_PLACE, device=self.device)

    @property
    def robot(self) -> Articulation:
        return self.scene["robot"]

    def _pre_physics_step(self, actions):
        pass

    def _apply_action(self):
        cmd = self.robot.actuators.target_command
        cmd.set_position_index(value=self.arm_target.contiguous(), joint_ids=self._arm)
        cmd.set_position_index(value=self.grip.expand(-1, 2).contiguous(), joint_ids=self._gl + self._gr)
        cmd.set_velocity_index(value=torch.zeros(self.num_envs, len(self._wheels), device=self.device), joint_ids=self._wheels)

    def _get_observations(self):
        return {"policy": torch.zeros(self.num_envs, 1, device=self.device)}

    def _get_rewards(self):
        return torch.zeros(self.num_envs, device=self.device)

    def _get_dones(self):
        z = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        return z, z

    def force(self, k):
        f = self.scene[k].data.normal_force_matrix_w.torch  # (N, 몸, 상대, 3)
        return torch.linalg.norm(f, dim=-1).amax(dim=(1, 2))

    def force_body(self, k):
        """가장 세게 닿은 몸 이름 (대마다)."""
        f = torch.linalg.norm(self.scene[k].data.normal_force_matrix_w.torch, dim=-1).amax(dim=2)  # (N, 몸)
        names = self.scene[k].body_names
        return [names[i] for i in f.argmax(dim=1).tolist()]


PHASES = ["grip", "carry", "place", "release", "after"]
PHASE_OF = {"rotate": 1, "carry": 1, "place": 2, "release": 3, "retract": 4, "return": 4, "rotate_back": 4}
SEGS = list(PHASE_OF)


def timeline(slot, dt, speed):
    """칸 궤적 -> 물리 스텝마다 (팔 목표 5, 손가락 hold/open, 단계) 목록. 관절 공간 직선, 속도 한도."""
    out, prev = [], list(Q_FRONT)
    for sg in slot["segments"]:
        ph = PHASE_OF[sg["name"]]
        g = 0 if sg["gripper"] == "hold" else 1
        for q in sg["q"]:
            n = max(1, int(math.ceil(max(abs(a - b) for a, b in zip(q, prev)) / (speed * dt))))
            for k in range(1, n + 1):
                out.append(([p + (a - p) * k / n for a, p in zip(q, prev)], g, ph, SEGS.index(sg["name"])))
            prev = q
        dwell = {"place": 0.3, "release": 0.6}.get(sg["name"], 0.0)
        out += [(list(prev), g, ph, SEGS.index(sg["name"]))] * int(dwell / dt)
    out += [(list(prev), 1, 4, len(SEGS) - 1)] * int(1.0 / dt)
    return out


def drop_test(u, case, is_box, ids):
    """팔 없이: 칸마다 퍽을 칸 축 · 칸 방향 (상자 면 = 반경 · 접선) 에 맞춰 퍽 바닥 = 칸 벽 윗면 + 5 mm 에서 놓음 -> 1.5 s 뒤."""
    dev, n, dt = u.device, u.num_envs, u.physics_dt
    rp, rq = u.robot.data.root_link_pos_w.torch, u.robot.data.root_link_quat_w.torch
    loc = torch.tensor([[s["x"], s["y"], s["wall_top_z"] + 0.005 + PUCK_H / 2] for s, _ in case], device=dev)
    th = torch.tensor([math.radians(s["theta_deg"]) for s, _ in case], device=dev)
    q_loc = torch.stack((torch.zeros_like(th), torch.zeros_like(th), torch.sin(th / 2), torch.cos(th / 2)), dim=1)  # (x,y,z,w), 루트 요 0 가정
    pose = torch.cat((rp + quat_apply(rq, loc), q_loc), dim=1)
    for k, m in (("puckC", ~is_box), ("puckB", is_box)):
        o = u.scene[k]
        cur = torch.cat((o.data.root_pos_w.torch, o.data.root_quat_w.torch), dim=1)
        o.write_root_pose_to_sim_index(root_pose=torch.where(m[:, None], pose, cur), env_ids=ids)
        o.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=ids)
    for _ in range(int(1.5 / dt)):
        u._apply_action()
        u.scene.write_data_to_sim()
        u.sim.step(render=args_cli.gui)
        u.scene.update(dt)
    pc = torch.where(is_box[:, None], u.scene["puckB"].data.root_pos_w.torch, u.scene["puckC"].data.root_pos_w.torch)
    qc = torch.where(is_box[:, None], u.scene["puckB"].data.root_quat_w.torch, u.scene["puckC"].data.root_quat_w.torch)
    ax = quat_apply(qc, torch.tensor([[0, 0, 1.0]], device=dev).expand(n, 3))
    bot = quat_apply_inverse(rq, pc - ax * (PUCK_H / 2) - rp)
    ok = 0
    for i, (s, shape) in enumerate(case):
        dz = float(bot[i, 2]) - s["floor_z"]
        tilt = math.degrees(math.acos(min(1.0, abs(float(ax[i, 2])))))
        ok += abs(dz) < 0.003 and tilt < 3
        print(f"[PalletDrop] {s['row']}열 {s['index']} {shape:3s} 칸 바닥 위 {dz * 1000:+6.1f} mm 기울기 {tilt:4.1f}°", flush=True)
    print(f"[PalletDrop] 요약 앉음 {ok}/{n}", flush=True)


def main():
    cfg = PalletCheckEnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    env = PalletCheckEnv(cfg)
    env.reset()
    u, dev, n, dt = env, env.device, env.num_envs, env.physics_dt
    ids = torch.arange(n, device=dev)
    slots = TRAJ["slots"]
    case = [(slots[i % len(slots)], "box" if (i // len(slots)) % 2 else "cyl") for i in range(n)]
    is_box = torch.tensor([c[1] == "box" for c in case], device=dev)

    def step(k_force=None, acc=None):
        u._apply_action()
        u.scene.write_data_to_sim()
        u.sim.step(render=args_cli.gui)
        u.scene.update(dt)

    def puck_pose():
        pc, qc = u.scene["puckC"].data.root_pos_w.torch, u.scene["puckC"].data.root_quat_w.torch
        pb, qb = u.scene["puckB"].data.root_pos_w.torch, u.scene["puckB"].data.root_quat_w.torch
        return torch.where(is_box[:, None], pb, pc), torch.where(is_box[:, None], qb, qc)

    def ee_frame():
        d = u.robot.data
        p, q = d.body_link_pos_w.torch[:, u._ee], d.body_link_quat_w.torch[:, u._ee]
        ex = quat_apply(q, torch.tensor([[1.0, 0, 0]], device=dev).expand(n, 3))
        ey = quat_apply(q, torch.tensor([[0, 1.0, 0]], device=dev).expand(n, 3))
        ez = quat_apply(q, torch.tensor([[0, 0, 1.0]], device=dev).expand(n, 3))
        return p, ex, ey, ez

    if args_cli.drop:
        drop_test(u, case, is_box, ids)
        env.close()
        return

    # 1. 앞 운반 자세에서 자리 잡기 (집게 벌림) -> 퍽을 손가락 사이에 -> 쥠
    for _ in range(int(0.5 / dt)):
        step()
    p, ex, ey, ez = ee_frame()
    center = p + ex * CENTER_EE
    R = torch.stack((ez, ey, -ex), dim=2)  # 퍽 x = 집게 z (반경 방향), 퍽 y = 닫는 축, 퍽 z = 위 (집게 반대)
    pose = torch.cat((center, quat_from_matrix(R)), dim=1)
    qj = u.robot.data.joint_pos.torch.clone()
    qj[:, u._gl[0]] = FINGER_AT_PUCK
    qj[:, u._gr[0]] = FINGER_AT_PUCK
    u.robot.write_joint_position_to_sim_index(position=qj, env_ids=ids)
    u.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(qj), env_ids=ids)
    u.grip[:] = FINGER_CLOSE
    for k, m in (("puckC", ~is_box), ("puckB", is_box)):
        o = u.scene[k]
        cur = torch.cat((o.data.root_pos_w.torch, o.data.root_quat_w.torch), dim=1)
        o.write_root_pose_to_sim_index(root_pose=torch.where(m[:, None], pose, cur), env_ids=ids)
        o.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=ids)
    if args_cli.debug:
        rp, rq = u.robot.data.root_link_pos_w.torch, u.robot.data.root_link_quat_w.torch
        b = lambda w: [round(v, 4) for v in quat_apply_inverse(rq[:1], w[:1] - rp[:1])[0].tolist()]
        print(f"[PalletDbg] 팔 관절 {[round(v, 3) for v in u.robot.data.joint_pos.torch[0, u._arm].tolist()]} (목표 {[round(v, 3) for v in Q_FRONT]})"
              f" 손가락 {u.robot.data.joint_pos.torch[0, u._gl[0]].item():.4f}", flush=True)
        print(f"[PalletDbg] 집게 원점 {b(p)} 집게 x {[round(v, 3) for v in ex[0].tolist()]} 기대 퍽 중심 {b(center)} 쿼터니언 {[round(v, 3) for v in pose[0, 3:].tolist()]}", flush=True)
        for k in range(61):
            pc_, qc_ = puck_pose()
            if k in (0, 1, 2, 5, 10, 30, 60):
                print(f"[PalletDbg] {k} 스텝 퍽 {b(pc_)} 자세 {[round(v, 3) for v in qc_[0].tolist()]} 퍽-몸체 {u.force('puckC_body')[0].item():.1f} N", flush=True)
            step()
        env.close()
        return
    u.grip[:] = FINGER_CLOSE
    for _ in range(int(0.6 / dt)):
        step()
    fing0 = u.robot.data.joint_pos.torch[:, u._gl[0]].clone()
    pe, ex, _, _ = ee_frame()
    pc0, _ = puck_pose()
    off0 = torch.linalg.norm(pc0 - (pe + ex * CENTER_EE), dim=1)
    print(f"[PalletCheck] 쥔 직후: 손가락 {[round(v, 4) for v in fing0.tolist()]} 퍽-손가락 가운데 mm {[round(v * 1000, 1) for v in off0.tolist()]}", flush=True)

    # 2 ~ 3. 궤적
    tl = [timeline(c[0], dt, args_cli.speed) for c in case]
    T = max(len(x) for x in tl)
    tgt = torch.tensor([[x[min(t, len(x) - 1)][0] for t in range(T)] for x in tl], device=dev)  # (n, T, 5)
    grp = torch.tensor([[x[min(t, len(x) - 1)][1] for t in range(T)] for x in tl], device=dev)
    phs = torch.tensor([[x[min(t, len(x) - 1)][2] for t in range(T)] for x in tl], device=dev)
    sgs = torch.tensor([[x[min(t, len(x) - 1)][3] for t in range(T)] for x in tl], device=dev)
    first = [dict() for _ in range(n)]  # 처음 닿은 때 (종류별): 구간, 시각, 관절, 퍽 바닥 (base)
    fmax = {k: torch.zeros(n, len(PHASES), device=dev) for k in ("arm", "finger", "puck")}
    slip = torch.zeros(n, device=dev)
    print(f"[PalletCheck] {n} 대, 궤적 {T} 스텝 ({T * dt:.1f} s), 쥔 손가락 {fing0.mean().item():.4f}", flush=True)
    bias = torch.zeros(n, 5, device=dev)
    for t in range(T):
        if args_cli.ki > 0:
            bias = torch.clamp(bias + args_cli.ki * dt * (tgt[:, t] - u.robot.data.joint_pos.torch[:, u._arm]), -args_cli.bias_max, args_cli.bias_max)
        u.arm_target[:] = tgt[:, t] + bias
        u.grip[:, 0] = torch.where(grp[:, t] > 0, torch.tensor(FINGER_RELEASED, device=dev), torch.tensor(FINGER_CLOSE, device=dev))
        step()
        ph = phs[:, t]
        fp = torch.where(is_box, u.force("puckB_body"), u.force("puckC_body"))
        for k, f in (("arm", u.force("arm_body")), ("finger", u.force("finger_body")), ("puck", fp)):
            fmax[k].scatter_reduce_(1, ph[:, None], f[:, None], reduce="amax")
            hit = (f > 2.0).nonzero().flatten().tolist()
            if hit:
                rp_, rq_ = u.robot.data.root_link_pos_w.torch, u.robot.data.root_link_quat_w.torch
                pc_, qc_ = puck_pose()
                az = quat_apply(qc_, torch.tensor([[0, 0, 1.0]], device=dev).expand(n, 3))
                pbb = quat_apply_inverse(rq_, pc_ - az * (PUCK_H / 2) - rp_)
                who = u.force_body({"arm": "arm_body", "finger": "finger_body"}[k]) if k != "puck" else None
                for i in hit:
                    if k not in first[i]:
                        first[i][k] = dict(link=who[i] if who else None, seg=SEGS[int(sgs[i, t])], t_s=round(t * dt, 2), q=[round(v, 3) for v in u.robot.data.joint_pos.torch[i, u._arm].tolist()],
                                           puck_bottom_b=[round(v, 4) for v in pbb[i].tolist()], N=round(float(f[i]), 1))
        held = ph <= 2
        if held.any():
            pe, ex, _, _ = ee_frame()
            pc, _ = puck_pose()
            slip = torch.where(held, torch.maximum(slip, torch.linalg.norm(pc - (pe + ex * CENTER_EE), dim=1)), slip)
        if t % 240 == 0:
            print(f"[PalletCheck] t={t * dt:5.1f} s 단계 {[PHASES[int(v)] for v in ph[:min(n, 12)].tolist()]}", flush=True)

    # 4. 결과 (base_link 좌표)
    d = u.robot.data
    rp, rq = d.root_link_pos_w.torch, d.root_link_quat_w.torch
    pc, qc = puck_pose()
    ax = quat_apply(qc, torch.tensor([[0, 0, 1.0]], device=dev).expand(n, 3))
    ax = ax * torch.where(ax[:, 2:3] < 0, -1.0, 1.0)
    bot = quat_apply_inverse(rq, pc - ax * (PUCK_H / 2) - rp)
    pxb = quat_apply_inverse(rq, quat_apply(qc, torch.tensor([[1.0, 0, 0]], device=dev).expand(n, 3)))
    res = []
    for i, (s, shape) in enumerate(case):
        dxy = math.hypot(float(bot[i, 0]) - s["x"], float(bot[i, 1]) - s["y"])
        dz = float(bot[i, 2]) - s["floor_z"]
        tilt = math.degrees(math.acos(min(1.0, float(ax[i, 2]))))
        yaw = math.degrees(math.atan2(float(pxb[i, 1]), float(pxb[i, 0])))
        face = ((yaw - s["theta_deg"] + 45.0) % 90.0) - 45.0
        ok = dxy < 0.003 and abs(dz) < 0.003 and tilt < 5.0 and float(slip[i]) < 0.015
        r = dict(slot=f"{s['row']}열 {s['index']}", puck_shape=shape, seated=ok, xy_err_mm=round(dxy * 1000, 1), bottom_above_floor_mm=round(dz * 1000, 1),
                 tilt_deg=round(tilt, 1), box_face_err_deg=round(face, 1) if shape == "box" else None, max_slip_mm=round(float(slip[i]) * 1000, 1),
                 max_force_N={k: {PHASES[j]: round(float(fmax[k][i, j]), 1) for j in range(1, len(PHASES))} for k in fmax}, first_contact=first[i])
        res.append(r)
        f = r["max_force_N"]
        print(f"[PalletCheck] {r['slot']} {shape:3s} {'앉음' if ok else '실패'} | 수평 {r['xy_err_mm']:5.1f} mm 높이 {r['bottom_above_floor_mm']:+6.1f} mm 기울기 {r['tilt_deg']:5.1f}°"
              f"{'' if shape != 'box' else f' 면 {face:+5.1f}°'} | 미끄럼 {r['max_slip_mm']:4.1f} mm | 팔-몸체 N carry {f['arm']['carry']} place {f['arm']['place']} after {f['arm']['after']}"
              f" | 손가락-몸체 place {f['finger']['place']} release {f['finger']['release']} after {f['finger']['after']} | 퍽-몸체 carry {f['puck']['carry']}", flush=True)
        for k, v in first[i].items():
            print(f"[PalletCheck]     처음 닿음 {k}: {v}", flush=True)
    summary = dict(seated=sum(r["seated"] for r in res), total=n, speed_rad_s=args_cli.speed, ki=args_cli.ki,
                   arm_body_max_N=round(float(fmax["arm"].max()), 1), finger_body_max_N=round(float(fmax["finger"].max()), 1))
    print(f"[PalletCheck] 요약 {summary}", flush=True)
    json.dump(dict(summary=summary, traj_max_tilt_deg=TRAJ.get("max_tilt_deg"), results=res), open(os.path.join(HERE, "pallet_place_check.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
