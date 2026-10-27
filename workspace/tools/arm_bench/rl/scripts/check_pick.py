"""물리 확인 (창 없이): 학습 전에 과제가 풀 수 있는 문제인지 - 손으로 짠 순서 (IK) 로 퍽을 집어 목표에 세워 놓아 본다.

    ./run.sh check [--envs 4] [--puck 0.16 0.04] [--target 0.13 -0.07]

순서 (집게 수평): 퍽 안쪽 위 → 내려가기 → 앞으로 뻗기 → 닫기 → 들기 → 목표 위로 → 내려놓기 → 열기 → 빼기 → 올리기
  (행동 = 과제와 같은 4개, 정책 대신 이 스크립트가 줌. 실제 집게 끝을 보고 따라감 = 처짐 보정).
확인: 시뮬 집게 끝 (USD 몸체) ↔ kinematics.py 순기구학 차이, 쥘 때 손가락 벌림 (퍽 30 mm 면 약 -6 mm), 들린 높이, 놓은 오차, 성공 판정.
결과: results/check_pick.json (+ 터미널 표)
"""

import argparse
import json
import os

import _common

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(description="arm_bench 집기·놓기 물리 확인")
parser.add_argument("--envs", type=int, default=4)
parser.add_argument("--puck", type=float, nargs=2, default=(0.30, 0.06), metavar=("X", "Y"))
parser.add_argument("--target", type=float, nargs=2, default=(0.27, -0.10), metavar=("X", "Y"))
add_launcher_args(parser)
args_cli = parser.parse_args()

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import arm_pick  # noqa: E402,F401
from arm_pick import kinematics as K  # noqa: E402
from arm_pick.pick_env_cfg import ArmPickEnvCfg  # noqa: E402

GRASP_Z = 0.03 + 0.015          # 집게 끝 높이 = 퍽 중심 + grasp_tip_above_center
BACK = 0.06                      # [m] 퍽 안쪽 (J1 축 쪽) 으로 물러난 거리 - 벌린 손가락 끝이 퍽 뒤 면에서 2 cm
PHASES = [  # 이름, 기준 (퍽/목표), 기준에서 J1 축 쪽으로 물러남 [m], 집게 끝 높이, 쥐기, 최대 걸음 (닫기·열기는 그만큼 기다림)
    ("퍽 안쪽 위", "puck", BACK, 0.11, -1.0, 150),
    ("내려가기", "puck", BACK, GRASP_Z, -1.0, 120),
    ("앞으로 뻗기", "puck", 0.0, GRASP_Z, -1.0, 120),
    ("닫기", "puck", 0.0, GRASP_Z, 1.0, 30),
    ("들기", "puck", 0.0, 0.08, 1.0, 120),
    ("목표 위로", "target", 0.0, 0.08, 1.0, 150),
    ("내려놓기", "target", 0.0, GRASP_Z + 0.002, 1.0, 120),
    ("열기", "target", 0.0, GRASP_Z + 0.002, -1.0, 30),
    ("빼기", "target", BACK, GRASP_Z + 0.002, -1.0, 120),
    ("올리기", "target", BACK, 0.11, -1.0, 90),
]


def waypoint(xy, back, z, n, dev):
    """기준 점에서 J1 축 쪽으로 back 만큼 물러난 점 (같은 방향), 높이 z"""
    d = torch.linalg.norm(xy - torch.tensor([K.J1_X, 0.0], device=dev))
    p = torch.tensor([K.J1_X, 0.0], device=dev) + (xy - torch.tensor([K.J1_X, 0.0], device=dev)) * (d - back) / d
    return torch.cat((p, torch.tensor([z], device=dev))).repeat(n, 1)


def main():
    cfg = ArmPickEnvCfg()
    cfg.scene.num_envs = args_cli.envs
    cfg.fixed_task = (tuple(args_cli.puck), tuple(args_cli.target))
    cfg.episode_length_s = 60.0
    cfg.show_target = False
    if args_cli.device is not None:
        cfg.sim.device = args_cli.device
    os.makedirs(_common.RESULTS, exist_ok=True)
    with launch_simulation(cfg, args_cli):
        env = gym.make(arm_pick.TASK, cfg=cfg)
        u = env.unwrapped
        env.reset()
        dev, n = u.device, u.num_envs
        puck_xy = torch.tensor(args_cli.puck, device=dev)
        target_xy = torch.tensor(args_cli.target, device=dev)
        fk_err = 0.0
        placed_flag = torch.zeros(n, dtype=torch.bool, device=dev)
        failed_flag = torch.zeros_like(placed_flag)
        result = dict(puck=args_cli.puck, target=args_cli.target, phases=[])
        for name, ref, back, z, grip, max_steps in PHASES:
            wp = waypoint(puck_xy if ref == "puck" else target_xy, back, z, n, dev)
            steps = 0
            for steps in range(1, max_steps + 1):
                a = torch.zeros(n, 4, device=dev)
                a[:, :3] = (0.5 * (wp - u.tip_pos()) / u.cfg.tip_step).clamp(-1.0, 1.0) * (~(placed_flag | failed_flag)).float().unsqueeze(1)
                a[:, 3] = grip
                env.step(a)
                placed_flag |= u.placed                         # 성공해도 끝나지 않음 - 그 걸음의 판정을 모음 (실패는 리셋됨)
                failed_flag |= u.failed
                tip = u.tip_pos()
                q = u.robot.data.joint_pos.torch[:, u._arm_ids]
                fx, fy, fz, _ = K.fk_tip(q[:, 0], q[:, 1], q[:, 2], q[:, 3])
                fk_err = max(fk_err, torch.linalg.norm(torch.stack((fx, fy, fz), 1) - tip, dim=1).max().item())
                settled = torch.linalg.norm(tip - wp, dim=1).max().item() < 0.003
                if name not in ("닫기", "열기") and settled:
                    break
            tip = u.tip_pos()
            puck, up_z = u.puck_state()
            ph = dict(name=name, steps=steps, tip=[round(v, 4) for v in tip[0].tolist()], tip_err_mm=round(torch.linalg.norm(tip - wp, dim=1).max().item() * 1000, 1),
                      finger_mm=round(u.finger()[0].item() * 1000, 2), puck=[round(v, 4) for v in puck[0].tolist()], puck_up=round(up_z[0].item(), 3),
                      grasped=int(u.grasped.sum().item()), lifted=int(u.lifted.sum().item()))
            result["phases"].append(ph)
            print(f"[check] {name:6s} {steps:3d} 걸음 | 끝 {ph['tip']} (목표와 {ph['tip_err_mm']:5.1f} mm) | 손가락 {ph['finger_mm']:+6.2f} mm | "
                  f"퍽 {ph['puck']} 세움 {ph['puck_up']} | 쥠 {ph['grasped']}/{n} 들림 {ph['lifted']}/{n}")
        puck, up_z = u.puck_state()
        err = torch.linalg.norm(puck[:, :2] - target_xy, dim=1)
        result.update(fk_max_err_mm=round(fk_err * 1000, 3), place_err_mm=[round(v * 1000, 1) for v in err.tolist()],
                      placed=[bool(v) for v in placed_flag.tolist()], failed=[bool(v) for v in failed_flag.tolist()],
                      max_lift_mm=round(max(p["puck"][2] for p in result["phases"]) * 1000 - 30.0, 1))
        ok = all(result["placed"]) and not any(result["failed"])
        print(f"[check] 순기구학 차이 최대 {result['fk_max_err_mm']} mm | 가장 높이 든 퍽 바닥 {result['max_lift_mm']} mm | "
              f"성공 {result['placed']} | 실패 (넘어짐·밀림) {result['failed']}")
        print(f"[check] {'통과' if ok else '실패'} - results/check_pick.json")
        with open(os.path.join(_common.RESULTS, "check_pick.json"), "w", encoding="utf-8") as f:
            json.dump(result, f, ensure_ascii=False, indent=1)
        env.close()


if __name__ == "__main__":
    main()
