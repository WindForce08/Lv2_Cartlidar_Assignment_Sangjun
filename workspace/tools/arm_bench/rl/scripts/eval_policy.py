"""학습한 정책 평가 (창 없이): 여러 대를 돌려 성공 (에피소드 끝에 목표에 세워 놓고 빠져 있음) · 실패 (넘어뜨림 · 밀림) 를 센다.

    ./run.sh eval [--num_envs 64] [--episodes 500] [--checkpoint 경로]

정책: logs/rsl_rl/arm_pick/<가장 최근>/model_<가장 큰 번호>.pt
결과: results/eval.json. 그 학습 폴더 exported/ 에 policy.pt (TorchScript) · policy.onnx (실물로 옮길 때 - 관측 정규화 포함)
"""

import argparse
import glob
import json
import os
import re

import _common

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(description="arm_bench 정책 평가")
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--episodes", type=int, default=500, help="이만큼 끝날 때까지")
parser.add_argument("--checkpoint", default=None)
add_launcher_args(parser)
args_cli = parser.parse_args()

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import arm_pick  # noqa: E402,F401
from arm_pick.agents.rsl_rl_ppo_cfg import ArmPickPPORunnerCfg  # noqa: E402
from arm_pick.pick_env_cfg import ArmPickEnvCfg  # noqa: E402

LOG_DIR = os.path.join(_common.ROOT, "logs", "rsl_rl", "arm_pick")


def latest_checkpoint():
    for run in reversed(sorted(glob.glob(os.path.join(LOG_DIR, "*")))):
        models = glob.glob(os.path.join(run, "model_*.pt"))
        if models:
            return max(models, key=lambda p: int(re.findall(r"model_(\d+)\.pt", p)[0]))
    return None


def main():
    path = args_cli.checkpoint or latest_checkpoint()
    if not path or not os.path.exists(path):
        raise SystemExit("체크포인트 (model_*.pt) 가 없음 - 먼저 ./run.sh train")
    cfg = ArmPickEnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    cfg.show_target = False
    if args_cli.device is not None:
        cfg.sim.device = args_cli.device
    os.makedirs(_common.RESULTS, exist_ok=True)
    with launch_simulation(cfg, args_cli):
        from importlib import metadata

        from rsl_rl.runners import OnPolicyRunner

        from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg

        agent = handle_deprecated_rsl_rl_cfg(ArmPickPPORunnerCfg(), metadata.version("rsl-rl-lib"))
        env = RslRlVecEnvWrapper(gym.make(arm_pick.TASK, cfg=cfg), clip_actions=agent.clip_actions)
        u = env.unwrapped
        runner = OnPolicyRunner(env, agent.to_dict(), log_dir=None, device=u.device)
        runner.load(path)
        policy = runner.get_inference_policy(device=u.device)
        export_dir = os.path.join(os.path.dirname(path), "exported")
        runner.export_policy_to_jit(path=export_dir, filename="policy.pt")
        runner.export_policy_to_onnx(path=export_dir, filename="policy.onnx")
        obs = env.get_observations()
        try:
            jit = torch.jit.load(os.path.join(export_dir, "policy.pt"), map_location=u.device)
            with torch.inference_mode():
                export_diff = float((policy(obs) - jit(obs["policy"])).abs().max())
        except Exception as ex:  # noqa: BLE001
            export_diff = f"확인 못 함 ({ex})"
        n_ep = n_ok = n_fail = n_lift = n_ever = 0
        t_ep = torch.zeros(u.num_envs, device=u.device)
        t_first = torch.full((u.num_envs,), float("nan"), device=u.device)   # 처음 성공 상태가 된 시각
        times = []
        while n_ep < args_cli.episodes:
            with torch.inference_mode():
                act = policy(obs)
            obs, _, dones, _ = env.step(act)
            t_ep += u.step_dt
            d = dones.bool()
            if d.any():   # 끝난 환경은 이미 리셋됨 - 끝난 이유는 리셋 전에 계산해 둔 판정으로
                n_ep += int(d.sum())
                n_ok += int(u.placed[d].sum())
                n_fail += int(u.failed[d].sum())
                n_lift += int(u.was_lifted[d].sum())
                n_ever += int(u.placed_ever[d].sum())
                t_ep[d] = 0.0
            first = u.placed & torch.isnan(t_first)
            t_first[first] = t_ep[first]
            if d.any():
                times += [v for v in t_first[d].tolist() if v == v]
                t_first[d] = float("nan")
        res = dict(checkpoint=path, episodes=n_ep, success_rate=round(n_ok / n_ep, 3), placed_ever_rate=round(n_ever / n_ep, 3),
                   failed_rate=round(n_fail / n_ep, 3), lifted_rate=round(n_lift / n_ep, 3),
                   mean_time_to_place_s=round(sum(times) / len(times), 2) if times else None, exported=export_dir, exported_policy_max_diff=export_diff)
        print(f"[arm_bench] 평가: {res}")
        with open(os.path.join(_common.RESULTS, "eval.json"), "w", encoding="utf-8") as f:
            json.dump(res, f, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
