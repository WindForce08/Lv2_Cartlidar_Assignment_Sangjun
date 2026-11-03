"""단계 A 평가 (창 없이): 고정 목표 집합 (환경 수 × 환경마다 에피소드 수, seed 고정) 에서 정책 (결정적 행동) 또는
기준선 추종기를 돌려 성공률 · 걸린 시간 · 시간 비율 · 넘어짐 · 최대 기울기 · 최대 전진 가속을 낸다.
회차가 바뀌어도 같은 목표에서 비교한다. 정책과 기준선은 같은 base_cmd (drive_node · diff_drive · 펌웨어 한계) 를 지난다.

    ./run.sh flat_eval [환경 수]       최근 학습 (logs/rsl_rl/waffle_drive_flat/<최근>/exported/policy.pt)
    ./run.sh flat_eval_pp [환경 수]    기준선: wz = 2 × 방향 오차 (±1.5), |오차| < 30° 일 때만 vx = 0.3 cos(오차)
결과: results/eval_flat_<학습 폴더 이름 | pp>.json. 시간 비율 = 걸린 시간 / (|처음 방향 오차| / 1.5 + 거리 / 0.3)
"""

import argparse
import glob
import json
import math
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--episodes", type=int, default=8, help="환경마다 에피소드 수 (목표 집합 = 환경 수 × 이것)")
parser.add_argument("--seed", type=int, default=12345, help="목표 집합 seed (회차끼리 비교하려면 바꾸지 말 것)")
parser.add_argument("--controller", choices=("policy", "pp"), default="policy")
parser.add_argument("--policy", default=None, help="exported/policy.pt (없으면 최근 학습)")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import waffle_drive  # noqa: E402,F401
from waffle_drive.base_cmd import SKILL_VX, SKILL_WZ  # noqa: E402
from waffle_drive.flat_env_cfg import WaffleDriveFlatEnvCfg  # noqa: E402


def pp_actions(obs: torch.Tensor) -> torch.Tensor:
    """기준선: 목표 쪽으로 돌고 직진 (방향 P 제어). 관측 순서 (flat_env): 5 · 6 = 몸체 좌표 목표 (x, y)

    곡률만 맞추는 추종기 (wz = vx · 2 sin(오차) / 거리) 는 시뮬 스키드 조향에서 실패한다: 앞으로 가며 주는 작은 회전 명령
    (< 약 0.15 rad/s) 은 거의 돌지 않고, 그 위로도 명령의 50 ~ 65 % 만 돈다 (2026-11-02 진단, 성공 56.5 %).
    """
    err = torch.atan2(obs[:, 6], obs[:, 5])
    wz = (2.0 * err).clamp(-SKILL_WZ, SKILL_WZ)
    vx = torch.where(err.abs() < math.radians(30), SKILL_VX * torch.cos(err), torch.zeros_like(err))
    return torch.stack((vx / SKILL_VX, wz / SKILL_WZ), dim=1)


def main():
    pol, path = None, None
    if args_cli.controller == "policy":
        path = args_cli.policy or (sorted(glob.glob(os.path.join(HERE, "logs/rsl_rl/waffle_drive_flat/*/exported/policy.pt"))) or [None])[-1]
        if path is None:
            raise SystemExit("정책이 없습니다 — ./run.sh flat 로 학습한 뒤 ./run.sh flat_eval")
        tag = os.path.basename(os.path.dirname(os.path.dirname(path)))
    else:
        tag = "pp"
    cfg = WaffleDriveFlatEnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    cfg.fixed_tasks_seed = args_cli.seed
    cfg.fixed_tasks_per_env = args_cli.episodes
    env = gym.make("Waffle-DriveFlat-Direct", cfg=cfg)
    u = env.unwrapped
    if path:
        pol = torch.jit.load(path, map_location=u.device).eval()
    obs, _ = env.reset()
    n, k = u.num_envs, args_cli.episodes
    rows = {key: [] for key in u.last_episode}
    count = torch.zeros(n, dtype=torch.long, device=u.device)
    for _ in range(int(k * cfg.episode_length_s / u.step_dt) + 10):
        with torch.inference_mode():
            a = pol(obs["policy"]) if pol is not None else pp_actions(obs["policy"])
        obs, _, term, trunc, _ = env.step(a)
        ended = term | trunc
        keep = ended & (count < k)
        if keep.any():
            for key, v in u.last_episode.items():
                rows[key].append(v[keep].clone())
        count += ended.long()
        if bool((count >= k).all()):
            break
    r = {key: torch.cat(v).float().cpu() for key, v in rows.items()}
    ok = r["success"] > 0.5
    t, ratio = r["time"][ok], (r["time"] / r["ideal_time"].clamp(min=1e-3))[ok]
    q = lambda x, p: round(x.quantile(p).item(), 3) if len(x) else None  # noqa: E731
    out = {
        "controller": args_cli.controller,
        "policy": path,
        "seed": args_cli.seed,
        "episodes": len(ok),
        "success_rate": round(ok.float().mean().item(), 4),
        "time_mean_s": round(t.mean().item(), 3) if len(t) else None,
        "time_p90_s": q(t, 0.9),
        "time_ratio_mean": round(ratio.mean().item(), 3) if len(ratio) else None,
        "time_ratio_p90": q(ratio, 0.9),
        "tipped_rate": round((r["tipped"] > 0.5).float().mean().item(), 4),
        "max_tilt_deg": round(r["max_tilt"].max().item(), 2),
        "max_accel_p50_mps2": q(r["max_accel"], 0.5),
        "max_accel_p99_mps2": q(r["max_accel"], 0.99),
        "accel_over_1p5_rate": round((r["max_accel"] > 1.5).float().mean().item(), 4),
    }
    os.makedirs(os.path.join(HERE, "results"), exist_ok=True)
    dst = os.path.join(HERE, "results", f"eval_flat_{tag}.json")
    with open(dst, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print(f"[Eval] {'정책 ' + path if path else '기준선 추종기 (pp)'}", flush=True)
    print(f"[Eval] 에피소드 {out['episodes']} 개 (seed {out['seed']}): 성공 {out['success_rate'] * 100:.1f} %, "
          f"걸린 시간 평균 {out['time_mean_s']} s · 90 % {out['time_p90_s']} s, 시간 비율 평균 {out['time_ratio_mean']} · 90 % {out['time_ratio_p90']}", flush=True)
    print(f"[Eval] 넘어짐 {out['tipped_rate'] * 100:.1f} %, 최대 기울기 {out['max_tilt_deg']}°, "
          f"에피소드별 최대 전진 가속 중앙값 {out['max_accel_p50_mps2']} · 99 % {out['max_accel_p99_mps2']} m/s² "
          f"(1.5 넘은 에피소드 {out['accel_over_1p5_rate'] * 100:.1f} %)", flush=True)
    print(f"[Eval] 저장: {dst}", flush=True)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
