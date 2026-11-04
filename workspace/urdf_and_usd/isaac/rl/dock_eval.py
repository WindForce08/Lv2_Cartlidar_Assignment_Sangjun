"""박스 앞 세우기 `Waffle-Dock-Direct` 평가 (창 없이): 목표 11 곳에 같은 수씩 대를 나눠, 대마다 첫 에피소드 결과를 셈.

결과 = 성공 · 면 넘음 · 차체 접촉 · 벗어남 · 시간 초과. 목표별 성공률, 성공까지 걸린 시간, 끝날 때 남은 오차 (앞뒤 · 옆 · 요).
시작 분포 · 관측 잡음은 학습과 같음 (dock_env_cfg.py). 행동 = 정책 평균 (잡음 없음).

    ./run_rl.sh dock_eval [환경 수]                    # 가장 최근 학습의 마지막 model_*.pt, 기본 220 대 (목표당 20)
    ./run_rl.sh dock_eval 440 --checkpoint logs/rsl_rl/waffle_dock/<시각>/model_500.pt
결과 JSON: 체크포인트 옆 eval_<모델 이름>.json (--out 으로 바꿈)
"""
import argparse
import glob
import json
import os
import re
import sys

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser(description="박스 앞 세우기 평가")
ap.add_argument("--num_envs", type=int, default=220)
ap.add_argument("--checkpoint", default=None, help="model_*.pt (학습 체크포인트) 또는 내보낸 policy.pt (JIT). 기본: 최근 학습의 마지막 모델")
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--out", default=None)
AppLauncher.add_app_launcher_args(ap)
a = ap.parse_args()
a.headless = True
app = AppLauncher(a).app

RL = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, RL)
import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import waffle_isaac  # noqa: E402,F401
from waffle_isaac.dock_env import RESULTS  # noqa: E402
from waffle_isaac.dock_env_cfg import WaffleDockEnvCfg  # noqa: E402


def latest_checkpoint() -> str:
    runs = sorted(glob.glob(os.path.join(RL, "logs", "rsl_rl", "waffle_dock", "*", "")))
    for run in reversed(runs):
        models = sorted(glob.glob(os.path.join(run, "model_*.pt")), key=lambda p: int(re.findall(r"model_(\d+)", p)[-1]))
        if models:
            return models[-1]
    raise SystemExit("학습 결과가 없음: logs/rsl_rl/waffle_dock/*/model_*.pt (먼저 ./run_rl.sh dock)")


def load_policy(path: str, device):
    """학습 체크포인트 (actor_state_dict: 관측 정규화 + MLP, ELU) 또는 JIT policy.pt -> 관측 -> 행동 평균"""
    try:
        return torch.jit.load(path, map_location=device).eval()
    except RuntimeError:
        pass
    sd = torch.load(path, map_location=device, weights_only=False)["actor_state_dict"]
    n_lin = sorted({int(k.split(".")[1]) for k in sd if k.startswith("mlp.") and k.endswith(".weight")})
    layers = []
    for j, i in enumerate(n_lin):
        w = sd[f"mlp.{i}.weight"]
        lin = torch.nn.Linear(w.shape[1], w.shape[0]).to(device)
        lin.weight.data.copy_(w)
        lin.bias.data.copy_(sd[f"mlp.{i}.bias"])
        layers.append(lin)
        if j < len(n_lin) - 1:
            layers.append(torch.nn.ELU())
    mlp = torch.nn.Sequential(*layers).eval()
    mean, std = sd.get("obs_normalizer._mean"), sd.get("obs_normalizer._std")

    def pol(obs):
        if mean is not None:
            obs = (obs - mean) / (std + 1e-2)  # rsl_rl EmpiricalNormalization (eps 0.01)
        return mlp(obs)

    return pol


ckpt = os.path.abspath(a.checkpoint) if a.checkpoint else latest_checkpoint()
cfg = WaffleDockEnvCfg()
cfg.scene.num_envs = a.num_envs
cfg.seed = a.seed
env = gym.make("Waffle-Dock-Direct", cfg=cfg)
u = env.unwrapped
n, dev = u.num_envs, u.device
T = len(u.target_names)
u.eval_target = torch.arange(n, device=dev) % T
pol = load_policy(ckpt, dev)

obs, _ = env.reset()
done = torch.zeros(n, dtype=torch.bool, device=dev)
result = torch.zeros(n, dtype=torch.long, device=dev)
err = torch.zeros(n, 3, device=dev)
length = torch.zeros(n, dtype=torch.long, device=dev)
for _ in range(int(u.max_episode_length) + 2):
    with torch.inference_mode():
        act = pol(obs["policy"])
    obs, _, term, trunc, _ = env.step(act)
    new = (term | trunc) & ~done
    result[new], err[new], length[new] = u.ep_result[new], u.ep_err[new], u.ep_len[new]
    done |= new
    if bool(done.all()):
        break

dt = u.step_dt
res = {"checkpoint": ckpt, "num_envs": n, "seed": a.seed, "targets": {}}
print(f"\n[DockEval] 정책 {ckpt}\n[DockEval] {n} 대, 목표 {T} 곳 x {n // T} 대, 대마다 첫 에피소드")
head = f"{'목표':8s} {'성공':>9s} " + " ".join(f"{RESULTS[c]:>6s}" for c in (2, 3, 4, 5)) + f" {'성공 시간 s':>11s} {'실패 끝 오차 mm (앞뒤 / 옆) · 요 °':>34s}"
print(head)


def row(name, m):
    k = int(m.sum())
    cnt = {RESULTS[c]: int(((result == c) & m).sum()) for c in RESULTS}
    ok = (result == 1) & m
    fail = m & (result != 1)
    t_ok = (length[ok].float() * dt).median().item() if ok.any() else float("nan")
    e = err[fail].abs() if fail.any() else torch.full((1, 3), float("nan"), device=dev)
    ea, el, ey = (e[:, 0].median().item() * 1000, e[:, 1].median().item() * 1000, e[:, 2].median().item() * 57.2958)
    print(f"{name:8s} {cnt['성공']:3d}/{k:3d} {cnt['성공'] / max(k, 1) * 100:3.0f}% "
          + " ".join(f"{cnt[RESULTS[c]]:6d}" for c in (2, 3, 4, 5))
          + f" {t_ok:11.1f} {ea:16.0f} / {el:4.0f} · {ey:4.1f}")
    return {"n": k, **cnt, "success_rate": cnt["성공"] / max(k, 1), "success_time_s_median": t_ok,
            "fail_err_median": {"along_mm": ea, "lat_mm": el, "yaw_deg": ey}}


tgt = u.eval_target
for i, name in enumerate(u.target_names):
    res["targets"][name] = row(name, tgt == i)
res["all"] = row("전체", torch.ones(n, dtype=torch.bool, device=dev))
print("(성공 시간 = 중앙값, 실패 끝 오차 = 실패한 대의 끝 오차 절댓값 중앙값)")

out = a.out or os.path.join(os.path.dirname(ckpt), f"eval_{os.path.splitext(os.path.basename(ckpt))[0]}.json")
with open(out, "w") as f:
    json.dump(res, f, ensure_ascii=False, indent=1)
print(f"[DockEval] 저장 {out}", flush=True)
env.close()
app.close()
