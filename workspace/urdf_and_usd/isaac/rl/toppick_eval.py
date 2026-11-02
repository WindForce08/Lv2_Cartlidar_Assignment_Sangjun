"""위에서 집기 `Waffle-TopPick-Direct` 실제 과제 평가 (창 없이, 먼 시작만): 대마다 첫 에피소드 결과 (1 s 세워 들기 성공, 넘어짐) 를
기울임 (팔레트 기둥 옆 4칸용 45°) · 수직 모드, 상자 · 원기둥 퍽별로 셈.

    ./run_rl.sh toppick_eval [환경 수]          # 기본 policies/toppick_latest.pt, 256 대
"""
import argparse, os, sys
from isaaclab.app import AppLauncher
ap = argparse.ArgumentParser(); ap.add_argument("--num_envs", type=int, default=256); ap.add_argument("--tilt_prob", type=float, default=0.5)
ap.add_argument("--policy", default=None); ap.add_argument("--near", type=float, default=0.0)
AppLauncher.add_app_launcher_args(ap); a = ap.parse_args(); a.headless = True
app = AppLauncher(a).app
RL = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, RL)
import gymnasium as gym, torch
import waffle_isaac  # noqa
from waffle_isaac.toppick_env_cfg import WaffleTopPickEnvCfg
path = a.policy or os.path.join(RL, "policies", "toppick_latest.pt")
cfg = WaffleTopPickEnvCfg(); cfg.scene.num_envs = a.num_envs; cfg.near_start_prob = a.near; cfg.tilt_prob = a.tilt_prob
env = gym.make("Waffle-TopPick-Direct", cfg=cfg); u = env.unwrapped
pol = torch.jit.load(path, map_location=u.device).eval()
obs, _ = env.reset(); n = u.num_envs
tilted, box = u.tilted.clone(), u.is_box.clone()
done = torch.zeros(n, dtype=torch.bool, device=u.device); ok = torch.zeros_like(done); topple = torch.zeros_like(done); maxlift = torch.zeros(n, device=u.device)
armbox = torch.zeros(n, device=u.device)
for t in range(int(cfg.episode_length_s / u.step_dt) - 2):
    with torch.inference_mode():
        act = pol(obs["policy"])
    live = ~done
    ok |= u.succeeded & live
    obs, r, term, trunc, _ = env.step(act)
    live2 = ~done
    topple |= (u.puck_up() < 0.7) & live2
    maxlift = torch.where(live2, torch.maximum(maxlift, u.lift()), maxlift)
    armbox += (u._contact("arm_box_contact") & live2).float()
    done |= term | trunc
ok |= u.succeeded & ~done
def s(m): return f"{int(ok[m].sum())}/{int(m.sum())} ({ok[m].float().mean().item()*100:.0f} %)"
print("[Eval] 정책", path)
print(f"[Eval] 성공 전체 {s(torch.ones_like(ok))} | 기울임 {s(tilted)} 수직 {s(~tilted)} | 기울임·상자 {s(tilted & box)} 기울임·원기둥 {s(tilted & ~box)} 수직·상자 {s(~tilted & box)} 수직·원기둥 {s(~tilted & ~box)}")
print(f"[Eval] 넘어짐 기울임 {int(topple[tilted].sum())} 수직 {int(topple[~tilted].sum())} | 팔-상자 닿은 스텝 평균 {armbox.mean().item():.1f} | 최대 들림 mm 중앙값 {maxlift.median().item()*1000:.0f}")
env.close(); app.close()
