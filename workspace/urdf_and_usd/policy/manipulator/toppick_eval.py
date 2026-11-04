"""위에서 집기 `Waffle-TopPick-Direct` 실제 과제 평가 (창 없이, 먼 시작만): 대마다 첫 에피소드 결과 (1 s 세워 들기 + 옆 퍽 그대로, 넘어짐) 를
배치 (무작위 · 실제 A · B · C, #78) · 기울임 / 수직 · 상자 / 원기둥 퍽별로 셈.

    ./run_rl.sh toppick_eval [환경 수] [--seed N] [--layout mix|rand|A|B|C] [--latch 0~1]
기본 policies/toppick_latest.pt, 256 대. 체크포인트를 고를 때와 보고할 때 시드를 다르게 (#71).
"""
import argparse, os, sys
from isaaclab.app import AppLauncher
ap = argparse.ArgumentParser(); ap.add_argument("--num_envs", type=int, default=256)
ap.add_argument("--tilt_prob", type=float, default=None, help="기본 = 환경 설정 (1.0)")
ap.add_argument("--policy", default=None); ap.add_argument("--near", type=float, default=0.0)
ap.add_argument("--seed", type=int, default=1, help="평가 시드 (학습 · 고르기와 다르게)")
ap.add_argument("--layout", default="mix", choices=["mix", "rand", "A", "B", "C"], help="mix = 환경 설정 비율, 그 밖은 그 배치만")
ap.add_argument("--latch", type=float, default=None, help="인식 좌표 고정 비율 (기본 = 환경 설정)")
ap.add_argument("--dr", default="all", help="물리 무작위화: all · none · 뺄 것만 -friction,-gains,-mass,-delay,-obs_noise (절제 실험)")
AppLauncher.add_app_launcher_args(ap); a = ap.parse_args(); a.headless = True
app = AppLauncher(a).app
RL = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, RL)
import gymnasium as gym, torch
import waffle_isaac  # noqa
from waffle_isaac.toppick_env import LAYOUTS
from waffle_isaac.toppick_env_cfg import WaffleTopPickEnvCfg, set_dr
path = a.policy or os.path.join(RL, "policies", "toppick_latest.pt")
cfg = WaffleTopPickEnvCfg(); cfg.scene.num_envs = a.num_envs; cfg.near_start_prob = a.near; cfg.seed = a.seed
set_dr(cfg, a.dr)
if a.tilt_prob is not None:
    cfg.tilt_prob = a.tilt_prob
if a.latch is not None:
    cfg.perc_latch_prob = a.latch
if a.layout == "rand":
    cfg.layout_prob = 0.0
elif a.layout != "mix":
    cfg.layout_prob = 1.0; cfg.tilt_prob = 1.0
    cfg.layout_weights = tuple(1.0 if k == a.layout else 0.0 for k in LAYOUTS[1:])
env = gym.make("Waffle-TopPick-Direct", cfg=cfg); u = env.unwrapped
pol = torch.jit.load(path, map_location=u.device).eval()
obs, _ = env.reset(); n = u.num_envs
tilted, box, lay, latched = u.tilted.clone(), u.is_box.clone(), u.layout.clone(), u.latched.clone()
done = torch.zeros(n, dtype=torch.bool, device=u.device); ok = torch.zeros_like(done); topple = torch.zeros_like(done); maxlift = torch.zeros(n, device=u.device)
nb_bad = torch.zeros_like(done); armbox = torch.zeros(n, device=u.device)
for t in range(int(cfg.episode_length_s / u.step_dt) - 2):
    with torch.inference_mode():
        act = pol(obs["policy"])
    live = ~done
    ok |= u.succeeded & live
    obs, r, term, trunc, _ = env.step(act)
    live2 = ~done
    topple |= (u.puck_up() < 0.7) & live2
    nb_bad |= ~u.nb_ok() & live2
    maxlift = torch.where(live2, torch.maximum(maxlift, u.lift()), maxlift)
    armbox += (u._contact("arm_box_contact") & live2).float()
    done |= term | trunc
ok |= u.succeeded & ~done
def s(m): return f"{int(ok[m].sum())}/{int(m.sum())} ({ok[m].float().mean().item()*100:.0f} %)" if m.any() else "-"
print("[Eval] 정책", path, "| 시드", a.seed, "| 배치", a.layout, "| 무작위화", a.dr)
print(f"[Eval] 성공 전체 {s(torch.ones_like(ok))} | " + " ".join(f"{k} {s(lay == i)}" for i, k in enumerate(LAYOUTS)))
print(f"[Eval] 기울임 {s(tilted)} 수직 {s(~tilted)} | 기울임·상자 {s(tilted & box)} 기울임·원기둥 {s(tilted & ~box)} | 좌표 고정 {s(latched)} 따라감 {s(~latched)}")
print(f"[Eval] 넘어짐 {int(topple.sum())} | 옆 퍽 넘어짐·밀림 {int(nb_bad.sum())} | 팔-상자 닿은 스텝 평균 {armbox.mean().item():.1f} | 최대 들림 mm 중앙값 {maxlift.median().item()*1000:.0f}")
env.close(); app.close()
