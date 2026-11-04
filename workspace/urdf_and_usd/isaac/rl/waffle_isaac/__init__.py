"""waffle_4wd 주행 강화 학습 과제 (팀 공유본 isaac/waffle_4wd_isaac 로봇 설정을 씀). import 하면 gym 에 등록된다.
  팔 과제 (TopPick · SidePick · Store · Insert) 와 팔레트 궤적은 urdf_and_usd/policy/manipulator 에 있음 (2026-11-04 정리).

  Waffle-Drive-Direct     주행 (무작위 목표점까지 4륜 스키드, drive_env.py)
  Waffle-Dock-Direct      박스 앞 세우기 (주사위 경기장, approach_poses.yaml final 자세로, dock_env.py)
"""

import gymnasium as gym

from . import agents

for _id, _mod, _cfg, _agent in (
    ("Waffle-Dock-Direct", "dock_env:WaffleDockEnv", "dock_env_cfg:WaffleDockEnvCfg", "WaffleDockPPORunnerCfg"),
    ("Waffle-Drive-Direct", "drive_env:WaffleDriveEnv", "drive_env_cfg:WaffleDriveEnvCfg", "WaffleDrivePPORunnerCfg"),
):
    gym.register(
        id=_id,
        entry_point=f"{__name__}.{_mod}",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.{_cfg}",
            "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:{_agent}",
            "default_agent": "rsl_rl",
        },
    )
