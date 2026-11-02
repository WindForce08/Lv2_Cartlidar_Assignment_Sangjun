"""waffle_4wd 매니퓰레이터 강화 학습 과제 (팀 공유본 isaac/waffle_4wd_isaac 로봇 설정을 씀). import 하면 gym 에 등록된다.

  Waffle-TopPick-Direct   B안 팔레트용 집기 (상자 위 퍽을 위에서 집어 들기, 2026-11-02). 놓기는 고정 궤적 (../pallet_traj.py, ../pallet_place_check.py)

  아래 셋은 옛 퍽 보관통 (C · P · N 통, 261028) 기준 - B안 팔레트에서는 쓰지 않음 (기록 · 비교용으로 남김):
  Waffle-SidePick-Direct  1 정렬·집기 (상자 위 퍽을 옆에서 집어 들기)
  Waffle-Store-Direct     2 옮기기·맞추기 (높이 들어 180° 돌려 뒤 스토리지 통 위에 맞춤)
  Waffle-Insert-Direct    3 넣기 (통 위 퍽을 통 안에 넣고 놓고 빼기)
"""

import gymnasium as gym

from . import agents

for _id, _mod, _cfg, _agent in (
    ("Waffle-TopPick-Direct", "toppick_env:WaffleTopPickEnv", "toppick_env_cfg:WaffleTopPickEnvCfg", "WaffleTopPickPPORunnerCfg"),
    ("Waffle-SidePick-Direct", "sidepick_env:WaffleSidePickEnv", "sidepick_env_cfg:WaffleSidePickEnvCfg", "WaffleSidePickPPORunnerCfg"),
    ("Waffle-Store-Direct", "store_env:WaffleStoreEnv", "store_env_cfg:WaffleStoreEnvCfg", "WaffleStorePPORunnerCfg"),
    ("Waffle-Insert-Direct", "insert_env:WaffleInsertEnv", "insert_env_cfg:WaffleInsertEnvCfg", "WaffleInsertPPORunnerCfg"),
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
