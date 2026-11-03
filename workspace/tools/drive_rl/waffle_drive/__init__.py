"""waffle_4wd 주행 강화학습 과제 (팀 isaac/waffle_4wd_isaac 로봇 설정을 읽기만 함). import 하면 gym 에 등록된다
(진입점은 문자열이라 Isaac 앱을 띄우기 전에 import 해도 됨).

  Waffle-DriveFlat-Direct   단계 A: 평지에서 무작위 목표점까지
  Waffle-DriveMaze-Direct   경기장 (dice_mission) 에서 경로 (course.yaml) 웨이포인트 따라가기
"""

import gymnasium as gym

from . import agents

gym.register(
    id="Waffle-DriveFlat-Direct",
    entry_point=f"{__name__}.flat_env:WaffleDriveFlatEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.flat_env_cfg:WaffleDriveFlatEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:WaffleDriveFlatPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

gym.register(
    id="Waffle-DriveMaze-Direct",
    entry_point=f"{__name__}.maze_env:WaffleDriveMazeEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.maze_env_cfg:WaffleDriveMazeEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:WaffleDriveMazePPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)
