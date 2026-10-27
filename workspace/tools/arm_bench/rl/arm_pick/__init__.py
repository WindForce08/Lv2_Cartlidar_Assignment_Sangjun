"""arm_bench 팔 강화학습 패키지 (Isaac Lab 3.0). import 하면 과제가 gym 에 등록된다 (진입점은 문자열 - 앱 실행 전에 import 해도 됨).

  ArmBench-PickPlace : 받침대에 고정한 팔로 테이블 위 퍽 (원기둥 Ø30 x 60 mm) 을 위에서 집어 목표 자리에 세워 놓기
"""

import gymnasium as gym

from . import agents

TASK = "ArmBench-PickPlace"

gym.register(
    id=TASK,
    entry_point=f"{__name__}.pick_env:ArmPickEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.pick_env_cfg:ArmPickEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:ArmPickPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)
