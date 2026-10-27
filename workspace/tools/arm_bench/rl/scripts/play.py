"""학습 (Isaac Lab 3.0 RSL-RL 학습 진입점을 그대로 씀, 과제 등록만 먼저). play.py 도 이 파일과 같음 (이름으로 구분).

    ./run.sh train [환경 수] [반복] [덮어쓰기...]    예: ./run.sh train 256 1500 env.episode_length_s=12
    ./run.sh play  [환경 수]                          창으로 재생 (화면 필요)

로그·체크포인트: logs/rsl_rl/arm_pick/<시각>/ (model_*.pt, params/env.yaml = 그때 설정 전체)
"""

import os
import sys

import _common  # noqa: F401

import warp as wp

wp.config.enable_backward = False

import arm_pick  # noqa: E402,F401  (gym 등록)

from isaaclab_rl.entrypoints import run_play_cli, run_train_cli  # noqa: E402

if __name__ == "__main__":
    argv = sys.argv[1:]
    if "--rl_library" not in argv:
        argv = ["--rl_library", "rsl_rl"] + argv
    play = os.path.basename(sys.argv[0]).startswith("play")
    raise SystemExit((run_play_cli if play else run_train_cli)(argv))
