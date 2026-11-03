"""주행 강화학습 학습 · 재생 (Isaac Lab 3.0 RSL-RL 진입점을 그대로 쓰고 과제 등록만 먼저). run.sh flat / flat_play 로 부름.

train.py 와 play.py 는 내용이 같다. 파일 이름이 play 로 시작하면 재생 (팀 rl/train.py 와 같은 방식).
로그 · 체크포인트: logs/rsl_rl/<experiment_name>/<시각>/ (실행한 폴더 기준 logs/, run.sh 는 이 폴더에서 실행)
"""

import os
import sys

import warp as wp

wp.config.enable_backward = False

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import waffle_drive  # noqa: E402,F401  (gym 등록)

from isaaclab_rl.entrypoints import run_play_cli, run_train_cli  # noqa: E402

if __name__ == "__main__":
    argv = sys.argv[1:]
    if "--rl_library" not in argv:
        argv = ["--rl_library", "rsl_rl"] + argv
    play = os.path.basename(sys.argv[0]).startswith("play")
    raise SystemExit((run_play_cli if play else run_train_cli)(argv))
