"""스크립트 공통: rl 폴더를 import 경로에 넣고 작업 폴더를 맞춤 (학습 로그는 rl/logs/rsl_rl/)."""

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # workspace/tools/arm_bench/rl
BENCH = os.path.dirname(ROOT)                                        # workspace/tools/arm_bench
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
os.chdir(ROOT)
RESULTS = os.path.join(ROOT, "results")
