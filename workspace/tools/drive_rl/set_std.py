"""RSL-RL 체크포인트의 탐색 폭 (행동 표준편차) 을 바꾼 사본을 만든다 (이어서 학습할 때 다시 탐색하게). run.sh maze_start 가 부름.

  python set_std.py 원본.pt 사본.pt 0.3

학습이 오래되면 탐색 폭이 작아져 (밤새 학습 0.035) 새 움직임을 거의 시도하지 않는다. 신경망 가중치 · 관측 정규화는 그대로 두고
actor 의 distribution.std_param (std_type "scalar" = 표준편차 그대로) 만 바꾼다.
"""

import sys

import torch

if __name__ == "__main__":
    src, dst, std = sys.argv[1], sys.argv[2], float(sys.argv[3])
    ck = torch.load(src, map_location="cpu", weights_only=False)
    p = ck["actor_state_dict"]["distribution.std_param"]
    print(f"탐색 폭 {[round(v, 3) for v in p.tolist()]} → {std}")
    ck["actor_state_dict"]["distribution.std_param"] = torch.full_like(p, std)
    torch.save(ck, dst)
