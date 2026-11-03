"""base_cmd 시험 (Isaac 없이). Isaac Lab venv 의 python 으로 그냥 실행한다: ./run.sh test (venv 에 pytest 없음)."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import torch  # noqa: E402

from waffle_drive.base_cmd import DD_AX, SKILL_VX, SKILL_WZ, BaseCmd  # noqa: E402

R, SEP, WMAX, DT = 0.033, 0.317, 10.995574, 1 / 120
CHECKS = []


def check(name):
    def wrap(f):
        CHECKS.append((name, f))
        return f
    return wrap


def cmd(n=1, mode="twist"):
    return BaseCmd(n, "cpu", R, SEP, WMAX, DT, mode)


def run(b, a, steps):
    out = None
    for _ in range(steps):
        out = b.step(torch.tensor([a], dtype=torch.float32))
    return out


@check("직진: vx 가 물리 스텝마다 1.5/120 씩 올라 24 스텝 (0.2 s) 에 0.3 m/s, 그 뒤 유지")
def _():
    b = cmd()
    reached = None
    for i in range(1, 121):
        wl, wr = b.step(torch.tensor([[1.0, 0.0]]))
        assert abs(b.vx.item() - min(DD_AX * DT * i, SKILL_VX)) < 1e-5, (i, b.vx.item())
        if reached is None and b.vx.item() >= SKILL_VX - 1e-5:
            reached = i
    assert reached == 24, reached
    assert abs(wl.item() - SKILL_VX / R) < 1e-4 and abs(wr.item() - SKILL_VX / R) < 1e-4   # 9.09 rad/s, 축소 없음


@check("감속: 0.3 m/s 에서 행동 0 이면 24 스텝에 0")
def _():
    b = cmd()
    run(b, [1.0, 0.0], 60)
    for i in range(1, 25):
        b.step(torch.tensor([[0.0, 0.0]]))
        assert abs(b.vx.item() - max(SKILL_VX - DD_AX * DT * i, 0.0)) < 1e-5, (i, b.vx.item())
    assert abs(b.vx.item()) < 1e-6


@check("제자리 회전: wz 1.5 → 바퀴 ∓7.20 rad/s (한계 안), vx 0")
def _():
    wl, wr = run(cmd(), [0.0, 1.0], 5)
    w = SKILL_WZ * SEP / 2 / R
    assert abs(wl.item() + w) < 1e-4 and abs(wr.item() - w) < 1e-4, (wl.item(), wr.item())


@check("직진 + 회전 (1, 1): 바깥 바퀴 16.3 → 10.996 으로 비례 축소, 좌우 비율 (= 회전 반경) 유지")
def _():
    wl, wr = run(cmd(), [1.0, 1.0], 60)
    raw_l, raw_r = (SKILL_VX - SKILL_WZ * SEP / 2) / R, (SKILL_VX + SKILL_WZ * SEP / 2) / R
    assert raw_r > WMAX
    assert abs(max(abs(wl.item()), abs(wr.item())) - WMAX) < 1e-4, (wl.item(), wr.item())
    assert abs(wl.item() / wr.item() - raw_l / raw_r) < 1e-5


@check("범위 밖 행동 (5, -5) 는 (1, -1) 과 같음")
def _():
    a = run(cmd(), [5.0, -5.0], 40)
    b = run(cmd(), [1.0, -1.0], 40)
    assert torch.allclose(a[0], b[0]) and torch.allclose(a[1], b[1])


@check("바퀴 모드: 행동 × 바퀴 한계 그대로")
def _():
    wl, wr = run(cmd(mode="wheels"), [0.5, -1.0], 1)
    assert abs(wl.item() - 0.5 * WMAX) < 1e-4 and abs(wr.item() + WMAX) < 1e-4


@check("환경마다 따로: reset 한 환경만 vx 0, 다른 행동은 서로 섞이지 않음")
def _():
    b = cmd(2)
    for _ in range(60):
        b.step(torch.tensor([[1.0, 0.0], [-1.0, 0.0]]))
    assert abs(b.vx[0].item() - SKILL_VX) < 1e-5 and abs(b.vx[1].item() + SKILL_VX) < 1e-5
    b.reset([0])
    assert b.vx[0].item() == 0.0 and abs(b.vx[1].item() + SKILL_VX) < 1e-5


@check("잘못된 action_mode 는 거부")
def _():
    try:
        cmd(mode="velocity")
    except ValueError:
        return
    raise AssertionError("거부하지 않음")


if __name__ == "__main__":
    failed = 0
    for name, f in CHECKS:
        try:
            f()
            print(f"통과  {name}")
        except AssertionError as e:
            failed += 1
            print(f"실패  {name}: {e}")
    print(f"base_cmd 시험 {len(CHECKS)} 개 중 {len(CHECKS) - failed} 개 통과")
    sys.exit(1 if failed else 0)
