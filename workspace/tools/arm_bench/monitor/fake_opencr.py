#!/usr/bin/env python3
"""가짜 팔 OpenCR: 가상 시리얼 (pty) 로 v5 STATE 를 100 Hz 로 보낸다 - 보드 없이 모니터 · 슬라이더 · RViz2 를 시험할 때.

    python3 monitor/fake_opencr.py [--link /tmp/arm_bench_fake_opencr] [--start 대기]     (보통은 launch 의 fake:=true 가 실행)

- --link 경로에 가상 포트 심볼릭 링크를 만든다 (프로그램은 이 경로를 포트로 연다).
- CMD 가 없으면 관절을 천천히 흔든다 (--start sway, 기본) 또는 실물 대기 자세에 가만히 있다 (--start 대기).
- CMD 를 받으면 펌웨어 안전 규칙을 간단히 흉내 낸다 (safety_core.h): config.h 범위 밖 CMD 거부 · 200 ms 무수신 = CMD_TIMEOUT ·
  TORQUE_ON 처음 = 현재 위치에서 HOLD · REARM 상승 에지 = HOLD 해제 · ESTOP = HOLD · TORQUE_ON 없는 CMD 50 번 + HOLD = 토크 OFF.
  토크가 켜져 있고 HOLD 가 아니면 팔 칸 목표를 따라 움직인다 (관절마다 최대 3 rad/s, 1 차 지연).
- 그리퍼 막힘 보호 (펌웨어 safety_core.h GripGuard, config.h GRIP_*) 도 같은 규칙으로: 막힌 채 stall_ms → 목표 = 막힌 자리 + squeeze,
  ESTOP · 토크 OFF 요청 HOLD 때 쥐는 중이면 그리퍼 목표 유지. --grip-block 으로 퍽을 쥔 것처럼 막는다.
"""
import argparse
import math
import os
import pty
import signal
import struct
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "..", "..", "ros2_ws", "src", "opencr_hardware", "scripts"))   # Lv2 벤치 도구
import opencr_arm_tool as tool  # noqa: E402

import gripper_map  # noqa: E402

AMP = [0.8, 0.5, 0.6, 0.7, 1.2, 0.5]      # [rad] joint1~4 · 손목 롤 · 그리퍼 모터 (sway)
PERIOD = [8.0, 6.0, 7.0, 5.0, 9.0, 4.0]   # [s]
STANDBY = [(t - 2048) * 2 * math.pi / 4096 for t in (1008, 691, 3057, 835, 2039, 2650)]   # 2026-10-27 실물 대기 자세 (그리퍼 다시 결합 뒤 닫힘 2650)
TYPE_CMD = 0x01
TORQUE_OFF_CMDS = 50
TIMEOUT_S = 0.2
GRIP = 5


class GripGuard:
    """펌웨어 safety_core.h GripGuard 와 같은 규칙 (tick)"""

    def __init__(self, p, lo, hi):
        self.p, self.lo, self.hi = p, min(lo, hi), max(lo, hi)
        self.latched = self.watching = False
        self.dir = 1
        self.hold = self.anchor = self.since = 0

    def reset(self):
        self.latched = self.watching = False

    def update(self, goal, pos, now):
        p = self.p
        if self.latched:
            if (goal - self.hold) * self.dir >= 0:
                return self.hold
            self.latched = False
        err = goal - pos
        if abs(err) <= p["stall"]:
            self.watching = False
            return goal
        if not self.watching or abs(pos - self.anchor) > p["progress"]:
            self.watching, self.anchor, self.since = True, pos, now
            return goal
        if now - self.since < p["stall_ms"] / 1000:
            return goal
        self.dir = 1 if err > 0 else -1
        self.hold = min(self.hi, max(self.lo, pos + self.dir * p["squeeze"]))
        self.latched, self.watching = True, False
        return self.hold


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--link", default="/tmp/arm_bench_fake_opencr")
    ap.add_argument("--start", choices=["sway", "대기"], default="sway")
    ap.add_argument("--grip-block", type=int, default=None, help="그리퍼가 이 tick 보다 더 닫히지 못함 (퍽을 쥔 것 흉내)")
    args = ap.parse_args()
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))      # launch 가 끝낼 때도 아래 finally 로 링크를 지움
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    if os.path.lexists(args.link):
        os.remove(args.link)
    os.symlink(os.ttyname(slave), args.link)
    print(f"[fake_opencr] 가상 포트 {args.link} -> {os.ttyname(slave)} (팔 보드 role 2, v{tool.PROTOCOL_VERSION})", flush=True)
    lo, hi = tool.load_limits()
    tick = lambda r: 2048 + r / gripper_map.RAD_PER_TICK                    # noqa: E731
    rad = lambda t: (t - 2048) * gripper_map.RAD_PER_TICK                   # noqa: E731
    grip_p = gripper_map.firmware_grip()
    guard = GripGuard(grip_p, round(tick(lo[GRIP])), round(tick(hi[GRIP])))
    print(f"[fake_opencr] 그리퍼 막힘 보호 (config.h): {grip_p}", flush=True)
    parser = tool.Parser()
    pos = list(STANDBY)
    target = list(pos)
    torque = hold = False
    commanded = False                  # CMD 를 한 번이라도 받았으면 sway 를 멈춤
    last_cmd, prev_rearm, off_run = -1.0, False, 0
    t0, t_prev, seq = time.monotonic(), time.monotonic(), 0
    try:
        while True:
            now = time.monotonic()
            dt, t_prev = now - t_prev, now
            try:
                data = os.read(master, 4096)
            except (BlockingIOError, OSError):
                data = b""
            for b in data:
                r = parser.push(b)
                if not r or r[0] != TYPE_CMD or len(r[1]) != struct.calcsize(tool.CMD_FMT):
                    continue
                v = struct.unpack(tool.CMD_FMT, r[1])
                arm, flags = v[7:13], v[14]
                if any(not (lo[i] <= arm[i] <= hi[i]) for i in range(6)):
                    continue                                    # 범위 밖 CMD 거부
                commanded, last_cmd = True, now
                def stop_hold():                                # 펌웨어 holdArmAtPresent(true): 쥐는 중이면 그리퍼 목표 유지
                    t = list(pos)
                    if abs(tick(target[GRIP]) - tick(pos[GRIP])) <= grip_p["stall"]:
                        t[GRIP] = target[GRIP]
                    return t
                if flags & tool.FLAG_TORQUE_ON:
                    off_run = 0
                    if not torque:
                        torque, hold, target = True, True, list(pos)
                        guard.reset()
                else:
                    off_run += 1
                    if torque and not hold:
                        hold, target = True, stop_hold()
                    if torque and hold and off_run >= TORQUE_OFF_CMDS:
                        torque = False
                        guard.reset()
                if flags & tool.FLAG_ESTOP and torque and not hold:
                    hold, target = True, stop_hold()
                elif flags & tool.FLAG_ESTOP and torque:
                    hold = True
                rearm = bool(flags & tool.FLAG_REARM)
                if rearm and not prev_rearm and hold and torque and not flags & tool.FLAG_ESTOP:
                    hold = False
                prev_rearm = rearm
                if torque and not hold:
                    target = list(arm)
            timeout = now - last_cmd > TIMEOUT_S
            if timeout and torque and not hold:
                hold, target = True, list(pos)
            if not commanded and args.start == "sway":
                t = now - t0
                pos = [AMP[i] * math.sin(2 * math.pi * t / PERIOD[i]) for i in range(6)]
            elif torque:
                target[GRIP] = rad(guard.update(round(tick(target[GRIP])), round(tick(pos[GRIP])), now))
                for i in range(6):
                    d = target[i] - pos[i]
                    pos[i] += max(-3.0 * dt, min(3.0 * dt, d * min(1.0, 20.0 * dt)))
            if args.grip_block is not None:                     # 퍽이 손가락을 막음 (토크와 상관없이)
                pos[5] = min(pos[5], (args.grip_block - 2048) * 2 * math.pi / 4096)
            err = (0x02 if timeout else 0) | (0x40 if hold else 0)
            full = [0.0] * 12
            full[tool.ARM0:tool.ARM0 + 6] = pos
            cur = [0.0] * 12
            if torque:
                cur[tool.ARM0:tool.ARM0 + 6] = [0.05 + 2.0 * abs(target[i] - pos[i]) for i in range(6)]
            payload = struct.pack(tool.STATE_FMT, tool.ROLE_ARM, *full, *([0.0] * 12), *cur, 1.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 9.81, 12.0, err, tool.PROTOCOL_VERSION, seq & 0xFF)
            try:
                os.write(master, tool.frame(tool.TYPE_STATE, payload))
            except (BlockingIOError, OSError):
                pass
            seq += 1
            time.sleep(0.01)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if os.path.islink(args.link):
            os.remove(args.link)


if __name__ == "__main__":
    main()
