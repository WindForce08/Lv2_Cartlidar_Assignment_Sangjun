#!/usr/bin/env python3
"""그리퍼 막힘 보호 (펌웨어 GripGuard) 실물 시험 - 장치 쓰기: 토크 ON · 그리퍼 이동 (승인 뒤에만).

    python3 tools/grip_test.py empty [--port /dev/ttyACM0]   빈 손: 열기 → 닫기 → 열기 (끝에서 끝까지 한 번에 명령) → 연 채 HOLD
    python3 tools/grip_test.py puck  [--port /dev/ttyACM0]   손가락 사이에 퍽을 넣은 뒤: 닫기 (3 s) → ESTOP (1.5 s) → 송신 중단 (HOLD)
    python3 tools/grip_test.py release [--port ...]           그리퍼 열기 (1.5 s) → 토크 끄기 (TORQUE_ON 없는 CMD 1 s - 펌웨어 D5-4: HOLD + 50 번 연속)
                                                              ⚠ 토크가 꺼지면 팔이 받쳐지지 않음 - 팔을 받치거나 스스로 서 있는 자세에서만

- 팔 관절 1~5 는 토크를 켠 순간 (펌웨어 HOLD) 의 위치를 그대로 명령 (움직이지 않음). 그리퍼 (6 번째 칸) 만 계단 명령.
- 슬라이더 (arm_control) 는 쓰지 않는다 - 슬라이더에도 약하게 쥐는 규칙이 있어 펌웨어가 한 일과 섞임.
- 중단: 중단 오류 비트 · 예상 밖 HOLD · 팔 관절 오차 > 0.15 rad · STATE 0.3 s 없음 → ESTOP 프레임 후 송신 중단 (펌웨어 HOLD, 토크 유지).
- 끝나면 송신을 멈춘다 → 펌웨어가 200 ms 뒤 HOLD (토크 유지, 마지막 목표). 토크 OFF 는 release (팔을 받친 뒤).
  토크를 끄는 동안 팔 관절이 0.15 rad 넘게 움직이면 (떨어짐) ESTOP + TORQUE_ON 을 보내 펌웨어가 그 자리에서 다시 잡는다.
- 기록: logs/grip/<empty|puck>_<시각>.csv (100 Hz: 시간 · 단계 · 오류 · 그리퍼 명령 · 위치 [tick] · 전류 [A] · 팔 관절 최대 오차)
"""
import argparse
import csv
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "..", "..", "ros2_ws", "src", "opencr_hardware", "scripts"))   # Lv2 벤치 도구
sys.path.insert(0, os.path.join(ROOT, "monitor"))
import opencr_arm_tool as tool  # noqa: E402

import gripper_map  # noqa: E402

GRIP = 5
OPEN, CLOSED = gripper_map.OPEN_TICK, gripper_map.CLOSED_TICK
MAX_ARM_ERR = 0.15


def tick(rad):
    return 2048 + rad / gripper_map.RAD_PER_TICK


def rad(t):
    return (t - 2048) * gripper_map.RAD_PER_TICK


class Run:
    def __init__(self, port, name):
        self.link = tool.Link(port)
        self.seq = 0
        self.cmd = None
        self.phase = "start"
        os.makedirs(os.path.join(ROOT, "logs", "grip"), exist_ok=True)
        self.path = os.path.join(ROOT, "logs", "grip", f"{name}_{time.strftime('%H%M%S')}.csv")
        self.f = open(self.path, "w", newline="")
        self.log = csv.writer(self.f)
        self.log.writerow(["t_s", "phase", "error", "grip_cmd_tick", "grip_pos_tick", "grip_cur_A", "arm_err_max_rad"])
        self.t0 = time.monotonic()
        self.rows = []
        self.last_seen = self.link.count

    def send(self, flags):
        self.link.ser.write(tool.encode_cmd(self.cmd, flags, self.seq))
        self.seq += 1

    def step(self, flags, check=True):
        """CMD 하나 보내고 10 ms 동안 STATE 를 받아 기록. 새 STATE 를 돌려줌"""
        t_start = time.monotonic()
        self.send(flags)
        s = None
        while time.monotonic() - t_start < tool.PERIOD:
            s = self.link.poll() or s
            time.sleep(0.001)
        s = self.link.last
        if s is None or self.link.count == self.last_seen:
            if time.monotonic() - getattr(self, "t_rx", t_start) > 0.3:
                self.abort("STATE 가 0.3 s 넘게 없음")
            return s
        self.last_seen, self.t_rx = self.link.count, time.monotonic()
        q = tool.arm_pos(s)
        arm_err = max(abs(q[i] - self.cmd[i]) for i in range(GRIP)) if self.cmd else 0.0
        row = (time.monotonic() - self.t0, self.phase, s["error"], tick(self.cmd[GRIP]) if self.cmd else 0, tick(q[GRIP]),
               s["effort"][tool.ARM0 + GRIP], arm_err)
        self.rows.append(row)
        self.log.writerow([f"{row[0]:.3f}", row[1], row[2], f"{row[3]:.0f}", f"{row[4]:.0f}", f"{row[5]:.3f}", f"{row[6]:.4f}"])
        if check and s["error"] & tool.ERR_ABORT:
            self.abort(f"오류 비트 {tool.err_text(s['error'])}")
        if check and self.phase not in ("torque_on", "rearm", "토크 끄기") and not self.phase.startswith("ESTOP") and s["error"] & tool.ERR_HOLD:
            self.abort("예상 밖 HOLD")
        if check and arm_err > MAX_ARM_ERR:
            self.abort(f"팔 관절 오차 {arm_err:.3f} rad > {MAX_ARM_ERR}")
        return s

    def abort(self, why):
        print(f"중단: {why} — ESTOP 프레임 후 송신 중단 (펌웨어 HOLD, 토크 유지)")
        self.phase = "abort"
        for _ in range(30):
            self.send(tool.FLAG_TORQUE_ON | tool.FLAG_ESTOP)
            time.sleep(tool.PERIOD)
        self.f.close()
        sys.exit(1)

    def start(self):
        s = self.link.wait_state()
        tool.check_identity(s)
        if s["error"] & tool.ERR_ABORT:
            sys.exit(f"시작 안 함: err={tool.err_text(s['error'])}")
        t_end = time.monotonic() + 2.0                     # 다른 프로그램이 명령 중이 아닌지 (펌웨어 무수신)
        while not (self.link.poll() or s)["error"] & tool.ERR_TIMEOUT:
            if time.monotonic() > t_end:
                sys.exit("펌웨어가 무수신 상태가 되지 않음 — 다른 프로그램이 CMD 를 보내는지 확인")
            time.sleep(0.01)
        self.cmd = tool.arm_pos(self.link.last)
        self.phase = "torque_on"                            # 토크 ON → 펌웨어는 현재 위치에서 HOLD
        ok = False
        for _ in range(200):
            s = self.step(tool.FLAG_TORQUE_ON, check=False)
            if s and s["error"] & tool.ERR_HOLD and not s["error"] & tool.ERR_TIMEOUT:
                ok = True
                break
        if not ok:
            self.abort("토크 ON 뒤 HOLD 를 보지 못함")
        self.cmd = tool.arm_pos(self.link.last)             # HOLD 위치 = 명령 (튀지 않게)
        self.phase = "rearm"
        for _ in range(3):
            self.step(tool.FLAG_TORQUE_ON | tool.FLAG_REARM, check=False)
        for _ in range(100):
            s = self.step(tool.FLAG_TORQUE_ON, check=False)
            if s and not s["error"] & tool.ERR_HOLD:
                break
        else:
            self.abort("REARM 뒤에도 HOLD")
        print(f"토크 ON · REARM 끝. 팔 [deg] {' '.join(f'{v * 57.2958:.1f}' for v in self.cmd[:5])} · 그리퍼 {tick(self.cmd[GRIP]):.0f} tick")

    def grip(self, name, target_tick, seconds, flags=tool.FLAG_TORQUE_ON):
        self.phase = name
        self.cmd[GRIP] = rad(target_tick)
        i0 = len(self.rows)
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            self.step(flags)
        self.summary(name, target_tick, self.rows[i0:])

    def torque_off(self, seconds):
        """TORQUE_ON 없는 CMD 를 seconds 동안 (펌웨어: HOLD 로 들어간 뒤 50 번 연속이면 토크 OFF). 끝에 전류로 확인"""
        self.phase = "토크 끄기"
        i0 = len(self.rows)
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            self.step(0)
        rows = self.rows[i0:]
        tail = [r for r in rows if r[0] - rows[-1][0] > -0.2]
        q = tool.arm_pos(self.link.last)
        cur = self.link.last["effort"][tool.ARM0:tool.ARM0 + 6]
        print(f"  토크 끄기     오류 {tool.err_text(rows[-1][2])} | 마지막 0.2 s 그리퍼 전류 {sum(abs(r[5]) for r in tail) / len(tail):.2f} A | "
              f"관절 전류 [A] {' '.join(f'{abs(c):.2f}' for c in cur)} | 팔 [deg] {' '.join(f'{v * 57.2958:.1f}' for v in q[:5])} "
              f"(명령과 최대 {max(r[6] for r in rows):.3f} rad)")

    def summary(self, name, target, rows):
        if not rows:
            print(f"  {name}: 기록 없음")
            return
        t0 = rows[0][0]
        pos = [r[4] for r in rows]
        cur = [abs(r[5]) for r in rows]
        reach = next((r[0] - t0 for r in rows if abs(r[4] - target) <= 20), None)
        tail = [r for r in rows if r[0] - rows[-1][0] > -0.5]
        print(f"  {name:12s} 명령 {target} tick | 시작 {pos[0]:.0f} → 끝 {pos[-1]:.0f} (명령과 {pos[-1] - target:+.0f}) | "
              f"20 tick 안 도달 {'-' if reach is None else f'{reach:.2f} s'} | 전류 최대 {max(cur):.2f} A · 마지막 0.5 s 평균 "
              f"{sum(abs(r[5]) for r in tail) / len(tail):.2f} A | 오류 {tool.err_text(rows[-1][2])}")

    def finish(self):
        self.f.close()
        after = "토크 OFF 상태로 둠" if self.phase == "토크 끄기" else "펌웨어 200 ms 뒤 HOLD (토크 유지)"
        print(f"송신 중단 → {after}. 기록 {os.path.relpath(self.path, ROOT)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["empty", "puck", "release"])
    ap.add_argument("--port", default="/dev/ttyACM0")
    a = ap.parse_args()
    r = Run(a.port, a.what)
    try:
        r.start()
        if a.what == "empty":
            r.grip("열기", OPEN, 2.5)
            r.grip("닫기 (빈 손)", CLOSED, 2.5)
            r.grip("열기", OPEN, 2.5)
        elif a.what == "release":
            r.grip("열기", OPEN, 1.5)
            r.torque_off(1.0)
        else:
            r.grip("닫기 (퍽)", CLOSED, 3.0)
            # 막힌 시각 (위치가 더 안 변함) 과 전류 변화를 0.1 s 단위로
            rows = [x for x in r.rows if x[1] == "닫기 (퍽)"]
            if rows:
                t0 = rows[0][0]
                print("  0.1 s 마다 (시간 · 위치 tick · 전류 A):", " ".join(f"{x[0] - t0:.1f}:{x[4]:.0f}/{abs(x[5]):.2f}"
                                                               for k, x in enumerate(rows) if k % 10 == 0))
            r.grip("ESTOP (쥔 채)", CLOSED, 1.5, tool.FLAG_TORQUE_ON | tool.FLAG_ESTOP)
    except KeyboardInterrupt:
        r.abort("Ctrl-C")
    r.finish()
    if a.what == "release":
        print("토크 OFF 확인은 위 전류 (0 근처) 로. 다시 켜려면 슬라이더 [시작] 또는 grip_test.py empty")


if __name__ == "__main__":
    main()
