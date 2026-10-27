#!/usr/bin/env python3
"""opencr_arm_tool.py — 팔 OpenCR (역할 2) 한 대를 ros2_control 없이 시험하는 도구 (Firmware 팀). pyserial 필요.

  monitor  STATE 만 읽는다. 포트에 한 바이트도 쓰지 않는다.
           토크가 꺼진 상태에서 관절을 손으로 돌려 각도 부호(ARM_DIR)를 확인할 때도 쓴다.
  move     v5 CMD 를 100 Hz 로 보내 토크 ON → HOLD 확인 → REARM → 목표를 --speed 이하로 천천히 바꾼다.
             --joint J --delta D   관절 J(1~5)를 현재 위치에서 D rad 움직였다가 제자리로
             --home                관절을 --order 순서로 하나씩 0 rad 로 (1~5)
           6 번째 칸 (실물은 그리퍼 개폐) 은 움직이지 않고 시작 때 위치를 유지한다.
           --dry-run 이면 현재 위치만 읽고 계획을 출력한다 (쓰기 없음). 실제로 움직이려면 --yes.
           오류 비트 · 예상 밖 HOLD · 추종 오차 · 목표 미도달 (머문 뒤 --max-final 초과) · Ctrl-C → ESTOP 프레임을 보낸 뒤 송신을 멈춘다.
           구간마다 관절별 최대 전류 [A] 를 출력한다.
           끝나면 송신을 멈춘다 → 펌웨어가 200 ms 뒤 HOLD (토크는 켜진 채 마지막 목표에서 버팀).
           토크를 끄려면 팔을 받친 뒤 모터 전원을 끈다 (물리 E-stop 회로는 없다).

프로토콜은 opencr_firmware/opencr_protocol.h (v5) 와 같아야 한다. 관절 한계는 opencr_firmware/config.h 에서 읽는다.
"""
import argparse
import re
import struct
import sys
import time
from pathlib import Path

import serial

TYPE_CMD = 0x01
TYPE_STATE = 0x81
PROTOCOL_VERSION = 5
ROLE_ARM = 2
FLAG_TORQUE_ON = 0x01
FLAG_ESTOP = 0x02
FLAG_REARM = 0x08
ERR_NAMES = [(0x01, 'SYNCREAD'), (0x02, 'CMD_TIMEOUT'), (0x04, 'HW_ESTOP'), (0x08, 'MOTOR_HW'),
             (0x10, 'MOTOR_CONFIG'), (0x20, 'ROLE_MISMATCH'), (0x40, 'HOLD'), (0x80, 'FAULT')]
ERR_ABORT = 0x01 | 0x08 | 0x10 | 0x20 | 0x80   # 이 비트가 보이면 움직이지 않는다
ERR_HOLD = 0x40
ERR_TIMEOUT = 0x02
CMD_FMT = '<B6f6ffBB'                 # role · wheel_vel[6] · arm_pos[6] · gripper_pos · flags · seq = 55 B
STATE_FMT = '<B12f12f12f4f3f3ffBBB'   # role · pos · vel · effort · quat · gyro · accel · battery · error · fw · seq = 192 B
MAX_PAYLOAD = 192
ARM0 = 6                              # STATE 의 팔 슬롯 시작
MOVABLE = 5                           # move 가 움직이는 관절 1~5 (6 번째 칸 = 그리퍼 개폐, 현재 위치 유지)
PERIOD = 0.01                         # 100 Hz
CONFIG_H = Path(__file__).resolve().parents[2] / 'opencr_firmware' / 'config.h'


def crc16(data):
    """Dynamixel 2.0 CRC-16 (CRC-16/BUYPASS: 다항식 0x8005, 초기값 0)."""
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(type_, payload):
    body = b'\xff\xfd' + bytes([type_, len(payload)]) + payload
    c = crc16(body)
    return body + bytes([c & 0xFF, c >> 8])


def encode_cmd(arm, flags, seq):
    return frame(TYPE_CMD, struct.pack(CMD_FMT, ROLE_ARM, *([0.0] * 6), *arm, 0.0, flags, seq & 0xFF))


def decode_state(pl):
    v = struct.unpack(STATE_FMT, pl)
    return {'role': v[0], 'pos': v[1:13], 'vel': v[13:25], 'effort': v[25:37], 'battery': v[47],
            'error': v[48], 'fw': v[49], 'seq': v[50]}


def err_text(e):
    return '|'.join(n for b, n in ERR_NAMES if e & b) or '-'


def self_check():
    assert struct.calcsize(CMD_FMT) == 55 and struct.calcsize(STATE_FMT) == 192
    assert crc16(bytes([0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01])) == 0x4E19


class Parser:
    """opencr_protocol.h FrameParser 와 같은 상태 기계."""

    def __init__(self):
        self.state, self.buf, self.need = 0, bytearray(), 0
        self.bad = 0

    def push(self, b):
        s = self.state
        if s == 0:
            if b == 0xFF:
                self.buf = bytearray([b])
                self.state = 1
        elif s == 1:
            if b == 0xFD:
                self.buf.append(b)
                self.state = 2
            elif b != 0xFF:
                self.state = 0
        elif s == 2:
            self.buf.append(b)
            self.state = 3
        elif s == 3:
            if b > MAX_PAYLOAD:
                self.state, self.bad = 0, self.bad + 1
                return None
            self.buf.append(b)
            self.need = b + 2
            self.state = 4
        else:
            self.buf.append(b)
            self.need -= 1
            if self.need == 0:
                self.state = 0
                body, rx = bytes(self.buf[:-2]), self.buf[-2] | (self.buf[-1] << 8)
                if rx != crc16(body):
                    self.bad += 1
                    return None
                return body[2], body[4:]
        return None


class Link:
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0)
        self.parser = Parser()
        self.last = None
        self.count = 0

    def poll(self):
        for b in self.ser.read(4096):
            r = self.parser.push(b)
            if r and r[0] == TYPE_STATE and len(r[1]) == MAX_PAYLOAD:
                self.last = decode_state(r[1])
                self.count += 1
        return self.last

    def wait_state(self, timeout=2.0):
        t_end = time.monotonic() + timeout
        n0 = self.count
        while time.monotonic() < t_end:
            self.poll()
            if self.count > n0:
                return self.last
            time.sleep(0.005)
        return None


def load_limits():
    text = CONFIG_H.read_text()

    def arr(name):
        m = re.search(name + r'\[ARM_COUNT\]\s*=\s*\{([^}]*)\}', text)
        return [float(x.strip().rstrip('f')) for x in m.group(1).split(',')]
    return arr('ARM_MIN_RAD'), arr('ARM_MAX_RAD')


def arm_pos(st):
    return list(st['pos'][ARM0:ARM0 + 6])


def check_identity(st):
    if st is None:
        sys.exit('STATE 없음 — 포트 · 전원 확인')
    if st['role'] != ROLE_ARM or st['fw'] != PROTOCOL_VERSION:
        sys.exit(f"팔 보드 v{PROTOCOL_VERSION} 가 아님: role={st['role']} fw={st['fw']}")


def monitor(args):
    link = Link(args.port)   # 이 함수는 write 를 부르지 않는다
    t_end = time.monotonic() + args.seconds
    t_print, n_print = time.monotonic(), link.count
    seq_prev, seq_gaps = None, 0
    while time.monotonic() < t_end:
        st = link.poll()
        if st is not None and st['seq'] != seq_prev:
            if seq_prev is not None and (st['seq'] - seq_prev) & 0xFF != 1:
                seq_gaps += 1
            seq_prev = st['seq']
        now = time.monotonic()
        if now - t_print >= 0.5:
            hz = (link.count - n_print) / (now - t_print)
            t_print, n_print = now, link.count
            if st is None:
                print('STATE 없음')
            else:
                deg = ' '.join(f'{q * 57.2958:7.1f}' for q in arm_pos(st))
                print(f"{hz:5.1f} Hz role={st['role']} fw={st['fw']} seq={st['seq']:3d} gaps={seq_gaps} "
                      f"err=0x{st['error']:02X} {err_text(st['error']):28s} arm[deg] {deg}")
        time.sleep(0.002)
    if link.last is None:
        sys.exit('STATE 를 한 번도 받지 못함')
    check_identity(link.last)


def plan_targets(args, q0):
    if args.home:
        order = [int(j) - 1 for j in args.order.split(',')]
        q, out = list(q0), []
        for j in order:
            q = list(q)
            q[j] = 0.0
            out.append((f'관절 {j + 1} → 0', q))
        return out
    j = args.joint - 1
    q1 = list(q0)
    q1[j] += args.delta
    return [(f'관절 {args.joint} {args.delta:+.3f} rad', q1), (f'관절 {args.joint} 제자리', list(q0))]


def move(args):
    lo, hi = load_limits()
    link = Link(args.port)
    st = link.wait_state()
    check_identity(st)
    if st['error'] & ERR_ABORT:
        sys.exit(f"움직이지 않음: err={err_text(st['error'])}")
    q0 = arm_pos(st)
    plan = plan_targets(args, q0)
    print('현재 [rad]', ' '.join(f'{q:+.3f}' for q in q0))
    for name, q in plan:
        bad = [i + 1 for i in range(6) if not lo[i] <= q[i] <= hi[i]]
        print(f'  {name:20s}', ' '.join(f'{x:+.3f}' for x in q), f'한계 밖 관절 {bad}' if bad else '')
        if bad:
            sys.exit('관절 한계 (config.h) 밖 목표 — 중단')
    if args.dry_run or not args.yes:
        print('쓰기 없음 (--yes 를 주면 실제로 움직임)')
        return
    # 이전 세션 (다른 실행 · 호스트) 의 명령이 끝나 펌웨어가 무수신이 될 때까지 기다린다 (읽기만).
    # 그래야 새 seq 를 기준으로 받고, 토크가 켜져 있으면 HOLD 에서 시작해 REARM 이 필요해진다 (호스트 wait_first_state 와 같은 규칙)
    t_end = time.monotonic() + 2.0
    while not (link.poll() or st)['error'] & ERR_TIMEOUT:
        if time.monotonic() > t_end:
            sys.exit('펌웨어가 무수신 상태가 되지 않음 — 다른 프로그램이 CMD 를 보내는지 확인')
        time.sleep(0.01)

    seq = 0
    cmd = list(q0)

    def send(flags):
        nonlocal seq
        link.ser.write(encode_cmd(cmd, flags, seq))
        seq += 1

    def cycle(flags, pred=None, timeout=1.0):
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            t0 = time.monotonic()
            send(flags)
            s = link.poll()
            if pred is not None and s is not None and pred(s):
                return s
            time.sleep(max(0.0, PERIOD - (time.monotonic() - t0)))
        return None

    def abort(why):
        print('중단:', why, '— ESTOP 프레임 후 송신 중단 (펌웨어는 현재 위치에서 HOLD, 토크 유지)')
        cycle(FLAG_TORQUE_ON | FLAG_ESTOP, timeout=0.3)
        sys.exit(1)

    try:
        # 1) 토크 ON — 펌웨어는 현재 위치를 목표로 잡고 HOLD
        s = cycle(FLAG_TORQUE_ON, lambda s: s['error'] & ERR_HOLD and not s['error'] & ERR_TIMEOUT, timeout=2.0)
        if s is None or s['error'] & ERR_ABORT:
            abort('토크 ON 뒤 HOLD 를 보지 못함' if s is None else err_text(s['error']))
        # 2) REARM 펄스 (0 → 1 → 0) → HOLD 해제 확인
        cycle(FLAG_TORQUE_ON | FLAG_REARM, timeout=0.03)
        s = cycle(FLAG_TORQUE_ON, lambda s: not s['error'] & ERR_HOLD, timeout=1.0)
        if s is None:
            abort('REARM 뒤에도 HOLD')
        # 3) 목표를 --speed 이하로 바꾼다
        step = args.speed * PERIOD
        for name, goal in plan:
            print('→', name)
            peak = [0.0] * 6

            def track(s):
                for i, a in enumerate(s['effort'][ARM0:ARM0 + 6]):
                    peak[i] = max(peak[i], abs(a))
                return False

            while True:
                t0 = time.monotonic()
                done = True
                for i in range(6):
                    d = goal[i] - cmd[i]
                    if abs(d) > step:
                        cmd[i] += step if d > 0 else -step
                        done = False
                    else:
                        cmd[i] = goal[i]
                send(FLAG_TORQUE_ON)
                s = link.poll()
                if s is not None:
                    track(s)
                    if s['error'] & (ERR_ABORT | ERR_HOLD):
                        abort(err_text(s['error']))
                    worst = max(abs(a - b) for a, b in zip(arm_pos(s), cmd))
                    if worst > args.max_track:
                        abort(f'추종 오차 {worst:.3f} rad > {args.max_track}')
                if done:
                    break
                time.sleep(max(0.0, PERIOD - (time.monotonic() - t0)))
            cycle(FLAG_TORQUE_ON, track, timeout=args.dwell)
            s = link.last
            reached = arm_pos(s)
            print('  도달 [rad]', ' '.join(f'{q:+.3f}' for q in reached), 'err', err_text(s['error']))
            print('  최대 전류 [A]', ' '.join(f'{a:.3f}' for a in peak))
            # 막힘 · 힘 부족으로 추종 오차 한계 안에서 멈춘 경우: 계속 밀지 않고 현재 위치에서 멈춘다
            miss = [i + 1 for i in range(6) if abs(reached[i] - goal[i]) > args.max_final]
            if miss:
                abort(f'목표 미도달 관절 {miss} (> {args.max_final} rad)')
    except KeyboardInterrupt:
        abort('Ctrl-C')
    print('완료 — 송신을 멈춥니다. 펌웨어는 200 ms 뒤 HOLD (토크 유지). 끄려면 팔을 받치고 모터 전원을 끄세요.')


def main():
    self_check()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    m = sub.add_parser('monitor', help='STATE 만 읽음 (쓰기 없음)')
    m.add_argument('--port', required=True)
    m.add_argument('--seconds', type=float, default=10.0)
    v = sub.add_parser('move', help='천천히 움직임 (--yes 필요)')
    v.add_argument('--port', required=True)
    g = v.add_mutually_exclusive_group(required=True)
    # 6 번째 칸은 실물에서 그리퍼 개폐 (config.h 벤치 임시 매핑) — 움직이지 않고 현재 위치만 유지
    g.add_argument('--joint', type=int, choices=range(1, MOVABLE + 1))
    g.add_argument('--home', action='store_true')
    v.add_argument('--delta', type=float, default=0.1, help='--joint 이동량 rad (기본 0.1)')
    v.add_argument('--order', default='1,2,3,4,5', help='--home 관절 순서 (1~5)')
    v.add_argument('--speed', type=float, default=0.1, help='목표 변화 속도 상한 rad/s (기본 0.1)')
    v.add_argument('--max-track', type=float, default=0.2, help='허용 추종 오차 rad (기본 0.2 ★)')
    v.add_argument('--max-final', type=float, default=0.02, help='머문 뒤 목표 도달 허용 오차 rad (기본 0.02 ★)')
    v.add_argument('--dwell', type=float, default=1.0, help='목표 도달 뒤 유지 s')
    v.add_argument('--dry-run', action='store_true')
    v.add_argument('--yes', action='store_true', help='실제로 토크를 켜고 움직임')
    args = ap.parse_args()
    if args.cmd == 'move' and not 0 < args.speed <= 0.5:
        sys.exit('--speed 는 0 초과 0.5 rad/s 이하')
    if args.cmd == 'move' and args.home:
        try:
            order = [int(j) for j in args.order.split(',')]
        except ValueError:
            sys.exit('--order 는 쉼표로 나눈 관절 번호')
        if not order or any(not 1 <= j <= MOVABLE for j in order):
            sys.exit(f'--order 는 관절 1~{MOVABLE} 만 (6 번째 칸 = 그리퍼, 움직이지 않음)')
    (monitor if args.cmd == 'monitor' else move)(args)


if __name__ == '__main__':
    main()
