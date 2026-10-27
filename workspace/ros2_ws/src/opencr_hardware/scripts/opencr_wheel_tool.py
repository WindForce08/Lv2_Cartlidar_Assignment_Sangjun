#!/usr/bin/env python3
"""opencr_wheel_tool.py — 주행 OpenCR (역할 1) 바퀴를 ROS 없이 저속으로 돌려 보는 도구 (Firmware 팀). pyserial 필요.

  spin   v5 CMD 를 100 Hz 로 보내 토크 ON → 서서히 가속 → 유지 → 서서히 감속 → 정지 → 토크 OFF.
           --wheel 1~4 | all    1 왼쪽 앞 · 2 왼쪽 뒤 · 3 오른쪽 앞 · 4 오른쪽 뒤 (config.h WHEEL_IDS 순서)
           --speed  바퀴 각속도 rad/s (기본 +1.0 ≈ 9.5 rpm, |값| ≤ 3.0). + = 로봇 앞으로 굴러가는 방향
           --seconds 최고 속도 유지 시간 (기본 2, ≤ 5)
         --yes 가 없으면 계획만 출력하고 쓰지 않는다.
         오류 비트 · Ctrl-C → 속도 0 + ESTOP 프레임 → 토크 OFF 프레임을 보낸 뒤 송신을 멈춘다.

바퀴를 바닥에서 띄운 상태로만 쓴다. 회전 방향은 눈으로 본다 — STATE 의 속도는 WHEEL_DIR 이 이미 적용된 값이라
부호가 명령과 같게 나오는 것이 정상이고, 실제 방향 확인에는 쓸 수 없다.
"""
import argparse
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))   # 같은 폴더의 opencr_arm_tool 과 프로토콜 코드를 공유
from opencr_arm_tool import (CMD_FMT, ERR_ABORT, ERR_TIMEOUT, FLAG_ESTOP, FLAG_TORQUE_ON, PERIOD,
                             PROTOCOL_VERSION, TYPE_CMD, Link, err_text, frame, self_check)

ROLE_WHEEL = 1
WHEEL_COUNT = 4
WHEEL_NAMES = ['왼쪽 앞', '왼쪽 뒤', '오른쪽 앞', '오른쪽 뒤']
MAX_SPEED = 3.0      # 이 도구의 상한 rad/s (펌웨어 한계 7.91 rad/s 보다 훨씬 낮게)
MAX_SECONDS = 5.0
RAMP_S = 0.5         # 가속 · 감속 시간


def encode_cmd(wheels, flags, seq):
    vel = list(wheels) + [0.0] * (6 - len(wheels))
    return frame(TYPE_CMD, struct.pack(CMD_FMT, ROLE_WHEEL, *vel, *([0.0] * 6), 0.0, flags, seq & 0xFF))


def check_identity(st):
    if st is None:
        sys.exit('STATE 없음 — 포트 · 전원 확인')
    if st['role'] != ROLE_WHEEL or st['fw'] != PROTOCOL_VERSION:
        sys.exit(f"주행 보드 v{PROTOCOL_VERSION} 가 아님: role={st['role']} fw={st['fw']}")


def spin(args):
    targets = list(range(WHEEL_COUNT)) if args.wheel == 'all' else [int(args.wheel) - 1]
    goal = [args.speed if i in targets else 0.0 for i in range(WHEEL_COUNT)]
    print('계획:', ', '.join(f'{WHEEL_NAMES[i]}(ID {i + 1})' for i in targets),
          f'{args.speed:+.2f} rad/s, 가속 {RAMP_S}s · 유지 {args.seconds}s · 감속 {RAMP_S}s')
    link = Link(args.port)
    st = link.wait_state()
    check_identity(st)
    print(f"현재 상태: err=0x{st['error']:02X} {err_text(st['error'])}, 배터리 {st['battery']:.1f} V")
    if st['error'] & ERR_ABORT:
        sys.exit(f"움직이지 않음: err={err_text(st['error'])} (RESET · 모터 연결 확인)")
    if not args.yes:
        print('쓰기 없음 (--yes 를 주면 실제로 돌림). 바퀴를 바닥에서 띄웠는지 먼저 확인.')
        return
    # 다른 프로그램이 CMD 를 보내는 중이면 섞이지 않게 무수신이 될 때까지 기다린다 (읽기만)
    t_end = time.monotonic() + 2.0
    while not (link.poll() or st)['error'] & ERR_TIMEOUT:
        if time.monotonic() > t_end:
            sys.exit('펌웨어가 무수신 상태가 되지 않음 — ROS 등 다른 프로그램이 CMD 를 보내는지 확인')
        time.sleep(0.01)

    seq = 0

    def send(wheels, flags):
        nonlocal seq
        link.ser.write(encode_cmd(wheels, flags, seq))
        seq += 1

    def run(duration, flags, wheels_at, check=True):
        """duration 초 동안 100 Hz 로 보낸다. wheels_at(t) → 바퀴 속도 목록."""
        t0 = time.monotonic()
        t_print = 0.0
        while True:
            t = time.monotonic() - t0
            if t >= duration:
                return
            tc = time.monotonic()
            w = wheels_at(t)
            send(w, flags)
            s = link.poll()
            if check and s is not None and s['error'] & ERR_ABORT:
                raise RuntimeError(err_text(s['error']))
            if s is not None and t - t_print >= 0.25:
                t_print = t
                meas = ' '.join(f'{v:+5.2f}' for v in s['vel'][:WHEEL_COUNT])
                cur = ' '.join(f'{a:+5.2f}' for a in s['effort'][:WHEEL_COUNT])
                print(f'  명령 {w[targets[0]]:+5.2f} | 측정 rad/s [{meas}] | 전류 A [{cur}] | {err_text(s["error"])}')
            time.sleep(max(0.0, PERIOD - (time.monotonic() - tc)))

    def stop_and_release(estop):
        flags = FLAG_TORQUE_ON | (FLAG_ESTOP if estop else 0)
        run(0.3, flags, lambda t: [0.0] * WHEEL_COUNT, check=False)
        run(0.3, 0, lambda t: [0.0] * WHEEL_COUNT, check=False)   # TORQUE_ON 없는 CMD → 바퀴 토크 즉시 OFF

    def scaled(k):
        return [g * k for g in goal]

    try:
        print('토크 ON (속도 0)')
        run(0.3, FLAG_TORQUE_ON, lambda t: [0.0] * WHEEL_COUNT)
        print('가속')
        run(RAMP_S, FLAG_TORQUE_ON, lambda t: scaled(t / RAMP_S))
        print('유지 — 바퀴가 앞으로 굴러가는 방향으로 도는지 보세요')
        run(args.seconds, FLAG_TORQUE_ON, lambda t: scaled(1.0))
        print('감속')
        run(RAMP_S, FLAG_TORQUE_ON, lambda t: scaled(max(0.0, 1.0 - t / RAMP_S)))
        stop_and_release(estop=False)
    except KeyboardInterrupt:
        print('중단: Ctrl-C — 정지 후 토크 OFF')
        stop_and_release(estop=True)
        sys.exit(1)
    except RuntimeError as e:
        print(f'중단: {e} — 정지 후 토크 OFF')
        stop_and_release(estop=True)
        sys.exit(1)
    s = link.wait_state(0.5) or link.last
    print(f"완료 — 토크 OFF, 송신 중단. 마지막 상태 err={err_text(s['error'])}")


def main():
    self_check()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('spin', help='바퀴를 저속으로 잠깐 돌림 (--yes 필요)')
    p.add_argument('--port', required=True)
    p.add_argument('--wheel', required=True, choices=['1', '2', '3', '4', 'all'])
    p.add_argument('--speed', type=float, default=1.0, help='바퀴 rad/s (기본 1.0, |값| ≤ 3.0)')
    p.add_argument('--seconds', type=float, default=2.0, help='유지 시간 s (기본 2, ≤ 5)')
    p.add_argument('--yes', action='store_true', help='실제로 토크를 켜고 돌림')
    args = ap.parse_args()
    if not 0 < abs(args.speed) <= MAX_SPEED:
        sys.exit(f'--speed 는 0 이 아니고 |값| ≤ {MAX_SPEED} rad/s')
    if not 0 < args.seconds <= MAX_SECONDS:
        sys.exit(f'--seconds 는 0 초과 {MAX_SECONDS} 이하')
    spin(args)


if __name__ == '__main__':
    main()
