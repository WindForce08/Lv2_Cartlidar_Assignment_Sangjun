#!/usr/bin/env python3
"""모터 현재 위치 (tick) 를 큰 글씨로 실시간 표시 - 그리퍼 다시 결합할 때 각도 맞추기용 (읽기만, 토크 명령 없음).

    python3 tools/live_tick.py [--port /dev/ttyACM0] [--id 15] [--target 1100] [--tol 50]

보드에는 모터 설정 도구 (tools/dxl_setup) 가 올라가 있어야 한다 ('r <id>' 명령으로 읽음). 팔 펌웨어가 올라가 있으면 응답이 없다.
목표 ± 허용 안이면 초록, 밖이면 주황. 창 아래에 tick 이 늘어나는 / 줄어드는 방향도 보여 줌.
"""
import argparse
import re
import threading
import time
import tkinter as tk

import serial


def reader(port, mid, out):
    s = serial.Serial(port, 115200, timeout=0.05)
    time.sleep(1.5)
    s.read(65536)
    buf = b""
    while out["alive"]:
        s.write(f"r {mid}\n".encode())
        t_end = time.time() + 0.6
        while time.time() < t_end:
            buf += s.read(4096)
            m = re.search(rb"present_position_tick=(-?\d+)", buf)
            if m:
                out["tick"], out["t"] = int(m.group(1)), time.time()
                t = re.search(rb"torque_enable=(\d)", buf)
                out["torque"] = int(t.group(1)) if t else None
                buf = b""
                break
            if b"ERR" in buf:
                out["err"] = buf.decode(errors="replace").strip().splitlines()[-1]
                buf = b""
                break
        time.sleep(0.1)
    s.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id", type=int, default=15)
    ap.add_argument("--target", type=int, default=1100)
    ap.add_argument("--tol", type=int, default=50)
    a = ap.parse_args()
    out = {"alive": True, "tick": None, "t": 0.0, "torque": None, "err": ""}
    th = threading.Thread(target=reader, args=(a.port, a.id, out), daemon=True)
    th.start()
    root = tk.Tk()
    root.title(f"모터 ID {a.id} 현재 위치 (tick)")
    big = tk.Label(root, text="-", font=("DejaVu Sans Mono", 72, "bold"), width=6)
    big.pack(padx=20, pady=(16, 4))
    info = tk.Label(root, text="", font=("Noto Sans CJK KR", 13))
    info.pack(padx=20)
    trend = tk.Label(root, text="", font=("Noto Sans CJK KR", 12), fg="#505050")
    trend.pack(padx=20, pady=(2, 14))
    hist = []

    def update():
        tck = out["tick"]
        if tck is None or time.time() - out["t"] > 1.5:
            big.config(text="-", bg="#d0d0d0")
            info.config(text=out["err"] or "읽는 중 (보드에 dxl_setup 이 올라가 있어야 함)")
        else:
            ok = abs(tck - a.target) <= a.tol
            big.config(text=str(tck), bg="#8fd18f" if ok else "#f0c070")
            info.config(text=f"목표 {a.target} ± {a.tol}  ({'맞음' if ok else f'{tck - a.target:+d}'})   토크 {'ON ⚠' if out['torque'] else 'OFF'}")
            hist.append(tck)
            del hist[:-8]
            if len(hist) >= 2 and abs(hist[-1] - hist[0]) > 5:
                trend.config(text="tick 늘어나는 중 ↑" if hist[-1] > hist[0] else "tick 줄어드는 중 ↓")
            else:
                trend.config(text="멈춤")
        root.after(150, update)

    def close():
        out["alive"] = False
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", close)
    update()
    root.mainloop()


if __name__ == "__main__":
    main()
