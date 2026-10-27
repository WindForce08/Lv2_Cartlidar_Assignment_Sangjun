#!/usr/bin/env python3
"""슬라이더로 실물 팔 관절을 직접 움직이는 창 (장치 쓰기: 토크 ON · 이동). 제어 · 안전 규칙은 arm_control.py.

    python3 slider/arm_slider.py --port /dev/ttyACM0 [--speed 0.2] [--max-track 0.25]     (보통은 slider/slider.launch.py)

- [시작] 을 눌러야 토크가 켜진다 (현재 자세에서 버팀 → REARM). 슬라이더는 그 순간의 실제 자세에서 시작해 팔이 튀지 않는다.
- 슬라이더 목표는 속도 한계 (창 아래 '속도', 기본 0.2 rad/s) 로만 따라간다. 범위는 config.h ARM_MIN/MAX_RAD.
- [정지] · 스페이스 · 창 닫기 · 오류 · 추종 오차 → ESTOP, 현재 위치에서 HOLD (토크 유지).
- [토크 끄기] 는 '팔을 받쳤음' 을 체크해야 눌린다 (토크가 꺼지면 팔이 떨어짐).
- 그리퍼는 손가락 간격 [mm] 슬라이더 (-11 닫힘 ~ +20 열림) → 모터 각 (monitor/gripper_map.py). 속도 한계 0.8 rad/s 따로,
  물체를 잡아 목표에 못 가면 '막힌 자리 + 쥐는 힘' 으로 목표를 내림 (계속 조이지 않게). 새 펌웨어는 보드가 먼저 함 (config.h GRIP_*).
- rclpy 가 있으면 /joint_states 를 같이 보내 RViz2 의 팔이 따라온다 (손가락 = monitor/gripper_map.py).
"""
import argparse
import math
import os
import signal
import sys
import tkinter as tk
from tkinter import font as tkfont

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "monitor"))
from arm_control import FW_GRIP, GRIP, ArmController, tool  # noqa: E402

import gripper_map  # noqa: E402

NAMES = ["J1 허리", "J2 어깨", "J3 팔꿈치", "J4 손목 굽힘", "J5 손목 롤"]
JOINTS = ["joint1", "joint2", "joint3", "joint4", "wrist_roll_joint", "gripper_motor_joint"]
GRIP_PWM_LIMIT = FW_GRIP["pwm_max"]   # 그리퍼 모터 EEPROM PWM Limit (2026-10-27 dxl_setup w 15 400 = 펌웨어 부팅 점검 상한) - 표시용
COLORS = {"IDLE": "#606060", "STOPPED": "#b06000", "TORQUE": "#2060c0", "REARM": "#2060c0", "RUN": "#108040", "OFF": "#a02020"}


def make_ros_publisher():
    try:
        import rclpy
        from sensor_msgs.msg import JointState
    except ImportError:
        return None, None
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # 종료 신호는 아래 main 이 받아 창을 정상으로 닫음
    node = rclpy.create_node("arm_slider")
    pub = node.create_publisher(JointState, "/joint_states", 10)

    def publish(s):
        q = tool.arm_pos(s)
        f = gripper_map.finger_from_rad(q[5])
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = JOINTS + ["gripper_left_joint", "gripper_right_joint"]
        msg.position = [float(x) for x in q] + [f, f]
        pub.publish(msg)
    return node, publish


class App:
    def __init__(self, root, ctrl, title):
        self.root, self.ctrl = root, ctrl
        root.title(title)
        base = tkfont.nametofont("TkDefaultFont")
        base.configure(family="Noto Sans CJK KR", size=11)
        root.option_add("*Font", base)
        self.state_lbl = tk.Label(root, text="", font=("Noto Sans CJK KR", 14, "bold"), fg="white", width=12)
        self.state_lbl.grid(row=0, column=0, sticky="w", padx=8, pady=6)
        self.msg_lbl = tk.Label(root, text="", anchor="w", justify="left", wraplength=640)
        self.msg_lbl.grid(row=0, column=1, columnspan=4, sticky="we", padx=8)
        self.scales, self.pos_lbl, self.cur_lbl = [], [], []
        self.dragging = False
        for i in range(GRIP):
            lo, hi = math.degrees(ctrl.lo[i]), math.degrees(ctrl.hi[i])
            tk.Label(root, text=NAMES[i], width=12, anchor="w").grid(row=1 + i, column=0, padx=8)
            sc = tk.Scale(root, from_=lo, to=hi, orient="horizontal", resolution=0.5, length=440, showvalue=True,
                          command=lambda v, i=i: self.on_slide(i, v))
            sc.grid(row=1 + i, column=1, columnspan=2, padx=4)
            sc.bind("<ButtonPress-1>", lambda e: setattr(self, "dragging", True))
            sc.bind("<ButtonRelease-1>", lambda e: setattr(self, "dragging", False))
            self.scales.append(sc)
            lp = tk.Label(root, text="", width=16, anchor="w", font=("DejaVu Sans Mono", 10))
            lp.grid(row=1 + i, column=3)
            self.pos_lbl.append(lp)
            lc = tk.Label(root, text="", width=9, anchor="e", font=("DejaVu Sans Mono", 10))
            lc.grid(row=1 + i, column=4, padx=6)
            self.cur_lbl.append(lc)
        r = 1 + GRIP
        tk.Label(root, text="그리퍼 [mm]", width=12, anchor="w").grid(row=r, column=0, padx=8)
        sc = tk.Scale(root, from_=gripper_map.FINGER_CLOSED * 1000, to=gripper_map.FINGER_OPEN * 1000, orient="horizontal",
                      resolution=0.5, length=440, showvalue=True, command=self.on_grip)
        sc.grid(row=r, column=1, columnspan=2, padx=4)
        self.scales.append(sc)
        for col, lst, w, anchor in ((3, self.pos_lbl, 16, "w"), (4, self.cur_lbl, 9, "e")):
            lb = tk.Label(root, text="", width=w, anchor=anchor, font=("DejaVu Sans Mono", 10))
            lb.grid(row=r, column=col, padx=6 if col == 4 else 0)
            lst.append(lb)
        N_MOVE = GRIP + 1
        self.grip_lbl = tk.Label(root, text="", anchor="w", fg="#606060")
        self.grip_lbl.grid(row=1 + N_MOVE, column=1, columnspan=4, sticky="w", padx=4)
        N_MOVE += 1
        sq = tk.Frame(root)
        sq.grid(row=1 + N_MOVE, column=0, columnspan=5, sticky="we", padx=8)
        tk.Label(sq, text="쥐는 힘 [tick]", width=12, anchor="w").pack(side="left")
        self.squeeze = tk.Scale(sq, from_=0, to=FW_GRIP["squeeze"], resolution=1, orient="horizontal", length=260,
                                command=lambda v: ctrl.set_squeeze(float(v) * gripper_map.RAD_PER_TICK))
        self.squeeze.set(round(ctrl.grip_squeeze / gripper_map.RAD_PER_TICK))
        self.squeeze.pack(side="left", padx=4)
        self.sq_lbl = tk.Label(sq, text="", anchor="w", fg="#404040")
        self.sq_lbl.pack(side="left", padx=6)
        N_MOVE += 1
        tk.Label(root, text="속도 [rad/s]", width=12, anchor="w").grid(row=2 + N_MOVE, column=0, padx=8)
        self.speed = tk.Scale(root, from_=0.05, to=0.5, resolution=0.05, orient="horizontal", length=200,
                              command=lambda v: setattr(ctrl, "speed", float(v)))
        self.speed.set(ctrl.speed)
        self.speed.grid(row=2 + N_MOVE, column=1, sticky="w", padx=4)
        bar = tk.Frame(root)
        bar.grid(row=3 + N_MOVE, column=0, columnspan=5, sticky="we", padx=8, pady=8)
        tk.Button(bar, text="시작 (토크 ON)", width=14, command=lambda: ctrl.request("start")).pack(side="left", padx=4)
        tk.Button(bar, text="정지 (스페이스)", width=14, bg="#c03030", fg="white", activebackground="#e04040",
                  command=lambda: ctrl.request("stop")).pack(side="left", padx=4)
        tk.Button(bar, text="현재 자세로", width=12, command=self.to_current).pack(side="left", padx=4)
        self.held = tk.BooleanVar(value=False)
        tk.Checkbutton(bar, text="팔을 받쳤음", variable=self.held).pack(side="left", padx=(24, 2))
        self.off_btn = tk.Button(bar, text="토크 끄기", width=10, command=self.torque_off)
        self.off_btn.pack(side="left", padx=4)
        tk.Label(root, text="열 순서: 현재 · 명령 [관절 deg, 그리퍼 mm] | 전류 [A]", fg="#707070").grid(row=4 + N_MOVE, column=0, columnspan=5,
                                                                                    sticky="w", padx=8)
        root.bind("<space>", lambda e: ctrl.request("stop"))
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.update()

    def on_slide(self, i, v):
        if self.ctrl.state == "RUN":
            self.ctrl.set_target(i, math.radians(float(v)))

    def on_grip(self, v):
        if self.ctrl.state == "RUN":
            self.ctrl.set_target(GRIP, gripper_map.rad_from_finger(float(v) / 1000))

    def to_current(self):
        snap = self.ctrl.snapshot()
        if snap["last"] is None:
            return
        q = tool.arm_pos(snap["last"])
        for i in range(GRIP):
            self.scales[i].set(math.degrees(q[i]))
            self.ctrl.set_target(i, q[i])
        self.scales[GRIP].set(gripper_map.finger_from_rad(q[GRIP]) * 1000)
        self.ctrl.set_target(GRIP, q[GRIP])

    def torque_off(self):
        if not self.held.get():
            self.msg_lbl.config(text="토크 끄기: 먼저 팔을 손으로 받치고 '팔을 받쳤음' 을 체크하세요 (토크가 꺼지면 팔이 떨어짐)")
            return
        self.ctrl.request("off")
        self.held.set(False)

    def update(self):
        snap = self.ctrl.snapshot()
        st, s = snap["state"], snap["last"]
        self.state_lbl.config(text=st, bg=COLORS.get(st, "#606060"))
        err = tool.err_text(s["error"]) if s else "STATE 없음"
        self.msg_lbl.config(text=f"{snap['msg']}\n오류 비트: {err}")
        if s is not None:
            q = tool.arm_pos(s)
            cur = s["effort"][tool.ARM0:tool.ARM0 + 6]
            show = (lambda x: math.degrees(x), lambda x: gripper_map.finger_from_rad(x) * 1000)   # 관절 deg, 그리퍼 mm
            for i in range(GRIP + 1):
                f = show[1] if i == GRIP else show[0]
                c = snap["cmd"][i] if snap["cmd"] and st in ("RUN", "TORQUE", "REARM") else None
                self.pos_lbl[i].config(text=f"{f(q[i]):7.1f} · " + (f"{f(c):7.1f}" if c is not None else "   -   "))
                self.cur_lbl[i].config(text=f"{cur[i]:.2f}")
                if st != "RUN":
                    self.scales[i].config(state="normal")
                    self.scales[i].set(f(q[i]))     # 시작 전 · 정지 중에는 실제 자세를 따라감 (시작 때 튀지 않게)
                    self.scales[i].config(state="disabled")
                else:
                    self.scales[i].config(state="normal")
            gt = gripper_map.unwrap_tick(2048 + q[GRIP] / gripper_map.RAD_PER_TICK)
            self.grip_lbl.config(text=f"그리퍼 모터 {math.degrees(q[GRIP]):.1f}° ({gt:.0f} tick). 닫힘 {gripper_map.CLOSED_TICK} · "
                                      f"열림 {gripper_map.OPEN_TICK} tick (gripper_map.py)")
            ticks = self.squeeze.get()
            pwm = min(GRIP_PWM_LIMIT, 6.25 * ticks)       # 위치 P 800 → PWM = 800/128 x 오차 tick, 모터 PWM 한계에서 잘림
            hold = "쥐는 중" if snap.get("grip_hold") else "쥐지 않음"
            self.sq_lbl.config(text=f"출력 약 {pwm / 885 * 100:.0f} % (펌웨어 상한 {FW_GRIP['squeeze']} tick, PWM 한계 {GRIP_PWM_LIMIT}) · {hold} · "
                                    f"그리퍼 전류 {cur[GRIP]:.2f} A")
        self.root.after(50, self.update)

    def close(self):
        self.ctrl.close()
        self.root.destroy()


def _tick(root):
    root.after(200, _tick, root)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--speed", type=float, default=0.2, help="명령 변화 상한 [rad/s] (창에서 0.05 ~ 0.5)")
    ap.add_argument("--max-track", type=float, default=0.25, help="추종 오차 한계 [rad] - 넘으면 정지")
    ap.add_argument("--title", default="")
    args, _ = ap.parse_known_args()
    node, publish = make_ros_publisher()
    ctrl = ArmController(args.port, speed=min(0.5, max(0.05, args.speed)), max_track=args.max_track, on_state=publish)
    root = tk.Tk()
    app = App(root, ctrl, args.title or f"arm_bench 슬라이더 - {args.port}")
    # launch 종료 (SIGTERM) · Ctrl-C (SIGINT) 에도 창 닫기와 같게: 움직이는 중이면 ESTOP 후 HOLD, 포트 놓음
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: root.after(0, app.close))
    root.after(200, lambda: None)                                  # 신호가 Tk 루프 안에서 처리되도록 주기적으로 깨움
    _tick(root)
    try:
        root.mainloop()
    finally:
        if ctrl.alive:
            ctrl.close()
        if node is not None:
            node.destroy_node()
            import rclpy
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
