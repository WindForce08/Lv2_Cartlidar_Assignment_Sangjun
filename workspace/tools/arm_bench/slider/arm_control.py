"""팔 OpenCR 직접 제어 (슬라이더 GUI 의 제어부, 화면 없이도 씀). 벤치 도구 opencr_arm_tool.py move 와 같은 순서 · 안전 규칙.

상태: IDLE (송신 안 함) → 시작 요청 → TORQUE (TORQUE_ON, HOLD 확인) → REARM (펄스, HOLD 해제 확인) → RUN (목표를 속도 한계로 따라감)
      정지 · 오류 → ESTOP (0.3 s) → STOPPED (송신 안 함 = 펌웨어가 현재 위치에서 HOLD, 토크 유지)
      토크 끄기 요청 → OFF (TORQUE_ON 없는 CMD 0.8 s = 펌웨어 D5-4 50 번 + HOLD → 토크 OFF, 팔이 떨어짐) → IDLE
시작 조건: 팔 보드 (role 2) · v5 · 중단 오류 비트 없음 · 펌웨어 무수신 (CMD_TIMEOUT, 다른 프로그램이 명령 안 함) · 현재 자세가 config.h 범위 안.
RUN 중 중단: 오류 비트 (중단 비트 · HOLD) · 추종 오차 > max_track · 0.3 s 넘게 STATE 없음.
6 번째 칸 (그리퍼) 도 움직인다 (속도 한계 grip_speed 따로). 그리퍼는 물체를 잡으면 목표에 못 가는 게 정상이라 추종 오차 정지에서 빼고,
대신 오차 GRIP_BLOCK_RAD 넘게 GRIP_BLOCK_S 막히면 목표를 "현재 위치 + 쥐는 힘 (닫는 쪽)" 으로 내려 약하게 쥔다.
(2026-10-27 실물: 1.5 s 기다리는 동안 그리퍼 모터가 과부하 (MOTOR_HW) - 모터 PWM 한계도 낮춤, README)
새 펌웨어 (GripGuard) 는 같은 일을 보드에서 먼저 한다 (config.h GRIP_*: 45 tick · 0.3 s → 막힌 자리 + 33 tick).
이 규칙은 문턱을 펌웨어와 같게 두고 시간만 길게 (0.5 s) - 펌웨어가 먼저 쥐고, 예전 펌웨어에서는 이 규칙이 쥔다.
쥐는 힘은 펌웨어가 쥔 뒤에는 GRIP_SQUEEZE_TICKS 보다 세게 할 수 없다 (더 세게는 config.h 를 바꿔 업로드).
"""
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "..", "..", "ros2_ws", "src", "opencr_hardware", "scripts"))   # Lv2 벤치 도구
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "monitor"))
import opencr_arm_tool as tool  # noqa: E402

import gripper_map  # noqa: E402

FW_GRIP = gripper_map.firmware_grip()   # 펌웨어 그리퍼 막힘 보호 값 (config.h)

PERIOD = 0.01          # 100 Hz
N_MOVE = 6             # 슬라이더로 움직이는 칸: 관절 1~5 + 그리퍼 (6 번째)
GRIP = 5
GRIP_BLOCK_RAD = FW_GRIP["stall"] * gripper_map.RAD_PER_TICK     # [rad] 그리퍼 목표 - 현재 가 이보다 크면 막힘 (펌웨어 GRIP_STALL_TICKS)
GRIP_BLOCK_S = 0.5     # [s] 이만큼 계속 막히면 목표를 내림 (펌웨어 0.3 s 보다 길게)
GRIP_SQUEEZE_RAD = FW_GRIP["squeeze"] * gripper_map.RAD_PER_TICK  # [rad] 막힌 자리에서 더 닫는 양 (기본 = 펌웨어 GRIP_SQUEEZE_TICKS)


class ArmController:
    def __init__(self, port, speed=0.2, max_track=0.25, on_state=None, grip_speed=0.8):
        self.port = port
        self.link = tool.Link(port)
        self.t_reopen = 0.0
        self.lo, self.hi = tool.load_limits()
        self.speed = speed               # [rad/s] 명령 변화 상한
        self.max_track = max_track       # [rad] 추종 오차 한계 (관절 1~5)
        self.grip_speed = grip_speed     # [rad/s] 그리퍼 명령 변화 상한
        self.grip_since = None           # 그리퍼가 막히기 시작한 시각
        self.grip_squeeze = GRIP_SQUEEZE_RAD   # [rad] 막힌 자리에서 더 닫는 양 = 쥐는 힘 (GUI 에서 바꿈)
        self.grip_hold_pos = None        # 쥐는 중이면 막힌 자리 (rad), 아니면 None
        self.on_state = on_state         # 새 STATE 마다 부름 (ROS 발행 등)
        self.state, self.msg = "IDLE", "대기 - [시작] 을 누르면 토크 ON (현재 자세에서 버팀)"
        self.cmd = self.target = None
        self.seq, self.req = 0, None
        self.last, self.count_seen, self.t_state = None, 0, time.monotonic()
        self.t_phase = 0.0
        self.lock = threading.Lock()
        self.alive = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    # ---- GUI 에서 부르는 것 ----
    def request(self, what):           # "start" | "stop" | "off"
        with self.lock:
            self.req = what

    def set_target(self, i, rad):
        with self.lock:
            if self.target is not None and i < N_MOVE:
                self.target[i] = min(self.hi[i], max(self.lo[i], rad))
                if i == GRIP:
                    self.grip_hold_pos = None     # 사용자가 그리퍼를 움직이면 쥐기 끝

    def set_squeeze(self, rad):
        """쥐는 힘 (막힌 자리에서 더 닫는 양). 쥐는 중이면 바로 목표에 반영"""
        with self.lock:
            self.grip_squeeze = max(0.0, rad)
            if self.grip_hold_pos is not None and self.state == "RUN":
                self.target[GRIP] = min(self.hi[GRIP], self.grip_hold_pos + self.grip_squeeze)

    def snapshot(self):
        with self.lock:
            return dict(state=self.state, msg=self.msg, last=self.last, cmd=list(self.cmd) if self.cmd else None,
                        target=list(self.target) if self.target else None, grip_hold=self.grip_hold_pos is not None)

    def close(self):
        """창을 닫을 때: 움직이는 중이면 제어 루프가 ESTOP 후 송신 중단 (HOLD, 토크 유지)"""
        if self.state in ("TORQUE", "REARM", "RUN"):
            self.request("stop")
            t_end = time.monotonic() + 1.5
            while self.state in ("TORQUE", "REARM", "RUN") and time.monotonic() < t_end:
                time.sleep(0.02)
        self.alive = False
        self.thread.join(timeout=1.0)

    # ---- 제어 루프 ----
    def _send(self, flags):
        if self.link is None:
            return
        self.link.ser.write(tool.encode_cmd(self.cmd, flags, self.seq))
        self.seq += 1

    def _set(self, state, msg):
        self.state, self.msg, self.t_phase = state, msg, time.monotonic()

    def _estop(self, why):
        t_end = time.monotonic() + 0.3
        while time.monotonic() < t_end:
            self._send(tool.FLAG_TORQUE_ON | tool.FLAG_ESTOP)
            if self.link is not None:
                self.link.poll()
            time.sleep(PERIOD)
        with self.lock:
            self._set("STOPPED", f"정지: {why} - 현재 위치에서 HOLD (토크 유지). [시작] 으로 다시, [토크 끄기] 로 끔")

    def _start_check(self, s):
        if s is None or time.monotonic() - self.t_state > 0.3:
            return "STATE 없음 - 보드 연결 · 전원 확인"
        if s["role"] != tool.ROLE_ARM or s["fw"] != tool.PROTOCOL_VERSION:
            return f"팔 보드 v{tool.PROTOCOL_VERSION} 가 아님 (role={s['role']} fw={s['fw']})"
        if s["error"] & tool.ERR_ABORT:
            return f"오류 비트 {tool.err_text(s['error'])} - 움직이지 않음"
        if not s["error"] & tool.ERR_TIMEOUT:
            return "펌웨어가 무수신 상태가 아님 - 다른 프로그램이 명령을 보내는지 확인 (모니터 · 벤치 도구 끄기)"
        q = tool.arm_pos(s)
        bad = [i + 1 for i in range(6) if not self.lo[i] <= q[i] <= self.hi[i]]
        if bad:
            return f"현재 자세가 config.h 범위 밖 (관절 {bad}) - 손으로 범위 안으로 옮긴 뒤 다시"
        return None

    def _loop(self):
        while self.alive:
            t0 = time.monotonic()
            if self.link is None:                      # 포트가 끊긴 뒤: 1 초마다 다시 열기 (그동안 송신 없음 → 펌웨어 HOLD)
                if t0 - self.t_reopen > 1.0:
                    self.t_reopen = t0
                    try:
                        self.link = tool.Link(self.port)
                        with self.lock:
                            self._set("IDLE", "포트 다시 열림 - [시작] 으로 다시")
                    except Exception:  # noqa: BLE001
                        pass
                time.sleep(PERIOD)
                continue
            try:
                s = self.link.poll()
            except Exception as e:  # noqa: BLE001
                try:
                    self.link.ser.close()
                except Exception:  # noqa: BLE001
                    pass
                self.link = None
                with self.lock:
                    self.last = None
                    self._set("IDLE", f"포트 오류 ({type(e).__name__}) - USB · 전원 확인. 다시 여는 중 (보드는 명령이 끊겨 HOLD)")
                continue
            if self.link.count != self.count_seen:
                self.count_seen, self.t_state = self.link.count, t0
                if self.on_state is not None:
                    try:
                        self.on_state(s)
                    except Exception:  # noqa: BLE001
                        pass
            with self.lock:
                self.last = s
                req, self.req = self.req, None
            try:
                self._step(s, req, t0)
            except Exception as e:  # noqa: BLE001
                with self.lock:
                    self._set("STOPPED", f"내부 오류: {e}")
            time.sleep(max(0.0, PERIOD - (time.monotonic() - t0)))

    def _step(self, s, req, now):
        st = self.state
        if req == "stop" and st in ("TORQUE", "REARM", "RUN"):
            self._estop("사용자 정지")
            return
        if req == "off" and st in ("RUN", "STOPPED"):
            with self.lock:
                self.cmd = [min(self.hi[i], max(self.lo[i], q)) for i, q in enumerate(tool.arm_pos(s))]
                self._set("OFF", "토크 끄는 중 (0.8 s) - 팔이 떨어지니 받치고 있을 것")
            return
        if req == "start" and st in ("IDLE", "STOPPED"):
            why = self._start_check(s)
            if why:
                with self.lock:
                    self.msg = "시작 안 함: " + why
                return
            with self.lock:
                q = tool.arm_pos(s)
                self.cmd, self.target = list(q), list(q)
                self._set("TORQUE", "토크 ON - HOLD 기다림")
            return
        stale = now - self.t_state > 0.3
        if st == "TORQUE":
            self._send(tool.FLAG_TORQUE_ON)
            if s is not None and s["error"] & tool.ERR_ABORT:
                self._estop(f"토크 ON 뒤 오류 {tool.err_text(s['error'])}")
            elif s is not None and s["error"] & tool.ERR_HOLD and not s["error"] & tool.ERR_TIMEOUT:
                with self.lock:
                    self._set("REARM", "REARM (HOLD 풀기)")
            elif now - self.t_phase > 2.0:
                self._estop("토크 ON 뒤 HOLD 를 보지 못함")
        elif st == "REARM":
            if now - self.t_phase < 0.03:
                self._send(tool.FLAG_TORQUE_ON | tool.FLAG_REARM)
            else:
                self._send(tool.FLAG_TORQUE_ON)
                if s is not None and not s["error"] & tool.ERR_HOLD:
                    with self.lock:
                        self._set("RUN", "움직이는 중 - 슬라이더로 조작. 정지: [정지] 또는 스페이스")
                elif now - self.t_phase > 1.0:
                    self._estop("REARM 뒤에도 HOLD")
        elif st == "RUN":
            with self.lock:
                for i in range(N_MOVE):
                    step = (self.grip_speed if i == GRIP else self.speed) * PERIOD
                    d = self.target[i] - self.cmd[i]
                    self.cmd[i] += max(-step, min(step, d))
            self._send(tool.FLAG_TORQUE_ON)
            if stale:
                self._estop("STATE 가 0.3 s 넘게 없음")
            elif s["error"] & (tool.ERR_ABORT | tool.ERR_HOLD):
                self._estop(f"오류 비트 {tool.err_text(s['error'])}")
            else:
                q = tool.arm_pos(s)
                worst = max(abs(q[i] - self.cmd[i]) for i in range(GRIP))
                if worst > self.max_track:
                    self._estop(f"추종 오차 {worst:.3f} rad > {self.max_track} (막힘 · 힘 부족?)")
                elif (self.target[GRIP] - q[GRIP]) * (self.cmd[GRIP] - q[GRIP]) > 0 and abs(self.cmd[GRIP] - q[GRIP]) > GRIP_BLOCK_RAD:
                    # 사용자 목표와 명령이 같은 쪽으로 현재 위치보다 GRIP_BLOCK_RAD 넘게 앞서 있음 = 막힘. 반대쪽 (예: 열기) 을 주면 바로 풀림
                    if self.grip_since is None:
                        self.grip_since = now
                    elif now - self.grip_since > GRIP_BLOCK_S:
                        closing = self.cmd[GRIP] > q[GRIP]
                        hold = min(self.hi[GRIP], q[GRIP] + self.grip_squeeze) if closing else q[GRIP]
                        with self.lock:
                            self.cmd[GRIP] = self.target[GRIP] = hold
                            self.grip_hold_pos = q[GRIP] if closing else None
                            self.msg = (f"그리퍼가 {GRIP_BLOCK_S} s 넘게 막힘 (물체를 잡음) - "
                                        + ("그 자리에서 약하게 쥠" if closing else "그 자리에서 멈춤"))
                        self.grip_since = None
                else:
                    self.grip_since = None
        elif st == "OFF":
            self._send(0)                                  # TORQUE_ON 없음
            if now - self.t_phase > 0.8:
                with self.lock:
                    self._set("IDLE", "토크 끔 (송신 중단). [시작] 으로 다시")
