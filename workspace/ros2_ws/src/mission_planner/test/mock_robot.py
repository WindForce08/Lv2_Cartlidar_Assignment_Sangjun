"""mission_node plan 모드 연동 시험용 가짜 서버 (drive · manipulator · perception). 실물 · 시뮬레이터 대신.

  - /drive/navigate · /drive/align · /drive/execute_skill · /manipulator/pose · /manipulator/pick · /manipulator/execute_skill: 잠깐 뒤 성공
    (execute_skill 은 이름을 기록, --reject <접두어> 면 그 이름은 거절)
  - /perception/set_mode: READ_SIGNAL 이면 /perception/signal CONFIRMED (--signal), READ_BARCODE 면 /perception/barcode READ (--barcode)
  - /perception/check_dice: observed · passed true (요청 칸 · 기대값 기록)
끝에 기록을 --out JSON 으로 씀 (/mission/status 가 DONE · FAILED 가 되면).
"""
import argparse
import json
import threading
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from robot_interfaces.action import AlignBase, ArmPose, ExecuteSkill, Pick
from robot_interfaces.msg import Barcode, MissionStatus, SignalSequence
from robot_interfaces.srv import CheckDice, SetPerceptionMode
from std_msgs.msg import Bool


class Mock(Node):
    def __init__(self, a):
        super().__init__("mock_robot")
        self.a = a
        self.log = dict(navigate=[], align=[], drive_skill=[], pose=[], pick=[], skill=[], set_mode=[], check_dice=[])
        self.mode = 0
        cg = ReentrantCallbackGroup()

        def server(kind, typ, name, result_fn):
            def cb(gh):
                self.log[kind].append(self.describe(kind, gh.request))
                time.sleep(a.delay)
                r = result_fn(gh.request)
                if r is None:
                    gh.abort()
                    return typ.Result(success=False, message="mock reject") if hasattr(typ.Result(), "message") else typ.Result()
                gh.succeed()
                return r
            ActionServer(self, typ, name, cb, callback_group=cg)

        server("navigate", NavigateToPose, "/drive/navigate", lambda g: NavigateToPose.Result())
        server("align", AlignBase, "/drive/align", lambda g: AlignBase.Result(success=True))
        server("drive_skill", ExecuteSkill, "/drive/execute_skill",
               lambda g: None if (a.reject and g.policy.startswith(a.reject)) else ExecuteSkill.Result(success=True))
        server("pose", ArmPose, "/manipulator/pose", lambda g: ArmPose.Result(success=True))
        server("pick", Pick, "/manipulator/pick", lambda g: Pick.Result(success=True, grasped=True))
        server("skill", ExecuteSkill, "/manipulator/execute_skill",
               lambda g: None if (a.reject and g.policy.startswith(a.reject)) else ExecuteSkill.Result(success=True))
        self.create_service(SetPerceptionMode, "/perception/set_mode", self.on_mode, callback_group=cg)
        self.create_service(CheckDice, "/perception/check_dice", self.on_dice, callback_group=cg)
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.sig_pub = self.create_publisher(SignalSequence, "/perception/signal", q)
        self.bar_pub = self.create_publisher(Barcode, "/perception/barcode", q)
        self.estop_pub = self.create_publisher(Bool, "/e_stop", QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.estop_pub.publish(Bool(data=False))
        self.create_subscription(MissionStatus, "/mission/status", self.on_status, 10, callback_group=cg)
        self.create_timer(0.3, self.pub_perception, callback_group=cg)
        self.t_mode = {}
        self.final = None
        self.steps_seen = []

    def describe(self, kind, g):
        if kind == "navigate":
            p = g.pose.pose
            import math
            return [round(p.position.x, 4), round(p.position.y, 4), round(2 * math.atan2(p.orientation.z, p.orientation.w), 4)]
        if kind == "align":
            return round(g.target_distance, 3)
        if kind == "pose":
            return g.name
        if kind == "pick":
            p = g.target.pose.position
            return [g.target.header.frame_id, round(p.x, 4), round(p.y, 4), round(p.z, 4)]
        if kind in ("skill", "drive_skill"):
            return g.policy
        return None

    def on_mode(self, req, res):
        self.mode = req.mode
        self.log["set_mode"].append(int(req.mode))
        for bit in (32, 64):
            if req.mode & bit:
                self.t_mode[bit] = time.time()
        res.success = True
        return res

    def on_dice(self, req, res):
        self.log["check_dice"].append(dict(frame=req.header.frame_id, cells=[[round(c.x, 3), round(c.y, 3), round(c.z, 3)] for c in req.cells],
                                           expect="".join(str(int(e)) for e in req.expect), yellow_shape=int(req.yellow_shape)))
        res.observed, res.passed, res.message = True, True, "mock ok"
        return res

    def pub_perception(self):
        now = self.get_clock().now().to_msg()
        if self.mode & 32 and time.time() - self.t_mode.get(32, 0) > self.a.signal_after:
            m = SignalSequence(status=SignalSequence.CONFIRMED, sequence=self.a.signal, cycles=2)
            m.header.stamp, m.header.frame_id = now, "base_link"
            self.sig_pub.publish(m)
        if self.mode & 64 and time.time() - self.t_mode.get(64, 0) > 1.0:
            b = Barcode(source=Barcode.READ, read_ok=True, text=self.a.barcode, symbols=3)
            b.header.stamp, b.header.frame_id = now, "base_link"
            self.bar_pub.publish(b)

    def on_status(self, m):
        if m.step_name and (not self.steps_seen or self.steps_seen[-1] != (m.step_index, m.step_name)):
            self.steps_seen.append((m.step_index, m.step_name))
        if m.state in (MissionStatus.DONE, MissionStatus.FAILED) and self.final is None:
            self.final = dict(state="DONE" if m.state == MissionStatus.DONE else "FAILED", reason=m.reason, step=m.step_name)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--signal", default="RGRRGRGGYGYG")
    ap.add_argument("--barcode", default="P2-01")
    ap.add_argument("--signal_after", type=float, default=2.0)
    ap.add_argument("--delay", type=float, default=0.02)
    ap.add_argument("--reject", default="")
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    rclpy.init()
    n = Mock(a)
    ex = MultiThreadedExecutor(num_threads=8)
    ex.add_node(n)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    t0 = time.time()
    while n.final is None and time.time() - t0 < a.timeout:
        time.sleep(0.2)
    out = dict(final=n.final or dict(state="TIMEOUT"), elapsed_s=round(time.time() - t0, 1), **n.log,
               steps_seen=[f"[{i}] {s}" for i, s in n.steps_seen])
    json.dump(out, open(a.out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"[MOCK] {out['final']} navigate {len(n.log['navigate'])} align {len(n.log['align'])} drive_skill {len(n.log['drive_skill'])} pick {len(n.log['pick'])} "
          f"skill {len(n.log['skill'])} check_dice {len(n.log['check_dice'])} ({out['elapsed_s']} s)", flush=True)
    ex.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
