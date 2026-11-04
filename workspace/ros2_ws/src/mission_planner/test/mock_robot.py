"""mission_node plan 모드 연동 시험용 가짜 서버 (drive · manipulator · perception). 실물 · 시뮬레이터 대신.

  - /drive/navigate · /drive/align · /drive/execute_skill · /manipulator/pose · /manipulator/pick · /manipulator/execute_skill: 잠깐 뒤 성공
    (execute_skill 은 이름을 기록, --reject <접두어> 면 그 이름은 거절)
  - /perception/set_mode: READ_SIGNAL 이면 /perception/signal CONFIRMED (--signal), READ_BARCODE 면 /perception/barcode READ (--barcode)
  - /perception/check_dice: observed · passed true (요청 칸 · 기대값 기록)
  - /perception/place_box/measure (#73): 마지막 navigate 목표에서 가까운 장소 L 의 지도 칸 중심을 --box_shift 만큼 밀고 --box_rot 만큼 돌린
    상자를 odom 기준 /perception/place_box · /status 로 냄 (map -> odom 정적 TF = --odom_offset). --box_fail 이면 ok false
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
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster
import math
import os
import yaml


class Mock(Node):
    def __init__(self, a):
        super().__init__("mock_robot")
        self.a = a
        self.log = dict(navigate=[], align=[], drive_skill=[], pose=[], pick=[], skill=[], set_mode=[], check_dice=[], place_box=[])
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
        # 장소 상자 측정 (#73)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pb_pub = self.create_publisher(PoseStamped, "/perception/place_box", latched)
        self.pb_status_pub = self.create_publisher(String, "/perception/place_box/status", latched)
        self.create_service(Trigger, "/perception/place_box/measure", self.on_place_box, callback_group=cg)
        ap_file = a.approach or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "approach_poses.yaml")
        table = yaml.safe_load(open(ap_file, encoding="utf-8"))
        self.places = {L: (table[L]["cells"]["0,0"]["x"], table[L]["cells"]["0,0"]["y"], table[L]["stage"]) for L in ("L1", "L2", "L3")}
        self.odom_off = [float(v) for v in a.odom_offset.split(",")]   # map -> odom (x, y, yaw deg)
        self.tf_pub = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = "map", "odom"
        t.transform.translation.x, t.transform.translation.y = self.odom_off[0], self.odom_off[1]
        yo = math.radians(self.odom_off[2])
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yo / 2), math.cos(yo / 2)
        self.tf_pub.sendTransform(t)
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

    def on_place_box(self, req, res):
        nav = self.log["navigate"][-1] if self.log["navigate"] else [0.0, 0.0, 0.0]
        L = min(self.places, key=lambda k: math.hypot(self.places[k][2][0] - nav[0], self.places[k][2][1] - nav[1]))
        if self.a.box_fail:
            self.log["place_box"].append(dict(place=L, ok=False))
            self.pb_status_pub.publish(String(data='{"ok": false, "why": "mock"}'))
            res.success, res.message = True, "mock measure (fail)"
            return res
        dx, dy = (float(v) for v in self.a.box_shift.split(","))
        mx, my = self.places[L][0] + dx, self.places[L][1] + dy               # 지도에서 밀린 상자 중심
        ox, oy, oyaw = self.odom_off[0], self.odom_off[1], math.radians(self.odom_off[2])
        c, s_ = math.cos(-oyaw), math.sin(-oyaw)
        px, py = c * (mx - ox) - s_ * (my - oy), s_ * (mx - ox) + c * (my - oy)   # odom 좌표
        yaw = math.radians(self.a.box_rot) - oyaw                              # 상자 요 (odom)
        self.pb_status_pub.publish(String(data=f'{{"ok": true, "frames": 3, "x": {px:.4f}, "y": {py:.4f}, "yaw": {yaw:.4f}, "frame": "odom"}}'))
        m = PoseStamped()
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), "odom"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = px, py, 0.07
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pb_pub.publish(m)
        self.log["place_box"].append(dict(place=L, map_center=[round(mx, 4), round(my, 4)], rot_deg=self.a.box_rot))
        res.success, res.message = True, "mock measure"
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
    ap.add_argument("--approach", default="", help="approach_poses.yaml (기본 ../config)")
    ap.add_argument("--box_shift", default="0.02,-0.01", help="장소 상자를 지도 칸에서 민 거리 dx,dy [m]")
    ap.add_argument("--box_rot", type=float, default=5.0, help="장소 상자 요 [deg]")
    ap.add_argument("--box_fail", action="store_true", help="place_box 측정 실패 (ok false)")
    ap.add_argument("--odom_offset", default="0.3,-0.2,10", help="map -> odom 정적 TF x,y,yaw[deg]")
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
