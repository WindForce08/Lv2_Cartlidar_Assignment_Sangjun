"""mission_planner_node — 관측 결과로 계획을 세워 mission_node 에 단계 문장으로 넘김.

구독: /perception/signal (SignalSequence, CONFIRMED 순서), /perception/barcode (Barcode, source READ · read_ok)
서비스: /mission_planner/plan (std_srvs/Trigger)
  success = true  -> message = 단계 문장 (줄바꿈으로 구분, mission_node plan 단계가 그대로 붙임)
  success = false -> message = 이유 (점멸 미확정 · 미등록 바코드 · 풀 수 없는 문제 등 -> mission 안전 정지)
기록 (log_dir): signal.json · allocation.json · plan.json · mission_lines.txt (발제 제출물 형식)
파라미터:
  pucks_file  퍽 12 개 등록표 (placement.csv 를 옮긴 yaml: pucks [{id, color, shape, box, side}]). signal · barcode 키가 있어도 무시 (관측값을 씀)
  mode        per_location | single_load
  fine_distance · skill_timeout · return_wait   mission_lines.to_lines 인자
  sdf         경기장 (비우면 패키지 worlds/dice_mission.sdf)
  log_dir     비우면 ~/.ros/mission_planner/<시각>
"""
from __future__ import annotations

import json
import os
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import Barcode, SignalSequence
from std_srvs.srv import Trigger

from .allocation import MissionError
from .cli import default_sdf
from .mission_lines import to_lines
from .planner import World, plan


class PlannerNode(Node):
    def __init__(self):
        super().__init__("mission_planner_node")
        self.pucks_file = self.declare_parameter("pucks_file", "").value
        self.mode = self.declare_parameter("mode", "per_location").value
        self.fine = float(self.declare_parameter("fine_distance", 0.13).value)
        self.skill_timeout = float(self.declare_parameter("skill_timeout", 40.0).value)
        self.return_wait = float(self.declare_parameter("return_wait", 3.0).value)
        sdf = self.declare_parameter("sdf", "").value or default_sdf()
        log_dir = self.declare_parameter("log_dir", "").value
        self.log_dir = log_dir or os.path.expanduser(os.path.join("~/.ros/mission_planner", time.strftime("%Y%m%d_%H%M%S")))
        self.world = World(sdf)  # 지도 · 접근 자세 · 비용 (시작 때 한 번, 약 2 s)
        self.signal = None   # (sequence, stamp)
        self.barcode = None  # (text, stamp)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(SignalSequence, "/perception/signal", self.on_signal, qos)
        self.create_subscription(Barcode, "/perception/barcode", self.on_barcode, qos)
        self.create_service(Trigger, "/mission_planner/plan", self.on_plan)
        self.get_logger().info(f"ready (mode {self.mode}, pucks {self.pucks_file or '없음'}, log {self.log_dir})")

    def on_signal(self, m: SignalSequence):
        if m.status == SignalSequence.CONFIRMED and len(m.sequence) == 12:
            self.signal = (m.sequence, m.header.stamp)

    def on_barcode(self, m: Barcode):
        if m.source == Barcode.READ and m.read_ok and m.text:
            self.barcode = (m.text.strip(), m.header.stamp)

    def on_plan(self, req, res):
        try:
            if self.signal is None:
                raise MissionError("확정된 점멸 순서 없음 (/perception/signal CONFIRMED)")
            if self.barcode is None:
                raise MissionError("읽은 바코드 없음 (/perception/barcode READ)")
            if not self.pucks_file:
                raise MissionError("pucks_file 파라미터 없음 (퍽 등록표)")
            prob = yaml.safe_load(open(self.pucks_file, encoding="utf-8"))
            prob = dict(signal=self.signal[0], barcode=self.barcode[0], pucks=prob["pucks"])
            p = plan(self.world, prob, self.mode)
            lines, notes = to_lines(self.world, p, self.fine, self.skill_timeout, self.return_wait)
        except (MissionError, OSError, KeyError, ValueError) as e:
            res.success, res.message = False, str(e)
            self.get_logger().error(f"plan 실패: {e}")
            return res
        os.makedirs(self.log_dir, exist_ok=True)
        dump = lambda name, obj: json.dump(obj, open(os.path.join(self.log_dir, name), "w", encoding="utf-8"), ensure_ascii=False, indent=1)  # noqa: E731
        dump("signal.json", dict(signal=p["signal"], barcode=p["barcode"], source="perception (/perception/signal CONFIRMED, /perception/barcode READ)"))
        dump("allocation.json", dict(groups=p["groups"], allocation=p["allocation"]))
        dump("plan.json", dict(p, mission_lines=lines, notes=notes))
        open(os.path.join(self.log_dir, "mission_lines.txt"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
        for w in p["warnings"] + notes:
            self.get_logger().warning(w)
        self.get_logger().info(f"plan: {p['signal']['sequence']} {p['barcode']['code']} -> "
                               + ", ".join(f"{L}={v['group']}:{v['ids']}" for L, v in p["allocation"].items())
                               + f", 단계 {len(lines)}, 비용 {p['cost_m']:.1f}")
        res.success, res.message = True, "\n".join(lines)
        return res


def main():
    rclpy.init()
    n = PlannerNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
