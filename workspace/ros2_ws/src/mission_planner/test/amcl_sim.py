"""AMCL 확인용 가짜 로봇 (ROS 2, 시험 도구 - 실물 · Gazebo 대신).

참값 자세가 계획 경로 (plan.json 의 navigate waypoints) 를 따라 움직이고,
  /clock (빠르게), TF odom -> base_link (미끄럼 · 눈금 오차가 쌓이는 오도메트리), /scan (SDF 원래 형상에 광선 추적, base_link 원점, 360 개)
을 낸다. 스캔에는 지도에 없는 박스 위 퍽 (스캔면 높이를 지나는 원기둥 12 개) 도 넣는다.
/amcl_pose 를 받아 참값과 비교해 오차 통계를 출력하고 끝낸다.

  python3 amcl_sim.py --plan plan.json --sdf worlds/dice_mission.sdf [--speedup 4]
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(__file__), "..")))
from mission_planner.arena import LIDAR_Z, load_sdf  # noqa: E402
from mission_planner.poses import PUCK_DX, START_BOXES  # noqa: E402


def segments(sdf):
    segs = []
    for s in load_sdf(sdf):
        if s.z0 <= LIDAR_Z <= s.z1:
            fp = s.footprint(24)
            segs += [(fp[k], fp[(k + 1) % len(fp)]) for k in range(len(fp))]
    for (cx, cy) in START_BOXES.values():  # 박스 위 퍽 (지도에 없음): 원기둥 r 15 mm, 바닥 0.07 ~ 0.13 -> 스캔면 지남
        for sx in (-1, 1):
            a = np.linspace(0, 2 * math.pi, 12, endpoint=False)
            pts = np.c_[cx + sx * PUCK_DX + 0.015 * np.cos(a), cy + 0.015 * np.sin(a)]
            segs += [(pts[k], pts[(k + 1) % 12]) for k in range(12)]
    A = np.array([s[0] for s in segs])
    B = np.array([s[1] for s in segs])
    return A, B


def raycast(A, B, x, y, angles, rmax=8.0):
    d = np.c_[np.cos(angles), np.sin(angles)]            # (n, 2)
    e = B - A                                             # (m, 2)
    w = A - np.array([x, y])                              # (m, 2)
    den = d[:, None, 0] * e[None, :, 1] - d[:, None, 1] * e[None, :, 0]
    t = (w[None, :, 0] * e[None, :, 1] - w[None, :, 1] * e[None, :, 0]) / np.where(np.abs(den) < 1e-12, np.nan, den)
    u = (w[None, :, 0] * d[:, None, 1] - w[None, :, 1] * d[:, None, 0]) / np.where(np.abs(den) < 1e-12, np.nan, den)
    t = np.where((t > 0.01) & (u >= 0) & (u <= 1), t, np.inf)
    r = np.nanmin(t, axis=1)
    return np.minimum(r, rmax + 1)


def trajectory(plan, home, v=0.25, w=1.0, dt=0.05):
    """navigate waypoints 를 따라 (직선 v, 제자리 회전 w) 참값 자세 목록."""
    pts = [home]
    for s in plan["steps"]:
        if s["type"] == "navigate" and s["waypoints"]:
            wp = s["waypoints"][1:] + [s["stage"][:2]]
            for p in wp:
                pts.append((p[0], p[1], None))
            pts.append((s["stage"][0], s["stage"][1], s["stage"][2]))
    out = [home]
    x, y, th = home
    for px, py, pyaw in pts[1:]:
        dist = math.hypot(px - x, py - y)
        if dist > 1e-3:
            tgt = math.atan2(py - y, px - x)
            while abs(math.remainder(tgt - th, 2 * math.pi)) > 1e-3:
                dth = math.remainder(tgt - th, 2 * math.pi)
                th += max(-w * dt, min(w * dt, dth))
                out.append((x, y, th))
            n = int(dist / (v * dt)) + 1
            for k in range(1, n + 1):
                out.append((x + (px - x) * k / n, y + (py - y) * k / n, th))
            x, y = px, py
        if pyaw is not None:
            while abs(math.remainder(pyaw - th, 2 * math.pi)) > 1e-3:
                dth = math.remainder(pyaw - th, 2 * math.pi)
                th += max(-w * dt, min(w * dt, dth))
                out.append((x, y, th))
    return out, dt


class Sim(Node):
    def __init__(self, a):
        super().__init__("amcl_sim")
        self.A, self.B = segments(a.sdf)
        plan = json.load(open(a.plan, encoding="utf-8"))
        self.traj, self.dt = trajectory(plan, (0.69365, 0.95, math.pi))
        if a.max_steps:
            self.traj = self.traj[:a.max_steps]
        self.k = 0
        self.t = 0.0
        self.rng = np.random.default_rng(1)
        self.odom = np.array(self.traj[0], dtype=float)  # 오도메트리 = 참값에서 시작, 오차 쌓임
        self.odom_scale = (1.0 + a.trans_err, 1.0 + a.rot_err)  # 눈금 오차 (미끄럼)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.tfb = TransformBroadcaster(self)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.errs, self.odom_errs, self.truth_at = [], [], {}
        self.angles = np.linspace(-math.pi, math.pi, 360, endpoint=False)
        self.timer = self.create_timer(self.dt / a.speedup, self.step)
        self.warm = int(3.0 / self.dt)  # 처음 3 s 는 제자리 (AMCL 이 첫 스캔을 받게)

    def stamp(self):
        from builtin_interfaces.msg import Time
        return Time(sec=int(self.t), nanosec=int((self.t % 1) * 1e9))

    def step(self):
        if self.warm > 0:
            self.warm -= 1
            k = 0
        else:
            self.k += 1
            k = self.k
            if k >= len(self.traj):
                self.finish()
                return
            # 오도메트리: 참값 증분을 로봇 좌표로 바꿔 눈금 오차 + 잡음을 넣어 쌓음
            x0, y0, t0 = self.traj[k - 1]
            x1, y1, t1 = self.traj[k]
            c, s = math.cos(t0), math.sin(t0)
            dfx, dfy = (x1 - x0) * c + (y1 - y0) * s, -(x1 - x0) * s + (y1 - y0) * c
            dth = math.remainder(t1 - t0, 2 * math.pi)
            dfx = dfx * self.odom_scale[0] + self.rng.normal(0, 0.002) * abs(dfx) / max(1e-6, 0.25 * self.dt)
            dth = dth * self.odom_scale[1] + self.rng.normal(0, 0.003) * (abs(dth) > 1e-6)
            ox, oy, oth = self.odom
            self.odom = np.array([ox + dfx * math.cos(oth) - dfy * math.sin(oth), oy + dfx * math.sin(oth) + dfy * math.cos(oth), oth + dth])
        self.t += self.dt
        st = self.stamp()
        self.clock_pub.publish(Clock(clock=st))
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = st, "odom", "base_link"
        tf.transform.translation.x, tf.transform.translation.y = float(self.odom[0]), float(self.odom[1])
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(self.odom[2] / 2), math.cos(self.odom[2] / 2)
        self.tfb.sendTransform(tf)
        tx, ty, tth = self.traj[k]
        self.truth_at[round(self.t, 3)] = (tx, ty, tth)
        odom_err = math.hypot(*(np.array(self.traj[k][:2]) - self.odom[:2]))  # 오도메트리만 (map=odom 가정) 의 오차
        self.odom_errs.append(odom_err)
        if int(round(self.t / self.dt)) % 2 == 0:  # 10 Hz 스캔
            r = raycast(self.A, self.B, tx, ty, self.angles + tth)
            r = r + self.rng.normal(0, 0.01, r.shape)
            m = LaserScan()
            m.header.stamp, m.header.frame_id = st, "base_link"
            m.angle_min, m.angle_max, m.angle_increment = -math.pi, math.pi - 2 * math.pi / 360, 2 * math.pi / 360
            m.range_min, m.range_max = 0.12, 8.0
            m.ranges = [float(v) if v <= 8.0 else float("inf") for v in r]
            self.scan_pub.publish(m)

    def on_amcl(self, msg):
        t = round(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, 3)
        tr = self.truth_at.get(t)
        if tr is None:
            return
        p = msg.pose.pose
        yaw = 2 * math.atan2(p.orientation.z, p.orientation.w)
        self.errs.append((t, math.hypot(p.position.x - tr[0], p.position.y - tr[1]), abs(math.remainder(yaw - tr[2], 2 * math.pi))))

    def finish(self):
        e = np.array([[x[1], x[2]] for x in self.errs]) if self.errs else np.zeros((0, 2))
        res = dict(updates=len(self.errs), duration_s=round(self.t, 1),
                   amcl_pos_err_mm=dict(mean=round(float(e[:, 0].mean()) * 1000, 1), p95=round(float(np.percentile(e[:, 0], 95)) * 1000, 1), max=round(float(e[:, 0].max()) * 1000, 1)) if len(e) else None,
                   amcl_yaw_err_deg=dict(mean=round(math.degrees(float(e[:, 1].mean())), 2), p95=round(math.degrees(float(np.percentile(e[:, 1], 95))), 2), max=round(math.degrees(float(e[:, 1].max())), 2)) if len(e) else None,
                   odom_only_err_mm=dict(final=round(self.odom_errs[-1] * 1000, 1), max=round(max(self.odom_errs) * 1000, 1)))
        print("[AMCL_SIM] " + json.dumps(res, ensure_ascii=False), flush=True)
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", required=True)
    ap.add_argument("--sdf", required=True)
    ap.add_argument("--speedup", type=float, default=4.0)
    ap.add_argument("--trans_err", type=float, default=0.02, help="직진 눈금 오차 (미끄럼)")
    ap.add_argument("--rot_err", type=float, default=0.06, help="회전 눈금 오차 (스키드 조향 미끄럼)")
    ap.add_argument("--max_steps", type=int, default=0)
    a = ap.parse_args()
    rclpy.init()
    n = Sim(a)
    try:
        rclpy.spin(n)
    except Exception:  # noqa: BLE001
        pass


if __name__ == "__main__":
    main()
