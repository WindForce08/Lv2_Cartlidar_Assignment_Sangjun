"""위치 잃음 감지 · 다시 찾기. 계산은 loc_core.py. backend = carto(Cartographer 순수 위치 추정) | amcl.
상태(/localization/status, std_msgs/String, Reliable · TRANSIENT_LOCAL depth 1, 10 Hz · 바뀔 때): OK · SUSPECT · LOST · RECOVERING · CONFIRM · FAILED.
/localization/lock(std_msgs/Bool, 같은 QoS): OK · SUSPECT 가 아니면 true — twist_mux lock 입력용.
감지: ① 스캔–지도 잔차(추정 자세 map→base_link 에서 두 라이다 끝점의 거리장 값, res_mode) > res_thr 가 suspect_s 이어지면 SUSPECT, lost_s 이어지면 LOST
      ② 위치 추정 자세(pose_topic) 증분과 EKF odom 증분 차 > jump_xy 또는 jump_yaw → LOST ③ pose_topic 이 gap_s 넘게 끊김 → LOST.
대응: OK · SUSPECT 가 아니면 명령 게이트(gate_in → gate_out, gate_in 이 none 이면 끔)가 속도 0 을 냄(20 Hz). LOST → RECOVERING(작업 스레드):
      마지막 정상 자세 ∘ (그 뒤 odom 이동) 둘레 근처 탐색 → 받아들이지 못하면 지도 전체 탐색. 받아들임 = 점수 ≤ accept_score 이고
      '다른 자리' 2등이 없거나 2등 − 1등 ≥ ambiguity_margin. 받아들이면 다시 넣기 → CONFIRM
      (carto: /finish_trajectory → /start_trajectory(initial_pose), amcl: /initialpose)
      (잔차 ≤ res_thr 가 ok_confirm_s 이어지면 OK, confirm_timeout_s 안에 안 되면 다시 LOST). 못 찾으면 retry_s 뒤 새 스캔으로 다시,
      max_recover_s 넘으면 FAILED(정지 유지 · 오류 기록). 사건은 event_log(jsonl)에도 씀."""
import json
import math
import threading
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from localization import loc_core as lc

OK_STATES = ('OK', 'SUSPECT')


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def tf_pose(t):
    return (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], math.atan2(math.sin(a[2] + b[2]), math.cos(a[2] + b[2])))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), -a[2])


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class LocMonitor(Node):
    def __init__(self):
        super().__init__('loc_monitor')
        P = [('backend', 'carto'), ('pose_topic', ''),             # pose_topic 빈 값 = carto /tracked_pose, amcl /amcl_pose
             ('map', ''), ('mission_frame', 'map'), ('odom_frame', 'odom'), ('base_frame', 'base_link'),
             ('scan_topics', ['/scan_front_filtered', '/scan_rear_filtered']), ('r_min', 0.15), ('r_max', 3.5), ('n_points', 90),
             ('res_r_max', 2.0), ('startup_grace_s', 15.0), ('tf_wait_s', 0.3),   # 스캔 시각 TF 가 아직 안 온 짝을 다음 스캔 때 다시 볼 최대 시간(모의 · 실물 시각)
             ('res_mode', 'cell_median'), ('check_mode', 'front'),   # 'interp_p75' · 'pair'(앞 · 뒤 중 나중에 온 스캔에서 짝 검사 → 스캔 주기 가까이)
             ('res_thr', 0.03), ('suspect_s', 0.5), ('lost_s', 2.0), ('jump_xy', 0.10), ('jump_yaw_deg', 5.0), ('gap_s', 0.5),
             ('gate_in', '/cmd_vel_gate_in'), ('gate_out', '/cmd_vel_nav'),
             ('initialpose_xy_sd', 0.05), ('initialpose_yaw_sd_deg', 5.0),
             ('carto_config_dir', ''), ('carto_config', 'carto_localization.lua'), ('frozen_trajectory_id', 0), ('trajectory_id', 1),
             ('align_x', 0.0), ('align_y', 0.0), ('align_yaw', 0.0),
             ('accept_score', 0.02), ('ambiguity_margin', 0.005), ('local_lin', 0.5), ('local_ang_deg', 30.0),
             ('retry_s', 2.0), ('max_recover_s', 60.0), ('ok_confirm_s', 1.0), ('confirm_timeout_s', 10.0), ('event_log', '')]
        for k, v in P:
            self.declare_parameter(k, v)
        g = self.g = lambda k: self.get_parameter(k).value
        self.field = lc.DistField(g('map'))
        self.align = (g('align_x'), g('align_y'), g('align_yaw'))
        self.cb = ReentrantCallbackGroup()
        self.buf = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.listener = tf2_ros.TransformListener(self.buf, self, spin_thread=True)
        self.state, self.state_t = 'OK', self.now_s()
        self.bad_since = None
        self.last_good = None            # (stamp, mission 자세, odom 자세)
        self.last_tracked = None         # (stamp, map 자세, odom 자세)
        self.last_tracked_rx = None      # 노드 시계
        self.confirm_since, self.confirm_ok_since = None, None
        self.recover_since = None
        self.traj = g('trajectory_id')
        self.scans = {}                  # topic → (stamp, pts)
        self.pending = []                # TF 가 아직 안 와 검사를 미룬 스캔 짝(오래된 것부터)
        self.T_laser = {}
        self.lock = threading.Lock()
        self.lost_lock = threading.Lock()      # lost() 가 여러 콜백 스레드에서 동시에 불려 작업 스레드를 두 번 만들지 않게
        self.t_start = None
        self.worker = None
        self.log_f = open(g('event_log'), 'a') if g('event_log') not in ('', 'none') else None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.backend = g('backend')
        if self.backend not in ('carto', 'amcl'):
            raise ValueError(f'backend {self.backend} — carto | amcl')
        self.pub_status = self.create_publisher(String, '/localization/status', latched)    # perception_survey 가 받음
        self.pub_lock = self.create_publisher(Bool, '/localization/lock', latched)         # twist_mux lock 용: OK · SUSPECT 가 아니면 true
        self.gate = g('gate_in') not in ('', 'none')
        if self.gate:
            self.pub_cmd = self.create_publisher(TwistStamped, g('gate_out'), 10)
            self.create_subscription(TwistStamped, g('gate_in'), self.on_cmd, 10, callback_group=self.cb)
            self.create_timer(0.05, self.gate_zero, callback_group=self.cb)
        for tp in g('scan_topics'):
            self.create_subscription(LaserScan, tp, lambda m, tp=tp: self.on_scan(tp, m), qos_profile_sensor_data, callback_group=self.cb)
        pose_topic = g('pose_topic') or ('/tracked_pose' if self.backend == 'carto' else '/amcl_pose')
        if self.backend == 'carto':
            from cartographer_ros_msgs import srv as carto_srv   # carto 일 때만 불러옴(AMCL 만 쓰는 기계에는 cartographer_ros 가 없을 수 있음)
            self.carto_srv = carto_srv
            self.create_subscription(PoseStamped, pose_topic, self.on_tracked, 10, callback_group=self.cb)
            self.cli_q = self.create_client(self.carto_srv.TrajectoryQuery, '/trajectory_query', callback_group=self.cb)
            self.cli_fin = self.create_client(self.carto_srv.FinishTrajectory, '/finish_trajectory', callback_group=self.cb)
            self.cli_start = self.create_client(self.carto_srv.StartTrajectory, '/start_trajectory', callback_group=self.cb)
        else:                                  # amcl_pose: map 좌표, 스캔 시각. 다시 넣기는 /initialpose
            self.create_subscription(PoseWithCovarianceStamped, pose_topic, self.on_tracked, 10, callback_group=self.cb)
            self.pub_init = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_timer(0.1, self.tick, callback_group=self.cb)
        self.res_win, self.n_skip = [], 0
        self.create_timer(10.0, self.report, callback_group=self.cb)
        self.event('start', backend=self.backend, gate=self.gate, traj=self.traj, res_thr=g('res_thr'), res_mode=g('res_mode'), check_mode=g('check_mode'))

    # ---- 공통 ----
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def event(self, kind, **kw):
        rec = {'t': round(self.now_s(), 3), 'event': kind, 'state': getattr(self, 'state', None)} | kw
        self.get_logger().info(json.dumps(rec, ensure_ascii=False))
        if self.log_f:
            self.log_f.write(json.dumps(rec, ensure_ascii=False) + '\n')
            self.log_f.flush()

    def set_state(self, s, **kw):
        with self.lock:
            if s == self.state:
                return
            old, self.state, self.state_t = self.state, s, self.now_s()
        self.event('state', old=old, new=s, **kw)
        self.pub_status.publish(String(data=s))
        self.pub_lock.publish(Bool(data=s not in OK_STATES))

    def lookup(self, parent, child, stamp, timeout=0.0):
        """콜백 안에서는 기다리지 않는다(timeout 0). 기다리면 코어가 적은 기계(Jetson)에서 실행기가 막혀 TF 수신이 밀리고 검사가 모두 실패한다(11-04 Jetson 부하 측정).
        다시 찾기 작업 스레드만 timeout 을 준다."""
        try:
            return tf_pose(self.buf.lookup_transform(parent, child, stamp, timeout=Duration(seconds=timeout)))
        except tf2_ros.TransformException:
            return None

    # ---- 명령 게이트 ----
    def on_cmd(self, m):
        if self.state not in OK_STATES:
            m.twist.linear.x = m.twist.linear.y = m.twist.angular.z = 0.0
        self.pub_cmd.publish(m)

    def gate_zero(self):
        if self.state in OK_STATES:
            return
        z = TwistStamped()
        z.header.stamp = self.get_clock().now().to_msg()
        z.header.frame_id = self.g('base_frame')
        self.pub_cmd.publish(z)

    # ---- 감지 ----
    def on_scan(self, topic, m):
        if topic not in self.T_laser:                        # 정적 · 3차원(라이다 roll π 장착) → 2×3
            try:
                t = self.buf.lookup_transform(self.g('base_frame'), m.header.frame_id, Time()).transform   # 아직 없으면 다음 스캔 때 다시
            except tf2_ros.TransformException:
                return
            self.T_laser[topic] = lc.T2d_from_tf(t.translation.x, t.translation.y, t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        st = Time.from_msg(m.header.stamp)
        pts = lc.scan_points(m.ranges, m.angle_min, m.angle_increment, self.T_laser[topic], self.g('r_min'), self.g('r_max'), self.g('n_points'))
        near = lc.scan_points(m.ranges, m.angle_min, m.angle_increment, self.T_laser[topic], self.g('r_min'), self.g('res_r_max'), self.g('n_points'))
        front = topic == self.g('scan_topics')[0]
        with self.lock:
            self.scans[topic] = (st, pts, near)
            others = [v for k, v in self.scans.items() if k != topic]
            if self.g('check_mode') == 'pair' and others and abs((st - others[0][0]).nanoseconds) * 1e-9 <= 0.06:
                key = (st if front else others[0][0]).nanoseconds         # 같은 짝(앞 stamp)을 두 번 검사하지 않음
                if key == getattr(self, 'last_pair', None):
                    return
                self.last_pair = key
        if self.g('check_mode') == 'pair':     # 앞 · 뒤 어느 쪽이 나중에 와도 짝이 맞으면 검사(옛 방식은 앞이 먼저 오면 지난 뒤 스캔과 0.1 s 차라 대부분 버림 → 약 1.1 Hz)
            if not others or abs((st - others[0][0]).nanoseconds) * 1e-9 > 0.06:
                return
            self.queue_check([(st, near), (others[0][0], others[0][2])] if front else [(others[0][0], others[0][2]), (st, near)], st)
            return
        if not front or not others:   # 앞 스캔이 올 때 둘을 합쳐 판단(뒤 스캔은 같은 주기 것)
            return
        if abs((st - others[0][0]).nanoseconds) * 1e-9 > 0.06:
            return
        self.queue_check([(st, near), (others[0][0], others[0][2])], st)

    def queue_check(self, scans, newest):
        """짝을 줄에 넣고, 스캔 시각 TF 가 들어온 짝부터 검사. tf_wait_s 넘게 TF 가 안 오면 건너뜀(n_skip)."""
        self.pending.append(scans)
        keep = []
        for sc in self.pending:
            if keep or not self.check(sc):        # 앞 짝이 아직이면 뒤 짝도 기다림(시각 순서 유지)
                if (newest - sc[0][0]).nanoseconds * 1e-9 > self.g('tf_wait_s'):
                    self.n_skip += 1
                else:
                    keep.append(sc)
        self.pending = keep

    def check(self, scans):
        """scans = [(stamp, 잔차용 점), …] — 스캔마다 자기 stamp 의 자세로 놓음(회전 중 앞 스캔 자세 하나로 뒤 스캔까지 놓으면 끝점이 밀림).
        잔차용 점은 res_r_max(2.0 m) 안만: 실물 LDS 한 바퀴(0.1 s) 쓸림이 먼 점일수록 커서. Isaac 라이다는 쓸림을 흉내 내지 않음(한계)."""
        st = scans[0][0]
        poses = [self.lookup(self.g('mission_frame'), self.g('base_frame'), t) for t, _ in scans]
        odom = self.lookup(self.g('odom_frame'), self.g('base_frame'), st)
        if any(p is None for p in poses) or odom is None:
            return False                          # TF 가 아직 — queue_check 가 다음 스캔 때 다시
        if sum(len(q) for _, q in scans) < 20:
            self.n_skip += 1
            return True
        pose = poses[0]
        r = lc.residual_multi(self.field, [(p, q) for p, (_, q) in zip(poses, scans)], self.g('res_mode'))
        self.res_win.append(r)
        t = st.nanoseconds * 1e-9
        bad = r > self.g('res_thr')
        s = self.state
        if s in OK_STATES:
            if not bad:
                self.bad_since = None
                self.last_good = (t, pose, odom)
                self.set_state('OK', residual=round(r, 4))
            else:
                self.bad_since = self.bad_since or t
                d = t - self.bad_since
                if d >= self.g('lost_s'):
                    self.lost('residual', residual=round(r, 4), bad_s=round(d, 2))
                elif d >= self.g('suspect_s'):
                    self.set_state('SUSPECT', residual=round(r, 4), bad_s=round(d, 2))
        elif s == 'CONFIRM':
            if bad:
                self.confirm_ok_since = None
            else:
                self.confirm_ok_since = self.confirm_ok_since or t
                if t - self.confirm_ok_since >= self.g('ok_confirm_s'):
                    self.bad_since = None
                    self.last_good = (t, pose, odom)
                    self.set_state('OK', residual=round(r, 4), recover_s=round(self.now_s() - self.recover_since, 2))
        return True

    def on_tracked(self, m):
        self.last_tracked_rx = self.now_s()
        st = Time.from_msg(m.header.stamp)
        odom = self.lookup(self.g('odom_frame'), self.g('base_frame'), st)
        q = m.pose.pose if self.backend == 'amcl' else m.pose
        p = (q.position.x, q.position.y, yaw_of(q.orientation))
        prev, self.last_tracked = self.last_tracked, (st, p, odom)
        if prev is None or odom is None or prev[2] is None or self.state not in OK_STATES:
            return
        dc = compose(inverse(prev[1]), p)           # 위치 추정 증분(차체 기준)
        do = compose(inverse(prev[2]), odom)        # EKF odom 증분
        ex, ey, eyaw = dc[0] - do[0], dc[1] - do[1], wrap(dc[2] - do[2])
        if math.hypot(ex, ey) > self.g('jump_xy') or abs(math.degrees(eyaw)) > self.g('jump_yaw_deg'):
            self.lost('jump', dxy=round(math.hypot(ex, ey), 3), dyaw_deg=round(math.degrees(eyaw), 2))

    def tick(self):
        self.pub_status.publish(String(data=self.state))
        self.pub_lock.publish(Bool(data=self.state not in OK_STATES))
        now = self.now_s()
        if now <= 0.0:                                         # 모의 시계 전
            return
        self.t_start = self.t_start or now
        if self.state in OK_STATES and self.last_tracked_rx is not None and now - self.last_tracked_rx > self.g('gap_s'):
            self.lost('gap', gap_s=round(now - self.last_tracked_rx, 2))
        if self.state in OK_STATES and self.last_tracked_rx is None and now - self.t_start > self.g('startup_grace_s'):
            self.lost('no_tracked_pose', after_s=round(now - self.t_start, 2))   # 시작 뒤 한 번도 안 옴
        if self.state == 'CONFIRM' and now - self.confirm_since > self.g('confirm_timeout_s'):
            self.event('confirm_timeout')
            self.lost('confirm_timeout')

    def report(self):
        w, self.res_win = self.res_win, []
        q = (lambda p: round(float(np.percentile(w, p)), 4)) if w else (lambda p: None)
        self.event('report', n_check=len(w), n_skip=self.n_skip, res_p50=q(50), res_p95=q(95), res_max=q(100), traj=self.traj)
        self.n_skip = 0

    # ---- 다시 찾기 ----
    def lost(self, why, **kw):
        with self.lost_lock:
            if self.state in ('LOST', 'RECOVERING', 'FAILED'):
                return
            if self.recover_since is None or self.state in OK_STATES:
                self.recover_since = self.now_s()
            self.set_state('LOST', why=why, **kw)
            if self.worker is None or not self.worker.is_alive():
                self.worker = threading.Thread(target=self.recover, daemon=True)
                self.worker.start()

    def latest_pts(self):
        with self.lock:
            v = list(self.scans.values())
        if len(v) < 2 or abs((v[0][0] - v[1][0]).nanoseconds) * 1e-9 > 0.06:
            return None, None
        return v[0][0], np.concatenate([v[0][1], v[1][1]])     # 다시 찾기는 차체가 멈춘 상태(게이트 0)라 한 자세로 놓아도 됨

    def recover(self):
        self.set_state('RECOVERING')
        while rclpy.ok():
            if self.now_s() - self.recover_since > self.g('max_recover_s'):
                self.set_state('FAILED', recover_s=round(self.now_s() - self.recover_since, 2))
                return
            st, pts = self.latest_pts()
            cand = None
            if pts is not None:
                odom = self.lookup(self.g('odom_frame'), self.g('base_frame'), st, timeout=0.1)   # 작업 스레드라 기다려도 됨
                if self.last_good is not None and odom is not None:
                    center = compose(self.last_good[1], compose(inverse(self.last_good[2]), odom))
                    t0 = time.monotonic()
                    L = lc.search(self.field, pts, center=center, lin=self.g('local_lin'), ang=math.radians(self.g('local_ang_deg')))
                    self.event('search_local', center=[round(v, 3) for v in center], best=[round(v, 3) for v in L['best']],
                               score=round(L['score'], 4), second_score=L['second_score'], wall_s=round(time.monotonic() - t0, 2))
                    cand = L if self.accept(L) else None
                if cand is None:
                    t0 = time.monotonic()
                    G = lc.search(self.field, pts, center=None)
                    self.event('search_global', best=[round(v, 3) for v in G['best']], score=round(G['score'], 4),
                               second=G['second'], second_score=G['second_score'], n_cand=G['n_cand'], wall_s=round(time.monotonic() - t0, 2))
                    cand = G if self.accept(G) else None
            if cand is not None and self.restart(cand['best']):
                self.confirm_since, self.confirm_ok_since = self.now_s(), None
                self.set_state('CONFIRM', pose=[round(v, 3) for v in cand['best']], score=round(cand['score'], 4))
                return
            time.sleep(self.g('retry_s'))           # 벽시계 — 모의 시계가 멈춰도 다시 시도

    def accept(self, r):
        if r['score'] > self.g('accept_score'):
            return False
        return r['second_score'] is None or r['second_score'] - r['score'] >= self.g('ambiguity_margin')

    def call(self, cli, req, timeout=10.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.01)
        return fut.result() if fut.done() else None

    def restart(self, mission_pose):
        return self.restart_carto(mission_pose) if self.backend == 'carto' else self.restart_amcl(mission_pose)

    def restart_amcl(self, mission_pose):
        """AMCL 에 /initialpose(map = 발제 좌표)로 다시 넣음. 입자는 이 자세 둘레(initialpose_*_sd)로 다시 뿌려짐"""
        m = PoseWithCovarianceStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.g('mission_frame')
        m.pose.pose.position.x, m.pose.pose.position.y = mission_pose[0], mission_pose[1]
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(mission_pose[2] / 2), math.cos(mission_pose[2] / 2)
        sxy, syaw = self.g('initialpose_xy_sd'), math.radians(self.g('initialpose_yaw_sd_deg'))
        m.pose.covariance[0] = m.pose.covariance[7] = sxy * sxy
        m.pose.covariance[35] = syaw * syaw
        self.pub_init.publish(m)
        self.event('initialpose', mission_pose=[round(v, 4) for v in mission_pose])
        self.last_tracked = None
        return True

    def restart_carto(self, mission_pose):
        """Cartographer 궤적을 끝내고 mission_pose(발제 좌표)에서 새로 시작"""
        r = self.call(self.cli_fin, self.carto_srv.FinishTrajectory.Request(trajectory_id=self.traj))
        self.event('finish_trajectory', traj=self.traj, code=None if r is None else r.status.code, msg=None if r is None else r.status.message)
        q = self.call(self.cli_q, self.carto_srv.TrajectoryQuery.Request(trajectory_id=self.g('frozen_trajectory_id')))
        if q is None or not q.trajectory:
            self.event('restart_fail', why='trajectory_query')
            return False
        fp = q.trajectory[0].pose
        first = (fp.position.x, fp.position.y, yaw_of(fp.orientation))
        rel = compose(inverse(first), compose(inverse(self.align), mission_pose))
        req = self.carto_srv.StartTrajectory.Request(configuration_directory=self.g('carto_config_dir'), configuration_basename=self.g('carto_config'),
                                                     use_initial_pose=True, relative_to_trajectory_id=self.g('frozen_trajectory_id'))
        req.initial_pose.position.x, req.initial_pose.position.y = rel[0], rel[1]
        req.initial_pose.orientation.z, req.initial_pose.orientation.w = math.sin(rel[2] / 2), math.cos(rel[2] / 2)
        r = self.call(self.cli_start, req)
        if r is None or r.status.code != 0:
            self.event('restart_fail', why='start_trajectory', code=None if r is None else r.status.code, msg=None if r is None else r.status.message)
            return False
        self.event('start_trajectory', old=self.traj, new=r.trajectory_id, mission_pose=[round(v, 4) for v in mission_pose])
        self.traj = r.trajectory_id
        self.last_tracked = None
        return True


def main():
    rclpy.init()
    n = LocMonitor()
    ex = SingleThreadedExecutor()      # 콜백이 기다리지 않으므로 하나로 충분(다시 찾기는 따로 작업 스레드)
    ex.add_node(n)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        if rclpy.ok():                 # 종료 신호가 대기 집합을 만드는 중에 context 를 닫은 경우만 조용히(그 밖은 그대로 오류)
            raise


if __name__ == '__main__':
    main()
