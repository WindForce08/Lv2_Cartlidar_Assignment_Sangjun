"""Cartographer 순수 위치 추정 시작. cartographer_node 를 -load_state_filename … -load_frozen_state true
-start_trajectory_with_default_topics false 로 띄운 뒤, 이 노드가 /start_trajectory 로 새 궤적을 '알려 준 출발 자세'에서 시작시킨다.
출발 자세는 발제 좌표(start_x/y/yaw)로 받고, mission_from_carto_map(align_x/y/yaw, 1단계 지도 저장 때 도면 맞춤 값)으로 Cartographer map 좌표로 바꾼다.
Cartographer 의 initial_pose 는 '기준 궤적(frozen 0)의 첫 노드 전역 자세'에 대한 상대 자세라서(pose_graph_2d.cc ComputeLocalToGlobalTransform ·
GetInterpolatedGlobalTrajectoryPose, 시각 0 → 첫 노드), /trajectory_query 로 첫 노드 자세를 읽어 상대 자세로 바꿔 보낸다. 보낸 뒤 끝남."""
import math

import rclpy
from cartographer_ros_msgs.srv import StartTrajectory, TrajectoryQuery
from rclpy.node import Node


def compose(a, b):
    """a ∘ b (x, y, yaw)"""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], math.atan2(math.sin(a[2] + b[2]), math.cos(a[2] + b[2])))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), -a[2])


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class CartoStart(Node):
    def __init__(self):
        super().__init__('carto_start')
        for k, v in [('carto_config_dir', ''), ('carto_config', 'carto_localization.lua'), ('frozen_trajectory_id', 0),
                     ('align_x', 0.0), ('align_y', 0.0), ('align_yaw', 0.0), ('start_x', 0.0), ('start_y', 0.0), ('start_yaw', 0.0),
                     ('wait_s', 120.0)]:
            self.declare_parameter(k, v)
        self.g = lambda k: self.get_parameter(k).value
        self.query = self.create_client(TrajectoryQuery, '/trajectory_query')
        self.start = self.create_client(StartTrajectory, '/start_trajectory')

    def call(self, cli, req):
        if not cli.wait_for_service(timeout_sec=self.g('wait_s')):
            raise RuntimeError(f'{cli.srv_name} 서비스 없음')
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        if fut.result() is None:
            raise RuntimeError(f'{cli.srv_name} 응답 없음')
        return fut.result()

    def first_node_pose(self, tid):
        r = self.call(self.query, TrajectoryQuery.Request(trajectory_id=tid))
        if not r.trajectory:
            raise RuntimeError(f'궤적 {tid} 노드 0개: {r.status.message}')
        p = r.trajectory[0].pose
        return (p.position.x, p.position.y, yaw_of(p.orientation)), len(r.trajectory)

    def start_at(self, carto_pose):
        """Cartographer map 좌표 자세에서 새 궤적 시작. 돌려줌: (새 궤적 id, 상태 코드, 메시지)"""
        tid = self.g('frozen_trajectory_id')
        first, n = self.first_node_pose(tid)
        rel = compose(inverse(first), carto_pose)
        req = StartTrajectory.Request(configuration_directory=self.g('carto_config_dir'), configuration_basename=self.g('carto_config'),
                                      use_initial_pose=True, relative_to_trajectory_id=tid)
        req.initial_pose.position.x, req.initial_pose.position.y = rel[0], rel[1]
        req.initial_pose.orientation.z, req.initial_pose.orientation.w = math.sin(rel[2] / 2), math.cos(rel[2] / 2)
        r = self.call(self.start, req)
        self.get_logger().info(f'기준 궤적 {tid}(노드 {n}) 첫 노드 {tuple(round(v, 4) for v in first)}, 상대 {tuple(round(v, 4) for v in rel)} '
                               f'→ 새 궤적 {r.trajectory_id}, 상태 {r.status.code} {r.status.message}')
        return r.trajectory_id, r.status.code, r.status.message


def main():
    rclpy.init()
    n = CartoStart()
    g = n.g
    align = (g('align_x'), g('align_y'), g('align_yaw'))
    start_mission = (g('start_x'), g('start_y'), g('start_yaw'))
    carto = compose(inverse(align), start_mission)
    n.get_logger().info(f'출발 발제 {start_mission} → Cartographer map {tuple(round(v, 4) for v in carto)}')
    try:
        _, code, _ = n.start_at(carto)
    except RuntimeError as e:
        n.get_logger().error(str(e))
        code = -1
    n.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if code == 0 else 1)


if __name__ == '__main__':
    main()
