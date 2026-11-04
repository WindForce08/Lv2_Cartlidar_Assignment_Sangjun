"""map→odom 중계(Cartographer 구성). Cartographer /tracked_pose(carto_map→base_link, 스캔 시각 stamp, use_pose_extrapolator false)와
같은 시각의 odom→base_link(팀 EKF, TF)로 map→odom = A · T(carto_map←base, t) · T(odom←base, t)⁻¹ 을 계산하고, 그 값을 'now + future_dt' stamp 로
/tf 에 낸다(rate Hz · 새 값 올 때마다). A = map(발제 좌표) ← carto_map 지도 맞춤 값(align_x/y/yaw).
AMCL 의 transform_tolerance 와 같은 방식: map→odom 은 천천히 변하므로 앞당겨도 오차가 작고, odom→base_link 가 최신이면 tf2 의 지금 ·
TimePointZero 조회가 과거 자세를 주지 않는다. 마지막 계산이 max_age 초보다 오래되면 내지 않음."""
import math

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.time import Time


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class MapOdomRelay(Node):
    def __init__(self):
        super().__init__('map_odom_relay')
        for k, v in [('map_frame', 'map'), ('align_x', 0.0), ('align_y', 0.0), ('align_yaw', 0.0), ('odom_frame', 'odom'), ('base_frame', 'base_link'), ('pose_topic', '/tracked_pose'),
                     ('rate', 20.0), ('future_dt', 0.1), ('max_age', 1.0), ('lookup_timeout', 0.2)]:
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value
        self.mf, self.of, self.bf = g('map_frame'), g('odom_frame'), g('base_frame')
        self.future, self.max_age, self.timeout = g('future_dt'), g('max_age'), g('lookup_timeout')
        self.align = (g('align_x'), g('align_y'), g('align_yaw'))
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self, spin_thread=True)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.value, self.got = None, None                # (x, y, yaw), 계산한 시각(노드 시계)
        self.n_ok = self.n_fail = 0
        self.create_subscription(PoseStamped, g('pose_topic'), self.on_pose, 10)
        self.create_timer(1.0 / g('rate'), self.publish)
        self.create_timer(10.0, self.report)

    def on_pose(self, msg):
        try:
            tf = self.buf.lookup_transform(self.of, self.bf, Time.from_msg(msg.header.stamp), timeout=Duration(seconds=self.timeout))
        except tf2_ros.TransformException as e:
            self.n_fail += 1
            self.get_logger().warning(f'odom→base_link 를 스캔 시각에 못 찾음: {e}', throttle_duration_sec=5.0)
            return
        mx, my, myaw = msg.pose.position.x, msg.pose.position.y, yaw_of(msg.pose.orientation)
        t = tf.transform
        ox, oy, oyaw = t.translation.x, t.translation.y, yaw_of(t.rotation)
        yaw = myaw - oyaw
        c, s = math.cos(yaw), math.sin(yaw)
        x, y = mx - (c * ox - s * oy), my - (s * ox + c * oy)            # carto_map → odom
        ax, ay, ayaw = self.align
        c, s = math.cos(ayaw), math.sin(ayaw)
        yaw += ayaw
        self.value = (ax + c * x - s * y, ay + s * x + c * y, math.atan2(math.sin(yaw), math.cos(yaw)))
        self.got = self.get_clock().now()
        self.n_ok += 1
        self.publish()

    def publish(self):
        if self.value is None:
            return
        now = self.get_clock().now()
        if (now - self.got).nanoseconds * 1e-9 > self.max_age:
            self.get_logger().warning('tracked_pose 가 끊김 — map→odom 을 내지 않음', throttle_duration_sec=5.0)
            return
        x, y, yaw = self.value
        m = TransformStamped()
        m.header.stamp = (now + Duration(seconds=self.future)).to_msg()
        m.header.frame_id, m.child_frame_id = self.mf, self.of
        m.transform.translation.x, m.transform.translation.y = x, y
        m.transform.rotation.z, m.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.br.sendTransform(m)

    def report(self):
        self.get_logger().info(f'map→odom 계산 {self.n_ok}, odom TF 못 찾음 {self.n_fail}')


def main():
    rclpy.init()
    try:
        rclpy.spin(MapOdomRelay())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        if rclpy.ok():                 # 종료 신호가 대기 집합을 만드는 중에 context 를 닫은 경우만 조용히(그 밖은 그대로 오류)
            raise


if __name__ == '__main__':
    main()
