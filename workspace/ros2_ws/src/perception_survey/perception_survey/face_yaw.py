"""서 있는 직육면체 퍽의 면 요 (perception/include/perception/face_yaw.hpp 와 같은 식, ROS 의존 없음 → 단위 시험).
/perception/pucks 를 받는 Python 노드(mission 등)가 import 해서 쓴다.
정의: pose.orientation 의 x 축을 그 frame(base_link) xy 평면에 투영한 각을 90° 주기로 접은 값 [−π/4, π/4).
대상: shape = SHAPE_CUBOID, standing, orientation_valid. x 축이 거의 수직이면 None."""
import math


def fold_quarter(a):
    """각을 90° 주기로 접어 [−π/4, π/4)."""
    return (a + math.pi / 4) % (math.pi / 2) - math.pi / 4


def face_yaw(qx, qy, qz, qw, min_horizontal=0.5):
    """쿼터니언 → 면 요 [rad] 또는 None."""
    ax = 1 - 2 * (qy * qy + qz * qz)
    ay = 2 * (qx * qy + qz * qw)
    if math.hypot(ax, ay) < min_horizontal:
        return None
    return fold_quarter(math.atan2(ay, ax))


def face_yaw_error(face, reference):
    """면 요와 기준 방향(예: J1 → 퍽 방향각)의 차이 [rad], [−π/4, π/4)."""
    return fold_quarter(face - reference)
