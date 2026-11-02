"""waffle_4wd Isaac Sim 6.2 ↔ ros2_control 브리지 (Isaac Lab 없이 isaacsim 단독). 실행은 run_bridge.sh 로 (ROS 환경 변수).

    ./run_bridge.sh [--headless] [--usd <waffle_4wd.usda>] [--cams front,rear,arm] [--lidars front,rear]
    호스트: ros2 launch opencr_hardware base_bringup.launch.py sim:=true use_teleop:=false

호스트 ros2_control 의 OpenCRHardware 자리를 topic_based_ros2_control/TopicBasedSystem 이 맡고, 이 스크립트가 실물 OpenCR 처럼:
  /isaac_joint_states    (발행)  저장소 관절 이름 (robot.urdf.xacro) 으로 위치 · 속도 · 힘
  /isaac_joint_commands  (구독)  바퀴 velocity · 팔 position. 0.5 s 끊기면 바퀴 정지 (펌웨어 comm_timeout 과 같게)
  /clock · /imu/data (imu_link, 뒤집어 단 보드 축) · /e_stop false 10 Hz (실물은 OpenCRHardware 가 발행, twist_mux 잠금 해제)
  카메라 (--cams, 30 Hz): /top_camera_{front,rear}/color/image_raw · color/camera_info · aligned_depth_to_color/image_raw (32FC1 m),
      /arm_camera/image_raw · camera_info, /tf_static top_camera_*_link -> _color_optical_frame (실물은 realsense2_camera 가 냄)
  라이다 (--lidars, 10 Hz): /scan_front · /scan_rear (LaserScan, laser_front · laser_rear) — PhysX 레이캐스트, 로봇 자신은 맞지 않음

관절 대응 (USD waffle_4wd_usd_261027_1711 ↔ 저장소): 바퀴 축은 둘 다 +y. 팔은 USD 축이 모두 반대라 부호를 뒤집는다 (영점 자세 · 링크 길이는 같음).
gripper_roll (그리퍼 개폐 모터, -0.7946 열림 ~ 0.9541 닫힘) 은 손가락 위치로 직선 변환 (USD 미믹은 Newton 전용이라 두 손가락을 각각 명령).
USD 위치값은 저장소 URDF 와 base_link 원점이 다르다 — 로봇 TF 는 호스트 robot_state_publisher 가 저장소 URDF 로 만든다 (docs/frames_tf.md).
"""
import argparse
import math
import time

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="waffle_4wd Isaac Sim 6.2 ros2_control 브리지")
parser.add_argument("--usd", default="/home/pa8/Downloads/urdf_test/waffle_4wd_usd_261027_1711/waffle_4wd/waffle_4wd.usda")
parser.add_argument("--headless", action="store_true", help="창 없이")
parser.add_argument("--cmd_timeout", type=float, default=0.5, help="명령이 이만큼 [s] 끊기면 바퀴 정지 (0 = 끔)")
parser.add_argument("--cams", default="front,rear,arm", help="발행할 카메라 (front,rear,arm 중, 빈 값 = 없음)")
parser.add_argument("--lidars", default="front,rear", help="발행할 라이다 (front,rear 중, 빈 값 = 없음)")
parser.add_argument("--test_seconds", type=float, default=0.0, help="(점검용) 이 시뮬레이션 시간 [s] 뒤 종료")
args = parser.parse_args()
app = SimulationApp({"headless": args.headless})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics, UsdShade  # noqa: E402

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
from isaacsim.core.experimental.objects import GroundPlane  # noqa: E402
from isaacsim.core.experimental.prims import Articulation  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

app_utils.enable_extension("isaacsim.ros2.core")   # jazzy rclpy 를 sys.path 에 (run_bridge.sh 가 라이브러리 경로를 줌)
app.update()
import rclpy  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

PHYS_HZ = 120
WARMUP_FRAMES = 240
FRICTION = 1.0                                   # 바닥 · 로봇 정지 · 미끄럼 마찰 (urdfv2 robot_params.json 과 같게)
WHEEL_DAMPING = 0.48                             # 바퀴 속도 드라이브 [N·m·s/rad] (urdfv2 검증값)
ARM_STIFFNESS, ARM_DAMPING = 19.0, 0.85          # 팔 위치 드라이브 (urdfv2 검증값)
FINGER_STIFFNESS, FINGER_DAMPING = 200.0, 5.0
# 저장소 관절 이름 -> (USD 관절, 부호)
WHEELS = {"wheel_left_front_joint": "Wheel_Revolute_Front_Left", "wheel_left_rear_joint": "Wheel_Revolute_Rear_Left",
          "wheel_right_front_joint": "Wheel_Revolute_Front_Right", "wheel_right_rear_joint": "Wheel_Revolute_Rear_Right"}
ARM = {"joint1": ("RMX52_Base_Yaw", -1.0), "joint2": ("RMX52_Shoulder", -1.0), "joint3": ("RMX52_Elbow", -1.0),
       "joint4": ("RMX52_Wrist_Pitch", -1.0), "joint5": ("RMX52_Wrist_Roll_Reversed", -1.0)}
GRIPPER = "gripper_roll"
GRIP_OPEN_RAD, GRIP_CLOSED_RAD = -0.7946, 0.9541           # 저장소 gripper_roll 한계 (열림 1550 · 닫힘 2650 tick)
FINGER_OPEN, FINGER_CLOSED = 0.026186, -0.001414           # USD gripper_left_joint 한계 [m] (오른쪽은 부호 반대)
IMU_RPY = (math.pi, 0.0, math.pi / 2)                      # 저장소 imu_link (base_link 기준, 뒤집어 장착)
# 카메라 (docs/frames_tf.md "Isaac Sim 발행 규약"). prim 은 USD 광학 프레임 아래에 두고 ROS 광학 축 (z 앞 · y 아래) -> USD 카메라 (-z 앞 · y 위) 로 x 축 180° 돌림
CAM_HZ = 30.0
H_APERTURE = 20.955                                        # USD 카메라 기본 수평 조리개 [mm]
D435_RES, D435_FX = (480, 640), 615.0                      # 실물 perception 컬러 640x480, HFOV 약 55° (Isaac Lab 3 시험 fx 614.7)
ARM_RES, ARM_HFOV = (480, 640), 105.0                      # Arducam 640x480 (perception 설정 · 이슈 #14). 대각 120° 어안 -> 핀홀 HFOV 105° 근사 ★ 실물 보정값으로
# 이름 -> (USD 광학 프레임 prim, 토픽 앞부분, frame_id, 깊이)
CAMS = {"front": ("top_camera_front_color_optical_frame", "/top_camera_front", "top_camera_front_color_optical_frame", True),
        "rear": ("top_camera_rear_color_optical_frame", "/top_camera_rear", "top_camera_rear_color_optical_frame", True),
        "arm": ("camera_optical_frame", "/arm_camera", "arm_camera_optical_frame", False)}
# realsense 드라이버 대신 내는 top_camera_*_link (= infra1 원점, 몸체 축) -> _color_optical_frame. USD 261027 값 (공장 외부보정 포함)
D435_Q_LINK_OPT = (-0.5, 0.5, -0.5, 0.5)                   # link -> optical (w, x, y, z)
D435_T_OPT_INFRA1 = (-0.02502057515084745, 0.0, 0.0)       # optical (두 IR 가운데) -> infra1
D435_T_OPT_COLOR = (-0.04011553432792425, 0.0000701552780810744, -0.000182516494533047)
D435_Q_OPT_COLOR = (0.99999166, -0.0032025536, -0.0017622522, 0.001824102)
# 라이다 LDS-03 (lidar.yaml scan_merger 입력). 이름 -> (USD 스캔 prim, 토픽, frame_id). USD lidar_*_scan 은 URDF laser_* 처럼 roll π (거꾸로 매달림)
LIDARS = {"front": ("lidar_front_scan", "/scan_front", "laser_front"), "rear": ("lidar_rear_scan", "/scan_rear", "laser_rear")}
LIDAR_HZ, LIDAR_SAMPLES = 10.0, 360                        # ★ 실물 LDS-03 회전 주기 · 한 바퀴 점 수 확인 후 맞춤
LIDAR_RANGE = (0.05, 12.0)                                 # [m] ★ lidar.yaml range_max 12.0 과 같게
BASE_LINK = "base_link"                                    # USD 관절 루트 강체 (ArticulationRootAPI) 이름


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def quat_matrix(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def matrix_quat(m):
    """회전 행렬 -> (w, x, y, z)"""
    t = np.trace(m)
    if t > 0:
        s = 2.0 * math.sqrt(t + 1.0)
        return 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k])
    v = [0.0, 0.0, 0.0]
    v[i], v[j], v[k] = 0.25 * s, (m[j, i] + m[i, j]) / s, (m[k, i] + m[i, k]) / s
    return (m[k, j] - m[j, k]) / s, *v


def finger_from_grip(rad):
    s = (min(GRIP_CLOSED_RAD, max(GRIP_OPEN_RAD, rad)) - GRIP_OPEN_RAD) / (GRIP_CLOSED_RAD - GRIP_OPEN_RAD)
    return FINGER_OPEN + s * (FINGER_CLOSED - FINGER_OPEN)


def grip_from_finger(m):
    s = (m - FINGER_OPEN) / (FINGER_CLOSED - FINGER_OPEN)
    return GRIP_OPEN_RAD + s * (GRIP_CLOSED_RAD - GRIP_OPEN_RAD)


def find_prim(stage, name, root="/World/waffle"):
    for prim in stage.Traverse():
        if prim.GetName() == name and str(prim.GetPath()).startswith(root):
            return str(prim.GetPath())
    raise RuntimeError(f"USD 에 {name} prim 이 없습니다")


def add_camera(stage, parent, res, fx, near=0.01):
    """parent (ROS 광학 프레임) 아래 핀홀 카메라. 정사각 화소 (fy = fx), 주점은 영상 가운데"""
    h, w = res
    cam = UsdGeom.Camera.Define(stage, f"{parent}/isaac_camera")
    cam.CreateHorizontalApertureAttr(H_APERTURE)
    cam.CreateVerticalApertureAttr(H_APERTURE * h / w)
    cam.CreateFocalLengthAttr(fx * H_APERTURE / w)
    cam.CreateClippingRangeAttr(Gf.Vec2f(near, 100.0))
    UsdGeom.Xformable(cam).AddOrientOp().Set(Gf.Quatf(0.0, 1.0, 0.0, 0.0))
    return str(cam.GetPath())


def camera_info(res, fx, frame_id):
    h, w = res
    m = CameraInfo()
    m.header.frame_id = frame_id
    m.height, m.width = h, w
    m.distortion_model = "plumb_bob"
    m.d = [0.0] * 5
    m.k = [fx, 0.0, w / 2, 0.0, fx, h / 2, 0.0, 0.0, 1.0]
    m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    m.p = [fx, 0.0, w / 2, 0.0, 0.0, fx, h / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
    return m


def d435_link_to_color():
    """top_camera_*_link (infra1) -> _color_optical_frame: (평행 이동, (w, x, y, z))"""
    R_lo = quat_matrix(*D435_Q_LINK_OPT)
    t = R_lo @ (np.array(D435_T_OPT_COLOR) - np.array(D435_T_OPT_INFRA1))
    return t, matrix_quat(R_lo @ quat_matrix(*D435_Q_OPT_COLOR))


class Cameras:
    """TiledCameraSensor 로 렌더 -> 내장 rclpy 로 영상 · CameraInfo · 깊이 직접 발행 (CAM_HZ, 시뮬레이션 시간)"""

    def __init__(self, stage, names):
        from isaacsim.sensors.experimental.rtx import TiledCameraSensor

        self.d435 = [n for n in names if CAMS[n][3]]
        self.arm = "arm" in names
        self.sensors = []   # (센서, 카메라 이름들, 해상도, fx, 깊이)
        if self.d435:
            paths = [add_camera(stage, find_prim(stage, CAMS[n][0]), D435_RES, D435_FX) for n in self.d435]
            self.sensors.append((TiledCameraSensor(paths, resolution=D435_RES, annotators=["rgb", "distance_to_image_plane"], tick_rate=CAM_HZ),
                                 self.d435, D435_RES, D435_FX, True))
        if self.arm:
            fx = ARM_RES[1] / 2 / math.tan(math.radians(ARM_HFOV) / 2)
            path = add_camera(stage, find_prim(stage, CAMS["arm"][0]), ARM_RES, fx, near=0.02)
            self.sensors.append((TiledCameraSensor(path, resolution=ARM_RES, annotators=["rgb"], tick_rate=CAM_HZ), ["arm"], ARM_RES, fx, False))
        self.next_t = 0.0

    def start(self, node):
        self.pubs = {}
        for _, names, res, fx, depth in self.sensors:
            for n in names:
                top, frame = CAMS[n][1], CAMS[n][2]
                color = f"{top}/color" if depth else top
                self.pubs[n] = (node.create_publisher(Image, f"{color}/image_raw", 5),
                                node.create_publisher(CameraInfo, f"{color}/camera_info", 5),
                                node.create_publisher(Image, f"{top}/aligned_depth_to_color/image_raw", 5) if depth else None,
                                camera_info(res, fx, frame))
        if self.d435:
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub_tf = node.create_publisher(TFMessage, "/tf_static", qos)
            t, (w, x, y, z) = d435_link_to_color()
            tfs = []
            for n in self.d435:
                tf = TransformStamped()
                tf.header.frame_id = f"{CAMS[n][1][1:]}_link"
                tf.child_frame_id = CAMS[n][2]
                tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = (float(c) for c in t)
                tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z = w, x, y, z
                tfs.append(tf)
            self.pub_tf.publish(TFMessage(transforms=tfs))

    def publish(self, t, stamp):
        if t < self.next_t:
            return
        self.next_t = t + 1.0 / CAM_HZ - 1e-6
        for sensor, names, (h, w), _, depth in self.sensors:
            rgb, _ = sensor.get_data("rgb")
            if rgb is None:   # 렌더 준비 중
                continue
            rgb = rgb.numpy()
            dist = sensor.get_data("distance_to_image_plane")[0].numpy() if depth else None
            for i, n in enumerate(names):
                pub_img, pub_info, pub_depth, info = self.pubs[n]
                img = Image(height=h, width=w, encoding="rgb8", step=w * 3)
                img.header.stamp, img.header.frame_id = stamp, CAMS[n][2]
                img.data = np.ascontiguousarray(rgb[i]).tobytes()
                pub_img.publish(img)
                info.header.stamp = stamp
                pub_info.publish(info)
                if dist is not None:
                    d = np.nan_to_num(dist[i, :, :, 0], nan=0.0, posinf=0.0, neginf=0.0).astype(np.float32)   # 맞은 곳 없으면 0 (realsense 와 같게)
                    dm = Image(height=h, width=w, encoding="32FC1", step=w * 4)
                    dm.header = img.header
                    dm.data = d.tobytes()
                    pub_depth.publish(dm)


def pose_matrix(stage, path):
    """USD prim 의 월드 4x4 (열 벡터 꼴: p_world = M @ [p, 1])"""
    m = UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)
    return np.array(m, dtype=float).T


class Lidars:
    """PhysX 레이캐스트 2D 라이다. 스캔 prim 의 x-y 평면에서 angle_min -π 부터 반시계로 LIDAR_SAMPLES 개 (laser_* 프레임 기준).
    차체가 움직이므로 관절 루트 강체 (base_link, Articulation.get_world_poses) 자세 x 시작 때 잰 base_link -> 스캔 prim 고정 변환으로 원점 · 방향을 구한다.
    (재생 중 RigidPrim 을 새로 만들면 물리 텐서 뷰가 무효화되어 관절 읽기가 실패한다 — 2026-11-02)"""

    def __init__(self, stage, names):
        from omni.physics.core import get_physics_scene_query_interface
        from pxr import PhysicsSchemaTools

        self.names = names
        self.sq = get_physics_scene_query_interface()
        self.int_to_path = PhysicsSchemaTools.intToSdfPath
        bases = [str(p.GetPath()) for p in stage.Traverse()
                 if p.GetName() == BASE_LINK and p.HasAPI(UsdPhysics.RigidBodyAPI) and str(p.GetPath()).startswith("/World/waffle")]
        if not bases:
            raise RuntimeError(f"USD 에 강체 {BASE_LINK} 가 없습니다")
        self.base_path = base = bases[0]
        inv_base = np.linalg.inv(pose_matrix(stage, base))
        self.T_wb0 = pose_matrix(stage, base)   # 확인용 (재생 직후 물리 자세와 같아야 함)
        self.T_base_scan = {n: inv_base @ pose_matrix(stage, find_prim(stage, LIDARS[n][0])) for n in names}
        a = -math.pi + np.arange(LIDAR_SAMPLES) * (2 * math.pi / LIDAR_SAMPLES)
        self.dirs = np.stack([np.cos(a), np.sin(a), np.zeros_like(a)], axis=1)   # 스캔 프레임 방향
        self.self_hit = {}   # 충돌 형상 id -> 로봇 자신인지 (캐시)
        self.next_t = 0.0

    def start(self, node, robot):
        self.robot = robot
        pos, _ = robot.get_world_poses()
        print(f"[waffle_bridge] 라이다 기준 {self.base_path}: USD {np.round(self.T_wb0[:3, 3], 4)} · 물리 {np.round(pos.numpy()[0], 4)}", flush=True)
        self.pubs = {n: node.create_publisher(LaserScan, LIDARS[n][1], 5) for n in self.names}

    def is_self(self, collision):
        hit = self.self_hit.get(collision)
        if hit is None:
            hit = self.self_hit[collision] = str(self.int_to_path(collision)).startswith("/World/waffle")
        return hit

    def ray(self, origin, d):
        best = [math.inf]

        def report(h):
            if h.distance < best[0] and not self.is_self(h.collision):
                best[0] = h.distance
            return True

        self.sq.raycast_all(origin, d, LIDAR_RANGE[1], report, False)
        return best[0]

    def publish(self, t, stamp):
        if t < self.next_t:
            return
        self.next_t = t + 1.0 / LIDAR_HZ - 1e-6
        pos, quat = self.robot.get_world_poses()
        T_wb = np.eye(4)
        T_wb[:3, :3] = quat_matrix(*quat.numpy()[0])
        T_wb[:3, 3] = pos.numpy()[0]
        for n in self.names:
            T = T_wb @ self.T_base_scan[n]
            o = tuple(float(c) for c in T[:3, 3])
            dirs = self.dirs @ T[:3, :3].T
            ranges = []
            for d in dirs:
                r = self.ray(o, tuple(float(c) for c in d))
                ranges.append(r if LIDAR_RANGE[0] <= r <= LIDAR_RANGE[1] else math.inf)   # 못 잼 = inf (REP 117)
            m = LaserScan()
            m.header.stamp, m.header.frame_id = stamp, LIDARS[n][2]
            m.angle_min, m.angle_increment = -math.pi, 2 * math.pi / LIDAR_SAMPLES
            m.angle_max = m.angle_min + (LIDAR_SAMPLES - 1) * m.angle_increment
            m.scan_time = 1.0 / LIDAR_HZ
            m.time_increment = m.scan_time / LIDAR_SAMPLES
            m.range_min, m.range_max = LIDAR_RANGE
            m.ranges = [float(r) for r in ranges]
            self.pubs[n].publish(m)


# ---------------------------------------------------------------- 장면
def build_scene():
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    # 로봇 1대는 CPU 물리가 빠르다 (GPU 는 매 프레임 관절 목표를 쓰면 동기화로 프레임 34 ms, CPU 8 ms — 2026-10-28 측정)
    SimulationManager.setup_simulation(dt=1.0 / PHYS_HZ, device="cpu")
    GroundPlane("/World/ground")
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(800.0)
    mat = UsdShade.Material.Define(stage, "/World/PhysicsMaterial")
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(FRICTION)
    pm.CreateDynamicFrictionAttr(FRICTION)
    pm.CreateRestitutionAttr(0.0)
    robot = stage.DefinePrim("/World/waffle", "Xform")
    robot.GetReferences().AddReference(args.usd)
    robot.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
    for path in ("/World/ground", "/World/waffle"):   # 물리 재질 바인딩은 아래 충돌 형상에 상속됨
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path)).Bind(
            mat, UsdShade.Tokens.weakerThanDescendants, "physics")
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.Scene) or prim.IsA(UsdPhysics.Scene):
            PhysxSchema.PhysxSceneAPI.Apply(prim).CreateTimeStepsPerSecondAttr(PHYS_HZ)
    return stage


def set_view(eye, target):
    try:
        from omni.kit.viewport.utility import get_active_viewport
        from omni.kit.viewport.utility.camera_state import ViewportCameraState

        vp = get_active_viewport()
        cs = ViewportCameraState(vp.camera_path, vp)
        cs.set_position_world(Gf.Vec3d(*eye), False)
        cs.set_target_world(Gf.Vec3d(*target), True)
    except Exception as e:  # noqa: BLE001
        print(f"[waffle_bridge] 시점 설정 생략 ({e})")


# ---------------------------------------------------------------- 브리지
class Bridge:
    def __init__(self, robot, cams, lidars):
        self.robot = robot
        self.cams = cams
        self.lidars = lidars
        idx = {n: i for i, n in enumerate(robot.dof_names)}
        self.wheel_names = list(WHEELS)
        self.wheel_ids = [idx[WHEELS[n]] for n in self.wheel_names]
        self.arm_names = list(ARM)
        self.arm_ids = [idx[ARM[n][0]] for n in self.arm_names]
        self.arm_sign = np.array([ARM[n][1] for n in self.arm_names])
        self.finger_ids = [idx["gripper_left_joint"], idx["gripper_right_joint"]]
        self.wheel_cmd = np.zeros(4)
        q = robot.get_dof_positions().numpy()[0]
        self.arm_cmd = q[self.arm_ids] * self.arm_sign
        self.grip_cmd = grip_from_finger(q[self.finger_ids[0]])
        self.last_cmd = None
        self.prev_vel = None
        self.R_bi = rpy_matrix(*IMU_RPY)

        rclpy.init()
        self.node = rclpy.create_node("isaac_waffle_bridge")
        self.pub_js = self.node.create_publisher(JointState, "/isaac_joint_states", 10)
        self.pub_clock = self.node.create_publisher(Clock, "/clock", 10)
        self.pub_imu = self.node.create_publisher(Imu, "/imu/data", 10)
        estop_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_estop = self.node.create_publisher(Bool, "/e_stop", estop_qos)   # robot_interfaces qos::e_stop 과 같게
        self.node.create_subscription(JointState, "/isaac_joint_commands", self.on_cmd, 10)
        self.next_estop = 0.0
        if cams:
            cams.start(self.node)
        if lidars:
            lidars.start(self.node, robot)

    def on_cmd(self, m):
        """TopicBasedSystem 명령: name 은 관절 전체, position · velocity 는 그 인터페이스를 가진 관절만 순서대로 담길 수 있음"""
        names = list(m.name)

        def pick(values, has):   # 관절마다 값 (없으면 None)
            sel = [n for n in names if has(n)]
            if len(values) == len(names):
                return {n: values[i] for i, n in enumerate(names)}
            if len(values) == len(sel):
                return dict(zip(sel, values))
            return {}

        vel = pick(list(m.velocity), lambda n: n in WHEELS)
        pos = pick(list(m.position), lambda n: n in ARM or n == GRIPPER)
        for i, n in enumerate(self.wheel_names):
            if n in vel and math.isfinite(vel[n]):
                self.wheel_cmd[i] = vel[n]
        for i, n in enumerate(self.arm_names):
            if n in pos and math.isfinite(pos[n]):
                self.arm_cmd[i] = pos[n]
        if GRIPPER in pos and math.isfinite(pos[GRIPPER]):
            self.grip_cmd = pos[GRIPPER]
        self.last_cmd = time.monotonic()

    def apply(self):
        r = self.robot
        wheels = self.wheel_cmd
        if args.cmd_timeout > 0 and (self.last_cmd is None or time.monotonic() - self.last_cmd > args.cmd_timeout):
            wheels = np.zeros(4)
        r.set_dof_velocity_targets(wheels.reshape(1, -1), dof_indices=self.wheel_ids)
        f = finger_from_grip(self.grip_cmd)
        r.set_dof_position_targets(np.concatenate([self.arm_cmd * self.arm_sign, [f, -f]]).reshape(1, -1),
                                   dof_indices=self.arm_ids + self.finger_ids)

    def publish(self, t, dt):
        r = self.robot
        stamp = rclpy.time.Time(seconds=t).to_msg()
        self.pub_clock.publish(Clock(clock=stamp))
        q = r.get_dof_positions().numpy()[0]
        v = r.get_dof_velocities().numpy()[0]
        try:
            e = r.get_dof_efforts().numpy()[0]
        except Exception:  # noqa: BLE001
            e = np.zeros_like(q)
        js = JointState()
        js.header.stamp = stamp
        js.name = self.wheel_names + self.arm_names + [GRIPPER]
        js.position = [float(x) for x in np.concatenate([q[self.wheel_ids], q[self.arm_ids] * self.arm_sign,
                                                          [grip_from_finger(q[self.finger_ids[0]])]])]
        js.velocity = [float(x) for x in np.concatenate([v[self.wheel_ids], v[self.arm_ids] * self.arm_sign,
                                                          [v[self.finger_ids[0]] * (GRIP_CLOSED_RAD - GRIP_OPEN_RAD)
                                                           / (FINGER_CLOSED - FINGER_OPEN)]])]
        js.effort = [float(x) for x in np.concatenate([e[self.wheel_ids], e[self.arm_ids] * self.arm_sign, [0.0]])]
        self.pub_js.publish(js)
        # IMU: base_link 자세 · 속도 (월드) -> imu_link (보드 축)
        _, quat = r.get_world_poses()
        lin, ang = r.get_velocities()
        R_wb = quat_matrix(*quat.numpy()[0])
        lin, ang = lin.numpy()[0], ang.numpy()[0]
        acc = np.zeros(3) if self.prev_vel is None or dt <= 0 else (lin - self.prev_vel) / dt
        self.prev_vel = lin
        R_wi = R_wb @ self.R_bi
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        w, x, y, z = matrix_quat(R_wi)
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = w, x, y, z
        gi = R_wi.T @ ang
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = (float(c) for c in gi)
        ai = R_wi.T @ (acc + np.array([0.0, 0.0, 9.81]))   # 가속도계 = 가속도 - 중력
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = (float(c) for c in ai)
        imu.orientation_covariance = [1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3]
        imu.angular_velocity_covariance = [1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4]
        imu.linear_acceleration_covariance = [1e-2, 0, 0, 0, 1e-2, 0, 0, 0, 1e-2]
        self.pub_imu.publish(imu)
        if t >= self.next_estop:
            self.pub_estop.publish(Bool(data=False))
            self.next_estop = t + 0.1
        if self.cams:
            self.cams.publish(t, stamp)
        if self.lidars:
            self.lidars.publish(t, stamp)

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    SimulationManager.switch_physics_engine("physx")
    stage = build_scene()
    for _ in range(5):
        app.update()
    robot = Articulation("/World/waffle")
    def names_of(arg, table, opt):
        names = [n.strip() for n in arg.split(",") if n.strip()]
        unknown = set(names) - set(table)
        if unknown:
            raise SystemExit(f"{opt} 에 모르는 이름 {sorted(unknown)} (가능: {', '.join(table)})")
        found = []
        for n in names:   # 고정 관절을 합친 USD 에는 센서 프레임 prim 이 없다 → 그 센서만 건너뜀
            try:
                find_prim(stage, table[n][0])
                found.append(n)
            except RuntimeError as e:
                print(f"[waffle_bridge] {opt} {n} 건너뜀: {e}", flush=True)
        return found

    cam_names = names_of(args.cams, CAMS, "--cams")
    lidar_names = names_of(args.lidars, LIDARS, "--lidars")
    cams = Cameras(stage, cam_names) if cam_names else None   # 센서는 재생 (play) 전에 만든다
    lidars = Lidars(stage, lidar_names) if lidar_names else None   # 고정 변환은 재생 전 USD 자세로 잰다
    app_utils.play(commit=True)
    for _ in range(3):
        app.update()
    n = len(robot.dof_names)
    wheel_mask = np.array(["Wheel" in d for d in robot.dof_names])
    finger_mask = np.array(["gripper" in d for d in robot.dof_names])
    stiff = np.where(wheel_mask, 0.0, np.where(finger_mask, FINGER_STIFFNESS, ARM_STIFFNESS)).reshape(1, n)
    damp = np.where(wheel_mask, WHEEL_DAMPING, np.where(finger_mask, FINGER_DAMPING, ARM_DAMPING)).reshape(1, n)
    robot.set_dof_gains(stiffnesses=stiff, dampings=damp)
    for _ in range(WARMUP_FRAMES):   # 첫 렌더 준비로 몇 초 멈추는 구간 — /clock 을 내기 전에 넘긴다 (호스트 컨트롤러 전환 5 s 제한)
        app.update()
    bridge = Bridge(robot, cams, lidars)
    if not args.headless:
        set_view((-0.9, -0.9, 0.7), (0.1, 0.0, 0.15))
    print(f"[waffle_bridge] 시작: 관절 {robot.dof_names}, 물리 {PHYS_HZ} Hz. 호스트에서 base_bringup sim:=true")
    t_prev = SimulationManager.get_simulation_time()
    wall0, sim0 = time.perf_counter(), t_prev
    while app.is_running():
        rclpy.spin_once(bridge.node, timeout_sec=0.0)
        bridge.apply()
        app.update()
        t = SimulationManager.get_simulation_time()
        bridge.publish(t, t - t_prev)
        t_prev = t
        lag = (t - sim0) - (time.perf_counter() - wall0)   # 실시간보다 빠르면 기다림
        if lag > 0:
            time.sleep(lag)
        if args.test_seconds and t - sim0 >= args.test_seconds:
            break
    bridge.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        app.close()
