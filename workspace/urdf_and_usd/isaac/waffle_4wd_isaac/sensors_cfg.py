"""waffle_4wd 센서 설정 (Isaac Lab 3.0). 값은 Gazebo 공유본 ``config/sensors.yaml`` 과 같고, 위치는 ``urdf/sensors.json``.

USD 변환은 고정 관절 (센서 frame) 을 부모 몸체에 합치므로, 센서마다 붙는 몸체 prim 과 그 몸체 기준 위치 · 자세를 sensors.json 에서 읽는다.
카메라 위치는 URDF optical frame 그대로 (z 앞, y 아래 = OffsetCfg convention "ros"). D435 깊이 (distance_to_image_plane) 는 frame 기준이라 더할 것 없음
(Gazebo 는 센서를 8 mm 앞에 두고 중계 노드가 고쳐 냄. Isaac 은 근거리 자름 0.105 m 로 D435 몸체 메시가 안 보여 frame 그대로 됨, 2026-10-29 시험).

| 함수 | 센서 | 기본 |
|---|---|---|
| ``lds03_cfg('lds03_front'|'lds03_rear', robot_prim, targets)`` | LDS-03 2D 라이다 (400 광선, 0.05 ~ 12 m, 10 Hz) | 켬 |
| ``tfmini_cfg(robot_prim, targets)`` | TFmini Plus (광선 1개, 0.1 ~ 12 m, 100 Hz) | 켬 |
| ``imu_cfg(robot_prim, 'imu')`` | OpenCR IMU (100 Hz). B안: 아래층 OpenCR 칩 하나 = imu_link (위층은 끔) | 켬 |
| ``d435_cfg(robot_prim, 'front'|'rear', stream)`` | D435 컬러 (rgb + 깊이) 또는 IR 좌 · 우 (640 x 480, 30 Hz) | 끔 (GPU) |
| ``arm_camera_cfg(robot_prim)`` | 팔 카메라 (1280 x 720, 30 Hz, 왜곡 없는 바늘구멍) | 끔 (GPU) |

LiDAR 는 ``targets`` 에 준 메쉬만 맞힌다 (MultiMeshRayCaster). 차체에 가리는 것까지 흉내 내려면 ``spawn_lidar_occluders`` 로
스캔면을 지나는 차체 상자 (옆벽 · 기둥 · 벨트 블록 · 맞은편 LDS-03 머리, 보이지 않음 · 충돌 없음) 를 만들고 ``lidar_targets`` 에 넣는다.
LDS-03 은 상판 밑에 뒤집어 달려 있어 광선 시작점을 머리 바깥 (start_radius 25 mm) 으로 민다. 거리 = 히트까지 + start_radius.
"""

from __future__ import annotations

import math
from collections.abc import Callable

import torch

import isaaclab.sim as sim_utils
from isaaclab.sensors import ImuCfg
from isaaclab.sensors.camera import TiledCameraCfg
from isaaclab.sensors.ray_caster import MultiMeshRayCasterCfg, patterns
from isaaclab.utils import configclass
from isaaclab.utils.math import quat_apply

from .robot_cfg import SENSORS

SPEC = SENSORS["spec"]
POSE = SENSORS["sensors"]
LDS03_START_RADIUS = 0.025   # [m] LDS-03 머리 반지름 (Ø35.3) 바깥에서 광선 시작
OCCLUDER_GROUP = "lidar_occluders"


def body_prim(robot_prim: str, sensor: str) -> str:
    """센서가 붙는 몸체 prim (예: '/World/Robot/Geometry/base_link')"""
    return f"{robot_prim}/{POSE[sensor]['usd_body']}"


# ---------------- LiDAR (LDS-03 x2) ----------------
def ring_lidar_pattern(cfg: RingLidarPatternCfg, device: str) -> tuple[torch.Tensor, torch.Tensor]:
    """2D 라이다 광선, 시작점을 광선 방향으로 start_radius 만큼 (라이다 머리 바깥).
    Isaac Lab 3.0 레이캐스터는 오프셋 회전을 광선 방향에만 적용하므로 시작점은 같은 회전 (start_rot) 으로 미리 돌린다."""
    starts, dirs = patterns.lidar_pattern(cfg, device)
    q = torch.tensor([cfg.start_rot], device=dirs.device, dtype=dirs.dtype).repeat(len(dirs), 1)
    return starts + quat_apply(q, dirs) * cfg.start_radius, dirs


@configclass
class RingLidarPatternCfg(patterns.LidarPatternCfg):
    func: Callable = ring_lidar_pattern
    start_radius: float = LDS03_START_RADIUS
    start_rot: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)


def lds03_cfg(name: str, robot_prim: str, targets: list) -> MultiMeshRayCasterCfg:
    """LDS-03 한 대. robot_prim: 로봇 루트 (예 '/World/Robot', '{ENV_REGEX_NS}/Robot'), targets: 맞힐 메쉬 (정규식 또는 RaycastTargetCfg)"""
    s, sp = POSE[name], SPEC["lds03"]
    res = 360.0 / float(sp["samples"])
    return MultiMeshRayCasterCfg(
        prim_path=f"{body_prim(robot_prim, name)}/{name}",
        update_period=1.0 / float(sp["update_rate_hz"]),
        offset=MultiMeshRayCasterCfg.OffsetCfg(pos=tuple(s["pos"]), rot=tuple(s["quat_xyzw"])),
        ray_alignment="base",
        pattern_cfg=RingLidarPatternCfg(channels=1, vertical_fov_range=(0.0, 0.0), horizontal_fov_range=(-180.0, 180.0 - res),
                                        horizontal_res=res, start_rot=tuple(s["quat_xyzw"])),
        max_distance=float(sp["range_m"][1]),
        mesh_prim_paths=targets,
        debug_vis=False,
    )


# ---------------- TFmini Plus ----------------
def single_ray_pattern(cfg: patterns.PatternBaseCfg, device: str) -> tuple[torch.Tensor, torch.Tensor]:
    """센서 +x 로 광선 1개 (Gazebo 공유본과 같음. 실물 빔은 3.6°)"""
    return torch.zeros((1, 3), device=device), torch.tensor([[1.0, 0.0, 0.0]], device=device)


@configclass
class SingleRayPatternCfg(patterns.PatternBaseCfg):
    func: Callable = single_ray_pattern


def tfmini_cfg(robot_prim: str, targets: list) -> MultiMeshRayCasterCfg:
    s, sp = POSE["tfmini"], SPEC["tfmini_plus"]
    return MultiMeshRayCasterCfg(
        prim_path=f"{body_prim(robot_prim, 'tfmini')}/tfmini",
        update_period=1.0 / float(sp["update_rate_hz"]),
        offset=MultiMeshRayCasterCfg.OffsetCfg(pos=tuple(s["pos"]), rot=tuple(s["quat_xyzw"])),
        ray_alignment="base",
        pattern_cfg=SingleRayPatternCfg(),
        max_distance=float(sp["range_m"][1]),
        mesh_prim_paths=targets,
        debug_vis=False,
    )


# ---------------- IMU ----------------
def imu_cfg(robot_prim: str, name: str = "imu") -> ImuCfg:
    s = POSE[name]
    return ImuCfg(prim_path=body_prim(robot_prim, name), update_period=1.0 / float(SPEC["imu"]["update_rate_hz"]),
                  offset=ImuCfg.OffsetCfg(pos=tuple(s["pos"]), rot=tuple(s["quat_xyzw"])))


# ---------------- 카메라 ----------------
def _pinhole(hfov: float, clip: tuple[float, float]) -> sim_utils.PinholeCameraCfg:
    f = 1.0   # 초점 거리 [cm] (aperture 와 비율만 의미 있음)
    return sim_utils.PinholeCameraCfg(focal_length=f, horizontal_aperture=2.0 * f * math.tan(hfov / 2.0), clipping_range=clip)


def _camera(robot_prim: str, sensor: str, width: int, height: int, hz: float, hfov: float, clip, data_types) -> TiledCameraCfg:
    s = POSE[sensor]
    return TiledCameraCfg(
        prim_path=f"{body_prim(robot_prim, sensor)}/{sensor}",
        update_period=1.0 / hz,
        width=width, height=height, data_types=list(data_types),
        offset=TiledCameraCfg.OffsetCfg(pos=tuple(s["pos"]), rot=tuple(s["quat_xyzw"]), convention="ros"),
        spawn=_pinhole(hfov, tuple(clip)),
    )


def d435_cfg(robot_prim: str, side: str = "front", stream: str = "color") -> TiledCameraCfg:
    """D435 한 스트림. stream 'color' = rgb + distance_to_image_plane (깊이, 컬러 광학 중심 기준), 'infra1' | 'infra2' = IR 좌 · 우 (rgb 로 흉내, 패턴 투사 없음)"""
    sp = SPEC["d435"]
    hfov = float(sp["color_hfov_rad"] if stream == "color" else sp["ir_hfov_rad"])
    types = ("rgb", "distance_to_image_plane") if stream == "color" else ("rgb",)
    return _camera(robot_prim, f"d435_{side}_{stream}", int(sp["width"]), int(sp["height"]), float(sp["update_rate_hz"]), hfov,
                   sp["depth_clip_m"], types)


def arm_camera_cfg(robot_prim: str) -> TiledCameraCfg:
    sp = SPEC["arm_camera"]
    return _camera(robot_prim, "arm_camera", int(sp["width"]), int(sp["height"]), float(sp["update_rate_hz"]), float(sp["hfov_rad"]),
                   sp["clip_m"], ("rgb",))


# ---------------- LiDAR 자기 가림 ----------------
def spawn_lidar_occluders(robot_prim: str) -> list[str]:
    """스캔면을 지나는 차체 상자를 base_link 아래에 보이지 않는 메쉬로 만듦 (충돌 없음). 만든 prim 경로 (robot_prim 정규식 그대로) 를 돌려줌.
    로봇을 만든 뒤, LiDAR 를 만들기 전에 부를 것."""
    names = sorted(set(POSE["lds03_front"]["occluder_boxes"]) | set(POSE["lds03_rear"]["occluder_boxes"]))
    boxes = {b["name"]: b for b in SENSORS["base_boxes"]}
    paths = []
    for n in names:
        b = boxes[n]
        if b["shape"] == "box":
            cfg = sim_utils.MeshCuboidCfg(size=tuple(b["size"]), visible=False)
        else:
            cfg = sim_utils.MeshCylinderCfg(radius=b["size"][0] / 2, height=b["size"][2], axis="Z", visible=False)
        p = f"{robot_prim}/Geometry/base_link/{OCCLUDER_GROUP}/{n}"
        cfg.func(p, cfg, translation=tuple(b["pos"]), orientation=tuple(b["quat_xyzw"]))
        paths.append(p)
    return paths


def lidar_targets(world_mesh_exprs: list[str], robot_prim: str | None = None) -> list:
    """LiDAR 대상 목록: 움직이지 않는 장면 메쉬 (정규식) + (robot_prim 을 주면) 로봇과 함께 움직이는 차체 가림 상자"""
    T = MultiMeshRayCasterCfg.RaycastTargetCfg
    out = [T(prim_expr=e, track_mesh_transforms=False) for e in world_mesh_exprs]
    if robot_prim is not None:
        out.append(T(prim_expr=f"{robot_prim}/Geometry/base_link/{OCCLUDER_GROUP}/.*", track_mesh_transforms=True))
    return out
