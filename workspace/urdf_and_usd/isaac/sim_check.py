"""waffle_4wd Isaac Lab 3.0 점검 (창 없이). 로봇 1대 + 둘레 상자 4개.

    ./run.sh check                      가벼운 판, 시작 자세 = robot_params.json 기본 (drive)
    ./run.sh check --full               정밀 판 (차체 볼록 메시)
    ./run.sh check --pose stow          다른 시작 자세 (drive | stow | zero)
    ./run.sh check --cameras            + D435 앞 · 뒤 · 팔 카메라 그림 (GPU 더 씀)
    ./run.sh window [--full]            Isaac Sim 창으로 같은 장면 (닫으면 끝)

점검 항목 -> results/check_<판>_<자세>.json, results/check.log, results/img/
1. USD 안 값 = robot_params.json (관절 구동 · 한계 · armature, 자기 충돌, 팔끼리 충돌 뺌, 마찰 재질)
2. 가만히 2 s: base_link 높이 · 기울기 · 팔 처짐
3. 자세 바꾸기 (zero · stow · drive 를 돌고 시작 자세로, stow <-> drive 는 zero 를 거쳐): 자기 충돌 켠 상태에서 목표에 닿는지
4. 직진 0.1 m/s x 3 s, 제자리 회전 3 s
5. LDS-03 x2: 상자까지 거리 (계산값과 비교), 차체에 가린 각도 / TFmini: 앞 상자 거리 / IMU: 가만히 가속도 · 각속도
6. (--cameras) D435 앞 컬러 · 깊이, 뒤 컬러, 팔 카메라 그림
"""

import argparse
import json
import math
import os
import time

from isaaclab.app import AppLauncher

HERE = os.path.dirname(os.path.abspath(__file__))
parser = argparse.ArgumentParser(description="waffle_4wd 점검")
parser.add_argument("--full", action="store_true", help="정밀 판 (차체 볼록 메시)")
parser.add_argument("--loaded", action="store_true", help="퍽 12개를 실은 가벼운 판")
parser.add_argument("--pose", default=None, choices=["drive", "stow", "zero"], help="시작 자세 (기본: robot_params.json)")
parser.add_argument("--cameras", action="store_true", help="카메라 그림도")
parser.add_argument("--window", action="store_true", help="창으로 보기 (run.sh window)")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
if not args_cli.window:
    args_cli.headless = True
if args_cli.cameras or args_cli.window:
    args_cli.enable_cameras = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sensors import Imu  # noqa: E402
from isaaclab.sensors.ray_caster import MultiMeshRayCaster  # noqa: E402
from isaaclab_physx.physics import PhysxCfg  # noqa: E402

from waffle_4wd_isaac import robot_cfg as R  # noqa: E402
from waffle_4wd_isaac import sensors_cfg as S  # noqa: E402

KIND = "full" if args_cli.full else ("loaded" if args_cli.loaded else "light")
OUT = os.path.join(HERE, "results")
os.makedirs(os.path.join(OUT, "img"), exist_ok=True)
LOG = open(os.path.join(OUT, "check.log"), "a", encoding="utf-8")
ROBOT = "/World/Robot"
# 둘레 상자 (월드 x 앞, y 왼쪽, 처음 자세 기준): 이름, 가운데 (x, y), 크기. 높이 0.3 m (스캔면 0.082 · TFmini 0.042 위)
OBST = [("front", (1.0, 0.0), (0.2, 1.0, 0.3)), ("rear", (-0.9, 0.0), (0.2, 1.0, 0.3)),
        ("left", (0.0, 0.8), (1.0, 0.2, 0.3)), ("right", (0.0, -1.0), (1.0, 0.2, 0.3))]


def L(*a):
    s = " ".join(str(x) for x in a)
    print("[waffle_4wd]", s, flush=True)
    LOG.write(s + "\n")
    LOG.flush()


def check_usd_values(stage) -> dict:
    """USD 안 값이 robot_params.json 과 같은지 (단위: 각 관절 강성 · 감쇠는 per deg, 속도 deg/s)"""
    from pxr import UsdPhysics, UsdShade

    P = R.PARAMS
    bad = []

    def near(a, b, tol=1e-3):
        return a is not None and abs(a - b) <= tol * max(1.0, abs(b))

    phys = stage.GetPrimAtPath(f"{ROBOT}/Physics")
    for j in phys.GetChildren():
        n = j.GetName()
        if n in R.WHEEL_JOINTS:
            g, kind, k, d, vel, arm = P["wheels"], "angular", 0.0, P["wheels"]["damping"], P["wheels"]["velocity_limit"], P["wheels"]["armature"]
            eff = g["effort_limit"]
        elif n in R.ARM_JOINTS:
            g, kind, k, d, vel, arm, eff = P["arm"], "angular", P["arm"]["stiffness"], P["arm"]["damping"], P["arm"]["velocity_limit"], P["arm"]["armature"], P["arm"]["effort_limit"]
        elif n in R.FINGER_JOINTS:
            g, kind, k, d, vel, arm, eff = P["fingers"], "linear", P["fingers"]["stiffness"], P["fingers"]["damping"], P["fingers"]["velocity_limit"], 0.0, P["fingers"]["force_limit"]
        else:
            continue
        sc = math.pi / 180.0 if kind == "angular" else 1.0
        got = {a: j.GetAttribute(f"drive:{kind}:physics:{a}").Get() for a in ("stiffness", "damping", "maxForce")}
        got["maxJointVelocity"] = j.GetAttribute("physxJoint:maxJointVelocity").Get()
        got["armature"] = j.GetAttribute("physxJoint:armature").Get()
        want = dict(stiffness=k * sc, damping=d * sc, maxForce=eff, maxJointVelocity=vel / sc, armature=arm)
        for a, w in want.items():
            if not near(got[a], w):
                bad.append(f"{n}.{a}: USD {got[a]} != {w:.6g}")
    root = stage.GetPrimAtPath(f"{ROBOT}/Geometry/base_link")
    sc_on = root.GetAttribute("physxArticulation:enabledSelfCollisions").Get()
    if bool(sc_on) != bool(P["contact"]["self_collision"]):
        bad.append(f"자기 충돌 USD {sc_on}")
    pairs = 0
    for b in R.ARM_BODIES:
        p = stage.GetPrimAtPath(f"{ROBOT}/{R.SENSORS['moving_bodies'][b]}")
        if p.HasAPI(UsdPhysics.FilteredPairsAPI):
            pairs += len(UsdPhysics.FilteredPairsAPI(p).GetFilteredPairsRel().GetTargets())
    geom = stage.GetPrimAtPath(f"{ROBOT}/Geometry")
    mat, _ = UsdShade.MaterialBindingAPI(geom).ComputeBoundMaterial("physics")
    mu = None
    if mat:
        m = UsdPhysics.MaterialAPI(mat.GetPrim())
        mu = (m.GetStaticFrictionAttr().Get(), m.GetDynamicFrictionAttr().Get())
    if mu is None or not (near(mu[0], P["contact"]["static_friction"]) and near(mu[1], P["contact"]["dynamic_friction"])):
        bad.append(f"마찰 재질 USD {mu}")
    if pairs != len(R.ARM_BODIES) * (len(R.ARM_BODIES) - 1) // 2:
        bad.append(f"팔끼리 충돌 뺀 쌍 {pairs}")
    return dict(ok=not bad, mismatches=bad, self_collision=bool(sc_on), filtered_pairs=pairs, friction=mu)


def main():
    res = dict(kind=KIND, params=R.describe())
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 120, device=args_cli.device, physics=PhysxCfg(),
                                                              physics_material=R.CONTACT_MATERIAL))
    g = sim_utils.GroundPlaneCfg(physics_material=R.CONTACT_MATERIAL)
    g.func("/World/ground", g)
    light = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.9, 0.9, 0.9))
    light.func("/World/Light", light)
    for nm, (x, y), size in OBST:
        c = sim_utils.MeshCuboidCfg(size=size, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.85, 0.45, 0.2)),
                                    collision_props=sim_utils.CollisionPropertiesCfg())
        c.func(f"/World/Obstacles/{nm}", c, translation=(x, y, size[2] / 2))
    pose_name = args_cli.pose or R.PARAMS["poses"]["default"]
    cfg = {"full": R.WAFFLE_4WD_FULL_CFG, "loaded": R.WAFFLE_4WD_LOADED_CFG, "light": R.WAFFLE_4WD_CFG}[KIND].replace(prim_path=ROBOT)
    cfg.init_state.joint_pos = R.pose(pose_name)
    robot = Articulation(cfg)
    occ = S.spawn_lidar_occluders(ROBOT)
    targets = S.lidar_targets(["/World/Obstacles/[^/]+", "/World/ground"], ROBOT)
    sensors = {}
    for n in ("lds03_front", "lds03_rear"):
        c = S.lds03_cfg(n, ROBOT, targets)
        c.spawn.func(c.prim_path, c.spawn)        # 센서 Xform (장면 관리자 없이 쓰므로 직접)
        sensors[n] = MultiMeshRayCaster(c)
    c = S.tfmini_cfg(ROBOT, targets)
    c.spawn.func(c.prim_path, c.spawn)
    sensors["tfmini"] = MultiMeshRayCaster(c)
    imu = Imu(S.imu_cfg(ROBOT, "imu"))
    cams = {}
    if args_cli.cameras or args_cli.window:
        from isaaclab.sensors.camera import TiledCamera

        cams = {"d435_front": TiledCamera(S.d435_cfg(ROBOT, "front", "color")), "d435_rear": TiledCamera(S.d435_cfg(ROBOT, "rear", "color")),
                "arm_camera": TiledCamera(S.arm_camera_cfg(ROBOT))}
    sim.reset()
    dt = sim.get_physics_dt()
    L(f"===== {time.strftime('%Y-%m-%d %H:%M')} 판 {KIND}, 시작 자세 {pose_name}")
    L(R.describe())
    L(f"관절 {robot.num_joints}: {robot.joint_names}")
    L(f"LiDAR 자기 가림 상자 {len(occ)} 개")
    from isaaclab.sim.utils import get_current_stage

    res["usd_values"] = check_usd_values(get_current_stage())
    L(f"1. USD 값 = robot_params.json: {'같음' if res['usd_values']['ok'] else '다름'} {res['usd_values']['mismatches'][:5]}, "
      f"자기 충돌 {res['usd_values']['self_collision']}, 팔끼리 뺀 쌍 {res['usd_values']['filtered_pairs']}, 마찰 {res['usd_values']['friction']}")
    res["mass"] = float(robot.data.body_mass.torch[0].sum())

    wl_ids, _ = robot.find_joints(R.WHEEL_JOINTS_LEFT, preserve_order=True)
    wr_ids, _ = robot.find_joints(R.WHEEL_JOINTS_RIGHT, preserve_order=True)
    hold_names = R.ARM_JOINTS + R.FINGER_JOINTS
    hold, _ = robot.find_joints(hold_names, preserve_order=True)
    wheels = list(wl_ids) + list(wr_ids)
    cmd = robot.actuators.target_command

    def pose_vec(name):
        p = R.pose(name)
        return torch.tensor([[p[n] for n in hold_names]], dtype=torch.float32, device=sim.device)

    target = pose_vec(pose_name)

    def step(n, wl=0.0, wr=0.0, tgt=None):
        tgt = target if tgt is None else tgt
        for _ in range(n):
            cmd.set_velocity_index(value=torch.tensor([[wl, wl, wr, wr]], dtype=torch.float32, device=sim.device), joint_ids=wheels)
            cmd.set_position_index(value=tgt, joint_ids=hold)
            robot.write_data_to_sim()
            sim.step()
            robot.update(dt)
            imu.update(dt)
            for c_ in cams.values():
                c_.update(dt)

    def st():
        d = robot.data
        return dict(pos=d.root_pos_w.torch[0].tolist(), yaw=float(d.heading_w.torch[0]), grav=d.projected_gravity_b.torch[0].tolist(),
                    q=d.joint_pos.torch[0, hold].tolist())

    def tilt(s):
        return math.degrees(math.acos(max(-1.0, min(1.0, -s["grav"][2]))))

    if args_cli.window:
        L("창: 닫으면 끝. 로봇은 시작 자세로 가만히 있음")
        while simulation_app.is_running():
            step(1)
        return

    # 2. 가만히
    t0 = time.time()
    step(int(2.0 / dt))
    rtf = 2.0 / (time.time() - t0)
    s0 = st()
    err0 = [round(a - b, 3) for a, b in zip(s0["q"], target[0].tolist())]
    res["rest"] = dict(base_z=s0["pos"][2], base_z_expected=R.ROOT_HEIGHT, tilt_deg=tilt(s0), joint_err=dict(zip(hold_names, err0)), rtf=rtf)
    L(f"2. 가만히 2 s: base_link 높이 {s0['pos'][2]:.4f} m (URDF {R.ROOT_HEIGHT}), 기울기 {tilt(s0):.2f}°, 관절 오차 {dict(zip(hold_names, err0))}, "
      f"실시간 {rtf:.2f}배")

    # 5. 센서 (가만히 있는 지금)
    for _ in range(3):
        step(1)
    root = torch.tensor(s0["pos"][:2])
    c_, s_ = math.cos(-s0["yaw"]), math.sin(-s0["yaw"])
    res["lidar"] = {}
    for n in ("lds03_front", "lds03_rear"):
        lid = sensors[n]
        lid.update(dt, force_recompute=True)
        hits = lid.data.ray_hits_w.torch[0].cpu()
        org = torch.tensor(S.POSE[n]["pos"])     # 몸체 기준 (로봇이 원점 · 정면 +x 에서 거의 그대로)
        org_w = torch.tensor([s0["pos"][0] + org[0], s0["pos"][1] + org[1], s0["pos"][2] + org[2]])
        d = torch.linalg.norm(hits - org_w, dim=1)
        ok = torch.isfinite(d)
        rel = (hits - torch.tensor(s0["pos"]))[:, :2]
        bx, by = c_ * rel[:, 0] - s_ * rel[:, 1], s_ * rel[:, 0] + c_ * rel[:, 1]
        own = ok & (bx.abs() < 0.17) & (by.abs() < 0.18) & (hits[:, 2] > 0.01)     # 차체에 가림 (바닥 히트 제외)
        per = {}
        for nm, (x, y), size in OBST:
            m = ok & ~own & ((hits[:, 0] - x).abs() <= size[0] / 2 + 0.01) & ((hits[:, 1] - y).abs() <= size[1] / 2 + 0.01)
            face = {"front": x - size[0] / 2 - float(org_w[0]), "rear": float(org_w[0]) - (x + size[0] / 2),
                    "left": y - size[1] / 2 - float(org_w[1]), "right": float(org_w[1]) - (y + size[1] / 2)}[nm]
            per[nm] = dict(rays=int(m.sum()), min_dist=round(float(d[m].min()), 4) if m.any() else None,
                           expected_perpendicular=round(face, 4))
        res["lidar"][n] = dict(sensor_height=round(float(org_w[2]), 4), blocked_by_robot_deg=round(float(own.sum()) * 0.9, 1), obstacles=per,
                               floor_hits=int((ok & (hits[:, 2].abs() < 0.005)).sum()))
        L(f"5. {n}: 스캔 높이 {org_w[2]:.4f} m, 차체에 가림 {res['lidar'][n]['blocked_by_robot_deg']}°, 바닥 히트 {res['lidar'][n]['floor_hits']}, "
          + ", ".join(f"{k} {v['min_dist']} m (수직 {v['expected_perpendicular']})" for k, v in per.items()))
    tf = sensors["tfmini"]
    tf.update(dt, force_recompute=True)
    h = tf.data.ray_hits_w.torch[0, 0].cpu()
    lens = torch.tensor(S.POSE["tfmini"]["pos"]) + torch.tensor(s0["pos"])
    dist = float(torch.linalg.norm(h - lens)) if torch.isfinite(h).all() else None
    exp = OBST[0][1][0] - OBST[0][2][0] / 2 - float(lens[0])
    res["tfmini"] = dict(dist=dist, expected=round(exp, 4))
    L(f"5. TFmini: {dist if dist is None else round(dist, 4)} m (앞 상자 계산 {exp:.4f})")
    acc = imu.data.lin_acc_b.torch[0].tolist()
    gyr = imu.data.ang_vel_b.torch[0].tolist()
    res["imu"] = dict(lin_acc_b=acc, ang_vel_b=gyr)
    L(f"5. IMU (imu_link): 가속도 {[round(v, 3) for v in acc]} m/s², 각속도 {[round(v, 4) for v in gyr]} rad/s")

    # 6. 카메라
    if cams:
        from PIL import Image

        step(6)
        res["cameras"] = {}
        for n, cam in cams.items():
            rgb = cam.data.output["rgb"].torch[0].cpu().numpy() if hasattr(cam.data.output["rgb"], "torch") else cam.data.output["rgb"][0].cpu().numpy()
            Image.fromarray(rgb[..., :3]).save(os.path.join(OUT, "img", f"{KIND}_{n}_rgb.png"))
            info = dict(rgb_mean=float(rgb[..., :3].mean()))
            if "distance_to_image_plane" in cam.data.output:
                dep = cam.data.output["distance_to_image_plane"]
                dep = (dep.torch if hasattr(dep, "torch") else dep)[0, ..., 0].cpu()
                fin = torch.isfinite(dep) & (dep > 0)
                info.update(depth_valid=float(fin.float().mean()), depth_center=float(dep[dep.shape[0] // 2, dep.shape[1] // 2]))
                im = (255 * (1 - (dep.clamp(0, 3) / 3))).nan_to_num(0).byte().numpy()
                Image.fromarray(im).save(os.path.join(OUT, "img", f"{KIND}_{n}_depth.png"))
            res["cameras"][n] = info
            L(f"6. {n}: {info} -> results/img/{KIND}_{n}_*.png")

    # 3. 자세 바꾸기 (자기 충돌 켠 상태)
    res["poses"] = {}
    # 대기 (stow) <-> 주행 (drive) 은 영 자세를 거쳐서: 가벼운 판에서는 J1 이 돌 때 접힌 팔이 D435 기둥 쪽 상자에 걸림
    order, prev = [], pose_name
    for p in [q for q in ("zero", "stow", "drive") if q != pose_name] + [pose_name]:
        if {prev, p} == {"stow", "drive"}:
            order.append("zero")
        order.append(p)
        prev = p
    cur = target.clone()
    for p in order:
        goal = pose_vec(p)
        n = int(3.0 / dt)
        for k in range(n):
            step(1, tgt=cur + (goal - cur) * min(1.0, k / (0.75 * n)))
        step(int(1.0 / dt), tgt=goal)
        s = st()
        err = [round(a - b, 3) for a, b in zip(s["q"], goal[0].tolist())]
        worst = max(err[:len(R.ARM_JOINTS)], key=abs)
        res["poses"][f"{len(res['poses']) + 1}_{p}"] = dict(joint_err=dict(zip(hold_names, err)), worst_arm_err=worst, tilt_deg=tilt(s), base_xy=s["pos"][:2])
        L(f"3. 자세 {p}: 팔 관절 오차 {dict(zip(R.ARM_JOINTS, err[:len(R.ARM_JOINTS)]))} (가장 큰 {worst}), 기울기 {tilt(s):.2f}°")
        cur = goal
    target = cur

    # 4. 직진 · 회전
    s1 = st()
    w = 0.1 / R.WHEEL_RADIUS
    t0 = time.time()
    step(int(3.0 / dt), w, w)
    rtf = 3.0 / (time.time() - t0)
    s2 = st()
    dx, dy = s2["pos"][0] - s1["pos"][0], s2["pos"][1] - s1["pos"][1]
    fwd = dx * math.cos(s1["yaw"]) + dy * math.sin(s1["yaw"])
    res["forward"] = dict(dist=fwd, expected=0.3, yaw_drift_deg=math.degrees(s2["yaw"] - s1["yaw"]), rtf=rtf)
    L(f"4. 직진 0.1 m/s x 3 s: {fwd:.3f} m (미끄럼 없을 때 0.300), 방향 틀어짐 {res['forward']['yaw_drift_deg']:.1f}°, 실시간 {rtf:.2f}배")
    step(int(1.0 / dt))
    s3 = st()
    step(int(3.0 / dt), -w, w)
    s4 = st()
    dyaw = math.degrees(math.atan2(math.sin(s4["yaw"] - s3["yaw"]), math.cos(s4["yaw"] - s3["yaw"])))
    ideal = math.degrees(2 * 0.1 * 3.0 / R.WHEEL_SEPARATION)
    res["spin"] = dict(yaw_deg=dyaw, ideal_no_slip_deg=ideal)
    L(f"4. 제자리 회전 (바퀴 ±0.1 m/s x 3 s): {dyaw:.1f}° (미끄럼 없을 때 {ideal:.1f}°)")
    res["finite"] = all(math.isfinite(v) for v in st()["pos"] + st()["q"])
    with open(os.path.join(OUT, f"check_{KIND}_{pose_name}.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=1)
    L(f"끝 (값 정상 {res['finite']}) -> results/check_{KIND}_{pose_name}.json")


if __name__ == "__main__":
    try:
        main()
    except Exception:  # noqa: BLE001  (앱 종료가 예외 출력을 삼키므로 로그에 남김)
        import traceback

        L(traceback.format_exc())
    finally:
        simulation_app.close()
