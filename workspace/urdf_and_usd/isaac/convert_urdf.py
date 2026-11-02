"""waffle_4wd URDF -> USD 2개 (가벼운 판 · 정밀 판), Isaac Sim 6.1 URDF 가져오기 3.0 (Isaac Lab UrdfConverter). 창 없이 실행.

    ./run.sh convert            (= <Isaac Lab python> convert_urdf.py)
    -> usd/light/waffle_4wd_light/waffle_4wd_light.usda   (충돌: 차체 상자, 학습 기본)
       usd/full/waffle_4wd_full/waffle_4wd_full.usda    (충돌: 차체 볼록 메시 423 개, 퍽 · 보관함 · 벽 가까이 시험)

변환 뒤 robot_params.json 값을 USD 맨 위 레이어에 써 넣는다 (Isaac Sim 창에서 USD 만 열어도 같은 값으로 움직이게):
관절 구동 (바퀴 = 속도, 팔 · 그리퍼 모터 · 손가락 = 위치) 강성 · 감쇠 · 힘 한계 · 최대 속도 · armature,
자기 충돌 켬 + 팔 몸체끼리 충돌 뺌, 로봇 전체 마찰 재질. 차체는 고정하지 않고 (이동 로봇), 고정 관절 (센서 frame 등) 은 부모 몸체에 합친다.
"""

import argparse
import glob
import json
import math
import os
import shutil

from isaaclab.app import AppLauncher

HERE = os.path.dirname(os.path.abspath(__file__))
parser = argparse.ArgumentParser(description="waffle_4wd URDF -> USD")
parser.add_argument("--only", choices=["light", "full", "loaded"], default=None, help="한 판만 변환 (기본: 셋 다)")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Sdf, Usd, UsdPhysics, UsdShade  # noqa: E402

from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg  # noqa: E402

from waffle_4wd_isaac.robot_cfg import (  # noqa: E402
    ARM_BODIES,
    ARM_JOINTS,
    FINGER_JOINTS,
    PARAMS,
    URDF_PATHS,
    USD_PATHS,
    WHEEL_JOINTS,
)

W, A, F, C = PARAMS["wheels"], PARAMS["arm"], PARAMS["fingers"], PARAMS["contact"]


def convert(kind: str) -> str:
    usd_dir = os.path.join(HERE, "usd", kind)
    if os.path.isdir(usd_dir):          # 가져오기 3.0 은 출력 폴더가 있으면 <이름>_1 로 새로 만들므로 지우고 같은 경로에
        shutil.rmtree(usd_dir)
    arm = "joint[1-5]|gripper_roll"
    drive = UrdfConverterCfg.JointDriveCfg(
        drive_type="force",
        target_type={"wheel_.*_joint": "velocity", arm: "position", "gripper_(left|right)_joint": "position"},
        gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
            stiffness={"wheel_.*_joint": 0.0, arm: A["stiffness"], "gripper_(left|right)_joint": F["stiffness"]},
            damping={"wheel_.*_joint": W["damping"], arm: A["damping"], "gripper_(left|right)_joint": F["damping"]},
        ),
    )
    cfg = UrdfConverterCfg(
        asset_path=URDF_PATHS[kind],
        usd_dir=usd_dir,
        fix_base=False,
        merge_fixed_joints=True,
        self_collision=bool(C["self_collision"]),
        collision_from_visuals=False,
        robot_type="Mobile Manipulators",
        force_usd_conversion=True,
        joint_drive=drive,
    )
    conv = UrdfConverter(cfg)
    out = glob.glob(os.path.join(usd_dir, "*", "*.usda"))[0]
    if os.path.realpath(out) != os.path.realpath(USD_PATHS[kind]):
        raise RuntimeError(f"USD 위치가 예상과 다름: {out} (robot_cfg.USD_PATHS 를 고칠 것)")
    print(f"[waffle_4wd] {kind}: {conv.usd_path}")
    return out


def bake(usd_path: str) -> dict:
    """robot_params.json 값을 USD 맨 위 레이어 (waffle_4wd.usda) 에 over 로 씀. payload 레이어는 그대로."""
    stage = Usd.Stage.Open(usd_path)
    stage.SetEditTarget(stage.GetRootLayer())
    robot = stage.GetDefaultPrim()
    rp = robot.GetPath()
    rep = dict(joints=0, filtered_pairs=0)

    def setf(prim, name, value, api=None):
        if api and api not in prim.GetAppliedSchemas():
            prim.AddAppliedSchema(api)
        prim.CreateAttribute(name, Sdf.ValueTypeNames.Float, custom=False).Set(float(value))

    for j in stage.GetPrimAtPath(rp.AppendChild("Physics")).GetChildren():
        n = j.GetName()
        if n in WHEEL_JOINTS:
            eff, vel, arm, kind = W["effort_limit"], W["velocity_limit"], W["armature"], "angular"
        elif n in ARM_JOINTS:
            eff, vel, arm, kind = A["effort_limit"], A["velocity_limit"], A["armature"], "angular"
        elif n in FINGER_JOINTS:
            eff, vel, arm, kind = F["force_limit"], F["velocity_limit"], 0.0, "linear"
        else:
            continue
        # PhysX 각 관절 속도는 [deg/s], 직선 관절은 [m/s]. armature 는 [kg·m²] / [kg]
        setf(j, "physxJoint:maxJointVelocity", math.degrees(vel) if kind == "angular" else vel, "PhysxJointAPI")
        setf(j, "physxJoint:armature", arm, "PhysxJointAPI")
        setf(j, f"drive:{kind}:physics:maxForce", eff)
        rep["joints"] += 1

    # 자기 충돌: 관절체 루트에서 켜고, 팔 몸체끼리는 FilteredPairs 로 뺌 (팔 ↔ 차체 · 바퀴만 부딪힘)
    root_body = None
    bodies = {}
    for p in Usd.PrimRange(robot):
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            root_body = p
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            bodies[p.GetName()] = p
    root_body.AddAppliedSchema("PhysxArticulationAPI")
    root_body.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool, custom=False).Set(bool(C["self_collision"]))
    missing = [b for b in ARM_BODIES if b not in bodies]
    if missing:
        raise RuntimeError(f"팔 몸체 {missing} 를 USD 에서 못 찾음")
    for i, a in enumerate(ARM_BODIES):
        others = [bodies[b].GetPath() for b in ARM_BODIES[i + 1:]]
        if others:
            UsdPhysics.FilteredPairsAPI.Apply(bodies[a]).CreateFilteredPairsRel().SetTargets(others)
            rep["filtered_pairs"] += len(others)

    # 로봇 전체 마찰 재질 (자식 충돌 형상이 따로 재질을 가져도 이 값이 이김)
    mat = UsdShade.Material.Define(stage, rp.AppendChild("PhysicsMaterials").AppendChild("robot_contact"))
    mapi = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    mapi.CreateStaticFrictionAttr(float(C["static_friction"]))
    mapi.CreateDynamicFrictionAttr(float(C["dynamic_friction"]))
    mapi.CreateRestitutionAttr(0.0)
    geom = stage.GetPrimAtPath(rp.AppendChild("Geometry"))
    UsdShade.MaterialBindingAPI.Apply(geom).Bind(mat, UsdShade.Tokens.strongerThanDescendants, "physics")
    stage.GetRootLayer().Save()
    rep["bodies"] = sorted(bodies)
    return rep


def main():
    kinds = [args_cli.only] if args_cli.only else ["light", "full", "loaded"]
    log = {}
    for k in kinds:
        path = convert(k)
        log[k] = dict(usd=os.path.relpath(path, HERE), **bake(path))
        print(f"[waffle_4wd] {k}: 관절 {log[k]['joints']} 개 값 씀, 팔 충돌 뺀 쌍 {log[k]['filtered_pairs']}, 몸체 {len(log[k]['bodies'])}")
    os.makedirs(os.path.join(HERE, "results"), exist_ok=True)
    with open(os.path.join(HERE, "results", "convert.json"), "w", encoding="utf-8") as f:
        json.dump(log, f, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    try:
        main()
    except Exception:  # noqa: BLE001  (앱 종료가 예외 출력을 삼키므로 직접 찍음)
        import traceback

        traceback.print_exc()
        raise
    finally:
        simulation_app.close()
