"""팔 URDF (../urdf/arm_bench.urdf) -> USD (Isaac Sim 6.1 URDF 가져오기, Isaac Lab UrdfConverter). 창 없이.

    ./run.sh convert      -> usd/arm_bench_sim/arm_bench_sim.usda (+ payloads/)

- 그리퍼 모터 가상 관절 (gripper_motor_joint, 빈 링크 - 펌웨어 6 번째 칸용) 만 빼고 urdf/arm_bench_sim.urdf 로 적은 뒤 변환
  (질량 없는 몸체는 물리에서 문제). 손가락 2개 (gripper_left/right_joint) 를 직접 움직인다.
- 받침대 (arm_base) 를 월드에 고정 (fix_base). 고정 관절 (link1 · end_effector_link · 카메라) 은 부모에 합침. 자기 충돌 끔.
- 메쉬는 원본 폴더 (../urdf/meshes) 를 그대로 가리킴. 충돌 형상은 URDF 의 상자 그대로.
"""

import argparse
import glob
import os
import shutil
import xml.etree.ElementTree as ET

import _common

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="arm_bench URDF -> USD")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg  # noqa: E402

SRC = os.path.join(_common.BENCH, "urdf", "arm_bench.urdf")
SIM_URDF = os.path.join(_common.ROOT, "urdf", "arm_bench_sim.urdf")
DROP_JOINTS, DROP_LINKS = {"gripper_motor_joint"}, {"gripper_motor_link"}


def write_sim_urdf():
    tree = ET.parse(SRC)
    r = tree.getroot()
    for el in list(r):
        if (el.tag == "joint" and el.get("name") in DROP_JOINTS) or (el.tag == "link" and el.get("name") in DROP_LINKS):
            r.remove(el)
    rel = os.path.relpath(os.path.join(_common.BENCH, "urdf"), os.path.dirname(SIM_URDF))
    for mesh in r.iter("mesh"):
        mesh.set("filename", os.path.join(rel, mesh.get("filename")))
    r.insert(0, ET.Comment(f" 자동 생성: rl/scripts/convert_urdf.py - {SRC} 에서 gripper_motor_joint 만 뺌. 직접 고치지 말 것 "))
    os.makedirs(os.path.dirname(SIM_URDF), exist_ok=True)
    tree.write(SIM_URDF, encoding="utf-8", xml_declaration=True)


def main():
    write_sim_urdf()
    for d in glob.glob(os.path.join(_common.ROOT, "usd", "arm_bench_sim*")):   # 가져오기 3.0 은 폴더가 있으면 _1 로 새로 만듦
        shutil.rmtree(d)
    from arm_pick.robot_cfg import ARM_DAMPING, ARM_STIFFNESS, FINGER_FORCE

    drive = UrdfConverterCfg.JointDriveCfg(
        drive_type="force",
        target_type="position",
        gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
            stiffness={**ARM_STIFFNESS, "gripper_.*_joint": FINGER_FORCE / 0.002},
            damping={"joint[1-4]": ARM_DAMPING, "wrist_roll_joint": ARM_DAMPING, "gripper_.*_joint": FINGER_FORCE / 0.002 * 0.02},
        ),
    )
    cfg = UrdfConverterCfg(
        asset_path=SIM_URDF,
        usd_dir=os.path.join(_common.ROOT, "usd"),
        fix_base=True,
        merge_fixed_joints=True,
        self_collision=False,
        collision_from_visuals=False,
        force_usd_conversion=True,
        joint_drive=drive,
    )
    conv = UrdfConverter(cfg)
    print(f"[arm_bench] USD: {conv.usd_path}")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
