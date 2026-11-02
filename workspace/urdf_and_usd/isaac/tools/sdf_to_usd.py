"""dice_mission.sdf (Gazebo 주사위 경기장) -> USD (Isaac Sim 6.1 / Isaac Lab 3.0 에서 열기).

SDF 는 static 모델 17 개, 상자 · 원통 · 평면만 쓰므로 형상을 그대로 옮긴다:
  visual    -> 색 입힌 Cube · Cylinder · 사각 Mesh (UsdPreviewSurface, SDF diffuse 색)
  collision -> 같은 형상 + UsdPhysics.CollisionAPI, purpose guide (화면에 안 그림), 강체 아님 (정적)
  plane     -> 겉모양은 사각 메시, 충돌은 두께 2 cm 상자 (윗면 z = 0)
  마찰      -> 정지 · 미끄럼 1.0 (Gazebo 기본값 = waffle_4wd robot_params.json 과 같음)
  빛        -> SDF 해 방향 그대로 DistantLight + 주변광 DomeLight
좌표 · 단위는 SDF 와 같다 (m, Z 위, 원점 = 경기장 왼쪽 아래 기둥 중심).

Isaac Lab 설치 Python (pxr 포함) 으로 실행, Isaac Sim 을 띄우지 않음 (몇 초):
    ~/pa-course/IsaacLab-6.1/.venv/bin/python sdf_to_usd.py [입력.sdf] [출력.usda]
"""

import math
import os
import sys
import xml.etree.ElementTree as ET

from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

HERE = os.path.dirname(os.path.abspath(__file__))
SDF_DEFAULT = os.path.join(HERE, "..", "..", "waffle_4wd_description", "worlds", "dice_mission.sdf")
OUT_DEFAULT = os.path.join(HERE, "..", "scenes", "dice_mission.usda")
FRICTION = 1.0
PLANE_THICKNESS = 0.02


def floats(text, n):
    v = [float(x) for x in (text or "").split()]
    return v if len(v) == n else [0.0] * n


def rpy_quat(r, p, y):
    """SDF RPY (고정축 X -> Y -> Z, R = Rz·Ry·Rx) -> Gf.Quatf"""
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return Gf.Quatf(cr * cp * cy + sr * sp * sy,
                    Gf.Vec3f(sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy))


def set_pose(prim, pose_text, scale=None):
    x, y, z, r, p, yw = floats(pose_text, 6)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    xf.AddOrientOp().Set(rpy_quat(r, p, yw))
    if scale is not None:
        xf.AddScaleOp().Set(Gf.Vec3f(*scale))


def unique_child(stage, parent, name):
    base = Tf.MakeValidIdentifier(name)
    path, i = parent.AppendChild(base), 1
    while stage.GetPrimAtPath(path):
        path, i = parent.AppendChild(f"{base}_{i}"), i + 1
    return path


class Converter:
    def __init__(self, stage):
        self.stage = stage
        self.looks = Sdf.Path("/World/Looks")
        UsdGeom.Scope.Define(stage, self.looks)
        self.materials = {}
        self.count = {"visual": 0, "collision": 0}

    def material(self, rgb):
        key = tuple(round(c, 4) for c in rgb)
        if key not in self.materials:
            name = "mat_" + "_".join(f"{int(round(c * 1000)):04d}" for c in key)
            mat = UsdShade.Material.Define(self.stage, self.looks.AppendChild(name))
            sh = UsdShade.Shader.Define(self.stage, mat.GetPath().AppendChild("Shader"))
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*key))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.7)
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            self.materials[key] = mat
        return self.materials[key]

    def shape(self, parent, el, kind):
        """visual / collision 하나 -> 형상 prim. 돌려줌: prim"""
        geom = el.find("geometry")
        path = unique_child(self.stage, parent, el.get("name") or kind)
        pose = el.findtext("pose")
        if geom.find("box") is not None:
            size = floats(geom.find("box").findtext("size"), 3)
            g = UsdGeom.Cube.Define(self.stage, path)
            g.CreateSizeAttr(1.0)
            set_pose(g.GetPrim(), pose, scale=size)
        elif geom.find("cylinder") is not None:
            c = geom.find("cylinder")
            g = UsdGeom.Cylinder.Define(self.stage, path)
            g.CreateRadiusAttr(float(c.findtext("radius")))
            g.CreateHeightAttr(float(c.findtext("length")))
            g.CreateAxisAttr("Z")
            set_pose(g.GetPrim(), pose)
        elif geom.find("plane") is not None:
            w, h = floats(geom.find("plane").findtext("size"), 2)
            if kind == "collision":   # 평면 충돌 = 얇은 상자, 윗면이 z 0
                g = UsdGeom.Cube.Define(self.stage, path)
                g.CreateSizeAttr(1.0)
                x, y, z, r, p, yw = floats(pose, 6)
                set_pose(g.GetPrim(), f"{x} {y} {z - PLANE_THICKNESS / 2} {r} {p} {yw}", scale=(w, h, PLANE_THICKNESS))
            else:
                g = UsdGeom.Mesh.Define(self.stage, path)
                g.CreatePointsAttr([(-w / 2, -h / 2, 0), (w / 2, -h / 2, 0), (w / 2, h / 2, 0), (-w / 2, h / 2, 0)])
                g.CreateFaceVertexCountsAttr([4])
                g.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
                g.CreateNormalsAttr([(0, 0, 1)] * 4)
                g.CreateExtentAttr([(-w / 2, -h / 2, 0), (w / 2, h / 2, 0)])
                g.CreateSubdivisionSchemeAttr("none")
                set_pose(g.GetPrim(), pose)
        else:
            raise ValueError(f"모르는 형상: {ET.tostring(geom, encoding='unicode')[:80]}")
        prim = g.GetPrim()
        if kind == "visual":
            m = el.find("material")
            rgb = floats(m.findtext("diffuse") if m is not None else "0.7 0.7 0.7 1", 4)[:3]
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.material(rgb))
        else:
            UsdPhysics.CollisionAPI.Apply(prim)
            g.CreatePurposeAttr(UsdGeom.Tokens.guide)
        self.count[kind] += 1
        return prim

    def model(self, world_path, m):
        mp = unique_child(self.stage, world_path, m.get("name"))
        set_pose(UsdGeom.Xform.Define(self.stage, mp).GetPrim(), m.findtext("pose"))
        for link in m.findall("link"):
            lp = unique_child(self.stage, mp, link.get("name") or "link")
            set_pose(UsdGeom.Xform.Define(self.stage, lp).GetPrim(), link.findtext("pose"))
            for v in link.findall("visual"):
                self.shape(lp, v, "visual")
            for c in link.findall("collision"):
                self.shape(lp, c, "collision")


def light_orient(direction):
    """DistantLight 는 local −Z 로 비춤 -> −Z 를 direction 으로 돌리는 Gf.Quatf"""
    rot = Gf.Rotation(Gf.Vec3d(0, 0, -1), Gf.Vec3d(*direction).GetNormalized())
    return Gf.Quatf(rot.GetQuat())


def main():
    src = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else SDF_DEFAULT)
    out = os.path.abspath(sys.argv[2] if len(sys.argv) > 2 else OUT_DEFAULT)
    world = ET.parse(src).getroot().find("world")

    stage = Usd.Stage.CreateNew(out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(root.GetPrim())
    stage.GetRootLayer().customLayerData = {"source": os.path.basename(src), "generator": "sdf_to_usd.py"}

    conv = Converter(stage)
    for m in world.findall("model"):
        conv.model(root.GetPath(), m)

    # 마찰: /World 아래 모든 충돌에 물려받게 (physics 용도 바인딩)
    pm = UsdShade.Material.Define(stage, conv.looks.AppendChild("physics_contact"))
    pma = UsdPhysics.MaterialAPI.Apply(pm.GetPrim())
    pma.CreateStaticFrictionAttr(FRICTION)
    pma.CreateDynamicFrictionAttr(FRICTION)
    pma.CreateRestitutionAttr(0.0)
    UsdShade.MaterialBindingAPI.Apply(root.GetPrim()).Bind(pm, UsdShade.Tokens.weakerThanDescendants, "physics")

    # 빛: SDF 해 방향 + 주변광
    sun_el = world.find("light")
    sun_dir = floats(sun_el.findtext("direction"), 3) if sun_el is not None else [-0.3, 0.2, -1.0]
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/sun")
    sun.CreateIntensityAttr(2500.0)
    sun.CreateAngleAttr(1.0)
    UsdGeom.Xformable(sun).AddOrientOp().Set(light_orient(sun_dir))
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/ambient")
    dome.CreateIntensityAttr(1200.0)

    stage.GetRootLayer().Save()
    print(f"{out}\n  모델 {len(world.findall('model'))}, visual {conv.count['visual']}, collision {conv.count['collision']}, "
          f"색 {len(conv.materials)}, 마찰 {FRICTION}")


if __name__ == "__main__":
    main()
