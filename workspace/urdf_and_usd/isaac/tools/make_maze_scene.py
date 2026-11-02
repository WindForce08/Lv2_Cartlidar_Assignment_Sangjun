#!/usr/bin/env python3
"""주사위 경기장 (dice_mission.usda) + waffle_4wd USD 장면 -> scenes/dice_mission_waffle_{full,light}.usda. pxr 없이 글로 쓴다.

    python3 tools/make_maze_scene.py                      # 기본: 출발 구역 (start_zone) 가운데, 신호등 (-X) 을 봄
    python3 tools/make_maze_scene.py --cell 0 0 --yaw 0   # 칸 (i, j) 가운데 (0 0 = 가장 왼쪽 아래), +X 를 봄

- 경기장: scenes/dice_mission.usda (사용자 제공, sdf_to_usd.py 로 worlds/dice_mission.sdf 를 옮긴 것) 를 바탕 레이어 (subLayers) 로.
  좌표 m, Z 위, 원점 = 경기장 왼쪽 아래 기둥 중심, 칸 간격 0.734 m (4.404 x 6.304 m = 6 x 8 칸 남짓). 물리 장면이 없어서 여기서 더함.
- 로봇: ../usd/<판>/waffle_4wd_<판>/waffle_4wd_<판>.usda 를 /World/waffle_4wd 로 참조.
  자리 = 차체 외곽 (팔 빼고, 뒤 끝 x -0.226 ~ TFmini 앞 0.140, 바퀴 y ±0.1676) 의 가운데를 맞출 점.
    기본 (사용자 2026-10-30): 출발 구역 start_zone (0.6 x 0.6 m 바닥 표시, 가운데 (0.734, 0.95)) 가운데, 방향 π = 신호등 (x 0.08) 을 봄
    (발제 주석 "로봇 출발 · 관측: (0.734 0.95) 에서 -X 를 봄"). --cell i j 를 주면 그 칸 가운데.
  방향 --yaw [rad] (0 = 지도 +X 를 봄). 높이 = 지면 위 base_link 높이 + 3 mm (robot_cfg 와 같음).
- 팔 · 손가락: robot_params.json 의 기본 자세 (drive) 를 관절 시작 각과 구동 목표로 (USD 단위 도 · m).
"""
import argparse
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]                # .../isaac
SCENES = HERE / 'scenes'
CELL = 0.734                                               # 경기장 칸 간격 [m]
START_ZONE = (0.734, 0.95, 0.6)                            # 출발 구역 가운데 x, y · 한 변 [m] (worlds/dice_mission.sdf start_zone)
BODY_X = (-0.2135, 0.1328)                                 # 차체 외곽 x (뒤 끝 ~ 상판 앞 끝) [m], base_link 기준. B안 모델: 뒤 무리 +12.5 mm, TFmini 렌즈 x 0.130
BODY_Y = 0.1676                                            # 바퀴 바깥 |y|


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cell', type=int, nargs=2, default=None, help='칸 번호 i (x) j (y), 0 0 = 가장 왼쪽 아래 (없으면 출발 구역)')
    ap.add_argument('--yaw', type=float, default=math.pi, help='로봇 방향 [rad], 0 = 지도 +X, π = 신호등 쪽 (기본)')
    a = ap.parse_args()

    params = json.loads((HERE / 'robot_params.json').read_text(encoding='utf-8'))
    pose = params['poses'][params['poses']['default']['value']]['value']
    root_z = json.loads((HERE / 'urdf/sensors.json').read_text(encoding='utf-8'))['root_height_m'] + 0.003

    if a.cell is None:
        cx, cy, side = START_ZONE
        where, place = '출발 구역 (start_zone)', 'start_zone'
    else:
        cx, cy, side = (a.cell[0] + 0.5) * CELL, (a.cell[1] + 0.5) * CELL, CELL
        where, place = f'칸 ({a.cell[0]}, {a.cell[1]})', f'--cell {a.cell[0]} {a.cell[1]}'
    off = -(BODY_X[0] + BODY_X[1]) / 2                     # 차체 외곽 가운데 -> base_link (로봇 좌표 x)
    x, y = cx + off * math.cos(a.yaw), cy + off * math.sin(a.yaw)
    qw, qz = math.cos(a.yaw / 2), math.sin(a.yaw / 2)
    margin = min(side / 2 - (BODY_X[1] - BODY_X[0]) / 2, side / 2 - BODY_Y)   # 방향 0 · π 기준

    joints = []
    for name, v in pose.items():
        prismatic = name.startswith('gripper_') and name.endswith('_joint')
        val = v if prismatic else math.degrees(v)
        kind = 'linear' if prismatic else 'angular'
        joints.append(f'''            over "{name}" (
                prepend apiSchemas = ["PhysicsJointStateAPI:{kind}"]
            )
            {{
                float drive:{kind}:physics:targetPosition = {val:.6g}
                float state:{kind}:physics:position = {val:.6g}
            }}
''')

    SCENES.mkdir(exist_ok=True)
    for kind in ('full', 'light'):
        robot = f'../usd/{kind}/waffle_4wd_{kind}/waffle_4wd_{kind}.usda'
        assert (SCENES / robot).resolve().exists(), f'{robot} 없음 - 먼저 ./run.sh convert'
        txt = f'''#usda 1.0
(
    doc = """주사위 경기장 + waffle_4wd ({kind}). 자동 생성: isaac/tools/make_maze_scene.py ({place}, --yaw {a.yaw:g}). 직접 고치지 말 것.
로봇: {where} 가운데 (차체 외곽 기준, 가장자리까지 {margin * 1000:.0f} mm 남음), 방향 {math.degrees(a.yaw):g} deg, 팔 = {params['poses']['default']['value']} 자세."""
    defaultPrim = "World"
    kilogramsPerUnit = 1
    metersPerUnit = 1
    upAxis = "Z"
    subLayers = [
        @./dice_mission.usda@
    ]
)

over "World"
{{
    def PhysicsScene "physicsScene"
    {{
        vector3f physics:gravityDirection = (0, 0, -1)
        float physics:gravityMagnitude = 9.81
    }}

    def Xform "waffle_4wd" (
        prepend references = @{robot}@
    )
    {{
        double3 xformOp:translate = ({x:.6f}, {y:.6f}, {root_z:.6f})
        quatd xformOp:orient = ({qw:.9f}, 0, 0, {qz:.9f})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]

        over "Physics"
        {{
{''.join(joints)}        }}
    }}
}}
'''
        out = SCENES / f'dice_mission_waffle_{kind}.usda'
        out.write_text(txt, encoding='utf-8')
        print(f'{out.relative_to(HERE)}: {where} 가운데, 로봇 base_link ({x:.4f}, {y:.4f}, {root_z:.5f}) m, 방향 {math.degrees(a.yaw):g} deg, '
              f'가장자리까지 {margin * 1000:.0f} mm')


if __name__ == '__main__':
    main()
