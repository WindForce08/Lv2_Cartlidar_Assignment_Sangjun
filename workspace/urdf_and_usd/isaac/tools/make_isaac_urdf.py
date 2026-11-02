#!/usr/bin/env python3
"""ROS 패키지 xacro -> Isaac 용 URDF 2개 (정밀 판 · 가벼운 판) + 센서 위치표. ROS 2 (xacro) 가 있는 PC 에서 실행.

    source /opt/ros/<배포판>/setup.bash
    python3 tools/make_isaac_urdf.py

- urdf/waffle_4wd_full.urdf  : 충돌 = 차체 볼록 메시 423 개 (패키지 urdf/waffle_4wd.urdf 와 같음)
- urdf/waffle_4wd_light.urdf : 충돌 = 차체 상자 12 개 (xacro collision:=light)
- 두 파일 모두 <gazebo> 를 빼고, 메시 경로를 package:// 대신 이 폴더 기준 상대 경로 (../../waffle_4wd_description/meshes/...) 로 바꾼다.
- urdf/sensors.json : USD 변환 때 고정 관절 (센서 frame) 이 부모 몸체에 합쳐지므로, 센서마다 붙는 몸체와 그 몸체 기준 위치 · 자세,
  사양 (config/sensors.yaml 의 시뮬 값), 몸체의 USD 경로, LiDAR 스캔면에 걸리는 차체 상자 (자기 가림용) 를 적는다.
팀원은 이 스크립트를 돌릴 필요 없음 (결과 파일이 함께 들어 있음). ROS 패키지 xacro 를 바꿨을 때만 다시 돌리고 convert 도 다시.
"""
import json
import math
import re
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parents[1]             # .../urdf_and_usd/isaac
PKG_NAME = 'waffle_4wd_description'
PKG = HERE.parent / PKG_NAME
XACRO = PKG / 'urdf' / 'waffle_4wd.urdf.xacro'
SENSORS_YAML = PKG / 'config' / 'sensors.yaml'
OUT = HERE / 'urdf'
MESH_PREFIX = f'../../{PKG_NAME}/'                       # urdf/ 에서 패키지 폴더까지

# Isaac 센서 이름 -> (URDF frame, 종류). 카메라는 optical frame (z 앞, y 아래 = Isaac Lab "ros" 규약)
SENSORS = {
    'lds03_front': ('laser_front', 'lidar'),
    'lds03_rear': ('laser_rear', 'lidar'),
    'tfmini': ('tfmini_link', 'range'),
    'imu': ('imu_link', 'imu'),
    'd435_front_color': ('top_camera_front_color_optical_frame', 'camera'),
    'd435_front_infra1': ('top_camera_front_infra1_optical_frame', 'camera'),
    'd435_front_infra2': ('top_camera_front_infra2_optical_frame', 'camera'),
    'd435_rear_color': ('top_camera_rear_color_optical_frame', 'camera'),
    'd435_rear_infra1': ('top_camera_rear_infra1_optical_frame', 'camera'),
    'd435_rear_infra2': ('top_camera_rear_infra2_optical_frame', 'camera'),
    'arm_camera': ('arm_camera_optical_frame', 'camera'),
}


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def quat_xyzw(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return [round(v, 6) for v in (x, y, z, w)]


def origin_tf(el):
    o = el.find('origin')
    T = np.eye(4)
    if o is not None:
        T[:3, 3] = [float(v) for v in o.get('xyz', '0 0 0').split()]
        T[:3, :3] = rpy_matrix(*[float(v) for v in o.get('rpy', '0 0 0').split()])
    return T


XACRO_ARGS = {'full': ['collision:=full'], 'light': ['collision:=light'], 'loaded': ['collision:=light', 'pucks:=true']}   # loaded = 가벼운 판 + 퍽 12개 (주행용 질량)


def xacro(kind):
    return subprocess.run(['xacro', str(XACRO)] + XACRO_ARGS[kind], check=True, capture_output=True, text=True).stdout


def clean(text):
    root = ET.fromstring(text)
    for g in root.findall('gazebo'):
        root.remove(g)
    n = 0
    for m in root.iter('mesh'):
        f = m.get('filename')
        if f.startswith(f'package://{PKG_NAME}/'):
            m.set('filename', f.replace(f'package://{PKG_NAME}/', MESH_PREFIX))
            n += 1
    return root, n


def tree(root):
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    def body(link):          # 고정 관절을 거슬러 올라가 USD 몸체 (움직이는 관절의 자식 또는 루트) 와 그 몸체 기준 변환
        T = np.eye(4)
        while link in joints and joints[link].get('type') == 'fixed':
            T = origin_tf(joints[link]) @ T
            link = joints[link].find('parent').get('link')
        return link, T
    def usd_path(b):         # URDF 가져오기 3.0: Geometry/base_link/<움직이는 몸체 사슬>
        chain = []
        while b in joints:
            chain.append(b)
            b = body(joints[b].find('parent').get('link'))[0]
        return '/'.join(['Geometry', b] + chain[::-1])
    return body, usd_path


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    roots = {}
    for kind in ('full', 'light', 'loaded'):
        root, n = clean(xacro(kind))
        roots[kind] = root
        ET.indent(root)
        path = OUT / f'waffle_4wd_{kind}.urdf'
        path.write_text('<?xml version="1.0" encoding="utf-8"?>\n<!-- 자동 생성: isaac/tools/make_isaac_urdf.py (xacro ' + ' '.join(XACRO_ARGS[kind])
                        + ', gazebo 태그 뺌, 메시는 상대 경로). 직접 고치지 말 것 -->\n' + ET.tostring(root, encoding='unicode') + '\n', encoding='utf-8')
        cols = sum(1 for _ in root.iter('collision'))
        print(f'{path.name}: 메시 경로 {n} 개, 충돌 형상 {cols} 개')

    root = roots['light']
    body, usd_path = tree(root)
    moving = sorted({j.find('child').get('link') for j in root.findall('joint') if j.get('type') != 'fixed'})
    spec = yaml.safe_load(SENSORS_YAML.read_text(encoding='utf-8'))
    sensors = {}
    for name, (frame, kind) in SENSORS.items():
        b, T = body(frame)
        pos = T[:3, 3].copy()
        # D435 컬러 · 깊이도 URDF optical frame 그대로: Isaac 은 근거리 자름 (0.105 m) 때문에 D435 몸체 메시 (frame 앞 0.2 ~ 0.6 mm) 가 안 보임
        # (2026-10-29 시험: 깊이 100 % 유효, 8 mm 앞에 둔 것보다 정확히 8.0 mm 김). Gazebo 는 8 mm 앞 + 중계 노드 보정이라 결과는 같은 frame 기준
        sensors[name] = dict(frame=frame, kind=kind, body=b, usd_body=usd_path(b), pos=[round(v, 6) for v in pos], quat_xyzw=quat_xyzw(T[:3, :3]))

    # LiDAR 자기 가림: 가벼운 판에서 base_link 몸체에 합쳐지는 링크 (차체 · LDS-03 · Jetson ...) 의 상자 · 원통 중 스캔면을 지나는 것
    boxes = []
    for lk in root.findall('link'):
        b, T_link = body(lk.get('name'))
        if b != 'base_link':
            continue
        for i, c in enumerate(lk.findall('collision')):
            g = c.find('geometry')[0]
            if g.tag not in ('box', 'cylinder'):
                continue
            T = T_link @ origin_tf(c)
            if g.tag == 'box':
                size = [float(v) for v in g.get('size').split()]
            else:
                r_, l_ = float(g.get('radius')), float(g.get('length'))
                size = [2 * r_, 2 * r_, l_]                      # 원통은 외접 상자로 (가림 판정용)
            half_z = 0.5 * float(np.abs(T[2, :3]) @ np.array(size))   # 돌아간 상자의 z 반높이
            boxes.append(dict(name=c.get('name') or f'{lk.get("name")}_{i}', link=lk.get('name'), shape=g.tag,
                              pos=[round(v, 6) for v in T[:3, 3]], quat_xyzw=quat_xyzw(T[:3, :3]), size=[round(v, 6) for v in size],
                              z_range=[round(T[2, 3] - half_z, 6), round(T[2, 3] + half_z, 6)]))
    for n in ('lds03_front', 'lds03_rear'):
        z = sensors[n]['pos'][2]
        # 자기 머리 (그 LDS-03 링크) 는 빼고 (광선 시작점이 머리 밖), 다른 LDS-03 은 넣음
        own = 'lds_front' if n == 'lds03_front' else 'lds_rear'
        sensors[n]['occluder_boxes'] = [bx['name'] for bx in boxes if bx['z_range'][0] <= z <= bx['z_range'][1] and bx['link'] != own]

    info = {
        '_설명': '센서 위치는 붙는 몸체 (body) 기준 [m], 자세는 쿼터니언 (x, y, z, w). 카메라는 optical frame (z 앞, y 아래 = Isaac Lab OffsetCfg convention "ros"). '
                 'D435 컬러 · 깊이도 URDF optical frame 그대로 (깊이 = frame 기준, 더할 것 없음). 생성: tools/make_isaac_urdf.py',
        'root_height_m': None,
        'moving_bodies': {b: usd_path(b) for b in moving},
        'sensors': sensors,
        'base_boxes': boxes,
        'spec': {
            'lds03': spec['lds03']['sim'], 'tfmini_plus': spec['tfmini_plus']['sim'], 'd435': spec['d435']['sim'],
            'arm_camera': spec['arm_camera']['sim'], 'imu': spec['imu']['sim'],
        },
    }
    # base_link 원점의 지면 위 높이: 바퀴 관절 z - 바퀴 반지름
    wj = next(j for j in root.findall('joint') if j.get('name') == 'wheel_left_front_joint')
    radius = float(spec['drive']['sim']['wheel_radius_m'])
    info['root_height_m'] = round(radius - float(wj.find('origin').get('xyz').split()[2]), 6)
    (OUT / 'sensors.json').write_text(json.dumps(info, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')
    print(f'sensors.json: 센서 {len(sensors)}, 차체 상자 {len(boxes)}, base_link 지면 위 {info["root_height_m"]} m')
    for n in ('lds03_front', 'lds03_rear'):
        print(f'  {n}: 스캔면을 지나는 상자 {sensors[n]["occluder_boxes"]}')


if __name__ == '__main__':
    main()
