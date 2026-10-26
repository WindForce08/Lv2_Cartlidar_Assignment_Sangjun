#!/usr/bin/env python3
"""import_cad_urdf.py — CAD 에서 만든 URDF (FreeCAD TeamRobot/URDF/make_urdf.py) 를 이 패키지로 가져온다 (Firmware 팀).

  python3 import_cad_urdf.py <CAD cartridar_01.urdf> [출력, 기본 ../urdf/<같은 파일 이름>]

- 겉모양 (<visual> · 메시 · 재질) 은 뺀다 (메시 23 MB 는 저장소에 넣지 않음). 링크 · 관절 · 충돌 형상 · 관성은 그대로.
- 이름 바꿈 (RENAME, 앞부분 일치): D435 는 Perception 이 쓰는 realsense camera_name (top_camera_front/rear) 으로.
  CAD 의 d435_*_link 원점 = 깊이 원점 (왼쪽 IR) 이라 realsense 드라이버의 <camera_name>_link 와 같은 자리 —
  드라이버가 그 아래에 top_camera_front_color_optical_frame 등을 붙인다.
- 맨 위에 원천 경로 · sha256 · 가져온 시각을 적는다. 출력 파일은 직접 고치지 않는다 — CAD 의 robot_params.json 을 고쳐
  다시 생성한 뒤 이 스크립트로 다시 가져온다. robot.urdf.xacro 가 이 파일을 include 하고 ros2_control 만 얹는다.
"""
import hashlib
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

RENAME = {'d435_front_': 'top_camera_front_', 'd435_rear_': 'top_camera_rear_'}


def renamed(name):
    for old, new in RENAME.items():
        if name.startswith(old):
            return new + name[len(old):]
    return name


def main():
    if len(sys.argv) not in (2, 3):
        sys.exit(__doc__)
    src = Path(sys.argv[1]).resolve()
    out = Path(sys.argv[2]) if len(sys.argv) == 3 else Path(__file__).resolve().parents[1] / 'urdf' / src.name
    data = src.read_bytes()
    root = ET.fromstring(data, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    if root.tag != 'robot':
        sys.exit(f'URDF 가 아님 (루트 {root.tag})')
    n_vis = 0
    for link in root.iter('link'):
        for v in link.findall('visual'):
            link.remove(v)
            n_vis += 1
    mats = root.findall('material')
    for m in mats:
        root.remove(m)
    n_ren = 0
    for el in root.iter():
        if el.tag in ('link', 'joint') or (el.tag in ('parent', 'child') and 'link' in el.attrib):
            key = 'name' if el.tag in ('link', 'joint') else 'link'
            new = renamed(el.get(key))
            if new != el.get(key):
                el.set(key, new)
                n_ren += 1
    root.insert(0, ET.Comment(f' 가져옴: {src} (sha256 {hashlib.sha256(data).hexdigest()[:16]}) {time.strftime("%Y-%m-%d %H:%M")} '
                              f'- scripts/import_cad_urdf.py. 겉모양 (visual {n_vis}, 재질 {len(mats)}) 은 뺌, '
                              f'이름 {", ".join(f"{o}* → {n}*" for o, n in RENAME.items())}. 직접 고치지 말 것 '))
    ET.indent(root, space=' ')
    out.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(out, encoding='utf-8', xml_declaration=True)
    joints = [j.get('name') for j in root.iter('joint') if j.get('type') != 'fixed']
    print(f'{out} — 링크 {len(root.findall("link"))} · 움직이는 관절 {len(joints)}: {", ".join(joints)} · visual {n_vis} 뺌 · '
          f'이름 {n_ren} 곳 바꿈')


if __name__ == '__main__':
    main()
