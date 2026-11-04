import glob
import os

from setuptools import find_packages, setup

package_name = "mission_planner"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob.glob("config/*.yaml")),
        (os.path.join("share", package_name, "maps"), glob.glob("maps/*.pgm") + glob.glob("maps/*.yaml")),
        (os.path.join("share", package_name, "worlds"), glob.glob("worlds/*.sdf")),
        (os.path.join("share", package_name, "launch"), glob.glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lv2 Cartlidar",
    maintainer_email="kanichong1@gmail.com",
    description="주사위 기억 로봇: 할당 · 적재 · 경로 계획기 (GTSP), 문서 좌표 정적 지도 · 접근 자세표",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "mission_make_maps = mission_planner.cli:make_maps",
        "mission_plan = mission_planner.cli:plan_cmd",
        "mission_planner_node = mission_planner.planner_node:main",
    ]},
)
