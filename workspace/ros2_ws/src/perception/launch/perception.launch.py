"""인지 노드를 한 컨테이너(perception_container)에 띄운다.

ros2 launch perception perception.launch.py use_sim_time:=true          # 시뮬레이션 (Isaac Sim 이 영상·라이다 발행)
ros2 launch perception perception.launch.py use_tfmini:=true           # 실물: TFmini 시리얼 읽기도 실행
선택 인자: config_dir:=<폴더>(그 폴더의 perception.yaml · lidar.yaml · tfmini.yaml · puck_survey.yaml 사용 — 실험용 설정 사본)
          use_survey:=false(B0 퍽 등록부 끔), survey_output:=<파일>(조사 완료 JSON, 기본 ~/.ros/puck_survey/puck_survey.json)

카메라·라이다 드라이버는 이 launch 에 없다. 시뮬레이션은 Isaac Sim, 실물은 드라이버 launch 가 같은 토픽 이름으로 발행한다.
"""
import atexit
import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

DEFAULT_CONFIG = os.path.join(get_package_share_directory('perception'), 'config')


def override(node, **values):
    """launch 인자 값을 노드 이름 항목의 임시 yaml 로 써서 맨 뒤 파일로 넘긴다.
    dict 나 -p 로 넘기면 설정 yaml 의 노드 이름 항목에 덮여 무시된다(이전 모듈 2026-10-07 시험)."""
    f = tempfile.NamedTemporaryFile('w', prefix=f'{node}_launch_', suffix='.yaml', delete=False)
    atexit.register(lambda path=f.name: os.path.exists(path) and os.remove(path))
    yaml.safe_dump({node: {'ros__parameters': values}}, f)
    f.close()
    return f.name


def as_bool(text):
    return str(text).lower() in ('true', '1', 'yes')


def _setup(context):
    cfg = context.launch_configurations['config_dir']
    sim = as_bool(context.launch_configurations['use_sim_time'])

    def node(plugin, name, yaml_file):
        return ComposableNode(
            package='perception', plugin=plugin, name=name,
            parameters=[os.path.join(cfg, yaml_file), override(name, use_sim_time=sim)],
            extra_arguments=[{'use_intra_process_comms': True}])

    nodes = [
        node('perception::PerceptionNode', 'perception_node', 'perception.yaml'),
        node('perception::ScanMergerNode', 'scan_merger', 'lidar.yaml'),
        node('perception::ObstacleStopNode', 'obstacle_stop', 'lidar.yaml'),
    ]
    if as_bool(context.launch_configurations['use_tfmini']):
        nodes.append(node('perception::TFminiNode', 'tfmini_node', 'tfmini.yaml'))
    actions = [ComposableNodeContainer(
        name='perception_container', namespace='', package='rclcpp_components',
        executable='component_container_mt', output='screen', composable_node_descriptions=nodes)]
    if as_bool(context.launch_configurations['use_survey']):   # B0 퍽 등록부: Python 노드라 컨테이너 밖 프로세스
        out = os.path.expanduser(context.launch_configurations['survey_output'])
        actions.append(Node(
            package='perception_survey', executable='puck_survey', name='puck_survey', output='screen',
            parameters=[os.path.join(cfg, 'puck_survey.yaml'),
                        override('puck_survey', use_sim_time=sim, output_path=out)]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='시뮬레이션 시계 사용'),
        DeclareLaunchArgument('use_tfmini', default_value='false', description='TFmini 시리얼 읽기 실행 (실물)'),
        DeclareLaunchArgument('config_dir', default_value=DEFAULT_CONFIG, description='설정 폴더'),
        DeclareLaunchArgument('use_survey', default_value='true', description='B0 퍽 등록부 (/perception/puck_survey)'),
        DeclareLaunchArgument('survey_output', default_value='~/.ros/puck_survey/puck_survey.json',
                              description='조사 완료 JSON 경로 (관측 이력은 같은 이름 .observations.jsonl)'),
        OpaqueFunction(function=_setup),
    ])
