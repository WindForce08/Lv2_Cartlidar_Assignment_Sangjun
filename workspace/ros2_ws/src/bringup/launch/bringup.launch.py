# 시연 실행: 카메라 드라이버 + 인지 + 판단 + 제어를 한 번에 띄움
#
#   ros2 launch bringup bringup.launch.py
#   ros2 launch bringup bringup.launch.py debug_image:=true             # PC 에서 오버레이 영상 확인
#   ros2 launch bringup bringup.launch.py debug_mask:=true              # PC 에서 마스크 영상 확인
#   ros2 launch bringup bringup.launch.py use_camera:=false             # bag 재생으로 시험
#   ros2 launch bringup bringup.launch.py use_control:=false            # OpenCR 없이 시험
#
# rqt·bag 녹화처럼 이 launch 밖에서 켜는 도구는 FASTDDS_DEFAULT_PROFILES_FILE 을 직접 지정해야 함

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    config_dir = os.path.join(get_package_share_directory('bringup'), 'config')
    camera_launch = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')

    debug_image = LaunchConfiguration('debug_image')
    debug_mask = LaunchConfiguration('debug_mask')
    use_camera = LaunchConfiguration('use_camera')
    use_control = LaunchConfiguration('use_control')
    port = LaunchConfiguration('port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'debug_image', default_value='false',
            description='인지 오버레이 영상 /target/debug_image 발행'),
        DeclareLaunchArgument(
            'debug_mask', default_value='false',
            description='인지 마스크 영상 /target/debug_mask 발행'),
        DeclareLaunchArgument(
            'use_camera', default_value='true',
            description='카메라 드라이버 실행 (bag 재생 시 false)'),
        DeclareLaunchArgument(
            'use_control', default_value='true',
            description='제어 노드 실행 (OpenCR 없이 시험 시 false)'),
        DeclareLaunchArgument(
            'port', default_value='/dev/opencr',
            description='OpenCR 시리얼 포트'),

        # 640x480 영상 프레임 손실 방지, 아래 모든 노드에 적용 (반드시 노드보다 먼저)
        SetEnvironmentVariable(
            'FASTDDS_DEFAULT_PROFILES_FILE', os.path.join(config_dir, 'fastdds_shm_big.xml')),

        # 1. 카메라 드라이버: color 640x480@30 + color 에 정렬된 depth (인지 거리 필터용)
        #    forwarding=False: bringup 인자(debug_image 등)는 넘기지 않고 아래 4개만 전달
        #    (넘기면 rs_launch.py 가 모르는 인자마다 'is not supported' 경고 출력)
        GroupAction(
            condition=IfCondition(use_camera),
            scoped=True,
            forwarding=False,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(camera_launch),
                    launch_arguments={
                        'rgb_camera.color_profile': '640,480,30',
                        'depth_module.depth_profile': '640,480,30',
                        'enable_depth': 'true',
                        'align_depth.enable': 'true',
                    }.items()),
            ]),

        # 2. 인지: perception.yaml 은 perception 패키지에 설치된 기본 경로 사용
        Node(
            package='perception',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[{
                'publish_debug_image': ParameterValue(debug_image, value_type=bool),
                'publish_debug_mask': ParameterValue(debug_mask, value_type=bool),
            }]),

        # 3. 판단: 상태 전이 + 모터 목표 계산
        Node(
            package='center',
            executable='center_node',
            name='center_node',
            output='screen',
            parameters=[os.path.join(config_dir, 'center.yaml')]),

        # 4. 제어: OpenCR 시리얼
        Node(
            package='control',
            executable='control_node',
            name='control_node',
            output='screen',
            condition=IfCondition(use_control),
            parameters=[{'port': port}]),
    ])
