# 센서 드라이버 (실물): 앞 · 뒤 D435 (realsense2_camera) · 팔 Arducam (usb_cam) · 앞 · 뒤 LDS-03 (coin_d4_driver)
#
#   ros2 launch bringup sensors.launch.py front_serial:=<앞 D435 시리얼> rear_serial:=<뒤 D435 시리얼>
#   ros2 launch bringup sensors.launch.py use_lidar:=false          # 하나씩 끄기: use_d435 · use_arm_camera · use_lidar
#   bringup.launch.py 가 use_sensors 로 포함한다 (sim:=true 면 기본 끔 — Isaac 브리지가 같은 토픽을 낸다)
#
# 토픽 · frame_id 는 perception.yaml · lidar.yaml 기본값과 Isaac 브리지 (workspace/tools/isaac_sim/waffle_bridge.py) 와 같다 (docs/frames_tf.md).
#   D435   /top_camera_{front,rear}/color/image_raw · color/camera_info · aligned_depth_to_color/image_raw (16UC1 mm)
#          frame_id top_camera_*_color_optical_frame. 드라이버가 top_camera_*_link (URDF) 아래 광학 프레임 TF 를 낸다
#   Arducam /arm_camera/image_raw · camera_info, frame_id arm_camera_optical_frame (URDF)
#   LDS-03 /scan_front · /scan_rear, frame_id laser_front · laser_rear (URDF, roll π 로 뒤집어 단 것을 반영 → 드라이버 reverse 는 끔)
#
# 드라이버 패키지가 없으면 그 센서만 경고하고 건너뛴다 (realsense2_camera · usb_cam · coin_d4_driver 는 apt / 소스로 따로 설치).
# ★ = 실물에서 확인할 값: D435 시리얼 두 개, Arducam 보정 파일 (camera_info_url), /dev/arm_cam · /dev/lidar_* (perception/udev/99-sensors.rules)
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch_ros.actions import Node

D435_PROFILE = '640,480,30'   # perception 컬러 640x480 30 Hz (depth.sync_tol_s 0.02 s 근거와 같은 설정)


def installed(pkg):
    try:
        get_package_share_directory(pkg)
        return True
    except PackageNotFoundError:
        return False


def as_bool(v):
    return v.lower() in ('true', '1', 'yes')


def launch_setup(context):
    lc = context.launch_configurations
    sim = {'use_sim_time': as_bool(lc['use_sim_time'])}
    actions = []

    def missing(pkg, what):
        actions.append(LogInfo(msg=f'[sensors] {pkg} 패키지가 없어 {what} 를 띄우지 않습니다 (설치 후 다시 실행)'))

    if as_bool(lc['use_d435']):
        # RGB 노출 (#89): 자동이면 어두울 때 노출이 길어져 흐림 -> 기본 수동 고정. 깊이 (depth_module) 는 그대로
        if as_bool(lc['rgb_auto_exposure']):
            rgb = {'rgb_camera.enable_auto_exposure': True}
        else:
            rgb = {'rgb_camera.enable_auto_exposure': False,
                   'rgb_camera.exposure': int(lc['rgb_exposure']), 'rgb_camera.gain': int(lc['rgb_gain'])}
        if installed('realsense2_camera'):
            for name, serial in (('top_camera_front', lc['front_serial']), ('top_camera_rear', lc['rear_serial'])):
                if not serial:
                    actions.append(LogInfo(msg=f'[sensors] ★ {name} 시리얼이 비었습니다 — D435 두 대면 어느 쪽이 앞인지 정해지지 않습니다'))
                # 노드 이름 = camera_name, 이름공간 '' → 토픽 /<camera_name>/..., frame_id <camera_name>_color_optical_frame
                actions.append(Node(
                    package='realsense2_camera', executable='realsense2_camera_node', name=name, namespace='',
                    output='screen',
                    parameters=[{
                        'camera_name': name,
                        'serial_no': serial,   # 파이썬 문자열이라 숫자로 바뀌지 않음 (rs_launch.py 의 "''" 따옴표는 명령줄 인자용). 빈 값 = 아무 장치
                        'enable_color': True, 'enable_depth': True,
                        'enable_infra1': False, 'enable_infra2': False, 'enable_gyro': False, 'enable_accel': False,
                        'rgb_camera.color_profile': D435_PROFILE,
                        **rgb,
                        'depth_module.depth_profile': D435_PROFILE,
                        'align_depth.enable': True,           # aligned_depth_to_color (컬러와 같은 크기)
                        'pointcloud.enable': False,
                        'publish_tf': True, 'tf_publish_rate': 0.0,   # 광학 프레임은 고정 TF (/tf_static)
                    }, sim]))
        else:
            missing('realsense2_camera', 'D435 두 대')

    if as_bool(lc['use_arm_camera']):
        if installed('usb_cam'):
            # 이름공간 arm_camera → /arm_camera/image_raw · /arm_camera/camera_info
            actions.append(Node(
                package='usb_cam', executable='usb_cam_node_exe', name='arm_camera_driver', namespace='arm_camera',
                output='screen',
                parameters=[{
                    'video_device': lc['arm_device'],
                    'framerate': 30.0,
                    'image_width': 640, 'image_height': 480,   # perception 팔 카메라 640x480 (이슈 #14 기준점도 이 크기)
                    'pixel_format': 'mjpeg2rgb',
                    'io_method': 'mmap',
                    'frame_id': 'arm_camera_optical_frame',
                    'camera_name': 'arm_camera',
                    'camera_info_url': lc['arm_camera_info_url'],   # ★ 비우면 보정 없이 (perception 은 태그 id 만)
                }, sim]))
        else:
            missing('usb_cam', '팔 Arducam')

    if as_bool(lc['use_lidar']):
        if installed('coin_d4_driver'):
            # 한 노드로 두 대 (multi_coin_d4_node, lidar_list 의 접두어마다 설정)
            lidar = {'lidar_list': ['lidar_front', 'lidar_rear']}
            for key, port, topic, frame in (('lidar_front', lc['lidar_front_port'], 'scan_front', 'laser_front'),
                                            ('lidar_rear', lc['lidar_rear_port'], 'scan_rear', 'laser_rear')):
                lidar.update({f'{key}.port': port, f'{key}.frame_id': frame, f'{key}.baudrate': 230400,
                              f'{key}.version': 4, f'{key}.topic_name': f'/{topic}',
                              f'{key}.reverse': False,   # 뒤집어 단 것은 URDF laser_* roll π 가 맡음 (둘 다 하면 다시 뒤집힘)
                              f'{key}.warmup_time': 5})
            actions.append(Node(package='coin_d4_driver', executable='multi_coin_d4_node', name='lidar_node',
                                output='screen', parameters=[lidar, sim],
                                sigterm_timeout='15', sigkill_timeout='15'))
        else:
            missing('coin_d4_driver', 'LDS-03 두 대')
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_d435', default_value='true', description='앞 · 뒤 D435 (realsense2_camera)'),
        DeclareLaunchArgument('front_serial', default_value='', description='★ 앞 D435 시리얼 (rs-enumerate-devices -s)'),
        DeclareLaunchArgument('rear_serial', default_value='', description='★ 뒤 D435 시리얼'),
        DeclareLaunchArgument('rgb_auto_exposure', default_value='false',
                              description='D435 RGB 자동 노출 (#89: 어두우면 노출이 길어져 흐림 -> 수동 고정)'),
        DeclareLaunchArgument('rgb_exposure', default_value='80',
                              description='★ D435 RGB 노출 (0.1 ms 단위, 80 = 8 ms ≈ 조명 120 Hz 한 주기). 경기장 조명에서 다시 고름'),
        DeclareLaunchArgument('rgb_gain', default_value='128', description='★ D435 RGB gain'),
        DeclareLaunchArgument('use_arm_camera', default_value='true', description='팔 Arducam (usb_cam)'),
        DeclareLaunchArgument('arm_device', default_value='/dev/arm_cam', description='perception/udev/99-sensors.rules 이름'),
        DeclareLaunchArgument('arm_camera_info_url', default_value='',
                              description='★ Arducam 보정 파일 (file:///... 또는 package://...). 비우면 보정 없음'),
        DeclareLaunchArgument('use_lidar', default_value='true', description='앞 · 뒤 LDS-03 (coin_d4_driver)'),
        DeclareLaunchArgument('lidar_front_port', default_value='/dev/lidar_front'),
        DeclareLaunchArgument('lidar_rear_port', default_value='/dev/lidar_rear'),
        OpaqueFunction(function=launch_setup),
    ])
