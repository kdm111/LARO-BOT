"""실물 카메라(UVC 웹캠). camera_sim.launch.py의 실물 짝이다.

sim 쪽 짝과 하는 일이 같다 - /camera/image_raw와 /camera/camera_info를 낸다.
다른 것은 그림의 출처뿐이다: 가제보 렌더러 대신 진짜 렌즈.

★ 포맷을 640x480 YUYV 30fps로 고정하는 이유 (2026-08-17 실측).
  이 카메라는 MJPG 10종 · YUYV 9종을 내고 640x480은 양쪽 다 30fps다.
  ① sim 카메라와 해상도가 같아 object_detector의 MIN_AREA · min_long 픽셀 상수가
     그대로 의미를 유지한다.
  ② MJPG는 색 경계에 압축 아티팩트가 생겨 HSV 임계값을 흔든다.

★ 토픽을 리맵하는 이유. usb_cam의 기본 토픽 이름은 상대명(image_raw)인데
  object_detector가 절대명을 코드에 박아 뒀다(object_detector.py:66,74).
  그래서 /camera/image_raw · /camera/camera_info로 올려 준다.

★ 컨테이너에서 안 열리면 코드가 아니라 권한이다.
  compose.gt730.yaml의 device_cgroup_rules에 'c 81:* rmw'(video4linux 메이저)가
  있어야 한다. 없으면 /dev 바인드 덕에 ls로는 멀쩡히 보이는데 open()만
  EPERM으로 막힌다. root여도 막힌다 - 팔(ttyACM, 166)과 똑같은 함정이다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_device = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='캡처 장치. /dev/video1은 같은 카메라의 메타데이터 노드라 그림이 없다'
    )

    # ★ 이 파일이 없으면 usb_cam은 k를 전부 0으로 발행하고, detector의
    #   pixel_to_world가 (u - cx) / fx 에서 0으로 나눈다. 구독 콜백 안에서
    #   터지는 예외라 main의 except KeyboardInterrupt가 못 잡고 노드가 죽는다.
    #   (지금은 object_detector가 fx > 0 가드로 먼저 막는다 - 23일차 타이핑분)
    camera_info_url = [
        'file://',
        PathJoinSubstitution([
            FindPackageShare('arm_bringup'), 'config', 'camera_info.yaml',
        ]),
    ]

    usb_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'pixel_format': 'yuyv2rgb',
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'camera_info_url': camera_info_url,
            # 실물에는 /clock이 없다. 켜져 있으면 이미지 스탬프가 0에 멈춘다.
            'use_sim_time': False,
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
    )

    return LaunchDescription([declare_device, usb_cam])
