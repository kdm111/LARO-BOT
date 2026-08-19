"""카메라 TF 두 개. sim과 실물이 같이 쓴다.

★ 이 파일이 extrinsic 보정값이 들어가는 자리다.
sim에서는 launch에 타이핑한 값이 곧 진실이었다 - 우리가 카메라를 거기 뒀으니까.
실물에서는 반대다. 삼각대에 놓인 카메라의 실제 위치를 재서 여기에 채워 넣는다.
그 재는 일이 M6 단계 3ⓑ이고, 합격선은 "정답 대조 ≤5mm"다.

TF가 둘인 이유:
  world -> camera_link          카메라가 어디에 있나 (우리가 재는 값)
  camera_link -> camera_optical  광축 규약 변환 (관례상 상수, 재지 않는다)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ---------- 인자 ----------
    # ★ use_sim_time을 인자로 뺀 이유. sim에서 이 노드만 벽시계 스탬프를 찍으면
    #   move_group의 tf2 버퍼가 'Detected jump back in time'으로 통째로 비워진다
    #   (2026-08-07 실측). 실물에는 /clock이 없으니 반대로 반드시 false여야 한다.
    #   같은 파일을 양쪽에서 쓰려면 이 값이 밖에서 들어와야 한다.
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='sim이면 true(가제보 /clock), 실물이면 false(벽시계)'
    )

    # 카메라 위치 6개. 실물 보정 결과를 이 기본값에 덮어쓴다.
    #
    # ★ 2026-08-19 삼각대 거치 후 1차 실측값 (사용자 자로 측정).
    #   cam_x -0.12  팔 베이스 원점에서 뒤로 12cm
    #   cam_z  0.33  ★ 테이블 상면 기준이다. 바닥 기준이 아니다 -
    #                world의 z=0이 팔이 놓인 테이블 면이라서다. 처음에 바닥에서 잰
    #                1m를 넣을 뻔했는데, 검출된 blob 크기가 그것을 부정했다:
    #                1m면 6cm 블록이 516px²로 보여야 하는데 실측이 3520px²였다.
    #                33cm로 계산하면 3184px² - 측면이 보이는 만큼의 차이라 맞는다.
    #   cam_pitch 0.667 rad = 38.2도  ★ 2026-08-19 확정. 눈대중 "60도"는 틀렸다.
    #                각도는 자로 못 재는 값이라 눈대중이 22도나 빗나갔다.
    #                푼 방법 = 크기 조건 역산: 블록 긴변이 화면에서 68px인데 실치수가
    #                6.0cm 이려면 카메라-블록 거리가 0.428m 여야 하고, 자로 잰 높이
    #                0.33 을 그대로 두면 광선이 수평에서 46.4도 아래여야 한다.
    #                그 픽셀이 광축보다 8.2도 아래이므로 pitch = 38.2도.
    #                ★ 검증: 이 값으로 실물 첫 파지가 성공했다(2026-08-19).
    #                detector 가 내는 긴변이 6.3cm(실치수 6.0)로 맞는지가 상시 점검 지표다.
    defaults = {
        'cam_x': '-0.12',
        'cam_y': '0.0',
        'cam_z': '0.33',
        'cam_roll': '0.0',
        'cam_pitch': '0.667',
        'cam_yaw': '0.0',
    }
    declares = [
        DeclareLaunchArgument(
            name, default_value=value,
            description='world -> camera_link (M6 단계 3ⓑ 실측값으로 교체)'
        )
        for name, value in defaults.items()
    ]

    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    # ---------- (1) 카메라가 어디에 있나 ----------
    # 부모는 world: 팔 URDF의 루트이자 MoveIt 플래닝 프레임(world -> link0는 항등).
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_tf',
        output='screen',
        arguments=[
            '--x', LaunchConfiguration('cam_x'),
            '--y', LaunchConfiguration('cam_y'),
            '--z', LaunchConfiguration('cam_z'),
            '--roll', LaunchConfiguration('cam_roll'),
            '--pitch', LaunchConfiguration('cam_pitch'),
            '--yaw', LaunchConfiguration('cam_yaw'),
            '--frame-id', 'world',
            '--child-frame-id', 'camera_link',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ---------- (2) 광축 규약 변환 ----------
    # URDF는 x축이 정면이지만, 핀홀 역투영 공식과 ROS 비전 관례(REP-103)는
    # z=광축, x=우, y=아래다. rpy = (-90, 0, -90)도가 그 둘을 잇는 고정 회전.
    # ★ 이 값은 재는 값이 아니다. 관례상 상수라 실물에서도 그대로다.
    camera_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_tf',
        output='screen',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '-1.5707963', '--pitch', '0.0', '--yaw', '-1.5707963',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_optical_frame',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        *declares,
        camera_tf,
        camera_optical_tf,
    ])
