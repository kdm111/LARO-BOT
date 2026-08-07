import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # ---------- (1) 런치 인자 ----------
    # scene 하나만 바꾸면 환경(블록 배치)이 통째로 갈린다.
    declare_scene = DeclareLaunchArgument(
        'scene',
        default_value='one_block',
        description='worlds/scene_<scene>.sdf 를 world로 로드 '
                    '(one_block/two_blocks/two_colors/unreachable/grasp_test)'
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='MoveIt RViz 창을 띄울지 여부'
    )

    # ---------- (2) 경로 계산 ----------
    perception_share = get_package_share_directory('arm_perception')
    bringup_share = get_package_share_directory('open_manipulator_bringup')
    moveit_share = get_package_share_directory('open_manipulator_moveit_config')

    # world는 '절대경로 + 확장자 없이'.
    # 벤더 omx_f_gazebo.launch.py가 gz_args에서 world + '.sdf'를 직접 붙인다.
    # 절대경로라 GZ_SIM_RESOURCE_PATH에 의존하지 않는다
    # (벤더가 그 env를 SetEnvironmentVariable로 덮어써서 export 방식은 안 통함).
    world_path = PathJoinSubstitution([
        perception_share,
        'worlds',
        ['scene_', LaunchConfiguration('scene')],  # 중첩 리스트 = 경로 조각 하나
    ])

    # ---------- (3) 가제보 (world + 팔 + 컨트롤러 3종 + /clock 브리지) ----------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'omx_f_gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items(),
    )

    # ---------- (4) MoveIt (move_group, 선택적 RViz) ----------
    # use_sim 기본값이 false라 반드시 true로 넘겨야 /clock을 쓴다.
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, 'launch', 'omx_f_moveit.launch.py')
        ),
        launch_arguments={
            'use_sim': 'true',
            'start_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )

    # ---------- (5) 스킬 서버 ----------
    # MoveGroupInterface가 /robot_description(_semantic)을 블로킹 대기하므로
    # move_group보다 먼저 떠도 알아서 기다린다 -> 순서 제어 불필요.
    skill_server = Node(
        package='arm_skills',
        executable='skill_server',
        output='screen',
        # ★ 가제보가 /clock을 발행하고 /joint_states도 sim 시간으로 찍는다.
        # 이게 없으면 MGI의 CurrentStateMonitor가 벽시계(1786090380)와
        # sim 시간(41.18)을 비교해 상태를 "낡았다"고 버린다 ->
        # "Failed to fetch current robot state" -> getCurrentJointValues()가 빈 벡터 ->
        # joint_distance가 0을 돌려 최근접 가지 선택이 통째로 무력화된다(2026-08-07 실측).
        parameters=[{'use_sim_time': True}],
    )

    # ---------- (6) 카메라 (씬 world에는 없다 -> launch가 얹는다) ----------
    # 블록은 world에 baking돼 있어 여기서 spawn하지 않는다.
    camera_xacro = os.path.join(perception_share, 'urdf', 'camera.urdf.xacro')
    camera_description = xacro.process_file(camera_xacro).toprettyxml(indent=' ')

    # create는 가제보가 뜰 때까지 자동 재시도한다.
    spawn_camera = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', camera_description,
            '-name', 'fixed_camera',
            '-allow_renaming', 'true',
        ],
    )

    # gz 이미지 토픽 -> ROS 토픽 브리지 (인지 노드가 구독할 창구)
    # 벤더 launch의 /clock 브리지도 기본 이름이 ros_gz_bridge라 이름이 겹친다.
    # name을 줘서 그래프에 /camera_bridge로 뜨게 한다 (동명 노드 경고 제거).
    # camera_info = 내부 파라미터(fx, fy, cx, cy). 픽셀->3D 역투영에 필요하다.
    # 코드에 숫자를 박지 않고 이 토픽에서 받는다 (실물 캘리브레이션과 같은 경로).
    image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
    )

    # ---------- (7) 카메라 TF ----------
    # 카메라는 팔과 별도 모델로 spawn돼서 robot_state_publisher가 다루지 않는다
    # -> TF 트리에 camera_link가 없다. 여기서 직접 심어준다.
    # 값은 camera.urdf.xacro의 camera_joint origin과 반드시 동일해야 한다
    # (어긋나면 검출 좌표가 통째로 틀어진다). 실물(M6)에선 이 값이 캘리브레이션 결과.
    # 부모는 world: 팔 URDF의 루트이자 MoveIt 플래닝 프레임(world -> link0는 항등).
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_tf',
        output='screen',
        arguments=[
            '--x', '-0.10', '--y', '0.0', '--z', '0.50',
            '--roll', '0.0', '--pitch', '0.9', '--yaw', '0.0',
            '--frame-id', 'world',
            '--child-frame-id', 'camera_link',
        ],
    )

    # 광축 규약 변환. URDF는 x축이 정면이지만,
    # 핀홀 역투영 공식과 ROS 비전 관례(REP-103)는 z=광축, x=우, y=아래다.
    # rpy = (-90, 0, -90)도가 그 둘을 잇는 고정 회전 (이 값은 관례상 상수).
    # 인지 노드는 이 optical 프레임에서 3D를 풀고 TF로 world까지 올린다.
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
    )

    return LaunchDescription([
        declare_scene,
        declare_use_rviz,
        gazebo,
        moveit,
        skill_server,
        spawn_camera,
        image_bridge,
        camera_tf,
        camera_optical_tf,
    ])
