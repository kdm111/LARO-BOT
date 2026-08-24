"""실물 셀 전체. sim.launch.py의 실물 짝이다.

sim.launch.py와 이 파일은 조립 설명서일 뿐이고, 부품은 대부분 같은 파일을 쓴다.
갈리는 것은 그림과 관절값의 출처 하나다:

    sim   가제보(omx_f_gazebo.launch.py) + camera_sim.launch.py
    실물  real_arm.launch.py            + real_camera.launch.py
    공용  camera_tf · perception · moveit · skills · agent   <- 양쪽이 같은 파일

★★ 기본값은 stage:=read - 팔 컨트롤러를 안 켠다. 즉 노드를 다 띄워도 팔은 안 움직인다.
  안전은 stage가 담당하고, 아래 스위치들은 "무엇을 볼지"만 정한다.
  MoveIt과 스킬 서버가 떠 있어도 read 단계에서는 실행이 컨트롤러가 없어 실패한다 - 의도한 것이다.

부품만 따로 보고 싶으면 각각 띄울 수 있다 (2026-08-19 전부 단독 기동 확인) :
    ros2 launch arm_bringup real_arm.launch.py       팔만    /joint_states 100Hz
    ros2 launch arm_bringup real_camera.launch.py    카메라만 /camera/image_raw 30Hz
    ros2 launch arm_bringup camera_tf.launch.py      TF만
    ros2 launch arm_bringup perception.launch.py     인지만  (카메라 + TF 필요)

★★ 순서를 타는 노드는 move_group 하나뿐이다 (2026-08-19 실측).
  벤더 moveit 설정에 URDF가 없어서 /robot_description을 토픽으로 기다리는데,
  10초 안에 안 오면 죽는다(exit -6). 그 토픽은 팔 쪽 robot_state_publisher가 낸다.
  그래서 moveit만 TimerAction으로 늦춘다. 나머지는 순서를 안 탄다 -
  skill_server는 액션 3개를 먼저 열고 기다리고, detector는 TF가 없으면 경고만 한다.

  ./dc730 exec sim bash -lc 'source /ws/install/setup.bash && \
      ros2 launch arm_bringup real.launch.py'                     # 전체, 읽기만
  ... ros2 launch arm_bringup real.launch.py stage:=hold          # 팔 컨트롤러까지
  ... ros2 launch arm_bringup real.launch.py agent:=false         # 스킬을 직접 때려볼 때

★ 끄기 전에 real_arm.launch.py 머리의 경고를 읽어라 - 끄면 팔이 무너진다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(share, name, condition=None, **args):
    """arm_bringup 안의 런치 하나를 끌어온다. 인자는 전부 문자열로 넘어간다."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', name)),
        launch_arguments=args.items() if args else None,
        condition=condition,
    )


def generate_launch_description():
    share = get_package_share_directory('arm_bringup')

    # ---------- (1) 인자 ----------
    # 팔 쪽 인자는 real_arm.launch.py의 것을 그대로 물려준다.
    declare_port = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyACM0',
        description='OpenRB-150이 잡힌 시리얼 포트'
    )
    declare_stage = DeclareLaunchArgument(
        'stage',
        default_value='read',
        description='read = 읽기만(팔 컨트롤러 없음) / hold = 팔 컨트롤러까지. '
                    '★ 실물 안전은 이 인자가 담당한다'
    )
    declare_device = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='웹캠 캡처 장치'
    )

    # ★ 기본은 셀 전체가 뜨는 것이다. sim.launch.py가 같은 선택을 한 이유가 남아 있다 -
    #   터미널을 손으로 여러 개 열던 시절에는 하나만 빠뜨려도 증상이 엉뚱하게 나타났다
    #   (검출이 없으면 "명령을 받고도 가만히 있다", agent가 없으면 "/robot_status가 없다").
    #   실물이라고 이 기본값을 뒤집지 않는다 - 팔을 막는 것은 stage이지 이 스위치가 아니다.
    switches = {
        'camera': '카메라(usb_cam)를 띄울지',
        'detector': '인지 노드를 띄울지 (카메라 + TF가 선행)',
        'moveit': 'move_group을 띄울지 (팔이 선행 - 없으면 10초 뒤 죽는다)',
        'skills': '스킬 서버를 띄울지',
        'agent': '판단 노드를 띄울지 (false면 스킬을 직접 때려볼 수 있다)',
    }
    declare_switches = [
        DeclareLaunchArgument(name, default_value='true', description=desc)
        for name, desc in switches.items()
    ]

    declare_start_rviz = DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        description='팔 URDF만 보는 RViz (관절 상태 확인용)'
    )
    declare_moveit_rviz = DeclareLaunchArgument(
        'moveit_rviz',
        default_value='false',
        description='MoveIt RViz (플래닝 씬까지 본다). start_rviz와 별개다'
    )
    declare_pick_center_y_trim = DeclareLaunchArgument(
        'pick_center_y_trim',
        default_value='0.045',
        description='work 구역 블록 pick을 화면 왼쪽(+y)으로 옮기는 실물 트림(m)'
    )
    declare_pick_counter_x_trim = DeclareLaunchArgument(
        'pick_counter_x_trim', default_value='0.015',
        description='counter pick을 로봇에서 먼 방향(+x)으로 옮기는 실물 트림(m)')
    declare_pick_shelf_x_trim = DeclareLaunchArgument(
        'pick_shelf_x_trim', default_value='0.0',
        description='shelf pick의 실물 +x 트림(m)')
    declare_pick_bin_x_trim = DeclareLaunchArgument(
        'pick_bin_x_trim', default_value='0.0',
        description='bin pick의 실물 +x 트림(m)')
    declare_pick_counter_y_trim = DeclareLaunchArgument(
        'pick_counter_y_trim', default_value='0.020',
        description='counter pick을 shelf 방향(+y)으로 옮기는 실물 트림(m)')
    declare_pick_shelf_y_trim = DeclareLaunchArgument(
        'pick_shelf_y_trim', default_value='0.0',
        description='shelf pick의 실물 +y 트림(m)')
    declare_pick_bin_y_trim = DeclareLaunchArgument(
        'pick_bin_y_trim', default_value='0.0',
        description='bin pick의 실물 +y 트림(m)')

    def on(name):
        return IfCondition(LaunchConfiguration(name))

    # ---------- (2) 실물 전용 - 팔과 카메라 ----------
    arm = _include(
        share, 'real_arm.launch.py',
        port_name=LaunchConfiguration('port_name'),
        stage=LaunchConfiguration('stage'),
        start_rviz=LaunchConfiguration('start_rviz'),
    )
    camera = _include(
        share, 'real_camera.launch.py', condition=on('camera'),
        video_device=LaunchConfiguration('video_device'),
    )

    # ---------- (3) 공용 - sim.launch.py도 같은 파일을 쓴다 ----------
    # ★ 실물은 전부 use_sim_time=false다. /clock이 없어서, 켜면 노드가 오지 않는
    #   시계를 기다리며 시간이 0에 멈춘다 - skill_server의 wait_gripper_settled 같은
    #   데드라인 루프가 영영 안 끝난다.
    camera_tf = _include(share, 'camera_tf.launch.py', use_sim_time='false')
    perception = _include(
        share, 'perception.launch.py', condition=on('detector'), use_sim_time='false')
    skills = _include(
        share, 'skills.launch.py', condition=on('skills'), use_sim_time='false',
        pick_center_y_trim=LaunchConfiguration('pick_center_y_trim'),
        pick_counter_x_trim=LaunchConfiguration('pick_counter_x_trim'),
        pick_shelf_x_trim=LaunchConfiguration('pick_shelf_x_trim'),
        pick_bin_x_trim=LaunchConfiguration('pick_bin_x_trim'),
        pick_counter_y_trim=LaunchConfiguration('pick_counter_y_trim'),
        pick_shelf_y_trim=LaunchConfiguration('pick_shelf_y_trim'),
        pick_bin_y_trim=LaunchConfiguration('pick_bin_y_trim'))
    agent = _include(share, 'agent.launch.py', condition=on('agent'))

    # ★ moveit만 늦춘다. robot_state_publisher가 /robot_description을 낼 때까지
    #   기다리는 10초 시한이 이 노드에만 있다. 이 머신에서 팔 브링업이 xacro 두 번을
    #   펼치느라 몇 초를 쓰므로, 여유를 두고 8초 뒤에 띄운다.
    moveit = TimerAction(
        period=8.0,
        actions=[_include(
            share, 'moveit.launch.py', condition=on('moveit'),
            use_sim='false', start_rviz=LaunchConfiguration('moveit_rviz'),
        )],
    )

    return LaunchDescription([
        declare_port,
        declare_stage,
        declare_device,
        *declare_switches,
        declare_start_rviz,
        declare_moveit_rviz,
        declare_pick_center_y_trim,
        declare_pick_counter_x_trim,
        declare_pick_shelf_x_trim,
        declare_pick_bin_x_trim,
        declare_pick_counter_y_trim,
        declare_pick_shelf_y_trim,
        declare_pick_bin_y_trim,
        arm,
        camera,
        camera_tf,
        perception,
        skills,
        agent,
        moveit,
    ])
