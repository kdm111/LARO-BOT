"""실물 팔 브링업. sim.launch.py의 실물 짝이다.

★★ 기본값이 「읽기만」이다. 그냥 띄우면 팔은 한 번도 안 움직인다.

왜 벤더 런치(omx_f.launch.py)를 include 하지 않는가 - 두 가지가 위험해서다.
  ① init_position 기본값이 true다. 컨트롤러가 올라온 직후 joint_trajectory_executor가
     initial_positions.yaml로 궤적을 쏜다. 중력으로 주저앉은 팔을 MoveIt 없이
     전관절 0(수직)까지 쓸어 올린다 - 첫 통전에서 팔이 튀어 오른다.
  ② robot_controller_spawner가 arm_controller · gripper_controller ·
     joint_state_broadcaster 셋을 한 spawner로 묶어 올린다. 셋을 따로 못 켠다.
     읽기만 하고 싶어도 팔 컨트롤러가 같이 활성화된다.
그래서 필요한 노드만 직접 쓴다. init_position 노드는 이 파일에 아예 없다.

stage 인자로 단계를 올린다. 한 단계씩 확인하고 올려라.

  read   ros2_control_node + robot_state_publisher + joint_state_broadcaster
         팔 컨트롤러를 안 켠다. omx_f.ros2_control.xacro의 disable_torque_at_init=true
         때문에 서보 토크도 꺼져 있어 팔은 손으로 움직일 수 있다.
         -> "값이 들어오는가"만 본다. ros2 topic echo /joint_states

  hold   + arm_controller + gripper_controller
         JointTrajectoryController는 활성화 시점의 현재 자세를 잡는다(hold).
         즉 켜는 순간 팔이 지금 있는 자리에서 굳는다 - 목표를 주기 전까지 안 움직인다.
         ★ 켜기 전에 팔을 안전한 자세로 손으로 두어라. 주저앉은 채로 켜면 그 자세로 굳는다.

  moveit 아직 안 붙였다. MoveIt은 관절 한계·속도 스케일·취소 경로를 먼저 손봐야 한다
         (handoff/milestones/M6_audit_2026-08-16.md 참조). skill_server도 마찬가지다.

  ./dc730 exec sim bash -lc 'source /ws/install/setup.bash && \
      ros2 launch arm_bringup real.launch.py'                              # read
  ... ros2 launch arm_bringup real.launch.py stage:=hold                   # hold

토크를 손으로 끄고 켜려면 :
  ros2 service call /dynamixel_hardware_interface/set_dxl_torque \
      std_srvs/srv/SetBool "{data: false}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---------- (1) 인자 ----------
    declare_port = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyACM0',
        description='OpenRB-150이 잡힌 시리얼 포트. 컨테이너 안에서 보여야 한다 '
                    '(compose.gt730.yaml의 device_cgroup_rules + /dev 바인드)'
    )
    declare_stage = DeclareLaunchArgument(
        'stage',
        default_value='read',
        description='read = 읽기만(팔 컨트롤러 없음, 토크 꺼짐) / hold = 팔 컨트롤러까지'
    )
    declare_rviz = DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        description='RViz로 관절 상태를 눈으로 볼지'
    )

    port_name = LaunchConfiguration('port_name')
    stage = LaunchConfiguration('stage')

    # stage가 hold 이상일 때만 팔 컨트롤러를 켠다.
    hold_stage = IfCondition(PythonExpression(["'", stage, "' == 'hold'"]))

    # ---------- (2) URDF ----------
    # 벤더 xacro를 그대로 부른다. use_sim:=false, use_mock_hardware:=false 가 실물 경로다.
    # ★ 이 셋 중 하나라도 true면 서보에 값이 안 나간다(가짜 하드웨어가 대신 답한다).
    # ★ ParameterValue(value_type=str)로 감싸야 한다. 안 감싸면 런치가 xacro 출력을
    #   YAML로 파싱하려다 죽는다("Unable to parse the value of parameter
    #   robot_description as yaml"). URDF는 XML이라 YAML로 읽힐 수가 없다.
    urdf = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_description'),
            'urdf', 'omx_f', 'omx_f.urdf.xacro',
        ]), ' ',
        'prefix:=""', ' ',
        'use_sim:=false', ' ',
        'use_mock_hardware:=false', ' ',
        'mock_sensor_commands:=false', ' ',
        'port_name:=', port_name, ' ',
        'ros2_control_type:=omx_f',
    ]), value_type=str)

    controller_config = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'),
        'config', 'omx_f', 'hardware_controller_manager.yaml',
    ])

    # ---------- (3) 제어 ----------
    # ★ use_sim_time을 명시적으로 False로 준다. 실물에는 /clock이 없다.
    #   켜져 있으면 노드가 오지 않는 시계를 기다리며 시간이 0에 멈춘다 -
    #   skill_server의 wait_gripper_settled 같은 데드라인 루프가 영영 안 끝난다.
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': urdf, 'use_sim_time': False}, controller_config],
        output='both',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf, 'use_sim_time': False}],
        output='both',
    )

    # ---------- (4) 컨트롤러 ----------
    # 브로드캐스터는 읽기 전용이다. 서보에 아무 명령도 안 보낸다.
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='both',
    )

    # 여기서부터가 팔에 값을 보내는 쪽이다. stage:=hold 에서만 올라간다.
    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='both',
        condition=hold_stage,
    )
    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='both',
        condition=hold_stage,
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('open_manipulator_description'),
            'rviz', 'open_manipulator.rviz',
        ])],
        parameters=[{'use_sim_time': False}],
        output='both',
        condition=IfCondition(LaunchConfiguration('start_rviz')),
    )

    return LaunchDescription([
        declare_port,
        declare_stage,
        declare_rviz,
        control_node,
        robot_state_publisher,
        jsb_spawner,
        arm_spawner,
        gripper_spawner,
        rviz,
    ])
