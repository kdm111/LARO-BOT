"""MoveIt(move_group). 벤더 런치를 감싸기만 한다. sim과 실물이 같이 쓴다.

★★ 이 스택에서 선행 조건을 안 지키면 죽는 노드는 이것 하나뿐이다 (2026-08-19 실측).
벤더 omx_f_moveit.launch.py의 MoveItConfigsBuilder는 srdf · joint_limits ·
kinematics만 넣고 .robot_description()을 부르지 않는다 - moveit_config 패키지에
URDF 파일 자체가 없다. 그래서 URDF는 robot_state_publisher가 토픽으로 쏴줘야 한다.

없으면 이렇게 죽는다:
  Could not find parameter robot_description and did not receive robot_description
  via std_msgs::msg::String subscription within 10.000000 seconds.
  -> Robot model not loaded -> Planning scene not configured -> terminate (exit -6)

즉 sim에서는 가제보 런치가, 실물에서는 real.launch.py의 robot_state_publisher가
먼저 있어야 한다. 부르는 쪽에서 TimerAction으로 늦춰 주는 이유가 이것이다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 벤더 인자 이름이 use_sim_time이 아니라 use_sim이다. 그대로 넘긴다.
    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='sim이면 true(가제보 /clock), 실물이면 false(벽시계)'
    )
    declare_start_rviz = DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        description='MoveIt RViz 창을 띄울지'
    )

    moveit_share = get_package_share_directory('open_manipulator_moveit_config')

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, 'launch', 'omx_f_moveit.launch.py')
        ),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'start_rviz': LaunchConfiguration('start_rviz'),
        }.items(),
    )

    return LaunchDescription([declare_use_sim, declare_start_rviz, moveit])
