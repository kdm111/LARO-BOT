"""스킬 서버(액션 3종) 하나. sim과 실물이 같이 쓴다.

MoveGroupInterface가 /robot_description(_semantic)을 블로킹 대기하므로
move_group보다 먼저 떠도 알아서 기다린다 -> 순서 제어가 필요 없다.
2026-08-19 실측으로 확인: 팔도 move_group도 없이 단독 기동해도 죽지 않고,
/pick · /place · /move_to 세 액션을 먼저 연 뒤 로그로만 경고한다.
(같은 상황에서 move_group은 10초 뒤 죽는다 - 그쪽만 순서를 탄다.)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ★ sim에서 이게 없으면 MGI의 CurrentStateMonitor가 벽시계와 sim 시간을 비교해
    #   상태를 "낡았다"고 버린다 -> "Failed to fetch current robot state" ->
    #   getCurrentJointValues()가 빈 벡터 -> joint_distance가 0을 돌려
    #   최근접 가지 선택이 통째로 무력화된다(2026-08-07 실측).
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='sim이면 true(가제보 /clock), 실물이면 false(벽시계)'
    )

    skill_server = Node(
        package='arm_skills',
        executable='skill_server',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    )

    return LaunchDescription([declare_use_sim_time, skill_server])
