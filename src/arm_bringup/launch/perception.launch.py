"""인지 노드(detector) 하나. sim과 실물이 같이 쓴다.

detector가 하는 일은 양쪽이 완전히 같다 - HSV로 픽셀을 찾고, camera_info의 K로
그 픽셀을 광선으로 되돌리고, TF로 world까지 올린다. 그래서 코드도 런치도 공용이다.
갈리는 것은 값뿐이다: HSV 임계값(조명) · PLANE_Z(실치수) · K(캘리브) · TF(거치).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ★ sim에서는 true여야 한다. /scene_state의 헤더 스탬프가 곧 방치 판정의
    #   기준이라, 벽시계를 찍으면 agent의 경과 시간 계산이 통째로 어긋난다.
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='sim이면 true(가제보 /clock), 실물이면 false(벽시계)'
    )

    detector = Node(
        package='arm_perception',
        executable='object_detector.py',
        name='object_detector',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    )

    return LaunchDescription([declare_use_sim_time, detector])
