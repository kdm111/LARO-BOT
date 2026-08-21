"""구역 사각형의 네 꼭지점을 팔로 짚어 본다. 눈으로 경계를 확인하는 지그다.

    ros2 launch arm_bringup trace_zone.launch.py                  # work 구역, 꼭지점당 2초
    ros2 launch arm_bringup trace_zone.launch.py zone:=counter
    ros2 launch arm_bringup trace_zone.launch.py dwell_sec:=3.0 loops:=2

★ 좌표를 여기 적지 않는다. cell_layout.yaml을 읽어서 파라미터로 넘긴다.
  그 파일 10행이 "M6에서는 노드가 파라미터로 읽게 된다"고 예고한 경로가 이것이다.
  구역을 옮기면 yaml 한 곳만 고치면 되고, 이 launch도 trace_zone.cpp도 안 건드린다.

⚠️ MoveIt 플래닝 씬에 테이블 위 물체가 없다. 계획이 물체를 피하지 못하므로
   책상을 비우고 돌릴 것. 특히 work 구역은 가까운 줄과 먼 줄 사이에 물체가
   놓여 있으면 팔이 그 위를 지난다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml

LAYOUT = os.path.join(
    get_package_share_directory('arm_bringup'), 'config', 'cell_layout.yaml')


def _launch_trace(context, *args, **kwargs):
    """yaml에서 구역 사각형을 꺼내 노드 파라미터로 넘긴다."""
    zone = LaunchConfiguration('zone').perform(context)

    with open(LAYOUT) as f:
        zones = yaml.safe_load(f)['zones']
    if zone not in zones:
        # 조용히 기본값으로 도는 것을 막는다(reach_once 11행과 같은 이유).
        raise RuntimeError(
            f'cell_layout.yaml 에 구역 "{zone}" 이 없다. 있는 것: {sorted(zones)}')
    rect = zones[zone]

    return [Node(
        package='arm_skills',
        executable='trace_zone',
        output='screen',
        parameters=[{
            'zone': zone,
            'x0': float(rect['x'][0]),
            'x1': float(rect['x'][1]),
            'y0': float(rect['y'][0]),
            'y1': float(rect['y'][1]),
            'z': float(LaunchConfiguration('z').perform(context)),
            'dwell_sec': float(LaunchConfiguration('dwell_sec').perform(context)),
            'loops': int(LaunchConfiguration('loops').perform(context)),
            'home_first': LaunchConfiguration('home_first').perform(context) == 'true',
            'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true',
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'zone', default_value='work',
            description='cell_layout.yaml 의 구역 이름'),
        # ★ 0.06 (2026-08-20 저녁 최종. 그 전에 0.02 -> 0.12 를 거쳤다).
        #   지금 구역 넷은 전부 이 높이에서 네 꼭지점이 풀린다. 높이 상한은 구역 크기와
        #   맞물리므로(구역이 클수록 낮아진다) 구역을 넓히면 여기도 다시 볼 것 -
        #   근거는 cell_layout.yaml 의 work 주석.
        #   ★ trace_zone.cpp 의 기본값과 같은 값을 여기에도 적어야 한다 - launch 는 항상
        #     파라미터를 넘기므로 cpp 기본값은 ros2 run 으로 직접 부를 때만 쓰인다.
        #     둘이 어긋나면 cpp 만 고치고 launch 로 돌렸을 때 조용히 옛 값으로 간다
        #     (실제로 한 번 그랬다 - cpp 를 0.06 으로 고쳤는데 0.02 로 돌았다).
        DeclareLaunchArgument(
            'z', default_value='0.06',
            description='꼭지점을 짚는 높이(m). 테이블면이 0'),
        DeclareLaunchArgument(
            'dwell_sec', default_value='2.0',
            description='꼭지점마다 멈춰 있는 시간(초)'),
        DeclareLaunchArgument(
            'loops', default_value='1',
            description='사각형을 몇 바퀴 돌 것인가'),
        DeclareLaunchArgument(
            'home_first', default_value='true',
            description='시작과 끝에 home 자세를 거칠 것인가'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='sim이면 true(가제보 /clock), 실물이면 false(벽시계)'),
        OpaqueFunction(function=_launch_trace),
    ])
