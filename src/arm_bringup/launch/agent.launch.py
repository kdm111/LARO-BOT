"""판단 노드(agent) 하나. sim과 실물이 같이 쓴다.

★ 여기만 sim에서도 sim 시계를 안 쓴다. 검증 마감(VERIFY_SEC)은 "검출이 멈춰도
끊긴다"가 목적이라 벽시계여야 한다. 방치 판정은 씬 헤더 스탬프끼리만 빼므로
영향이 없다 (agent.py 안에 시계가 둘인 이유 - _verify_tick은 벽시계, _loitering은 sim).
그래서 이 파일에는 use_sim_time 인자가 없다. 없는 게 맞다.

LLM은 sim이든 실물이든 pod를 HTTP로 본다(2026-08-12부터 동일).
접속 주소는 OLLAMA_HOST 환경변수 - 컨테이너가 compose에서 물려받는다.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    agent = Node(
        package='arm_agent',
        executable='agent',
        name='agent',
        output='screen',
    )

    return LaunchDescription([agent])
