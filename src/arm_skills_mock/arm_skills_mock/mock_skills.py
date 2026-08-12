"""
mock_skills : 가짜 스킬 서버.

진짜 팔도 MoveIt도 없이 Pick/Place/MoveTo 액션 서버를 흉내낸다.
파라미터로 실패를 주입해서 에이전트의 복구 루프를 미리 검증한다.

"""

import time

from arm_interfaces.action import MoveTo, Pick, Place
from arm_interfaces.msg import ErrorCode, FailureReport, Stage
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

STAGES = {
    'pick': [Stage.PLAN, Stage.APPROACH, Stage.GRASP, Stage.LIFT],
    'place': [Stage.PLAN, Stage.TRANSFER, Stage.RELEASE, Stage.RETREAT],
    'move_to': [Stage.PLAN, Stage.EXECUTE],
}

DEFAULTS = {
    'pick': {
        'duration_sec': 1.0,
        'fail_code': ErrorCode.GRASP_FAILED,
        'fail_stage': Stage.GRASP,
    },
    'place': {
        'duration_sec': 1.0,
        'fail_code': ErrorCode.PLANNING_FAILED,
        'fail_stage': Stage.PLAN,
    },
    'move_to': {
        'duration_sec': 0.5,
        'fail_code': ErrorCode.PLANNING_FAILED,
        'fail_stage': Stage.PLAN,
    },
}


class MockSkills(Node):
    def __init__(self):
        super().__init__('mock_skills')
        # 파라미터 선언. ROS2는 선언 안한 파라미터를 못 읽고 못 쓴다.
        # 기본적으로 파라미터의 타입은 선언 시점에 고정되고 바뀌지 않는다.
        # 명시된 기본값에서 추론되거나 디스크립터로 명시된 값.
        # ROS2 param 리스트에 기대한게 안 보이면 내가 지금 보고 있는 노드가 과거 노드인지 확인해야 한다.
        # symlink-install 덕에 빌드는 필요없지만 파이썬 파일은 읽힐때 재시작 하므로 다시 읽어야 한다.

        for name, d in DEFAULTS.items():
            self.declare_parameter(f'{name}.duration_sec', d['duration_sec'])
            self.declare_parameter(f'{name}.fail_until_attempt', 0)
            self.declare_parameter(f'{name}.fail_code', d['fail_code'])
            self.declare_parameter(f'{name}.fail_stage', d['fail_stage'])
        self.get_logger().info(f'mock_skills 액션 서버 시작 {len(DEFAULTS)}개')
        self._move_to_server = ActionServer(  # 클래스의 내부로 숨겨야, 파이썬 GC가 수거하지 않는다.
            self,  # 액션 서버가 붙을 노드
            MoveTo,  # 액션 타입
            'move_to',  # 액션 이름(상대 이름)
            self.execute_move_to,  # 목표를 받으면 실행할 콜백
        )
        self._pick_server = ActionServer(
            self,  # 액션 서버가 붙을 노드
            Pick,  # 액션 타입
            'pick',  # 액션 이름(상대 이름)
            self.execute_pick,  # 목표를 받으면 실행할 콜백
        )
        self._place_server = ActionServer(
            self,  # 액션 서버가 붙을 노드
            Place,  # 액션 타입
            'place',  # 액션 이름
            self.execute_place,  # 받으면 실행할 콜백
        )

    # goal_handle, 액션 이름, 액션 타입, 제어, 목표
    # obejct_id는 함수의 의미, target은 로그용
    def _execute(self, goal_handle, name, action_type, object_id, target):
        """action을 받으면 실행되는 callback 함수."""
        goal = goal_handle.request  # action goal은 request안에 있다.
        self.get_logger().info(
            f'{name} 시작 : target={target} attempt={goal.attempt}'
        )
        duration = self.get_parameter(f'{name}.duration_sec').value
        fail_code = self.get_parameter(f'{name}.fail_code').value
        fail_stage = self.get_parameter(f'{name}.fail_stage').value
        fail_until = self.get_parameter(f'{name}.fail_until_attempt').value

        will_fail = goal.attempt <= fail_until
        for i, stage in enumerate(STAGES[name]):
            feedback = action_type.Feedback()
            feedback.stage = stage
            feedback.progress = i / len(STAGES[name])
            goal_handle.publish_feedback(feedback)

            self.get_logger().info(f'[{stage}]')
            time.sleep(duration)  # 행동마다 슬립 # param set 하면 그 값으로 바뀐 뒤 바로 먹혀야 한다.
            # 슬립 다음에 실패하는 이유
            # 앞에두면 시도 조차 하지 않기 때문.
            if will_fail and stage == fail_stage:  # 실패 사례
                self.get_logger().warn(f'[{stage}] 실패 주입 (code={fail_code})')
                goal_handle.abort()
                result = action_type.Result()
                result.success = False
                result.failure = self.make_failure(
                    code=fail_code,
                    stage=stage,
                    object_id=object_id,
                    detail=f'주입된 실패: {name}({target}) attempt={goal.attempt}',
                    attempt=goal.attempt,  # 서버는 세지 않고 에이전트가 셀 예정
                )
                return result
        goal_handle.succeed()
        result = action_type.Result()
        result.success = True
        result.failure = self.make_failure(ErrorCode.SUCCESS, '', '', '', goal.attempt)
        return result

    def execute_move_to(self, goal_handle):
        return self._execute(goal_handle, 'move_to', MoveTo, '', goal_handle.request.pose_id)

    def execute_pick(self, goal_handle):
        goal = goal_handle.request
        return self._execute(goal_handle, 'pick', Pick, goal.object_id, goal.object_id)

    def execute_place(self, goal_handle):
        goal = goal_handle.request
        return self._execute(goal_handle, 'place', Place, goal.object_id, goal.target_id)

    def make_failure(self, code, stage, object_id, detail, attempt):
        """
        FailureReport를 만드는 유일한 통로.

        어디서도 FailureReport를 직접 생성하지 않고 code에 ErrorCode 상수가 아닌 값이 들어갈 경로를 여기 하나로 좁힌다.
        """
        report = FailureReport()
        report.code = code
        report.stage = stage
        report.object_id = object_id
        report.detail = detail
        report.stamp = self.get_clock().now().to_msg()
        report.attempt = attempt
        return report


def main(args=None):
    rclpy.init(args=args)
    mock_skills = MockSkills()
    rclpy.spin(mock_skills)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
