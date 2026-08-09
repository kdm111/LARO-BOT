"""arm_agent : agent의 귀로 사용자의 명령을 받는 agent 노드.

입력(구독) : /command (std_msgs/String)
출력 : 로그 출력
노드 이름 : arm_agent
"""

from arm_interfaces.action import MoveTo, Pick, Place
from arm_interfaces.msg import ErrorCode, SceneState

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from std_msgs.msg import String

from . import llm_planner


# 복구 전략
# 전략은 goal 파라미터가 아니라 시퀀스 조작이다.
# REGRASP/RESCAN : 복구 자세로 물러나 재인지한 뒤 실패한 스텝을 다시 밟는다.
RETRY = 'RETRY'
REGRASP = 'REGRASP'
RESCAN = 'RESCAN'
ABORT = 'ABORT'
# 에러 코드 -> 전략. 등록 안된 코드는 기본 abort(멈추는게 가장 안전)
STRATEGY = {
    ErrorCode.PLANNING_FAILED: RETRY,
    ErrorCode.EXECUTION_TIMEOUT: RETRY,
    ErrorCode.GRASP_FAILED: REGRASP,
    ErrorCode.OBJECT_MOVED: RESCAN,
    ErrorCode.OBJECT_NOT_FOUND: RESCAN,
    ErrorCode.GRIPPER_EMPTY: REGRASP,
    ErrorCode.UNREACHABLE: ABORT,
    ErrorCode.INTERNAL_ERROR: ABORT,
}

MAX_ATTEMPTS = 3
MAX_RECOVERY = 2


class Agent(Node):
    """명령을 받아 계획을 세우고 스킬 액션으로 실행하는 노드."""

    def __init__(self):
        """구독 1개(/command)와 액션 클라이언트 3개를 만든다."""
        super().__init__('agent')   # 부모 생성자 호출 및 노드 이름
        self.create_subscription(   # command 를 구독하고 있어 명령을 수신할 수 있음
            String,
            '/command',
            self.on_command,
            10
        )
        self._move_to_client = ActionClient(
            self,
            MoveTo,     # 액션 타입
            'move_to'   # 액션 명칭
        )
        self._pick_client = ActionClient(
            self,
            Pick,
            'pick'
        )
        self._place_client = ActionClient(
            self,
            Place,
            'place'
        )
        # 현재 agent가 보고 있을 씬
        self._scene_ids = None
        self.create_subscription(
            SceneState,
            '/scene_state',
            self.on_scene_state,
            10
        )
        # 쓸 모델을 짧은 이름으로 준다.
        # ros2 param set /agent model llama, exaone으로 바꿀 수 있다.
        self.declare_parameter('model', 'exaone')
        # 복구 자세 정의 : 팔이 시야를 가리지 않도록 비워놓는다.
        # ros2 param set /agent recovery_pose init으로 실행 중 교체 가능
        self.declare_parameter('recovery_pose', 'home')
        self._recovery = 0  # 복구 예산 사용량. 명령 단위로만 리셋

    # 인지 노드가 보내고 있는 스냅샷
    def on_scene_state(self, msg):
        """씬에 있는 물체 id 목록을 갱신한다."""
        self._scene_ids = [obj.object_id for obj in msg.objects]

    # 해당 액션 서버로 보내는 라우터
    # 계획을 세우고 실행 시작하는 함수
    def on_command(self, msg):
        """명령 문자열을 계획으로 바꿔 실행을 시작한다."""
        command = msg.data.strip()
        if not command:
            self.get_logger().warn('빈 명령')
            return

        # 사다리 1~2칸 LLM에게 묻는다. 검증을 통과한 계획만 돌아온다.
        # 파라미터는 콜백 안에서 매번 읽는다.
        model = self.get_parameter('model').value
        steps = llm_planner.plan(command, self._scene_ids, llm_planner.make_ollama_caller(model))
        if steps is not None:
            plan = self._steps_to_goals(steps)
            self.get_logger().info(f'LLM 계획 채택 : {steps}')
        else:
            # 사다리 3칸 문자열 파싱
            plan = self._build_plan(command.split())
            self.get_logger().warn(f'LLM 계획 실패 > 문자열 파서 폴백 : {command}')

        if plan is None:
            self.get_logger().warn(f'잘못된 명령 : {command}')
            return
        self._plan = plan
        self._step = 0
        self._attempt = 1
        self._recovery = 0  # 새 명령 = 복구 초기화
        self._run_step()

    # 계획을 만드는 함수
    def _build_plan(self, parts):
        skill = parts[0]
        if skill == 'move_to' and len(parts) == 2:
            client = self._move_to_client
            goal = MoveTo.Goal()
            goal.target_name = parts[1]
            return [(client, goal)]
        elif skill == 'pick' and len(parts) == 2:
            client = self._pick_client
            goal = Pick.Goal()
            goal.object_id = parts[1]
            return [(client, goal)]
        elif skill == 'place' and len(parts) == 3:
            client = self._place_client
            goal = Place.Goal()
            goal.object_id = parts[1]
            goal.target_id = parts[2]
            return [(client, goal)]
        elif skill == 'deliver' and len(parts) == 3:
            pick_goal = Pick.Goal()
            pick_goal.object_id = parts[1]
            place_goal = Place.Goal()
            place_goal.object_id = parts[1]
            place_goal.target_id = parts[2]
            return [(self._pick_client, pick_goal), (self._place_client, place_goal)]
        return None

    # LLM 계획(dict 리스트)을 실행 고리가 쓰는 (client, goal) 리스트로 바꾼다.
    # Skill 이름은 plan() 검증기가 이미 걸렀으므로 else 분기가 없다.
    def _steps_to_goals(self, steps):
        goals = []
        for step in steps:
            skill = step['skill']
            if skill == 'move_to':
                goal = MoveTo.Goal()
                goal.target_name = step['target_name']
                goals.append((self._move_to_client, goal))
            elif skill == 'pick':
                goal = Pick.Goal()
                goal.object_id = step['object_id']
                goals.append((self._pick_client, goal))
            elif skill == 'place':
                goal = Place.Goal()
                goal.object_id = step['object_id']
                goal.target_id = step['target_id']
                goals.append((self._place_client, goal))
        return goals

    # 스텝 하나 실행
    def _run_step(self):
        if self._step >= len(self._plan):
            self.get_logger().info('시퀀스 완료')
            return
        client, goal = self._plan[self._step]
        goal.attempt = self._attempt
        client.wait_for_server()
        goal_future = client.send_goal_async(goal)
        goal_future.add_done_callback(self.on_goal_response)

    # 수락 되면 결과값 처리
    def on_goal_response(self, goal_future):
        """goal이 수락됐는지 확인하고 결과 콜백을 건다."""
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('goal 거부됨')
            return
        self.get_logger().info('goal 수락됨')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)   # 결과 콜백

    # 결과값 콜백과 시퀀스 다음 스텝 실행
    # 에러 코드에 따른 복구 전략 변경
    def on_result(self, result_future):
        """성공/실패를 판정해 다음 스텝·재시도·중단을 고른다."""
        result = result_future.result().result
        code = result.failure.code
        if result.success:
            self._step += 1
            self._attempt = 1
            self.get_logger().info(
                f'결과 : success={result.success}, code={result.failure.code}'
            )
            self._run_step()
            return
        # LLM에게 전략을 묻고 검증 통과한 라벨만 돌아온다.
        model = self.get_parameter('model').value
        strategy = llm_planner.choose_recovery(
            code, result.failure.stage, result.failure.detail, self._attempt,
            self._recovery, MAX_RECOVERY, llm_planner.make_recovery_caller(model))

        if strategy is None:
            strategy = STRATEGY.get(code, ABORT)
            self.get_logger().warn('LLM 전략 실패 > 표 폴백')
        self.get_logger().warn(
            f'실패 code={code} stage={result.failure.stage} > 전략 {strategy}'
        )
        if strategy == RETRY:
            self._do_retry(code)
        elif strategy in (REGRASP, RESCAN):
            self._do_recover(strategy)
        else:
            self.get_logger().error(f'ABORT(code={code}) > 시퀀스 중단')

    def _do_retry(self, code):
        """같은 goal을 다시 보낸다. OMPL은 확률적이라 시드가 바뀌면 풀린다."""
        if self._attempt >= MAX_ATTEMPTS:
            self.get_logger().error(f'RETRY {MAX_ATTEMPTS}회에서 소진(code={code}) > 중단')
            return
        self._attempt += 1
        self.get_logger().warn(f'RETRY attempt={self._attempt}')
        self._run_step()

    def _do_recover(self, strategy):
        """복구 자세로 물러 났다가 실패한 스텝을 다시 밟는다.

        복구 스텝을 self._step 자리에 끼워넣는다. 성공하면 _step이 1 늘어
        원래 실패한 스텝으로 정확히 돌아온다. skill_server가 실행 시점에 latest_scene_을 읽으므로 같은 goal이여도 갱신된 좌표를 쓴다.

        실패한 스텝이 place면 손이 비어있다. 물러나는 것만으로는 place를 다시 할 수 없으므로 pick을 하나 더 둔다.
        """
        if self._recovery >= MAX_RECOVERY:
            self.get_logger().error(f'{strategy} {MAX_RECOVERY}회 소진 > 중단')
            return
        self._recovery += 1
        self._attempt = 1
        pose = self.get_parameter('recovery_pose').value
        move_goal = MoveTo.Goal()
        move_goal.target_name = pose
        steps = [(self._move_to_client, move_goal)]

        # 실패한 스텝이 place면 들고 있어야 할 물체가 없다. 다시 집는 스텝을 붙인다.
        _, failed_goal = self._plan[self._step]
        if isinstance(failed_goal, Place.Goal):
            pick_goal = Pick.Goal()
            pick_goal.object_id = failed_goal.object_id
            steps.append((self._pick_client, pick_goal))

        # 실패한 스텝 앞에 순서대로 끼운다.
        for offset, step in enumerate(steps):
            self._plan.insert(self._step + offset, step)
        self.get_logger().warn(
            f'{strategy} {self._recovery} / {MAX_RECOVERY} >'
            f'{pose}로 물러나 재인지, 복구 {len(steps)} 스텝 삽입'
        )
        self._run_step()


def main(args=None):
    """노드 진입점."""
    rclpy.init(args=args)
    agent = Agent()
    rclpy.spin(agent)

    rclpy.shutdown()
