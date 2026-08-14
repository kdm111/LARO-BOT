"""arm_agent : agent의 귀로 사용자의 명령을 받는 agent 노드.

입력(구독) : /command (std_msgs/String)
출력 : 로그 출력
노드 이름 : arm_agent
"""

import math

from arm_interfaces.action import MoveTo, Pick, Place
from arm_interfaces.msg import ErrorCode, RobotStatus, SceneState

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import String

from . import llm_planner

# 로봇 상태
IDLE = 'IDLE'  # 쉬는 중
RUNNING = 'RUNNING'  # 정상 실행 중
RECOVERING = 'RECOVERING'  # 실패 후 수습 중
ABORTED_WAIT = 'ABORTED_WAIT'  # 사람이 조치할 때까지 멈춤
RESUME = 'RESUME'  # 사람이 조치를 끝내고 로봇을 다시 재개하라고 명령함

# 명령 구분
HUMAN = 'human'
SELF = 'self'

# 구역 전략
# 좌표 구역
ZONE = {
    'shelf': (0.15, 0.15),  # 창고
    'work': (0.21, 0.06),  # 작업 구역
    'serve': (0.15, -0.15)  # 서빙
}
ZONE_RADIUS = 0.05
LOITER_SEC = 2.0
TIDY_TARGET = 'shelf'


def zone_of(x, y):
    """관측된 장소가 어디인지 돌려준다."""
    for name, (zone_x, zone_y) in ZONE.items():
        if math.hypot(x - zone_x, y - zone_y) <= ZONE_RADIUS:
            return name
    return None


def tidy_steps(object_id):
    """방치된 물건을 창고로 되돌리는 명령을 생성한다."""
    return [
        {'skill': 'pick', 'object_id': object_id},
        {'skill': 'place', 'object_id': object_id, 'target_id': TIDY_TARGET}
    ]


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
    ErrorCode.UNDEFINED_POSE: ABORT,
    ErrorCode.INTERNAL_ERROR: ABORT,
}

MAX_ATTEMPTS = 3
MAX_RECOVERY = 2


class Agent(Node):
    """명령을 받아 계획을 세우고 스킬 액션으로 실행하는 노드."""

    def __init__(self):
        """구독 1개(/command)와 액션 클라이언트 3개를 만든다."""
        super().__init__('agent')   # 부모 생성자 호출 및 노드 이름
        self._state = IDLE  # 현재 로봇의 상태
        self._source = None  # 현재 작업의 주체 기록
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
        # 방치 판정에 필요한 물건을 처음본 씬
        # 구역 판정용
        self._scene_xy = {}
        self._first_seen = {}  # 여기에 ojbect_id와 시간이 찍힌다 씬에서 사라지면 지운다.
        self._scene_stamp = None  # 마지막 스냅삿 시각. 경과 시간의 기준
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
        self.create_timer(1.0, self._idle_tick)

        # 자가 명령 파라미터 정리
        self._tidy_tries = {}  # object_id -> 정리 시도 횟수
        self._ignored = set()  # 정리 포기한 물체. 사람이 치울때까지 건드리지 않음
        self._tidy_id = None  # 지금 정리 중인 물체

        # 현재 상태 발행 타이머 및 발행 등록
        self._status_pub = self.create_publisher(RobotStatus, '/robot_status', 10)
        self.create_timer(1.0, self._publish_status)

    # 인지 노드가 보내고 있는 스냅샷
    def on_scene_state(self, msg):
        """씬에 있는 물체 id 목록과 처음 본 시각을 갱신한다."""
        now = Time.from_msg(msg.header.stamp)
        self._scene_stamp = now
        self._scene_ids = [obj.object_id for obj in msg.objects]
        self._scene_xy = {
            obj.object_id: (obj.pose.pose.position.x, obj.pose.pose.position.y)
            for obj in msg.objects
        }
        # 구역에 물체가 들어온 시간을 재고 구역이 바뀌면 다시 잰다.
        for object_id, (x, y) in self._scene_xy.items():
            zone = zone_of(x, y)
            seen = self._first_seen.get(object_id)
            if seen is None or seen[0] != zone:
                self._first_seen[object_id] = (zone, now)
        # 스냅샷에서 빠진 물체는 기록을 지운다. 다시 나타나면 처음부터 다시 센다.
        for object_id in list(self._first_seen):
            if object_id not in self._scene_ids:
                del self._first_seen[object_id]

    # 작업 구역에 방치되어 있는지 판단하는 함수
    def _loitering(self):
        """작업 구역에 LOITER_SEC 이상 머문 물체 id 하나를 돌려준다. 없으면 None."""
        if self._scene_stamp is None:
            return None
        for object_id, (zone, since) in self._first_seen.items():
            if zone != 'work':
                continue
            if object_id in self._ignored:
                continue
            if (self._scene_stamp - since).nanoseconds >= LOITER_SEC * 1e9:
                return object_id
        return None

    def _idle_tick(self):
        """쉴 때만 돈다. 작업 구역에 방치된 물건을 찾아 스스로 치운다."""
        if self._state != IDLE:
            return
        object_id = self._loitering()
        if object_id is None:
            return

        self._tidy_id = object_id
        self._tidy_tries[object_id] = self._tidy_tries.get(object_id, 0) + 1
        self.get_logger().info(
            f'자가 정리 실시 : {object_id} > {TIDY_TARGET} '
            f'{self._tidy_tries[object_id]} / {MAX_ATTEMPTS}')
        self._dispatch(tidy_steps(object_id), SELF)

    def _publish_status(self):
        """로봇의 상태를 밖으로 알린다."""
        msg = RobotStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self._state
        msg.source = self._source or ''
        msg.ignored = sorted(self._ignored)
        self._status_pub.publish(msg)

    # 인간의 명령이 들어오는 곳
    def on_command(self, msg):
        """토픽 메시지를 다듬어 넘기는 곳."""
        command = msg.data.strip()
        if not command:
            self.get_logger().warn('빈 명령')
            return
        # 재개는 계획이 아니라 상태 제어 LLM에 보내지 않음
        if command == RESUME:
            self._resume()
            return
        steps = self._to_steps(command)
        if steps is None:
            self.get_logger().warn(f'잘못된 명령 : {command}')
            return
        if self._state != IDLE:
            self.get_logger().warn(f'작업 중이라 명령을 받지 않는다 : {command}')
            return
        self._dispatch(steps, HUMAN)

    # 사람 명령과 자가 명령이 만나 계쇡으로 바꿔 실행 시키는 곳
    def _dispatch(self, steps, source):
        """명령을 계획으로 바꾸어 실행한다."""
        steps, error = llm_planner.validate_steps(steps, self._scene_ids)
        if error is not None:
            self.get_logger().warn(f'검증 거부 : {error} | {source}')
            self._finish(False)
            return
        self._plan = self._steps_to_goals(steps)  # 여기서 한번만 goal로
        # 마지막 작업이 플레이스 일경우 recovery_pose로 최종 위치 설정
        if steps[-1]['skill'] == 'place':
            home_goal = MoveTo.Goal()
            home_goal.pose_id = self.get_parameter('recovery_pose').value
            self._plan.append((self._move_to_client, home_goal))
        self._step = 0
        self._attempt = 1
        self._recovery = 0  # 새 명령 = 복구 초기화
        self._state = RUNNING
        self._source = source  # 이 시퀀스를 누가 시켰나
        self._run_step()

    def _finish(self, ok):
        """시퀀스를 종료하는 시점."""
        self._state = IDLE

        # 자가 정리 시퀀스를 종료
        object_id = self._tidy_id
        self._tidy_id = None

        # 정상 실행 완료
        if ok:
            self._tidy_tries.pop(object_id, None)  # 성공 예산 원복
        # 재시도 횟수 초과
        elif self._tidy_tries.get(object_id, 0) >= MAX_ATTEMPTS:
            self._ignored.add(object_id)
            self._state = ABORTED_WAIT
            self.get_logger().error(
                f'정리 불가 : {object_id} {MAX_ATTEMPTS}회 실패 사람 확인 요청')
        self._source = ''

    def _resume(self):
        """사람이 조치를 끝내고 로봇이 재실행한다."""
        if self._state != ABORTED_WAIT:
            self.get_logger().warn(f'재개할 것이 없다. {self._state}')
            return
        self._ignored.clear()
        self._tidy_tries.clear()
        self.get_logger().info('로봇 재실행 - 다시 씬의 상태를 감지함')
        pose = self.get_parameter('recovery_pose').value
        self._dispatch([{'skill': 'move_to', 'pose_id': pose}], HUMAN)

    # 텍스트를 실행을 위한 단계로 변경한다.
    def _to_steps(self, command):
        model = self.get_parameter('model').value
        steps = llm_planner.plan(command, self._scene_ids, llm_planner.make_ollama_caller(model))
        if steps is not None:
            self.get_logger().info(f'LLM 계획 채택 : {steps}')
            return steps

        # 문자열 파싱
        self.get_logger().warn(f'LLM 계획 실패 > 문자열 파서 폴백 : {command}')
        return self._build_plan(command.split())

    # 계획을 만드는 함수
    def _build_plan(self, parts):
        skill = parts[0]
        if skill == 'move_to' and len(parts) == 2:
            return [{'skill': skill, 'pose_id': parts[1]}]
        elif skill == 'pick' and len(parts) == 2:
            return [{'skill': skill, 'object_id': parts[1]}]
        elif skill == 'place' and len(parts) == 3:
            return [{'skill': skill, 'object_id': parts[1], 'target_id': parts[2]}]
        elif skill == 'deliver' and len(parts) == 3:
            return [{'skill': 'pick', 'object_id': parts[1]},
                    {'skill': 'place', 'object_id': parts[1], 'target_id': parts[2]}]
        return None

    # LLM 계획(dict 리스트)을 실행 고리가 쓰는 (client, goal) 리스트로 바꾼다.
    # Skill 이름은 plan() 검증기가 이미 걸렀으므로 else 분기가 없다.
    def _steps_to_goals(self, steps):
        goals = []
        for step in steps:
            skill = step['skill']
            if skill == 'move_to':
                goal = MoveTo.Goal()
                goal.pose_id = step['pose_id']
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
            self._finish(True)
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
            self._finish(False)
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
            self._finish(False)

    def _do_retry(self, code):
        """같은 goal을 다시 보낸다. OMPL은 확률적이라 시드가 바뀌면 풀린다."""
        if self._attempt >= MAX_ATTEMPTS:
            self.get_logger().error(f'RETRY {MAX_ATTEMPTS}회에서 소진(code={code}) > 중단')
            self._finish(False)
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
            self._finish(False)
            return
        self._state = RECOVERING
        self._recovery += 1
        self._attempt = 1
        pose = self.get_parameter('recovery_pose').value
        move_goal = MoveTo.Goal()
        move_goal.pose_id = pose
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
