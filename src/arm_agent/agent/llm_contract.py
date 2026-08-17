# ---- 계약: arm_interfaces의 액션 3종을 LLM이 읽을 수 있는 형태로 옮긴 것 ----
# 값은 그 스킬의 필수 인자. deliver는 여기 없다 - 문자열 파서의 축약일 뿐이고
# 계약이 아니므로 LLM에게 가르치지 않는다(펼친 pick+place를 받는다).
SKILLS = {
    'pick': ('object_id',),
    'place': ('object_id', 'target_id'),
    'move_to': ('pose_id',),
}

STRATEGIES = ('RETRY', 'REGRASP', 'RESCAN', 'ABORT')
# Ollama format에 넘길 복구 전략 응답 스키마.
# 문법 강제라 이 모양을 벗어난 JSON은 구조적으로 못 나온다.
RECOVERY_SCHEMA = {
    'type': 'object',
    'properties': {'strategy': {'type': 'string', 'enum': list(STRATEGIES)}},
    'required': ['strategy'],
}

# ErrorCode.msg의 실값. 숫자를 프롬프트에 넣으면 모델이 못 읽으므로 이름으로 바꾼다.
# arm_interfaces를 import하면 ROS 없이 테스트가 못 돈다 -> 계약의 LLM 사본이다.
ERROR_NAMES = {
    1: 'OBJECT_NOT_FOUND', 2: 'UNREACHABLE', 3: 'PLANNING_FAILED',
    4: 'GRASP_FAILED', 5: 'OBJECT_MOVED', 6: 'GRIPPER_EMPTY',
    7: 'EXECUTION_TIMEOUT', 8: 'UNDEFINED_POSE', 99: 'INTERNAL_ERROR',
}

# 같은 조건으로 다시 해도 결과가 같다. 모델이 RETRY를 고르면 못 닿는 목표에 세번 매달린다.
# 발행되지 않는 포즈를 리턴받았을 때도 막는다.
NO_RETRY_CODES = (2, 8, 99)

# SRDF omx_f.srdf의 arm 그룹 group_state 실제 값. observe는 아직 없다(M5 이관).
POSE_IDS = ('init', 'home')

# place가 놓을 수 있는 유일한 장소(작업대 오른쪽 위 고정).
# 카메라가 검출하는 물체가 아니라 고정 좌표라 scene_ids와 무관하게 검사한다.
TARGET_IDS = ('counter', 'bin', 'shelf_block', 'shelf_ring')

# 지금 계약에서 가장 긴 계획은 deliver의 2스텝이다.
# 과잉 생성(요청하지 않은 단계 삽입)의 1차 방어선이기도 하다.
MAX_STEPS = 2

# format 인자로 주면 Ollama가 이 스키마를 문법(GBNF)으로 컴파일해
# 위반하는 토큰을 샘플링 단계에서 아예 못 뽑게 막는다.
# -> 파싱 불가능한 JSON이 구조적으로 나올 수 없다. 단 "유효 != 올바름"이라
#    내용 검증은 _validate_step이 따로 한다.
PLAN_SCHEMA = {
    'type': 'array',
    'maxItems': MAX_STEPS,
    'items': {
        'type': 'object',
        'properties': {
            'skill': {'type': 'string', 'enum': list(SKILLS)},
            'object_id': {'type': 'string'},
            'target_id': {'type': 'string'},
            'pose_id': {'type': 'string'},
        },
        'required': ['skill'],
    },
}
