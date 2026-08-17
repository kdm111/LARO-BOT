import json
import logging

from .llm_contract import (
    ERROR_NAMES, MAX_STEPS, NO_RETRY_CODES, POSE_IDS, SKILLS, STRATEGIES, TARGET_IDS,
)

_LOG = logging.getLogger(__name__)

# 재프롬프트로 고칠수 없는 실패. 다시 물으면 모델이 씬 목록에 있는 다른 물체를 갈아끼운다.
# 씬에 없는 물체
NOT_IN_SCENE = '씬에 없는 물체'


def _parse_and_validate(raw, scene_ids):
    """응답 문자열을 계획으로 바꾼다. (계획, None) 또는 (None, 실패이유).

    빈 배열은 여기서 '정당한 거부'로 통과시킨다.
    validate_steps는 여전히 []을 거부한다.
    """
    try:
        steps = json.loads(raw)
    except (TypeError, ValueError) as exc:
        # 앞뒤에 산문이 붙었거나 중간에 잘린 경우가 여기로 온다.
        # 관대하게 JSON만 긁어내지 않는다 - "거의 맞는" 출력을 통과시키게 된다.
        return None, f'JSON 파싱 실패: {exc}'

    if steps == []:
        _LOG.info('모델이 할수 없다고 답함- 거부')
        return [], None

    return validate_steps(steps, scene_ids)


def validate_steps(steps, scene_ids=None):
    """steps를 검증 4층에 통과시킨다. (steps, None) 또는 (None, 실패이유)."""
    if not isinstance(steps, list) or not steps:
        return None, '계획은 비어 있지 않은 배열이어야 한다'
    if len(steps) > MAX_STEPS:
        return None, f'스텝 {len(steps)}개는 상한 {MAX_STEPS}를 넘을 수 없다'
    for step in steps:
        error = _validate_step(step, scene_ids)
        if error is not None:
            return None, error

    # 계약의 구조에서 나오는 규칙: move_to는 단독 으로만 온다.
    # pick,plce,move_to는 각 1스텝
    if len(steps) > 1 and any(step['skill'] == 'move_to' for step in steps):
        return None, 'move_to는 단독 스텝으로만 유효'
    return steps, None


def _parse_and_validate_recovery(raw, code):
    """복구 전략 응답을 검증한다. (라벨, None) 또는 (None, 실패이유).

    plan()의 _parse_and_validate와 같은 자리 - 모델 출력이 지나는 유일한 문이다.
    """
    try:                                             # 1층 형식
        data = json.loads(raw)
    except (TypeError, ValueError) as exc:
        return None, f'JSON 파싱 실패: {exc}'

    if not isinstance(data, dict):                   # 2층 모양
        return None, '전략은 객체 하나여야 한다'
    if 'strategy' not in data:
        return None, "'strategy' 필드가 없다"

    # 대소문자와 앞뒤 공백은 형식 문제지 의미 문제가 아니다. 정규화해서 통과시킨다.
    label = str(data['strategy']).strip().upper()
    if label not in STRATEGIES:                      # 3층 화이트리스트
        return None, f'계약에 없는 전략: {label!r}'

    # 4층. 문법도 화이트리스트도 통과하지만 올바르지 않은 답을 여기서 막는다.
    # 도달 불가는 같은 조건으로 다시 해도 도달 불가다(HANDOFF §3.4 실측).
    if code in NO_RETRY_CODES and label == 'RETRY':
        return None, f'{ERROR_NAMES.get(code, code)}는 재시도해도 결과가 같다'

    return label, None


def _validate_step(step, scene_ids):
    """스텝 하나를 검사한다. 통과면 None, 아니면 실패 이유 문자열."""
    if not isinstance(step, dict):
        return f'스텝이 객체가 아니다: {step!r}'

    skill = step.get('skill')
    if skill not in SKILLS:
        return f'계약에 없는 스킬: {skill!r}'

    for field in SKILLS[skill]:
        if not step.get(field):
            return f'{skill}에 필수 인자 {field}가 없다'

    if skill == 'move_to' and step['pose_id'] not in POSE_IDS:
        return f'SRDF에 없는 pose_id: {step["pose_id"]!r}'

    # target_id는 고정 장소 하나뿐이라 화이트리스트로 잡는다.
    # 씬과 무관하므로 scene_ids가 없어도 검사한다 - pose_id와 같은 취급.
    if skill == 'place' and step['target_id'] not in TARGET_IDS:
        return f'없는 장소: {step["target_id"]!r}'

    if scene_ids is not None and 'object_id' in SKILLS[skill]:
        if step['object_id'] not in scene_ids:
            return f'{NOT_IN_SCENE}: {step["object_id"]!r}'

    return None
