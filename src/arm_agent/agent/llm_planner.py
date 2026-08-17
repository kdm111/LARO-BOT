"""자연어 명령 -> 검증된 스킬 계획(JSON) 변환기.

rclpy를 임포트하지 않는 순수 모듈이다('노드 != 도구' 원칙).
덕분에 ROS 없이 pytest로 돌릴 수 있고, agent 노드는 이걸 감싸는 얇은 래퍼가 된다.
arm_kinematics(순수 lib) <-> skill_server(노드) 관계와 같은 구조다.

설계 사상: LLM 출력을 신뢰할 수 없는 외부 입력으로 간주하고
도구 스키마(계약) + 검증 계층 + 폴백 사다리로 감싼다.
"""

import logging

from .llm_ollama import _call_ollama
from .llm_prompts import _build_prompt, _build_recovery_prompt, _retry_prompt
from .llm_validate import NOT_IN_SCENE, _parse_and_validate, _parse_and_validate_recovery


_LOG = logging.getLogger(__name__)


def plan(command, scene_ids=None, call_llm=None):
    """자연어 명령을 검증된 스킬 계획으로 바꾼다. 실패하면 None.

    None을 받은 호출자는 기존 문자열 파서로 내려간다(폴백 사다리 3번째 칸).
    실패 이유는 여기서 로그로만 남긴다 - 호출자의 동작이 어차피 하나뿐이라
    이유로 분기할 일이 없고, 로그는 프롬프트 개선 재료가 된다.

    scene_ids가 None이면 씬 정보가 아직 없다는 뜻이므로 물체 대조를 건너뛴다.
    call_llm은 테스트에서 가짜 함수를 주입하는 자리다(프롬프트 -> 응답 문자열).

    반환 : 스텝 리스트(계획), None(실패), [](정당한 거부, 명령을 아예 실행할 수 없다. : 빨간 블록이 있을 때 파란 링을 집어라)
    """
    if call_llm is None:
        call_llm = _call_ollama

    prompt = _build_prompt(command, scene_ids)

    # 1차 시도
    raw = _safe_call(call_llm, prompt)
    steps, error = _parse_and_validate(raw, scene_ids)
    if error is None:
        return steps
    _LOG.warning('계획 거부(1차): %s | 원문: %r', error, raw)

    # 없는 물건으로 돌아올 경우 되묻지 않는다.
    if error.startswith(NOT_IN_SCENE):
        return []

    # 2차 시도 - 무엇이 왜 틀렸는지 알려주고 다시 묻는다. 딱 한 번만.
    raw = _safe_call(call_llm, _retry_prompt(prompt, raw, error))
    steps, error = _parse_and_validate(raw, scene_ids)
    if error is None:
        return steps
    _LOG.warning('계획 거부(2차): %s | 원문: %r -> 문자열 파서로 폴백', error, raw)
    return None


def _safe_call(call_llm, prompt):
    """LLM 호출을 감싼다. 어떤 예외가 나도 에이전트를 죽이지 않는다."""
    # OSError = 연결 거부·타임아웃·HTTP 에러(URLError/HTTPError가 모두 이 밑)
    # ValueError = 응답 본문이 JSON이 아님 / KeyError = 응답 구조가 예상과 다름
    try:
        return call_llm(prompt)
    except (OSError, ValueError, KeyError) as exc:
        _LOG.warning('LLM 호출 실패: %s', exc)
        return ''   # 파싱 단계에서 거부되고 폴백으로 이어진다


def choose_recovery(code, stage, detail='', attempt=1,
                    recovery_used=0, max_recovery=2, call_llm=None):
    """실패 보고를 보고 복구 전략을 하나 고른다. 못 고르면 None.

    None이면 호출자가 STRATEGY 표로 폴백한다
    - plan()이 문자열 파서로 떨어지는 자리와 같다.
    """
    # 예산이 끝났으면 묻지 않는다. 무한루프 방지를 모델 판단에 맡기지 않는다.
    if recovery_used >= max_recovery:
        return 'ABORT'
    if call_llm is None:
        call_llm = _call_ollama

    prompt = _build_recovery_prompt(code, stage, detail, attempt,
                                    recovery_used, max_recovery)
    raw = _safe_call(call_llm, prompt)
    label, error = _parse_and_validate_recovery(raw, code)
    if label is not None:
        return label

    # 사다리 2칸. 거부 사유를 붙여 정확히 한 번만 다시 묻는다.
    # 더 물으면 실패 하나가 모델을 무한히 호출하고 복구가 실패보다 비싸진다.
    _LOG.warning('전략 거부(1차): %s | 원문: %r', error, raw)
    raw = _safe_call(call_llm, _retry_prompt(prompt, raw, error))
    label, error = _parse_and_validate_recovery(raw, code)
    if label is not None:
        return label

    _LOG.warning('전략 거부(2차): %s | 원문: %r -> 표로 폴백', error, raw)
    return None
