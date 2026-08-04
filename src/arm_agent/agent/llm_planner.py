"""자연어 명령 -> 검증된 스킬 계획(JSON) 변환기.

rclpy를 임포트하지 않는 순수 모듈이다('노드 != 도구' 원칙).
덕분에 ROS 없이 pytest로 돌릴 수 있고, agent 노드는 이걸 감싸는 얇은 래퍼가 된다.
arm_kinematics(순수 lib) <-> skill_server(노드) 관계와 같은 구조다.

설계 사상: LLM 출력을 신뢰할 수 없는 외부 입력으로 간주하고
도구 스키마(계약) + 검증 계층 + 폴백 사다리로 감싼다.
"""

import json
import logging
import os
import urllib.request

_LOG = logging.getLogger(__name__)

# ---- 계약: arm_interfaces의 액션 3종을 LLM이 읽을 수 있는 형태로 옮긴 것 ----
# 값은 그 스킬의 필수 인자. deliver는 여기 없다 - 문자열 파서의 축약일 뿐이고
# 계약이 아니므로 LLM에게 가르치지 않는다(펼친 pick+place를 받는다).
SKILLS = {
    'pick': ('object_id',),
    'place': ('object_id', 'target_id'),
    'move_to': ('target_name',),
}

# SRDF omx_f.srdf의 arm 그룹 group_state 실제 값. observe는 아직 없다(M5 이관).
TARGET_NAMES = ('init', 'home')

# place가 놓을 수 있는 유일한 장소(작업대 오른쪽 위 고정).
# 카메라가 검출하는 물체가 아니라 고정 좌표라 scene_ids와 무관하게 검사한다.
TARGET_IDS = ('bin',)

# 지금 계약에서 가장 긴 계획은 deliver의 2스텝이다.
# 과잉 생성(요청하지 않은 단계 삽입)의 1차 방어선이기도 하다.
MAX_STEPS = 2

# 짧은 이름 -> Ollama 모델 태그. launch의 scene:=one_block 과 같은 사상이다.
# 여기 없는 이름을 주면 태그 그대로 쓴다(새 모델을 코드 수정 없이 시험하려고).
MODELS = {
    'llama': 'llama3.1:8b',
    'exaone': 'exaone3.5:7.8b',
}

# 컨테이너 안에서는 호스트의 Ollama를 봐야 한다. 환경변수로 갈아끼운다.
OLLAMA_HOST = os.environ.get('OLLAMA_HOST', 'http://localhost:11434')
# 기본 모델 = exaone3.5:7.8b (2026-08-04 확정, 근거 eval_results/eval_*_v2.csv).
# 자체 시험지 182케이스에서 정확도 1위(128/182)이면서 지연도 1위(0.47초)였다.
OLLAMA_MODEL = os.environ.get('OLLAMA_MODEL', 'exaone3.5:7.8b')
HTTP_TIMEOUT_SEC = 30.0

# format 인자로 주면 Ollama가 이 스키마를 문법(GBNF)으로 컴파일해
# 위반하는 토큰을 샘플링 단계에서 아예 못 뽑게 막는다.
# -> 파싱 불가능한 JSON이 구조적으로 나올 수 없다. 단 "유효 != 올바름"이라
#    내용 검증은 아래 _validate_step이 따로 한다.
PLAN_SCHEMA = {
    'type': 'array',
    'maxItems': MAX_STEPS,
    'items': {
        'type': 'object',
        'properties': {
            'skill': {'type': 'string', 'enum': list(SKILLS)},
            'object_id': {'type': 'string'},
            'target_id': {'type': 'string'},
            'target_name': {'type': 'string'},
        },
        'required': ['skill'],
    },
}


def plan(command, scene_ids=None, call_llm=None):
    """자연어 명령을 검증된 스킬 계획으로 바꾼다. 실패하면 None.

    None을 받은 호출자는 기존 문자열 파서로 내려간다(폴백 사다리 3번째 칸).
    실패 이유는 여기서 로그로만 남긴다 - 호출자의 동작이 어차피 하나뿐이라
    이유로 분기할 일이 없고, 로그는 프롬프트 개선 재료가 된다.

    scene_ids가 None이면 씬 정보가 아직 없다는 뜻이므로 물체 대조를 건너뛴다.
    call_llm은 테스트에서 가짜 함수를 주입하는 자리다(프롬프트 -> 응답 문자열).
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


def _parse_and_validate(raw, scene_ids):
    """응답 문자열을 계획으로 바꾼다. (계획, None) 또는 (None, 실패이유)."""
    try:
        steps = json.loads(raw)
    except (TypeError, ValueError) as exc:
        # 앞뒤에 산문이 붙었거나 중간에 잘린 경우가 여기로 온다.
        # 관대하게 JSON만 긁어내지 않는다 - "거의 맞는" 출력을 통과시키게 된다.
        return None, f'JSON 파싱 실패: {exc}'

    if not isinstance(steps, list) or not steps:
        return None, '계획은 비어 있지 않은 배열이어야 한다'
    if len(steps) > MAX_STEPS:
        return None, f'스텝 {len(steps)}개는 상한 {MAX_STEPS}개를 넘는다'

    for step in steps:
        error = _validate_step(step, scene_ids)
        if error is not None:
            return None, error

    # 계약의 구조에서 나오는 규칙: move_to는 어떤 유효한 계획에서도 단독으로만 온다
    # (pick/place/move_to는 각 1스텝, deliver는 pick+place). 스킬 이름도 스텝 수도
    # 합법인 채로 통과하는 "요청하지 않은 move_to 삽입"을 이걸로 잡는다.
    if len(steps) > 1 and any(step['skill'] == 'move_to' for step in steps):
        return None, 'move_to는 단독 스텝으로만 유효하다'

    return steps, None


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

    if skill == 'move_to' and step['target_name'] not in TARGET_NAMES:
        return f'SRDF에 없는 자세 이름: {step["target_name"]!r}'

    # target_id는 고정 장소 하나뿐이라 화이트리스트로 잡는다.
    # 씬과 무관하므로 scene_ids가 없어도 검사한다 - target_name과 같은 취급.
    if skill == 'place' and step['target_id'] not in TARGET_IDS:
        return f'없는 장소: {step["target_id"]!r}'

    if scene_ids is not None and 'object_id' in SKILLS[skill]:
        if step['object_id'] not in scene_ids:
            return f'씬에 없는 물체: {step["object_id"]!r}'

    return None


def _build_prompt(command, scene_ids):
    """도구 스키마와 씬 정보를 담은 프롬프트를 만든다.

    지시문은 영어다(모델이 더 잘 따른다). 명령 자체는 한국어여도 된다.
    문법으로 형식을 강제해도 모델은 그 문법의 존재를 모르므로,
    스키마를 프롬프트에도 문자열로 적어준다(Ollama 공식 권고).
    """
    scene = ', '.join(scene_ids) if scene_ids else '(unknown)'
    return (
        'You output ONLY a JSON array of steps. No prose, no explanation.\n'
        '\n'
        'Available skills (use EXACTLY these names and fields):\n'
        '  {"skill": "pick",    "object_id": "<object>"}\n'
        '  {"skill": "place",   "object_id": "<object>", "target_id": "<target>"}\n'
        '  {"skill": "move_to", "target_name": "<name>"}\n'
        '\n'
        f'Valid target_name values: {", ".join(TARGET_NAMES)}\n'
        f'Valid target_id values: {", ".join(TARGET_IDS)}\n'
        f'Objects present in the scene: {scene}\n'
        '\n'
        'Rules:\n'
        f'- Output at most {MAX_STEPS} steps.\n'
        '- Do NOT add steps that were not requested.\n'
        '  In particular, never insert move_to before pick or place.\n'
        '- move_to may only appear as a single step on its own.\n'
        '- Use only object_id values from the scene list above.\n'
        '\n'
        f'Command: {command}'
    )


def _retry_prompt(prompt, raw, error):
    """1차 실패를 이유와 함께 되돌려주는 재프롬프트."""
    return (
        f'{prompt}\n'
        '\n'
        'Your previous answer was REJECTED.\n'
        f'Previous answer: {raw}\n'
        f'Reason: {error}\n'
        'Return only a corrected JSON array.'
    )


def _call_ollama(prompt, model=None):
    """호스트에서 도는 Ollama에 한 번 물어보고 응답 문자열을 돌려준다.

    temperature 0 = 확률표에서 항상 1등만 고른다(greedy) -> 같은 입력에 같은 출력.
    temperature 0.3 = 같은 입력에서 다른 출력이 나올 수 있다.
    0으로 두면 완전 재현되지만 재프롬프트가 같은 답을 반복한다.
    로컬 백엔드의 최대 장점인 재현성이 이 한 줄에서 나온다.
    urllib만 쓰므로 새 의존성이 없다.
    """
    body = json.dumps({
        'model': model or OLLAMA_MODEL,
        'stream': False,
        'format': PLAN_SCHEMA,
        'options': {'temperature': 0.3},
        'messages': [{'role': 'user', 'content': prompt}],
    }).encode('utf-8')

    request = urllib.request.Request(
        f'{OLLAMA_HOST}/api/chat',
        data=body,
        headers={'Content-Type': 'application/json'})

    with urllib.request.urlopen(request, timeout=HTTP_TIMEOUT_SEC) as response:
        payload = json.loads(response.read().decode('utf-8'))
    return payload['message']['content']


def make_ollama_caller(model):
    """모델을 고정한 호출 함수를 만든다. plan()의 call_llm 자리에 꽂는다.

    테스트가 FakeLLM을 꽂는 그 자리와 같다 -> 주입구를 새로 뚫지 않는다.
    짧은 이름(llama, exaone)이면 MODELS로 펼치고, 없으면 태그 그대로 쓴다.
    """
    tag = MODELS.get(model, model)

    def call(prompt):
        return _call_ollama(prompt, tag)

    return call
