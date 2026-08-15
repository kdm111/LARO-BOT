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
            'pose_id': {'type': 'string'},
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

    반환 : 스텝 리스트(계획), None(실패), [](정당한 거부, 명령을 아예 실행할 수 없다. : 빨간 블록이 있을 때 파란 링을 집어라)
    """
    if call_llm is None:
        call_llm = _call_ollama

    prompt = _build_prompt(command, scene_ids)

    # 1차 시도
    raw = _safe_call(call_llm, prompt)
    # 검증기에 넣기전에 가른다.
    # _parse_and_validate에 넘기면 계획은 빈 배열이 될 수 없다 판단되어 재 프롬프트로 밀려난다.
    if _is_refusal(raw):
        _LOG.info('현재 실행할 수 없는 명령 혹은 모델이 거부함 원문 %r', raw)
        return []
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


def _is_refusal(raw):
    """모델이 빈 배열로 할 수 없다고 대답했는가."""
    try:
        return json.loads(raw) == []
    except (TypeError, ValueError):
        return False


def _parse_and_validate(raw, scene_ids):
    """응답 문자열을 계획으로 바꾼다. (계획, None) 또는 (None, 실패이유)."""
    try:
        steps = json.loads(raw)
    except (TypeError, ValueError) as exc:
        # 앞뒤에 산문이 붙었거나 중간에 잘린 경우가 여기로 온다.
        # 관대하게 JSON만 긁어내지 않는다 - "거의 맞는" 출력을 통과시키게 된다.
        return None, f'JSON 파싱 실패: {exc}'

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
        '  {"skill": "move_to", "pose_id": "<name>"}\n'
        '\n'
        f'Valid pose_id values: {", ".join(POSE_IDS)}\n'
        f'Valid target_id values: {", ".join(TARGET_IDS)}\n'
        f'Objects present in the scene: {scene}\n'
        '\n'
        'Rules:\n'
        f'- Output at most {MAX_STEPS} steps.\n'
        '- Do NOT add steps that were not requested.\n'
        '  In particular, never insert move_to before pick or place.\n'
        '- move_to may only appear as a single step on its own.\n'
        '- Use only object_id values from the scene list above.\n'
        # 실측 실패(2026-08-07): "빨간 블록 집어" -> object_id를 명령의 한국어 단어
        # 그대로 복사해 검증에서 거부됐다. 같은 명령이 red_block으로 나온 적도 있어
        # 비결정적이다. 번역이 필요하다는 것을 규칙과 예시로 명시한다.
        '- The command may name objects in Korean. Translate them to the EXACT\n'
        '  English id from the scene list. Example: "빨간 블록" -> "red_block".\n'
        '- NEVER copy a word from the command into object_id. Always pick an id\n'
        '  that appears verbatim in the scene list above.\n'
        # 거부 경로. 2026-08-15에 열었다 - _is_refusal이 검증기 앞에서 []를 가려내므로
        # 이제 재프롬프트로 밀려나지 않는다(그 전에는 넣으면 안 되는 줄이었다).
        # 이 줄이 없으면 모델은 못 하는 명령에도 억지로 씬 목록에서 아무거나 골라
        # 낸다 - 조용히 틀린 일을 하는 것이 아무것도 안 하는 것보다 나쁘다.
        '- If the command cannot be done with the skills, objects, and targets\n'
        '  listed above, output an empty array [] and nothing else.\n'
        '- NEVER substitute a different object or target to make it work.\n'
        '  Asking for something absent is a refusal, not a naming mistake.\n'
        '\n'
        f'Command: {command}'
    )


def _retry_prompt(prompt, raw, error):
    """1차 실패를 이유와 함께 되돌려주는 재프롬프트.

    실측 실패(2026-08-07): 1차가 '씬에 없는 물체'로 거부되자 2차가 []를 냈다.
    거부당했다는 사실만 보고 "할 수 있는 게 없다"로 후퇴한 것이다.
    그래서 의도는 유지하고 틀린 필드만 고치라고 못박는다.
    """
    return (
        f'{prompt}\n'
        '\n'
        'Your previous answer was REJECTED.\n'
        f'Previous answer: {raw}\n'
        f'Reason: {error}\n'
        'Fix ONLY the field that was wrong and keep the original intent.\n'
        'Do NOT return an empty array here - the command was already judged\n'
        'to be achievable. If an object_id was rejected, replace it with the\n'
        'closest matching id from the scene list.\n'
        'Return only a corrected JSON array.'
    )


def _call_ollama(prompt, model=None, schema=PLAN_SCHEMA):
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
        'format': schema,
        'options': {'temperature': 0.3},
        'messages': [{'role': 'user', 'content': prompt}],
    }).encode('utf-8')

    # User-Agent를 명시하는 이유: OLLAMA_HOST가 runpod 프록시(*.proxy.runpod.net)일 때
    # 앞단 CDN이 urllib의 기본 UA("Python-urllib/3.x")를 봇으로 보고 403을 준다.
    # curl은 되는데 코드만 죽는 증상이 여기서 나온다. 값 자체는 무엇이든 상관없다.
    request = urllib.request.Request(
        f'{OLLAMA_HOST}/api/chat',
        data=body,
        headers={
            'Content-Type': 'application/json',
            'User-Agent': 'singlearm-agent/1.0',
        })

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


def make_recovery_caller(model):
    """복구 스키마를 물린 호출 함수. choose_recovery의 call_llm 자리에 꽂는다.

    make_ollama_caller와 같은 모양(prompt 하나만 받는다)이라
    테스트의 FakeLLM을 두 하네스가 그대로 공유한다.
    """
    tag = MODELS.get(model, model)

    def call(prompt):
        return _call_ollama(prompt, tag, RECOVERY_SCHEMA)

    return call


def _build_recovery_prompt(code, stage, detail, attempt, recovery_used, max_recovery):
    """실패 보고를 모델이 읽을 수 있는 형태로 옮긴다.

    스키마를 문법으로 강제해도 모델은 문법의 존재를 모른다.
    그래서 출력 형식을 프롬프트에도 문자열로 적는다(Ollama 공식 권고).
    """
    name = ERROR_NAMES.get(code, f'UNKNOWN({code})')
    return (
        'You choose ONE recovery strategy for a robot arm that failed a skill.\n'
        '\n'
        'Strategies:\n'
        '- RETRY   : send the same goal again. Useful only when the failure is random.\n'
        '            Motion planning is sampling based, so a new seed may solve it.\n'
        '- REGRASP : back off to a safe pose, look again, then redo the failed step.\n'
        '            Use when the gripper missed the object or dropped it.\n'
        '- RESCAN  : same motion as REGRASP. Use when the object was not found.\n'
        '- ABORT   : stop the whole command. Use when retrying cannot help.\n'
        '\n'
        'Failure report:\n'
        f'- error          : {name}\n'
        f'- stage          : {stage}\n'
        f'- detail         : {detail}\n'
        f'- attempt        : {attempt}\n'
        f'- recovery used  : {recovery_used} of {max_recovery}\n'
        '\n'
        'Rules:\n'
        '- Answer with one JSON object only: {"strategy": "<ONE OF THE ABOVE>"}\n'
        '- If the arm physically cannot reach the target, retrying changes nothing.\n'
        '- If you are unsure, choose ABORT. Stopping is always safe.\n'
    )


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
