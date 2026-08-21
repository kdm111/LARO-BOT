import json
import os
import urllib.request

from .llm_contract import PLAN_SCHEMA, RECOVERY_SCHEMA

# 짧은 이름 -> Ollama 모델 태그. launch의 scene:=one_block 과 같은 사상이다.
# 여기 없는 이름을 주면 태그 그대로 쓴다(새 모델을 코드 수정 없이 시험하려고).
MODELS = {
    'llama': 'llama3.1:8b',
    'exaone': 'exaone3.5:7.8b',
    'gemma': 'gemma4:26b',
    'qwen': 'qwen3.5:9b',
}

# 컨테이너 안에서는 호스트의 Ollama를 봐야 한다. 환경변수로 갈아끼운다.
OLLAMA_HOST = os.environ.get('OLLAMA_HOST', 'http://localhost:11434')
# 기본 모델 = gemma4:26b (2026-08-22 확정, 근거 eval_*_runpod-20260821-allmodels-k3.csv).
# 242케이스(10개 언어)에서 220/242 로 1위, N2 이중정답 재채점 시 229/242(94.6%).
# 2위 qwen3.5:9b(218/242)와 McNemar 동률이지만 사용자가 정확도 우선으로 선택했다.
# 지연은 3.70초/호출로 qwen(2.13초)보다 느리다 - 속도가 필요해지면 qwen 이 대안.
# (구 기본값 exaone3.5:7.8b 는 2026-08-04 v2 시험지 기준 1위였다 - 세대 교체)
OLLAMA_MODEL = os.environ.get('OLLAMA_MODEL', 'gemma4:26b')
HTTP_TIMEOUT_SEC = 30.0


def _call_ollama(prompt, model=None, schema=PLAN_SCHEMA):
    """호스트에서 도는 Ollama에 한 번 물어보고 응답 문자열을 돌려준다.

    temperature 0 = 확률표에서 항상 1등만 고른다(greedy) -> 같은 입력에 같은 출력.
    temperature 0.3 = 같은 입력에서 다른 출력이 나올 수 있다.
    0으로 두면 완전 재현되지만 재프롬프트가 같은 답을 반복한다.
    로컬 백엔드의 최대 장점인 재현성이 이 한 줄에서 나온다.
    urllib만 쓰므로 새 의존성이 없다.
    """
    # think=False : 추론(thinking) 모드를 끈다.
    # 로봇 명령에 답 앞의 긴 추론은 지연으로만 나타난다. 2026-08-16 실측 —
    # qwen3:8b가 켠 채 6.43초(추론 591자), 끄면 4.12초. exaone3.5:7.8b는
    # 추론 모드가 없어서 값에 영향도 없고 오류도 안 난다(0.75 vs 0.78초).
    # 모든 모델을 "바로 답한다"는 같은 조건에 세워야 비교가 성립한다.
    body = json.dumps({
        'think': False,
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
