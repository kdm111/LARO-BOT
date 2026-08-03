"""llm_planner의 파서·검증기·폴백 사다리를 검증하는 결정론 테스트.

LLM은 가짜 함수로 주입해 차단한다 -> 모델도 네트워크도 없이 항상 같은 결과가 나온다.
모델 자체의 품질(한국어 이해, 계획의 타당성)은 여기서 재지 않는다.
그건 실호출 시험지의 몫이고, 채점 방식도 pass^k로 다르다.
"""

from arm_agent.agent.llm_planner import plan

# 이 씬에 실재하는 물체. plan()이 object_id를 대조할 목록이다.
SCENE = ['red_block']


class FakeLLM:
    """정해둔 응답을 순서대로 돌려주고 호출 횟수를 세는 가짜 LLM.

    응답이 떨어지면 마지막 것을 계속 돌려준다.
    덕분에 거부 케이스는 응답을 하나만 줘도 재프롬프트까지 같은 답을 받는다.
    """

    def __init__(self, *responses):
        self._responses = list(responses)
        self.calls = 0

    def __call__(self, prompt):
        index = min(self.calls, len(self._responses) - 1)
        self.calls += 1
        return self._responses[index]


# ---------- 정상 경로 ----------

def test_single_skill():
    # 정상 경로의 기준선. 이게 깨지면 나머지 테스트는 판정할 의미가 없다.
    llm = FakeLLM('[{"skill": "pick", "object_id": "red_block"}]')
    assert plan('pick red_block', SCENE, llm) == [
        {'skill': 'pick', 'object_id': 'red_block'}]
    assert llm.calls == 1   # 한 번에 통과했으니 재프롬프트가 없어야 한다


def test_two_step_plan():
    # deliver는 계약에 없는 축약이다(문자열 파서의 편의 문법).
    # LLM은 계약에 있는 pick + place 두 스텝으로 펼쳐야 한다.
    llm = FakeLLM(
        '[{"skill": "pick", "object_id": "red_block"},'
        ' {"skill": "place", "object_id": "red_block", "target_id": "bin"}]')
    out = plan('deliver red_block bin', SCENE, llm)
    assert [step['skill'] for step in out] == ['pick', 'place']


def test_named_pose():
    # move_to만 target_name(SRDF 이름 자세)을 쓴다. object_id가 아니다.
    llm = FakeLLM('[{"skill": "move_to", "target_name": "home"}]')
    assert plan('move_to home', SCENE, llm) == [
        {'skill': 'move_to', 'target_name': 'home'}]


def test_scene_unknown_skips_object_check():
    # scene_ids가 None이면 "씬 정보가 아직 없다"는 뜻이다.
    # /scene_state 도착 전에 계획을 통째로 막아버리면 안 되므로 물체 대조를 건너뛴다.
    llm = FakeLLM('[{"skill": "pick", "object_id": "blue_block"}]')
    assert plan('pick blue_block', None, llm) is not None


# ---------- 거부해야 하는 것 ----------

def test_unknown_skill_rejected():
    # 1강 성질 ②(환각). grab은 문맥상 매우 자연스럽지만 계약에 없는 이름이다.
    llm = FakeLLM('[{"skill": "grab", "object_id": "red_block"}]')
    assert plan('pick red_block', SCENE, llm) is None
    assert llm.calls == 2   # 재프롬프트 1회를 쓰고 포기했다


def test_missing_required_param_rejected():
    # 스킬 이름은 맞는데 필수 인자가 없다.
    # 화이트리스트만으로는 못 잡는 층이라 파라미터 검사가 따로 필요하다.
    llm = FakeLLM('[{"skill": "pick"}]')
    assert plan('pick red_block', SCENE, llm) is None


def test_prose_around_json_rejected():
    # 1강 성질 ③(형식 누출). format:{schema}를 쓰면 나오지 않아야 하지만,
    # 폴백 사다리는 "그래도 나왔을 때"를 위해 존재한다. 관대하게 긁지 않고 거부한다.
    llm = FakeLLM(
        '물론이죠! 다음은 계획입니다:\n'
        '[{"skill": "pick", "object_id": "red_block"}]')
    assert plan('pick red_block', SCENE, llm) is None


def test_truncated_json_rejected():
    # 문법은 지켰는데 토큰 한도나 조기 종료로 잘린 경우.
    # 5강에서 짚은 형식 강제의 공통 함정 ②가 이것이다.
    llm = FakeLLM('[{"skill": "pick", "object_id": "red_bl')
    assert plan('pick red_block', SCENE, llm) is None


def test_step_limit_rejected():
    # 과잉 생성의 1차 방어선. 지금 계약에서 가장 긴 계획은 deliver의 2스텝이다.
    llm = FakeLLM(
        '[{"skill": "move_to", "target_name": "home"},'
        ' {"skill": "pick", "object_id": "red_block"},'
        ' {"skill": "move_to", "target_name": "init"}]')
    assert plan('pick red_block', SCENE, llm) is None


def test_object_not_in_scene_rejected():
    # 3강 "유효한 JSON != 올바른 JSON". 형식은 완벽한데 씬에 없는 물체다.
    # plan()이 scene_ids 인자를 받는 이유가 바로 이 케이스다.
    llm = FakeLLM('[{"skill": "pick", "object_id": "blue_block"}]')
    assert plan('pick blue_block', SCENE, llm) is None


def test_unknown_target_name_rejected():
    # observe는 SRDF에 아직 없다(M5로 이관됨).
    # BFCL의 '관련성 판별'과 같은 상황 - 할 수 없는 일은 하지 말고 거부해야 한다.
    llm = FakeLLM('[{"skill": "move_to", "target_name": "observe"}]')
    assert plan('move_to observe', SCENE, llm) is None


def test_unknown_target_id_rejected():
    # target_id도 화이트리스트다(고정 장소 bin 하나뿐).
    # 실호출에서 모델이 프롬프트의 자리표시자를 글자 그대로 베껴 낸 적이 있다.
    # scene_ids와 무관한 검사라 씬 정보가 없어도 잡아야 한다.
    llm = FakeLLM(
        '[{"skill": "place", "object_id": "red_block", "target_id": "<target>"}]')
    assert plan('place red_block bin', SCENE, llm) is None
    assert plan('place red_block bin', None, llm) is None


def test_move_to_mixed_with_pick_rejected():
    # 4강에서 5번 중 3번 관측된 실제 실패: 요청하지 않은 move_to 삽입.
    # 스킬 이름도 스텝 수도 합법이라 화이트리스트와 스텝 상한을 둘 다 통과한다.
    # 잡는 근거는 계약의 구조다 - move_to는 어떤 유효한 계획에서도 단독으로만 온다.
    llm = FakeLLM(
        '[{"skill": "move_to", "target_name": "home"},'
        ' {"skill": "pick", "object_id": "red_block"}]')
    assert plan('pick red_block', SCENE, llm) is None


# ---------- 폴백 사다리 ----------

def test_retry_once_then_success():
    # 사다리 2번째 칸. 무엇이 틀렸는지 알려주고 다시 물으면 고칠 확률이 꽤 높다.
    llm = FakeLLM(
        '[{"skill": "grab", "object_id": "red_block"}]',
        '[{"skill": "pick", "object_id": "red_block"}]')
    assert plan('pick red_block', SCENE, llm) == [
        {'skill': 'pick', 'object_id': 'red_block'}]
    assert llm.calls == 2


def test_retry_limited_to_one():
    # 재프롬프트는 정확히 1회. 3회 이상 불렸다면 사다리가 새는 것이고,
    # 실패한 명령 하나가 모델을 무한히 호출하게 된다.
    llm = FakeLLM('[{"skill": "grab"}]')
    assert plan('pick red_block', SCENE, llm) is None
    assert llm.calls == 2
