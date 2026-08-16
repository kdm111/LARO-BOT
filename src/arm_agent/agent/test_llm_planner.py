"""llm_planner의 파서·검증기·폴백 사다리를 검증하는 결정론 테스트.

LLM은 가짜 함수로 주입해 차단한다 -> 모델도 네트워크도 없이 항상 같은 결과가 나온다.
모델 자체의 품질(한국어 이해, 계획의 타당성)은 여기서 재지 않는다.
그건 실호출 시험지의 몫이고, 채점 방식도 pass^k로 다르다.
"""

from agent.llm_planner import choose_recovery, plan

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
    # move_to만 pose_id(SRDF group_state 이름)를 쓴다. object_id가 아니다.
    llm = FakeLLM('[{"skill": "move_to", "pose_id": "home"}]')
    assert plan('move_to home', SCENE, llm) == [
        {'skill': 'move_to', 'pose_id': 'home'}]


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
        '[{"skill": "move_to", "pose_id": "home"},'
        ' {"skill": "pick", "object_id": "red_block"},'
        ' {"skill": "move_to", "pose_id": "init"}]')
    assert plan('pick red_block', SCENE, llm) is None


def test_object_not_in_scene_rejected():
    # 3강 "유효한 JSON != 올바른 JSON". 형식은 완벽한데 씬에 없는 물체다.
    # plan()이 scene_ids 인자를 받는 이유가 바로 이 케이스다.
    #
    # ★ 2026-08-16에 반환이 None -> []로 바뀌었다. 뜻이 다르다:
    #   None = 계획을 못 냈다 -> 문자열 파서로 내려간다(폴백 사다리)
    #   []   = 낼 계획이 없다 -> 사람에게 되묻고 멈춘다(정당한 거부)
    #   없는 물건은 파서가 대신 풀 수 있는 문제가 아니므로 내려보낼 이유가 없다.
    #   되묻지도 않는다 - 다시 물으면 모델이 씬에 있는 다른 물체로 갈아끼우고,
    #   그게 실제 사고를 냈다(초록을 시켰는데 빨강을 옮기고 served까지 올림).
    llm = FakeLLM('[{"skill": "pick", "object_id": "blue_block"}]')
    assert plan('pick blue_block', SCENE, llm) == []


def test_not_in_scene_does_not_reprompt():
    """없는 물건이면 2차 시도 자체가 없어야 한다. 되묻는 순간 바꿔치기가 열린다."""
    llm = FakeLLM('[{"skill": "pick", "object_id": "blue_block"}]')
    assert plan('pick blue_block', SCENE, llm) == []
    # FakeLLM은 호출될 때마다 같은 답을 주지만, 호출 횟수는 센다.
    assert llm.calls == 1, f'재프롬프트가 일어났다(호출 {llm.calls}회)'


def test_unknown_pose_id_rejected():
    # observe는 SRDF에 아직 없다(M5로 이관됨).
    # BFCL의 '관련성 판별'과 같은 상황 - 할 수 없는 일은 하지 말고 거부해야 한다.
    llm = FakeLLM('[{"skill": "move_to", "pose_id": "observe"}]')
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
        '[{"skill": "move_to", "pose_id": "home"},'
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


# ---------- choose_recovery : 복구 전략 선택 ----------
# 계획 하네스와 요소 6개는 같고 계약만 다르다.
# 입력 = FailureReport(code/stage/detail/attempt) + 복구 예산, 출력 = 전략 라벨 하나.
# None이면 호출자가 지금 쓰던 STRATEGY 표로 폴백한다.

GRASP_FAILED = 4        # arm_interfaces ErrorCode 실값. 테스트가 ROS에 의존하지 않게 숫자로 둔다.
UNREACHABLE = 2
OBJECT_NOT_FOUND = 1
PLANNING_FAILED = 3
UNDEFINED_POSE = 8


def test_strategy_retry():
    # 정상 경로의 기준선. 경로 생성 실패는 OMPL이 확률적이라 재시도가 유효하다.
    llm = FakeLLM('{"strategy": "RETRY"}')
    assert choose_recovery(PLANNING_FAILED, 'PLAN', call_llm=llm) == 'RETRY'


def test_strategy_regrasp():
    # 못 집었으면 물러나 재인지 후 다시 집는다. M5의 주력 복구다.
    llm = FakeLLM('{"strategy": "REGRASP"}')
    assert choose_recovery(GRASP_FAILED, 'GRASP', call_llm=llm) == 'REGRASP'


def test_strategy_rescan():
    # 물체를 못 찾았으면 시야를 비우고 다시 본다.
    llm = FakeLLM('{"strategy": "RESCAN"}')
    assert choose_recovery(OBJECT_NOT_FOUND, 'PLAN', call_llm=llm) == 'RESCAN'


def test_strategy_abort():
    # 도달 불가에 ABORT는 계약이 정한 답이다. 모델이 맞게 골랐으면 그대로 쓴다.
    llm = FakeLLM('{"strategy": "ABORT"}')
    assert choose_recovery(UNREACHABLE, 'APPROACH', call_llm=llm) == 'ABORT'


def test_strategy_retry_rejected_on_unreachable():
    # 4층. 문법도 화이트리스트도 통과하지만 올바르지 않은 답을 여기서 막는다.
    # 못 닿는 목표는 몇 번을 보내도 못 닿는다 -> None으로 떨어뜨려 표 폴백에 맡긴다.
    llm = FakeLLM('{"strategy": "RETRY"}')
    assert choose_recovery(UNREACHABLE, 'APPROACH', call_llm=llm) is None


def test_strategy_retry_rejected_on_undefined_pose():
    # SRDF는 런타임에 바뀌지 않는다 -> UNREACHABLE과 같은 칸(NO_RETRY_CODES).
    # 이 코드는 에이전트 화이트리스트(POSE_IDS)와 실제 SRDF가 어긋났을 때만 나온다.
    llm = FakeLLM('{"strategy": "RETRY"}')
    assert choose_recovery(UNDEFINED_POSE, 'PLAN', call_llm=llm) is None


def test_strategy_retry_allowed_on_planning_failed():
    # 대조군. NO_RETRY_CODES가 넓어지면 이 테스트가 먼저 깨진다.
    # OMPL은 확률적이라 새 시드로 풀릴 수 있다 - 재시도는 여기서 정당하다.
    llm = FakeLLM('{"strategy": "RETRY"}')
    assert choose_recovery(PLANNING_FAILED, 'PLAN', call_llm=llm) == 'RETRY'


def test_strategy_normalized():
    # 대소문자와 공백은 형식 문제지 의미 문제가 아니다. 정규화해서 통과시킨다.
    # 이걸 거부하면 멀쩡한 판단이 표 폴백으로 떨어져 LLM을 쓰는 의미가 준다.
    llm = FakeLLM('{"strategy": " regrasp "}')
    assert choose_recovery(GRASP_FAILED, 'GRASP', call_llm=llm) == 'REGRASP'


def test_strategy_unknown_label_rejected():
    # 화이트리스트 밖. 계약에 없는 라벨은 실행 고리가 해석할 수 없다.
    llm = FakeLLM('{"strategy": "RESTART"}')
    assert choose_recovery(GRASP_FAILED, 'GRASP', call_llm=llm) is None


def test_strategy_replan_rejected():
    # REPLAN은 전략 이름으로는 존재하지만 몸통이 없어 스키마에서 뺐다.
    # 구현되지 않은 도구를 계약에 올리면 모델이 그걸 고르고 시퀀스가 그냥 멈춘다.
    llm = FakeLLM('{"strategy": "REPLAN"}')
    assert choose_recovery(GRASP_FAILED, 'GRASP', call_llm=llm) is None


def test_strategy_prose_leak_rejected():
    # 산문이 섞이면 JSON 파싱이 깨진다. 문법 강제를 못 쓰는 경로(폴백 모델)를 대비한다.
    llm = FakeLLM('추천드립니다: {"strategy": "REGRASP"}')
    assert choose_recovery(GRASP_FAILED, 'GRASP', call_llm=llm) is None


def test_strategy_empty_response_rejected():
    # Ollama가 죽으면 _safe_call이 예외를 먹고 빈 문자열을 준다.
    # 여기서 None으로 떨어져야 에이전트가 표 폴백으로 계속 산다.
    llm = FakeLLM('')
    assert choose_recovery(GRASP_FAILED, 'GRASP', call_llm=llm) is None


def test_strategy_wrong_field_rejected():
    # 필드 이름이 다르면 값이 맞아도 계약 위반이다. 키를 관대하게 받으면 계약이 흐려진다.
    llm = FakeLLM('{"action": "REGRASP"}')
    assert choose_recovery(GRASP_FAILED, 'GRASP', call_llm=llm) is None


def test_strategy_array_rejected():
    # 전략은 하나만 고른다. 배열은 plan()의 형식이지 이쪽 계약이 아니다.
    llm = FakeLLM('["REGRASP"]')
    assert choose_recovery(GRASP_FAILED, 'GRASP', call_llm=llm) is None


def test_strategy_unreachable_cannot_retry():
    # 문법도 화이트리스트도 통과하지만 올바르지 않은 답. 계획 하네스의 4층에 해당한다.
    # 도달 불가는 몇 번을 해도 도달 불가다. 재시도하면 못 닿는 목표에 세 번 매달린다.
    llm = FakeLLM('{"strategy": "RETRY"}')
    assert choose_recovery(UNREACHABLE, 'APPROACH', call_llm=llm) is None


def test_strategy_budget_exhausted_forces_abort():
    # 복구 예산이 끝났으면 답은 정해져 있다. 무한루프 방지를 모델 판단에 맡기지 않는다.
    # 거부(None)가 아니라 ABORT 강제인 이유는, 표로 떨어져도 같은 답이 나와야 하기 때문이다.
    llm = FakeLLM('{"strategy": "REGRASP"}')
    assert choose_recovery(GRASP_FAILED, 'GRASP', recovery_used=2, max_recovery=2,
                           call_llm=llm) == 'ABORT'


def test_strategy_retry_prompt_recovers():
    # 폴백 사다리 2칸. 거부 사유를 붙여 다시 물으면 고쳐 오는 경우가 실제로 있다.
    # 어제 RESCAN 실행에서 모델이 재프롬프트에 빈 배열로 답한 것이 같은 자리다.
    llm = FakeLLM('{"strategy": "RESTART"}', '{"strategy": "REGRASP"}')
    assert choose_recovery(GRASP_FAILED, 'GRASP', call_llm=llm) == 'REGRASP'
    assert llm.calls == 2


def test_strategy_retry_limited_to_one():
    # 재프롬프트는 정확히 1회. 실패 하나가 모델을 무한히 부르면 복구가 더 비싸진다.
    llm = FakeLLM('깨짐')
    assert choose_recovery(GRASP_FAILED, 'GRASP', call_llm=llm) is None
    assert llm.calls == 2
