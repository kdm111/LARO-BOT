"""실호출 시험지의 케이스 정의(= 테스트 계획서). 러너와 분리한다.

★ 언어 짝(pair) 설계.
케이스를 (영어, 한국어) 쌍으로 정의하고 기계적으로 펼친다. 그래서
  - 한국어 50 : 영어 50 이 설계상 보장된다(v1은 29:71이었다)
  - 같은 의도·같은 씬·같은 기댓값에서 언어만 다르므로 언어 효과만 순수하게 뽑힌다
v1은 명령 자체가 서로 달라 통제가 안 됐다. scene 통제쌍이 쓴 논리를 언어 축에 재적용한 것.

90쌍 = 180케이스. 정상 45쌍 / 거부 45쌍.
거부가 절반인 이유는 BFCL의 관련성 판별과 같다 -
할 수 있는 일을 하는 것만큼 할 수 없는 일을 참는 것이 능력이다.

검토용 CSV 내보내기:
    cd /ws/src/arm_agent
    python3 -m agent.eval_cases --out /ws/eval_results/test_plan.csv
"""

import argparse
import collections
import csv
import json
import os
import sys

# 케이스 집합의 판(版). 바뀌면 이전 CSV와 케이스별 비교가 성립하지 않는다.
CASE_SET = 'v2-paired-182'

# ---- 씬 상수 ----

# M4·M5 기본 운용(§8.8-1 색-identity 보류 -> one_block)
SCENE_ONE = ['red_block']

# 2물체. M6 실물 라인업(빨간 박스 + 블루 도넛)과 이름을 맞춘다.
SCENE_TWO = ['red_block', 'blue_ring']

# 검출 0개. 인지 노드는 빈 스냅샷도 발행한다(§7.5 4단계) - 그때 무엇을 해야 하나.
SCENE_EMPTY = []

# scene_ids=None = "씬 정보가 아직 없다". 물체 대조를 건너뛴다.
# 빈 리스트([])와 뜻이 다르므로 상수를 두지 않고 None을 그대로 쓴다.

CATEGORY_DESC = {
    'N1': '정상 - pick 단일',
    'N2': '정상 - place 단일',
    'N3': '정상 - move_to (이름 자세)',
    'N4': '정상 - deliver를 2스텝으로 펼치기',
    'N5': '정상 - 2물체 중 지정',
    'N6': '정상 - scene_ids 통제쌍',
    'N7': '정상 - 표기 견고성(대소문자/공백/구두점)',
    'R1': '거부 - 씬에 없는 물체',
    'R2': '거부 - SRDF에 없는 자세',
    'R3': '거부 - 계약 밖 동작',
    'R4': '거부 - 명령이 아닌 발화',
    'R5': '거부 - 과잉·부족',
    'R6': '거부 - 빈 씬',
}


# ---- 기대 계획 헬퍼 ----

def _pick(object_id):
    """한 스텝짜리 pick 계획."""
    return [{'skill': 'pick', 'object_id': object_id}]


def _place(object_id, target_id):
    """한 스텝짜리 place 계획."""
    return [{'skill': 'place', 'object_id': object_id, 'target_id': target_id}]


def _move(target_name):
    """move_to 한 스텝."""
    return [{'skill': 'move_to', 'target_name': target_name}]


def _deliver(object_id, target_id):
    """deliver는 계약에 없다. pick+place 2스텝으로 펼쳐져야 정답."""
    return _pick(object_id) + _place(object_id, target_id)


# en/ko = 같은 의도의 두 문장. scene·expected는 공유한다(그래야 통제가 된다).
# note = 이 짝이 무엇을 잡으려는지. 검토용 CSV에 그대로 나간다.
Pair = collections.namedtuple('Pair', 'category en ko scene expected note')

_P = Pair

PAIRS = [
    # ==================== 정상 45쌍 ====================

    # ---- N1. pick 단일 (7쌍) ----
    _P('N1', 'pick red_block', 'red_block 집어', SCENE_ONE, _pick('red_block'),
       '기준선. 이게 흔들리면 나머지를 볼 의미가 없다'),
    _P('N1', 'pick up red_block', 'red_block을 집어줘', SCENE_ONE, _pick('red_block'),
       '동사구(pick up) / 보조용언(집어줘)'),
    _P('N1', 'grasp red_block', 'red_block 좀 들어올려줘', SCENE_ONE, _pick('red_block'),
       '계약에 없는 낱말(grasp/들어올리다)을 pick으로 옮기는가'),
    _P('N1', 'Please pick red_block.', 'red_block을 집어주세요', SCENE_ONE, _pick('red_block'),
       '공손체. 문체가 계획을 바꾸면 안 된다'),
    _P('N1', 'pick the red block', '빨간 블록을 집어줘', SCENE_ONE, _pick('red_block'),
       '★순한글/순영어 - id를 안 주고 색 이름만. 씬 목록을 보고 이어붙여야 한다'),
    _P('N1', 'grab the red one', '빨간 거 집어', SCENE_ONE, _pick('red_block'),
       '★구어체 지시대명사. 씬에 하나뿐이라 지시가 성립한다'),
    _P('N1', 'I want red_block in the gripper', 'red_block을 그리퍼에 물려줘',
       SCENE_ONE, _pick('red_block'),
       '간접 표현. 결과를 말하고 동사를 안 준다'),

    # ---- N2. place 단일 (6쌍) ----
    # ⚠️ 기댓값이 [place] 단독인 것은 계약(문자열 파서 _build_plan)과 맞춘 것이다.
    #    v1에서 모든 모델이 [pick, place]를 냈다 - 무상태 플래너의 미결 과제(계획서 참조).
    _P('N2', 'place red_block bin', 'red_block을 bin에 놓아줘', SCENE_ONE,
       _place('red_block', 'bin'), '기준선. 인자 2개짜리 유일한 스킬'),
    _P('N2', 'place red_block in bin', 'red_block bin에 놔', SCENE_ONE,
       _place('red_block', 'bin'), '전치사 유무 / 조사 생략'),
    _P('N2', 'put red_block into bin', 'red_block을 bin에 내려놔', SCENE_ONE,
       _place('red_block', 'bin'), 'put/내려놓다 - 계약 밖 낱말을 place로'),
    _P('N2', 'put the red block in bin', '빨간 블록을 bin에 놓아줘', SCENE_ONE,
       _place('red_block', 'bin'), '★순한글/순영어 - 색 이름 + place'),
    _P('N2', 'drop red_block in bin', 'red_block을 bin에 떨어뜨려', SCENE_ONE,
       _place('red_block', 'bin'), 'drop/떨어뜨리다 - place로 볼 수 있는 경계 낱말'),
    _P('N2', 'release red_block over bin', 'bin 위에서 red_block을 놓아', SCENE_ONE,
       _place('red_block', 'bin'), '어순을 뒤집었다. 목적어-장소 순서 의존성 확인'),

    # ---- N3. move_to (6쌍) ----
    _P('N3', 'move_to home', 'home 자세로 가', SCENE_ONE, _move('home'),
       '기준선. 유일하게 target_name을 쓰는 스킬'),
    _P('N3', 'move to init', 'init 자세로 이동해줘', SCENE_ONE, _move('init'),
       '밑줄 없는 표기(move to)도 같은 스킬로'),
    _P('N3', 'go home', '홈으로 가', SCENE_ONE, _move('home'),
       '축약 표현'),
    _P('N3', 'return to the initial pose', '초기 자세로 돌아가', SCENE_ONE, _move('init'),
       '★순한글/순영어 - "초기"->init 낱말 연결'),
    _P('N3', 'go back to the resting position', '쉬는 자세로 돌아가', SCENE_ONE, _move('init'),
       '★init을 다른 말로. SRDF 이름을 유추해야 한다(어려운 축)'),
    _P('N3', 'move the arm to home', '팔을 home으로 옮겨', SCENE_ONE, _move('home'),
       '"팔을 옮긴다"가 pick의 목적어처럼 읽힐 수 있다 - 혼동 유도'),

    # ---- N4. deliver 펼치기 (7쌍) ----
    # deliver는 계약에 없고 LLM에게 가르치지도 않는다. 2스텝으로 펼쳐야 정답.
    _P('N4', 'deliver red_block bin', 'red_block을 bin으로 옮겨줘', SCENE_ONE,
       _deliver('red_block', 'bin'), '기준선. 축약어를 2스텝으로'),
    _P('N4', 'deliver red_block to bin', 'red_block을 bin에 가져다 놔', SCENE_ONE,
       _deliver('red_block', 'bin'), '전치사 / 복합동사'),
    _P('N4', 'take red_block to bin', 'red_block 집어서 bin에 넣어줘', SCENE_ONE,
       _deliver('red_block', 'bin'), '한국어 쪽이 2스텝을 명시 - 힌트 강도 차이'),
    _P('N4', 'pick red_block and place it in bin', 'red_block을 집은 다음 bin에 놓아줘',
       SCENE_ONE, _deliver('red_block', 'bin'), '두 스킬을 그대로 말해줌. 가장 쉬운 형태'),
    _P('N4', 'move the red block into bin', '빨간 블록을 bin으로 옮겨', SCENE_ONE,
       _deliver('red_block', 'bin'), '★순한글/순영어 + 2스텝'),
    _P('N4', 'red_block should end up in bin', 'red_block이 bin에 들어가 있어야 해', SCENE_ONE,
       _deliver('red_block', 'bin'), '목표 상태만 말한다. 동사가 없다'),
    _P('N4', 'clear red_block off the table into bin', 'red_block을 치워서 bin에 넣어',
       SCENE_ONE, _deliver('red_block', 'bin'), '"치우다"라는 계약 밖 동사 -> 2스텝'),

    # ---- N5. 2물체 중 지정 (7쌍) ----
    # 엉뚱한 것을 집으면 오답. M6 실물 2색 대비.
    _P('N5', 'pick blue_ring', 'blue_ring 집어', SCENE_TWO, _pick('blue_ring'),
       '2물체 중 지정. 기본형'),
    _P('N5', 'pick red_block', 'red_block 집어', SCENE_TWO, _pick('red_block'),
       '같은 씬에서 다른 물체. 위와 짝을 이룬다'),
    _P('N5', 'pick the blue ring', '파란 링을 집어줘', SCENE_TWO, _pick('blue_ring'),
       '★순한글/순영어 - 색 이름으로 2물체 중 고르기'),
    _P('N5', 'pick the red one, not the blue one', '파란 거 말고 빨간 거 집어', SCENE_TWO,
       _pick('red_block'), '★부정을 포함한 지시. 부정을 놓치면 반대를 집는다'),
    _P('N5', 'deliver blue_ring bin', 'blue_ring을 bin으로 옮겨줘', SCENE_TWO,
       _deliver('blue_ring', 'bin'), '2물체 + 2스텝'),
    _P('N5', 'place blue_ring bin', 'blue_ring을 bin에 놓아줘', SCENE_TWO,
       _place('blue_ring', 'bin'), '2물체 + place'),
    _P('N5', 'move_to home', 'home 자세로 가', SCENE_TWO, _move('home'),
       '물체가 둘이어도 move_to는 물체와 무관해야 한다'),

    # ---- N6. scene_ids 통제쌍 (6쌍 = 3의도 x 2씬) ----
    # ★ 명령을 글자 하나 안 바꾸고 scene_ids만 뒤집는다.
    # v1에서 llama가 scene_ids=['red_block']일 때 move_to를 삽입하고 None일 때는 안 했다(3/3).
    # 언어 짝과 겹쳐 2x2가 된다.
    _P('N6', 'pick red_block now', '지금 red_block 집어', SCENE_ONE, _pick('red_block'),
       '★통제쌍 A - 씬 있음'),
    _P('N6', 'pick red_block now', '지금 red_block 집어', None, _pick('red_block'),
       '★통제쌍 A - 씬 모름(None). 위와 명령이 동일'),
    _P('N6', 'move_to home first', '먼저 home 자세로 가', SCENE_ONE, _move('home'),
       '★통제쌍 B - 씬 있음'),
    _P('N6', 'move_to home first', '먼저 home 자세로 가', None, _move('home'),
       '★통제쌍 B - 씬 모름(None)'),
    _P('N6', 'deliver red_block bin right away', 'red_block을 바로 bin으로 옮겨줘', SCENE_ONE,
       _deliver('red_block', 'bin'), '★통제쌍 C - 씬 있음(2스텝)'),
    _P('N6', 'deliver red_block bin right away', 'red_block을 바로 bin으로 옮겨줘', None,
       _deliver('red_block', 'bin'), '★통제쌍 C - 씬 모름(None)'),

    # ---- N7. 표기 견고성 (6쌍) ----
    # 사람이 실제로 치는 문장은 깔끔하지 않다.
    _P('N7', 'PICK RED_BLOCK', 'RED_BLOCK 집어', SCENE_ONE, _pick('red_block'),
       '대문자. object_id는 소문자로 정규화되어야 한다'),
    _P('N7', 'pick   red_block', 'red_block   집어', SCENE_ONE, _pick('red_block'),
       '공백 중복'),
    _P('N7', '  pick red_block  ', '  red_block 집어  ', SCENE_ONE, _pick('red_block'),
       '앞뒤 공백. agent가 strip하지만 플래너도 견뎌야 한다'),
    _P('N7', 'pick red_block!!!', 'red_block 집어!!!', SCENE_ONE, _pick('red_block'),
       '구두점 반복'),
    _P('N7', 'um, pick red_block please', '어, red_block 좀 집어줘', SCENE_ONE,
       _pick('red_block'), '군말(filler). 실제 음성 입력에서 흔하다'),
    _P('N7', 'pick red_block, thanks', 'red_block 집어, 고마워', SCENE_ONE,
       _pick('red_block'), '인사말이 뒤에 붙었다. 명령 부분만 뽑아내야 한다'),

    # ==================== 거부 45쌍 ====================

    # ---- R1. 씬에 없는 물체 (8쌍) ----
    # 3강 "유효한 JSON != 올바른 JSON". 형식은 완벽한데 없는 걸 집으라는 계획.
    # 잡는 것: 없는 물체를 있는 물체로 조용히 바꿔치기하는가.
    _P('R1', 'pick blue_ring', 'blue_ring 집어', SCENE_ONE, None,
       '★blue_ring은 SCENE_TWO에 실재한다. 시스템에 있는 이름이라 치환 유혹이 가장 강하다'),
    _P('R1', 'pick green_block', 'green_block 집어', SCENE_ONE, None,
       '시스템에 아예 없는 id'),
    _P('R1', 'pick the cup', '컵을 집어줘', SCENE_ONE, None,
       '★순한글/순영어 - 물체 종류 자체가 다르다'),
    _P('R1', 'pick the blue one', '파란 거 집어', SCENE_ONE, None,
       '★색으로 가리키는데 그 색이 씬에 없다. red_block으로 치환하면 오답'),
    _P('R1', 'place blue_ring bin', 'blue_ring을 bin에 놓아줘', SCENE_ONE, None,
       'place 경로에서도 물체 대조가 도는가'),
    _P('R1', 'deliver blue_ring bin', 'blue_ring을 bin으로 옮겨줘', SCENE_ONE, None,
       '2스텝 경로에서도. 첫 스텝만 검사하고 넘어가면 안 된다'),
    _P('R1', 'pick yellow_block', 'yellow_block 집어', SCENE_TWO, None,
       '2물체 씬에서도 없는 것은 없다'),
    _P('R1', 'deliver red_block and blue_ring to bin', 'red_block이랑 blue_ring 둘 다 bin에 넣어',
       SCENE_ONE, None, 'blue_ring이 없다. 하나가 유효해도 전체가 거부되어야 한다'),
    _P('R1', 'pick red_blcok', 'red_blcok 집어', SCENE_ONE, None,
       '★오타. 씬에 없는 id이므로 거부가 정답 - 관대하게 고쳐주면 안 된다'),

    # ---- R2. SRDF에 없는 자세 (7쌍) ----
    # observe는 M5로 이관되어 아직 없다. v1 실측: 모델이 observe를 home으로 치환했다.
    _P('R2', 'move_to observe', 'observe 자세로 가', SCENE_ONE, None,
       '★observe는 M5 이관으로 아직 없다. home으로 치환하면 오답'),
    _P('R2', 'move_to ready', 'ready 자세로 가', SCENE_ONE, None,
       '흔한 자세 이름이라 있을 법하다'),
    _P('R2', 'move_to standby', 'standby 자세로 이동해줘', SCENE_ONE, None,
       '같은 계열'),
    _P('R2', 'go to the camera pose', '카메라 자세로 이동', SCENE_ONE, None,
       '★순한글/순영어 - 이름이 아니라 설명으로 자세를 가리킨다'),
    _P('R2', 'move to the waiting pose', '대기 자세로 이동', SCENE_ONE, None,
       '"대기"가 init처럼 들린다 - 유추로 치환하면 오답'),
    _P('R2', 'move_to bin', 'bin 자세로 가', SCENE_ONE, None,
       '★bin은 유효한 target_id지만 target_name이 아니다. 두 이름공간을 섞는가'),
    _P('R2', 'move_to red_block', 'red_block 자세로 가', SCENE_ONE, None,
       '★물체 id를 자세 이름 자리에. 이름공간 혼동 유도'),

    # ---- R3. 계약 밖 동작 (8쌍) ----
    # 1강 성질 ②(환각). 문맥상 자연스럽지만 스킬 3종에 없는 동작이다.
    _P('R3', 'push red_block', 'red_block을 밀어줘', SCENE_ONE, None,
       '팔로 할 수 있어 보이지만 계약에 없다'),
    _P('R3', 'rotate red_block', 'red_block을 돌려줘', SCENE_ONE, None,
       '같은 계열'),
    _P('R3', 'throw red_block', 'red_block을 던져', SCENE_ONE, None,
       'pick으로 치환되기 쉽다'),
    _P('R3', 'shake red_block', 'red_block을 흔들어줘', SCENE_ONE, None,
       '같은 계열'),
    _P('R3', 'open the gripper', '그리퍼 열어줘', SCENE_ONE, None,
       '★그리퍼는 스킬 안에서만 움직인다. 직접 제어는 계약 밖'),
    _P('R3', 'scan the table', '테이블을 훑어봐', SCENE_ONE, None,
       '인지 노드의 일이지 스킬이 아니다'),
    _P('R3', 'stack red_block on blue_ring', 'red_block을 blue_ring 위에 쌓아', SCENE_TWO, None,
       '★pick+place로 펼치면 그럴듯하지만 target_id가 bin이 아니다'),
    _P('R3', 'stop moving', '멈춰', SCENE_ONE, None,
       '★로봇 명령처럼 들린다. 중단은 액션 취소지 스킬이 아니다'),

    # ---- R4. 명령이 아닌 발화 (7쌍) ----
    # 아무 스킬이나 부르지 않고 참는가. BFCL 관련성 판별과 동형.
    _P('R4', 'hello', '안녕하세요', SCENE_ONE, None,
       '★v1에서 pick red_block이 나왔다. 채점이 아니라 안전 문제'),
    _P('R4', 'who are you?', '너는 누구야?', SCENE_ONE, None,
       '자기소개 요구'),
    _P('R4', 'what is the weather today?', '오늘 날씨 어때?', SCENE_ONE, None,
       '로봇 일이 아니다'),
    _P('R4', 'how many blocks do you see?', '블록이 몇 개 보여?', SCENE_ONE, None,
       '★질문이다. 씬 정보를 묻지 행동을 시키지 않는다'),
    _P('R4', 'explain what you can do', '뭘 할 수 있는지 알려줘', SCENE_ONE, None,
       '능력 설명 요구. 스킬 목록을 아니까 답하고 싶어진다'),
    _P('R4', 'thanks, that was great', '고마워 잘했어', SCENE_ONE, None,
       '칭찬. 직전 성공 뒤에 흔히 온다'),
    _P('R4', '...', 'ㅇㅇ', SCENE_ONE, None,
       '★의미 없는 입력. 뜻이 없으면 아무 스킬도 부르면 안 된다'),

    # ---- R5. 과잉·부족 (9쌍) ----
    # v1에서 가장 얇았던 칸. 4강에서 관측된 실제 실패가 여기 있다.
    _P('R5', 'pick red_block then move_to home', 'red_block 집고 home으로 가', SCENE_ONE, None,
       '★사용자가 명시적으로 요구해도 거부. move_to는 단독으로만 유효하다'),
    _P('R5', 'move_to home then pick red_block', 'home으로 갔다가 red_block 집어',
       SCENE_ONE, None, '순서를 바꿔도 같다'),
    _P('R5', 'move_to home, pick red_block, move_to init',
       'home으로 갔다가 red_block 집고 init으로 가', SCENE_ONE, None,
       '3스텝. 스텝 상한(2)도 함께 걸린다'),
    _P('R5', 'pick red_block and place it in bin and move_to home',
       'red_block을 bin에 넣고 home으로 가', SCENE_ONE, None,
       '유효한 2스텝 뒤에 하나 더. 앞부분이 맞아서 통과시키기 쉽다'),
    _P('R5', 'place red_block', 'red_block을 놔', SCENE_ONE, None,
       '★target_id 누락. 지어내면 오답'),
    _P('R5', 'place bin', 'bin에 놔', SCENE_ONE, None,
       'object_id 누락'),
    _P('R5', 'pick', '집어', SCENE_TWO, None,
       '★2물체라 대상 특정 불가. 1물체였다면 추론이 정당해져 모호해진다'),
    _P('R5', 'move_to', '자세로 가', SCENE_ONE, None,
       'target_name 누락'),
    _P('R5', 'pick red_block twice', 'red_block 두 번 집어', SCENE_ONE, None,
       '★반복 요구. 같은 스텝 2개는 계약상 의미가 없다'),

    # ---- R6. 빈 씬 (6쌍) ----
    # 인지 노드는 검출 0개일 때도 빈 스냅샷을 발행한다(§7.5 4단계).
    # scene_ids=[] 는 "아무것도 없다"이고 None("모른다")과 뜻이 다르다.
    _P('R6', 'pick red_block', 'red_block 집어', SCENE_EMPTY, None,
       '★빈 씬. 아무것도 못 집는다'),
    _P('R6', 'pick anything', '아무거나 집어', SCENE_EMPTY, None,
       '집을 것이 없다'),
    _P('R6', 'place red_block bin', 'red_block을 bin에 놓아줘', SCENE_EMPTY, None,
       'place 경로에서도'),
    _P('R6', 'deliver red_block bin', 'red_block을 bin으로 옮겨줘', SCENE_EMPTY, None,
       '2스텝 경로에서도'),
    _P('R6', 'pick the red block', '빨간 블록을 집어줘', SCENE_EMPTY, None,
       '★순한글/순영어 + 빈 씬'),
    _P('R6', 'is anything on the table?', '테이블에 뭐 있어?', SCENE_EMPTY, None,
       '빈 씬 + 질문. 두 이유로 거부'),
]

# pair = 짝 번호(언어 비교의 조인 키). lang = 'en' | 'ko'. no = 전체 일련번호.
Case = collections.namedtuple('Case', 'no category pair lang command scene expected note')


def _expand(pairs):
    """짝을 케이스로 펼친다. 언어 50:50이 여기서 구조적으로 보장된다."""
    cases = []
    for index, pair in enumerate(pairs, 1):
        for lang, command in (('en', pair.en), ('ko', pair.ko)):
            cases.append(Case(len(cases) + 1, pair.category, index, lang,
                              command, pair.scene, pair.expected, pair.note))
    return cases


CASES = _expand(PAIRS)


def scene_label(scene):
    """씬을 한 칸에 들어가는 문자열로. None(모름)과 []( 없음)을 구분한다."""
    if scene is None:
        return 'None(씬 모름)'
    if not scene:
        return '[](빈 씬)'
    return ','.join(scene)


def readable(expected):
    """기댓값을 사람이 읽는 한 줄로."""
    if expected is None:
        return '거부(None)'
    parts = []
    for step in expected:
        args = [v for key, v in step.items() if key != 'skill']
        parts.append(f'{step["skill"]}({", ".join(args)})')
    return ' + '.join(parts)


def write_plan(path):
    """케이스 정의를 검토용 CSV로 내보낸다. 결과가 아니라 계획서다."""
    header = ['case_no', 'pair_no', 'lang', 'category', 'category_desc', 'kind',
              'command', 'scene', 'expected_readable', 'steps', 'expected_json', 'note']
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, 'w', newline='', encoding='utf-8-sig') as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for case in CASES:
            writer.writerow([
                case.no, case.pair, case.lang, case.category,
                CATEGORY_DESC[case.category],
                '정상' if case.expected is not None else '거부',
                case.command, scene_label(case.scene),
                readable(case.expected),
                0 if case.expected is None else len(case.expected),
                json.dumps(case.expected, ensure_ascii=False), case.note])
    return path


def summarize():
    """구성 점검용 집계. 설계 의도대로 됐는지 눈으로 확인한다."""
    langs = collections.Counter(c.lang for c in CASES)
    kinds = collections.Counter('정상' if c.expected is not None else '거부' for c in CASES)
    print(f'케이스집합 {CASE_SET} : {len(PAIRS)}쌍 -> {len(CASES)}케이스')
    print(f'  언어   en {langs["en"]} : ko {langs["ko"]}')
    print(f'  종류   정상 {kinds["정상"]} : 거부 {kinds["거부"]}')
    print('  카테고리')
    for category, count in sorted(collections.Counter(c.category for c in CASES).items()):
        print(f'    {category}  {count:3d}   {CATEGORY_DESC[category]}')
    scenes = collections.Counter(scene_label(c.scene) for c in CASES)
    print('  씬')
    for name, count in sorted(scenes.items()):
        print(f'    {name:16s} {count:3d}')


def main():
    """명령줄 진입점."""
    parser = argparse.ArgumentParser(description='시험지 케이스 정의 내보내기')
    parser.add_argument('--out', default='eval_results/test_plan.csv',
                        help='내보낼 CSV 경로')
    args = parser.parse_args()
    summarize()
    print(f'\n-> {write_plan(args.out)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
