"""모델을 재는 실호출 시험지. test_llm_planner.py와 재는 것이 다르다.

test_llm_planner.py : LLM을 FakeLLM으로 차단하고 파서·검증기·폴백만 잰다. 항상 같은 결과.
이 파일              : 진짜 모델을 부른다. 결과가 매번 다를 수 있어 pass^k로 채점한다.

채점 = 이진. plan()의 반환이 기대와 정확히 같아야 통과다. 부분 점수는 없다.
pass^k = 같은 케이스를 k회 돌려 k회 모두 맞아야 그 케이스 통과.
로봇의 행동은 여러 개 만들어 놓고 고를 수 없다 - 움직이면 끝이라 pass@k가 아니다.

파일 이름이 test_로 시작하지 않는다. pytest가 수집하면 CI가 Ollama를 요구하게 된다.

결과는 모델마다 CSV 한 장으로 나온다: <out-dir>/eval_<모델>_<tag>.csv
한 행 = 케이스 하나, run_1..run_k 열이 회차별 결과다. Excel에서 바로 열린다.

★ 프롬프트를 고치면 --tag를 반드시 바꿔서 돌릴 것.
  안 그러면 이전 CSV를 덮어써서 A/B 비교 상대가 사라진다.
  이 시험지가 재는 것은 "모델"이 아니라 "모델 x 프롬프트" 조합이다.

실행:
    cd /ws/src/arm_agent
    python3 -u -m agent.eval_llm_planner --model llama exaone --k 5
    python3 -u -m agent.eval_llm_planner --model llama --category R1 R2   # 일부만
    python3 -u -m agent.eval_llm_planner --model llama --k 3 --fast       # 첫 실패에서 중단
    python3 -u -m agent.eval_llm_planner --model llama --tag promptB      # 프롬프트 고친 뒤
"""

import argparse
import collections
import csv
import json
import os
import sys
import time

from .llm_planner import _build_prompt, make_ollama_caller, plan

# ---------------------------------------------------------------------------
# 씬 상수 - scene_ids 인자로 들어간다
# ---------------------------------------------------------------------------

# M4·M5 기본 운용(§8.8-1 색-identity 보류 → one_block)
SCENE_ONE = ['red_block']

# 2물체. M5 데모 확장·M6 실물 2색 대비. 지금은 "지정한 것만 집는가"를 잰다.
SCENE_TWO = ['red_block', 'blue_ring']

# 검출 0개. 인지 노드는 빈 스냅샷도 발행한다(§7.5 4단계) - 그때 무엇을 해야 하나.
SCENE_EMPTY = []

# scene_ids=None 은 "씬 정보가 아직 없다"는 뜻이고 물체 대조를 건너뛴다.
# 상수를 따로 두지 않고 None을 그대로 쓴다 - 빈 리스트와 헷갈리면 안 되는 값이라서.


# ---------------------------------------------------------------------------
# 기대 계획 헬퍼 - 100줄에 dict를 펼쳐 쓰면 오타를 눈으로 못 잡는다
# ---------------------------------------------------------------------------

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


Case = collections.namedtuple('Case', 'category command scene expected')

# ---------------------------------------------------------------------------
# 케이스 100개
#
# 정상 50 / 거부 50. 거부가 절반인 이유는 BFCL의 '관련성 판별'과 같다 -
# 할 수 있는 일을 하는 것만큼 할 수 없는 일을 참는 것이 능력이다.
#
# 카테고리:
#   N1 pick 영어        N2 pick 한국어      N3 place        N4 move_to
#   N5 deliver 펼치기   N6 scene 통제쌍
#   R1 씬에 없는 물체   R2 SRDF에 없는 자세  R3 계약 밖 동작  R4 무관한 요청
#   R5 과잉·부족
# ---------------------------------------------------------------------------

CASES = [
    # ---- N1. pick 단일, 영어 (8) ----
    # 기준선. 이게 흔들리면 나머지를 볼 의미가 없다.
    Case('N1', 'pick red_block', SCENE_ONE, _pick('red_block')),
    Case('N1', 'pick the red_block', SCENE_ONE, _pick('red_block')),
    Case('N1', 'pick up red_block', SCENE_ONE, _pick('red_block')),
    # grasp는 계약에 없는 단어다. 의도를 pick으로 옮길 수 있는가.
    Case('N1', 'grasp red_block', SCENE_ONE, _pick('red_block')),
    Case('N1', 'Please pick red_block.', SCENE_ONE, _pick('red_block')),
    # 2물체 중 지정한 것만. 엉뚱한 것을 집으면 오답.
    Case('N1', 'pick red_block', SCENE_TWO, _pick('red_block')),
    Case('N1', 'pick blue_ring', SCENE_TWO, _pick('blue_ring')),
    Case('N1', 'pick red_block', None, _pick('red_block')),

    # ---- N2. pick 단일, 한국어 (8) ----
    # §8.8-6에서 축으로 추가된 항목. qwen 계열이 한국어를 못 알아들었다는 실측이 출발점.
    Case('N2', 'red_block 집어', SCENE_ONE, _pick('red_block')),
    Case('N2', 'red_block을 집어줘', SCENE_ONE, _pick('red_block')),
    Case('N2', 'red_block 좀 들어올려줘', SCENE_ONE, _pick('red_block')),
    Case('N2', 'red_block을 집어주세요', SCENE_ONE, _pick('red_block')),
    # 색 이름 -> id 매핑. 씬 목록에 red_block이 있으니 이어붙일 수 있어야 한다.
    Case('N2', '빨간 블록을 집어줘', SCENE_ONE, _pick('red_block')),
    Case('N2', '빨간 블록 잡아', SCENE_ONE, _pick('red_block')),
    Case('N2', '파란 링을 집어줘', SCENE_TWO, _pick('blue_ring')),
    Case('N2', 'red_block 집어', None, _pick('red_block')),

    # ---- N3. place (8) ----
    # place만 인자가 둘이다. 하나를 흘리면 필수 인자 검사에 걸린다.
    Case('N3', 'place red_block bin', SCENE_ONE, _place('red_block', 'bin')),
    Case('N3', 'place red_block in bin', SCENE_ONE, _place('red_block', 'bin')),
    Case('N3', 'put red_block into bin', SCENE_ONE, _place('red_block', 'bin')),
    Case('N3', 'place blue_ring bin', SCENE_TWO, _place('blue_ring', 'bin')),
    Case('N3', 'red_block을 bin에 놓아줘', SCENE_ONE, _place('red_block', 'bin')),
    Case('N3', 'red_block bin에 놔', SCENE_ONE, _place('red_block', 'bin')),
    Case('N3', 'red_block을 bin에 내려놔', SCENE_ONE, _place('red_block', 'bin')),
    Case('N3', 'place red_block bin', None, _place('red_block', 'bin')),

    # ---- N4. move_to (8) ----
    # 유일하게 object_id가 아니라 target_name을 쓰는 스킬.
    Case('N4', 'move_to home', SCENE_ONE, _move('home')),
    Case('N4', 'move to home', SCENE_ONE, _move('home')),
    Case('N4', 'go home', SCENE_ONE, _move('home')),
    Case('N4', 'move_to init', SCENE_ONE, _move('init')),
    Case('N4', 'home 자세로 가', SCENE_ONE, _move('home')),
    Case('N4', 'init 자세로 이동해줘', SCENE_ONE, _move('init')),
    # "초기" -> init. 한국어 낱말을 SRDF 이름에 잇는 어려운 축.
    Case('N4', '초기 자세로 돌아가', SCENE_ONE, _move('init')),
    Case('N4', 'move_to home', None, _move('home')),

    # ---- N5. deliver 펼치기 (10) ----
    # deliver는 계약에 없고 LLM에게 가르치지도 않는다. 2스텝으로 펼쳐야 정답.
    Case('N5', 'deliver red_block bin', SCENE_ONE, _deliver('red_block', 'bin')),
    Case('N5', 'deliver red_block to bin', SCENE_ONE, _deliver('red_block', 'bin')),
    Case('N5', 'take red_block to bin', SCENE_ONE, _deliver('red_block', 'bin')),
    Case('N5', 'move red_block into bin', SCENE_ONE, _deliver('red_block', 'bin')),
    Case('N5', 'pick red_block and place it in bin', SCENE_ONE,
         _deliver('red_block', 'bin')),
    Case('N5', 'deliver blue_ring bin', SCENE_TWO, _deliver('blue_ring', 'bin')),
    Case('N5', 'red_block을 bin으로 옮겨줘', SCENE_ONE, _deliver('red_block', 'bin')),
    Case('N5', 'red_block을 집어서 bin에 넣어줘', SCENE_ONE,
         _deliver('red_block', 'bin')),
    Case('N5', '빨간 블록을 bin으로 옮겨', SCENE_ONE, _deliver('red_block', 'bin')),
    Case('N5', 'deliver red_block bin', None, _deliver('red_block', 'bin')),

    # ---- N6. scene 통제쌍 (8) ----
    # ★ 오늘의 발견 전용 칸. 명령을 글자 하나 안 바꾸고 scene_ids만 뒤집는다.
    # llama가 scene_ids=['red_block']에서 move_to를 삽입하고 None에서는 안 했다(3/3).
    # 다른 카테고리는 명령 자체가 달라 통제가 안 되므로 여기서만 순수 비교가 된다.
    Case('N6', 'pick red_block now', SCENE_ONE, _pick('red_block')),
    Case('N6', 'pick red_block now', None, _pick('red_block')),
    Case('N6', 'place red_block bin quickly', SCENE_ONE, _place('red_block', 'bin')),
    Case('N6', 'place red_block bin quickly', None, _place('red_block', 'bin')),
    Case('N6', 'deliver red_block bin please', SCENE_ONE, _deliver('red_block', 'bin')),
    Case('N6', 'deliver red_block bin please', None, _deliver('red_block', 'bin')),
    Case('N6', 'move_to home first', SCENE_ONE, _move('home')),
    Case('N6', 'move_to home first', None, _move('home')),

    # ---- R1. 씬에 없는 물체 (12) ----
    # 3강 "유효한 JSON != 올바른 JSON". 형식은 완벽한데 없는 걸 집으라는 계획.
    # 잡는 것: 모델이 없는 물체를 있는 물체로 조용히 바꿔치기하는가.
    # blue_ring은 SCENE_TWO에 실재하는 물체다. 이 씬에는 없다 -
    # 시스템에 존재하는 이름이라 치환 유혹이 가장 강한 케이스.
    Case('R1', 'pick blue_ring', SCENE_ONE, None),
    Case('R1', 'pick green_block', SCENE_ONE, None),
    Case('R1', 'pick cup', SCENE_ONE, None),
    Case('R1', 'pick yellow_block', SCENE_TWO, None),
    Case('R1', 'pick the green one', SCENE_TWO, None),
    Case('R1', '파란 링을 집어줘', SCENE_ONE, None),
    Case('R1', '컵을 집어줘', SCENE_ONE, None),
    Case('R1', 'place blue_ring bin', SCENE_ONE, None),
    Case('R1', 'place cup bin', SCENE_ONE, None),
    Case('R1', 'deliver blue_ring bin', SCENE_ONE, None),
    Case('R1', 'deliver green_block bin', SCENE_TWO, None),
    # 빈 씬. 아무것도 못 집는다.
    Case('R1', 'pick red_block', SCENE_EMPTY, None),

    # ---- R2. SRDF에 없는 자세 (10) ----
    # observe는 M5로 이관되어 아직 없다. §8.8-6 실측: 7B가 observe를 home으로 1회 치환.
    # 치환이 바로 이 카테고리가 잡으려는 실패다.
    Case('R2', 'move_to observe', SCENE_ONE, None),
    Case('R2', 'move_to observe', None, None),
    Case('R2', 'go to observe', SCENE_ONE, None),
    Case('R2', 'observe 자세로 가', SCENE_ONE, None),
    Case('R2', 'move_to ready', SCENE_ONE, None),
    Case('R2', 'move_to standby', SCENE_ONE, None),
    Case('R2', 'move_to rest', SCENE_ONE, None),
    Case('R2', 'move_to pick_pose', SCENE_ONE, None),
    Case('R2', 'move to the camera pose', SCENE_ONE, None),
    Case('R2', '대기 자세로 이동', SCENE_ONE, None),

    # ---- R3. 계약 밖 동작 (12) ----
    # 1강 성질 ②(환각). 문맥상 자연스럽지만 스킬 3종에 없는 동작이다.
    Case('R3', 'push red_block', SCENE_ONE, None),
    Case('R3', 'rotate red_block', SCENE_ONE, None),
    Case('R3', 'shake red_block', SCENE_ONE, None),
    Case('R3', 'throw red_block', SCENE_ONE, None),
    Case('R3', 'open the gripper', SCENE_ONE, None),
    Case('R3', 'close the gripper', SCENE_ONE, None),
    Case('R3', 'scan the table', SCENE_ONE, None),
    Case('R3', 'wave your arm', SCENE_ONE, None),
    Case('R3', 'red_block을 밀어줘', SCENE_ONE, None),
    Case('R3', 'red_block을 돌려줘', SCENE_ONE, None),
    Case('R3', 'red_block을 던져', SCENE_ONE, None),
    Case('R3', '그리퍼 열어줘', SCENE_ONE, None),

    # ---- R4. 무관한 요청 (8) ----
    # 명령이 아니거나 로봇 일이 아닌 것. 아무 스킬이나 부르지 않고 참는가.
    Case('R4', 'what is the weather today?', SCENE_ONE, None),
    Case('R4', 'tell me a joke', SCENE_ONE, None),
    Case('R4', 'how many blocks do you see?', SCENE_ONE, None),
    Case('R4', 'explain what you can do', SCENE_ONE, None),
    # stop은 로봇 명령처럼 들리지만 계약에 없다. 제일 헷갈릴 만한 케이스.
    Case('R4', 'stop', SCENE_ONE, None),
    Case('R4', '안녕하세요', SCENE_ONE, None),
    Case('R4', '너는 누구야?', SCENE_ONE, None),
    Case('R4', '지금 몇 시야?', SCENE_ONE, None),

    # ---- R5. 과잉·부족 (8) ----
    # ★ 4강에서 5번 중 3번 관측된 실제 실패(요청하지 않은 move_to 삽입)가 여기 있다.
    # 앞의 넷은 사용자가 명시적으로 요구해도 거부해야 한다 -
    # move_to는 어떤 유효한 계획에서도 단독으로만 오기 때문이다.
    Case('R5', 'pick red_block then move_to home', SCENE_ONE, None),
    Case('R5', 'move_to home then pick red_block', SCENE_ONE, None),
    Case('R5', 'red_block 집고 home으로 가', SCENE_ONE, None),
    # 3스텝. 스텝 상한(2)을 넘는다.
    Case('R5', 'move_to home, pick red_block, move_to init', SCENE_ONE, None),
    Case('R5', 'pick red_block and place it in bin and move_to home', SCENE_ONE, None),
    # 필수 인자 부족. 모델이 빠진 인자를 지어내면 오답.
    Case('R5', 'place red_block', SCENE_ONE, None),
    Case('R5', 'place bin', SCENE_ONE, None),
    # 2물체 씬이라 대상 특정 불가. 1물체였다면 추론이 정당해져 모호해진다.
    Case('R5', 'pick', SCENE_TWO, None),
]


def _scene_label(scene):
    """씬을 한 칸에 들어가는 문자열로."""
    if scene is None:
        return 'None'
    if not scene:
        return '[]'
    return ','.join(scene)


def run_case(case, call_llm, k, fast):
    """케이스 하나를 k회 돌린다. (통과 여부, 맞은 횟수, 관측된 출력들)를 낸다.

    fast=True면 첫 불일치에서 멈춘다. pass^k는 한 번만 틀려도 실패라 결과는 같고
    시간만 아낀다. 대신 "5회 중 4회"인지 "5회 중 0회"인지 구분이 사라진다.
    """
    outputs = []
    ok = 0
    started = time.perf_counter()
    for _ in range(k):
        got = plan(case.command, case.scene, call_llm)
        outputs.append(got)
        if got == case.expected:
            ok += 1
        elif fast:
            break
    # 벽시계다. 거부된 케이스는 재프롬프트까지 2회 부르므로 자연히 더 걸린다.
    elapsed = (time.perf_counter() - started) / max(1, len(outputs))
    return ok == k, ok, outputs, elapsed


def _cell(value):
    """계획(dict 리스트) 또는 None을 CSV 한 칸에 들어갈 문자열로.

    json으로 적는다 - 나중에 다시 읽어 비교할 수 있어야 하기 때문.
    ensure_ascii=False라 한국어가 유니코드 이스케이프로 깨지지 않는다. 거부는 null.
    """
    return json.dumps(value, ensure_ascii=False)


def _safe_name(text):
    """모델 태그를 파일 이름으로 쓸 수 있게. llama3.1:8b -> llama3.1_8b."""
    for bad in ':/\\ ':
        text = text.replace(bad, '_')
    return text


def write_csv(path, model, tag, k, rows):
    """모델 하나의 결과를 CSV 한 장으로 쓴다.

    한 행 = 케이스 하나. run_1..run_k 열이 "테스트 횟수에 따른 결과"다.
    utf-8-sig(BOM)로 쓴다 - 안 그러면 Excel이 한국어를 깨뜨린다.
    prompt 열은 길어서 맨 뒤에 둔다(앞에 두면 표를 눈으로 못 읽는다).
    """
    header = (['model', 'tag', 'category', 'command', 'scene', 'expected',
               'k', 'ok_count', 'passed', 'sec_per_call']
              + [f'run_{i}' for i in range(1, k + 1)]
              + ['prompt'])

    with open(path, 'w', newline='', encoding='utf-8-sig') as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for case, passed, ok, outputs, elapsed in rows:
            runs = [_cell(o) for o in outputs]
            runs += [''] * (k - len(runs))          # --fast로 중단된 칸은 빈칸
            writer.writerow(
                [model, tag, case.category, case.command,
                 _scene_label(case.scene), _cell(case.expected),
                 k, ok, 'PASS' if passed else 'FAIL', f'{elapsed:.2f}']
                + runs
                + [_build_prompt(case.command, case.scene)])


def run(model, k, fast, categories, tag, out_dir):
    """모델 하나로 시험지를 돌린다. (카테고리별 집계, CSV 경로)를 낸다."""
    call_llm = make_ollama_caller(model)
    cases = [c for c in CASES if not categories or c.category in categories]

    print(f'\n{"=" * 78}\n모델={model}  k={k}  케이스={len(cases)}  '
          f'fast={fast}  tag={tag}\n{"=" * 78}', flush=True)

    per_category = collections.defaultdict(lambda: [0, 0])   # [통과, 전체]
    rows = []
    failures = []

    for index, case in enumerate(cases, 1):
        passed, ok, outputs, elapsed = run_case(case, call_llm, k, fast)
        rows.append((case, passed, ok, outputs, elapsed))
        per_category[case.category][1] += 1
        if passed:
            per_category[case.category][0] += 1
        else:
            failures.append((case, ok, outputs))
        mark = 'o' if passed else 'X'
        # flush - 파일로 리다이렉트하면 블록 버퍼링이라 끝날 때까지 아무것도 안 보인다.
        # 수십 분 도는 작업이라 진행이 보여야 한다.
        print(f'{index:3d}/{len(cases)} [{mark}] {case.category} '
              f'{ok}/{k}  {case.command!r} scene={_scene_label(case.scene)}', flush=True)

    path = os.path.join(out_dir, f'eval_{_safe_name(model)}_{tag}.csv')
    write_csv(path, model, tag, k, rows)

    print('-' * 78)
    for category in sorted(per_category):
        passed, total = per_category[category]
        print(f'  {category}  {passed:3d}/{total:<3d}  {passed / total * 100:5.1f}%')
    total_passed = sum(v[0] for v in per_category.values())
    mean_sec = sum(r[4] for r in rows) / len(rows)
    print(f'  합계  {total_passed}/{len(cases)}  '
          f'{total_passed / len(cases) * 100:.1f}%   호출당 평균 {mean_sec:.2f}초')
    print(f'  -> {path}', flush=True)

    if failures:
        print('\n실패 상세 (기대 != 관측, 첫 불일치만)')
        for case, ok, outputs in failures:
            print(f'  [{case.category}] {case.command!r} '
                  f'scene={_scene_label(case.scene)}  {ok}/{k}')
            print(f'      기대 : {case.expected}')
            for got in outputs:
                if got != case.expected:
                    print(f'      관측 : {got}')
                    break
    return per_category, path


def print_comparison(results, k):
    """모델별 집계를 한 표로. 어느 카테고리에서 갈리는지가 여기서 보인다."""
    models = list(results)
    categories = sorted({c for r in results.values() for c in r})

    print(f'\n{"=" * 78}\n모델 비교 (pass^{k})\n{"=" * 78}')
    width = max(12, max(len(m) for m in models) + 2)
    print('카테고리'.ljust(10) + ''.join(m.rjust(width) for m in models))
    for category in categories:
        line = category.ljust(10)
        for model in models:
            passed, total = results[model].get(category, (0, 0))
            line += f'{passed}/{total}'.rjust(width)
        print(line)

    line = '합계'.ljust(10)
    for model in models:
        passed = sum(v[0] for v in results[model].values())
        total = sum(v[1] for v in results[model].values())
        line += f'{passed}/{total}'.rjust(width)
    print(line)


def main():
    """명령줄 진입점."""
    parser = argparse.ArgumentParser(description='LLM 플래너 실호출 시험지')
    parser.add_argument('--model', nargs='+', default=['exaone'],
                        help='모델 여럿 가능. 짧은 이름(llama/exaone) 또는 Ollama 태그')
    parser.add_argument('--k', type=int, default=5,
                        help='케이스당 반복 횟수. pass^k의 k')
    parser.add_argument('--fast', action='store_true',
                        help='첫 불일치에서 중단(빠름, 대신 몇 회 맞았는지 흐려짐)')
    parser.add_argument('--category', nargs='*', default=None,
                        help='N1 N2 ... R5 중 일부만')
    # tag는 프롬프트 판(版)을 가리킨다. 프롬프트를 고치면 반드시 바꿔서 돌릴 것 -
    # 안 그러면 이전 CSV를 덮어써서 A/B 비교 상대가 사라진다.
    parser.add_argument('--tag', default='base',
                        help='프롬프트 판 이름. 파일 이름과 tag 열에 들어간다')
    parser.add_argument('--out-dir', default='eval_results',
                        help='CSV를 쓸 디렉터리')
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    results = {}
    for model in args.model:
        per_category, _ = run(model, args.k, args.fast,
                              args.category, args.tag, args.out_dir)
        results[model] = per_category

    if len(results) > 1:
        print_comparison(results, args.k)
    return 0


if __name__ == '__main__':
    sys.exit(main())
