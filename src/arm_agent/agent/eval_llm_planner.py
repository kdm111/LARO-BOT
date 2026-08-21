"""모델을 재는 실호출 시험지. test_llm_planner.py와 재는 것이 다르다.

test_llm_planner.py : LLM을 FakeLLM으로 차단하고 파서·검증기·폴백만 잰다. 항상 같은 결과.
이 파일              : 진짜 모델을 부른다. 결과가 매번 다를 수 있어 pass^k로 채점한다.

채점 = 이진. plan()의 반환이 기대와 정확히 같아야 통과다. 부분 점수는 없다.
pass^k = 같은 케이스를 k회 돌려 k회 모두 맞아야 그 케이스 통과.
로봇의 행동은 여러 개 만들어 놓고 고를 수 없다 - 움직이면 끝이라 pass@k가 아니다.

★ 2026-08-15 채점 정정 - 거부가 두 모양이 됐다.
  케이스의 expected=None은 "None을 반환해야 한다"가 아니라 "거부해야 한다"는 뜻이다.
  그 시절엔 거부의 모양이 None 하나뿐이라 둘이 같았는데, []에 '정당한 거부'라는 뜻이
  생기면서 갈라졌다.
    []   = 모델이 스스로 "못한다"고 답했다.
    None = 모델이 아무거나 냈고 검증기가 잡아 문자열 파서로 내려보냈다.
  로봇의 행동은 둘 다 같다(사람에게 되묻고 멈춘다). 그래서 둘 다 통과로 친다 -
  안 그러면 모델이 정직해질수록 점수가 떨어지는 거꾸로 된 시험지가 된다.
  대신 어느 쪽이었는지를 refusal_kind 열에 남긴다. 그게 '안전망이 몇 번 일했나'다.
  ※ eval_results/의 기존 CSV는 옛 채점으로 매긴 것이다. 거부 카테고리(R1~R6)의
    passed 열은 새 규칙과 직접 비교할 수 없다 - run_* 열의 원본 출력으로 다시 세야 한다.

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
    python3 -u -m agent.eval_llm_planner --model llama --category L1 L2   # 다국어 일부
    python3 -u -m agent.eval_llm_planner --model llama --lang ja zh de fr # 언어 일부
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

from .eval_cases import CASES, CASE_SET
from .llm_ollama import make_ollama_caller
from .llm_planner import plan
from .llm_prompts import _build_prompt


def _scene_label(scene):
    """씬을 한 칸에 들어가는 문자열로."""
    if scene is None:
        return 'None'
    if not scene:
        return '[]'
    return ','.join(scene)


def is_refusal(got):
    """plan()이 낸 것이 거부인가. []와 None 둘 다 로봇은 멈추고 사람에게 되묻는다."""
    return got is None or got == []


def matches(got, expected):
    """채점 한 판. expected=None은 '거부해야 한다'는 뜻이지 'None이어야 한다'가 아니다."""
    if expected is None:
        return is_refusal(got)
    return got == expected


def refusal_kind(outputs):
    """거부 케이스가 어느 쪽으로 거부했나. 안전망이 몇 번 일했는지를 본다."""
    kinds = {'모델' if o == [] else '검증기' if o is None else '거부아님'
             for o in outputs}
    if not kinds:
        return ''
    return '+'.join(sorted(kinds))


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
        if matches(got, case.expected):
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
    header = (['model', 'tag', 'case_set', 'category', 'pair', 'lang',
               'command', 'scene', 'expected',
               'k', 'ok_count', 'passed', 'refusal_kind', 'sec_per_call']
              + [f'run_{i}' for i in range(1, k + 1)]
              + ['prompt'])

    with open(path, 'w', newline='', encoding='utf-8-sig') as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for case, passed, ok, outputs, elapsed in rows:
            runs = [_cell(o) for o in outputs]
            runs += [''] * (k - len(runs))          # --fast로 중단된 칸은 빈칸
            writer.writerow(
                [model, tag, CASE_SET, case.category, case.pair, case.lang,
                 case.command, _scene_label(case.scene), _cell(case.expected),
                 k, ok, 'PASS' if passed else 'FAIL',
                 refusal_kind(outputs) if case.expected is None else '',
                 f'{elapsed:.2f}']
                + runs
                + [_build_prompt(case.command, case.scene)])


def run(model, k, fast, categories, languages, tag, out_dir):
    """모델 하나로 시험지를 돌린다. (카테고리별 집계, CSV 경로)를 낸다."""
    call_llm = make_ollama_caller(model)
    cases = [
        case for case in CASES
        if (not categories or case.category in categories)
        and (not languages or case.lang in languages)
    ]

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
                        help='N1 ... R6 또는 L1 ... L6 중 일부만')
    parser.add_argument('--lang', nargs='*',
                        choices=tuple(sorted({case.lang for case in CASES})),
                        help='언어 일부만: de en es fr it ja ko pt ru zh')
    # tag는 프롬프트 판(版)을 가리킨다. 프롬프트를 고치면 반드시 바꿔서 돌릴 것 -
    # 안 그러면 이전 CSV를 덮어써서 A/B 비교 상대가 사라진다.
    parser.add_argument('--tag', default='v3-multilingual',
                        help='프롬프트 판 이름. 파일 이름과 tag 열에 들어간다')
    parser.add_argument('--out-dir', default='eval_results',
                        help='CSV를 쓸 디렉터리')
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    results = {}
    for model in args.model:
        per_category, _ = run(model, args.k, args.fast,
                              args.category, args.lang, args.tag, args.out_dir)
        results[model] = per_category

    if len(results) > 1:
        print_comparison(results, args.k)
    return 0


if __name__ == '__main__':
    sys.exit(main())
