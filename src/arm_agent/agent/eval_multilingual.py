"""10개 언어로 LLM 플래너를 실제 호출해 pass^k를 측정한다.

pytest는 번역 이해를 측정하지 않는다. 이 러너만 실제 Ollama 모델을 부른다.

실행 예:
    cd src/arm_agent
    python3 -u -m agent.eval_multilingual --model exaone --k 1
    python3 -u -m agent.eval_multilingual --model exaone --lang ja zh de fr
    python3 -u -m agent.eval_multilingual --model exaone --scenario sort_green_reject
    python3 -u -m agent.eval_multilingual --model exaone llama --k 3 --fast
"""

import argparse
import collections
import csv
import json
import os
import sys

from .eval_llm_planner import refusal_kind, run_case
from .eval_multilingual_cases import CASES, CASE_SET, LANGUAGES, SCENARIOS
from .llm_ollama import make_ollama_caller
from .llm_prompts import _build_prompt


def _safe_name(text):
    """Ollama 태그를 파일 이름에 안전한 문자열로 바꾼다."""
    for bad in ':/\\ ':
        text = text.replace(bad, '_')
    return text


def _scene_label(scene):
    """씬을 CSV 한 칸에 들어가는 문자열로 바꾼다."""
    if scene is None:
        return 'None'
    if not scene:
        return '[]'
    return ','.join(scene)


def _cell(value):
    """계획 또는 None을 손실 없이 JSON 문자열로 바꾼다."""
    return json.dumps(value, ensure_ascii=False)


def write_csv(path, model, tag, k, rows):
    """한 행에 한 언어·시나리오의 기대와 반복 관측값을 기록한다."""
    header = (
        ['model', 'tag', 'case_set', 'case_no', 'scenario', 'category',
         'lang', 'language', 'command', 'scene', 'expected', 'k',
         'ok_count', 'passed', 'refusal_kind', 'sec_per_call']
        + [f'run_{index}' for index in range(1, k + 1)]
        + ['note', 'prompt']
    )
    with open(path, 'w', newline='', encoding='utf-8-sig') as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for case, passed, ok, outputs, elapsed in rows:
            runs = [_cell(output) for output in outputs]
            runs += [''] * (k - len(runs))
            writer.writerow([
                model, tag, CASE_SET, case.no, case.scenario, case.category,
                case.lang, case.language, case.command, _scene_label(case.scene),
                _cell(case.expected), k, ok, 'PASS' if passed else 'FAIL',
                refusal_kind(outputs) if case.expected is None else '',
                f'{elapsed:.2f}',
            ] + runs + [case.note, _build_prompt(case.command, case.scene)])


def _print_summary(rows):
    """언어별·시나리오별 성공률을 출력한다."""
    by_lang = collections.defaultdict(lambda: [0, 0])
    by_scenario = collections.defaultdict(lambda: [0, 0])
    for case, passed, _, _, _ in rows:
        by_lang[case.lang][1] += 1
        by_scenario[case.scenario][1] += 1
        if passed:
            by_lang[case.lang][0] += 1
            by_scenario[case.scenario][0] += 1

    print('\n언어별')
    for lang in LANGUAGES:
        if lang not in by_lang:
            continue
        passed, total = by_lang[lang]
        print(f'  {lang:2s} {LANGUAGES[lang]:10s} {passed:2d}/{total:<2d} '
              f'{passed / total * 100:5.1f}%')

    print('\n시나리오별')
    for scenario in SCENARIOS:
        if scenario.key not in by_scenario:
            continue
        passed, total = by_scenario[scenario.key]
        print(f'  {scenario.key:24s} {passed:2d}/{total:<2d} '
              f'{passed / total * 100:5.1f}%')


def run(model, k, fast, languages, scenarios, tag, out_dir):
    """선택한 모델과 부분집합으로 시험하고 CSV를 만든다."""
    cases = [
        case for case in CASES
        if (not languages or case.lang in languages)
        and (not scenarios or case.scenario in scenarios)
    ]
    call_llm = make_ollama_caller(model)
    rows = []

    print(f'\n모델={model} k={k} 케이스={len(cases)} fast={fast} tag={tag}',
          flush=True)
    for index, case in enumerate(cases, 1):
        passed, ok, outputs, elapsed = run_case(case, call_llm, k, fast)
        rows.append((case, passed, ok, outputs, elapsed))
        mark = 'o' if passed else 'X'
        print(f'{index:3d}/{len(cases)} [{mark}] {case.lang} '
              f'{case.scenario} {ok}/{k} {case.command!r}', flush=True)

    filename = f'eval_{_safe_name(model)}_{tag}_{CASE_SET}.csv'
    path = os.path.join(out_dir, filename)
    write_csv(path, model, tag, k, rows)
    _print_summary(rows)
    total = sum(1 for _, passed, _, _, _ in rows if passed)
    print(f'\n합계 {total}/{len(rows)} '
          f'{total / len(rows) * 100:.1f}% -> {path}', flush=True)
    return rows, path


def main():
    """명령줄 진입점."""
    parser = argparse.ArgumentParser(description='10개 언어 LLM 플래너 실호출 시험')
    parser.add_argument('--model', nargs='+', default=['exaone'],
                        help='짧은 모델 이름 또는 Ollama 태그')
    parser.add_argument('--k', type=int, default=1,
                        help='케이스당 반복 횟수(pass^k). 빠른 점검 기본값은 1')
    parser.add_argument('--fast', action='store_true',
                        help='첫 불일치에서 해당 케이스 반복 중단')
    parser.add_argument('--lang', nargs='*', choices=tuple(LANGUAGES),
                        help='일부 언어만 실행: ko en ja zh de fr es it pt ru')
    parser.add_argument(
        '--scenario', nargs='*', choices=tuple(s.key for s in SCENARIOS),
        help='일부 시나리오만 실행')
    parser.add_argument('--tag', default='multilingual-v1',
                        help='프롬프트 판 이름')
    parser.add_argument('--out-dir', default='eval_results', help='CSV 출력 폴더')
    args = parser.parse_args()

    if args.k < 1:
        parser.error('--k는 1 이상이어야 한다')
    os.makedirs(args.out_dir, exist_ok=True)
    for model in args.model:
        run(model, args.k, args.fast, args.lang, args.scenario,
            args.tag, args.out_dir)
    return 0


if __name__ == '__main__':
    sys.exit(main())
