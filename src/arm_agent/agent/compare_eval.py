"""eval_llm_planner가 뽑은 CSV들을 읽어 모델을 비교한다.

★ 총점 비교는 노이즈에 묻힌다.
100케이스 이항 표본의 표준오차가 약 4~5%p이고, 두 모델의 총점 차이를 볼 때는
두 오차가 합쳐져 약 7%p가 된다 -> 14%p쯤 벌어져야 유의하다. 3강에서 배운
"측정 노이즈보다 작은 차이는 무시한다"를 이제 우리 시험지 자신에게 적용하는 것.

★ 그런데 우리 데이터는 짝(paired)이다 - 같은 100케이스를 두 모델에 똑같이 돌렸다.
그래서 "둘 다 맞음/둘 다 틀림"은 정보가 없고, 갈린 케이스(불일치 쌍)만 보면
공통 난이도가 상쇄되어 훨씬 작은 차이도 잡힌다. 이것이 McNemar의 논리다.

실행:
    cd /ws/src/arm_agent
    python3 -m agent.compare_eval ../../eval_results/*_base.csv
    python3 -m agent.compare_eval --show-cases ../../eval_results/*_base.csv
"""

import argparse
import collections
import csv
import itertools
import sys
import unicodedata

# 정상 카테고리. 여기서 하나라도 크게 무너지면 총점이 좋아도 쓸 수 없다.
NORMAL_CATEGORIES = ('N1', 'N2', 'N3', 'N4', 'N5', 'N6')

# 탈락선. 정상 카테고리 중 하나라도 이 아래면 총점과 무관하게 후보에서 뺀다.
FLOOR = 0.5

# McNemar 카이제곱 임계값(자유도 1). 연속성 보정을 쓴다.
_CHI2 = ((10.83, 'p<0.001'), (6.63, 'p<0.01'), (3.84, 'p<0.05'))

# 불일치 쌍이 이보다 적으면 검정 자체가 의미 없다.
_MIN_DISCORDANT = 10


def _width(text):
    """터미널 표시 폭. 한글은 두 칸을 먹는다."""
    return sum(2 if unicodedata.east_asian_width(c) in 'WF' else 1 for c in text)


def _pad(text, size, right=False):
    """표시 폭 기준으로 채운다. ljust/rjust는 한글에서 어긋난다."""
    fill = ' ' * max(0, size - _width(text))
    return fill + text if right else text + fill


def load(path):
    """CSV 한 장을 읽는다. (모델, 태그, {케이스키: 행})."""
    with open(path, newline='', encoding='utf-8-sig') as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f'빈 CSV: {path}')
    cases = {(r['category'], r['command'], r['scene']): r for r in rows}
    return rows[0]['model'], rows[0]['tag'], cases


def summary(data):
    """모델 x 카테고리 표. 운영 축(초/호출)도 같은 표에 둔다."""
    models = list(data)
    categories = sorted({k[0] for cases in data.values() for k in cases})
    label = max(_width(m) for m in models) + 2

    print('=' * (12 + label * len(models)))
    print('총점 (pass^k)')
    print('=' * (12 + label * len(models)))
    print(_pad('카테고리', 12) + ''.join(_pad(m, label, True) for m in models))

    for category in categories:
        line = _pad(category, 12)
        for model in models:
            rows = [r for k, r in data[model].items() if k[0] == category]
            ok = sum(1 for r in rows if r['passed'] == 'PASS')
            line += _pad(f'{ok}/{len(rows)}', label, True)
        print(line)

    line = _pad('합계', 12)
    for model in models:
        rows = data[model].values()
        ok = sum(1 for r in rows if r['passed'] == 'PASS')
        line += _pad(f'{ok}/{len(rows)}', label, True)
    print(line)

    line = _pad('초/호출', 12)
    for model in models:
        secs = [float(r['sec_per_call']) for r in data[model].values()]
        line += _pad(f'{sum(secs) / len(secs):.2f}', label, True)
    print(line)

    # 탈락선 검사 - 총점이 높아도 정상 경로 하나가 무너지면 못 쓴다.
    print()
    for model in models:
        broken = []
        for category in NORMAL_CATEGORIES:
            rows = [r for k, r in data[model].items() if k[0] == category]
            if not rows:
                continue
            rate = sum(1 for r in rows if r['passed'] == 'PASS') / len(rows)
            if rate < FLOOR:
                broken.append(f'{category}({rate * 100:.0f}%)')
        verdict = f'탈락 - {", ".join(broken)}' if broken else '통과'
        print(f'  탈락선({FLOOR * 100:.0f}%) : {_pad(model, label)} {verdict}')


def pair_compare(data, model_a, model_b, show_cases):
    """짝 비교. 갈린 케이스만 세고 McNemar로 판정한다."""
    cases_a, cases_b = data[model_a], data[model_b]
    shared = sorted(set(cases_a) & set(cases_b))

    both = neither = 0
    only_a, only_b = [], []
    for key in shared:
        pa = cases_a[key]['passed'] == 'PASS'
        pb = cases_b[key]['passed'] == 'PASS'
        if pa and pb:
            both += 1
        elif not pa and not pb:
            neither += 1
        elif pa:
            only_a.append(key)
        else:
            only_b.append(key)

    print(f'\n{"-" * 70}\n짝 비교 : {model_a}  vs  {model_b}   (공통 {len(shared)}케이스)\n'
          f'{"-" * 70}')
    print(f'  둘 다 통과      {both:3d}   <- 정보 없음(공통 난이도)')
    print(f'  둘 다 실패      {neither:3d}   <- 정보 없음')
    print(f'  {_pad(model_a, 12)}만 통과  {len(only_a):3d}   <- 불일치')
    print(f'  {_pad(model_b, 12)}만 통과  {len(only_b):3d}   <- 불일치')

    b_count, c_count = len(only_a), len(only_b)
    total = b_count + c_count
    if total < _MIN_DISCORDANT:
        print(f'\n  판정 : 불일치 {total}건뿐 - 표본 부족, 동률로 읽는다')
    else:
        chi2 = (abs(b_count - c_count) - 1) ** 2 / total
        level = next((s for t, s in _CHI2 if chi2 > t), None)
        winner = model_a if b_count > c_count else model_b
        if level:
            print(f'\n  판정 : {winner} 우세 '
                  f'(McNemar chi2={chi2:.1f}, {level})')
        else:
            print(f'\n  판정 : 동률 (McNemar chi2={chi2:.1f}, 유의하지 않음)')

    # 불일치가 어느 카테고리에 몰렸는지 - 이게 결정의 실제 근거가 된다.
    for name, keys in ((model_a, only_a), (model_b, only_b)):
        if not keys:
            continue
        counts = collections.Counter(k[0] for k in keys)
        spread = ' '.join(f'{c}:{n}' for c, n in sorted(counts.items()))
        print(f'    {name}만 통과 분포  {spread}')
        if show_cases:
            for key in keys:
                print(f'      [{key[0]}] {key[1]!r} scene={key[2]}')


def main():
    """명령줄 진입점."""
    parser = argparse.ArgumentParser(description='eval CSV 비교')
    parser.add_argument('csv', nargs='+', help='eval_*.csv 경로들')
    parser.add_argument('--show-cases', action='store_true',
                        help='불일치 케이스를 하나씩 나열')
    args = parser.parse_args()

    data, tags = {}, set()
    for path in args.csv:
        model, tag, cases = load(path)
        data[model] = cases
        tags.add(tag)

    if len(tags) > 1:
        print(f'⚠️  태그가 섞여 있다: {sorted(tags)} - '
              f'다른 프롬프트끼리 비교하는 중일 수 있다\n')

    summary(data)
    for model_a, model_b in itertools.combinations(data, 2):
        pair_compare(data, model_a, model_b, args.show_cases)
    return 0


if __name__ == '__main__':
    sys.exit(main())
