"""10개 언어 실호출 시험지 자체를 검증하는 결정론 pytest.

여기서는 모델의 번역 능력을 주장하지 않는다.
언어별 문장이 빠짐없이 있고, 통제할 씬과 기대 ROS 2 스킬이 동일하며,
기대 계획이 현재 계약을 통과하는지만 검사한다. 실제 언어 성공률은
eval_multilingual.py가 Ollama를 호출해 측정한다.
"""

import collections

from agent.eval_multilingual_cases import (
    CASES, CASE_SET, LANGUAGES, SCENARIOS,
)
from agent.llm_validate import validate_steps


EXPECTED_LANGUAGES = {'ko', 'en', 'ja', 'zh', 'de', 'fr', 'es', 'it', 'pt', 'ru'}


def test_suite_version_and_size():
    """판 이름과 10언어 x 6시나리오 크기를 고정한다."""
    assert CASE_SET == 'v1-multilingual-10x6'
    assert set(LANGUAGES) == EXPECTED_LANGUAGES
    assert len(SCENARIOS) == 6
    assert len(CASES) == len(LANGUAGES) * len(SCENARIOS) == 60


def test_every_scenario_has_all_ten_languages():
    """번역 하나가 빠져도 조용히 9개 언어 시험지가 되지 않아야 한다."""
    for scenario in SCENARIOS:
        assert set(scenario.commands) == EXPECTED_LANGUAGES
        assert all(command.strip() for command in scenario.commands.values())


def test_each_language_has_the_same_six_scenarios():
    """언어별 난이도가 달라지지 않도록 시나리오 구성을 대조한다."""
    by_lang = collections.defaultdict(set)
    for case in CASES:
        by_lang[case.lang].add(case.scenario)
    expected = {scenario.key for scenario in SCENARIOS}
    assert set(by_lang) == EXPECTED_LANGUAGES
    assert all(scenarios == expected for scenarios in by_lang.values())


def test_commands_are_unique():
    """서로 다른 언어·의도가 같은 문장으로 복사되지 않았는지 잡는다."""
    commands = [case.command for case in CASES]
    assert len(commands) == len(set(commands))


def test_expected_plans_match_current_contract():
    """정상 기댓값이 현재 스킬·ID 계약을 실제로 통과해야 한다."""
    for scenario in SCENARIOS:
        if scenario.expected is None:
            continue
        got, error = validate_steps(scenario.expected, scenario.scene)
        assert error is None
        assert got == scenario.expected


def test_positive_and_refusal_scenarios_are_both_present():
    """다국어 능력과 안전한 거부를 함께 측정한다."""
    positive = [scenario for scenario in SCENARIOS if scenario.expected is not None]
    refusal = [scenario for scenario in SCENARIOS if scenario.expected is None]
    assert len(positive) == 4
    assert len(refusal) == 2


def test_green_defect_scenario_routes_to_bin():
    """새 초록 불량품 시나리오가 전 언어에서 같은 행동을 요구한다."""
    scenario = next(s for s in SCENARIOS if s.key == 'sort_green_reject')
    assert scenario.scene == ['red_block', 'green_block']
    assert scenario.expected == [
        {'skill': 'pick', 'object_id': 'green_block'},
        {'skill': 'place', 'object_id': 'green_block', 'target_id': 'bin'},
    ]


def test_multilingual_cases_are_attached_after_legacy_suite():
    """기존 182개 번호를 보존하고 새 60개가 뒤에 붙어야 한다."""
    from agent.eval_cases import (
        BASE_CASES, CASES as COMBINED_CASES, CASE_SET as COMBINED_CASE_SET,
    )

    assert len(BASE_CASES) == 182
    assert len(COMBINED_CASES) == 242
    assert COMBINED_CASE_SET == 'v3-multilingual-242'
    assert COMBINED_CASES[:182] == BASE_CASES
    assert [case.no for case in COMBINED_CASES] == list(range(1, 243))
    assert {case.category for case in COMBINED_CASES[182:]} == {
        'L1', 'L2', 'L3', 'L4', 'L5', 'L6',
    }
