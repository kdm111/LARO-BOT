"""10개 언어 LLM 실호출 시험지의 케이스 정의.

이 파일은 번역 문자열을 pytest로 통과시키기 위한 테스트가 아니다.
모델의 언어 이해는 실제 모델을 호출해야만 측정할 수 있다.
여기서는 같은 의도·씬·기댓값을 10개 언어로 고정해 언어만 바뀐
통제쌍을 만든다.

구성은 언어당 6케이스다.
  - 정상 4개: red pick, 2물체 중 blue pick, green 불량품 분류, home 복귀
  - 거부 2개: 씬에 없는 yellow, 계약 밖 push

실행은 ``python3 -m agent.eval_multilingual``을 사용한다. 같은 케이스를 반복해
pass^k로 측정하며, 결과 CSV에는 언어별 성공률과 모델 원본 출력이 남는다.
"""

import collections


CASE_SET = 'v1-multilingual-10x6'

# ISO 639-1 언어 코드. 순서는 CSV와 터미널 보고서에 그대로 사용한다.
LANGUAGES = {
    'ko': '한국어',
    'en': 'English',
    'ja': '日本語',
    'zh': '简体中文',
    'de': 'Deutsch',
    'fr': 'Français',
    'es': 'Español',
    'it': 'Italiano',
    'pt': 'Português',
    'ru': 'Русский',
}

SCENE_RED = ['red_block']
SCENE_RED_BLUE = ['red_block', 'blue_ring']
SCENE_RED_GREEN = ['red_block', 'green_block']


def _pick(object_id):
    """한 스텝 pick 기대 계획."""
    return [{'skill': 'pick', 'object_id': object_id}]


def _move(pose_id):
    """한 스텝 move_to 기대 계획."""
    return [{'skill': 'move_to', 'pose_id': pose_id}]


def _deliver(object_id, target_id):
    """운반 명령의 기대 계획: pick 뒤 place."""
    return [
        {'skill': 'pick', 'object_id': object_id},
        {'skill': 'place', 'object_id': object_id, 'target_id': target_id},
    ]


Scenario = collections.namedtuple(
    'Scenario', 'key category commands scene expected note')


SCENARIOS = [
    Scenario(
        'pick_red',
        'L1',
        {
            'ko': '빨간 블록을 집어줘.',
            'en': 'Pick up the red block.',
            'ja': '赤いブロックを持ち上げてください。',
            'zh': '请拿起红色积木。',
            'de': 'Hebe bitte den roten Block auf.',
            'fr': 'Prends le bloc rouge, s’il te plaît.',
            'es': 'Recoge el bloque rojo, por favor.',
            'it': 'Raccogli il blocco rosso, per favore.',
            'pt': 'Pegue o bloco vermelho, por favor.',
            'ru': 'Подними красный блок, пожалуйста.',
        },
        SCENE_RED,
        _pick('red_block'),
        '자연어 색 이름을 scene의 red_block ID로 정규화하는가',
    ),
    Scenario(
        'pick_blue_ring',
        'L2',
        {
            'ko': '파란 링을 집어줘.',
            'en': 'Pick up the blue ring.',
            'ja': '青いリングを持ち上げてください。',
            'zh': '请拿起蓝色圆环。',
            'de': 'Hebe bitte den blauen Ring auf.',
            'fr': 'Prends l’anneau bleu, s’il te plaît.',
            'es': 'Recoge el anillo azul, por favor.',
            'it': 'Raccogli l’anello blu, per favore.',
            'pt': 'Pegue o anel azul, por favor.',
            'ru': 'Подними синее кольцо, пожалуйста.',
        },
        SCENE_RED_BLUE,
        _pick('blue_ring'),
        '2물체 씬에서 언어와 무관하게 blue_ring을 정확히 고르는가',
    ),
    Scenario(
        'sort_green_reject',
        'L3',
        {
            'ko': '초록 블록을 집어서 불량품 수거함에 넣어줘.',
            'en': 'Pick up the green block and place it in the reject bin.',
            'ja': (
                '緑のブロックを持ち上げて、'
                '不良品用の箱に入れてください。'
            ),
            'zh': '请拿起绿色积木，并把它放进次品箱。',
            'de': 'Hebe den grünen Block auf und lege ihn in den Ausschussbehälter.',
            'fr': 'Prends le bloc vert et dépose-le dans le bac de rebut.',
            'es': 'Recoge el bloque verde y colócalo en el contenedor de rechazo.',
            'it': 'Raccogli il blocco verde e mettilo nel contenitore degli scarti.',
            'pt': 'Pegue o bloco verde e coloque-o no recipiente de rejeitos.',
            'ru': (
                'Подними зелёный блок и положи его '
                'в контейнер для брака.'
            ),
        },
        SCENE_RED_GREEN,
        _deliver('green_block', 'bin'),
        '초록 불량품을 pick+place로 펼치고 목적지를 bin으로 정규화하는가',
    ),
    Scenario(
        'return_home',
        'L4',
        {
            'ko': 'home 자세로 돌아가 줘.',
            'en': 'Return to the home pose.',
            'ja': 'home 姿勢に戻ってください。',
            'zh': '请返回 home 姿势。',
            'de': 'Fahre bitte in die Home-Position zurück.',
            'fr': 'Reviens à la position home.',
            'es': 'Vuelve a la posición home.',
            'it': 'Torna alla posizione home.',
            'pt': 'Volte para a posição home.',
            'ru': 'Вернись в положение home.',
        },
        SCENE_RED,
        _move('home'),
        '동작 표현만 번역하고 SRDF 고유명 home은 보존하는가',
    ),
    Scenario(
        'reject_missing_yellow',
        'L5',
        {
            'ko': '노란 블록을 집어줘.',
            'en': 'Pick up the yellow block.',
            'ja': '黄色いブロックを持ち上げてください。',
            'zh': '请拿起黄色积木。',
            'de': 'Hebe bitte den gelben Block auf.',
            'fr': 'Prends le bloc jaune, s’il te plaît.',
            'es': 'Recoge el bloque amarillo, por favor.',
            'it': 'Raccogli il blocco giallo, per favore.',
            'pt': 'Pegue o bloco amarelo, por favor.',
            'ru': 'Подними жёлтый блок, пожалуйста.',
        },
        SCENE_RED,
        None,
        '씬에 없는 색을 red_block으로 바꿔치기하지 않고 거부하는가',
    ),
    Scenario(
        'reject_push',
        'L6',
        {
            'ko': '빨간 블록을 밀어줘.',
            'en': 'Push the red block.',
            'ja': '赤いブロックを押してください。',
            'zh': '请推动红色积木。',
            'de': 'Schiebe bitte den roten Block.',
            'fr': 'Pousse le bloc rouge, s’il te plaît.',
            'es': 'Empuja el bloque rojo, por favor.',
            'it': 'Spingi il blocco rosso, per favore.',
            'pt': 'Empurre o bloco vermelho, por favor.',
            'ru': 'Толкни красный блок, пожалуйста.',
        },
        SCENE_RED,
        None,
        '계약 밖 push를 pick으로 축소하지 않고 거부하는가',
    ),
]


Case = collections.namedtuple(
    'Case', 'no scenario category lang language command scene expected note')


def _expand(scenarios):
    """시나리오를 언어별 케이스로 펼친다."""
    cases = []
    for scenario in scenarios:
        for lang, language in LANGUAGES.items():
            cases.append(Case(
                len(cases) + 1,
                scenario.key,
                scenario.category,
                lang,
                language,
                scenario.commands[lang],
                scenario.scene,
                scenario.expected,
                scenario.note,
            ))
    return cases


CASES = _expand(SCENARIOS)
