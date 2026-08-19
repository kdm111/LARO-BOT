# 구역 = 사각형 (x0, x1, y0, y1). base_link 기준, +y=왼쪽.
# ★ 진실은 arm_bringup/config/cell_layout.yaml 이다. 여기와 arm_skills의
#   kTargets(params.hpp), scene_*.sdf의 zone_* 판이 그 사본이고, test_cell_layout.py가 넷을 대조한다.
#   하나만 고치면 pytest가 빨개진다.
ZONE = {
    'shelf': (0.070, 0.150, 0.155, 0.215),         # 창고. 08-19 실물에서 하나로 통일
    'work': (0.100, 0.230, -0.040, 0.040),         # 작업 구역. 물체 셋이 들어간다
    'counter': (0.100, 0.180, -0.115, -0.055),     # 카운터 - 주문의 종착지
    'bin': (0.100, 0.180, -0.205, -0.145),         # 수거함 - 불량품. 카운터에서 3cm 바깥
}
LOITER_SEC = 2.0
# 물체별 정리 목적지. 물체가 늘면 여기 한 줄이 는다.
DEST = {
    'red_block': 'shelf',
    'blue_ring': 'shelf',
    'green_block': 'bin',   # 불량품은 창고로 되돌리지 않는다. 버린다.
}


def zone_of(x, y):
    """좌표가 들어 있는 구역을 돌려준다. 어디에도 안 들어가면 None.

    최근접 판정을 버리고 사각형 포함으로 바꿨다 - 구역이 커지면서
    "중심에서 얼마나 가까운가"가 "어느 구역 안인가"와 어긋났다.
    구역끼리 겹치지 않는 것은 test_cell_layout.py가 보장한다.
    """
    for name, (x0, x1, y0, y1) in ZONE.items():
        if x0 <= x <= x1 and y0 <= y <= y1:
            return name
    return None


def clean_steps(object_id):
    """방치된 물건을 제 창고로 되돌리는 명령을 생성한다."""
    return [
        {'skill': 'pick', 'object_id': object_id},
        {'skill': 'place', 'object_id': object_id, 'target_id': DEST[object_id]}
    ]


def placed_in(object_id, target_id, first_seen):
    """물체가 목적지에 실제로 서 있는가.

    skill이 성공을 돌려줘도 운반 중 떨어뜨리면 실패한다.
    씬이 진실이고 액션 결과는 주장이다.
    """
    if target_id is None:
        return True  # move_to만 있는 시퀀스 - 검증할 물체가 없다.

    seen = first_seen.get(object_id)
    if seen is None:
        return False  # 씬에서 사라짐. 떨어뜨림
    return seen[0] == target_id
