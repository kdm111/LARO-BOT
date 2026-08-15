# RobotStatus : 로봇이 지금 무얼 하고 있는지 밖으로 알리는 메시지

std_msgs/Header header  # stamp = 상태를 찍은 시각
string state  # IDLE, RUNNING, RECOVERING, ABORTED_WAIT
string source  # 지금 일을 시킨 주체
string[] ignored # 정리를 포기한 물체 사람이 불러야 함.
uint32 served  # 카운터로 내보낸 건수 (사람이 주문함)
uint32 cleaned  # 스스로 치운 건수 (자가 명령)
uint32 discarded  # bin으로 버린 횟수 (사람이 명령함)
uint32 aborted  # 시퀀스를 포기한 횟수
# 구역 점유 : "구역:물체" 목록. 예 ["work:red_block", "shelf_ring:blue_ring"]
# 구역 판정(zone_of)은 agent 안에 있어 밖에서 계산할 수 없다 - 그래서 결과를 실어 보낸다.
# 로봇이 "믿고 있는" 배치라 /scene_state의 좌표와 함께 보면 판정이 맞는지도 대조된다.
string[] zone_objects
