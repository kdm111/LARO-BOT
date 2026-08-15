# RobotStatus : 로봇이 지금 무얼 하고 있는지 밖으로 알리는 메시지

std_msgs/Header header  # stamp = 상태를 찍은 시각
string state  # IDLE, RUNNING, RECOVERING, ABORTED_WAIT
string source  # 지금 일을 시킨 주체
string[] ignored # 정리를 포기한 물체 사람이 불러야 함.
uint32 served  # 카운터로 내보낸 건수 (사람이 주문함)
uint32 cleaned  # 스스로 치운 건수 (자가 명령)
uint32 discarded  # bin으로 버린 횟수 (사람이 명령함)
uint32 aborted  # 시퀀스를 포기한 횟수
