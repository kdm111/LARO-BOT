# RobotStatus : 로봇이 지금 무얼 하고 있는지 밖으로 알리는 메시지
std_msgs/Header header  # stamp = 상태를 찍은 시각
string state  # IDLE, RUNNING, RECOVERING
string source  # 지금 일을 시킨 주체
string[] ignored # 정리를 포기한 물체 사람이 불러야 함.