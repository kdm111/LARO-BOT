# 발행되지 않는다.
# ROS2의 메시지 정의 언어네는 enum 타입이 존재하지 않고 int32, string, float64같은 원시타입만 존재한다.
# 따라서 필드는 하나도 없고 상수만 있는 메시지를 사용한다.
# 즉 이파일은 런타임 데이터가 아니라 컴파일 타임 이름공간이다.
int32 SUCCESS=0 # 성공
int32 OBJECT_NOT_FOUND=1 # 물체를 찾을 수 없음
# 역기구학. 포즈를 만드려면 각도를 몇 도로 둬야 하는가? 
# 팔이 만들수 없는 자세라면 각도 제한에 걸리거나
int32 UNREACHABLE=2 # 닿을 수 없는 거리에 있음
# 모션 플래닝
# IK가 목표 관절값을 내놨으니 아무데도 부딪히지 않고 가는 궤적을 찾는다.
# 목표 자체는 도달 가능 가는 길이 막혀있으면 failed 처리 된다.
# 이는 확률형 이라서 시드를 바꾸거나 플래닝 시간을 늘리면 종종 해결된다.
int32 PLANNING_FAILED=3 # 경로 생성을 할 수 없음
 # 그리퍼 닫힘 위치로 판정한 파지 실패 
int32 GRASP_FAILED=4 # 짚을 수 없음.
# 그랩 하러 갔더니 인지 시점 포즈에 물건이 없다. -> RESCAN
int32 OBJECT_MOVED=5
# 운반 중 낙하함 -> REGRASP
int32 GRIPPER_EMPTY=6  # 운반 중 낙하
# 실행 시간 초과
int32 EXECUTION_TIMEOUT=7 # 실행 시간 초과
# 사전에 정의되지 않은 자세
int32 UNDEFINED_POSE=8 # 없는 포즈
int32 INTERNAL_ERROR=99 # 알 수 없는 에러

