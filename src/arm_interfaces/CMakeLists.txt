# 작업 지시서
# 실제로 만들어 내느 경로
# 패키지가 설치된 경로를 찾아 헤더, 라이브러리 위치를 변수에 담아라.



# 1단계 : 정규화 : .msg, .action 파일을 변환한다.
# 2단계 : 액션 해체 : 액션은 사실 토픽과 서비스를 조립해서 만든 것이다.
# action의 msg를 보고 SendGaol, GetResult, FeedbackMessage, CancelGoal이 생성된다.
# 그래서 액션을 띄우면 없던 토픽과 서비스가 보인다.
# 3단계 : 코드 생성 :  rosidl_type
cmake_minimum_required(VERSION 3.8)
project(arm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate C++/Python types for these interfaces. Both the agent (client) and
# motion (server) depend on this package, so the contract lives here once.
rosidl_generate_interfaces(${PROJECT_NAME}
  # 메시지
  "msg/DetectedObject.msg"
  "msg/SceneState.msg"
  "msg/ErrorCode.msg"
  "msg/FailureReport.msg"
  "msg/Stage.msg"
  "msg/RobotStatus.msg"
  # 액션
  "action/Pick.action"
  "action/Place.action"
  "action/MoveTo.action"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
