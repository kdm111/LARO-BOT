#include "arm_skills/skill_server.hpp"

SkillServer::SkillServer()
: Node("skill_server")
{
  // ★ 2026-08-22 실행 watchdog 예산(초). 파라미터인 이유는 검증 가능성이다 -
  //   값을 1초로 줄이면 정상 이동에서도 발동해 실물로 시험할 수 있다.
  //   기본 15초 : 관측된 최장 이동(~4초, 든 손 저속 포함)의 여유배.
  declare_parameter("exec_watchdog_sec", 15.0);
  // move_to 액션 서버 등록 (상대 이름 > /move_to). 콜백 3개 연결.
  move_to_server_ = rclcpp_action::create_server<MoveTo>(
    this, "move_to",
    std::bind(&SkillServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&SkillServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&SkillServer::handle_accepted, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "skill_server 시작: move_to 대기 중");
  // pick 액션 서버
  pick_server_ = rclcpp_action::create_server<Pick>(
    this, "pick",
    std::bind(&SkillServer::handle_goal_pick, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&SkillServer::handle_cancel_pick, this, std::placeholders::_1),
    std::bind(&SkillServer::handle_accepted_pick, this, std::placeholders::_1));
  // place 액션 서버
  place_server_ = rclcpp_action::create_server<Place>(
    this, "place",
    std::bind(&SkillServer::handle_goal_place, this, std::placeholders::_1,
    std::placeholders::_2),
    std::bind(&SkillServer::handle_cancel_place, this, std::placeholders::_1),
    std::bind(&SkillServer::handle_accepted_place, this, std::placeholders::_1));
    // 인지 노드가 발행하는 스냅샷 구독
  scene_sub_ = create_subscription<arm_interfaces::msg::SceneState>(
      "/scene_state", 10, std::bind(&SkillServer::on_scene_state, this, std::placeholders::_1));
  // 그리퍼 관절 실측값 (멤버 선언은 private: 영역에 있다)
  joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&SkillServer::on_joint_states, this, std::placeholders::_1));
}

// joint_states에서 그리퍼 구동 관절만 뽑아 보관. gripper_joint_2는 mimic이라 쓰지 않는다.
// 부하가 걸리면구속이 어긋난다.
void SkillServer::on_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] != "gripper_joint_1") {
      continue;
    }
    std::lock_guard<std::mutex> lock(joint_mutex_);
    gripper_pos_ = msg->position[i];
    gripper_vel_ = (i < msg->velocity.size()) ? msg->velocity[i] : 0.0;
    gripper_seen_ = true;
    return;
  }
}

// MoveResult -> 계약. 실패 이유를 ErrorCode에 싣는다.(유일한 통로)
int32_t SkillServer::code_of(MoveResult r)
{
  if (r == MoveResult::UNREACHABLE) {
    return arm_interfaces::msg::ErrorCode::UNREACHABLE;
  }
  if (r == MoveResult::TIMEOUT) {
    return arm_interfaces::msg::ErrorCode::EXECUTION_TIMEOUT;
  }
  return arm_interfaces::msg::ErrorCode::PLANNING_FAILED;
}
std::string SkillServer::detail_of(MoveResult r)
{
  if (r == MoveResult::UNREACHABLE) {
    return "두 경로 모두 IK 도달 불가";
  }
  if (r == MoveResult::EXEC_FAILED) {
    return "경로는 나왔으나 실행 실패";
  }
  if (r == MoveResult::TIMEOUT) {
    return "실행 시간 초과 - watchdog 정지";
  }
  return "경로 계획 실패";
}

// ★ 2026-08-22 single-flight : 실행 중에는 어느 스킬이든 새 goal 을 거부한다.
//   compare_exchange 로 busy_ 를 수락 시점에 원자적으로 잡는다 - 두 goal 이
//   동시에 들어와도 한쪽만 통과한다. 해제는 execute_* 의 BusyRelease 가드.
rclcpp_action::GoalResponse SkillServer::handle_goal(
  const rclcpp_action::GoalUUID & /*UUID*/, std::shared_ptr<const MoveTo::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "move_to 목표 수신: target=%s", goal->pose_id.c_str());
  bool idle = false;
  if (!busy_.compare_exchange_strong(idle, true)) {
    RCLCPP_WARN(get_logger(), "move_to 거부 : 다른 스킬 실행 중(single-flight)");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
// ★ 2026-08-22 : 취소 = 실제 정지. 전에는 ACCEPT 만 반환하고 아무도 멈추지
//   않았다 - "받는 척"이었다. stop()은 MoveIt 실행 관리자를 통해 컨트롤러의
//   현재 궤적 goal 까지 취소시킨다(JTC 가 현 위치에서 감속 정지).
//   이 콜백은 executor 스레드에서 돌고 실행은 별도 스레드라 blocking 중인
//   move()를 밖에서 끊는 구조가 성립한다. 실행 스레드는 move() 반환 후
//   is_canceling()을 보고 canceled 로 마감한다(skill_move_to.cpp).
rclcpp_action::CancelResponse SkillServer::handle_cancel(
  const std::shared_ptr<GoalHandleMoveTo>/*gh*/)
{
  RCLCPP_WARN(get_logger(), "move_to 취소 요청 - 팔 정지");
  if (move_group_) {
    move_group_->stop();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}
rclcpp_action::GoalResponse SkillServer::handle_goal_pick(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const Pick::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "pick 목표 수신: object=%s", goal->object_id.c_str());
  bool idle = false;
  if (!busy_.compare_exchange_strong(idle, true)) {   // single-flight(handle_goal 주석)
    RCLCPP_WARN(get_logger(), "pick 거부 : 다른 스킬 실행 중(single-flight)");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
// ★ 2026-08-22 : pick/place 는 취소를 "명시적으로 거부"한다. 전에는 수락하고
//   무시했다 - 클라이언트가 멈춘 줄 알지만 팔은 계속 가는 최악의 거짓이었다.
//   시퀀스 중간 정지는 물건을 든 채 서는 상태를 설계해야 해서 별도 작업이다 -
//   그때까지는 "취소 불가"를 정직하게 알린다.
rclcpp_action::CancelResponse SkillServer::handle_cancel_pick(const std::shared_ptr<GoalHandlePick>)
{
  RCLCPP_WARN(get_logger(), "pick 취소 요청 거부 - 시퀀스 중간 정지는 미구현");
  return rclcpp_action::CancelResponse::REJECT;
}
rclcpp_action::GoalResponse SkillServer::handle_goal_place(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const Place::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "place 목표 수신: object=%s target=%s", goal->object_id.c_str(),
    goal->target_id.c_str());
  bool idle = false;
  if (!busy_.compare_exchange_strong(idle, true)) {   // single-flight(handle_goal 주석)
    RCLCPP_WARN(get_logger(), "place 거부 : 다른 스킬 실행 중(single-flight)");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse SkillServer::handle_cancel_place(
  const std::shared_ptr<GoalHandlePlace>)
{
  // pick 과 같은 이유로 거부한다(위 주석).
  RCLCPP_WARN(get_logger(), "place 취소 요청 거부 - 시퀀스 중간 정지는 미구현");
  return rclcpp_action::CancelResponse::REJECT;
}
// 수락 되면 실행은 별도의 스레드로 진행 (콜백 스레드를 막으면 안됨)
void SkillServer::handle_accepted(const std::shared_ptr<GoalHandleMoveTo> goal_handle)
{
  std::thread{std::bind(&SkillServer::execute_move_to, this, goal_handle)}.detach();   // detach 함수가 끝나면 자동 반환된다.
}
void SkillServer::handle_accepted_pick(const std::shared_ptr<GoalHandlePick> goal_handle)
{
  std::thread{std::bind(&SkillServer::execute_pick, this, goal_handle)}.detach();
}
void SkillServer::handle_accepted_place(const std::shared_ptr<GoalHandlePlace> goal_handle)
{
  std::thread{std::bind(&SkillServer::execute_place, this, goal_handle)}.detach();
}
// 최신 스냅샷을 저장하고 executor 콜백 스레드에서 실행
void SkillServer::on_scene_state(const arm_interfaces::msg::SceneState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(scene_mutex_);
  latest_scene_ = *msg;
}
// object_id를 통해 최신 스냅샷을 뒤져 위치를 꺼냄
bool SkillServer::lookup_object(
  const std::string & object_id, double & x, double & y, double & z, double & yaw)
{
  std::lock_guard<std::mutex> lock(scene_mutex_);
  for (const auto & obj : latest_scene_.objects) {
    if (obj.object_id == object_id) {
      x = obj.pose.pose.position.x;
      y = obj.pose.pose.position.y;
      z = obj.pose.pose.position.z;
      // 인지는 테이블 평면 위 z축 회전만
      const auto & q = obj.pose.pose.orientation;
      yaw = 2.0 * std::atan2(q.z, q.w);
      return true;
    }
  }
  return false;
}
// FailureReport를 만드는 유일한 통로 (make_failure와 같은 계약)
arm_interfaces::msg::FailureReport SkillServer::make_failure(
  int32_t code, const std::string & stage, const std::string & detail, uint8_t attempt)
{
  arm_interfaces::msg::FailureReport report;
  report.code = code;
  report.stage = stage;
  report.object_id = "";   // move_to는 팔만 움직이는 동작으로 대상 물체가 존재하지 않음.
  report.detail = detail;
  report.attempt = attempt;
  report.stamp = now();   // rclcpp::Node::now() -> Time
  return report;
}

std::shared_ptr<Pick::Result> SkillServer::make_pick_result(
  bool ok, uint8_t attempt, int32_t code, const std::string & stage, const std::string & detail)
{
  auto result = std::make_shared<Pick::Result>();
  result->success = ok;
  result->failure = make_failure(
    ok ? arm_interfaces::msg::ErrorCode::SUCCESS : code,
    ok ? "" : stage,
    ok ? "" : detail, attempt);
  return result;
}

std::shared_ptr<Place::Result> SkillServer::make_place_result(
  bool ok, uint8_t attempt, int32_t code, const std::string & stage, const std::string & detail)
{
  auto result = std::make_shared<Place::Result>();
  result->success = ok;
  result->failure = make_failure(
    ok ? arm_interfaces::msg::ErrorCode::SUCCESS : code,
    ok ? "" : stage,
    ok ? "" : detail, attempt);
  return result;
}
