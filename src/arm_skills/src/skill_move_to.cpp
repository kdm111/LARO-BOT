#include "arm_skills/skill_server.hpp"

// 실제 일 - 지금은 자리표시자: 로그만 찍고 성공 반환 (MoveGroupInterface는 다음 스텝)
void SkillServer::execute_move_to(const std::shared_ptr<GoalHandleMoveTo> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  RCLCPP_INFO(get_logger(), "move_to 실행 : %s", goal->pose_id.c_str());

  // SRDF에 정의된 pose로 목표 설정 (arm group init /home)
  // 존재하지 않을 경우 없는 자세 에러 설정 이후 불가판정
  if (!move_group_->setNamedTarget(goal->pose_id)) {
    auto result = std::make_shared<MoveTo::Result>();
    result->success = false;
    result->failure = make_failure(
      arm_interfaces::msg::ErrorCode::UNDEFINED_POSE,
      arm_interfaces::msg::Stage::PLAN,
      "SRDF에 없는 pose_id : " + goal->pose_id, goal->attempt);
    goal_handle->abort(result);
    return;
  }
  // plan + execute (블로킹). SUCCESS면 성공. 벤더 qnode.cpp와 같은 판정
  const bool ok = (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);

  auto result = std::make_shared<MoveTo::Result>();
  result->success = ok;
  if (ok) {
    result->failure = make_failure(
      arm_interfaces::msg::ErrorCode::SUCCESS, "", "", goal->attempt);
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "move_to 성공 : %s", goal->pose_id.c_str());
  } else {
    result->failure = make_failure(
      arm_interfaces::msg::ErrorCode::PLANNING_FAILED,
      arm_interfaces::msg::Stage::PLAN,
      "move() 실패 : " + goal->pose_id, goal->attempt);
    goal_handle->abort(result);
    RCLCPP_WARN(get_logger(), "move_to 실패 : %s", goal->pose_id.c_str());
  }
}
