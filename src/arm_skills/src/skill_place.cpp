#include "arm_skills/params.hpp"
#include "arm_skills/skill_server.hpp"

void SkillServer::execute_place(const std::shared_ptr<GoalHandlePlace> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  RCLCPP_INFO(get_logger(), "place 실행 : target=%s", goal->target_id.c_str());


  const auto it = kTargets.find(goal->target_id);
  if (it == kTargets.end()) {
    goal_handle->abort(
      make_place_result(
        false, goal->attempt, arm_interfaces::msg::ErrorCode::UNDEFINED_POSE,
        arm_interfaces::msg::Stage::TRANSFER, "계약에 없는 target_id"));
    return;
  }
  const auto pit = kGrasps.find(goal->object_id);
  if (pit == kGrasps.end()) {
    goal_handle->abort(
      make_place_result(
        false, goal->attempt, arm_interfaces::msg::ErrorCode::INTERNAL_ERROR,
        arm_interfaces::msg::Stage::TRANSFER, "파지 파라미터가 없는 물체"));
    return;
  }
  const double tgt_x = it->second.first;
  const double tgt_y = it->second.second;
  const double tgt_z = pit->second.place_z;
  const double approach_phi = -M_PI / 2;   // 그리퍼가 아래를 향하는 접근각
  const double approach_dz = 0.06;    // 물체 위 6cm 에서 접근

  // 속이 빈 물체는 벽을 물고 있어서 물체 중심이 TCP에서 offset만큼 떨어져 있다.
  // 놓을 때 TCP를 목표 중심에 두면 물체는 그만큼 밀려 놓인다(실측 11cm 이탈).
  // 잡을 때와 같은 방향으로 TCP를 옮기고 손목도 같은 각으로 맞춘다.
  const GraspSpec & ps = pit->second;
  double put_x = tgt_x;
  double put_y = tgt_y;
  std::optional<double> put_yaw = std::nullopt;
  if (ps.offset > 0.0) {
    const double dir = std::atan2(-tgt_y, -tgt_x);   // 목표 중심 -> 로봇
    put_x = tgt_x + ps.offset * std::cos(dir);
    put_y = tgt_y + ps.offset * std::sin(dir);
    put_yaw = std::remainder(dir, M_PI);
  }

  if (const auto r = move_to_pose(
      put_x, put_y, tgt_z + approach_dz, approach_phi, "place-approach", put_yaw);
    r != MoveResult::OK)
  {
    goal_handle->abort(
      make_place_result(
        false, goal->attempt, code_of(r),
        arm_interfaces::msg::Stage::TRANSFER, detail_of(r)));
    return;
  }
  // transfer 파지 확인 : 운반 중 낙하면 손가락이 끝까지 닫혀 q가 0으로 떨어진다.
  // 실측 근거 - 낙하 순간 q 0.32 -> 0.0001
  if (!is_holding(ps.hold_eps, "운반 중 파지 확인")) {
    locked_elbow_.reset();
    goal_handle->abort(
      make_place_result(
        false, goal->attempt, arm_interfaces::msg::ErrorCode::GRIPPER_EMPTY,
        arm_interfaces::msg::Stage::TRANSFER, "운반 중 물체를 놓쳤다."));
    return;
  }
  if (const auto r = move_to_pose(put_x, put_y, tgt_z, approach_phi, "place-lower", put_yaw);
    r != MoveResult::OK)
  {
    goal_handle->abort(
      make_place_result(
        false, goal->attempt, code_of(r),
        arm_interfaces::msg::Stage::TRANSFER, detail_of(r)));
    return;
  }
  move_gripper("open");
  locked_elbow_.reset();   // 그리퍼 놓음 잠금 해제
  if (const auto r = move_to_pose(
      put_x, put_y, std::max(tgt_z + approach_dz, kRetreatZ),
      approach_phi, "place-retreat", put_yaw);
    r != MoveResult::OK)
  {
    goal_handle->abort(
      make_place_result(
        false, goal->attempt, code_of(r),
        arm_interfaces::msg::Stage::RETREAT, detail_of(r)));
    return;
  }
  RCLCPP_INFO(get_logger(), "place 완료 : %s", goal->target_id.c_str());
  goal_handle->succeed(make_place_result(true, goal->attempt));
}
