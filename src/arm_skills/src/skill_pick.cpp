#include "arm_skills/params.hpp"
#include "arm_skills/skill_server.hpp"

void SkillServer::execute_pick(const std::shared_ptr<GoalHandlePick> goal_handle)
{
  locked_elbow_.reset();   // 새로운 pick 시작 = 이전 잠금 무효
  const auto goal = goal_handle->get_goal();
  RCLCPP_INFO(get_logger(), "pick 실행 : object=%s", goal->object_id.c_str());

  double obj_x, obj_y, obj_z, obj_yaw;
  if (!lookup_object(goal->object_id, obj_x, obj_y, obj_z, obj_yaw)) {
    RCLCPP_WARN(get_logger(), "/scene_state에 %s 없음", goal->object_id.c_str());
    auto result = std::make_shared<Pick::Result>();
    result->success = false;
    result->failure = make_failure(
      arm_interfaces::msg::ErrorCode::OBJECT_NOT_FOUND,
      arm_interfaces::msg::Stage::PLAN,
      "/scene_state에서 " + goal->object_id + " 못 찾음", goal->attempt);
    goal_handle->abort(result);
    return;
  }
  const double approach_phi = -M_PI / 2;   // 그리퍼가 아래를 향하는 접근각
  const double approach_dz = 0.06;    // 물체 위 6cm 에서 접근

  // 인지가 준 물체의 긴 축을 기준으로 짧은 축의 각을 찾아야 하므로 90도 회전한다.
  // 어떻게 잡을 것인가의 주체는 skill에 있다.
  // remainder(x, PI)는 [-PI/2, PI/2]로 접는다. 그리퍼는 180도 대칭이라 같은 파지가 된다.
  // TCP는 손가락의 끝이고 손가락은 거기서 위로 뻗는다.. 물체 중심에 TCP를 두면 윗절반만 물려서 미끄러진다.
  const auto git = kGrasps.find(goal->object_id);
  if (git == kGrasps.end()) {
    goal_handle->abort(
      make_pick_result(
        false, goal->attempt, arm_interfaces::msg::ErrorCode::INTERNAL_ERROR,
        arm_interfaces::msg::Stage::PLAN, "파지 파라미터가 없는 물체"));
    return;
  }
  const GraspSpec & gs = git->second;

  // 파지점. 속이 빈 물체는 중심이 아니라 벽을 문다.
  double grasp_x = obj_x;
  double grasp_y = obj_y;
  double grasp_yaw;
  if (gs.offset > 0.0) {
    // 로봇 쪽 벽으로 옮긴다. 반대편 벽을 물면 손목이 물체를 넘어가야 하고
    // 반경도 멀어져 도달 한계(실측 0.25~0.27)에 더 가까워진다.
    const double dir = std::atan2(-obj_y, -obj_x);   // 물체 중심 -> 로봇
    grasp_x = obj_x + gs.offset * std::cos(dir);
    grasp_y = obj_y + gs.offset * std::sin(dir);
    grasp_yaw = std::remainder(dir, M_PI);           // 그리퍼가 반경 방향으로 닫힌다
  } else {
    grasp_yaw = std::remainder(obj_yaw + M_PI_2, M_PI);
  }
  RCLCPP_INFO(
    get_logger(), "파지점 (%.3f, %.3f) z=%.3f yaw=%.1f도",
    grasp_x, grasp_y, obj_z - gs.depth, grasp_yaw * 180.0 / M_PI);
  move_gripper("open");
  if (const auto r = move_to_pose(
      grasp_x, grasp_y, obj_z + approach_dz, approach_phi, "approach", grasp_yaw);
    r != MoveResult::OK)
  {
    goal_handle->abort(
      make_pick_result(
        false, goal->attempt, code_of(r),
        arm_interfaces::msg::Stage::APPROACH, detail_of(r)));
    return;
  }
  if (const auto r = move_to_pose(
      grasp_x, grasp_y, obj_z - gs.depth, approach_phi, "grasp", grasp_yaw);
    r != MoveResult::OK)
  {
    goal_handle->abort(
      make_pick_result(
        false, goal->attempt, code_of(r),
        arm_interfaces::msg::Stage::GRASP, detail_of(r)));
    return;
  }
  // close의 결과가 곧 파지 판정 lift전에 분기해야함.
  // 빈 손으로 들어올리면 성공한 pick이 되어 GRASP_FAILED가 영원히 잡히지 않는다.
  move_gripper("close");
  if (!is_holding(gs.hold_eps)) {
    RCLCPP_WARN(get_logger(), "파지 실패 : %s (그리퍼가 끝까지 닫힘)", goal->object_id.c_str());
    // 여기서 열지 않는다. 판정이 틀렸을 때 (실제로 쥐고 있을 때 물건이 떨어진다.)
    // move_gripper("open");   // 다음 시도를 위해 열어둠 REGRASP
    goal_handle->abort(
      make_pick_result(
        false, goal->attempt,
        arm_interfaces::msg::ErrorCode::GRASP_FAILED,
        arm_interfaces::msg::Stage::GRASP,
        "그리퍼가 끝까지 닫힘 - 물체를 못잡음"));
    return;
  }
  locked_elbow_ = last_elbow_;  // 쥐었다. =grasp에 쓴 가지로 고정
  RCLCPP_INFO(get_logger(), "가지 %s 고정(파지 중)", *locked_elbow_ ? "up" : "down");

  if (const auto r = move_to_pose(
      grasp_x, grasp_y, obj_z + approach_dz, approach_phi, "lift", grasp_yaw);
    r != MoveResult::OK)
  {
    goal_handle->abort(
      make_pick_result(
        false, goal->attempt, code_of(r),
        arm_interfaces::msg::Stage::LIFT, detail_of(r)));
    return;
  }
  RCLCPP_INFO(get_logger(), "pick 완료 : %s", goal->object_id.c_str());
  goal_handle->succeed(make_pick_result(true, goal->attempt));
}
