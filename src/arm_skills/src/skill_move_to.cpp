#include "arm_skills/skill_server.hpp"

// MoveTo 실행 : SRDF 이름 자세로 계획+실행(move()), watchdog 감시, 취소 시
// canceled 마감. ("자리표시자" 시절 주석이 오래 낡아 있었다 - 2026-08-22 갱신)
void SkillServer::execute_move_to(const std::shared_ptr<GoalHandleMoveTo> goal_handle)
{
  const BusyRelease release{busy_};   // 어느 return 으로 나가든 single-flight 해제
  const auto goal = goal_handle->get_goal();
  RCLCPP_INFO(get_logger(), "move_to 실행 : %s", goal->pose_id.c_str());
  // move_to 는 물체를 들고 부르는 동작이 아니다. pick 이 켜 둔 든 손 속도를 되돌린다
  // (pick 이 실패로 끝나거나 place 를 안 거치고 home 으로 보낼 때가 있다).
  set_carry_speed(false);

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
  // ★ 2026-08-22 watchdog : move_to_pose 의 것과 같은 감시(motion.cpp 주석).
  //   named 자세 이동은 move()가 계획+실행을 한 번에 하므로 여기서 따로 감싼다.
  const double budget_sec = get_parameter("exec_watchdog_sec").as_double();
  std::atomic<bool> exec_done{false};
  std::atomic<bool> dog_fired{false};
  std::thread dog(
    [this, &exec_done, &dog_fired, budget_sec]() {
      const auto t0 = std::chrono::steady_clock::now();
      while (!exec_done) {
        if (std::chrono::steady_clock::now() - t0 >
          std::chrono::duration<double>(budget_sec))
        {
          dog_fired = true;
          move_group_->stop();
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    });
  const bool ok = (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
  exec_done = true;
  dog.join();

  // ★ 2026-08-22 : 취소로 끊긴 경우. handle_cancel 이 stop()을 불러 위 move()가
  //   중간에 반환된다 - 실패(abort)가 아니라 canceled 로 마감해야 액션 상태가
  //   진실을 말한다. 팔은 stop() 시점 자리에 감속 정지해 있다.
  if (goal_handle->is_canceling()) {
    auto result = std::make_shared<MoveTo::Result>();
    result->success = false;
    result->failure = make_failure(
      arm_interfaces::msg::ErrorCode::CANCELED,
      arm_interfaces::msg::Stage::EXECUTE,
      "취소로 정지 : " + goal->pose_id, goal->attempt);
    goal_handle->canceled(result);
    RCLCPP_WARN(get_logger(), "move_to 취소됨 : %s (현 위치 정지)", goal->pose_id.c_str());
    return;
  }

  // watchdog 발동 : 취소가 아닌데 예산 초과로 세웠다. 취소 확인이 먼저다 -
  // 취소로 멈춘 것을 시간 초과로 잘못 보고하지 않는다.
  if (dog_fired) {
    auto result = std::make_shared<MoveTo::Result>();
    result->success = false;
    result->failure = make_failure(
      arm_interfaces::msg::ErrorCode::EXECUTION_TIMEOUT,
      arm_interfaces::msg::Stage::EXECUTE,
      "실행 시간 초과 - watchdog 정지 : " + goal->pose_id, goal->attempt);
    goal_handle->abort(result);
    RCLCPP_ERROR(
      get_logger(), "move_to watchdog 발동 : %s (예산 %.1f초 초과 - 팔 정지)",
      goal->pose_id.c_str(), budget_sec);
    return;
  }

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
