#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

#include <arm_interfaces/action/move_to.hpp>

using MoveTo = arm_interfaces::action::MoveTo;
using GoalHandleMoveTo = rclcpp_action::ServerGoalHandle<MoveTo>;

class SkillServer : public rclcpp::Node
{
public:
  SkillServer()
  : Node("skill_server")
  {
    // move_to 액션 서버 등록 (상대 이름 > /move_to). 콜백 3개 연결.
    move_to_server_ = rclcpp_action::create_server<MoveTo>(
      this, "move_to",
      std::bind(&SkillServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SkillServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&SkillServer::handle_accepted, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "skill_server 시작: move_to 대기 중");
  }
  void init_move_group()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "arm");   // "arm" = SRDF 그룹명
    RCLCPP_INFO(
      get_logger(), "arm 연결됨 : %zu개, planning frame=%s",
      move_group_->getJointNames().size(),
      move_group_->getPlanningFrame().c_str());
  }

private:
  rclcpp_action::Server<MoveTo>::SharedPtr move_to_server_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  // 목표 수락 여부 > 지금은 무조건 수락
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*UUID*/, std::shared_ptr<const MoveTo::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "move_to 목표 수신: target=%s", goal->target_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  // 취소 요청 -> 지금은 무조건 수락
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveTo>/*gh*/)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  // 수락 되면 실행은 별도의 스레드로 진행 (콜백 스레드를 막으면 안됨)
  void handle_accepted(const std::shared_ptr<GoalHandleMoveTo> goal_handle)
  {
    std::thread{std::bind(&SkillServer::execute, this, goal_handle)}.detach();
  }
  // 실제 일 - 지금은 자리표시자: 로그만 찍고 성공 반환 (MoveGroupInterface는 다음 스텝)
  void execute(const std::shared_ptr<GoalHandleMoveTo> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    RCLCPP_INFO(get_logger(), "실행(placeholder): %s로 이동한다고 치자", goal->target_name.c_str());

    auto result = std::make_shared<MoveTo::Result>();
    result->success = true; // 아직 실제 이동은 넣지 않음
    goal_handle->succeed(result);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SkillServer>();

  // MGI가 로봇 모델을 토픽으로 받으려면 노드가 spin 중이여야 한다. -> 백그라운드 executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  node->init_move_group();   // spin 시작 후 모델 수신 + shared_from_this 유효
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
