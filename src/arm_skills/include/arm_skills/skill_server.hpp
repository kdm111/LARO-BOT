#pragma once

#include <memory>
#include <thread>
#include <string>
#include <cmath>
#include <vector>
#include <mutex>
#include <optional>
#include <algorithm>
#include <chrono>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <arm_interfaces/msg/scene_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

#include <arm_interfaces/action/move_to.hpp>
#include <arm_interfaces/action/pick.hpp>
#include <arm_interfaces/action/place.hpp>

#include <arm_interfaces/msg/error_code.hpp>
#include <arm_interfaces/msg/failure_report.hpp>
#include <arm_interfaces/msg/stage.hpp>

#include <arm_kinematics/ik.hpp>   // solve_ik, to_motor_angles

using MoveTo = arm_interfaces::action::MoveTo;
using Pick = arm_interfaces::action::Pick;
using Place = arm_interfaces::action::Place;

using GoalHandleMoveTo = rclcpp_action::ServerGoalHandle<MoveTo>;
using GoalHandlePick = rclcpp_action::ServerGoalHandle<Pick>;
using GoalHandlePlace = rclcpp_action::ServerGoalHandle<Place>;

class SkillServer : public rclcpp::Node
{
public:
  SkillServer();
  void init_move_group();
  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg);

private:
  // move_to_pose의 결과 bool로는 못 닿음과 길이 막힘이 구분되지 않는다.
  enum class MoveResult {OK, UNREACHABLE, PLAN_FAILED, EXEC_FAILED};

  // 팔꿈치 가지 고정. 값이 있으면 그 가지만 쓴다.(물체를 쥐고 있는 동안)
  // grasp에서 down으로 잡고, lift에서 up으로 계획이 잡혀 팔이 통째로 뒤집힌다.
  std::optional<bool> locked_elbow_;
  // 실행에 성공한 가지. 이 값으로 잠근다.
  bool last_elbow_ = false;
  rclcpp_action::Server<MoveTo>::SharedPtr move_to_server_;
  rclcpp_action::Server<Pick>::SharedPtr pick_server_;
  rclcpp_action::Server<Place>::SharedPtr place_server_;
  rclcpp::Subscription<arm_interfaces::msg::SceneState>::SharedPtr scene_sub_;
  arm_interfaces::msg::SceneState latest_scene_;  // 최신 스냅샷 scene_mutex_로만 접근
  std::mutex scene_mutex_;

  // 그리퍼 관절 실측값. MoveIt의 상태 캐시는 move() 직후 정착 전 값을 준다
  // (2026-08-07 실측: 캐시 0.4887 vs /joint_states 0.00004) -> 원천을 직접 본다.
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  double gripper_pos_ = 0.0;
  double gripper_vel_ = 0.0;
  bool gripper_seen_ = false;
  std::mutex joint_mutex_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  // 팔과는 별개로 그리퍼는 움직일 수 있다.
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

  static double joint_distance(const std::vector<double> & a, const std::vector<double> & b);
  static int32_t code_of(MoveResult r);
  static std::string detail_of(MoveResult r);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveTo::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveTo>);
  rclcpp_action::GoalResponse handle_goal_pick(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Pick::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel_pick(const std::shared_ptr<GoalHandlePick>);
  rclcpp_action::GoalResponse handle_goal_place(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Place::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel_place(const std::shared_ptr<GoalHandlePlace>);
  void handle_accepted(const std::shared_ptr<GoalHandleMoveTo> goal_handle);
  void handle_accepted_pick(const std::shared_ptr<GoalHandlePick> goal_handle);
  void handle_accepted_place(const std::shared_ptr<GoalHandlePlace> goal_handle);

  void on_scene_state(const arm_interfaces::msg::SceneState::SharedPtr msg);
  bool lookup_object(
    const std::string & object_id, double & x, double & y, double & z, double & yaw);
  arm_interfaces::msg::FailureReport make_failure(
    int32_t code, const std::string & stage, const std::string & detail, uint8_t attempt);

  MoveResult move_to_pose(
    double x, double y, double z, double phi, const char * label,
    std::optional<double> grasp_yaw = std::nullopt);
  moveit::core::MoveItErrorCode move_gripper(const char * named);
  bool is_holding(double eps, const char * tag = "파지 판정");

  void execute_move_to(const std::shared_ptr<GoalHandleMoveTo> goal_handle);
  void execute_pick(const std::shared_ptr<GoalHandlePick> goal_handle);
  std::shared_ptr<Pick::Result> make_pick_result(
    bool ok, uint8_t attempt,
    int32_t code = arm_interfaces::msg::ErrorCode::PLANNING_FAILED,
    const std::string & stage = arm_interfaces::msg::Stage::APPROACH,
    const std::string & detail = "pick 이동 실패");
  void execute_place(const std::shared_ptr<GoalHandlePlace> goal_handle);
  std::shared_ptr<Place::Result> make_place_result(
    bool ok, uint8_t attempt,
    int32_t code = arm_interfaces::msg::ErrorCode::PLANNING_FAILED,
    const std::string & stage = arm_interfaces::msg::Stage::TRANSFER,
    const std::string & detail = "place 이동 실패");
  std::optional<double> wait_gripper_settled(
    double pos_eps = 0.001, int stable_ms = 200, int timeout_ms = 5000);
};
