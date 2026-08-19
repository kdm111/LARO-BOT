#include "arm_skills/skill_server.hpp"

void SkillServer::init_move_group()
{
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), "arm");   // "arm" = SRDF 그룹명
  RCLCPP_INFO(
    get_logger(), "arm 연결됨 : %zu개, planning frame=%s",
    move_group_->getJointNames().size(),
    move_group_->getPlanningFrame().c_str());
  move_group_->setMaxVelocityScalingFactor(0.05);
  move_group_->setMaxAccelerationScalingFactor(0.05);
  gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), "gripper");   // gripper = SRDF 두 번째 그룹
  RCLCPP_INFO(
    get_logger(), "gripper 연결됨 : %zu개",
    gripper_group_->getJointNames().size());
}

// 관절공간 L1 거리. "팔이 얼마나 크게 움직여야 하는가"
// 관절 값을 못 읽으면 0을 돌려 기존 순서를 유지한다.
double SkillServer::joint_distance(const std::vector<double> & a, const std::vector<double> & b)
{
  if (a.size() != b.size()) {
    return 0.0;
  }
  double sum = 0.0;
  for (size_t i = 0; i < a.size(); ++i) {
    sum += std::abs(a[i] - b[i]);
  }
  return sum;
}

// 목표(base 좌표, phi)로 이동. 가지를 "현재 자세와 가까운 순"으로 시도한다.
// 호출마다 가지를 새로 고르면 물체를 쥔 채 팔이 통째로 뒤집혀 놓친다.
// locked_elbow_가 있으면 그 가지만 쓴다(파지 중). 반환 = OK/UNREACHABLE/PLAN_FAILED
SkillServer::MoveResult SkillServer::move_to_pose(
  double x, double y, double z, double phi, const char * label,
  std::optional<double> grasp_yaw)
{
  // 잠겨 있다면 그 가지 쪽으로만 가고(이동을 최소) 아니면 둘 다 후보로 지정
  const std::vector<bool> branches =
    locked_elbow_.has_value() ?
    std::vector<bool>{*locked_elbow_} : std::vector<bool>{false, true};

  const std::vector<double> current = move_group_->getCurrentJointValues();

  struct Candidate
  {
    bool elbow_up;
    std::vector<double> target;
    double dist;
  };
  std::vector<Candidate> candidates;

  for (bool elbow_up : branches) {
    const auto geometry = arm_kinematics::solve_ik(x, y, z, phi, elbow_up, grasp_yaw);
    if (!geometry.reachable) {
      continue;   // 이 가지는 기하학적으로 도달 불가
    }
    const auto m = arm_kinematics::to_motor_angles(geometry);
    std::vector<double> target = {m.theta1, m.theta2, m.theta3, m.theta4, m.theta5};
    candidates.push_back({elbow_up, target, joint_distance(current, target)});
  }

  // 어느 가지도 IK가 안 풀렸다. = 팔이 못 닿는다. 계획 문제가 아니다
  if (candidates.empty()) {
    RCLCPP_WARN(
      get_logger(), "%s IK 도달 불가 (%.3f, %.3f, %.3f)", label, x, y, z);
    return MoveResult::UNREACHABLE;
  }
  // 현재 자세와 가까운 순. stable이라 거리가 같으면 기존 순서 우선
  std::stable_sort(
    candidates.begin(), candidates.end(),
    [](const Candidate & a, const Candidate & b) {return a.dist < b.dist;});

  for (const auto & c : candidates) {
    move_group_->setJointValueTarget(c.target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_->execute(plan);
      last_elbow_ = c.elbow_up;
      // 관절각(모터)까지 찍는다. 마지막 값 theta5가 그리퍼 롤 = 파지 회전 확인용.
      RCLCPP_INFO(
        get_logger(),
        "%s 자세 도달(가지 %s, 관절거리 %.3f) 관절각[%.3f %.3f %.3f %.3f %.3f]",
        label, c.elbow_up ? "up" : "down", c.dist,
        c.target[0], c.target[1], c.target[2], c.target[3], c.target[4]);
      return MoveResult::OK;   // 성공한 가지에서 즉시 끝낸다
    }
    RCLCPP_WARN(get_logger(), "%s 가지 %s 계획 실패", label, c.elbow_up ? "up" : "down");
  }
  // IK는 풀렸는데 어느 가지도 경로가 안 나왔다 = 길이 막혔다
  return MoveResult::PLAN_FAILED;
}
