#include "arm_skills/skill_server.hpp"
#include <cmath>

// kRealJointMin/kRealJointMax는 params.hpp로 옮겼다(2026-08-20). trace_zone과 공유한다.
#include "arm_skills/params.hpp"

void SkillServer::init_move_group()
{
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), "arm");   // "arm" = SRDF 그룹명
  RCLCPP_INFO(
    get_logger(), "arm 연결됨 : %zu개, planning frame=%s",
    move_group_->getJointNames().size(),
    move_group_->getPlanningFrame().c_str());
  set_carry_speed(false);   // 빈 손 기본값. 든 구간은 skill 쪽에서 켠다.
  gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), "gripper");   // gripper = SRDF 두 번째 그룹
  // 파지 판정 전용 경로. 여는 것은 MoveIt 으로 해도 되지만 "닫고 확인"만은
  // 컨트롤러를 직접 불러야 한다(gripper.cpp grasp_check 주석의 실측 참조).
  gripper_cmd_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
    this, "/gripper_controller/gripper_cmd");
  RCLCPP_INFO(
    get_logger(), "gripper 연결됨 : %zu개",
    gripper_group_->getJointNames().size());
}

// ★ 2026-08-22 안전 종료 : 세우고 -> home -> 토크 유지(선언부 주석 참조).
//   스킬 실행 중이었다면 stop()이 그 동작을 끊는다 - 물체를 든 채라도 home 으로
//   간다(놓치는 것보다 든 채 서 있는 편이 안전하다). 실패해도 그 자리 유지 -
//   토크는 어차피 켠 채 종료하므로 무너지지 않는다.
void SkillServer::safe_park()
{
  if (!move_group_) {
    return;   // init_move_group 전에 종료 - 움직인 적도 없다
  }
  RCLCPP_WARN(get_logger(), "종료 절차 : 동작 중단 후 home 으로 이동(토크 유지)");
  move_group_->stop();
  set_carry_speed(false);
  if (move_group_->setNamedTarget("home") &&
    move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_WARN(get_logger(), "종료 절차 : home 도달 - 토크 유지한 채 종료");
  } else {
    RCLCPP_ERROR(get_logger(), "종료 절차 : home 이동 실패 - 현 자세로 종료(토크 유지)");
  }
}

// 물체를 든 구간만 느리게 간다. 근거는 skill_server.hpp 선언부 주석.
//   빈 손 0.30 / 0.45 - 2026-08-20 사용자가 올린 값. home<->init 이 8.3초에서 2.6초가 됐다.
//   든 손 0.10 / 0.10 - 그 전에 쓰던 값. 이 값으로 돌던 동안 미끄러짐이 없었다.
// ⚠️ 이건 가설에 대한 대책이다. "가속이 커서 빠진다"를 아직 실측으로 확정하지 않았다.
//    느린 쪽에서도 빠지면 원인은 파지 형상(둥근 면을 평평한 손가락으로 무는 것)이다.
void SkillServer::set_carry_speed(bool carrying)
{
  const double vel = carrying ? 0.10 : 0.30;
  const double acc = carrying ? 0.10 : 0.45;
  move_group_->setMaxVelocityScalingFactor(vel);
  move_group_->setMaxAccelerationScalingFactor(acc);
  RCLCPP_INFO(
    get_logger(), "이동 속도 : %s (vel %.2f / acc %.2f)",
    carrying ? "든 손" : "빈 손", vel, acc);
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
  std::optional<double> grasp_yaw, bool wrist_canonical)
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
    // ★ 2026-08-21 밤 : 손목 해 고정(근거는 skill_server.hpp 선언부 주석).
    //   j5 를 (-90, 90]도로 접는다 - 180도 돌린 해는 대칭 그리퍼로는 같은
    //   파지라서 접어도 TCP·닫힘축이 안 변한다.
    if (wrist_canonical) {
      while (target[4] > M_PI_2) {target[4] -= M_PI;}
      while (target[4] <= -M_PI_2) {target[4] += M_PI;}
    }
    bool within = true;
    for (size_t i = 0; i < target.size(); ++i) {
      if (target[i] < kRealJointMin[i] || target[i] > kRealJointMax[i]) {
        RCLCPP_WARN(
          get_logger(), "%s 가지 %s 버림 : joint%zu=%.3f이 실물 한계 [%.3f, %.3f] 밖",
          label, elbow_up ? "up" : "down", i + 1,
          target[i], kRealJointMin[i], kRealJointMax[i]);
        within = false;
        break;
      }
    }
    if (!within) {
      continue;
    }
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
      // ★ 2026-08-22 watchdog : 실행이 예산 안에 못 끝나면 팔을 세운다.
      //   시리얼 무응답류의 "영원히 안 끝나는 execute"가 표적이다 - execute 는
      //   블로킹이라 감시는 별도 스레드가 한다. 예산은 파라미터 exec_watchdog_sec
      //   (기본 15초 - 관측된 최장 이동 ~4초의 여유배. 검증 때 1초로 줄여 발동을
      //   실물 확인했다). 발동하면 stop()이 execute 를 끊고 TIMEOUT 으로 보고한다.
      const double budget_sec =
        get_parameter("exec_watchdog_sec").as_double();
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
      const auto exec = move_group_->execute(plan);
      exec_done = true;
      dog.join();
      if (dog_fired) {
        RCLCPP_ERROR(
          get_logger(), "%s watchdog 발동 : 예산 %.1f초 초과 - 팔 정지",
          label, budget_sec);
        return MoveResult::TIMEOUT;
      }
      if (exec != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(
          get_logger(), "%s 가지 %s 실행 실패 (code %d) 다음 가지로 이동",
          label, c.elbow_up ? "up" : "down", exec.val);
        return MoveResult::EXEC_FAILED;
      }
      last_elbow_ = c.elbow_up;  // 실행에 성공하면 잠금 후보로 두고 그 가지로 향하도록 설정
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
