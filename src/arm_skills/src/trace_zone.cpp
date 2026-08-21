// 구역 사각형의 네 꼭지점을 차례로 짚는 확인용 도구 (2026-08-20).
//
// 왜 move_to 액션이 아닌가: move_to는 SRDF의 named target만 받는다
// (skill_move_to.cpp 11행). 꼭지점은 자세 이름이 아니라 좌표라서 그 문을 못 쓴다.
// 그렇다고 SRDF(third_party)에 자세 넷을 박아 넣으면 구역을 옮길 때마다
// 벤더 파일을 고쳐야 한다 - 구역은 우리 숫자고 SRDF는 로봇 숫자다.
//
// 왜 별도 노드인가: skill_server의 move_to_pose는 멤버 함수라 밖에서 못 부른다.
// 여기서는 파지가 없어 가지 잠금(locked_elbow_)도 필요 없으므로, 같은 순서
// (IK -> 실물 관절 한계 -> 현재 자세와 가까운 가지 -> MoveIt 계획/실행)만
// 짧게 다시 쓴다. 안전 한계 상수는 params.hpp를 같이 본다 - 복사하지 않는다.
//
// 사각형은 파라미터로 받는다. 좌표를 여기 박으면 cell_layout.yaml의 사본이
// 다섯 개가 된다. 값을 물려주는 쪽은 arm_bringup/launch/trace_zone.launch.py 이고,
// 그 launch가 yaml을 읽는다(cell_layout.yaml 10행이 예고한 M6 경로).
//
// ⚠️ MoveIt 플래닝 씬에 테이블 위 물체가 등록되어 있지 않다(arm_skills 어디에도
//   CollisionObject가 없다). 계획은 물체를 못 본다 - 책상을 비우고 돌릴 것.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <arm_kinematics/ik.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

#include "arm_skills/params.hpp"

namespace
{

struct Corner
{
  const char * label;
  double x;
  double y;
};

// 현재 자세에서 목표 관절값까지의 L1 거리. motion.cpp의 joint_distance와 같은 뜻.
double joint_distance(const std::vector<double> & a, const std::vector<double> & b)
{
  if (a.size() != b.size()) {
    return 0.0;   // 현재 자세를 못 읽었다. 후보 순서를 흔들지 않는다.
  }
  double sum = 0.0;
  for (size_t i = 0; i < a.size(); ++i) {
    sum += std::abs(a[i] - b[i]);
  }
  return sum;
}

}  // namespace

class TraceZone : public rclcpp::Node
{
public:
  TraceZone()
  : Node("trace_zone")
  {
    // 기본값을 두지 않는 것이 원칙이지만(reach_once 11행의 교훈) ROS 파라미터는
    // 선언 자체에 값이 필요하다. 그래서 사각형은 NaN으로 선언해 두고 아래에서 막는다.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    zone_ = declare_parameter<std::string>("zone", "(이름 없음)");
    x0_ = declare_parameter<double>("x0", nan);
    x1_ = declare_parameter<double>("x1", nan);
    y0_ = declare_parameter<double>("y0", nan);
    y1_ = declare_parameter<double>("y1", nan);
    // 꼭지점을 짚는 높이. 테이블면이 0이다.
    // ★ 0.06 (2026-08-20 저녁 최종. 0.02 -> 0.12 를 거쳐 여기로 왔다).
    //   높이는 구역 크기와 맞물린다 - 구역에서 가장 먼 꼭지점이 상한을 정하고,
    //   구역이 클수록 그 상한이 낮아진다. 지금 구역 넷은 전부 이 높이에서 풀린다.
    //   ★ 구역을 넓히면 이 값을 반드시 다시 볼 것. 안 그러면 먼 꼭지점만 조용히
    //     "도달 불가"로 빠지고 나머지 셋만 짚는다.
    //   ★ launch 는 항상 z 를 넘긴다. 여기만 고치면 launch 로 돌렸을 때 안 바뀐다 -
    //     trace_zone.launch.py 의 기본값도 같이 고칠 것.
    z_ = declare_parameter<double>("z", 0.06);
    dwell_sec_ = declare_parameter<double>("dwell_sec", 2.0);
    loops_ = declare_parameter<int>("loops", 1);
    // 그리퍼는 아래를 향한다. pick/place와 같은 접근각.
    phi_ = declare_parameter<double>("phi", -M_PI / 2.0);
    home_first_ = declare_parameter<bool>("home_first", true);
  }

  // 생성자에서 shared_from_this()를 못 쓴다. 노드가 spin에 올라간 뒤 부른다.
  bool setup()
  {
    for (const auto & [name, v] : {std::pair<const char *, double>{"x0", x0_},
        {"x1", x1_}, {"y0", y0_}, {"y1", y1_}})
    {
      if (std::isnan(v)) {
        RCLCPP_ERROR(
          get_logger(),
          "파라미터 %s 가 없다. 사각형 네 값을 모두 넘겨야 한다 - "
          "trace_zone.launch.py 를 쓰면 cell_layout.yaml 에서 읽어 채워 준다.", name);
        return false;
      }
    }
    if (z_ < kMinZ) {
      // reach_once 25행과 같은 작업공간 하한. 테이블 아래를 겨누면 서보가 과부하난다.
      RCLCPP_ERROR(get_logger(), "z=%.3f 은 테이블면(0) 아래다. 최소 %.3f", z_, kMinZ);
      return false;
    }
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "arm");
    move_group_->setMaxVelocityScalingFactor(0.1);
    move_group_->setMaxAccelerationScalingFactor(0.1);
    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "gripper");
    return true;
  }

  void run()
  {
    // 사각형 둘레를 한 바퀴 도는 순서. x0가 카메라 쪽(가까운 줄), y0가 오른쪽이다.
    const std::vector<Corner> corners = {
      {"가까운-오른쪽", x0_, y0_},
      {"가까운-왼쪽", x0_, y1_},
      {"먼-왼쪽", x1_, y1_},
      {"먼-오른쪽", x1_, y0_},
    };

    RCLCPP_INFO(
      get_logger(),
      "구역 '%s' 꼭지점 순회 시작 : x[%.4f, %.4f] y[%.4f, %.4f], z=%.3f, "
      "꼭지점당 %.1f초 정지, %d바퀴",
      zone_.c_str(), x0_, x1_, y0_, y1_, z_, dwell_sec_, loops_);
    RCLCPP_WARN(
      get_logger(),
      "계획은 테이블 위 물체를 보지 못한다(플래닝 씬에 없음). 책상을 비웠는지 확인할 것.");

    // ★ 손가락부터 모은다(2026-08-20, 사용자 요청). 벌어진 그리퍼는 개구부가 약 10cm라
    //   (params.hpp open_pos 주석) 꼭지점을 짚을 때 손가락 끝이 사각형 밖으로 나간다.
    //   "어디를 짚었는가"가 흐려지고, 옆 물체를 쓸 위험도 커진다.
    //   close 자세로 보내면 컨트롤러가 토크를 건 채 끝까지 모은다 - 빈 손이라
    //   SUCCESS로 끝난다(막히면 CONTROL_FAILED. gripper.cpp의 판정 근거 참조).
    if (!gripper_group_->setNamedTarget("close")) {
      RCLCPP_WARN(get_logger(), "SRDF에 gripper 'close' 자세가 없다 - 벌린 채로 간다");
    } else if (gripper_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
      // 실패해도 순회는 계속한다. 손가락이 벌어져 있을 뿐 팔은 갈 수 있다.
      RCLCPP_WARN(get_logger(), "그리퍼 close 실패 - 벌린 채로 간다");
    } else {
      RCLCPP_INFO(get_logger(), "그리퍼 close(토크 걸림)");
    }

    if (home_first_) {
      // 어디서 시작하든 같은 자리에서 출발한다. 첫 이동의 경로가 매번 달라지지 않는다.
      go_named("home");
    }

    int failed = 0;
    for (int loop = 0; loop < loops_; ++loop) {
      for (const auto & c : corners) {
        if (!rclcpp::ok()) {
          return;
        }
        if (go_xy(c)) {
          RCLCPP_INFO(get_logger(), "  %s 도달 - %.1f초 정지", c.label, dwell_sec_);
          // 궤적이 끝난 뒤 실제로 멈춰 있는 시간. 사람이 눈으로 확인하는 창이다.
          std::this_thread::sleep_for(
            std::chrono::duration<double>(dwell_sec_));
        } else {
          ++failed;   // 한 점이 실패해도 나머지는 계속 짚는다. 어디가 안 되는지가 정보다.
        }
      }
    }

    if (home_first_) {
      go_named("home");
    }
    if (failed == 0) {
      RCLCPP_INFO(get_logger(), "순회 끝 - 꼭지점 %zu개 전부 도달", corners.size() * loops_);
    } else {
      RCLCPP_ERROR(
        get_logger(), "순회 끝 - %d개 실패. 그 꼭지점은 팔이 닿지 않거나 길이 막혔다.", failed);
    }
  }

private:
  static constexpr double kMinZ = 0.005;

  void go_named(const std::string & pose_id)
  {
    if (!move_group_->setNamedTarget(pose_id)) {
      RCLCPP_WARN(get_logger(), "SRDF에 없는 자세 : %s - 건너뛴다", pose_id.c_str());
      return;
    }
    if (move_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(get_logger(), "%s 이동 실패", pose_id.c_str());
    }
  }

  // motion.cpp의 move_to_pose와 같은 순서. 파지가 없으므로 가지 잠금은 없다.
  bool go_xy(const Corner & c)
  {
    const std::vector<double> current = move_group_->getCurrentJointValues();

    struct Candidate
    {
      bool elbow_up;
      std::vector<double> target;
      double dist;
    };
    std::vector<Candidate> candidates;

    for (bool elbow_up : {false, true}) {
      const auto geometry = arm_kinematics::solve_ik(c.x, c.y, z_, phi_, elbow_up);
      if (!geometry.reachable) {
        continue;
      }
      const auto m = arm_kinematics::to_motor_angles(geometry);
      std::vector<double> target = {m.theta1, m.theta2, m.theta3, m.theta4, m.theta5};
      bool within = true;
      for (size_t i = 0; i < target.size(); ++i) {
        if (target[i] < kRealJointMin[i] || target[i] > kRealJointMax[i]) {
          RCLCPP_WARN(
            get_logger(), "%s 가지 %s 버림 : joint%zu=%.3f이 실물 한계 [%.3f, %.3f] 밖",
            c.label, elbow_up ? "up" : "down", i + 1,
            target[i], kRealJointMin[i], kRealJointMax[i]);
          within = false;
          break;
        }
      }
      if (within) {
        candidates.push_back({elbow_up, target, joint_distance(current, target)});
      }
    }

    if (candidates.empty()) {
      RCLCPP_ERROR(
        get_logger(), "  %s (%.4f, %.4f, %.3f) 도달 불가 - IK 해 없음",
        c.label, c.x, c.y, z_);
      return false;
    }
    std::stable_sort(
      candidates.begin(), candidates.end(),
      [](const Candidate & a, const Candidate & b) {return a.dist < b.dist;});

    for (const auto & cand : candidates) {
      move_group_->setJointValueTarget(cand.target);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(get_logger(), "  %s 가지 %s 계획 실패", c.label, cand.elbow_up ? "up" : "down");
        continue;
      }
      if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "  %s 가지 %s 실행 실패", c.label, cand.elbow_up ? "up" : "down");
        return false;   // 실행 실패는 계획 실패와 다르다. 팔이 이미 움직였을 수 있다.
      }
      return true;
    }
    RCLCPP_ERROR(get_logger(), "  %s 어느 가지도 경로가 안 나왔다 - 길이 막혔다", c.label);
    return false;
  }

  std::string zone_;
  double x0_, x1_, y0_, y1_, z_, dwell_sec_, phi_;
  int loops_;
  bool home_first_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TraceZone>();

  // MoveGroupInterface가 현재 상태를 읽으려면 노드가 돌고 있어야 한다.
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() {exec.spin();});

  int rc = 0;
  if (node->setup()) {
    node->run();
  } else {
    rc = 2;
  }

  exec.cancel();
  spinner.join();
  rclcpp::shutdown();
  return rc;
}
