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

// 놓을 자리(base_link 기준). ★ 같은 좌표가 세 곳에 있다 - 하나만 고치면 조용히 어긋난다:
//   ① scene_cell.sdf 의 zone_* 모델 pose  ② agent.py 의 ZONE(구역 판정)  ③ 여기(팔이 실제로 가는 곳)
//   counter = 카운터(zone_counter, 주문의 종착지), bin = 수거함(zone_bin, 불량품)
//   shelf_block / shelf_ring = 물체별 창고.
//   ※ eval 시험지 182케이스가 "bin"을 놓는 자리로 쓴다. 그 값은 계약 어휘로 살아 있으므로
//     파싱 시험은 그대로 유효하다 - 다만 의미는 "수거함"으로 바뀌었다.
static const std::map<std::string, std::pair<double, double>> kTargets = {
  {"counter", {0.06, -0.145}},
  {"bin", {0.11, -0.145}},
  {"shelf_block", {0.06, 0.145}},
  {"shelf_ring", {0.11, 0.145}}
};

// 물체별 파지 파라미터. 어떻게 잡을 것인가의 주체는 skill이다.
//   depth   : 인지가 준 물체 중심 z에서 TCP(손가락 끝)를 얼마나 더 내릴 것인가.
//             손가락은 TCP에서 위로 뻗으므로 중심에 두면 윗절반만 물려 미끄러진다.
//   offset  : 물체 중심에서 파지점까지의 반경 거리. 가운데가 빈 물체(링)는
//             중심을 물면 허공을 문다 - 벽으로 옮겨 물어야 한다.
//   place_z : 놓을 때의 TCP 높이. 물체 높이가 다르면 놓는 높이도 다르다.
//   hold_eps: 파지 판정 임계. 문 두께가 다르면 손가락 관절값이 자릿수째로 다르다.
//             빈 손은 어느 물체든 0에 가깝다(낙하 순간 0.32 -> 0.0001 실측).
struct GraspSpec
{
  double depth;
  double offset;
  double place_z;
  double hold_eps;
};
// 놓고 물러나는 최소 높이. place_z + approach_dz로만 잡으면 납작한 물체(링)에서
// 6.6cm밖에 안 올라가고, 이어지는 home 복귀에서 팔이 방금 놓은 물체를 쓸고 간다
// (2026-08-14 실측 9cm 이탈). 그래서 하한을 둔다.
// ★ 상한은 도달 범위다: 수거함(r=0.242)에서 z=0.12는 IK 불가였고 0.1075는 됐다.
//    높이를 올릴수록 닿는 반경이 줄어든다.
static constexpr double kRetreatZ = 0.10;

static const std::map<std::string, GraspSpec> kGrasps = {
  // 블록 4x6x4cm. 중심 z=0.02에서 1cm 내려 아랫절반을 문다.
  // place_z 0.020 : 구 0.0475는 3.75cm 위에서 떨어뜨리는 값이었고, 접근 높이가
  //   0.1075가 되어 먼 자리(수거함 r=0.248)에서 IK 도달 불가였다(2026-08-15 실측).
  {"red_block", {0.010, 0.00000, 0.0200, 0.050}},
  // 불량품. red_block과 모양·크기·질량이 같아 값도 같다.
  {"green_block", {0.010, 0.00000, 0.0200, 0.050}},
  // 링 바깥지름 6.0 / 안지름 3.5 / 두께 1.2cm. 중심 z=0.006.
  // 벽 중심 반경 = (3.0+1.75)/2 = 2.375cm. 거기에 TCP를 두고 반경 방향으로 닫으면
  // 안쪽 손가락은 구멍(반경 1.75)에, 바깥 손가락은 링 밖(반경 3.0)에 놓여 벽을 문다.
  // 벽 1.25cm를 물면 파지 직후 0.114, 운반 중 0.048까지 내려간다(2026-08-14 실측).
  // 블록 기준 0.05를 그대로 쓰면 쥐고 있는데도 "빈 손"으로 판정해 place가 중단됐다.
  {"blue_ring", {0.004, 0.02375, 0.0060, 0.020}}
};

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
  void init_move_group()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "arm");   // "arm" = SRDF 그룹명
    RCLCPP_INFO(
      get_logger(), "arm 연결됨 : %zu개, planning frame=%s",
      move_group_->getJointNames().size(),
      move_group_->getPlanningFrame().c_str());
    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "gripper");   // gripper = SRDF 두 번째 그룹
    RCLCPP_INFO(
      get_logger(), "gripper 연결됨 : %zu개",
      gripper_group_->getJointNames().size());
  }
  // joint_states에서 그리퍼 구동 관절만 뽑아 보관. gripper_joint_2는 mimic이라 쓰지 않는다.
  // 부하가 걸리면구속이 어긋난다.
  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg)
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

private:
  // move_to_pose의 결과 bool로는 못 닿음과 길이 막힘이 구분되지 않는다.
  enum class MoveResult {OK, UNREACHABLE, PLAN_FAILED};

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

  // 관절공간 L1 거리. "팔이 얼마나 크게 움직여야 하는가"
  // 관절 값을 못 읽으면 0을 돌려 기존 순서를 유지한다.
  static double joint_distance(const std::vector<double> & a, const std::vector<double> & b)
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
  // MoveResult -> 계약. 실패 이유를 ErrorCode에 싣는다.(유일한 통로)
  static int32_t code_of(MoveResult r)
  {
    return r == MoveResult::UNREACHABLE ?
           arm_interfaces::msg::ErrorCode::UNREACHABLE :
           arm_interfaces::msg::ErrorCode::PLANNING_FAILED;
  }
  static std::string detail_of(MoveResult r)
  {
    return r == MoveResult::UNREACHABLE ? "두 경로 모두 IK 도달 불가" : "경로 계획 실패";
  }

  // 목표 수락 여부 > 지금은 무조건 수락
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*UUID*/, std::shared_ptr<const MoveTo::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "move_to 목표 수신: target=%s", goal->pose_id.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  // 취소 요청 -> 지금은 무조건 수락
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveTo>/*gh*/)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  rclcpp_action::GoalResponse handle_goal_pick(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Pick::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "pick 목표 수신: object=%s", goal->object_id.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse handle_cancel_pick(const std::shared_ptr<GoalHandlePick>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  rclcpp_action::GoalResponse handle_goal_place(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Place::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "place 목표 수신: object=%s target=%s", goal->object_id.c_str(),
      goal->target_id.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse handle_cancel_place(const std::shared_ptr<GoalHandlePlace>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  // 수락 되면 실행은 별도의 스레드로 진행 (콜백 스레드를 막으면 안됨)
  void handle_accepted(const std::shared_ptr<GoalHandleMoveTo> goal_handle)
  {
    std::thread{std::bind(&SkillServer::execute_move_to, this, goal_handle)}.detach();   // detach 함수가 끝나면 자동 반환된다.
  }
  void handle_accepted_pick(const std::shared_ptr<GoalHandlePick> goal_handle)
  {
    std::thread{std::bind(&SkillServer::execute_pick, this, goal_handle)}.detach();
  }
  void handle_accepted_place(const std::shared_ptr<GoalHandlePlace> goal_handle)
  {
    std::thread{std::bind(&SkillServer::execute_place, this, goal_handle)}.detach();
  }
  // 최신 스냅샷을 저장하고 executor 콜백 스레드에서 실행
  void on_scene_state(const arm_interfaces::msg::SceneState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(scene_mutex_);
    latest_scene_ = *msg;
  }
  // object_id를 통해 최신 스냅샷을 뒤져 위치를 꺼냄
  bool lookup_object(
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
  arm_interfaces::msg::FailureReport make_failure(
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
  // 목표(base 좌표, phi)로 이동. 가지를 "현재 자세와 가까운 순"으로 시도한다.
  // 호출마다 가지를 새로 고르면 물체를 쥔 채 팔이 통째로 뒤집혀 놓친다.
  // locked_elbow_가 있으면 그 가지만 쓴다(파지 중). 반환 = OK/UNREACHABLE/PLAN_FAILED
  MoveResult move_to_pose(
    double x, double y, double z, double phi, const char *label,
    std::optional<double> grasp_yaw = std::nullopt)
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
  // 그리퍼를 이름 자세로 (open/close). 반환값 = MoveIt 실행 결과 코드
  // 이 코드를 파지 판정 재료로 사용
  moveit::core::MoveItErrorCode move_gripper(const char *named)
  {
    gripper_group_->setNamedTarget(named);
    const auto code = gripper_group_->move();
    // 관절값은 여기서 읽지 않는다 - move() 직후 MoveIt 캐시는 정착 전 값을 준다
    // (실측: 캐시 0.4887 vs /joint_states 0.00004). 판정용 값은 is_holding()이 찍는다.
    RCLCPP_INFO(
      get_logger(), "그리퍼 %s : %s", named,
      moveit::core::errorCodeToString(code).c_str());
    return code;
  }
  // close 결과 코드 -> 쥐고 있는가  -> 상식과 반대로 읽힌다.
  // CONTROL_FAILED(-4) = 손가락이 물체에 막혀 목표까지 못닫힌다. 쥐고 있다.
  // SUCCESS(1) -> 끝까지 닫힘 = 사이에 아무것도 없다. = 빈 그리퍼 판정
  // 근거 체인
  // 1. GripperActionController - 오차가 goal_tolerance(0.01) 밖인 채로
  // stall_timeout(1.0) 동안 속도가 stall_velocity_threshold(0.001) 아래면
  // stalled=true, reached_goal=false로 setAborted(allow_stalling 기본 false)
  // 2. MoveIt GripperCommandControllerHandle : allow_failure 기본 false라 ABORTED를 그대로 전파
  // 3. 따라 MGI move()가 CONTROL_FAILED를 돌려준다.

  // 파지 판정 : 닫힘 목표에 도달했는가 물체 두께가 0이면 도달한다.
  // 에러 코드가 아니라 관절 위치를 본다. - stall은 sim 접촉 해석에 따라 안 날 수 있다.
  // ⚠️ 정의역 : pinch(두께로 개구부를 막는) 물체에 한정.
  // 링도 pinch로 잡는다 - 구멍에 손가락을 통과시키는 hook이 아니라 벽(1.25cm)을
  // 안팎에서 무는 방식이라 이 신호가 그대로 산다(2026-08-14 실측 0.114).
  // 임계는 물체마다 다르다 -> kGrasps.hold_eps.
  bool is_holding(double eps, const char *tag = "파지 판정")
  {
    const auto q = wait_gripper_settled();
    if (!q.has_value()) {
      RCLCPP_WARN(get_logger(), "%s 그리퍼 정착 실패 - 쥐었다고 판정하지 않는다", tag);
      return false;
    }
    const bool held = std::abs(*q) > eps;
    RCLCPP_INFO(
      get_logger(), "%s 판정 : 관절=%.4f (임계 %.4f) -> %s",
      tag, *q, eps, held ? "쥠" : "빈 손");
    return held;
  }
  // 실제 일 - 지금은 자리표시자: 로그만 찍고 성공 반환 (MoveGroupInterface는 다음 스텝)
  void execute_move_to(const std::shared_ptr<GoalHandleMoveTo> goal_handle)
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
  void execute_pick(const std::shared_ptr<GoalHandlePick> goal_handle)
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
  std::shared_ptr<Pick::Result> make_pick_result(
    bool ok, uint8_t attempt,
    int32_t code = arm_interfaces::msg::ErrorCode::PLANNING_FAILED,
    const std::string & stage = arm_interfaces::msg::Stage::APPROACH,
    const std::string & detail = "pick 이동 실패")
  {
    auto result = std::make_shared<Pick::Result>();
    result->success = ok;
    result->failure = make_failure(
      ok ? arm_interfaces::msg::ErrorCode::SUCCESS : code,
      ok ? "" : stage,
      ok ? "" : detail, attempt);
    return result;
  }
  void execute_place(const std::shared_ptr<GoalHandlePlace> goal_handle)
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
  std::shared_ptr<Place::Result> make_place_result(
    bool ok, uint8_t attempt,
    int32_t code = arm_interfaces::msg::ErrorCode::PLANNING_FAILED,
    const std::string & stage = arm_interfaces::msg::Stage::TRANSFER,
    const std::string & detail = "place 이동 실패")
  {
    auto result = std::make_shared<Place::Result>();
    result->success = ok;
    result->failure = make_failure(
      ok ? arm_interfaces::msg::ErrorCode::SUCCESS : code,
      ok ? "" : stage,
      ok ? "" : detail, attempt);
    return result;
  }
  // 그리퍼가 멈출 때까지 기다렸다가 그때의 위치를 돌려준다. 못 멈추면 nullopt.
  // ★ 속도가 아니라 위치 변화를 본다 (2026-08-08 실측으로 교체).
  // /joint_states의 gripper_joint_1 velocity가 항상 0으로 온다 - 실제로 초당 0.44로
  // 움직이는 순간에도 그렇다(900행 표본에서 |v|>0.001이 0건, 최댓값 2e-5).
  // 속도 기준이면 첫 샘플부터 조건이 성립해 「기다림」이 사라진다. 그 결과
  // 닫히는 도중의 통과값 0.4296을 「쥠」으로 오판했다 - 실제로는 빈 손이었다.
  // 위치는 엔코더가 직접 주는 1차 신호고 속도는 누가 미분해 채워주는 파생 신호다.
  // 채워주지 않으면 판정이 불가능하다. 그래서 이미 믿고 있는 신호(위치)로 통일한다
  // - is_holding()의 임계 비교도 같은 값을 쓴다.
  std::optional<double> wait_gripper_settled(
    double pos_eps = 0.001, int stable_ms = 200, int timeout_ms = 5000)
  {
    // ★ [2026-08-11] 시계를 노드 시계로 바꿨다. launch가 use_sim_time:=true 를 주므로
    // 이건 시뮬 시간이다. 전에는 steady_clock(현실 시간)으로 5초를 셌는데,
    // 재는 대상(그리퍼가 닫히는 물리)은 시뮬 시간에 산다. RTF가 0.25로 떨어진 머신에서
    // 「5초」가 시뮬 1.25초가 되어 정착 전에 데드라인이 끝났고, 성공한 파지가
    // GRASP_FAILED -> REGRASP -> ABORT 연쇄로 뒤집혔다(08-10 실측: close 495.196 ->
    // 정착 실패 500.214, 차이 5.018초 = 타임아웃 값과 일치).
    // 범위 주의: 이건 sim 전용 수정이다. 실기는 use_sim_time=false라 now()가 곧 현실
    // 시계이므로 이 교체가 아무것도 바꾸지 않는다. 실기에서 그리퍼가 5초 안에 못 닫히는
    // 경우는 시계 어긋남이 아니라 「예산 5초가 작다」는 별개 문제이고 처방도 다르다
    // (타임아웃 확대 또는 정착 기준 변경). M5 §6의 「실기에서도 똑같이 오판한다」는
    // 두 문제를 한 문장으로 묶은 것이라 M6에서 그대로 쓰면 오독이 된다.
    const rclcpp::Time deadline =
      now() + rclcpp::Duration::from_seconds(timeout_ms / 1000.0);
    double stable_acc_ms = 0.0;   // 20을 더하지 않는다. 실제로 흐른 시뮬 시간을 더한다
    rclcpp::Time prev = now();
    std::optional<double> last;
    while (now() < deadline) {
      // 잠은 현실 시간으로 잔다. 이건 폴링 간격일 뿐이고 판정에는 쓰지 않는다.
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      const rclcpp::Time t = now();
      const double dt_ms = (t - prev).seconds() * 1000.0;
      prev = t;                  // seen 여부와 무관하게 항상 전진시킨다.
                                 // 뒤에 두면 관절을 못 본 구간이 다음 바퀴의 dt에
                                 // 합산돼 안정 시간이 부풀려진다.
      double pos = 0.0;
      bool seen = false;
      {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        pos = gripper_pos_;
        seen = gripper_seen_;
      }
      if (!seen) {
        continue;
      }
      // 직전 표본과 견준다. 안 변하는 상태가 stable_ms(시뮬) 이어지면 멈춘 것으로 본다.
      stable_acc_ms = (last.has_value() && std::abs(pos - *last) < pos_eps)
        ? stable_acc_ms + dt_ms : 0.0;
      last = pos;
      if (stable_acc_ms >= stable_ms) {
        return pos;
      }
    }
    return std::nullopt;
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
