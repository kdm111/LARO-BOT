#include <memory>
#include <thread>
#include <string>
#include <cmath>
#include <vector>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <arm_interfaces/msg/scene_state.hpp>
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

private:
  rclcpp_action::Server<MoveTo>::SharedPtr move_to_server_;
  rclcpp_action::Server<Pick>::SharedPtr pick_server_;
  rclcpp_action::Server<Place>::SharedPtr place_server_;
  rclcpp::Subscription<arm_interfaces::msg::SceneState>::SharedPtr scene_sub_;
  arm_interfaces::msg::SceneState latest_scene_;  // 최신 스냅샷 scene_mutex_로만 접근
  std::mutex scene_mutex_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  // 팔과는 별개로 그리퍼는 움직일 수 있다.
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

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
  // 목표(base 좌표, phi)로 이동. elbow up/down 둘 다 plan()으로 시도해 되는 가지를 execute
  // 성공하면 true, (approach, grasp가 이 함수를 높이만 바꿔서 두 번 부를 예정)
  bool move_to_pose(
    double x, double y, double z, double phi, const char *label,
    std::optional<double> grasp_yaw = std::nullopt)
  {
    bool approached = false;

    for (bool elbow_up : {false, true}) {
      const auto geometry = arm_kinematics::solve_ik(x, y, z, phi, elbow_up, grasp_yaw);
      if (!geometry.reachable) {
        continue;   // 이 가지는 기하학적으로 도달 불가
      }
      const auto motor_angles = arm_kinematics::to_motor_angles(geometry);   // 기하각 -> 모터각 변환
      RCLCPP_INFO(
        get_logger(), "%s 가지 %s 관절각(모터), [%.3f, %.3f, %.3f, %.3f, %.3f]",
        label, elbow_up ? "up" : "down",
        motor_angles.theta1, motor_angles.theta2, motor_angles.theta3, motor_angles.theta4,
        motor_angles.theta5);
      // solve_ik가 푼 관절각을 move_group에 목표로 준다
      // Move는 이름 자세이지만 여기에서는 목표 지점으로 관절을 전달해야 한다.
      // move_group은 관젉밧까지 충돌없는 경로를 계획한다
      const std::vector<double> joint_target = {
        motor_angles.theta1, motor_angles.theta2, motor_angles.theta3, motor_angles.theta4,
        motor_angles.theta5};
      move_group_->setJointValueTarget(joint_target);

      // plan()은 움직이지 않고 계획만 진행 충돌,리밋에 걸리면 실패하고 다음으로 이동
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group_->execute(plan);   // 이 가지의 계획을 실행
        RCLCPP_INFO(get_logger(), "%s 자세 도달(가지 %s)", label, elbow_up ? "up" : "down");
        approached = true;
        break;    // 성공한 가지에서 멈춤 (up 먼저 시도하므로 up이 되면 up으로)
      }
      RCLCPP_WARN(get_logger(), "가지 %s 도달 불가 (%.2f, %.2f, %.2f)", elbow_up ? "up" : "down", x,
        y, z);
    }
    return approached;
  }
  // 그리퍼를 이름 자세로 (open/close). MGI로 여닫기만 하고 파지 판정은 추후 예정
  void move_gripper(const char *named)
  {
    gripper_group_->setNamedTarget(named);
    gripper_group_->move();
    RCLCPP_INFO(get_logger(), "그리퍼 %s", named);
  }
  // 실제 일 - 지금은 자리표시자: 로그만 찍고 성공 반환 (MoveGroupInterface는 다음 스텝)
  void execute_move_to(const std::shared_ptr<GoalHandleMoveTo> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    RCLCPP_INFO(get_logger(), "move_to 실행 : %s", goal->target_name.c_str());

    // SRDF에 정의된 이름 자세로 목표 설정 (arm group init /home)
    move_group_->setNamedTarget(goal->target_name);
    // plan + execute (블로킹). SUCCESS면 성공. 벤더 qnode.cpp와 같은 판정
    const bool ok = (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);

    auto result = std::make_shared<MoveTo::Result>();
    result->success = ok;
    if (ok) {
      result->failure = make_failure(
        arm_interfaces::msg::ErrorCode::SUCCESS, "", "", goal->attempt);
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "move_to 성공 : %s", goal->target_name.c_str());
    } else {
      result->failure = make_failure(
        arm_interfaces::msg::ErrorCode::PLANNING_FAILED,
        arm_interfaces::msg::Stage::PLAN,
        "move() 실패 : " + goal->target_name, goal->attempt);
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(), "move_to 실패 : %s", goal->target_name.c_str());
    }
  }
  void execute_pick(const std::shared_ptr<GoalHandlePick> goal_handle)
  {
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
    const double grasp_depth = 0.01;
    const double grasp_yaw = std::remainder(obj_yaw + M_PI_2, M_PI);
    move_gripper("open");
    if (!move_to_pose(obj_x, obj_y, obj_z + approach_dz, approach_phi, "approach", grasp_yaw)) {
      goal_handle->abort(make_pick_result(false, goal->attempt));
      return;
    }
    if (!move_to_pose(obj_x, obj_y, obj_z - grasp_depth, approach_phi, "grasp", grasp_yaw)) {
      goal_handle->abort(make_pick_result(false, goal->attempt));
      return;
    }
    move_gripper("close");
    if (!move_to_pose(obj_x, obj_y, obj_z + approach_dz, approach_phi, "lift", grasp_yaw)) {
      goal_handle->abort(make_pick_result(false, goal->attempt));
      return;
    }
    RCLCPP_INFO(get_logger(), "pick 완료 : %s", goal->object_id.c_str());
    goal_handle->succeed(make_pick_result(true, goal->attempt));
  }
  std::shared_ptr<Pick::Result> make_pick_result(bool ok, uint8_t attempt)
  {
    auto result = std::make_shared<Pick::Result>();
    result->success = ok;
    result->failure = make_failure(
      ok ? arm_interfaces::msg::ErrorCode::SUCCESS :
           arm_interfaces::msg::ErrorCode::PLANNING_FAILED,
      ok ? "" : arm_interfaces::msg::Stage::APPROACH,
      ok ? "" : "pick 이동 실패", attempt);
    return result;
  }
  void execute_place(const std::shared_ptr<GoalHandlePlace> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    RCLCPP_INFO(get_logger(), "place 실행 : target=%s", goal->target_id.c_str());

    const double tgt_x = 0.169;
    const double tgt_y = 0.10;  // pick 자리와 다른 곳에
    const double tgt_z = 0.0475;
    const double approach_phi = -M_PI / 2;   // 그리퍼가 아래를 향하는 접근각
    const double approach_dz = 0.06;    // 물체 위 6cm 에서 접근

    if (!move_to_pose(tgt_x, tgt_y, tgt_z + approach_dz, approach_phi, "place-approach")) {
      goal_handle->abort(make_place_result(false, goal->attempt));
      return;
    }
    if (!move_to_pose(tgt_x, tgt_y, tgt_z, approach_phi, "place-lower")) {
      goal_handle->abort(make_place_result(false, goal->attempt));
      return;
    }
    move_gripper("open");
    if (!move_to_pose(tgt_x, tgt_y, tgt_z + approach_dz, approach_phi, "place-retreat")) {
      goal_handle->abort(make_place_result(false, goal->attempt));
      return;
    }
    RCLCPP_INFO(get_logger(), "place 완료 : %s", goal->target_id.c_str());
    goal_handle->succeed(make_place_result(true, goal->attempt));
  }
  std::shared_ptr<Place::Result> make_place_result(bool ok, uint8_t attempt)
  {
    auto result = std::make_shared<Place::Result>();
    result->success = ok;
    result->failure = make_failure(
      ok ? arm_interfaces::msg::ErrorCode::SUCCESS :
           arm_interfaces::msg::ErrorCode::PLANNING_FAILED,
      ok ? "" : arm_interfaces::msg::Stage::TRANSFER,
      ok ? "" : "place 이동 실패", attempt);
    return result;
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
