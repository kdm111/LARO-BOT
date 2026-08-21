#pragma once

#include <atomic>
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
#include <control_msgs/action/gripper_command.hpp>
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
  // ★ 2026-08-22 안전 종료. 진행 중 동작을 세우고 home 으로 옮긴 뒤 토크를 유지한
  //   채 반환한다(사용자 결정 ① - 토크를 끄면 중력으로 무너지는 팔이라 차단은
  //   전원 스위치의 몫). main.cpp 의 신호 처리에서 부른다.
  void safe_park();
  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg);

private:
  // move_to_pose의 결과 bool로는 못 닿음과 길이 막힘이 구분되지 않는다.
  // ★ 2026-08-22 TIMEOUT 추가 : 실행이 예산(exec_watchdog_sec) 안에 못 끝나
  //   watchdog 이 팔을 세운 경우. EXECUTION_TIMEOUT 으로 보고된다.
  enum class MoveResult {OK, UNREACHABLE, PLAN_FAILED, EXEC_FAILED, TIMEOUT};

  // ★ 2026-08-22 single-flight. 실행 중에는 어느 스킬이든 새 goal 을 거부한다.
  //   전에는 무조건 수락해서 detach 스레드 둘이 한 팔(move_group_)을 동시에
  //   조종할 수 있었다. 수락 시점에 compare_exchange 로 잡고(handle_goal_*),
  //   execute_* 가 끝나는 모든 경로에서 푼다(각 함수 첫 줄의 해제 가드).
  std::atomic<bool> busy_{false};
  // execute_* 함수 전체를 감싸 어느 return 으로 나가든 busy_ 를 푼다.
  struct BusyRelease
  {
    std::atomic<bool> & flag;
    ~BusyRelease() {flag = false;}
  };

  // 팔꿈치 가지 고정. 값이 있으면 그 가지만 쓴다.(물체를 쥐고 있는 동안)
  // grasp에서 down으로 잡고, lift에서 up으로 계획이 잡혀 팔이 통째로 뒤집힌다.
  std::optional<bool> locked_elbow_;
  // 실행에 성공한 가지. 이 값으로 잠근다.
  bool last_elbow_ = false;
  // ★ 2026-08-20 밤 : 쥠 판정 순간의 그리퍼 관절값. grasp_check 가 쥠에서 set,
  //   빈 손에서 reset. place 가 운반 중 낙하 임계를 이 값 기준으로 세운다 -
  //   유지 목표 되걸기(과부하 대책) 이후로는 낙하해도 관절이 0 까지 안 가기 때문이다.
  //   근거는 skill_place.cpp 운반 확인 주석. release·낙하 abort 에서도 reset.
  std::optional<double> held_q_;
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
  // ★ 2026-08-20 추가. 파지 판정용으로 gripper_controller 를 직접 부른다.
  //   MoveIt 을 거치지 않는 유일한 액추에이터 경로다 - 이유는 gripper.cpp grasp_check 주석.
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_cmd_client_;

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

  // ★ 2026-08-21 밤 wrist_canonical 추가. theta5 와 theta5±180도 는 대칭 그리퍼로는
  //   같은 닫힘축이지만 "손가락 자리"가 맞바뀐다. 실물 손끝은 TCP 에 대해 비대칭
  //   (-y 로 ~1.4cm, kHookYTrim)이라 자리가 바뀌면 비대칭 부호가 반전되어 자리마다
  //   다른 방향으로 부딪히는 산포가 됐다(hook 실측). true 면 j5 를 (-90,90]도로
  //   접어 항상 같은 손가락이 같은 쪽에 오게 한다. hook 이동에서 쓴다.
  MoveResult move_to_pose(
    double x, double y, double z, double phi, const char * label,
    std::optional<double> grasp_yaw = std::nullopt, bool wrist_canonical = false);
  // ★ 2026-08-20 추가. 물체를 든 구간만 느리게 간다.
  //   링이 운반 중에 두 번 빠졌다(파지는 0.2838 로 강했는데도). 매끄러운 원통을
  //   평평한 손가락 두 점으로 무는 구조라 옆으로 미는 힘에 약하다.
  //   전역으로 낮추면 빈 손 이동까지 느려지므로 든 구간만 낮춘다.
  //   근거값: 오늘 0.1/0.1 로 돌던 동안에는 블록 운반에서 미끄러짐이 없었고,
  //   0.3/0.45 로 올린 뒤 링이 빠지기 시작했다.
  void set_carry_speed(bool carrying);
  moveit::core::MoveItErrorCode move_gripper(const char * named);
  // 이름 자세 대신 관절값을 직접 준다. 물체마다 벌리는 폭이 다를 때 쓴다.
  moveit::core::MoveItErrorCode move_gripper_to(double pos);
  // ★ 2026-08-20 추가. 그리퍼 목표 하나를 gripper_controller 액션으로 직접 보낸다.
  //   수락까지만 확인하고 결과는 기다리지 않는다 - 이유는 gripper.cpp 주석.
  bool send_gripper_goal(double pos, const char * tag, const char * what);
  // ★ 2026-08-20 신설. 닫고 나서 "쥐었는가"를 한 번에 판정한다.
  //   닫기를 gripper_controller 액션으로 직접 보낸다 - MoveIt 경유는 못 믿는다.
  //   같은 날 밤부터는 판정 직후 목표를 "잰 값 − kGripPreload"로 되걸어 서보가
  //   계속 밀지 않게 한다(ID16 과부하 대책). 근거는 gripper.cpp 의 구현부 주석.
  bool grasp_check(double eps, const char * tag = "파지 판정");
  // 측정 전용 판정. 관절값만 임계와 견주고 그리퍼에 아무 명령도 안 보낸다.
  // place 의 운반 중 확인이 쓴다(재조임은 파지를 깨뜨린다 - skill_place.cpp 주석).
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
  // ★ 2026-08-20 pos_eps 0.001 -> 0.002. 0.001 은 sim 값이다 - 가제보는 위치가
  //   연속이라 임의로 작은 값을 써도 됐지만, 실물 DYNAMIXEL 은 0.0015339807878856412
  //   rad 단위로 양자화된다(xm430_w350.model). 1틱 떨림이 항상 0.001 을 넘어
  //   안정 카운터가 매번 0으로 리셋되고 정착 판정이 영영 안 난다.
  //   실측(green_block): 운반 중 확인이 5.000초 타임아웃 - 그때 그리퍼는 0.23317 로
  //   완전히 멈춰 있었다. 팔이 움직이며 부하가 변할 때 1틱 떨림이 생긴다.
  //   0.002 는 1틱(0.00153)보다 크고 2틱(0.00307)보다 작다 - 떨림은 허용하되
  //   실제 이동(20ms 에 여러 틱)은 여전히 "움직이는 중"으로 잡는다.
  //   08-08 에 속도 -> 위치로 바꾼 것과 같은 결의 실물 보정이다.
  std::optional<double> wait_gripper_settled(
    double pos_eps = 0.002, int stable_ms = 200, int timeout_ms = 5000);
};
