#include "arm_skills/params.hpp"   // kGripPreload
#include "arm_skills/skill_server.hpp"

// 그리퍼를 이름 자세로 (open/close). 반환값 = MoveIt 실행 결과 코드
// 이 코드를 파지 판정 재료로 사용
moveit::core::MoveItErrorCode SkillServer::move_gripper(const char * named)
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
// 그리퍼를 임의 폭으로. SRDF 이름(open/close)으로는 물체마다 다른 폭을 못 준다.
// SRDF는 third_party라 건드리지 않고 관절값을 직접 넣는다.
moveit::core::MoveItErrorCode SkillServer::move_gripper_to(double pos)
{
  // ★ 관절 이름으로 지정한다. 벡터로 넘기면 안 된다 - gripper 그룹은 관절이 둘이라
  //   (gripper_joint_1 + 수동 gripper_joint_2) 크기가 안 맞고, MoveIt 은 그럴 때
  //   목표 설정을 조용히 거부한 뒤 move()가 "제자리 이동"으로 SUCCESS 를 돌려준다.
  //   로그는 성공인데 그리퍼는 안 움직인다(2026-08-19 실측).
  if (!gripper_group_->setJointValueTarget("gripper_joint_1", pos)) {
    RCLCPP_ERROR(get_logger(), "그리퍼 목표 %.2f 설정 실패", pos);
    return moveit::core::MoveItErrorCode::FAILURE;
  }
  const auto code = gripper_group_->move();
  RCLCPP_INFO(
    get_logger(), "그리퍼 %.2f 로 : %s", pos,
    moveit::core::errorCodeToString(code).c_str());
  return code;
}

// 그리퍼 목표 하나를 gripper_controller 액션으로 보낸다. 수락까지만 확인하고
// 결과는 기다리지 않는다 - 결과는 안 오는 게 보통이다(실측: 물체를 문 채 60초
// 무응답. 부하 때 엔코더 1틱 떨림이 stall_velocity_threshold 를 넘어 stall
// 타이머가 계속 리셋된다. 같은 상황에서 6.7초 만에 온 적도 있다 - 들쭉날쭉).
// async_get_result 는 등록만 해 둔다 - 안 하면 결과가 도착했을 때 rclcpp_action
// 이 "결과 받을 사람 없음" 경고를 찍는다.
bool SkillServer::send_gripper_goal(double pos, const char * tag, const char * what)
{
  using GripperCommand = control_msgs::action::GripperCommand;
  using namespace std::chrono_literals;

  if (!gripper_cmd_client_ || !gripper_cmd_client_->wait_for_action_server(2s)) {
    RCLCPP_ERROR(get_logger(), "%s : gripper_controller 액션 서버 없음(%s)", tag, what);
    return false;
  }
  GripperCommand::Goal goal;
  goal.command.position = pos;
  goal.command.max_effort = 0.0;   // 0 = 컨트롤러 기본. 실제 상한은 서보 Goal Current 다.
  auto goal_future = gripper_cmd_client_->async_send_goal(goal);
  if (goal_future.wait_for(3s) != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "%s : %s(%.3f) 목표 수락 대기 실패", tag, what, pos);
    return false;
  }
  const auto handle = goal_future.get();
  if (!handle) {
    RCLCPP_ERROR(get_logger(), "%s : %s(%.3f) 목표 거부됨", tag, what, pos);
    return false;
  }
  (void)gripper_cmd_client_->async_get_result(handle);
  return true;
}

// 닫고 나서 쥐었는지 판정한다. 2026-08-20 신설, 같은 날 밤 "한 번 집고 넘어가기"로 개편.
//
// 왜 MoveIt(move_gripper("close"))이 아니라 액션을 직접 때리는가 - 실측 때문이다.
//   같은 "빈 손"인데
//     MoveIt close 직후          관절 0.2209 (임계 0.050 을 훌쩍 넘어 "쥠"으로 오판)
//     액션을 직접 보낸 뒤        관절 0.0015 (끝까지 닫힘)
//   MoveIt 경유는 손가락이 끝까지 닫히기 전에 끝나 버린다. 그 상태의 관절값은
//   "물었을 때"(0.20~0.22)와 구분이 안 된다. 못 집었는데 pick 이 성공으로 끝나고,
//   실패가 보고되지 않으니 agent 의 GRASP_FAILED -> REGRASP 도 영영 안 돈다.
//
// ★ 2026-08-20 밤 개편. 구 코드는 액션 결과를 10초까지 기다렸다 - 그 10초 내내
//   서보는 목표 0.0 을 향해 전류 한계로 갈렸고, 판정 뒤에도 close 가 걸린 채
//   운반해서 갈림이 이어졌다. 오늘 ID16 이 두 번 토크를 잃은(전원 보호 추정)
//   유력 원인이다. 어차피 결과가 와도 판정 재료는 "정착한 관절값"이었으므로 :
//     close 전송 -> 관절 정착 대기 -> 그 값으로 판정 -> 즉시 유지 목표로 되걸기
//   되걸기 : 쥠이면 "잰 값 − kGripPreload"(가볍게만 눌러 쥔다),
//            빈 손이면 잰 값 그대로(조임 해제 - 과닫힘 -0.018 도 여기서 풀린다).
//
// 판정 규칙 : 손가락이 끝까지(0 근처) 닫혔으면 사이에 아무것도 없다. held = q > eps.
//   실측 : 빈 손 0.0015 / 블록 0.2025 / 링 전체 물음 0.3267.
//   sim 은 접촉 해석에 따라 stall 신호가 안 날 수 있는데, 관절값 판정은 그와
//   무관하게 물체 두께에서 멈춘 값을 본다 - 두 환경에서 같은 식이 선다.
//   ★ 절댓값이 아니라 부호 있는 비교다. 물체가 있으면 관절값은 반드시 양수고,
//     음수는 닫힘 지점을 지나쳐 들어간 것 - 빈 손에서만 생긴다(실측 -0.0184 가
//     |x| 비교로 "쥠" 오판을 낸 적이 있다).
//
// 통신·정착이 실패하면 "빈 손"으로 돌려준다. 모르면 쥐었다고 말하지 않는다 -
// 없는 물체를 들고 place 로 넘어가는 것보다 pick 이 시끄럽게 실패하는 편이 낫다.
bool SkillServer::grasp_check(double eps, const char * tag)
{
  held_q_.reset();
  if (!send_gripper_goal(0.0, tag, "close")) {
    return false;
  }
  // 수락 -> 실물 손가락이 움직이기 시작할 때까지 지연이 있다(컨트롤러 주기 +
  // 시리얼 쓰기 + 서보 프로파일 시작). 그 사이에 정착 판정(200ms 무변화)이 먼저
  // 성립하면 "벌린 채"(kGripperOpen)를 재 버린다 - 아래 kStillOpenQ 와 이중 방어.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const auto q = wait_gripper_settled();
  if (!q.has_value()) {
    // 판정 불가. 다만 close(0.0)를 건 채 떠나면 물체를 물고 있을 때 계속 갈리므로
    // 마지막으로 본 관절값 쪽으로 목표를 되걸어 둔다.
    double seen_pos = 0.0;
    bool seen = false;
    {
      std::lock_guard<std::mutex> lock(joint_mutex_);
      seen_pos = gripper_pos_;
      seen = gripper_seen_;
    }
    if (seen) {
      send_gripper_goal(std::max(seen_pos - kGripPreload, kGripCloseFloor), tag, "hold(정착 실패)");
    }
    RCLCPP_ERROR(get_logger(), "%s : close 후 관절 정착 실패 - 쥠으로 보지 않는다", tag);
    return false;
  }
  // 어떤 물체도 이보다 클 수 없다(최대 실측 0.3267 - 링 전체 물음). 이 값이면 잰
  // 것은 물체가 아니라 아직 안 닫힌 손이다. "쥠"으로 두면 빈 손 lift 가 되고,
  // 유지 목표가 close 를 취소해 영영 안 닫힌다 - 실패로 처리하고 제자리에 묶는다.
  static constexpr double kStillOpenQ = 0.5;
  if (*q > kStillOpenQ) {
    send_gripper_goal(*q, tag, "hold(close 미반영)");
    RCLCPP_ERROR(
      get_logger(), "%s : 관절=%.4f - close 가 반영되지 않았다(벌린 채) - 쥠으로 보지 않는다",
      tag, *q);
    return false;
  }
  const bool held = *q > eps;
  // 하한이 0.0 이 아니라 kGripCloseFloor 다 - 0.0 클램프는 얇은 것(링 벽 0.017)
  // 에서 조임을 절반으로 깎아 낙하를 불렀다(2026-08-21, params.hpp 주석).
  const double hold = held ? std::max(*q - kGripPreload, kGripCloseFloor) : *q;
  if (!send_gripper_goal(hold, tag, "hold")) {
    RCLCPP_WARN(get_logger(), "%s : 유지 목표 되걸기 실패 - close(0.0)가 걸린 채다", tag);
  }
  if (held) {
    held_q_ = *q;
  }
  RCLCPP_INFO(
    get_logger(), "%s : 관절=%.4f (임계 %.4f) -> %s (유지 목표 %.4f)",
    tag, *q, eps, held ? "쥠" : "빈 손", hold);
  return held;
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
bool SkillServer::is_holding(double eps, const char * tag)
{
  const auto q = wait_gripper_settled();
  if (!q.has_value()) {
    RCLCPP_WARN(get_logger(), "%s 그리퍼 정착 실패 - 쥐었다고 판정하지 않는다", tag);
    return false;
  }
  const bool held = *q > eps;
  RCLCPP_INFO(
    get_logger(), "%s 판정 : 관절=%.4f (임계 %.4f) -> %s",
    tag, *q, eps, held ? "쥠" : "빈 손");
  return held;
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
std::optional<double> SkillServer::wait_gripper_settled(
  double pos_eps, int stable_ms, int timeout_ms)
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
