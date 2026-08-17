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
  const bool held = std::abs(*q) > eps;
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
