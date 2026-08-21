#include "arm_skills/params.hpp"
#include "arm_skills/skill_server.hpp"

void SkillServer::execute_place(const std::shared_ptr<GoalHandlePlace> goal_handle)
{
  const BusyRelease release{busy_};   // 어느 return 으로 나가든 single-flight 해제
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
  // 물체마다 다르다(params.hpp 주석). 단 놓으러 갈 때는 물체를 쥐고 있으므로
  // 너무 낮게 접근하면 쥔 물체가 테이블에 끌린다 - 6cm 를 하한으로 둔다.
  const double approach_dz = std::max(pit->second.approach_dz, 0.06);

  // 놓는 점 : pinch 는 목표 중심 그대로(TCP = 물체 중심).
  // ★ 2026-08-21 : hook 물체는 잡을 때 TCP 가 링 중심에서 +y 로 kHookHalfSpan
  //   비껴 있다(skill_pick.cpp 파지점 주석) - 링은 TCP 의 -y 쪽에 매달려 있다.
  //   TCP 를 목표 중심에 두면 링이 그만큼 -y 로 밀려 놓인다(옛 벽 파지의
  //   "실측 11cm 이탈"과 같은 원리). 잡을 때와 같은 오프셋·손목각(0)으로 상쇄한다.
  const GraspSpec & ps = pit->second;
  // ★ 2026-08-22 감싸기 전환 : 링이 TCP 의 -y 쪽 트림만큼에 매달린다(감싸기는
  //   중심 물음이라 벽 오프셋 몫이 없다). 상쇄도 트림만 남긴다.
  // ★ +y 영역 팔 편향 보정(arm_y_bias, params.hpp 주석). place 는 물체를 못 보는
  //   맹목 조준이라 편향이 그대로 착지 오차가 된다 - 실측 직선으로 상쇄한다.
  const double put_x = tgt_x;
  const double put_y = tgt_y + arm_y_bias(tgt_y) + (ps.hook ? kHookYTrim : 0.0);
  const std::optional<double> put_yaw =
    ps.hook ? std::optional<double>(0.0) : std::nullopt;

  if (const auto r = move_to_pose(
      put_x, put_y, tgt_z + approach_dz, approach_phi, "place-approach", put_yaw, ps.hook);
    r != MoveResult::OK)
  {
    goal_handle->abort(
      make_place_result(
        false, goal->attempt, code_of(r),
        arm_interfaces::msg::Stage::TRANSFER, detail_of(r)));
    return;
  }
  // transfer 파지 확인 : 재조임 없이 "측정만" 한다. 한때 grasp_check(닫고 나서
  // 판정)로 바꿨다가 되돌렸다 - 확인하는 행위가 파지를 깨뜨렸다(2026-08-20 실측).
  //     place 직전 0.0138(제대로 물림) -> grasp_check 의 close 후 0.0031(빠짐)
  //   링은 원형이라 닫는 힘이 옆으로 밀어낸다(params.hpp blue_ring 주석과 같은 현상).
  //   블록은 단단히 막혀 더 조여도 안 빠지지만 링은 빠진다. 이미 쥐고 있는 물체를
  //   다시 조일 이유가 없다 - pick 과 달리 여기서 close 는 얻는 것 없이 위험만 있다.
  // ★ 2026-08-20 밤, 낙하 임계를 물체별 hold_eps 에서 "쥔 값 기준"으로 바꿨다.
  //   구 근거("낙하면 손가락이 끝까지 닫혀 q 0.32 -> 0.0001")는 close(0.0)를 문 채
  //   다니던 때 이야기다. 지금은 grasp_check 가 목표를 쥔 값 − kGripPreload 로
  //   되걸므로(ID16 과부하 대책) 낙하하면 손가락이 0 이 아니라 그 목표에서 멈춘다.
  //   쥐고 있으면 물체에 막혀 목표보다 kGripPreload 만큼 벌어져 있다 - 둘의
  //   한가운데(쥔 값 − kGripPreload/2)로 가른다. 쥔 값을 모르면(스킬 서버 재시작
  //   직후의 place 단독 호출) 물체별 hold_eps 로 후퇴한다.
  const double drop_eps = held_q_.has_value() ?
    std::max(ps.hold_eps, *held_q_ - kGripPreload / 2.0) :
    ps.hold_eps;
  if (!is_holding(drop_eps, "운반 중 파지 확인")) {
    locked_elbow_.reset();
    held_q_.reset();
    set_carry_speed(false);   // 놓쳤으니 더는 든 손이 아니다
    goal_handle->abort(
      make_place_result(
        false, goal->attempt, arm_interfaces::msg::ErrorCode::GRIPPER_EMPTY,
        arm_interfaces::msg::Stage::TRANSFER, "운반 중 물체를 놓쳤다."));
    return;
  }
  // 놓는 높이. TCP 를 지면에서 4cm 로 내린 뒤 손을 편다(2026-08-20 사용자 결정).
  //
  // 경위 - 세 번째 판이다.
  //   ① 원래   : tgt_z(물체별 place_z)까지 내려가 살며시 놓았다. 정확했지만
  //              내려가는 동작에서 물체가 꼬였다.
  //   ② 그 다음: place-lower 를 아예 없애고 approach 높이에서 놓았다. 꼬임은
  //              사라졌는데 낙차가 6cm 이라 링이 2.3cm 밀려 구역을 벗어났다.
  //   ③ 지금   : 그 사이의 한 높이로 고정한다. 내려가긴 하되 바닥까지는 안 간다.
  //
  // 물체별 place_z 가 아니라 고정값인 것이 요점이다 - 지면 기준 한 높이에서 놓으면
  // 물체가 무엇이든 낙차가 "TCP 4cm 에서 그 물체 밑면까지"로 일정해진다.
  //   블록 : 밑면이 TCP-3.5cm -> 낙차 0.5cm
  //   링   : 밑면이 TCP-1.0cm -> 낙차 3.0cm
  // ⚠️ 링 쪽 낙차가 아직 크다. 링이 계속 밀리면 이 값을 물체별로 나눠야 한다.
  // ★ 2026-08-22 새벽 : 그 경고가 현실이 됐다 - work 에 놓은 링이 모서리로
  //   착지해 종이 밖 (0.35, -0.35)까지 굴러갔다(도달 범위 밖). hook 물체는
  //   낙차를 2cm 로 줄인다(TCP 3cm). 감싸기 파지의 링 밑면은 TCP-1cm 다.
  constexpr double kReleaseZ = 0.04;
  // ★ 낙차 2cm(0.03)로도 굴렀다(work -> r=0.26 지점, 두 번째) - 1.2cm 로 더 내린다.
  const double release_z = ps.hook ? 0.022 : kReleaseZ;
  if (const auto r = move_to_pose(
      put_x, put_y, release_z, approach_phi, "place-release", put_yaw, ps.hook);
    r != MoveResult::OK)
  {
    goal_handle->abort(
      make_place_result(
        false, goal->attempt, code_of(r),
        arm_interfaces::msg::Stage::TRANSFER, detail_of(r)));
    return;
  }
  // ★ 2026-08-20 밤 : "open"(SRDF, =1.0) 대신 kGripperOpen. 1.0 은 기구 끝점
  //   (실측 0.9925) 너머라 열어 둔 내내 서보가 끝점을 밀었다 - ID16 토크 상실
  //   (전원 보호 추정)의 유력 원인 중 하나. 근거는 params.hpp kGripperOpen 주석.
  move_gripper_to(kGripperOpen);
  locked_elbow_.reset();   // 그리퍼 놓음 잠금 해제
  held_q_.reset();         // 손을 폈다 - 쥔 값도 함께 무효
  set_carry_speed(false);  // 손을 폈다. 물러나는 길은 빈 손 속도로 간다.
  if (const auto r = move_to_pose(
      put_x, put_y, std::max(tgt_z + approach_dz, ps.retreat_z),
      approach_phi, "place-retreat", put_yaw, ps.hook);
    r != MoveResult::OK)
  {
    goal_handle->abort(
      make_place_result(
        false, goal->attempt, code_of(r),
        arm_interfaces::msg::Stage::RETREAT, detail_of(r)));
    return;
  }
  // ★ 2026-08-21 : 놓고 물러난 뒤 home 으로 복귀한다(사용자 결정 - place 가 스스로
  //   마무리하는 편이 깔끔하다). home 은 SRDF 이름 자세다(skill_move_to 와 같은 경로).
  //   복귀가 실패해도 place 는 성공으로 보고한다 - 물체는 이미 놓였고, 여기서
  //   abort 하면 agent 가 "놓기 실패"로 읽어 멀쩡한 배달을 복구하려 든다.
  if (!move_group_->setNamedTarget("home") ||
    move_group_->move() != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_WARN(get_logger(), "place 후 home 복귀 실패 - place 자체는 성공으로 보고한다");
  }
  RCLCPP_INFO(get_logger(), "place 완료 : %s", goal->target_id.c_str());
  goal_handle->succeed(make_place_result(true, goal->attempt));
}
