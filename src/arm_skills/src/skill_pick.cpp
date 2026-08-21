#include <algorithm>

#include "arm_skills/params.hpp"
#include "arm_skills/skill_server.hpp"

void SkillServer::execute_pick(const std::shared_ptr<GoalHandlePick> goal_handle)
{
  const BusyRelease release{busy_};   // 어느 return 으로 나가든 single-flight 해제
  locked_elbow_.reset();   // 새로운 pick 시작 = 이전 잠금 무효
  const auto goal = goal_handle->get_goal();
  RCLCPP_INFO(get_logger(), "pick 실행 : object=%s", goal->object_id.c_str());

  // ★ 2026-08-21 : 집기 전에 "지금 이후에 관측된" 씬을 기다린다(사용자 결정 -
  //   REGRASP 는 홈 복귀 -> 재스캔 -> 재시도). latest_scene_ 는 팔이 시야를 가리던
  //   때의 프레임일 수 있고, 그 좌표는 오늘 두 번 연속 옆을 헛짚게 했다(마스크가
  //   조각나 큰 blob 의 중심만 발행 - perception 로그 "blob 2개"). agent 의
  //   _do_recover 주석은 "실행 시점에 갱신된 좌표를 쓴다"고 하는데, 검출기가 ~1Hz 라
  //   그건 보장이 아니라 운이었다. 여기서 기다리면 보장이 된다(보통 1초 안).
  //   3초(약 3프레임) 안에 안 오면 인지가 멈춘 것 - 낡은 좌표로 조준하느니
  //   시끄럽게 실패한다.
  const rclcpp::Time pick_start = now();
  bool scene_fresh = false;
  for (int i = 0; i < 60 && !scene_fresh; ++i) {
    {
      std::lock_guard<std::mutex> lock(scene_mutex_);
      scene_fresh = rclcpp::Time(latest_scene_.header.stamp) >= pick_start;
    }
    if (!scene_fresh) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
  if (!scene_fresh) {
    goal_handle->abort(
      make_pick_result(
        false, goal->attempt, arm_interfaces::msg::ErrorCode::OBJECT_NOT_FOUND,
        arm_interfaces::msg::Stage::PLAN, "재스캔 대기 3초 초과 - /scene_state 갱신 없음"));
    return;
  }

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
  const double approach_dz = gs.approach_dz;   // 물체마다 다르다(params.hpp 주석)

  // 파지점 : pinch 는 물체 중심 - 통째로 가로질러 문다.
  // ★ 2026-08-20 : 속이 빈 물체를 "벽"으로 옮겨 물던 분기를 지웠다(벽 1.25cm 가
  //   카메라 오차보다 작았다). ★ 2026-08-21 : hook 으로 비낌 조준이 부활했다 -
  //   단 표적이 벽이 아니라 "구멍"이다. 중심에서 +y 로 손끝 반간격(3.25cm 실측)을
  //   옮기면 안쪽 손끝이 구멍 정중앙에 내려온다. 허용 오차 = 구멍 반지름 ±1.75cm
  //   로, 보정된 조준(±0.5cm)의 세 배가 넘는 표적이다. 중앙 조준 훅은 손끝 반경
  //   3.25 > 링 바깥반경 3.0 이라 구멍에 "우연히만" 빠졌다(오늘 성공 1 실패 2).
  //   방향은 +y(shelf 쪽) 고정, 닫힘각 0.
  //   ★ 닫힘축 확정(2026-08-21 저녁, 사용자 육안 실측) : yaw 0 에서 손가락은
  //     "좌우(y 축)"로 벌어진다. j5=0 으로 팔을 뻗어 세워 두고 직접 봤다 -
  //     옛 08-19 주석("+y 오프셋에 yaw 0 = 반경 닫힘")이 옳았다.
  //     그 전에 블록 성공(긴변 y 향, grasp_yaw -2.4)에서 "yaw 0 = x 닫힘"을
  //     역산했던 것은 틀린 추론이었다 - 그 가설로 -x 오프셋을 시도해 손가락이
  //     링의 -x 쪽 허공에서 좌우로 닫았다(관절 -0.0015, 완전 빈 닫힘).
  //   오프셋은 벌어짐 축(y)과 같은 축이어야 하고, 크기는 벽 한가운데
  //   (kHookPinchOffset, params.hpp 주석)다 - 아래쪽(-y) 손끝은 구멍 안,
  //   위쪽 손끝은 링 밖에 서서 +y 벽을 반경 방향으로 동시에 문다.
  // ★ 2026-08-22 : 벽 물음(kHookPinchOffset 비낌 조준)을 버리고 "감싸기"로 전환.
  //   넓은 진입(0.6, 간격 7.5cm)에서 중심(+비대칭 트림)을 겨냥하면 두 손끝이
  //   링 바깥 양쪽 ~0.75cm 씩 균형으로 서고, 닫으면 링 전체가 두껍게 물린다
  //   (관절 ~0.32). 이 모드가 오늘 유일하게 "집고 + 들고"를 다 통과했다
  //   (1mm 깊이 + 수직 스텝 lift + 온전한 조임의 합작). 벽 물음은 벌림-오프셋이
  //   짝이라 벌림을 키우면 기하가 깨졌고, 물음 자체도 들 때 자주 빠졌다.
  const double grasp_x = obj_x + (gs.hook ? kHookXTrim : 0.0);
  const double grasp_y = obj_y + (gs.hook ? kHookYTrim : 0.0);
  const double grasp_yaw = gs.hook ? 0.0 : std::remainder(obj_yaw + M_PI_2, M_PI);
  RCLCPP_INFO(
    get_logger(), "파지점 (%.3f, %.3f) z=%.3f yaw=%.1f도",
    grasp_x, grasp_y, obj_z - gs.depth, grasp_yaw * 180.0 / M_PI);
  // ★ 2026-08-22 : 내려가기 전에 "들 수 있는 자리인지"를 먼저 푼다.
  //   r=0.258 에서 grasp(z=0.001)는 풀리는데 lift(z=0.10)는 IK 불가였다 -
  //   물체를 문 채 4.5cm 에서 갇혀 pick 실패 + 수동 구출이 필요했다.
  //   집기 전에 lift 목표의 도달을 확인하면 물체를 건드리지 않고 실패한다.
  {
    const double z_lift = std::max(obj_z + approach_dz, gs.retreat_z);
    bool liftable = false;
    for (bool elbow_up : {false, true}) {
      const auto g = arm_kinematics::solve_ik(
        grasp_x, grasp_y, z_lift, approach_phi, elbow_up, grasp_yaw);
      if (g.reachable) {
        liftable = true;
        break;
      }
    }
    if (!liftable) {
      goal_handle->abort(
        make_pick_result(
          false, goal->attempt, arm_interfaces::msg::ErrorCode::UNREACHABLE,
          arm_interfaces::msg::Stage::PLAN, "잡을 수는 있으나 들 수 없는 자리 - 내려가지 않음"));
      return;
    }
  }

  // 물체마다 벌리는 폭이 다르다(params.hpp의 open_pos 주석 참조).
  // ★ 2026-08-21 hook 물체(링)는 "반만(open_pos=0.5) 벌리고" 내려간다 - 근거는
  //   params.hpp GraspSpec::hook 주석. 벌림은 MoveIt 이 아니라 직접 보내고
  //   정착값까지 확인한다. 확인 없이 내려가면 직전 벌림(활짝 등) 그대로 내려가
  //   손끝이 구멍/벽이 아닌 엉뚱한 곳에 선다.
  if (gs.hook) {
    bool entry_ok = send_gripper_goal(gs.open_pos, "hook 진입", "entry-half-open");
    std::optional<double> eq;
    if (entry_ok) {
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      eq = wait_gripper_settled();
    }
    if (!eq.has_value() || std::abs(*eq - gs.open_pos) > 0.10) {
      goal_handle->abort(
        make_pick_result(
          false, goal->attempt, arm_interfaces::msg::ErrorCode::GRASP_FAILED,
          arm_interfaces::msg::Stage::GRASP, "hook 진입 반벌림 확인 실패 - 내려가지 않음"));
      return;
    }
  } else {
    move_gripper_to(gs.open_pos);
  }
  if (const auto r = move_to_pose(
      grasp_x, grasp_y, obj_z + approach_dz, approach_phi, "approach", grasp_yaw, gs.hook);
    r != MoveResult::OK)
  {
    goal_handle->abort(
      make_pick_result(
        false, goal->attempt, code_of(r),
        arm_interfaces::msg::Stage::APPROACH, detail_of(r)));
    return;
  }
  // ★ 2026-08-21 밤 : hook 은 마지막 하강을 1cm 이하 구간으로 쪼개 사실상
  //   수직으로 내려간다(사용자 관찰 - 내려오는 손끝이 링 벽을 옆에서 쳤다).
  //   move_to_pose 는 관절 공간 점대점이라 approach -> grasp 2.7cm 를 한 번에
  //   가면 TCP 가 옆으로 휘고, 그 휨이 구멍 조준 여유(±1cm 미만)를 넘는다.
  //   구간을 짧게 나누면 구간마다 거의 직선이라 휨이 무시할 만하다.
  //   블록(pinch)은 기존 한 번 하강 그대로 - 그 경로로 잘 되고 있다.
  {
    const double z_hi = obj_z + approach_dz;
    const double z_lo = obj_z - gs.depth;
    const int steps = gs.hook ?
      std::max(1, static_cast<int>(std::ceil((z_hi - z_lo) / 0.01))) : 1;
    for (int i = 1; i <= steps; ++i) {
      const double z = z_hi + (z_lo - z_hi) * i / steps;
      if (const auto r = move_to_pose(
          grasp_x, grasp_y, z, approach_phi, "grasp", grasp_yaw, gs.hook);
        r != MoveResult::OK)
      {
        goal_handle->abort(
          make_pick_result(
            false, goal->attempt, code_of(r),
            arm_interfaces::msg::Stage::GRASP, detail_of(r)));
        return;
      }
    }
  }
  // close의 결과가 곧 파지 판정 lift전에 분기해야함.
  // 빈 손으로 들어올리면 성공한 pick이 되어 GRASP_FAILED가 영원히 잡히지 않는다.
  // ★ 2026-08-20 : move_gripper("close") + is_holding() 을 grasp_check() 하나로 합쳤다.
  //   닫기를 MoveIt 으로 보내면 손가락이 끝까지 닫히기 전에 끝나서, 빈 손인데도
  //   관절이 0.22 로 읽혀 "쥠"이 됐다(실측). grasp_check 는 컨트롤러를 직접 부른다.
  if (!grasp_check(gs.hold_eps)) {
    RCLCPP_WARN(get_logger(), "파지 실패 : %s (그리퍼가 끝까지 닫힘)", goal->object_id.c_str());
    // 여기서 열지 않는다. 판정이 틀렸을 때 (실제로 쥐고 있을 때 물건이 떨어진다.)
    // move_gripper("open");   // 다음 시도를 위해 열어둠 REGRASP
    // ★ 2026-08-21 : 위로 물러나 home 까지 돌아간 뒤에 abort 한다(사용자 결정 -
    //   REGRASP 는 "홈 복귀 -> 재스캔 -> 재시도"). 팔이 물체 옆에 선 채 abort 하면
    //   재시도의 검출을 팔이 가려 같은 헛조준을 반복한다(2026-08-21 실측 두 번).
    //   먼저 retreat_z 로 곧장 위로 빼서 방금 못 집은 물체를 쓸고 가지 않게 한다.
    //   복귀 실패는 로그만 - abort 코드는 GRASP_FAILED 를 유지해야 agent 전략이
    //   REGRASP 로 남는다.
    if (move_to_pose(
        grasp_x, grasp_y, std::max(obj_z + approach_dz, gs.retreat_z),
        approach_phi, "grasp-fail-retreat", grasp_yaw, gs.hook) != MoveResult::OK ||
      !move_group_->setNamedTarget("home") ||
      move_group_->move() != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "파지 실패 후 home 복귀 실패 - 선 자리에서 abort");
    }
    goal_handle->abort(
      make_pick_result(
        false, goal->attempt,
        arm_interfaces::msg::ErrorCode::GRASP_FAILED,
        arm_interfaces::msg::Stage::GRASP,
        "그리퍼가 끝까지 닫힘 - 물체를 못잡음"));
    return;
  }
  locked_elbow_ = last_elbow_;  // 쥐었다. =grasp에 쓴 가지로 고정
  set_carry_speed(true);        // 여기부터 물체를 들고 간다. place 의 open 에서 푼다.
  RCLCPP_INFO(get_logger(), "가지 %s 고정(파지 중)", *locked_elbow_ ? "up" : "down");

  // ★ 2026-08-21 밤 : hook 은 첫 4cm 를 수직 스텝으로 뺀다(사용자 관찰 - 집기가
  //   성공해도 드는 과정에서 떨어뜨린다). 관절 공간 lift 는 하강과 똑같이 옆으로
  //   휘는데, 얇은 벽 물음은 조임을 더 키울 수 없어(유지 하한 -0.02 = 실물 과닫힘
  //   한계) 옆 쓸림이 그대로 낙하가 된다. 링이 테이블을 벗어날 때까지만 수직으로
  //   가고, 그 위부터는 기존 lift 한 번에 간다.
  if (gs.hook) {
    for (double z = obj_z - gs.depth + 0.01; z <= 0.045; z += 0.01) {
      if (const auto r = move_to_pose(
          grasp_x, grasp_y, z, approach_phi, "lift-step", grasp_yaw, gs.hook);
        r != MoveResult::OK)
      {
        goal_handle->abort(
          make_pick_result(
            false, goal->attempt, code_of(r),
            arm_interfaces::msg::Stage::LIFT, detail_of(r)));
        return;
      }
    }
  }

  if (const auto r = move_to_pose(
      // ★ 2026-08-19: 들어올리는 높이는 접근 높이와 따로 둔다. approach_dz 를 링용으로
      //   2.5cm 까지 낮췄더니 lift 도 같이 낮아져 링이 테이블에 끌렸다(TCP 0.031 -
      //   링은 TCP 보다 2cm 아래라 바닥에서 1cm). retreat_z 주석의 "납작한 물체에서
      //   6.6cm 밖에 안 올라간다"와 같은 현상이 lift 에서 난 것이다.
      //   retreat_z(링 0.10)까지 올린다. 6cm 로는 링이 여전히 낮게 끌린다(사용자 실물 확인).
      //   ※ 높이가 크게 바뀌면 IK 가 다른 손목 해를 골라 j5 가 뒤집히는 경우가 있다
      //   (한 번 관측: +1.346 -> -1.352). 재발하면 손목 가지도 잠가야 한다.
      grasp_x, grasp_y, std::max(obj_z + approach_dz, gs.retreat_z), approach_phi,
      "lift", grasp_yaw, gs.hook);
    r != MoveResult::OK)
  {
    goal_handle->abort(
      make_pick_result(
        false, goal->attempt, code_of(r),
        arm_interfaces::msg::Stage::LIFT, detail_of(r)));
    return;
  }
  // ★ 2026-08-22 : lift 직후 재판정(사용자 결정 - "낙하는 pick 이후에 체크").
  //   드는 중 낙하는 지금까지 place 의 운반 확인에서야 잡혔다 - pick 이 성공으로
  //   끝나 버려서다. 판정 기준은 place 와 같다(쥔 값 - kGripPreload/2, 측정 전용).
  //   낙하면 home 으로 접고 GRIPPER_EMPTY 로 abort - agent 의 REGRASP(홈 복귀 +
  //   재스캔 + 재시도)가 그대로 받는다. GRASP_FAILED(못 잡음)와 코드가 달라
  //   "잡았다 떨어짐"이 로그에서도 구분된다.
  const double lift_eps = held_q_.has_value() ?
    std::max(gs.hold_eps, *held_q_ - kGripPreload / 2.0) : gs.hold_eps;
  if (!is_holding(lift_eps, "lift 후 파지 확인")) {
    locked_elbow_.reset();
    held_q_.reset();
    set_carry_speed(false);
    if (!move_group_->setNamedTarget("home") ||
      move_group_->move() != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "lift 낙하 후 home 복귀 실패 - 선 자리에서 abort");
    }
    goal_handle->abort(
      make_pick_result(
        false, goal->attempt, arm_interfaces::msg::ErrorCode::GRIPPER_EMPTY,
        arm_interfaces::msg::Stage::LIFT, "들어올리는 중 물체를 놓쳤다"));
    return;
  }
  RCLCPP_INFO(get_logger(), "pick 완료 : %s", goal->object_id.c_str());
  goal_handle->succeed(make_pick_result(true, goal->attempt));
}
