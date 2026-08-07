/*
같은 물리 점인데 좌표 원점은 다르다.
랜덤한 도달 가능한 자세 하나를 만들고 그 손끝을 두 solver의 프레임으로 각각 표현 -> 각자 자기 목표를 풀게 한다.
그 각각 solver를 풀게 만들고 KDL은 local minima로 일부 실패함. 게다가 KDL엔 방향 자유를 줌

정기구학으로 각도에서 위치를 내놓은다. 이걸로 목표를 만들고
위치를 통해 각도를 내놓는다. 이걸로 solver들이 해결할 것
*/
#include <chrono>  // 시간 측정
#include <cmath>
#include <cstdio>
#include <random>

#include <Eigen/Dense>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/jntarray.hpp>

#include <arm_kinematics/ik.hpp>
#include <arm_kinematics/kdl_chain.hpp>

using arm_kinematics::L1;
using arm_kinematics::L2;
using arm_kinematics::L3;

// 벤치 목표 하나 : 같은 물리 자세를 두 solver의 프레임으로 각각 표현.
struct BenchTarget
{
  double x;   // 우리 solver 입력 (base_link 기준 - solve_ik의 공개 형식)
  double y;
  double z;
  double phi;   // 접근각
  KDL::Frame kdl_goal;   // KDL solver 입력
  double tip_r;  // 진단용 : 손끝이 수평거리인가 음수면 팔이 뒤로 접힌다.

};
// IkSolution의 5각을 KDL 관절 배열
KDL::JntArray to_jntarray(const arm_kinematics::IkSolution & a)
{
  KDL::JntArray q(5);
  q(0) = a.theta1;
  q(1) = a.theta2;
  q(2) = a.theta3;
  q(3) = a.theta4;
  q(4) = a.theta5;
  return q;
}
// KDL 100% 결과가
double kdl_reach_error(
  KDL::ChainFkSolverPos_recursive & fk, const KDL::JntArray & q, const KDL::Frame & goal)
{
  KDL::Frame reached;
  fk.JntToCart(q, reached);
  const double dx = reached.p.x() - goal.p.x();
  const double dy = reached.p.y() - goal.p.y();
  const double dz = reached.p.z() - goal.p.z();
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}
// 난수 생성기 rng와 KDL_FK 솔버 fk를 참조(&)로 받는다. 참조라서 매 호출마다 같은 rng가 이어서 다음 난수를 뽑는다.
BenchTarget make_target(std::mt19937 & rng, KDL::ChainFkSolverPos_recursive & fk)
{
  // 각 관절이 어느 범위에서 랜덤일지 정의한다. uniform_real_distribution(a,b) = a와 b사이 실수를 하나 균등하게 뽑는 주사위
  // ★ 정의역 = 로봇 앞쪽 반공간. 뒤쪽은 애초에 카메라 시야 밖이라 물체를 못 찾는다.
  // 이 제한이 solve_ik의 뒤접힘 가드(ik.cpp px<0)와 정확히 맞물린다:
  // cos(th1)>0이면 px = tip.r*cos(th1) 의 부호 = tip.r의 부호 -> 가드가 뒤접힘만 정확히 잡는다.
  // (전 범위로 뽑으면 가드가 정상 목표까지 죽여 성공률이 45%로 무너진다. 2026-08-07 실측)
  std::uniform_real_distribution<double> yaw(-M_PI / 2, M_PI / 2);   // 앞쪽 반
  std::uniform_real_distribution<double> shoulder(-1.0, 1.0);  // +-1 rad
  std::uniform_real_distribution<double> elbow(0.3, 2.0);   // 0.3~2.0 양수이고 쫙 편 특이 자세(0)와 완전 접힘(n)을 피함
  std::uniform_real_distribution<double> approach(-M_PI / 2 - 0.4, -M_PI / 2 + 0.4);  // phi(접근각), -n/2 아래 근처 +-0.4 위에서 아래로 집는 각도들

  const double th1 = yaw(rng);  // 실제로 yaw 주사위를 rng로 한번 굴려라.
  const double th2 = shoulder(rng);
  const double th3 = elbow(rng);
  const double phi = approach(rng);
  const double th4 = phi - th2 - th3;

  BenchTarget t;  // 결과 담을 구조체를 만들고
  t.phi = phi;  // phi 저장

  // 프레임 목표 : 기하 FK 손끝(r,z) -> wolder(x,y,z)
  auto tip = arm_kinematics::get_forward_kinematics(th2, th3, th4, L1, L2, L3); // 정역학으로 어디 위치에 가는 지 계산
  // 어깨 기준 (r,z) -> base_link 기준(x,y,z)
  t.x = tip.r * std::cos(th1) + arm_kinematics::BASE_TO_SHOULDER_X;
  t.y = tip.r * std::sin(th1);  // y축은 오프셋 없음
  t.z = tip.z + arm_kinematics::BASE_TO_SHOULDER_Z;
  t.tip_r = tip.r;

  arm_kinematics::IkSolution g{th1, th2, th3, th4, 0.0, true};
  fk.JntToCart(to_jntarray(arm_kinematics::to_motor_angles(g)), t.kdl_goal);

  return t;
}
// (삭제 2026-08-07) get_position_error — 채점은 전부 독립 오라클 kdl_reach_error가 한다.
// 한 번도 호출되지 않았고, base_link 전환 후에는 t.x에 어깨 오프셋이 섞여 값도 틀렸다.
int main()
{
  KDL::Chain chain = arm_kinematics::build_omx_chain();
  KDL::ChainFkSolverPos_recursive fk(chain);

  // KDL 수치 IK : 방향 가중치 0 = position-only (벤더 position_only_ik 재현)
  Eigen::Matrix<double, 6, 1> weights;
  weights << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
  KDL::ChainIkSolverPos_LMA kdl_ik(chain, weights);

  std::mt19937 rng(1);   // 고정 시드 삽입
  const int N = 5000;
  const double eps = 1e-3;   // 1mm 이내면 도달 처리

  int solve_ik_ok = 0, kdl_ok = 0;
  int solve_ik_unreachable = 0;   // sol.reachable == false  공식 밖
  int solve_ik_missed = 0;   // reachable인데 실제론 닿지 않는 곳
  int missed_neg_r = 0;
  // 가드(px<0)에 잘렸는데 tip.r는 양수 = 뒤접힘이 아닌 정상 목표를 죽인 것 = 오폭
  int killed_valid = 0;
  double solve_ik_total = 0.0, solve_ik_us_max = 0.0;
  double kdl_total = 0.0, kdl_us_max = 0.0;

  for (int i = 0; i < N; ++i) {
    BenchTarget t = make_target(rng, fk);

    // solve-ik 해석 ik
    const auto t0 = std::chrono::steady_clock::now();
    arm_kinematics::IkSolution sol = arm_kinematics::solve_ik(t.x, t.y, t.z, t.phi);
    const auto t1 = std::chrono::steady_clock::now();
    const double solve_ik_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    solve_ik_total += solve_ik_us;
    if (solve_ik_us > solve_ik_us_max) {solve_ik_us_max = solve_ik_us;}
    const KDL::JntArray solve_ik_q = to_jntarray(arm_kinematics::to_motor_angles(sol));
    if (!sol.reachable) {
      ++solve_ik_unreachable;
      if (t.tip_r > 0.0) {++killed_valid;}   // 뒤접힘이 아닌데 잘렸다
    } else if (kdl_reach_error(fk, solve_ik_q, t.kdl_goal) < eps) {
      ++solve_ik_ok;
    } else {
      ++solve_ik_missed;
      if (t.tip_r < 0.0) {
        ++missed_neg_r;
      }
    }

    // KDL 수치 IK
    KDL::JntArray q_init(5);  // 출발점 = 0 (모터 영점)
    KDL::JntArray q_out(5);  // 담는 곳
    const auto t2 = std::chrono::steady_clock::now();
    kdl_ik.CartToJnt(q_init, t.kdl_goal, q_out);
    const auto t3 = std::chrono::steady_clock::now();
    const double kdl_us = std::chrono::duration<double, std::micro>(t3 - t2).count();
    kdl_total += kdl_us;
    if (kdl_us > kdl_us_max) {kdl_us_max = kdl_us;}
    if (kdl_reach_error(fk, q_out, t.kdl_goal) < eps) {++kdl_ok;}
  }
  std::printf("목표 %d개 채점 KDL FK로 1mm이내 도달\n\n", N);
  std::printf("       성공률    평균시간    최악시간\n");
  std::printf("solve_ik  %5.1f%% %9.3f us %9.3f us\n",
              100.0 * solve_ik_ok / N, solve_ik_total / N, solve_ik_us_max);
  std::printf("kdl_ik  %5.1f%% %9.3f us %9.3f us\n",
              100.0 * kdl_ok / N, kdl_total / N, kdl_us_max);
  std::printf("solve_ik  공식 밖 %d, 오류 %d 중 tip.r <0: %d\n",
              solve_ik_unreachable, solve_ik_missed, missed_neg_r);
  // 가드 조준 진단 : 잘린 것 중 몇 개가 뒤접힘이 아닌 정상 목표였나
  std::printf("가드 오폭   공식 밖 %d 중 tip.r>0(정상인데 잘림): %d\n",
              solve_ik_unreachable, killed_valid);
  return 0;
}
