// ik값의 검수
#include <gtest/gtest.h> // google의 테스트 프레임워크 불러오기
#include <cmath>
#include "arm_kinematics/ik.hpp"

TEST(ReachDistance, CASE_One) { // 매크로 첫째 묶음 이름, 둘째 케이스 이름
  // 부동소수점 연산은 EXPECT_EQ를 쓰면 안된다. 부동소수점은 틀릴 수도 있으니까
  EXPECT_NEAR(arm_kinematics::get_reach_distance(3.0, 4.0), 5.0, 1e-9);
}

TEST(ReachDistance, CASE_Zero) {
  EXPECT_NEAR(arm_kinematics::get_reach_distance(0.0, 0.0), 0.0, 1e-9);
}

TEST(BaseAngle, PositiveX)
{
  EXPECT_NEAR(arm_kinematics::get_base_angle(1.0, 0.0), 0.0, 1e-9);
}

TEST(BaseAngle, PositiveY)
{
  EXPECT_NEAR(arm_kinematics::get_base_angle(0.0, 1.0), M_PI / 2, 1e-9);
}

TEST(BaseAngle, SecondQuadrant)
{
  // (-1.0, 1.0) > 135(deg) atan(py/px) 였으면 -45도로 틀렸을 자리
  EXPECT_NEAR(arm_kinematics::get_base_angle(-1.0, 1.0), 3 * M_PI / 4, 1e-9);
}
TEST(ElbowAngle, RightAngle)
{
  auto result = arm_kinematics::get_elbow_angle(std::sqrt(2.0), 1.0, 1.0);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result.value(), M_PI / 2, 1e-9);
}
TEST(ElbowAngle, StraitArm)
{
  auto result = arm_kinematics::get_elbow_angle(2.0, 1.0, 1.0);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result.value(), 0.0, 1e-9);
}
TEST(ElbowAngle, Unreachable)
{
  auto result = arm_kinematics::get_elbow_angle(3.0, 1.0, 1.0);
  EXPECT_FALSE(result.has_value());
}
TEST(ShoulderAngle, Diagonal)
{
  auto result = arm_kinematics::get_shoulder_angle(1.0, 1.0, 1.0, 1.0);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result.value(), 0.0, 1e-9);
}
TEST(ShoulderAngle, Horizontal)
{
  auto result = arm_kinematics::get_shoulder_angle(std::sqrt(2.0), 0.0, 1.0, 1.0);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result.value(), -M_PI / 4, 1e-9);
}
TEST(ShoulderAngle, Unreachable)
{
  auto result = arm_kinematics::get_shoulder_angle(3.0, 0.0, 1.0, 1.0);
  EXPECT_FALSE(result.has_value());
}
TEST(WristPoint, DownApproach)
{
  auto w = arm_kinematics::get_wrist_point(1.0, 0.0, 0.5, -M_PI / 2);
  EXPECT_NEAR(w.r, 1.0, 1e-9);
  EXPECT_NEAR(w.z, 0.5, 1e-9);
}
TEST(WristPoint, HorizontalApproach)
{
  auto w = arm_kinematics::get_wrist_point(1.0, 1.0, 0.5, 0.0);
  EXPECT_NEAR(w.r, 0.5, 1e-9);
  EXPECT_NEAR(w.z, 1.0, 1e-9);
}
TEST(WristAngle, allAligned)
{
  EXPECT_NEAR(arm_kinematics::get_wrist_angle(0.0, 0.0, 0.0), 0.0, 1e-9);
}
TEST(WristAngle, PointDown)
{
  EXPECT_NEAR(arm_kinematics::get_wrist_angle(-M_PI / 2, M_PI / 2, 0.0), -M_PI, 1e-9);
}
TEST(ForwardKinematics, StraightOut)
{
  // 모든 각 0, 링크, 1:1:1 팔이 수평으로 펴지고 손 끝이 (3.0)에 위치함
  auto tip = arm_kinematics::get_forward_kinematics(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
  EXPECT_NEAR(tip.r, 3.0, 1e-9);
  EXPECT_NEAR(tip.z, 0.0, 1e-9);
}
TEST(ForwardKinematics, StraightUp)
{
  // 팔 전체를 수직으로 펴는 행동
  auto tip = arm_kinematics::get_forward_kinematics(M_PI / 2, 0.0, 0.0, 1.0, 1.0, 1.0);
  EXPECT_NEAR(tip.r, 0.0, 1e-9);
  EXPECT_NEAR(tip.z, 3.0, 1e-9);
}
// 어깨각, 팔꿈치각 손목값을 받아 순서대로 진행하는 테스트 전용 함수
// 입력 각도를 넣어서 FK 손끝(theta2, theta3, theta4)이 예상 위치에 있는지 확인
// 손끝위치 (r,z)를 통해 IK 손끝을 가려면 각도가 얼마인가?
// FK == IK 이면 테스트 통과
static void expect_round_trip(double theta2, double theta3, double theta4)
{
  const double l1 = arm_kinematics::L1, l2 = arm_kinematics::L2, l3 = arm_kinematics::L3;
  bool elbow_up = (theta3 >= 0.0);   // theta3 부호로 어느 가지인지 결정

  auto tip = arm_kinematics::get_forward_kinematics(theta2, theta3, theta4, l1, l2, l3);
  double phi = theta2 + theta3 + theta4;

  auto wrist = arm_kinematics::get_wrist_point(tip.r, tip.z, l3, phi);
  double d = arm_kinematics::get_reach_distance(wrist.r, wrist.z);

  auto t3 = arm_kinematics::get_elbow_angle(d, l1, l2, elbow_up);
  auto t2 = arm_kinematics::get_shoulder_angle(wrist.r, wrist.z, l1, l2, elbow_up);
  ASSERT_TRUE(t3.has_value());
  ASSERT_TRUE(t2.has_value());
  double t4 = arm_kinematics::get_wrist_angle(phi, t2.value(), t3.value());

  EXPECT_NEAR(t2.value(), theta2, 1e-9);
  EXPECT_NEAR(t3.value(), theta3, 1e-9);
  EXPECT_NEAR(t4, theta4, 1e-9);
}
TEST(RoundTrip, VariousElbowUp)
{
  expect_round_trip(0.0, M_PI / 3, 0.0);
  expect_round_trip(M_PI / 6, M_PI / 2, 0.0);
  expect_round_trip(-M_PI / 6, M_PI / 3, M_PI / 6);
  expect_round_trip(M_PI / 4, 2 * M_PI / 3, -M_PI / 4);
}
TEST(RoundTrip, VariousElbowDown)
{
  expect_round_trip(M_PI / 2, -M_PI / 3, 0.0);
  expect_round_trip(M_PI / 3, -M_PI / 2, 0.0);
  expect_round_trip(M_PI / 4, -M_PI / 3, M_PI / 6);
}
TEST(SolveIk, ReachableRoundTrips)
{
  // 도달 가능한 목표(평면, y=0) -> solve -> FK로 재구성하면 목표로 돌아오나
  // 15cm 5cm
  double tx = 0.15, tz = 0.05, phi = -M_PI / 2;   // 아래 쪽으로 접근
  double ux = tx + arm_kinematics::BASE_TO_SHOULDER_X;
  double uz = tz + arm_kinematics::BASE_TO_SHOULDER_Z;
  auto sol = arm_kinematics::solve_ik(ux, 0.0, uz, phi);
  ASSERT_TRUE(sol.reachable);
  auto tip = arm_kinematics::get_forward_kinematics(
    sol.theta2, sol.theta3, sol.theta4,
    arm_kinematics::L1, arm_kinematics::L2, arm_kinematics::L3);
  EXPECT_NEAR(tip.r, tx, 1e-9);
  EXPECT_NEAR(tip.z, tz, 1e-9);
}
TEST(SolveIk, Unreachable)
{
  // 가로로 30cm 초과 상황에 도달, base 기준 좌표 전달
  auto sol = arm_kinematics::solve_ik(0.3, 0.0, 0.1, -M_PI / 2);   // 팔 길이 밖
  EXPECT_FALSE(sol.reachable);
}
TEST(SolveIk, BackfoldGuard)
{
  // px < 0이 되는 목표는 도달 불가가 맞다.
  auto sol = arm_kinematics::solve_ik(-0.05, 0.0, 0.0, -M_PI / 2);
  EXPECT_FALSE(sol.reachable);
}
TEST(SolveIK, ElbowBranchesDiffer)
{
  // 같은 목표라 할지라도 해는 두 가지가 나올 수 있다.
  // 둘 다 도달 가능하고 theta3부호가 반대여야 한다.
  const double x = 0.169, z = 0.0475, phi = -M_PI / 2;
  auto up = arm_kinematics::solve_ik(x, 0.0, z, phi, true);
  auto down = arm_kinematics::solve_ik(x, 0.0, z, phi, false);
  ASSERT_TRUE(up.reachable);
  ASSERT_TRUE(down.reachable);
  EXPECT_GT(up.theta3, 0.0);   // 위 해
  EXPECT_LT(down.theta3, 0.0);   // 아래 해
}
TEST(SolveIk, GraspYawOmittedKeepsTheta5Zero)
{
  // grasp_yaw를 안 넘기면 theta5가 예전처럼 0으로 남는지
  auto sol = arm_kinematics::solve_ik(0.20, 0.06, 0.02, -M_PI / 2);
  ASSERT_TRUE(sol.reachable);
  EXPECT_NEAR(sol.theta5, 0.0, 1e-9);
}
TEST(SolveIk, GraspYawSubtractsBaseRotation)
{
  // grasp_yaw를 넘기면 theta5가 (grasp_yaw - theta1)이 되는지
  // theta5를 world 각도로 착각해 theta1을 안 뺀 것이 파지 실패의 원인
  const double grasp_yaw = 0.5;
  auto sol = arm_kinematics::solve_ik(0.20, 0.06, 0.02, -M_PI / 2, true, grasp_yaw);
  ASSERT_TRUE(sol.reachable);
  EXPECT_GT(std::abs(sol.theta1), 1e-3);  // theta1이 0이면 이 테스트는 무의미
  EXPECT_NEAR(sol.theta5, grasp_yaw - sol.theta1, 1e-9);
}
TEST(ToMotorAngles, StraightToMotor)
{
  arm_kinematics::IkSolution geometry{};
  auto motor = arm_kinematics::to_motor_angles(geometry);
  EXPECT_NEAR(motor.theta2, arm_kinematics::UPPER_ARM_TILT, 1e-9);
  EXPECT_NEAR(motor.theta3, -arm_kinematics::UPPER_ARM_TILT, 1e-9);
  EXPECT_NEAR(motor.theta4, 0.0, 1e-9);
}
TEST(ToMotorAngles, GeometryZeroPoseToMotorZero)
{
  arm_kinematics::IkSolution geometry{};
  geometry.theta2 = arm_kinematics::UPPER_ARM_TILT;
  geometry.theta3 = -arm_kinematics::UPPER_ARM_TILT;
  auto motor = arm_kinematics::to_motor_angles(geometry);
  EXPECT_NEAR(motor.theta2, 0.0, 1e-9);
  EXPECT_NEAR(motor.theta3, 0.0, 1e-9);
  EXPECT_NEAR(motor.theta4, 0.0, 1e-9);
}
