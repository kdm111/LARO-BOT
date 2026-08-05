#include "arm_kinematics/ik.hpp" // cpp가 hpp의 선언을 읽어 컴파일러가 대조하게 만든다ㅏ. 리턴 타입이나 인자를 틀리면 에러가 나온다.
#include <cmath> // 시스템 표준헤더 불러오기

namespace arm_kinematics
{ // hpp에서 상자에 담아 선언했으니 cpp에서도 같은 상자 안에서 정의해야 짝이 맞는다.
double get_reach_distance(double r, double z)
{
  return std::sqrt(r * r + z * z);   // std::pow(r,2)도 되지만 정수 제곱에는 곱셈이 빠르다. 하지만 카메라 좌표가 들어오면 어떻게 될지 스스로 분석할 것
}
double get_base_angle(double px, double py)
{
  return std::atan2(py, px);   // 각도 변환 생성
}
std::optional<double> get_elbow_angle(double d, double l1, double l2, bool elbow_up)
{
  double cos_theta = (d * d - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
  if (std::fabs(cos_theta) > 1.0) {
    return std::nullopt;
  }
  double theta = std::acos(cos_theta);
  return elbow_up ? theta : -theta;
}
std::optional<double> get_shoulder_angle(
  double r, double z, double l1, double l2, bool elbow_up)
{
  double d = get_reach_distance(r, z);   // 어깨에서 목표까지의 직선 거리 D
  // 보정각 beta의 코사인, 삼각형(l1, d, l2)에서 어깨 꼭지점의 각도이다.
  double cos_beta = (l1 * l1 + d * d - l2 * l2) / (2.0 * l1 * d);
  // 도달 가능성 검사. |cos|이 1을 넘으면 그런 삼각형은 존재하지 않는다.
  if (std::fabs(cos_beta) > 1.0) {
    return std::nullopt;
  }
  // 어깨에서 목표를 똑바로 겨누는 각. 수평 x축 기준 목표 방향
  double aim = std::atan2(z, r);
  // cos_beta로부터 실제 보정각. 팔꿈치가 접혀서 위팔이 더 들려야 하는 양
  double beta = std::acos(cos_beta);
  return elbow_up ? aim - beta : aim + beta;
}
Point2D get_wrist_point(double r, double z, double l3, double phi)
{
  // 그리퍼는 손목에서 목표 방향으로 L3 만큼 뻗는다. target = wrist + L3 * (cos(phi), sin(phi))
  // 뒤집으면 손목 = 목표 - L3 * (cos(phi), sin(phi))
  double wrist_r = r - l3 * std::cos(phi);
  double wrist_z = z - l3 * std::sin(phi);
  return {wrist_r, wrist_z}; // Point2D 반환
}
double get_wrist_angle(double phi, double theta2, double theta3)
{
  // theta2 + theta3 + theta4 = phi를 theta4에 대해 푼 것
  return phi - theta2 - theta3;
}
Point2D get_forward_kinematics(
  double theta2, double theta3, double theta4, double l1, double l2,
  double l3)
{
  // 각 링크의 절대 방향 = 관절각 누적합
  double a2 = theta2;   // 위팔 방향
  double a3 = theta2 + theta3;   // 아래팔 방향
  double a4 = theta2 + theta3 + theta4;   // 그리퍼 방향

  // 각 링크 벡터(길이,방향)를 끝에서 끝으로 더해 손끝 좌표를 만든다.
  double tip_r = l1 * std::cos(a2) + l2 * std::cos(a3) + l3 * std::cos(a4);
  double tip_z = l1 * std::sin(a2) + l2 * std::sin(a3) + l3 * std::sin(a4);
  return {tip_r, tip_z};
}
IkSolution solve_ik(
  double x, double y, double z, double phi, bool elbow_up,
  std::optional<double> grasp_yaw)
{
  IkSolution sol{};   // 전부 0/false로 초기화

  const double px = x - BASE_TO_SHOULDER_X;  // 어깨 기준 x (base x-offset이 여기서 반영)
  const double py = y;  // y는 항상 축이 같음
  const double pz = z - BASE_TO_SHOULDER_Z;   // 어깨 기준 높이 (어깨보다 아래면 음수)

  sol.theta1 = get_base_angle(px, py);   // theta1 base yaw 값
  double r = get_reach_distance(px, py);   // 평면 안 수평거리

  // 뒤접힘 가드 : 어깨 기준 앞쪽으로만 유효
  // px < 0 은 팔이 뒤로 접히는 자세로 theta1을 180도 뒤집어 거짓 성공을 내던 스팟
  // 도달 불가로 표현
  if (px < 0.0) {
    sol.reachable = false;
    return sol;
  }
  Point2D wrist = get_wrist_point(r, pz, L3, phi);   // 2R이 닿을 손목점
  double d = get_reach_distance(wrist.r, wrist.z);   // 손목 점까지의 거리

  auto t3 = get_elbow_angle(d, L1, L2, elbow_up);   // elbow_up 해당 위치까지 닿을 팔꿈치 각도
  auto t2 = get_shoulder_angle(wrist.r, wrist.z, L1, L2, elbow_up);   // 해당 목표 위치까지 어깨 각도
  if (!t3.has_value() || !t2.has_value()) {
    sol.reachable = false;   // 범위 바깥 혹은 너무 가까운 위치. 도달 불가능 판정
    return sol;
  }
  sol.theta2 = t2.value();
  sol.theta3 = t3.value();
  sol.theta4 = get_wrist_angle(phi, sol.theta2, sol.theta3);
  // 이제 그리퍼 닫힘 축은 wolrd의 grasp_yaw에 맞춘다.
  // theta5는 world 기준이 아니라 theta1만큼 돌아간 팔 기준이라 그만큼 빼준다.
  // 이 뺄셈이 빠진 것이 파지 실패 원인이다.
  sol.theta5 = grasp_yaw.has_value() ? (grasp_yaw.value() - sol.theta1) : 0.0;
  sol.reachable = true;
  return sol;
}
IkSolution to_motor_angles(const IkSolution & geometry)
{
  IkSolution motor = geometry;   // reachable, theta1, theta5는 변하지 않는다.
  motor.theta2 = UPPER_ARM_TILT - geometry.theta2;   // 위팔 기울기 보정
  motor.theta3 = -UPPER_ARM_TILT - geometry.theta3;   //
  motor.theta4 = -geometry.theta4;   // 손목 부호는 반전
  return motor;
}
} // namespace arm_kinematics
