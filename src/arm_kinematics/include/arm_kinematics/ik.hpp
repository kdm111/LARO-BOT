#pragma once // 이 헤더는 한번만 포함되고 여러 파일이 같은 헤더를 include 할 때 중복 삽입을 방지한다.
#include <optional>

// 함수를 arm_kinematics::reach_distance라는 이름표 상자에 넣는다. 즉 다른 라이브러리에 비슷한 이름의 함수가 있어도 부딪히지 않는다.
namespace arm_kinematics
{
// 실제 omx-f의 링크 길이(m)
constexpr double L1 = 0.12052;   // 어깨 -> 팔꿈치 sqrt(0.0415^2 + 0.11315^2)
constexpr double L2 = 0.162;   // 팔꿈치 -> 손목
constexpr double L3 = 0.12063;   // 손목 -> TCP 0.0287 + 0.09193
// 위팔 에는 하드웨어 적으로 고정각이 존재한다.
// 처음 모델에서는 theta=0이 될 경우 팔이 직선으로 펴져야 하지만 실제로는 그러지 않았다.
// 그러므로 모델의 0과 모터의 0이 고정각만큼 차이가 나 있고 이 어긋남을 매우는 각을 정의해야 한다.
// urdf 문서를 참조하면 joint3의 origin은 (0.0415, 0.11315)로 정의되어 있다.
// joint2와 joint3는 서로 이어져 있다.
// joint3는 joint2를 기준으로 앞으로 0.0415m, 위로 0.11315m 뻗어있다.
constexpr double UPPER_ARM_TILT = 1.2192;
// base_link(link0) 원점 -> 어깨(joint2) 위이. URDF joint1+joint2 origin 누적
// solve_ik는 이제 base_link 기준 좌표를 받고 도입에서 이값을 빼 어깨 기준으로 바꾼다.
constexpr double BASE_TO_SHOULDER_X = -0.01125;
constexpr double BASE_TO_SHOULDER_Z = 0.0975;
struct Point2D
{
  double r;
  double z;
};
// IK의 해를 나타내는 구조체. 각 관절각과 해당 위치에 도달 가능한 지 확인한다.
struct IkSolution
{
  double theta1;
  double theta2;
  double theta3;
  double theta4;
  double theta5;
  bool reachable;
};
// grasp_yaw = 그리퍼 닫힘 축을 맞출 world 기준 방향(rad), 넘기지 않으면 theta5는 0으로 초기값을으로 유지
IkSolution solve_ik(
  double x, double y, double z, double phi, bool elbow_up = true,
  std::optional<double> grasp_yaw = std::nullopt);
// gazebo로 해당 위치로 팔 이동
IkSolution to_motor_angles(const IkSolution & geometry);
// 손목점이 실제도 도달 해야 하는 점. 접근 방향에서 그리퍼 크기만큼 물러난 곳
Point2D get_wrist_point(double r, double z, double l3, double phi);
// 순기구학 : 관절각 + 링크 길이 > 손끝 위치(평면 r,z) IK 왕복 검증용
Point2D get_forward_kinematics(
  double theta2, double theta3, double theta4, double l1, double l2,
  double l3);
double get_reach_distance(double r, double z);   // 함수 선언
double get_base_angle(double px, double py);   // 물체를 팔에 중심에 위치하도록 베이스 이동
// 팔꿈치 각도 계산 bool 대수 포함으로 해당 팔꿈치가 위로 향하게 할지 아래로 향하게 할지 결정한다.
std::optional<double> get_elbow_angle(double d, double l1, double l2, bool elbow_up = true);
// 어깨 각도 계산
std::optional<double> get_shoulder_angle(
  double r, double z, double l1, double l2,
  bool elbow_up = true);
double get_wrist_angle(double phi, double theta2, double theta3);
}
