#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <arm_kinematics/ik.hpp>

int main(int argc, char **argv)
{
  // 목표점 (base_link = world 기준. ik.hpp 19행 주석 참조)
  //
  // ★ 2026-08-19: 인자를 안 주면 기본값으로 조용히 도는 것을 막는다.
  //   그날 호출 스크립트가 좌표 추출에 실패해 빈 문자열을 넘겼는데, 옛 기본값
  //   (0.20, 0, -0.05)이 그대로 쓰여 팔이 테이블 아래를 향했고 서보가 과부하
  //   알람(빨간 LED)을 띄웠다. 조용한 기본값은 실물에서 위험하다.
  if (argc < 4) {
    std::printf("사용법: reach_once <x> <y> <z> [elbow_up 0|1]\n");
    std::printf("  좌표는 base_link(=world) 기준 미터. z는 테이블면이 0.\n");
    return 2;
  }
  const double x = std::atof(argv[1]);
  const double y = std::atof(argv[2]);
  const double z = std::atof(argv[3]);
  const double phi = -M_PI / 2; // 접근각 : 그리퍼 아래로

  // ★ 작업공간 하한. 테이블면이 z=0 이므로 그 아래로 내려가는 목표는 IK 를 풀기
  //   전에 막는다. Gate 2 S1(관절 한계)의 짝인 작업공간 검사이고, 08-16 감사에도
  //   없던 항목이다 - sim 에서는 팔이 바닥을 뚫고 지나가도 아무 일이 없었다.
  constexpr double kMinZ = 0.005;
  if (z < kMinZ) {
    std::printf("거부: z=%.3f 은 테이블면(0) 아래다. 최소 %.3f\n", z, kMinZ);
    return 3;
  }

  // ★ 2026-08-19 추가: 팔꿈치 가지 선택. 기본은 기존과 같은 up.
  //   실물 서보 한계에서는 up 해가 거의 전부 범위 밖이라(joint3 한계 [-1.571,1.675])
  //   실기 좌표를 확인할 때는 argv[4]=0 으로 down 을 뽑아야 한다.
  bool elbow_up = true;
  if (argc >= 5) {
    elbow_up = (std::atoi(argv[4]) != 0);
  }
  const arm_kinematics::IkSolution sol = arm_kinematics::solve_ik(x, y, z, phi, elbow_up);
  if (!sol.reachable) {
    std::printf("도달 불가: (%.3f, %.3f, %.3f)\n", x, y, z);
    return 1;
  }
  const arm_kinematics::IkSolution m = arm_kinematics::to_motor_angles(sol);
  std::printf("목표 (%.3f, %.3f, %.3f), phi=%.2f, elbow=%s\n", x, y, z, phi, elbow_up ? "up" : "down");
  std::printf("모터각 j1..j5 = [%.4f, %.4f, %.4f, %.4f, %.4f]\n\n",
    m.theta1, m.theta2, m.theta3, m.theta4, m.theta5);
  std::printf("아래 Gazebo 띄운 셀에 붙여 넣기\n");
  std::printf(
    "ros2 topic pub --once /arm_controller/joint_trajectory "
    "trajectory_msgs/msg/JointTrajectory "
    "\"{joint_names: [joint1, joint2, joint3, joint4, joint5], "
    "points: [{positions: [%.4f, %.4f, %.4f, %.4f, %.4f], "
    "time_from_start: {sec: 2}}]}\"\n",
    m.theta1, m.theta2, m.theta3, m.theta4, m.theta5);
  return 0;
}
