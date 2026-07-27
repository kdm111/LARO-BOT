#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <arm_kinematics/ik.hpp>

int main(int argc, char **argv)
{
  // 목표점 (solve_ik 프레임 : 어깨 원점, z는 어깨 기준)
  double x = 0.20, y = 0.0, z = -0.05;
  const double phi = -M_PI / 2; // 접근각 : 그리퍼 아래로
  if (argc >= 4) {
    x = std::atof(argv[1]);
    y = std::atof(argv[2]);
    z = std::atof(argv[3]);
  }

  const arm_kinematics::IkSolution sol = arm_kinematics::solve_ik(x, y, z, phi);
  if (!sol.reachable) {
    std::printf("도달 불가: (%.3f, %.3f, %.3f)\n", x, y, z);
    return 1;
  }
  const arm_kinematics::IkSolution m = arm_kinematics::to_motor_angles(sol);
  std::printf("목표 (%.3f, %.3f, %.3f), phi=%.2f\n", x, y, z, phi);
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
