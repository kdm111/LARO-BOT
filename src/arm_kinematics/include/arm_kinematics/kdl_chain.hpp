#pragma once

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>

namespace arm_kinematics
{
// OMX 팔을 KDL 체인으로 조립한다.
// KDL 규약 : joint는 segment 뿌리에 있다.

KDL::Chain build_omx_chain()
{
  using KDL::Vector;
  using KDL::Frame;
  using KDL::Joint;
  using KDL::Segment;

  // 축은 도는 선이지 뻗는 방향이 아니다. 관절의 축은 회전의 중심이 되는 선, 움직임에 수직인 피벗선이다.
  const Vector axis_z(0.0, 0.0, 1.0);   // yaw(joint1) 위(하늘)
  const Vector axis_y(0.0, 1.0, 0.0);   // pitch(joint2,3,4), 앞(팔이 뻗는 방향)
  const Vector axis_x(1.0, 0.0, 0.0);   // roll(joint5), 왼쪽

  KDL::Chain chain;

  // joint1은 z축을 중심으로 회전한다.
  // joint(위치, 회전축, )
  const Vector off1(-0.01125, 0.0, 0.034);
  chain.addSegment(Segment(Joint(off1, axis_z, Joint::RotAxis), Frame(off1)));

  // y축을 중심으로 회전한다.
  const Vector off2(0.0, 0.0, 0.0635);
  chain.addSegment(Segment(Joint(off2, axis_y, Joint::RotAxis), Frame(off2)));

  const Vector off3(0.0415, 0.0, 0.11315);
  chain.addSegment(Segment(Joint(off3, axis_y, Joint::RotAxis), Frame(off3)));

  const Vector off4(0.162, 0.0, 0.0);
  chain.addSegment(Segment(Joint(off4, axis_y, Joint::RotAxis), Frame(off4)));

  // x축을 중심으로 회전한다.
  const Vector off5(0.0287, 0.0, 0.0);
  chain.addSegment(Segment(Joint(off5, axis_x, Joint::RotAxis), Frame(off5)));

  // end_effector는 위치와 회전축이 존재하지 않는다. 고정되어 있다.
  const Vector off_end_effector(0.09193, -0.0016, 0.0);
  chain.addSegment(Segment(Joint(Joint::Fixed), Frame(off_end_effector)));

  return chain;
}
}
