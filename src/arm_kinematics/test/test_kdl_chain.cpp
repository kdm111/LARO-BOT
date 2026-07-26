#include <gtest/gtest.h>  // 테스트 프레임워크
#include <kdl/chain.hpp>  // KDL Chain, Segment, Joint
#include <kdl/frames.hpp>  // Vector, Frame, Rotation
#include <cmath>  // M_PI 불러오기
#include <kdl/chainfksolverpos_recursive.hpp>  // FK 솔버
#include <kdl/jntarray.hpp> // 관절값 배열
#include <Eigen/Dense>   // Eigen::Matrix (IK 가중치 벡터)
#include <kdl/chainiksolverpos_lma.hpp>   // KDL 수치 IK
#include <arm_kinematics/kdl_chain.hpp>   // kdl 체인 선언
using arm_kinematics::build_omx_chain;

// 체인의 구조가 팔과 정확한가
TEST(KdlChain, StructureMatchesArm)
{
  KDL::Chain chain = build_omx_chain();
  EXPECT_EQ(chain.getNrOfSegments(), 6u);  // 관절 5 + ee 고정 = 6 segment
  EXPECT_EQ(chain.getNrOfJoints(), 5u);   // 움직이는 관절 5개(Fixed는 안 셈)
}
// 정지 자세에서 손끝 위치가 맞는가?
// 관절 0이면 회전이 전부 단위행렬 오프셋들의 단순 벡터합.
TEST(KdlChain, ForwardKinematicsAtZero)
{
  KDL::Chain chain = build_omx_chain();
  KDL::ChainFkSolverPos_recursive fk(chain);   // 체인을 받아 FK를 푸는 솔버

  KDL::JntArray q(chain.getNrOfJoints());   // 관절 5개 전부 0(헤더가 보장함)
  KDL::Frame tip;   // 결과(손끝 위치+자세)를 받을 그릇

  int rc = fk.JntToCart(q, tip);   // q를 넣으면 tip을 채운다.
  ASSERT_GE(rc, 0);   // theta=E_NOERROR, 음수가 나오면 Error 처리

  // 관절이 0이면 회전이 전부 단위행렬 -> 손끝 = 오프셋들의 단순 합
  EXPECT_NEAR(tip.p.x(), 0.31288, 1e-4);
  EXPECT_NEAR(tip.p.y(), -0.0016, 1e-4);
  EXPECT_NEAR(tip.p.z(), 0.21065, 1e-4);
}
// z축회전으로 joint1의 축이 z인지 확인한다.
TEST(KdlChain, BaseYawKeepsHeight)
{
  KDL::Chain chain = build_omx_chain();
  KDL::ChainFkSolverPos_recursive fk(chain);

  KDL::JntArray q(chain.getNrOfJoints());
  KDL::Frame tip0;  // tip0 높이
  fk.JntToCart(q, tip0);   // 관절 9에서의 손끝 높이

  q(0) = M_PI / 2.0;   // joint1만 90도 회전 (base yaw)
  KDL::Frame tip1;
  fk.JntToCart(q, tip1);

  // z축 회전은 높이를 바꾸지 않는다. joint1 축이 z가 아니면 z가 틀어져 실패.
  EXPECT_NEAR(tip1.p.z(), tip0.p.z(), 1e-9);  // 높이는 그대로여야 한다.
}
// KDL 수치 IK가 도달 가능한 목표를 실제로 푸는가 (posision-only)
// 알려진 관절각 q_true를 FK로 목표 프레임으로 바꾼 뒤 -> 반드시 도달 가능
// KDL IK로 되풀어소 q_out을 얻고 FK(q_out)가 그 목표 "위치"에 오는지 본다.
// q_out == q_true 일 필요는 없다. 검사 대상은 "위치가 맞나"지 "각이 같나"는 아니다
TEST(KdlChain, NumericIkReachesTarget)
{
  KDL::Chain chain = build_omx_chain();
  KDL::ChainFkSolverPos_recursive fk(chain);

  // posision-only 가중치 위치 xyz = 1 방향 3축 = 0
  // 벤더 kinematics.yaml의 position_only_ik : true를 그대로 재현
  Eigen::Matrix<double, 6, 1> weights;
  weights << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
  KDL::ChainIkSolverPos_LMA ik(chain, weights);

  // 도달 가능한 목표 만들기 : 적당한 괄절각 -> FK -> 목표 프레임
  KDL::JntArray q_true(chain.getNrOfJoints());
  q_true(0) = 0.3;
  q_true(1) = -0.6;
  q_true(2) = 1.0;
  q_true(3) = -0.4;
  q_true(4) = 0.0;
  KDL::Frame target;
  fk.JntToCart(q_true, target);

  // 출발점(초기 추정) q_init: 여기서 어느 해로 수렴할지가 갈린다.(local minima의 출발점)
  KDL::JntArray q_init(chain.getNrOfJoints());   // 전부 0
  KDL::JntArray q_out(chain.getNrOfJoints());
  int rc = ik.CartToJnt(q_init, target, q_out);
  ASSERT_EQ(rc, 0);

  // 되푼 각을 FK로 돌려 목표 위치에 왔는지 확인
  KDL::Frame reached;
  fk.JntToCart(q_out, reached);
  EXPECT_NEAR(reached.p.x(), target.p.x(), 1e-4);
  EXPECT_NEAR(reached.p.y(), target.p.y(), 1e-4);
  EXPECT_NEAR(reached.p.z(), target.p.z(), 1e-4);
}
