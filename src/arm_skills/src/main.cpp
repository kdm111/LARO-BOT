#include <atomic>
#include <csignal>

#include "arm_skills/skill_server.hpp"

// ★ 2026-08-22 안전 종료(사용자 결정 ① : 종료 시 home 으로 이동 후 유지).
//   rclcpp 기본 SIGINT 핸들러는 컨텍스트를 즉시 내려 버려서 종료 시점에 MoveIt
//   호출이 불가능하다. 신호를 우리가 받아 플래그만 세우고, 컨텍스트가 살아 있는
//   동안 safe_park()로 home 을 밟은 뒤에 내려간다.
//   launch 의 SIGINT -> SIGTERM 유예는 기본 수 초다 - home 이동(~3초)이 그 안에
//   끝나도록 빈 손 속도로 간다. SIGTERM 도 같은 플래그로 받는다.
namespace
{
std::atomic<bool> g_shutdown{false};
void on_signal(int)
{
  g_shutdown = true;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  auto node = std::make_shared<SkillServer>();

  // MGI가 로봇 모델을 토픽으로 받으려면 노드가 spin 중이여야 한다. -> 백그라운드 executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  node->init_move_group();   // spin 시작 후 모델 수신 + shared_from_this 유효

  while (!g_shutdown && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  node->safe_park();   // home 이동 후 유지. 토크 차단은 전원 스위치(사용자)의 몫.

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
