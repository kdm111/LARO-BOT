#!/usr/bin/env python3
"""move_to 취소 -> 실물 정지 데모/검증 스크립트.

2026-08-22 새벽 실물 검증에 쓴 절차 그대로다(M6_SAFETY_2026-08-22.md §1):
  init 이동을 보내고 1.2초 시점에 cancel_goal_async -> 기대 결과는
  ① 팔이 목표까지 가지 않고 중간 감속 정지
  ② 액션 상태 5(CANCELED)
  ③ 정지 후 1초 관절 드리프트 = 0 (완전 정지)

VIDEO_SHOOTING_PLAN.md §11이 "검증된 스크립트만 사용"을 요구해서 파일로 굳혔다.
CLI Ctrl+C 도 cancel 을 보내긴 하지만, 이 스크립트는 취소 시점과 판정을
재현 가능하게 고정한다는 점이 다르다.

사용법 (컨테이너 안, ROS_DOMAIN_ID=48 환경에서):
  python3 tools/cancel_demo.py            # init 으로 출발, 1.2초에 취소
  python3 tools/cancel_demo.py home 0.8   # 대상 자세와 취소 시점(초) 지정

끝난 뒤 팔은 공중에 서 있다 - home 복귀는 별도로 보낸다:
  ros2 action send_goal /move_to arm_interfaces/action/MoveTo "{pose_id: home, attempt: 0}"
"""
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from arm_interfaces.action import MoveTo
from sensor_msgs.msg import JointState

STATUS_NAMES = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


def main():
    pose_id = sys.argv[1] if len(sys.argv) > 1 else 'init'
    cancel_at = float(sys.argv[2]) if len(sys.argv) > 2 else 1.2

    rclpy.init()
    node = Node('cancel_demo')
    joints = {}
    node.create_subscription(
        JointState, '/joint_states',
        lambda m: joints.__setitem__('pos', list(m.position)), 10)

    client = ActionClient(node, MoveTo, '/move_to')
    if not client.wait_for_server(timeout_sec=8):
        print('실패: /move_to 액션 서버 없음 (skills 가 떠 있는가?)')
        return 1

    goal = MoveTo.Goal()
    goal.pose_id = pose_id
    goal.attempt = 0
    t0 = time.time()
    send = client.send_goal_async(goal)
    while rclpy.ok() and not send.done():
        rclpy.spin_once(node, timeout_sec=0.05)
    handle = send.result()
    print(f'수락: {handle.accepted} (pose_id={pose_id})')
    if not handle.accepted:
        return 1

    while time.time() - t0 < cancel_at:
        rclpy.spin_once(node, timeout_sec=0.05)
    print(f'취소 전송 (t={time.time() - t0:.2f}s)')
    handle.cancel_goal_async()

    result = handle.get_result_async()
    while rclpy.ok() and not result.done():
        rclpy.spin_once(node, timeout_sec=0.05)
    status = result.result().status
    print(f'액션 상태: {status} ({STATUS_NAMES.get(status, "?")}) - 5 여야 성공')

    p1 = list(joints.get('pos', []))
    t1 = time.time()
    while time.time() - t1 < 1.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    p2 = list(joints.get('pos', []))
    drift = max(abs(a - b) for a, b in zip(p1, p2)) if p1 and p2 else -1.0
    print(f'정지 후 1초 관절 드리프트: {drift:.4f} rad (감속 꼬리 제외 후 0 근처여야 함)')
    print('판정:', 'PASS' if status == 5 else 'FAIL')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if status == 5 else 1


if __name__ == '__main__':
    raise SystemExit(main())
