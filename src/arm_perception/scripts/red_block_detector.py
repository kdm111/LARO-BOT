#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# 빨강은 색상환의 시작점이자 끝점이라 구간이 둘로 나뉜다.
# OpenCV H 범위는 0~179. S/V 0~255
RED_LOWER_1 = np.array([0, 120, 70])
RED_UPPER_1 = np.array([10, 255, 255])
RED_LOWER_2 = np.array([170, 120, 70])
RED_UPPER_2 = np.array([180, 255, 255])

MIN_AREA = 100  # 이보다 작은 덩어리는 잡음으로 버린다.


class RedBlockDetector(Node):

    def __init__(self):
        super().__init__('red_block_detector')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.on_image, 10)
        # 튜닝용 디버그 영상 rqt_image_view로 보면서 임계값을 조정한다.
        self.debug_pub = self.create_publisher(Image, '/perception/debug_image', 10)
        self.get_logger().info('red_block_detector 시작. /camera/image_raw 구독')

    def on_image(self, msg):
        # ROS Image -> OpenCV 배열, bgr8을 명시해야 한다.
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # BGR -> HSV 조명이 변해도 H는 변하지 않는다.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 빨강 두 구간을 각각 마스크로 만들고 합친다. 결과는 0/255짜리 흑백 이미지
        mask = (cv2.inRange(hsv, RED_LOWER_1, RED_UPPER_1)
                | cv2.inRange(hsv, RED_LOWER_2, RED_UPPER_2))

        # 열기 (침식 -> 팽창) 잡음은 소멸시키고 살아남은 덩어리는 원래 크기로 복구
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # 흰 덩어리들의 외곽선
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        centers = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < MIN_AREA:
                continue
            # 모멘트 좌표들의 합계. m00이 개수(면적), m10/m01이 u/v 좌표의 합
            m = cv2.moments(c)
            if m['m00'] == 0:
                continue
            u = int(m['m10'] / m['m00'])
            v = int(m['m01'] / m['m00'])
            centers.append((u, v, area))
            cv2.circle(frame, (u, v), 5, (0, 255, 0), -1)  # 디버그 영상에 초록색 점 표시

        # 면적 큰 순 = 크고 확실한 것 순
        centers.sort(key=lambda c: c[2], reverse=True)

        if centers:
            found = ', '.join(f'({u}, {v}) area={a:.0f}' for u, v, a in centers)
            self.get_logger().info(
                f'검출 {len(centers)}개 {found}', throttle_duration_sec=1.0)
        else:
            self.get_logger().warn('빨강 없음', throttle_duration_sec=2.0)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))


def main():
    rclpy.init()
    node = RedBlockDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
