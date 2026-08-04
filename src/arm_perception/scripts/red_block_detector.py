#!/usr/bin/env python3

from arm_interfaces.msg import DetectedObject, SceneState
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
import tf2_ros

# 빨강은 색상환의 시작점이자 끝점이라 구간이 둘로 나뉜다.
# OpenCV H 범위는 0~179. S/V 0~255
RED_LOWER_1 = np.array([0, 120, 70])
RED_UPPER_1 = np.array([10, 255, 255])
RED_LOWER_2 = np.array([170, 120, 70])
RED_UPPER_2 = np.array([180, 255, 255])

MIN_AREA = 100  # 이보다 작은 덩어리는 잡음으로 버린다.
PLANE_Z = 0.02 # 블록 중심의 높이. 블록 높이 0.04의 절반. 이 평면과 광선을 만나게 해서 거리를 정한다.
WORLD_FRAME = 'world' # MoveIt 플래닝 프레임. skill_server가 알아듣는 좌표계
OPTICAL_FRAME = 'camera_optical_frame' 

class RedBlockDetector(Node):

    def __init__(self):
        super().__init__('red_block_detector')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.on_image, 10)

        # 내부 파라미터는 코드에 박지 않고 토픽에서 받는다. 실물 캘리브레이션과 같은 경로
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.on_camera_info, 10)
        # TF 버퍼가 world <- camera_optical_frame 변환을 계속 모아둔다. listner가 채워준다.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listner = tf2_ros.TransformListener(self.tf_buffer, self)
        # scene_state 발행
        self.scene_pub = self.create_publisher(SceneState, '/scene_state', 10)
        # 튜닝용 디버그 영상 rqt_image_view로 보면서 임계값을 조정한다.
        self.debug_pub = self.create_publisher(Image, '/perception/debug_image', 10)
        self.get_logger().info('red_block_detector 시작. /camera/image_raw 구독')

    def on_camera_info(self, msg):
        # K는 행 우선 9개 [fx 0 cs / 0 fy cy / 0 0 1]. 필요한 4개만 꺼낸다.
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def pixel_to_world(self, u, v, stamp):
        """픽셀 하나를 테이블 평면 위의 world 좌표로 되돌린다."""
        if self.fx is None:
            return None # camera_info가 오지 안으면 변환 금지
        try:
            tf = self.tf_buffer.lookup_transform(WORLD_FRAME, OPTICAL_FRAME, stamp)
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'TF 없음 : {e}', throttle_duration_sec=2.0)
            return None
        # 광선을 점 두 개로 표현. near는 렌즈의 중심이고 far은 깊의 1에서의 점
        # 두 점이 정해지면 그 둘을 잇는 직선을 광선으로 본다.
        near = PointStamped()
        near.header.frame_id = OPTICAL_FRAME
        far = PointStamped()
        far.header.frame_id = OPTICAL_FRAME
        far.point.x = (u - self.cx) / self.fx
        far.point.y = (v - self.cy) / self.fy
        far.point.z = 1.0

        # 회전행렬도 쿼터니언도 직접 다루지 않음. TF가 두 점을 world로 움직이고 옮겨진 두 점으로 광선을 다시 만듬
        near_w = do_transform_point(near, tf).point
        far_w = do_transform_point(far, tf).point

        dz = far_w.z - near_w.z
        if dz >= 0:
            return None # 광선이 아래로 향하지 않음. 평면과 만나지 않는다.

        # 평면까지 내려가야 할 높이 / 광선이 1칸 내려갈ㄸ ㅐ내려가는 높이. 늘려야할 매수
        t = (PLANE_Z - near_w.z) / dz
        x = near_w.x + t * (far_w.x - near_w.x)
        y = near_w.y + t * (far_w.y - near_w.y)
        return (x, y)

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

        # scene 생성
        scene = SceneState()
        scene.header.stamp = msg.header.stamp
        scene.header.frame_id = WORLD_FRAME

        parts = []
        for i, (u, v, area) in enumerate(centers):
            p = self.pixel_to_world(u, v, msg.header.stamp)
            if p is None:
                parts.append(f'({u}, {v}) -> 변환 불가')
                continue
            obj = DetectedObject()
            # 색만으로는 구별이 되지 않으므로 여러 개면 면적 내림차순 번호를 부여한다.
            obj.object_id = 'red_block' if len(centers) == 1 else f'red_block_{i+1}'
            obj.pose.header.stamp = msg.header.stamp
            obj.pose.header.frame_id = WORLD_FRAME
            obj.pose.pose.position.x = p[0]
            obj.pose.pose.position.y = p[1]
            obj.pose.pose.position.z = PLANE_Z
            # 무게 중심 하나만으로는 자세가 나오지 않는다. 
            obj.pose.pose.orientation.w = 1.0
            obj.last_seen = msg.header.stamp
            scene.objects.append(obj)
            parts.append(f'{obj.object_id} ({p[0]:.3f}, {p[1]:.3f})')
            
        self.scene_pub.publish(scene)
        
        if parts:
            self.get_logger().info(
                f'/scene_state {len(scene.objects)}개 : ' + ', '.join(parts), 
                throttle_duration_sec=1.0)
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
