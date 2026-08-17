#!/usr/bin/env python3

import math

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

MIN_AREA = 100  # 이보다 작은 덩어리는 잡음으로 버린다.
WORLD_FRAME = 'world'  # MoveIt 플래닝 프레임. skill_server가 알아듣는 좌표계
OPTICAL_FRAME = 'camera_optical_frame'

# 물체 맵 정리 = 현재 한 색 = 한 물체 계약, 같은 색 두 개를 두면 충돌이 일어남
# hsv      : (lower, upper) 구간 목록. OpenCV H 범위는 0~179, S/V는 0~255.
#            빨강은 색상환의 시작점이자 끝점이라 구간이 둘로 나뉜다.
# plane_z  : 물체 중심의 높이. 광선을 이 평면과 만나게 해서 거리를 정한다.
#            ★ 물체마다 다르다. 납작한 링에 블록 값을 쓰면 좌표가 1cm쯤 밀린다.
# min_long : 가림 게이트. ★ 실치수가 아니라 "이 카메라에서 관측되는" 하한이다.
#            minAreaRect는 윗면+옆면의 합집합을 감싸므로 관측 길이가 실치수보다 크고,
#            블록의 위치·자세에 따라 6.3~8.4cm로 흔들린다(2026-08-07 실측, 5개 지점).
#            상한은 두지 않는다 - 정상 블록을 오폭한다(8.4cm 실측). 조각난 blob만 거른다.
OBJECTS = {
    'red_block': {
        'hsv': [
            (np.array([0, 120, 70]), np.array([10, 255, 255])),
            (np.array([170, 120, 70]), np.array([180, 255, 255])),
        ],
        'plane_z': 0.02,    # 블록 높이 0.04의 절반
        'min_long': 0.055,  # 조각난 blob 상한 4.9cm과 정상 하한 6.3cm 사이
    },
    'green_block': {
        # 불량품. red_block과 모양·크기가 같아 plane_z/min_long을 공유한다.
        # 색 RGB(0.05, 0.75, 0.15) = H 64 부근. 빨강(0/175)·파랑(113)과 안 겹친다.
        'hsv': [
            (np.array([50, 120, 70]), np.array([80, 255, 255])),
        ],
        'plane_z': 0.02,
        'min_long': 0.055,
    },
    'blue_ring': {
        # 링 색은 RGB(0.05, 0.25, 0.9) = H 113 부근. 구역 판은 채도 0이라 안 걸린다.
        'hsv': [
            (np.array([100, 120, 70]), np.array([130, 255, 255])),
        ],
        'plane_z': 0.006,   # 링 두께 0.012의 절반 (blue_ring.sdf 실치수)
        # 2026-08-14 실측 4개 지점: 정상 5.8~6.2cm(실치수 6.0 - 납작해서 부풀림이 작다),
        # 블록에 가려 조각났을 때 4.5cm. 그 사이에 둔다.
        'min_long': 0.052,
    },
}


class ObjectDetector(Node):

    def __init__(self):
        super().__init__('object_detector')
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
        self.get_logger().info('object_detector 시작. /camera/image_raw 구독')

    # 카메라 정보 파악
    def on_camera_info(self, msg):
        # K는 행 우선 9개 [fx 0 cs / 0 fy cy / 0 0 1]. 필요한 4개만 꺼낸다.
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        # 캘리브레이션이 안된 카메라는 K를 전부 0으로 되돌린다.
        # 0
        if not (self.fx > 0 and self.fy > 0):
            self.get_logger().error(
                f'camera_info가 캘리브레이션되지 않음 캘리브 yaml을 물려야 함{self.fx} {self.fy}', throttle_duration_sec = 2.0)
            self.fx = None
            self.fy = None
            self.cx = None
            self.cy = None
            

    def pixel_to_world(self, u, v, stamp, plane_z):
        """픽셀 하나를 plane_z 평면 위의 world 좌표로 되돌린다."""
        # 현재 카메라 값이 아니면 막지 못한다.
        if self.fx is None:
            return None  # camera_info가 오지 안으면 변환 금지
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
            return None  # 광선이 아래로 향하지 않음. 평면과 만나지 않는다.

        # 평면까지 내려가야 할 높이 / 광선이 1칸 내려갈ㄸ ㅐ내려가는 높이. 늘려야할 매수
        t = (plane_z - near_w.z) / dz
        x = near_w.x + t * (far_w.x - near_w.x)
        y = near_w.y + t * (far_w.y - near_w.y)
        return (x, y)

    def long_axis(self, box, stamp, plane_z):
        """회전 사각형의 긴 축을 plane_z 평면 위의 yaw(rad), 길이(m)로 되돌린다."""
        # 코너 4개는 이웃끼리 붙어 있다. 이웃한 두 변 중 긴 쪽이 긴 축이다.
        if np.linalg.norm(box[1] - box[0]) >= np.linalg.norm(box[2] - box[1]):
            # box[0]보다 box[1]이 길다. 짧은 변 두 개의 중점을 이으면 긴 축이 된다.
            a = (box[1] + box[2]) / 2.0
            b = (box[3] + box[0]) / 2.0
        else:
            a = (box[0] + box[1]) / 2.0
            b = (box[2] + box[3]) / 2.0
        # 픽셀 각도를 변환하지 않는다. 점 두 개를 world로 되돌린 뒤 거기서 각도를 만든다.
        wa = self.pixel_to_world(a[0], a[1], stamp, plane_z)
        wb = self.pixel_to_world(b[0], b[1], stamp, plane_z)
        if wa is None or wb is None:
            return None
        yaw = math.atan2(wa[1] - wb[1], wa[0] - wb[0])
        length = math.hypot(wa[0] - wb[0], wa[1] - wb[1])
        # 사각형의 긴축은 180도 대칭이다. 프레임마다 부호가 뒤집히지 않게 한 구간으로 접는다.
        return ((yaw + math.pi / 2.0) % math.pi - math.pi / 2.0, length)

    def find_blob(self, hsv, frame, spec, object_id):
        """한 물체의 색 마스크에서 가장 큰 덩어리 하나를 찾는다. 없으면 None.

        frame에 디버그 표시를 그린다(초록 점 = 중심, 파란 사각 = minAreaRect).
        """
        # 이 물체의 색 구간들을 각각 마스크로 만들어 합친다. 0/255짜리 흑백 이미지
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lower, upper in spec['hsv']:
            mask |= cv2.inRange(hsv, lower, upper)
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
            # 최소 외접 회전 사각형. 물체가 어느 쪽을 향하는 지는 여기서만 나온다.
            box = cv2.boxPoints(cv2.minAreaRect(c))
            centers.append((u, v, area, box))
            cv2.circle(frame, (u, v), 5, (0, 255, 0), -1)  # 디버그 영상에 초록색 점 표시
            cv2.drawContours(frame, [box.astype(np.int32)], 0, (255, 0, 0), 2)

        if not centers:
            return None

        # 한 색 = 한 물체 계약. 같은 색 blob이 여러 개여도 가장 큰 것 하나만 쓴다.
        # 팔에 가려져 조각나면 blob이 여러 개로 보이는데, 그때 id가 흔들리면
        # 복구 후 재시도할 물건을 잃어버린다. 복구 대상은 항상 같아야 한다.
        centers.sort(key=lambda c: c[2], reverse=True)   # 면적 큰 순 = 크고 확실한 것 순
        if len(centers) > 1:
            self.get_logger().warn(
                f'{object_id} blob {len(centers)}개 - 가장 큰 것만 발행',
                throttle_duration_sec=2.0
            )
        u, v, _, box = centers[0]
        return (u, v, box)

    def to_object(self, found, object_id, spec, stamp):
        """찾은 덩어리를 DetectedObject로 바꾼다. (물체, yaw, 관측 길이) 또는 None.

        yaw와 길이는 로그에 쓰라고 같이 돌려준다. 물체마다 따로 찍으면
        throttle이 호출 지점 기준이라 사전 순서상 앞선 물체만 통과한다.
        """
        u, v, box = found
        p = self.pixel_to_world(u, v, stamp, spec['plane_z'])
        if p is None:
            self.get_logger().warn(
                f'{object_id} ({u}, {v}) -> 변환 불가', throttle_duration_sec=2.0)
            return None
        # 긴 축을 world에서 되돌린다. yaw와 길이를 한 번에 받는다.
        got = self.long_axis(box, stamp, spec['plane_z'])
        if got is None:
            # 예전엔 yaw=0.0으로 발행했다. 모르는 각도를 0도라고 말하는 건 거짓말이다.
            self.get_logger().warn('축 복원 실패 - 발행 안 함', throttle_duration_sec=2.0)
            return None
        yaw, length = got
        # 가림 게이트. 물체가 팔에 가려지면 보이는 색이 조각난다.
        # 조각의 minAreaRect 중심은 실제 중심이 아니다(실측 y 오차 26.8mm).
        # 틀린 좌표를 발행하느니 아무것도 발행하지 않는다.
        # ⚠️ 약한 가림(6.6cm)은 정상 범위와 겹쳐 여기서 못 막는다 - 팔을 치우고 재야 한다.
        if length < spec['min_long']:
            self.get_logger().warn(
                f'{object_id} 긴 변 {length * 100:.1f}cm - 가림 의심, 발행 안 함',
                throttle_duration_sec=2.0)
            return None

        obj = DetectedObject()
        obj.object_id = object_id   # 색이 곧 identity다
        obj.pose.header.stamp = stamp
        obj.pose.header.frame_id = WORLD_FRAME
        obj.pose.pose.position.x = p[0]
        obj.pose.pose.position.y = p[1]
        obj.pose.pose.position.z = spec['plane_z']
        # 테이블 평면 위의 회전이라 z축 회전 쿼터니언 하나로 계산한다.
        obj.pose.pose.orientation.z = math.sin(yaw / 2.0)
        obj.pose.pose.orientation.w = math.cos(yaw / 2.0)
        obj.last_seen = stamp
        return (obj, yaw, length)

    def on_image(self, msg):
        # ROS Image -> OpenCV 배열, bgr8을 명시해야 한다.
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # BGR -> HSV 조명이 변해도 H는 변하지 않는다.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 스냅샷은 물체가 여럿이어도 하나다. 빈 스냅샷도 발행한다.
        scene = SceneState()
        scene.header.stamp = msg.header.stamp
        scene.header.frame_id = WORLD_FRAME

        parts = []
        for object_id, spec in OBJECTS.items():
            found = self.find_blob(hsv, frame, spec, object_id)
            if found is None:
                continue
            got = self.to_object(found, object_id, spec, msg.header.stamp)
            if got is None:
                continue
            obj, yaw, length = got
            scene.objects.append(obj)
            # 긴 변도 여기서 같이 찍는다. 물체별로 따로 찍으면 throttle에 먹힌다.
            parts.append(
                f'{obj.object_id} '
                f'({obj.pose.pose.position.x:.3f}, {obj.pose.pose.position.y:.3f}) '
                f'yaw {math.degrees(yaw):.1f}도 긴변 {length * 100:.1f}cm')

        self.scene_pub.publish(scene)

        if parts:
            self.get_logger().info(
                f'/scene_state {len(scene.objects)}개 : ' + ', '.join(parts),
                throttle_duration_sec=1.0)
        else:
            self.get_logger().warn('물체 없음', throttle_duration_sec=2.0)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))


def main():
    rclpy.init()
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
