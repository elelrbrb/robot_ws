#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════
# Day 50 — Depth 이미지 처리 + 3D 좌표 추출 노드
    # 기능:
        # 1. Aligned Depth 이미지 구독
        # 2. 특정 픽셀의 3D 좌표 계산 (역투영)
        # 3. ROI 영역의 평균 깊이 계산
        # 4. 가까운 물체 감지 (깊이 기반)

# 토픽:
    # 구독: /camera/aligned_depth_to_color/image_raw
        # /camera/color/camera_info
    # 발행: /depth_info (진단)
# ════════════════════════════════════════════════════════════

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

from cv_bridge import CvBridge
    # cv_bridge: ROS Image ↔ OpenCV np.array 변환!
    # sudo apt install ros-humble-cv-bridge

import numpy as np
import math



class DepthProcessorNode(Node):
    """Aligned Depth 이미지를 처리하는 노드."""

    def __init__(self):
        super().__init__('depth_processor_node')

        self.bridge = CvBridge()
            # CvBridge: ROS ↔ OpenCV 변환기
            # Image (ROS 메시지) → np.ndarray (OpenCV)
            # 이것 없이는 이미지 처리 불가!

        self.camera_info = None
            # CameraInfo를 한 번 받아서 저장

        # ---- 구독 ----
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/aligned_depth_to_color/camera_info',
            self.info_callback,
            10
        )

        # ---- 발행 ----
        self.info_pub = self.create_publisher(String, '/depth_info', 10)

        self.frame_count = 0
        self.get_logger().info('[DepthProcessor] Waiting for depth + camera_info...')



    def info_callback(self, msg: CameraInfo):
        """CameraInfo 저장 (1회만 로깅)."""
        if self.camera_info is None:
            self.get_logger().info(
                f'[CameraInfo] fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}, '
                f'cx={msg.k[2]:.1f}, cy={msg.k[5]:.1f}, '
                f'size={msg.width}x{msg.height}'
            )
        self.camera_info = msg



    def depth_callback(self, msg: Image):
        """Aligned Depth 이미지 처리."""
        if self.camera_info is None:
            return

        self.frame_count += 1

        # ---- ROS Image → NumPy 배열 ----
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # passthrough: 원본 인코딩 그대로!
            # Depth: 16UC1 → np.uint16 (밀리미터)
            # 각 픽셀 = 0~65535 mm

        # ---- 기본 통계 ----
        valid_mask = depth_image > 0
            # 0 = 측정 실패!
        valid_count = np.count_nonzero(valid_mask)
        total = depth_image.size

        if valid_count == 0:
            return

        valid_depths = depth_image[valid_mask].astype(np.float32) / 1000.0
            # mm → m


        # ---- 이미지 중앙 픽셀의 3D 좌표 ----
        center_u = msg.width // 2
        center_v = msg.height // 2
        center_3d = self._deproject(center_u, center_v, depth_image)


        # ---- 전방 영역 최소 깊이 (장애물 감지) ----
        # 이미지 중앙 1/3 영역
        h, w = depth_image.shape
        roi = depth_image[h//3:2*h//3, w//3:2*w//3]
        roi_valid = roi[roi > 0].astype(np.float32) / 1000.0

        min_depth = np.min(roi_valid) if len(roi_valid) > 0 else float('inf')


        # ---- 결과 출력 (10 프레임마다) ----
        if self.frame_count % 10 == 0:
            info = (
                f'[Frame {self.frame_count}] '
                f'Valid: {valid_count}/{total} ({100*valid_count/total:.0f}%) | '
                f'Range: {np.min(valid_depths):.2f}~{np.max(valid_depths):.2f}m | '
                f'Median: {np.median(valid_depths):.2f}m'
            )

            if center_3d:
                info += f' | Center3D: ({center_3d[0]:.3f}, {center_3d[1]:.3f}, {center_3d[2]:.3f})'

            info += f' | FrontMin: {min_depth:.2f}m'

            self.get_logger().info(info)

            status_msg = String()
            status_msg.data = info
            self.info_pub.publish(status_msg)



    def _deproject(self, u, v, depth_image):
        """픽셀 + 깊이 → 3D 좌표."""
        if v < 0 or v >= depth_image.shape[0]:
            return None
        if u < 0 or u >= depth_image.shape[1]:
            return None

        depth_mm = int(depth_image[v, u])
        if depth_mm == 0:
            return None

        depth_m = depth_mm / 1000.0

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        X = (u - cx) * depth_m / fx
        Y = (v - cy) * depth_m / fy
        Z = depth_m

        return (X, Y, Z)



def main(args=None):
    rclpy.init(args=args)
    node = DepthProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()