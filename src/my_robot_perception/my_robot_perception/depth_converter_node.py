#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


class DepthConverterNode(Node):
    """Depth Camera 32FC1 → 16UC1 변환 노드"""

    def __init__(self):
        super().__init__('depth_converter_node')

        self.bridge = CvBridge()

        # 구독: 32FC1 depth 이미지
        self.depth_sub = self.create_subscription(
            Image,
            '/depth_camera/depth/image_raw',
            self.depth_callback,
            10
        )

        # 발행: 16UC1 depth 이미지
        self.depth_pub = self.create_publisher(
            Image,
            '/depth_camera/depth/image_raw_16UC1',
            10
        )

        self.get_logger().info('[DepthConverter] Ready to convert 32FC1 → 16UC1')



    def depth_callback(self, msg: Image):
        # 32FC1 → OpenCV
        depth_float = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

        # meter → millimeter, uint16 변환
        depth_uint16 = np.clip(depth_float * 1000.0, 0, 65535).astype(np.uint16)

        # ROS Image 메시지로 변환
        img_msg = self.bridge.cv2_to_imgmsg(depth_uint16, encoding='16UC1')
        img_msg.header = msg.header

        # 발행
        self.depth_pub.publish(img_msg)



def main(args=None):
    rclpy.init(args=args)
    node = DepthConverterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()