#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# Day 77 — 테스트 경로 발행 노드

# nav_msgs/Path 형식으로 테스트 경로를 발행.
# Nav2 없이도 Pure Pursuit을 테스트할 수 있음!

# 사용:
    # ros2 run my_robot_control path_publisher_node \
    # --ros-args -p path_type:=square -p delay:=3.0
# ════════════════════════════════════════════════════════════════

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import math
from my_robot_control.path_generator import (
    generate_straight, generate_circle,
    generate_square, generate_slalom, generate_figure8
)



class PathPublisherNode(Node):
    """테스트 경로 발행 노드."""

    def __init__(self):
        super().__init__('path_publisher_node')

        self.declare_parameter('path_type', 'square')
        self.declare_parameter('delay', 3.0)
        # delay: 시작 후 N초 뒤에 경로 발행 → Gazebo + EKF가 안정화될 시간!

        self.declare_parameter('path_frame', 'odom')
        self.declare_parameter('side_length', 2.0)
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('resolution', 0.05)

        path_type = self.get_parameter('path_type').value
        delay = self.get_parameter('delay').value
        frame = self.get_parameter('path_frame').value
        side = self.get_parameter('side_length').value
        radius = self.get_parameter('radius').value
        res = self.get_parameter('resolution').value


        # Transient Local QoS: 구독자가 나중에 연결되어도 받음!
        path_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.path_pub = self.create_publisher(Path, '/path', path_qos)


        # 경로 생성
        self.waypoints = self._generate_path(
            path_type, side, radius, res
        )
        self.frame = frame
        self.path_type = path_type


        # 지연 후 발행
        self.timer = self.create_timer(delay, self.publish_path)

        self.get_logger().info(
            f'[PathPub] Will publish "{path_type}" path '
            f'({len(self.waypoints)} points) in {delay}s...'
        )



    def _generate_path(self, path_type, side, radius, res):
        """경로 생성."""
        if path_type == 'straight':
            return generate_straight(side, res)
        elif path_type == 'circle':
            return generate_circle(radius, res)
        elif path_type == 'square':
            return generate_square(side, res)
        elif path_type == 'slalom':
            return generate_slalom(0.5, 2.0, 6.0, res)
        elif path_type == 'figure8':
            return generate_figure8(radius, res)
        else:
            self.get_logger().warn(
                f'[PathPub] Unknown type "{path_type}", using straight.'
            )
            return generate_straight(3.0, res)



    def publish_path(self):
        """경로 발행."""
        # 한 번만 발행!
        self.timer.cancel()

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.frame

        for wx, wy in self.waypoints:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.position.z = 0.0
            # 방향은 다음 점을 향하도록!
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

        self.path_pub.publish(path_msg)
        self.get_logger().info(
            f'[PathPub] ✅ Published "{self.path_type}" path! '
            f'({len(self.waypoints)} points)'
        )


    


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()