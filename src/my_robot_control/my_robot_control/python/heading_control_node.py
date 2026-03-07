#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# heading_control_node.py
# Day 74 — 방향 제어 ROS 2 노드

# 기능:
    # 1. 목표 Yaw 수신 (/goal_heading)
    # 2. 현재 Yaw 구독 (/odometry/filtered)
    # 3. PID → ω 계산
    # 4. cmd_vel 발행 (v=0, ω=PID)
    # 5. 도달 시 완료 알림

# 사용법:
    # ros2 run my_robot_control heading_control_node
    # ros2 topic pub /goal_heading std_msgs/Float64 "{data: 1.57}"
    # → 로봇이 90° (π/2) 방향으로 회전!
# ════════════════════════════════════════════════════════════════

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool, String

import math

from my_robot_control.heading_controller import HeadingPIDController, normalize_angle



class HeadingControlNode(Node):
    """방향 제어 ROS 2 노드."""

    def __init__(self):
        super().__init__('heading_control_node')

        # ── 파라미터 ──
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.5)
        self.declare_parameter('max_omega', 1.0)
        self.declare_parameter('min_omega', 0.05)
        self.declare_parameter('goal_tolerance', 0.02)
        self.declare_parameter('deceleration_zone', 0.3)
        self.declare_parameter('control_rate', 50.0)

        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        max_omega = self.get_parameter('max_omega').value
        min_omega = self.get_parameter('min_omega').value
        tol = self.get_parameter('goal_tolerance').value
        decel = self.get_parameter('deceleration_zone').value
        rate = self.get_parameter('control_rate').value

        # ── PID 컨트롤러 ──
        self.controller = HeadingPIDController(
            kp=kp, ki=ki, kd=kd,
            max_omega=max_omega,
            min_omega=min_omega,
            goal_tolerance=tol,
            deceleration_zone=decel,
        )

        # ── 상태 ──
        self.current_theta = 0.0
        self.goal_theta = None
        self.active = False
        self.odom_received = False

        # ── 구독 ──
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered',
            self.odom_callback, 10
        )

        self.goal_sub = self.create_subscription(
            Float64, '/goal_heading',
            self.goal_callback, 10
        )

        # ── 발행 ──
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_key', 10)
        self.done_pub = self.create_publisher(Bool, '/heading_done', 10)
        self.diag_pub = self.create_publisher(String, '/heading/diagnostics', 10)

        # ── 제어 타이머 ──
        self.dt = 1.0 / rate
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f'[Heading] Initialized. Kp={kp}, Ki={ki}, Kd={kd}, '
            f'Rate={rate}Hz, Tol={math.degrees(tol):.1f}°'
        )



    def odom_callback(self, msg: Odometry):
        """현재 θ 추출."""
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny, cosy)
        self.odom_received = True



    def goal_callback(self, msg: Float64):
        """목표 θ 수신."""
        self.goal_theta = normalize_angle(msg.data)
        self.active = True
        self.controller.reset()
        diff = math.degrees(
            self.controller.error if self.controller.error != 0
            else normalize_angle(self.goal_theta - self.current_theta)
        )
        self.get_logger().info(
            f'[Heading] New goal: {math.degrees(self.goal_theta):.1f}° '
            f'(current: {math.degrees(self.current_theta):.1f}°, '
            f'diff: {math.degrees(normalize_angle(self.goal_theta - self.current_theta)):.1f}°)'
        )



    def control_loop(self):
        """주기적 제어 루프."""
        if not self.active or not self.odom_received or self.goal_theta is None:
            return

        # PID 계산
        omega = self.controller.compute(
            self.current_theta, self.goal_theta, self.dt
        )

        # cmd_vel 발행
        cmd = Twist()
        cmd.linear.x = 0.0  # 제자리 회전!
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)

        # 도달 확인
        if self.controller.is_at_goal():
            self.active = False

            # 정지 명령
            stop = Twist()
            self.cmd_pub.publish(stop)

            # 완료 알림
            done_msg = Bool()
            done_msg.data = True
            self.done_pub.publish(done_msg)

            final_err = math.degrees(abs(
                normalize_angle(self.goal_theta - self.current_theta)
            ))
            self.get_logger().info(
                f'[Heading] ✅ Goal reached! '
                f'Final error: {final_err:.2f}°'
            )

        # 진단 (10회마다)
        diag = self.controller.get_diagnostics()
        msg = String()
        msg.data = (
            f'err={diag["error_deg"]:+.1f}° '
            f'ω={omega:+.3f} '
            f'P={diag["p_term"]:+.3f} I={diag["i_term"]:+.3f} D={diag["d_term"]:+.3f}'
        )
        self.diag_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HeadingControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 정지 명령
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.get_logger().info('[Heading] Shutdown.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()