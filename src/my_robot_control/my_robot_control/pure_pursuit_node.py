#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# Day 77 — Pure Pursuit ROS 2 경로 추종 노드

# 기능:
    # ① /path 구독 → 경로 설정
    # ② /odometry/filtered 구독 → 현재 위치
    # ③ Pure Pursuit → /cmd_vel 발행
    # ④ RViz 시각화 마커 발행
    # ⑤ 진단 정보 발행
    # ⑥ 경로 끝 도달 시 자동 정지

# 사용:
    # ros2 run my_robot_control pure_pursuit_node
    # ros2 run my_robot_control path_publisher_node --ros-args -p path_type:=square
# ════════════════════════════════════════════════════════════════

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import String, Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from my_robot_control.pure_pursuit import PurePursuitController, Pose2D


def quaternion_to_yaw(q) -> float:
    """Quaternion → Yaw 변환."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)



class PurePursuitNode(Node):

    def __init__(self):
        super().__init__('pure_pursuit_node')

        # ════════════════════════════════════
        # 파라미터 선언
        # ════════════════════════════════════
        self.declare_parameter('lookahead_distance', 0.4)
        self.declare_parameter('min_lookahead', 0.15)
        self.declare_parameter('max_lookahead', 1.0)
        self.declare_parameter('adaptive_lookahead', True)
        self.declare_parameter('lookahead_gain', 0.5)
        self.declare_parameter('desired_velocity', 0.3)
        self.declare_parameter('min_velocity', 0.05)
        self.declare_parameter('max_velocity', 0.5)
        self.declare_parameter('max_omega', 1.5)
        self.declare_parameter('goal_tolerance', 0.1)
        self.declare_parameter('velocity_curvature_gain', 0.3)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('path_frame', 'odom')

        # 파라미터 읽기
        l_d = self.get_parameter('lookahead_distance').value
        l_d_min = self.get_parameter('min_lookahead').value
        l_d_max = self.get_parameter('max_lookahead').value
        adaptive = self.get_parameter('adaptive_lookahead').value
        l_d_gain = self.get_parameter('lookahead_gain').value
        v_desired = self.get_parameter('desired_velocity').value
        v_min = self.get_parameter('min_velocity').value
        v_max = self.get_parameter('max_velocity').value
        max_omega = self.get_parameter('max_omega').value
        goal_tol = self.get_parameter('goal_tolerance').value
        v_curv_gain = self.get_parameter('velocity_curvature_gain').value
        rate = self.get_parameter('control_rate').value
        self.path_frame = self.get_parameter('path_frame').value


        # ════════════════════════════════════
        # Pure Pursuit 엔진 (Day 76!)
        # ════════════════════════════════════
        self.controller = PurePursuitController(
            lookahead_distance=l_d,
            min_lookahead=l_d_min,
            max_lookahead=l_d_max,
            adaptive_lookahead=adaptive,
            lookahead_gain=l_d_gain,
            desired_velocity=v_desired,
            min_velocity=v_min,
            max_velocity=v_max,
            max_omega=max_omega,
            goal_tolerance=goal_tol,
            velocity_curvature_gain=v_curv_gain,
        )


        # ════════════════════════════════════
        # 상태
        # ════════════════════════════════════
        self.robot_pose = Pose2D()
        self.odom_received = False
        self.path_received = False
        self.tracking_active = False
        self.goal_reached = False

        # 통계
        self.cte_sum_sq = 0.0
        self.cte_count = 0
        self.cte_max = 0.0
        self.start_time = None


        # ════════════════════════════════════
        # 구독
        # ════════════════════════════════════
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered',
            self.odom_callback, 10
        )

        # Path는 Transient Local QoS → 구독자가 나중에 연결되어도 마지막 경로를 받을 수 있음
        path_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.path_sub = self.create_subscription(
            Path, '/path',
            self.path_callback, path_qos
        )


        # ════════════════════════════════════
        # 발행
        # ════════════════════════════════════
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_key', 10)
        self.diag_pub = self.create_publisher(
            String, '/pursuit/diagnostics', 10
        )
        self.done_pub = self.create_publisher(
            Bool, '/pursuit/done', 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/pursuit/markers', 10
        )


        # ════════════════════════════════════
        # 제어 타이머
        # ════════════════════════════════════
        self.dt = 1.0 / rate
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f'[PP] Initialized. l_d={l_d}m, v={v_desired}m/s, '
            f'rate={rate}Hz, adaptive={adaptive}'
        )



        # ════════════════════════════════════
        # Watchdog
        # ════════════════════════════════════
        self.last_odom_time = None
        self.odom_timeout = 0.5  # 0.5초 동안 odom 없으면 정지!
        self.watchdog_timer = self.create_timer(0.1, self.watchdog)

        # ════════════════════════════════════
        # Slew Rate
        # ════════════════════════════════════
        self.prev_v = 0.0
        self.prev_omega = 0.0
        self.max_v_rate = 0.5     # m/s² (최대 가속도!)
        self.max_omega_rate = 2.0  # rad/s² (최대 각가속도!)

        # ════════════════════════════════════
        # 파라미터 변경 콜백
        # ════════════════════════════════════
        self.add_on_set_parameters_callback(self.on_parameter_change)


        



    # ════════════════════════════════════════════════════════
    # 콜백
    # ════════════════════════════════════════════════════════
    def odom_callback(self, msg: Odometry):
        """EKF 출력에서 현재 위치 추출."""
        self.robot_pose.x = msg.pose.pose.position.x
        self.robot_pose.y = msg.pose.pose.position.y
        self.robot_pose.theta = quaternion_to_yaw(
            msg.pose.pose.orientation
        )
        self.odom_received = True

        self.last_odom_time = self.get_clock().now()



    def path_callback(self, msg: Path):
        """경로 수신 → Pure Pursuit에 설정."""

        if len(msg.poses) < 2:
            self.get_logger().warn('[PP] Path too short (< 2 points)!')
            return

        # Path → [(x, y), ...] 변환
        waypoints = []
        for ps in msg.poses:
            waypoints.append((
                ps.pose.position.x,
                ps.pose.position.y
            ))

        self.controller.set_path(waypoints)
        self.path_received = True
        self.tracking_active = True
        self.goal_reached = False

        # 통계 초기화
        self.cte_sum_sq = 0.0
        self.cte_count = 0
        self.cte_max = 0.0
        self.start_time = self.get_clock().now()

        self.get_logger().info(
            f'[PP] Path received! {len(waypoints)} waypoints, '
            f'frame={msg.header.frame_id}'
        )



    # ════════════════════════════════════════════════════════
    # 제어 루프
    # ════════════════════════════════════════════════════════
    def control_loop(self):
        """주기적 제어 (50Hz)."""

        if not self.odom_received:
            return

        if not self.tracking_active or self.goal_reached:
            return

        if not self.path_received:
            return

        # ── Pure Pursuit 계산 ──
        cmd = self.controller.compute(self.robot_pose)

        # ── 목표 도달? ──
        if cmd.at_goal:
            self._on_goal_reached()
            return


        # ── Slew Rate 적용 (수정: 이전에는 호출되지 않음) ──
        v_cmd, omega_cmd = self._apply_slew_rate(
            cmd.linear_velocity, 
            cmd.angular_velocity
        )


        # ── cmd_vel 발행 ──
        twist = Twist()
        twist.linear.x = v_cmd
        twist.angular.z = omega_cmd
        self.cmd_pub.publish(twist)

        # ── CTE 통계 ──
        cte_abs = abs(cmd.cross_track_error)
        self.cte_sum_sq += cte_abs ** 2
        self.cte_count += 1
        if cte_abs > self.cte_max:
            self.cte_max = cte_abs

        # ── 시각화 ──
        self._publish_markers(cmd)

        # ── 진단 (10스텝마다) ──
        if self.cte_count % 10 == 0:
            self._publish_diagnostics(cmd)


    def _on_goal_reached(self):
        """목표 도달 처리."""
        self.goal_reached = True
        self.tracking_active = False

        # 정지!
        stop = Twist()
        self.cmd_pub.publish(stop)

        # 완료 알림
        done_msg = Bool()
        done_msg.data = True
        self.done_pub.publish(done_msg)

        # 최종 통계
        elapsed = 0.0
        if self.start_time is not None:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        cte_rmse = 0.0
        if self.cte_count > 0:
            cte_rmse = math.sqrt(self.cte_sum_sq / self.cte_count)

        self.get_logger().info(
            f'[PP] ✅ Goal reached!\n'
            f'  Time: {elapsed:.1f}s\n'
            f'  CTE RMSE: {cte_rmse*100:.1f}cm\n'
            f'  CTE Max: {self.cte_max*100:.1f}cm\n'
            f'  Progress: {self.controller.get_progress()*100:.0f}%'
        )



    # ════════════════════════════════════════════════════════
    # 시각화 (RViz 마커!)
    # ════════════════════════════════════════════════════════

    def _publish_markers(self, cmd):
        """
        RViz 시각화 마커 발행.

        ① Lookahead Point (빨간 구!)
        ② Robot → Lookahead 연결선 (빨간 선!)
        ③ CTE 표시선 (노란 선!)
        """
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # ── 마커 1: Lookahead Point ──
        la_marker = Marker()
        la_marker.header.frame_id = self.path_frame
        la_marker.header.stamp = stamp
        la_marker.ns = 'pursuit'
        la_marker.id = 0
        la_marker.type = Marker.SPHERE
        la_marker.action = Marker.ADD
        la_marker.pose.position.x = cmd.lookahead_x
        la_marker.pose.position.y = cmd.lookahead_y
        la_marker.pose.position.z = 0.1
        la_marker.pose.orientation.w = 1.0
        la_marker.scale.x = 0.08
        la_marker.scale.y = 0.08
        la_marker.scale.z = 0.08
        la_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)
        la_marker.lifetime.sec = 0
        la_marker.lifetime.nanosec = 200_000_000  # 0.2s
        markers.markers.append(la_marker)


        # ── 마커 2: Robot → Lookahead 선 ──
        line_marker = Marker()
        line_marker.header.frame_id = self.path_frame
        line_marker.header.stamp = stamp
        line_marker.ns = 'pursuit'
        line_marker.id = 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.02  # 선 두께

        p1 = Point()
        p1.x = self.robot_pose.x
        p1.y = self.robot_pose.y
        p1.z = 0.05

        p2 = Point()
        p2.x = cmd.lookahead_x
        p2.y = cmd.lookahead_y
        p2.z = 0.05

        line_marker.points = [p1, p2]
        line_marker.color = ColorRGBA(r=1.0, g=0.3, b=0.0, a=0.7)
        line_marker.lifetime.sec = 0
        line_marker.lifetime.nanosec = 200_000_000
        markers.markers.append(line_marker)


        # ── 마커 3: CTE 표시 ──
        if cmd.nearest_index < len(self.controller.path) - 1:
            idx = cmd.nearest_index
            ax, ay = self.controller.path[idx]
            bx, by = self.controller.path[idx + 1]

            # 로봇에서 경로까지 수직 투영점
            abx = bx - ax
            aby = by - ay
            ab_sq = abx**2 + aby**2
            if ab_sq > 1e-9:
                arx = self.robot_pose.x - ax
                ary = self.robot_pose.y - ay
                t_proj = (arx * abx + ary * aby) / ab_sq
                t_proj = max(0.0, min(1.0, t_proj))
                proj_x = ax + t_proj * abx
                proj_y = ay + t_proj * aby

                cte_marker = Marker()
                cte_marker.header.frame_id = self.path_frame
                cte_marker.header.stamp = stamp
                cte_marker.ns = 'pursuit'
                cte_marker.id = 2
                cte_marker.type = Marker.LINE_STRIP
                cte_marker.action = Marker.ADD
                cte_marker.scale.x = 0.015

                p3 = Point()
                p3.x = self.robot_pose.x
                p3.y = self.robot_pose.y
                p3.z = 0.03

                p4 = Point()
                p4.x = proj_x
                p4.y = proj_y
                p4.z = 0.03

                cte_marker.points = [p3, p4]
                cte_marker.color = ColorRGBA(
                    r=1.0, g=1.0, b=0.0, a=0.8
                )
                cte_marker.lifetime.sec = 0
                cte_marker.lifetime.nanosec = 200_000_000
                markers.markers.append(cte_marker)

        self.marker_pub.publish(markers)



    # ════════════════════════════════════════════════════════
    # 진단
    # ════════════════════════════════════════════════════════
    def _publish_diagnostics(self, cmd):
        """진단 정보 발행."""
        cte_rmse = 0.0
        if self.cte_count > 0:
            cte_rmse = math.sqrt(self.cte_sum_sq / self.cte_count)

        msg = String()
        msg.data = (
            f'v={cmd.linear_velocity:.2f} '
            f'ω={cmd.angular_velocity:+.2f} '
            f'κ={cmd.curvature:+.2f} '
            f'CTE={cmd.cross_track_error*100:+.1f}cm '
            f'RMSE={cte_rmse*100:.1f}cm '
            f'Max={self.cte_max*100:.1f}cm '
            f'Prog={self.controller.get_progress()*100:.0f}%'
        )
        self.diag_pub.publish(msg)


    



    def watchdog(self):
        """통신 감시 — odom이 끊기면 긴급 정지!"""
        if self.last_odom_time is None:
            return
        if not self.tracking_active:
            return

        elapsed = (self.get_clock().now() -
                   self.last_odom_time).nanoseconds * 1e-9

        if elapsed > self.odom_timeout:
            # 긴급 정지!
            stop = Twist()
            self.cmd_pub.publish(stop)
            self.tracking_active = False

            self.get_logger().error(
                f'[PP] ⚠️ WATCHDOG! No odom for {elapsed:.1f}s! '
                f'Emergency stop!'
            )

    
    def _apply_slew_rate(self, v_cmd, omega_cmd):
        """속도 변화율 제한 — 급격한 명령 방지!"""

        # 선속도 제한
        dv = v_cmd - self.prev_v
        max_dv = self.max_v_rate * self.dt
        if abs(dv) > max_dv:
            v_cmd = self.prev_v + max_dv * (1 if dv > 0 else -1)

        # 각속도 제한
        domega = omega_cmd - self.prev_omega
        max_do = self.max_omega_rate * self.dt
        if abs(domega) > max_do:
            omega_cmd = self.prev_omega + max_do * (1 if domega > 0 else -1)

        self.prev_v = v_cmd
        self.prev_omega = omega_cmd

        return v_cmd, omega_cmd



    def on_parameter_change(self, params):
        """
            파라미터 실시간 변경 처리
                → 실행 중에 파라미터 변경! → 즉시 반영!
                → 튜닝 시 재시작 없이 실험!
        """
        from rcl_interfaces.msg import SetParametersResult

        for param in params:
            name = param.name
            val = param.value

            if name == 'lookahead_distance':
                self.controller.l_d_base = val
                self.get_logger().info(f'[PP] l_d → {val}')

            elif name == 'desired_velocity':
                self.controller.v_desired = val
                self.get_logger().info(f'[PP] v → {val}')

            elif name == 'velocity_curvature_gain':
                self.controller.v_curv_gain = val
                self.get_logger().info(f'[PP] curv_gain → {val}')

            elif name == 'max_omega':
                self.controller.max_omega = val
                self.get_logger().info(f'[PP] max_ω → {val}')

            elif name == 'adaptive_lookahead':
                self.controller.adaptive = val
                self.get_logger().info(f'[PP] adaptive → {val}')

        return SetParametersResult(successful=True)






def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.get_logger().info('[PP] Shutdown — stop command sent.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()