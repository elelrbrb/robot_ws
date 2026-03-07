#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# figure8_challenge.py
# Day 81 — Figure-8 Challenge: 종합 평가 + 실시간 시각화
#
# 기능:
#   ① Figure-8 경로 생성 + 발행
#   ② Pure Pursuit 추종
#   ③ 실시간 메트릭 모니터링
#   ④ 구간별 성능 분석 (상단 원, 교차점, 하단 원)
#   ⑤ 완료 시 종합 리포트
# ════════════════════════════════════════════════════════════════

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

import math
import numpy as np
from typing import List, Tuple

from my_robot_control.pure_pursuit import PurePursuitController, Pose2D
from my_robot_control.figure8_path import generate_figure8_smooth, compute_path_properties


def quat_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))



class Figure8ChallengeNode(Node):
    """Figure-8 Challenge 노드."""

    def __init__(self):
        super().__init__('figure8_challenge_node')

        # ── 파라미터 ──
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('velocity', 0.25)
        self.declare_parameter('lookahead', 0.35)
        self.declare_parameter('curv_gain', 0.3)
        self.declare_parameter('num_laps', 1)
        self.declare_parameter('delay', 5.0)

        self.radius = self.get_parameter('radius').value
        v = self.get_parameter('velocity').value
        l_d = self.get_parameter('lookahead').value
        cg = self.get_parameter('curv_gain').value
        self.num_laps = self.get_parameter('num_laps').value
        delay = self.get_parameter('delay').value


        # ── 경로 생성 ──
        self.path = generate_figure8_smooth(
            radius=self.radius, resolution=0.03
        )
        self.props = compute_path_properties(self.path)
        self.get_logger().info(
            f'[F8] Path: {self.props["num_points"]} pts, '
            f'Length={self.props["total_length"]:.2f}m, '
            f'MinRadius={self.props["min_radius"]:.2f}m'
        )

        # 멀티 랩!
        if self.num_laps > 1:
            full_path = list(self.path)
            for _ in range(self.num_laps - 1):
                full_path.extend(self.path[1:])
            self.path = full_path
            self.get_logger().info(
                f'[F8] {self.num_laps} laps → {len(self.path)} pts'
            )


        # ── Pure Pursuit ──
        self.controller = PurePursuitController(
            lookahead_distance=l_d,
            min_lookahead=0.12,
            max_lookahead=0.8,
            adaptive_lookahead=True,
            lookahead_gain=0.5,
            desired_velocity=v,
            min_velocity=0.05,
            max_velocity=0.4,
            max_omega=1.5,
            goal_tolerance=0.12,
            velocity_curvature_gain=cg,
        )
        self.controller.set_path(self.path)

        # ── 상태 ──
        self.robot_pose = Pose2D()
        self.odom_received = False
        self.active = False
        self.start_time = None

        # ── 구간 분류 ──
        self._classify_segments()

        # ── 데이터 수집 ──
        self.gt_log = []       # (t, x, y, θ, vx, wz)
        self.cte_log = []      # (t, cte, segment_type)
        self.cmd_log = []      # (t, v, ω)
        self.he_log = []       # (t, heading_error)


        # ── 구독 ──
        self.gt_sub = self.create_subscription(
            Odometry, '/ground_truth/odom',
            self.gt_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered',
            self.odom_callback, 10
        )

        # ── 발행 ──
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_key', 10)
        path_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1
        )
        self.path_pub = self.create_publisher(Path, '/path', path_qos)
        self.diag_pub = self.create_publisher(
            String, '/f8/diagnostics', 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/f8/markers', 10
        )


        # ── 타이머 ──
        self.control_timer = self.create_timer(0.02, self.control_loop)
        self.publish_timer = self.create_timer(delay, self.start_challenge)

        self.get_logger().info(
            f'[F8] Challenge ready! Starting in {delay}s...'
        )



    # ═══════════════════════════════════════════════════════
    #  구간 분류
    # ═══════════════════════════════════════════════════════
    def _classify_segments(self):
        """
        경로를 구간별로 분류.

        ① 상단 원 (y > 0.1)
        ② 하단 원 (y < -0.1)
        ③ 교차 구간 (|y| < 0.1)
        """
        self.segment_types = []
        for x, y in self.path:
            if y > 0.1 * self.radius:
                self.segment_types.append('upper')
            elif y < -0.1 * self.radius:
                self.segment_types.append('lower')
            else:
                self.segment_types.append('crossover')


    # ═══════════════════════════════════════════════════════
    #  콜백
    # ═══════════════════════════════════════════════════════

    def gt_callback(self, msg: Odometry):
        if not self.active:
            return
        t = self._elapsed()
        p = msg.pose.pose.position
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.gt_log.append((
            t, p.x, p.y, yaw,
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z
        ))

    def odom_callback(self, msg: Odometry):
        self.robot_pose.x = msg.pose.pose.position.x
        self.robot_pose.y = msg.pose.pose.position.y
        self.robot_pose.theta = quat_to_yaw(msg.pose.pose.orientation)
        self.odom_received = True


    def _elapsed(self) -> float:
        if self.start_time is None:
            return 0.0
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9


    # ═══════════════════════════════════════════════════════
    #  시작
    # ═══════════════════════════════════════════════════════
    def start_challenge(self):
        """도전 시작!"""
        self.publish_timer.cancel()

        # 경로 발행
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        for wx, wy in self.path:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self.path_pub.publish(path_msg)

        self.active = True
        self.start_time = self.get_clock().now()

        self.get_logger().info(
            f'[F8] ═══════════════════════════════════════')
        self.get_logger().info(
            f'[F8]  FIGURE-8 CHALLENGE STARTED!')
        self.get_logger().info(
            f'[F8]  Radius={self.radius}m, Laps={self.num_laps}')
        self.get_logger().info(
            f'[F8] ═══════════════════════════════════════')



    # ═══════════════════════════════════════════════════════
    #  제어 루프
    # ═══════════════════════════════════════════════════════
    def control_loop(self):
        if not self.active or not self.odom_received:
            return

        # Pure Pursuit
        cmd = self.controller.compute(self.robot_pose)
        t = self._elapsed()

        if cmd.at_goal:
            self._on_complete()
            return

        # cmd_vel 발행
        twist = Twist()
        twist.linear.x = cmd.linear_velocity
        twist.angular.z = cmd.angular_velocity
        self.cmd_pub.publish(twist)

        # CTE 기록 (구간 포함!)
        seg_idx = min(cmd.nearest_index, len(self.segment_types) - 1)
        seg_type = self.segment_types[seg_idx]
        self.cte_log.append((t, cmd.cross_track_error, seg_type))
        self.cmd_log.append((t, cmd.linear_velocity, cmd.angular_velocity))

        # Heading error 계산
        if cmd.nearest_index < len(self.path) - 1:
            ax, ay = self.path[cmd.nearest_index]
            bx, by = self.path[cmd.nearest_index + 1]
            path_theta = math.atan2(by - ay, bx - ax)
            he = normalize_angle(self.robot_pose.theta - path_theta)
            self.he_log.append((t, he))

        # 진단 (50스텝마다)
        if len(self.cte_log) % 50 == 0:
            self._publish_diagnostics(cmd, seg_type)



    # ═══════════════════════════════════════════════════════
    #  완료 처리
    # ═══════════════════════════════════════════════════════
    def _on_complete(self):
        self.active = False

        stop = Twist()
        self.cmd_pub.publish(stop)

        self.get_logger().info('[F8] ✅ CHALLENGE COMPLETE!')
        self._generate_report()


    def _generate_report(self):
        """종합 리포트."""

        if len(self.cte_log) < 10:
            self.get_logger().warn('[F8] Insufficient data!')
            return

        total_time = self.cte_log[-1][0]

        # ── 전체 CTE ──
        cte_all = np.array([c[1] for c in self.cte_log])
        cte_rmse = float(np.sqrt(np.mean(cte_all**2)))
        cte_max = float(np.max(np.abs(cte_all)))

        # ── 구간별 CTE ──
        seg_cte = {'upper': [], 'lower': [], 'crossover': []}
        for t, cte, seg in self.cte_log:
            seg_cte[seg].append(cte)

        seg_stats = {}
        for seg, vals in seg_cte.items():
            if vals:
                arr = np.array(vals)
                seg_stats[seg] = {
                    'rmse': float(np.sqrt(np.mean(arr**2))),
                    'max': float(np.max(np.abs(arr))),
                    'count': len(vals),
                }
            else:
                seg_stats[seg] = {'rmse': 0, 'max': 0, 'count': 0}

        # ── Heading Error ──
        he_arr = np.array([h[1] for h in self.he_log]) if self.he_log else np.array([0])
        he_rmse = float(np.sqrt(np.mean(he_arr**2)))

        # ── Smoothness ──
        if len(self.cmd_log) > 1:
            omega_arr = np.array([c[2] for c in self.cmd_log])
            dt = total_time / len(self.cmd_log)
            omega_jerk = np.diff(omega_arr) / dt
            smooth = float(np.sqrt(np.mean(omega_jerk**2)))
        else:
            smooth = 0.0

        # ── 속도 통계 ──
        v_arr = np.array([c[1] for c in self.cmd_log])
        v_mean = float(np.mean(v_arr))

        # ── 리포트 출력 ──
        print()
        print('╔' + '═' * 60 + '╗')
        print('║' + '  FIGURE-8 CHALLENGE — FINAL REPORT'.center(60) + '║')
        print('╠' + '═' * 60 + '╣')
        print('║' + f'  Radius: {self.radius}m | '
              f'Laps: {self.num_laps} | '
              f'Time: {total_time:.1f}s'.ljust(60) + '║')
        print('╠' + '═' * 60 + '╣')
        print('║' + ''.ljust(60) + '║')
        print('║' + '  ┌─ Overall Performance ──────────────────────┐'.ljust(60) + '║')
        print('║' + f'  │  CTE RMSE:      {cte_rmse*100:>6.2f} cm'
              f'                     │'.ljust(60) + '║')
        print('║' + f'  │  CTE Max:       {cte_max*100:>6.2f} cm'
              f'                     │'.ljust(60) + '║')
        print('║' + f'  │  HE RMSE:       {math.degrees(he_rmse):>6.2f} °'
              f'                      │'.ljust(60) + '║')
        print('║' + f'  │  Smoothness:    {smooth:>6.2f} rad/s²'
              f'                │'.ljust(60) + '║')
        print('║' + f'  │  Avg Velocity:  {v_mean:>6.3f} m/s'
              f'                  │'.ljust(60) + '║')
        print('║' + '  └────────────────────────────────────────────┘'.ljust(60) + '║')
        print('║' + ''.ljust(60) + '║')
        print('║' + '  ┌─ Segment Analysis ─────────────────────────┐'.ljust(60) + '║')
        for seg in ['upper', 'lower', 'crossover']:
            s = seg_stats[seg]
            label = {'upper': 'Upper Loop  ', 'lower': 'Lower Loop  ',
                     'crossover': 'Crossover   '}[seg]
            print('║' + f'  │  {label} RMSE={s["rmse"]*100:>5.2f}cm  '
                  f'Max={s["max"]*100:>5.2f}cm  '
                  f'({s["count"]:>4} pts) │'.ljust(60) + '║')
        print('║' + '  └────────────────────────────────────────────┘'.ljust(60) + '║')
        print('║' + ''.ljust(60) + '║')

        # 등급
        if cte_rmse * 100 < 2.0:
            grade = '★★★★★ EXCELLENT'
        elif cte_rmse * 100 < 3.5:
            grade = '★★★★☆ GOOD'
        elif cte_rmse * 100 < 6.0:
            grade = '★★★☆☆ ACCEPTABLE'
        elif cte_rmse * 100 < 10.0:
            grade = '★★☆☆☆ NEEDS TUNING'
        else:
            grade = '★☆☆☆☆ POOR'

        print('║' + f'  GRADE: {grade}'.ljust(60) + '║')
        print('║' + ''.ljust(60) + '║')
        print('╚' + '═' * 60 + '╝')
        print()

        # ── 시각화 저장 ──
        self._save_plots(cte_all, seg_cte, he_arr, total_time)



    def _save_plots(self, cte_all, seg_cte, he_arr, total_time):
        """결과 시각화."""
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(2, 2, figsize=(14, 12))
        fig.suptitle('Figure-8 Challenge — Results', fontsize=13,
                     fontweight='bold')

        # Panel 1: XY 궤적
        ax = axes[0, 0]
        px = [p[0] for p in self.path]
        py = [p[1] for p in self.path]
        ax.plot(px, py, 'k--', linewidth=2, alpha=0.4, label='Path')
        if self.gt_log:
            ax.plot([g[1] for g in self.gt_log],
                    [g[2] for g in self.gt_log],
                    'b-', linewidth=1.2, label='Robot (GT)')
        ax.plot(0, 0, 'r*', markersize=15, label='Crossover')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Trajectory')
        ax.legend(fontsize=8)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)

        # Panel 2: CTE 구간별!
        ax = axes[0, 1]
        colors = {'upper': 'blue', 'crossover': 'red', 'lower': 'green'}
        for seg, vals in seg_cte.items():
            if vals:
                # 시간 인덱스 근사
                indices = [i for i, (_, _, s) in enumerate(self.cte_log)
                           if s == seg]
                times = [self.cte_log[i][0] for i in indices]
                ax.scatter(times, [abs(v)*100 for v in vals],
                           s=1, c=colors[seg], alpha=0.5, label=seg)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('|CTE| (cm)')
        ax.set_title('CTE by Segment')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # Panel 3: 속도 + 곡률
        ax = axes[1, 0]
        if self.cmd_log:
            t_cmd = [c[0] for c in self.cmd_log]
            v_cmd = [c[1] for c in self.cmd_log]
            w_cmd = [c[2] for c in self.cmd_log]
            ax.plot(t_cmd, v_cmd, 'g-', linewidth=1, label='v')
            ax.plot(t_cmd, w_cmd, 'r-', linewidth=0.8, alpha=0.7,
                    label='ω')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity')
        ax.set_title('Velocity Profile')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # Panel 4: 구간별 요약
        ax = axes[1, 1]
        segs = ['upper', 'crossover', 'lower']
        rmse_vals = []
        max_vals = []
        for s in segs:
            if seg_cte[s]:
                arr = np.array(seg_cte[s])
                rmse_vals.append(float(np.sqrt(np.mean(arr**2))) * 100)
                max_vals.append(float(np.max(np.abs(arr))) * 100)
            else:
                rmse_vals.append(0)
                max_vals.append(0)

        x_pos = np.arange(len(segs))
        width = 0.35
        ax.bar(x_pos - width/2, rmse_vals, width,
               label='RMSE', color='steelblue', alpha=0.8)
        ax.bar(x_pos + width/2, max_vals, width,
               label='Max', color='coral', alpha=0.8)
        ax.set_xticks(x_pos)
        ax.set_xticklabels(['Upper\nLoop', 'Cross-\nover', 'Lower\nLoop'])
        ax.set_ylabel('CTE (cm)')
        ax.set_title('Segment Performance')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3, axis='y')

        plt.tight_layout()
        fig.savefig('figure8_result.png', dpi=150, bbox_inches='tight')
        plt.close(fig)
        print('Saved: figure8_result.png')



    def _publish_diagnostics(self, cmd, seg_type):
        """실시간 진단."""
        t = self._elapsed()
        cte_recent = [abs(c[1]) for c in self.cte_log[-100:]]
        rmse_recent = math.sqrt(sum(c**2 for c in cte_recent) / len(cte_recent)) if cte_recent else 0

        msg = String()
        msg.data = (
            f't={t:.1f}s seg={seg_type:>9s} '
            f'CTE={cmd.cross_track_error*100:+5.1f}cm '
            f'RMSE100={rmse_recent*100:.1f}cm '
            f'v={cmd.linear_velocity:.2f} ω={cmd.angular_velocity:+.2f} '
            f'prog={self.controller.get_progress()*100:.0f}%'
        )
        self.diag_pub.publish(msg)






def main(args=None):
    rclpy.init(args=args)
    node = Figure8ChallengeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop = Twist()
        node.cmd_pub.publish(stop)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()