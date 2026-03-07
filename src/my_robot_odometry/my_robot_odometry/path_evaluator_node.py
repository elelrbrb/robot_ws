#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# Day 80 — 경로 추종 평가 노드

# 실시간으로 GT, EKF, cmd_vel, path를 수집하고
# 추종 완료 시 종합 평가 리포트 생성!
# ════════════════════════════════════════════════════════════════

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

import math
import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional


def quat_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class TimedPose:
    t: float
    x: float
    y: float
    theta: float
    vx: float = 0.0
    omega: float = 0.0


@dataclass
class EvalRecord:
    """수집 데이터."""
    gt_poses: List[TimedPose] = field(default_factory=list)
    ekf_poses: List[TimedPose] = field(default_factory=list)
    cmd_vels: List[Tuple[float, float, float]] = field(default_factory=list)
    # (t, v_cmd, omega_cmd)
    path: List[Tuple[float, float]] = field(default_factory=list)



class PathEvaluatorNode(Node):
    """경로 추종 평가 노드."""

    def __init__(self):
        super().__init__('path_evaluator_node')

        self.record = EvalRecord()
        self.collecting = False
        self.start_time = None

        # ── 구독 ──
        self.gt_sub = self.create_subscription(
            Odometry, '/ground_truth/odom',
            self.gt_callback, 10
        )
        self.ekf_sub = self.create_subscription(
            Odometry, '/odometry/filtered',
            self.ekf_callback, 10
        )
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel_key',
            self.cmd_callback, 10
        )


        path_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.path_sub = self.create_subscription(
            Path, '/path', self.path_callback, path_qos
        )
        self.done_sub = self.create_subscription(
            Bool, '/pursuit/done', self.done_callback, 10
        )

        self.get_logger().info('[Eval] Waiting for path and tracking...')



    def _get_elapsed(self) -> float:
        if self.start_time is None:
            return 0.0
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9



    def path_callback(self, msg: Path):
        self.record.path = [
            (ps.pose.position.x, ps.pose.position.y)
            for ps in msg.poses
        ]
        self.collecting = True
        self.start_time = self.get_clock().now()
        self.get_logger().info(
            f'[Eval] Path received ({len(self.record.path)} pts). '
            f'Collecting data...'
        )


    def gt_callback(self, msg: Odometry):
        if not self.collecting:
            return
        t = self._get_elapsed()
        p = msg.pose.pose.position
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        self.record.gt_poses.append(
            TimedPose(t, p.x, p.y, yaw, vx, wz)
        )


    def ekf_callback(self, msg: Odometry):
        if not self.collecting:
            return
        t = self._get_elapsed()
        p = msg.pose.pose.position
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        self.record.ekf_poses.append(
            TimedPose(t, p.x, p.y, yaw, vx, wz)
        )


    def cmd_callback(self, msg: Twist):
        if not self.collecting:
            return
        t = self._get_elapsed()
        self.record.cmd_vels.append(
            (t, msg.linear.x, msg.angular.z)
        )


    def done_callback(self, msg: Bool):
        if msg.data and self.collecting:
            self.collecting = False
            self.get_logger().info('[Eval] Tracking done! Computing...')
            self._evaluate()


    def _evaluate(self):
        """종합 평가 실행."""
        evaluator = PathTrackingEvaluator(self.record)
        report = evaluator.compute_all()
        evaluator.print_report(report)
        evaluator.save_plots('eval_result')
        self.get_logger().info('[Eval] ✅ Evaluation complete!')





class PathTrackingEvaluator:
    """경로 추종 정량 평가 엔진."""

    def __init__(self, record: EvalRecord):
        self.record = record
        self.path = record.path

    # ═══════════════════════════════════════════════════════
    #  CTE 계산
    # ═══════════════════════════════════════════════════════
    def _compute_cte_for_pose(self, x: float, y: float) -> Tuple[float, int]:
        min_dist_sq = float('inf')
        best_cte = 0.0
        best_idx = 0

        for i in range(len(self.path) - 1):
            ax, ay = self.path[i]
            bx, by = self.path[i + 1]

            abx = bx - ax
            aby = by - ay
            ab_sq = abx**2 + aby**2
            if ab_sq < 1e-12:
                continue

            arx = x - ax
            ary = y - ay
            t = (arx * abx + ary * aby) / ab_sq
            t = max(0.0, min(1.0, t))

            proj_x = ax + t * abx
            proj_y = ay + t * aby

            dx = x - proj_x
            dy = y - proj_y
            dist_sq = dx**2 + dy**2

            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                # 부호: cross product!
                cross = abx * ary - aby * arx
                signed_dist = cross / math.sqrt(ab_sq)
                best_cte = signed_dist
                best_idx = i

        return best_cte, best_idx


    def _compute_heading_error(self, theta: float,
                                seg_idx: int) -> float:
        """경로 접선 방향 대비 heading error."""
        if seg_idx >= len(self.path) - 1:
            seg_idx = len(self.path) - 2

        ax, ay = self.path[seg_idx]
        bx, by = self.path[seg_idx + 1]
        path_theta = math.atan2(by - ay, bx - ax)
        return normalize_angle(theta - path_theta)



    # ═══════════════════════════════════════════════════════
    #  메트릭 계산
    # ═══════════════════════════════════════════════════════
    def compute_all(self) -> dict:
        """전체 메트릭 계산."""

        gt = self.record.gt_poses
        ekf = self.record.ekf_poses
        cmds = self.record.cmd_vels

        if len(gt) < 10:
            return {'error': 'Insufficient data'}


        # ── CTE (GT 기준!) ──
        cte_list = []
        he_list = []
        for p in gt:
            cte, seg = self._compute_cte_for_pose(p.x, p.y)
            cte_list.append(cte)
            he = self._compute_heading_error(p.theta, seg)
            he_list.append(he)

        cte_arr = np.array(cte_list)
        he_arr = np.array(he_list)


        # ── EKF vs GT (상태 추정 정확도) ──
        ekf_pos_err = []
        if len(ekf) >= len(gt) // 2:
            # 시간 기반 매칭 (가장 가까운 시간!)
            ekf_idx = 0
            for gp in gt:
                while (ekf_idx < len(ekf) - 1 and
                       abs(ekf[ekf_idx + 1].t - gp.t) < abs(ekf[ekf_idx].t - gp.t)):
                    ekf_idx += 1
                if ekf_idx < len(ekf):
                    dx = ekf[ekf_idx].x - gp.x
                    dy = ekf[ekf_idx].y - gp.y
                    ekf_pos_err.append(math.sqrt(dx**2 + dy**2))


        # ── 속도 추종 ──
        v_actual = [p.vx for p in gt]
        omega_actual = [p.omega for p in gt]

        # ── Smoothness ──
        dt_avg = (gt[-1].t - gt[0].t) / len(gt) if len(gt) > 1 else 0.02
        omega_arr = np.array(omega_actual)
        v_arr = np.array(v_actual)
        omega_jerk = np.diff(omega_arr) / dt_avg if len(omega_arr) > 1 else np.array([0])
        v_jerk = np.diff(v_arr) / dt_avg if len(v_arr) > 1 else np.array([0])


        # ── Path Length Ratio ──
        robot_dist = 0.0
        for i in range(1, len(gt)):
            dx = gt[i].x - gt[i-1].x
            dy = gt[i].y - gt[i-1].y
            robot_dist += math.sqrt(dx**2 + dy**2)

        path_dist = 0.0
        for i in range(1, len(self.path)):
            dx = self.path[i][0] - self.path[i-1][0]
            dy = self.path[i][1] - self.path[i-1][1]
            path_dist += math.sqrt(dx**2 + dy**2)

        plr = robot_dist / path_dist if path_dist > 0 else 0.0


        # ── 리포트 ──
        report = {
            # CTE
            'cte_rmse': float(np.sqrt(np.mean(cte_arr**2))),
            'cte_max': float(np.max(np.abs(cte_arr))),
            'cte_mean': float(np.mean(np.abs(cte_arr))),
            'cte_std': float(np.std(cte_arr)),

            # Heading Error
            'he_rmse': float(np.sqrt(np.mean(he_arr**2))),
            'he_max': float(np.max(np.abs(he_arr))),

            # EKF accuracy
            'ekf_pos_rmse': (float(np.sqrt(np.mean(np.array(ekf_pos_err)**2)))
                             if ekf_pos_err else -1),

            # Velocity
            'v_mean': float(np.mean(np.abs(v_arr))),
            'v_std': float(np.std(v_arr)),

            # Smoothness
            'omega_jerk_rmse': float(np.sqrt(np.mean(omega_jerk**2))),
            'v_jerk_rmse': float(np.sqrt(np.mean(v_jerk**2))),

            # Completion
            'total_time': gt[-1].t,
            'path_length_ratio': plr,
            'path_length': path_dist,
            'robot_distance': robot_dist,

            # Raw arrays (for plotting)
            '_gt': gt, '_ekf': ekf, '_cmds': cmds,
            '_cte': cte_arr, '_he': he_arr,
            '_omega_jerk': omega_jerk,
        }

        return report



    # ═══════════════════════════════════════════════════════
    #  리포트 출력
    # ═══════════════════════════════════════════════════════
    def print_report(self, r: dict):
        """콘솔 리포트."""

        print()
        print('═' * 62)
        print('  PATH TRACKING EVALUATION REPORT')
        print('═' * 62)
        print()
        print('  ┌─ Cross-Track Error (CTE) ──────────────────────┐')
        print(f'  │  RMSE:      {r["cte_rmse"]*100:>8.2f} cm                  │')
        print(f'  │  Max:       {r["cte_max"]*100:>8.2f} cm                  │')
        print(f'  │  Mean:      {r["cte_mean"]*100:>8.2f} cm                  │')
        print(f'  │  Std Dev:   {r["cte_std"]*100:>8.2f} cm                  │')
        print('  └────────────────────────────────────────────────┘')
        print()
        print('  ┌─ Heading Error ────────────────────────────────┐')
        print(f'  │  RMSE:      {math.degrees(r["he_rmse"]):>8.2f} °                   │')
        print(f'  │  Max:       {math.degrees(r["he_max"]):>8.2f} °                   │')
        print('  └────────────────────────────────────────────────┘')
        print()
        print('  ┌─ EKF Accuracy ─────────────────────────────────┐')
        if r['ekf_pos_rmse'] >= 0:
            print(f'  │  Position RMSE: {r["ekf_pos_rmse"]*100:>6.2f} cm                │')
        else:
            print(f'  │  Position RMSE: N/A (insufficient data)      │')
        print('  └────────────────────────────────────────────────┘')
        print()
        print('  ┌─ Velocity & Smoothness ────────────────────────┐')
        print(f'  │  v mean:         {r["v_mean"]:>6.3f} m/s                │')
        print(f'  │  v std:          {r["v_std"]:>6.3f} m/s                │')
        print(f'  │  ω jerk RMSE:    {r["omega_jerk_rmse"]:>6.2f} rad/s²             │')
        print(f'  │  v jerk RMSE:    {r["v_jerk_rmse"]:>6.3f} m/s²               │')
        print('  └────────────────────────────────────────────────┘')
        print()
        print('  ┌─ Completion ───────────────────────────────────┐')
        print(f'  │  Total Time:     {r["total_time"]:>6.1f} s                  │')
        print(f'  │  Path Length:    {r["path_length"]:>6.2f} m                  │')
        print(f'  │  Robot Distance: {r["robot_distance"]:>6.2f} m                  │')
        print(f'  │  Length Ratio:   {r["path_length_ratio"]:>6.3f}                    │')
        print('  └────────────────────────────────────────────────┘')
        print()

        # 등급 판정!
        cte = r['cte_rmse'] * 100
        if cte < 2.0:
            grade = '★★★★★ EXCELLENT'
        elif cte < 4.0:
            grade = '★★★★☆ GOOD'
        elif cte < 8.0:
            grade = '★★★☆☆ ACCEPTABLE'
        elif cte < 15.0:
            grade = '★★☆☆☆ NEEDS TUNING'
        else:
            grade = '★☆☆☆☆ POOR'

        print(f'  GRADE: {grade}')
        print(f'  (Based on CTE RMSE: {cte:.1f}cm)')
        print()
        print('═' * 62)



    # ═══════════════════════════════════════════════════════
    #  시각화
    # ═══════════════════════════════════════════════════════
    def save_plots(self, prefix: str):
        """6패널 평가 시각화."""
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        r = self.compute_all()
        gt = r['_gt']
        cte = r['_cte']
        he = r['_he']

        fig, axes = plt.subplots(3, 2, figsize=(14, 14))
        fig.suptitle(
            f'Path Tracking Evaluation | '
            f'CTE RMSE={r["cte_rmse"]*100:.1f}cm, '
            f'Max={r["cte_max"]*100:.1f}cm, '
            f'Time={r["total_time"]:.1f}s',
            fontsize=12, fontweight='bold'
        )

        # ── Panel 1: XY 궤적 비교 ──
        ax = axes[0, 0]
        px = [p[0] for p in self.path]
        py = [p[1] for p in self.path]
        ax.plot(px, py, 'k--', linewidth=2, alpha=0.5, label='Path')
        ax.plot([p.x for p in gt], [p.y for p in gt],
                'b-', linewidth=1.2, label='Robot (GT)')
        if r['_ekf']:
            ekf = r['_ekf']
            ax.plot([p.x for p in ekf], [p.y for p in ekf],
                    'r-', linewidth=0.8, alpha=0.5, label='EKF')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Trajectory Comparison')
        ax.legend(fontsize=8)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)

        # ── Panel 2: CTE over time ──
        ax = axes[0, 1]
        t_arr = [p.t for p in gt[:len(cte)]]
        ax.plot(t_arr, cte * 100, 'b-', linewidth=0.8)
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax.fill_between(t_arr, -3, 3, alpha=0.05, color='green',
                         label='±3cm band')
        ax.axhline(y=r['cte_rmse']*100, color='r', linestyle=':',
                    alpha=0.7, label=f'RMSE={r["cte_rmse"]*100:.1f}cm')
        ax.axhline(y=-r['cte_rmse']*100, color='r', linestyle=':',
                    alpha=0.7)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('CTE (cm)')
        ax.set_title('Cross-Track Error')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # ── Panel 3: Heading Error ──
        ax = axes[1, 0]
        he_deg = np.degrees(he)
        ax.plot(t_arr[:len(he_deg)], he_deg, 'g-', linewidth=0.8)
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Heading Error (°)')
        ax.set_title(f'Heading Error (RMSE={math.degrees(r["he_rmse"]):.1f}°)')
        ax.grid(True, alpha=0.3)

        # ── Panel 4: 속도 프로파일 ──
        ax = axes[1, 1]
        t_gt = [p.t for p in gt]
        ax.plot(t_gt, [p.vx for p in gt], 'g-', linewidth=1,
                label='v actual')
        ax.plot(t_gt, [p.omega for p in gt], 'r-', linewidth=0.8,
                alpha=0.7, label='ω actual')
        if r['_cmds']:
            cmd_t = [c[0] for c in r['_cmds']]
            cmd_v = [c[1] for c in r['_cmds']]
            cmd_w = [c[2] for c in r['_cmds']]
            ax.plot(cmd_t, cmd_v, 'g--', linewidth=0.5, alpha=0.5,
                    label='v cmd')
            ax.plot(cmd_t, cmd_w, 'r--', linewidth=0.5, alpha=0.5,
                    label='ω cmd')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity')
        ax.set_title('Velocity Profile')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

        # ── Panel 5: Smoothness (ω jerk) ──
        ax = axes[2, 0]
        omega_jerk = r['_omega_jerk']
        t_jerk = t_gt[:len(omega_jerk)]
        ax.plot(t_jerk, omega_jerk, 'm-', linewidth=0.5, alpha=0.7)
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('ω Jerk (rad/s²)')
        ax.set_title(f'Angular Jerk (RMSE={r["omega_jerk_rmse"]:.2f})')
        ax.grid(True, alpha=0.3)

        # ── Panel 6: 성능 요약 바 차트 ──
        ax = axes[2, 1]
        labels = ['CTE\nRMSE', 'CTE\nMax', 'HE\nRMSE', 'v\nStd']
        values = [
            r['cte_rmse'] * 100,
            r['cte_max'] * 100,
            math.degrees(r['he_rmse']),
            r['v_std'] * 100,
        ]
        # 목표값
        targets = [3.0, 10.0, 5.0, 5.0]
        colors = ['green' if v <= t else 'orange' if v <= t*2 else 'red'
                  for v, t in zip(values, targets)]

        bars = ax.bar(labels, values, color=colors, alpha=0.7, edgecolor='black')
        # 목표선
        for i, t in enumerate(targets):
            ax.plot([i-0.4, i+0.4], [t, t], 'k--', linewidth=1.5)
        ax.set_ylabel('Value (cm or °)')
        ax.set_title('Performance Summary (dashed = target)')
        ax.grid(True, alpha=0.3, axis='y')

        plt.tight_layout()
        fig.savefig(f'{prefix}.png', dpi=150, bbox_inches='tight')
        plt.close(fig)






def main(args=None):
    rclpy.init(args=args)
    node = PathEvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()