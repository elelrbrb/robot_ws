#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# Day 69 Advanced — ROS 2 Bag Data-Driven EKF Evaluator
# ════════════════════════════════════════════════════════════════

import numpy as np
import math
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import List

# ROS 2 파싱용
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

def normalize_angle(angle: float) -> float:
    """각도를 [-π, +π]로 정규화."""
    return math.atan2(math.sin(angle), math.cos(angle))

def quaternion_to_yaw(q) -> float:
    """Quaternion에서 Yaw 각도 추출."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)



# ═══════════════════════════════════════════════════════════════
#  데이터 구조
# ═══════════════════════════════════════════════════════════════

@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

@dataclass
class TrajectoryData:
    name: str = ""
    timestamps: List[float] = field(default_factory=list)
    poses: List[Pose2D] = field(default_factory=list)
    covariances: List[np.ndarray] = field(default_factory=list)

@dataclass
class EvalMetrics:
    name: str = ""
    ate_rmse: float = 0.0
    ate_mean: float = 0.0
    ate_max: float = 0.0
    ate_median: float = 0.0
    yaw_rmse: float = 0.0
    yaw_max: float = 0.0
    rpe_rmse: float = 0.0
    rpe_yaw_rmse: float = 0.0
    nees_mean: float = 0.0
    consistency_95: float = 0.0
    convergence_time: float = 0.0
    final_pos_err: float = 0.0
    final_yaw_err: float = 0.0



# ═══════════════════════════════════════════════════════════════
#  ROS 2 Bag 파서 & 동기화 (핵심 추가 파트)
# ═══════════════════════════════════════════════════════════════

def parse_ros2_bag(bag_path: str) -> dict:
    """
    Bag 파일에서 3개의 Odometry 토픽을 파싱
    """
    print(f"[Parser] Opening Bag: {bag_path}")
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)

    gt_data = TrajectoryData(name="Ground Truth")
    odom_data = TrajectoryData(name="Raw Odom")
    ekf_data = TrajectoryData(name="Custom EKF")

    count = 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        
        if topic in ['/ground_truth/odom', '/odom', '/odometry/filtered']:
            msg = deserialize_message(data, Odometry)
            time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            pose = Pose2D(
                x=msg.pose.pose.position.x,
                y=msg.pose.pose.position.y,
                theta=quaternion_to_yaw(msg.pose.pose.orientation)
            )
            
            if topic == '/ground_truth/odom':
                gt_data.timestamps.append(time_sec)
                gt_data.poses.append(pose)
            elif topic == '/odom':
                odom_data.timestamps.append(time_sec)
                odom_data.poses.append(pose)
                odom_data.covariances.append(None)
            elif topic == '/odometry/filtered':
                ekf_data.timestamps.append(time_sec)
                ekf_data.poses.append(pose)
                
                # 6x6 Covariance에서 X, Y, Yaw(0, 1, 5) 추출하여 3x3 구성
                cov_flat = msg.pose.covariance
                P = np.array([
                    [cov_flat[0], cov_flat[1], cov_flat[5]],
                    [cov_flat[6], cov_flat[7], cov_flat[11]],
                    [cov_flat[30], cov_flat[31], cov_flat[35]]
                ])
                ekf_data.covariances.append(P)
        
        count += 1
        if count % 10000 == 0:
            print(f"[Parser] Parsed {count} messages...")

    print(f"[Parser] Done. GT: {len(gt_data.poses)}, Odom: {len(odom_data.poses)}, EKF: {len(ekf_data.poses)}")
    
    # 시간 정규화 (0초부터 시작하도록)
    t0 = min(gt_data.timestamps[0], odom_data.timestamps[0], ekf_data.timestamps[0])
    gt_data.timestamps = [t - t0 for t in gt_data.timestamps]
    odom_data.timestamps = [t - t0 for t in odom_data.timestamps]
    ekf_data.timestamps = [t - t0 for t in ekf_data.timestamps]

    return {'gt': gt_data, 'odom': odom_data, 'ekf': ekf_data}



def sync_trajectories(target: TrajectoryData, base: TrajectoryData, max_time_diff=0.05) -> TrajectoryData:
    """
    target 궤적을 base 궤적의 타임스탬프에 맞춰 동기화 (Nearest Neighbor).
    """
    synced = TrajectoryData(name=target.name)
    target_idx = 0
    n_target = len(target.timestamps)

    for base_t in base.timestamps:
        # base 시간과 가장 가까운 target 시간 찾기
        while target_idx < n_target - 1 and abs(target.timestamps[target_idx + 1] - base_t) < abs(target.timestamps[target_idx] - base_t):
            target_idx += 1
            
        if target_idx < n_target and abs(target.timestamps[target_idx] - base_t) <= max_time_diff:
            synced.timestamps.append(base_t)
            synced.poses.append(target.poses[target_idx])
            if len(target.covariances) > target_idx and target.covariances[target_idx] is not None:
                synced.covariances.append(target.covariances[target_idx])
            else:
                synced.covariances.append(None)
        else:
            # 매칭 실패 시 None을 넣거나 건너뜀 (여기서는 편의상 건너뜀)
            pass

    return synced



# ═══════════════════════════════════════════════════════════════
#  메트릭 계산 (기존 로직 유지)
# ═══════════════════════════════════════════════════════════════
def compute_ate(estimated: TrajectoryData, ground_truth: TrajectoryData) -> tuple:
    n = min(len(estimated.poses), len(ground_truth.poses))
    ate = np.zeros(n)
    yaw_err = np.zeros(n)
    for i in range(n):
        dx = estimated.poses[i].x - ground_truth.poses[i].x
        dy = estimated.poses[i].y - ground_truth.poses[i].y
        ate[i] = math.sqrt(dx**2 + dy**2)
        yaw_err[i] = abs(normalize_angle(estimated.poses[i].theta - ground_truth.poses[i].theta))
    return ate, yaw_err

def compute_rpe(estimated: TrajectoryData, ground_truth: TrajectoryData, delta_steps: int = 50) -> tuple:
    n = min(len(estimated.poses), len(ground_truth.poses))
    rpe_pos = []
    rpe_yaw = []
    for i in range(0, n - delta_steps, delta_steps):
        j = i + delta_steps
        dx_est = estimated.poses[j].x - estimated.poses[i].x
        dy_est = estimated.poses[j].y - estimated.poses[i].y
        dth_est = normalize_angle(estimated.poses[j].theta - estimated.poses[i].theta)

        dx_gt = ground_truth.poses[j].x - ground_truth.poses[i].x
        dy_gt = ground_truth.poses[j].y - ground_truth.poses[i].y
        dth_gt = normalize_angle(ground_truth.poses[j].theta - ground_truth.poses[i].theta)

        rpe_pos.append(math.sqrt((dx_est - dx_gt)**2 + (dy_est - dy_gt)**2))
        rpe_yaw.append(abs(normalize_angle(dth_est - dth_gt)))
    return np.array(rpe_pos), np.array(rpe_yaw)

def compute_nees(estimated: TrajectoryData, ground_truth: TrajectoryData) -> np.ndarray:
    n = min(len(estimated.poses), len(ground_truth.poses))
    nees = []
    for i in range(n):
        if i >= len(estimated.covariances): break
        P = estimated.covariances[i]
        if P is None or P.shape != (3, 3): continue
        e = np.array([
            estimated.poses[i].x - ground_truth.poses[i].x,
            estimated.poses[i].y - ground_truth.poses[i].y,
            normalize_angle(estimated.poses[i].theta - ground_truth.poses[i].theta)
        ])
        try:
            P_inv = np.linalg.inv(P)
            nees_val = float(e.T @ P_inv @ e)
            if nees_val < 100: nees.append(nees_val)
        except np.linalg.LinAlgError: pass
    return np.array(nees)

def compute_consistency(ate: np.ndarray, estimated: TrajectoryData) -> float:
    n = min(len(ate), len(estimated.covariances))
    count_within = 0
    for i in range(n):
        P = estimated.covariances[i]
        if P is None: continue
        sigma_pos = math.sqrt(P[0, 0] + P[1, 1])
        if ate[i] < 2.0 * sigma_pos: count_within += 1
    return (count_within / n * 100) if n > 0 else 0.0

def evaluate(estimated: TrajectoryData, ground_truth: TrajectoryData) -> EvalMetrics:
    metrics = EvalMetrics(name=estimated.name)
    ate, yaw_err = compute_ate(estimated, ground_truth)
    if len(ate) == 0: return metrics
    metrics.ate_rmse = float(np.sqrt(np.mean(ate**2)))
    metrics.ate_mean = float(np.mean(ate))
    metrics.ate_max = float(np.max(ate))
    metrics.ate_median = float(np.median(ate))
    metrics.yaw_rmse = float(np.sqrt(np.mean(yaw_err**2)))
    metrics.yaw_max = float(np.max(yaw_err))

    rpe_pos, rpe_yaw = compute_rpe(estimated, ground_truth, delta_steps=50)
    if len(rpe_pos) > 0:
        metrics.rpe_rmse = float(np.sqrt(np.mean(rpe_pos**2)))
        metrics.rpe_yaw_rmse = float(np.sqrt(np.mean(rpe_yaw**2)))

    if len(estimated.covariances) > 0 and estimated.covariances[0] is not None:
        nees = compute_nees(estimated, ground_truth)
        if len(nees) > 0: metrics.nees_mean = float(np.mean(nees))
        metrics.consistency_95 = compute_consistency(ate, estimated)

    return metrics



# ═══════════════════════════════════════════════════════════════
#  시각화 및 메인
# ═══════════════════════════════════════════════════════════════
# ═══════════════════════════════════════════════════════════════
#  시각화
# ═══════════════════════════════════════════════════════════════

def plot_comparison(ground_truth: TrajectoryData,
                    trajectories: List[TrajectoryData],
                    metrics_list: List[EvalMetrics],
                    test_name: str = "") -> plt.Figure:
    """6패널 비교 시각화."""

    fig, axes = plt.subplots(2, 3, figsize=(18, 11))
    fig.suptitle(f'EKF Performance Analysis — {test_name}',
                 fontsize=14, fontweight='bold')

    colors = ['red', 'blue', 'green', 'purple', 'orange']

    gt_x = [p.x for p in ground_truth.poses]
    gt_y = [p.y for p in ground_truth.poses]
    gt_t = ground_truth.timestamps
    gt_theta = [p.theta for p in ground_truth.poses]

    # ═══ Panel 1: XY 궤적 ═══
    ax = axes[0, 0]
    ax.plot(gt_x, gt_y, 'k-', linewidth=2.5, label='Ground Truth')
    for i, traj in enumerate(trajectories):
        tx = [p.x for p in traj.poses]
        ty = [p.y for p in traj.poses]
        ax.plot(tx, ty, color=colors[i], linewidth=1.2,
                alpha=0.8, label=traj.name)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('XY Trajectory')
    ax.legend(fontsize=8)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # ═══ Panel 2: Yaw 비교 ═══
    ax = axes[0, 1]
    ax.plot(gt_t[:len(gt_theta)],
            np.degrees(gt_theta), 'k-', linewidth=2, label='GT')
    for i, traj in enumerate(trajectories):
        tt = traj.timestamps[:len(traj.poses)]
        th = [math.degrees(p.theta) for p in traj.poses]
        ax.plot(tt[:len(th)], th, color=colors[i],
                alpha=0.7, label=traj.name)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Yaw (°)')
    ax.set_title('Heading Comparison')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ═══ Panel 3: ATE 시간 변화 ═══
    ax = axes[0, 2]
    for i, traj in enumerate(trajectories):
        ate, _ = compute_ate(traj, ground_truth)
        tt = traj.timestamps[:len(ate)]
        ax.plot(tt, ate, color=colors[i], alpha=0.8,
                label=f'{traj.name} (RMSE={metrics_list[i].ate_rmse:.3f}m)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position Error (m)')
    ax.set_title('ATE Over Time')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ═══ Panel 4: Yaw Error 시간 변화 ═══
    ax = axes[1, 0]
    for i, traj in enumerate(trajectories):
        _, yaw_err = compute_ate(traj, ground_truth)
        tt = traj.timestamps[:len(yaw_err)]
        ax.plot(tt, np.degrees(yaw_err), color=colors[i],
                alpha=0.7, label=traj.name)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Yaw Error (°)')
    ax.set_title('Yaw Error Over Time')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ═══ Panel 5: NEES (EKF만) ═══
    ax = axes[1, 1]
    for i, traj in enumerate(trajectories):
        if len(traj.covariances) > 0:
            nees = compute_nees(traj, ground_truth)
            if len(nees) > 0:
                tt = traj.timestamps[:len(nees)]
                ax.plot(tt, nees, color=colors[i],
                        alpha=0.6, label=traj.name)
    ax.axhline(y=3.0, color='gray', linestyle='--',
               label='Ideal (χ²₃=3)')
    ax.axhline(y=7.815, color='gray', linestyle=':',
               label='95% bound')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('NEES')
    ax.set_title('NEES (Consistency Check)')
    ax.legend(fontsize=8)
    ax.set_ylim(0, 15)
    ax.grid(True, alpha=0.3)

    # ═══ Panel 6: 바 차트 요약 ═══
    ax = axes[1, 2]
    x_pos = np.arange(len(metrics_list))
    width = 0.35
    bars1 = ax.bar(x_pos - width/2,
                   [m.ate_rmse * 100 for m in metrics_list],
                   width, label='ATE RMSE (cm)',
                   color=[colors[i] for i in range(len(metrics_list))],
                   alpha=0.7)
    bars2 = ax.bar(x_pos + width/2,
                   [math.degrees(m.yaw_rmse) for m in metrics_list],
                   width, label='Yaw RMSE (°)',
                   color=[colors[i] for i in range(len(metrics_list))],
                   alpha=0.4)

    ax.set_xlabel('Method')
    ax.set_ylabel('Error')
    ax.set_title('Summary Comparison')
    ax.set_xticks(x_pos)
    ax.set_xticklabels([m.name for m in metrics_list], fontsize=8)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, axis='y')

    # 바 위에 값 표시
    for bar in bars1:
        h = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2, h,
                f'{h:.1f}', ha='center', va='bottom', fontsize=7)

    plt.tight_layout()
    return fig





def main():

    bag_path = 'ekf_evaluation_bag1'
    
    # 1. 파싱
    raw_data = parse_ros2_bag(bag_path)
    
    gt = raw_data['gt']
    odom_raw = raw_data['odom']
    ekf_raw = raw_data['ekf']

    # 2. 동기화 (Ground Truth 시간에 맞춤)
    print("\n[Sync] Synchronizing trajectories...")
    odom_synced = sync_trajectories(odom_raw, gt)
    ekf_synced = sync_trajectories(ekf_raw, gt)

    # 3. 평가
    m_odom = evaluate(odom_synced, gt)
    m_ekf = evaluate(ekf_synced, gt)

    # 출력 (간략화)
    print('\n' + '═' * 50)
    print(f'  [ FINAL REPORT: {bag_path} ]')
    print('═' * 50)
    print(f"  Odom ATE RMSE: {m_odom.ate_rmse:.4f} m  | Yaw RMSE: {math.degrees(m_odom.yaw_rmse):.2f}°")
    print(f"  EKF  ATE RMSE: {m_ekf.ate_rmse:.4f} m  | Yaw RMSE: {math.degrees(m_ekf.yaw_rmse):.2f}°")
    if m_odom.ate_rmse > 0:
        imp = (1 - m_ekf.ate_rmse / m_odom.ate_rmse) * 100
        print(f"  >> EKF Improvement: +{imp:.1f}%")
    print('═' * 50)

    # 시각화 함수 호출 (기존 코드의 plot_comparison 재사용)
    fig = plot_comparison(gt, [odom_synced, ekf_synced], [m_odom, m_ekf], test_name="Bag Replay")
    fig.savefig('ultimate_ekf_portfolio.png', dpi=300)
    print("[Plot] Saved to ultimate_ekf_portfolio.png")


if __name__ == '__main__':
    main()