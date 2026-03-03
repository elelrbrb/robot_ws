#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# ekf_evaluator.py
# Day 69 — EKF 성능 정량 평가 + 시각화 + 보고서 생성
#
# 3가지 비교: Raw Odom vs 직접 EKF vs robot_localization
# 3가지 경로: 직선, 사각형, 8자
# 7가지 메트릭: ATE_RMSE, ATE_Max, ATE_Mean, RPE_RMSE,
#               Yaw_RMSE, NEES_Mean, Consistency_95%
# ════════════════════════════════════════════════════════════════

import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from dataclasses import dataclass, field
from typing import List, Dict, Optional


def normalize_angle(angle: float) -> float:
    """각도를 [-π, +π]로 정규화."""
    return math.atan2(math.sin(angle), math.cos(angle))


# ═══════════════════════════════════════════════════════════════
#  데이터 구조
# ═══════════════════════════════════════════════════════════════

@dataclass
class Pose2D:
    """2D 포즈: x, y, theta."""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


@dataclass
class TrajectoryData:
    """궤적 데이터: 타임스탬프 + 포즈 목록."""
    name: str = ""
    timestamps: List[float] = field(default_factory=list)
    poses: List[Pose2D] = field(default_factory=list)
    covariances: List[np.ndarray] = field(default_factory=list)
    # covariances: 3×3 행렬 리스트 (EKF 출력용)


@dataclass
class EvalMetrics:
    """평가 결과 메트릭."""
    name: str = ""

    # ATE (Absolute Trajectory Error)
    ate_rmse: float = 0.0       # [m]
    ate_mean: float = 0.0       # [m]
    ate_max: float = 0.0        # [m]
    ate_median: float = 0.0     # [m]

    # Yaw Error
    yaw_rmse: float = 0.0      # [rad]
    yaw_max: float = 0.0       # [rad]

    # RPE (Relative Pose Error, 1초 간격)
    rpe_rmse: float = 0.0      # [m]
    rpe_yaw_rmse: float = 0.0  # [rad]

    # Consistency (일관성)
    nees_mean: float = 0.0     # NEES 평균
    consistency_95: float = 0.0 # ATE < 2σ인 비율 [%]

    # 수렴 시간
    convergence_time: float = 0.0  # [s]

    # 최종 오차 (복귀 정확도)
    final_pos_err: float = 0.0  # [m]
    final_yaw_err: float = 0.0  # [rad]


# ═══════════════════════════════════════════════════════════════
#  메트릭 계산
# ═══════════════════════════════════════════════════════════════

def compute_ate(estimated: TrajectoryData,
                ground_truth: TrajectoryData) -> tuple:
    """
    ATE 계산.

    Returns:
        (ate_array, yaw_err_array): 각 시점의 위치/Yaw 오차
    """
    n = min(len(estimated.poses), len(ground_truth.poses))
    ate = np.zeros(n)
    yaw_err = np.zeros(n)

    for i in range(n):
        dx = estimated.poses[i].x - ground_truth.poses[i].x
        dy = estimated.poses[i].y - ground_truth.poses[i].y
        ate[i] = math.sqrt(dx**2 + dy**2)

        dtheta = normalize_angle(
            estimated.poses[i].theta - ground_truth.poses[i].theta
        )
        yaw_err[i] = abs(dtheta)

    return ate, yaw_err


def compute_rpe(estimated: TrajectoryData,
                ground_truth: TrajectoryData,
                delta_steps: int = 50) -> tuple:
    """
    RPE 계산 (delta_steps 간격).

    delta_steps=50 at 50Hz → 1초 간격!

    Returns:
        (rpe_pos_array, rpe_yaw_array)
    """
    n = min(len(estimated.poses), len(ground_truth.poses))
    rpe_pos = []
    rpe_yaw = []

    for i in range(0, n - delta_steps, delta_steps):
        j = i + delta_steps

        # 추정 상대 이동
        dx_est = estimated.poses[j].x - estimated.poses[i].x
        dy_est = estimated.poses[j].y - estimated.poses[i].y
        dth_est = normalize_angle(
            estimated.poses[j].theta - estimated.poses[i].theta
        )

        # GT 상대 이동
        dx_gt = ground_truth.poses[j].x - ground_truth.poses[i].x
        dy_gt = ground_truth.poses[j].y - ground_truth.poses[i].y
        dth_gt = normalize_angle(
            ground_truth.poses[j].theta - ground_truth.poses[i].theta
        )

        # 오차
        err_x = dx_est - dx_gt
        err_y = dy_est - dy_gt
        rpe_pos.append(math.sqrt(err_x**2 + err_y**2))
        rpe_yaw.append(abs(normalize_angle(dth_est - dth_gt)))

    return np.array(rpe_pos), np.array(rpe_yaw)


def compute_nees(estimated: TrajectoryData,
                 ground_truth: TrajectoryData) -> np.ndarray:
    """
    NEES (Normalized Estimation Error Squared) 계산.

    NEES_k = e_kᵀ × P_k⁻¹ × e_k

    잘 튜닝된 EKF: NEES ~ χ²(3), 평균 = 3.0
    """
    n = min(len(estimated.poses), len(ground_truth.poses))
    nees = []

    for i in range(n):
        if i >= len(estimated.covariances):
            break

        P = estimated.covariances[i]
        if P is None or P.shape != (3, 3):
            continue

        e = np.array([
            estimated.poses[i].x - ground_truth.poses[i].x,
            estimated.poses[i].y - ground_truth.poses[i].y,
            normalize_angle(
                estimated.poses[i].theta - ground_truth.poses[i].theta
            )
        ])

        try:
            P_inv = np.linalg.inv(P)
            nees_val = float(e.T @ P_inv @ e)
            if nees_val < 100:
                # 100 이상은 이상치
                nees.append(nees_val)
        except np.linalg.LinAlgError:
            pass

    return np.array(nees)


def compute_consistency(ate: np.ndarray,
                        estimated: TrajectoryData) -> float:
    """
    일관성: ATE < 2σ인 비율.

    2σ = 95% 신뢰구간 → 이 비율이 ~95%여야!
    """
    n = min(len(ate), len(estimated.covariances))
    count_within = 0

    for i in range(n):
        P = estimated.covariances[i]
        if P is None:
            continue

        sigma_pos = math.sqrt(P[0, 0] + P[1, 1])
        # 위치 불확실성 = √(σ²_x + σ²_y)

        two_sigma = 2.0 * sigma_pos
        if ate[i] < two_sigma:
            count_within += 1

    return (count_within / n * 100) if n > 0 else 0.0


def compute_convergence_time(estimated: TrajectoryData,
                             threshold: float = 0.05) -> float:
    """
    수렴 시간: P 대각 합이 threshold 이하로 떨어지는 시간.
    """
    for i, P in enumerate(estimated.covariances):
        if P is None:
            continue
        trace = P[0, 0] + P[1, 1] + P[2, 2]
        if trace < threshold:
            if i < len(estimated.timestamps):
                return estimated.timestamps[i]
    return float('inf')


def evaluate(estimated: TrajectoryData,
             ground_truth: TrajectoryData,
             delta_steps: int = 50) -> EvalMetrics:
    """전체 메트릭 계산."""
    metrics = EvalMetrics(name=estimated.name)

    # ATE
    ate, yaw_err = compute_ate(estimated, ground_truth)
    metrics.ate_rmse = float(np.sqrt(np.mean(ate**2)))
    metrics.ate_mean = float(np.mean(ate))
    metrics.ate_max = float(np.max(ate))
    metrics.ate_median = float(np.median(ate))

    # Yaw
    metrics.yaw_rmse = float(np.sqrt(np.mean(yaw_err**2)))
    metrics.yaw_max = float(np.max(yaw_err))

    # RPE
    rpe_pos, rpe_yaw = compute_rpe(
        estimated, ground_truth, delta_steps
    )
    if len(rpe_pos) > 0:
        metrics.rpe_rmse = float(np.sqrt(np.mean(rpe_pos**2)))
        metrics.rpe_yaw_rmse = float(np.sqrt(np.mean(rpe_yaw**2)))

    # NEES
    if len(estimated.covariances) > 0:
        nees = compute_nees(estimated, ground_truth)
        if len(nees) > 0:
            metrics.nees_mean = float(np.mean(nees))

        # Consistency
        metrics.consistency_95 = compute_consistency(
            ate, estimated
        )

        # Convergence
        metrics.convergence_time = compute_convergence_time(
            estimated
        )

    # Final Error
    if len(estimated.poses) > 0 and len(ground_truth.poses) > 0:
        last_est = estimated.poses[-1]
        last_gt = ground_truth.poses[-1]
        metrics.final_pos_err = math.sqrt(
            (last_est.x - last_gt.x)**2 +
            (last_est.y - last_gt.y)**2
        )
        metrics.final_yaw_err = abs(normalize_angle(
            last_est.theta - last_gt.theta
        ))

    return metrics


# ═══════════════════════════════════════════════════════════════
#  보고서 출력
# ═══════════════════════════════════════════════════════════════

def print_comparison_report(metrics_list: List[EvalMetrics],
                            test_name: str = ""):
    """비교 보고서 출력."""

    print('\n' + '═' * 72)
    print(f'  EKF PERFORMANCE REPORT — {test_name}')
    print('═' * 72)

    # 헤더
    names = [m.name for m in metrics_list]
    col_w = max(14, max(len(n) for n in names) + 2)

    header = f'  {"Metric":<22}'
    for name in names:
        header += f'{name:>{col_w}}'
    print(header)
    print('  ' + '─' * (22 + col_w * len(names)))

    # 행 출력 헬퍼
    def row(label, values, fmt='.4f', unit=''):
        line = f'  {label:<22}'
        for v in values:
            line += f'{v:{col_w}{fmt}}'
        if unit:
            line += f'  {unit}'
        print(line)

    row('ATE RMSE',
        [m.ate_rmse for m in metrics_list], '.4f', 'm')
    row('ATE Mean',
        [m.ate_mean for m in metrics_list], '.4f', 'm')
    row('ATE Max',
        [m.ate_max for m in metrics_list], '.4f', 'm')
    row('ATE Median',
        [m.ate_median for m in metrics_list], '.4f', 'm')

    print('  ' + '─' * (22 + col_w * len(names)))

    row('Yaw RMSE',
        [math.degrees(m.yaw_rmse) for m in metrics_list], '.2f', '°')
    row('Yaw Max',
        [math.degrees(m.yaw_max) for m in metrics_list], '.2f', '°')

    print('  ' + '─' * (22 + col_w * len(names)))

    row('RPE RMSE (1s)',
        [m.rpe_rmse for m in metrics_list], '.4f', 'm')
    row('RPE Yaw RMSE (1s)',
        [math.degrees(m.rpe_yaw_rmse) for m in metrics_list], '.2f', '°')

    print('  ' + '─' * (22 + col_w * len(names)))

    row('NEES Mean (ideal=3)',
        [m.nees_mean for m in metrics_list], '.2f', '')
    row('Consistency (95%CI)',
        [m.consistency_95 for m in metrics_list], '.1f', '%')

    print('  ' + '─' * (22 + col_w * len(names)))

    row('Final Pos Error',
        [m.final_pos_err for m in metrics_list], '.4f', 'm')
    row('Final Yaw Error',
        [math.degrees(m.final_yaw_err)
         for m in metrics_list], '.2f', '°')

    # 개선율 (첫 번째 대비)
    if len(metrics_list) >= 2:
        base = metrics_list[0]
        print('\n  [Improvement vs ' + base.name + ']')
        for m in metrics_list[1:]:
            if base.ate_rmse > 0:
                imp = (1 - m.ate_rmse / base.ate_rmse) * 100
                print(f'    {m.name}: ATE RMSE {imp:+.1f}%')
            if base.yaw_rmse > 0:
                imp_y = (1 - m.yaw_rmse / base.yaw_rmse) * 100
                print(f'    {m.name}: Yaw RMSE {imp_y:+.1f}%')

    print('═' * 72)


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


# ═══════════════════════════════════════════════════════════════
#  합성 시뮬레이션 (테스트용)
# ═══════════════════════════════════════════════════════════════

def simulate_path(path_name: str, seed: int = 42) -> dict:
    """
    합성 경로 시뮬레이션.
    GT + Raw Odom + EKF 출력을 생성.

    path_name: 'straight', 'square', 'figure8'
    """
    from my_robot_control.ekf_core import DiffDriveEKF

    np.random.seed(seed)
    dt = 0.02  # 50Hz

    # 경로 정의
    if path_name == 'straight':
        segments = [(0.3, 0.0, 10.0)]
        # 3m 직선
    elif path_name == 'square':
        segments = []
        for _ in range(4):
            segments.append((0.3, 0.0, 2.0/0.3))
            segments.append((0.0, 0.5, (math.pi/2)/0.5))
    elif path_name == 'figure8':
        segments = []
        # 왼쪽 원 (반시계)
        segments.append((0.3, 0.3, (2*math.pi)/0.3))
        # 오른쪽 원 (시계)
        segments.append((0.3, -0.3, (2*math.pi)/0.3))
    else:
        segments = [(0.3, 0.0, 5.0)]

    # 시뮬레이터 초기화
    ekf = DiffDriveEKF(
        motion_noise_alpha=[0.05, 0.01, 0.01, 0.05],
        odom_noise=np.array([0.001, 0.001, 0.005]),
        imu_noise=0.01,
    )

    gt = TrajectoryData(name='Ground Truth')
    odom = TrajectoryData(name='Raw Odom')
    ekf_traj = TrajectoryData(name='Custom EKF')

    # 상태
    gt_x, gt_y, gt_th = 0.0, 0.0, 0.0
    od_x, od_y, od_th = 0.0, 0.0, 0.0
    t = 0.0

    for v_cmd, omega_cmd, duration in segments:
        steps = int(duration / dt)
        for _ in range(steps):
            t += dt

            # GT
            tm = gt_th + omega_cmd * dt / 2
            gt_x += v_cmd * math.cos(tm) * dt
            gt_y += v_cmd * math.sin(tm) * dt
            gt_th = normalize_angle(gt_th + omega_cmd * dt)

            # Noisy odom
            v_n = v_cmd + np.random.normal(0, 0.02)
            w_n = omega_cmd + np.random.normal(0, 0.03)
            tm_o = od_th + w_n * dt / 2
            od_x += v_n * math.cos(tm_o) * dt
            od_y += v_n * math.sin(tm_o) * dt
            od_th = normalize_angle(od_th + w_n * dt)

            z_odom = np.array([
                od_x + np.random.normal(0, 0.005),
                od_y + np.random.normal(0, 0.005),
                normalize_angle(od_th + np.random.normal(0, 0.005))
            ])

            theta_imu = normalize_angle(
                gt_th + np.random.normal(0, 0.05)
            )

            # EKF
            ekf.predict(v_n, w_n, dt)
            ekf.update_odometry(z_odom)
            if int(t / 0.01) % 2 == 0:
                ekf.update_imu(theta_imu)

            state = ekf.get_state()
            P = ekf.get_covariance()

            # 기록
            gt.timestamps.append(t)
            gt.poses.append(Pose2D(gt_x, gt_y, gt_th))

            odom.timestamps.append(t)
            odom.poses.append(Pose2D(od_x, od_y, od_th))
            odom.covariances.append(None)

            ekf_traj.timestamps.append(t)
            ekf_traj.poses.append(Pose2D(state[0], state[1], state[2]))
            ekf_traj.covariances.append(P.copy())

    return {
        'gt': gt,
        'odom': odom,
        'ekf': ekf_traj,
        'path_name': path_name,
    }


# ═══════════════════════════════════════════════════════════════
#  메인
# ═══════════════════════════════════════════════════════════════

def main():
    """3가지 경로에서 성능 평가 실행."""

    all_results = {}

    for path in ['straight', 'square', 'figure8']:
        print(f'\n▶ Simulating: {path}...')
        sim = simulate_path(path)

        # 메트릭 계산
        m_odom = evaluate(sim['odom'], sim['gt'])
        m_odom.name = 'Raw Odom'

        m_ekf = evaluate(sim['ekf'], sim['gt'])
        m_ekf.name = 'Custom EKF'

        # 보고서 출력
        print_comparison_report([m_odom, m_ekf], test_name=path)

        # 시각화
        fig = plot_comparison(
            sim['gt'],
            [sim['odom'], sim['ekf']],
            [m_odom, m_ekf],
            test_name=path
        )
        filename = f'ekf_eval_{path}.png'
        fig.savefig(filename, dpi=150, bbox_inches='tight')
        print(f'  Saved: {filename}')
        plt.close(fig)

        all_results[path] = {
            'odom': m_odom,
            'ekf': m_ekf,
        }

    # 최종 요약
    print('\n\n' + '█' * 72)
    print('  FINAL SUMMARY — ALL PATHS')
    print('█' * 72)

    print(f'\n  {"Path":<12} {"Odom RMSE":>12} {"EKF RMSE":>12} '
          f'{"Improvement":>14} {"EKF Yaw":>10}')
    print('  ' + '─' * 62)

    for path in ['straight', 'square', 'figure8']:
        r = all_results[path]
        imp = ((1 - r['ekf'].ate_rmse / max(r['odom'].ate_rmse, 1e-9))
               * 100)
        print(f'  {path:<12} {r["odom"].ate_rmse:>10.4f}m '
              f'{r["ekf"].ate_rmse:>10.4f}m '
              f'{imp:>+12.1f}% '
              f'{math.degrees(r["ekf"].yaw_rmse):>8.2f}°')

    print('█' * 72)


if __name__ == '__main__':
    main()
