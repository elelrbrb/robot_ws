#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# test_ekf.py
# Day 65 — EKF 단위 테스트 + 시각화
#
# 구성:
#   1. 합성 로봇 시뮬레이터 (Ground Truth 생성)
#   2. 노이즈 센서 시뮬레이터 (오도메트리 + IMU 노이즈)
#   3. EKF 실행
#   4. 결과 비교: GT vs Raw Odom vs EKF
# ════════════════════════════════════════════════════════════════

import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

# ekf_core.py에서 import
from my_robot_control.ekf_core import DiffDriveEKF, normalize_angle


class RobotSimulator:
    """
    합성 로봇: Ground Truth 궤적 생성.

    완벽한 모션 모델 (노이즈 없음!)
    → 이것이 "정답"!
    """

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def move(self, v: float, omega: float, dt: float):
        """완벽한 RK2 모션."""
        theta_mid = self.theta + omega * dt / 2.0
        self.x += v * math.cos(theta_mid) * dt
        self.y += v * math.sin(theta_mid) * dt
        self.theta += omega * dt
        self.theta = normalize_angle(self.theta)

    def get_pose(self):
        return np.array([self.x, self.y, self.theta])


class NoisySensorSimulator:
    """
    노이즈 센서: Ground Truth에 노이즈를 추가하여 측정값 생성.
    """

    def __init__(self,
                 odom_noise_std=None,
                 imu_noise_std=0.05):
        """
        Args:
            odom_noise_std: [σ_x, σ_y, σ_θ] 오도메트리 노이즈
            imu_noise_std: σ_θ IMU Yaw 노이즈
        """
        if odom_noise_std is None:
            odom_noise_std = [0.03, 0.03, 0.05]
        self.odom_std = np.array(odom_noise_std)
        self.imu_std = imu_noise_std

        # 오도메트리 적분기 (노이즈 누적!)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0

    def get_odometry(self, v_true, omega_true, dt):
        """
        노이즈가 추가된 오도메트리 출력.

        노이즈 모델:
          v_measured = v_true + N(0, σ_v)
          ω_measured = ω_true + N(0, σ_ω)
          → 이 노이즈가 적분되어 오도메트리 누적 오차 발생!
        """
        # 속도에 노이즈 추가
        v_noisy = v_true + np.random.normal(0, 0.02)
        omega_noisy = omega_true + np.random.normal(0, 0.03)

        # 노이즈 속도로 적분 (Dead Reckoning)
        theta_mid = self.odom_theta + omega_noisy * dt / 2.0
        self.odom_x += v_noisy * math.cos(theta_mid) * dt
        self.odom_y += v_noisy * math.sin(theta_mid) * dt
        self.odom_theta += omega_noisy * dt
        self.odom_theta = normalize_angle(self.odom_theta)

        # 약간의 추가 위치 노이즈 (인코더 양자화 등)
        z_odom = np.array([
            self.odom_x + np.random.normal(0, 0.005),
            self.odom_y + np.random.normal(0, 0.005),
            self.odom_theta + np.random.normal(0, 0.005),
        ])
        z_odom[2] = normalize_angle(z_odom[2])

        return z_odom, v_noisy, omega_noisy

    def get_imu_yaw(self, true_theta):
        """
        노이즈가 추가된 IMU Yaw.

        BNO055: 절대 자세 (드리프트 작음, 자기장 노이즈!)
        """
        noise = np.random.normal(0, self.imu_std)
        return normalize_angle(true_theta + noise)


def plot_covariance_ellipse(ax, mean, cov, n_std=2.0,
                             color='blue', alpha=0.3):
    """
    2D 공분산 타원 그리기.

    타원의 축: 공분산의 고유벡터 방향
    타원의 크기: 고유값의 제곱근 × n_std
    """
    if cov[0, 0] <= 0 or cov[1, 1] <= 0:
        return

    eigenvalues, eigenvectors = np.linalg.eigh(cov[:2, :2])
    # eigh: 대칭 행렬 전용 → 항상 실수 고유값!

    # 고유값이 음수면 스킵 (수치 오류)
    eigenvalues = np.maximum(eigenvalues, 0)

    # 타원 각도
    angle = math.degrees(math.atan2(eigenvectors[1, 0],
                                      eigenvectors[0, 0]))

    # 타원 크기 (n_std × σ)
    width = 2 * n_std * math.sqrt(eigenvalues[0])
    height = 2 * n_std * math.sqrt(eigenvalues[1])

    ellipse = Ellipse(xy=(mean[0], mean[1]),
                       width=width, height=height,
                       angle=angle,
                       facecolor=color, alpha=alpha,
                       edgecolor=color, linewidth=1)
    ax.add_patch(ellipse)


def run_simulation():
    """
    메인 시뮬레이션 실행.

    시나리오: 사각형 경로 (2m × 2m)
      ① 직진 2m
      ② 좌회전 90°
      ③ 직진 2m
      ④ 좌회전 90°
      ⑤ 직진 2m
      ⑥ 좌회전 90°
      ⑦ 직진 2m
      ⑧ 좌회전 90° → 시작점 복귀!
    """

    dt = 0.02  # 50Hz (오도메트리 주기)
    v_cmd = 0.3  # 0.3 m/s
    omega_cmd = 0.5  # 0.5 rad/s (회전 시)

    # 경로 정의 (명령 시퀀스)
    # (v, ω, duration)
    path_segments = []
    for _ in range(4):
        # 직진 2m → 시간 = 2.0/0.3 ≈ 6.67s
        path_segments.append((v_cmd, 0.0, 2.0 / v_cmd))
        # 좌회전 90° → 시간 = (π/2)/0.5 ≈ 3.14s
        path_segments.append((0.0, omega_cmd, (math.pi / 2) / omega_cmd))

    # 시뮬레이터 초기화
    robot = RobotSimulator()
    sensors = NoisySensorSimulator(
        odom_noise_std=[0.03, 0.03, 0.05],
        imu_noise_std=0.05
    )
    ekf = DiffDriveEKF(
        motion_noise_alpha=[0.05, 0.01, 0.01, 0.05],
        odom_noise=np.array([0.001, 0.001, 0.005]),
        imu_noise=0.01,
    )

    # 기록 저장소
    history = {
        'gt_x': [], 'gt_y': [], 'gt_theta': [],
        'odom_x': [], 'odom_y': [], 'odom_theta': [],
        'ekf_x': [], 'ekf_y': [], 'ekf_theta': [],
        'sigma_x': [], 'sigma_y': [], 'sigma_theta': [],
        'time': [],
        'ekf_P': [],  # 공분산 스냅샷
    }

    t = 0.0
    imu_interval = 0.01  # 100Hz IMU
    imu_timer = 0.0

    # ═══════════════════════════════════════
    # 시뮬레이션 루프
    # ═══════════════════════════════════════
    for v, omega, duration in path_segments:
        steps = int(duration / dt)
        for _ in range(steps):
            t += dt

            # ──── Ground Truth 업데이트 ────
            robot.move(v, omega, dt)
            gt = robot.get_pose()

            # ──── 센서 측정 ────
            z_odom, v_meas, omega_meas = sensors.get_odometry(
                v, omega, dt
            )
            theta_imu = sensors.get_imu_yaw(gt[2])

            # ──── EKF Predict ────
            ekf.predict(v_meas, omega_meas, dt)

            # ──── EKF Update: 오도메트리 (매 스텝) ────
            ekf.update_odometry(z_odom)

            # ──── EKF Update: IMU (100Hz) ────
            imu_timer += dt
            if imu_timer >= imu_interval:
                ekf.update_imu(theta_imu)
                imu_timer = 0.0

            # ──── 기록 ────
            state = ekf.get_state()
            unc = ekf.get_uncertainty()

            history['gt_x'].append(gt[0])
            history['gt_y'].append(gt[1])
            history['gt_theta'].append(gt[2])
            history['odom_x'].append(z_odom[0])
            history['odom_y'].append(z_odom[1])
            history['odom_theta'].append(z_odom[2])
            history['ekf_x'].append(state[0])
            history['ekf_y'].append(state[1])
            history['ekf_theta'].append(state[2])
            history['sigma_x'].append(unc['sigma_x'])
            history['sigma_y'].append(unc['sigma_y'])
            history['sigma_theta'].append(unc['sigma_theta_deg'])
            history['time'].append(t)
            history['ekf_P'].append(ekf.get_covariance().copy())

    # NumPy로 변환
    for key in history:
        if key != 'ekf_P':
            history[key] = np.array(history[key])

    return history, ekf


def plot_results(h):
    """시뮬레이션 결과 4패널 시각화."""

    fig, axes = plt.subplots(2, 2, figsize=(14, 12))

    # ═══ Panel 1: XY 궤적 비교 ═══
    ax = axes[0, 0]
    ax.plot(h['gt_x'], h['gt_y'], 'k-', linewidth=2,
            label='Ground Truth', zorder=3)
    ax.plot(h['odom_x'], h['odom_y'], 'r-', linewidth=1,
            alpha=0.6, label='Raw Odometry')
    ax.plot(h['ekf_x'], h['ekf_y'], 'b-', linewidth=1.5,
            label='EKF Fused')

    # 불확실성 타원 (10스텝마다)
    step = max(1, len(h['ekf_x']) // 20)
    for i in range(0, len(h['ekf_x']), step):
        plot_covariance_ellipse(
            ax,
            [h['ekf_x'][i], h['ekf_y'][i]],
            h['ekf_P'][i],
            n_std=2.0,
            color='blue', alpha=0.15
        )

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('XY Trajectory: GT vs Odometry vs EKF')
    ax.legend(loc='upper left')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # ═══ Panel 2: Yaw 비교 ═══
    ax = axes[0, 1]
    ax.plot(h['time'], np.degrees(h['gt_theta']), 'k-',
            linewidth=2, label='GT')
    ax.plot(h['time'], np.degrees(h['odom_theta']), 'r-',
            alpha=0.6, label='Odometry')
    ax.plot(h['time'], np.degrees(h['ekf_theta']), 'b-',
            linewidth=1.5, label='EKF')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Yaw (°)')
    ax.set_title('Heading: GT vs Odometry vs EKF')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # ═══ Panel 3: 위치 오차 (RMSE) ═══
    ax = axes[1, 0]
    odom_err = np.sqrt(
        (h['odom_x'] - h['gt_x'])**2 +
        (h['odom_y'] - h['gt_y'])**2
    )
    ekf_err = np.sqrt(
        (h['ekf_x'] - h['gt_x'])**2 +
        (h['ekf_y'] - h['gt_y'])**2
    )
    ax.plot(h['time'], odom_err, 'r-', alpha=0.7,
            label=f'Odometry (RMSE={np.sqrt(np.mean(odom_err**2)):.4f}m)')
    ax.plot(h['time'], ekf_err, 'b-',
            label=f'EKF (RMSE={np.sqrt(np.mean(ekf_err**2)):.4f}m)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position Error (m)')
    ax.set_title('Position Error Over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # ═══ Panel 4: 불확실성 (σ) ═══
    ax = axes[1, 1]
    ax.plot(h['time'], h['sigma_x'], 'r-', label='σ_x (m)')
    ax.plot(h['time'], h['sigma_y'], 'g-', label='σ_y (m)')
    ax.plot(h['time'], h['sigma_theta'], 'b-', label='σ_θ (°)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Standard Deviation')
    ax.set_title('EKF Uncertainty (2σ = 95% confidence)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def print_final_report(h):
    """최종 성능 보고."""

    gt_final = np.array([h['gt_x'][-1], h['gt_y'][-1]])
    odom_final = np.array([h['odom_x'][-1], h['odom_y'][-1]])
    ekf_final = np.array([h['ekf_x'][-1], h['ekf_y'][-1]])

    odom_err_pos = np.linalg.norm(odom_final - gt_final)
    ekf_err_pos = np.linalg.norm(ekf_final - gt_final)

    odom_err_yaw = abs(normalize_angle(
        h['odom_theta'][-1] - h['gt_theta'][-1]
    ))
    ekf_err_yaw = abs(normalize_angle(
        h['ekf_theta'][-1] - h['gt_theta'][-1]
    ))

    # 전체 RMSE
    odom_rmse = np.sqrt(np.mean(
        (h['odom_x'] - h['gt_x'])**2 +
        (h['odom_y'] - h['gt_y'])**2
    ))
    ekf_rmse = np.sqrt(np.mean(
        (h['ekf_x'] - h['gt_x'])**2 +
        (h['ekf_y'] - h['gt_y'])**2
    ))

    print('=' * 60)
    print('         EKF PERFORMANCE REPORT')
    print('=' * 60)
    print(f'  Path: Square 2m × 2m (total ~16m)')
    print(f'  Duration: {h["time"][-1]:.1f}s')
    print(f'  Steps: {len(h["time"])}')
    print()
    print(f'  --- Final Position Error ---')
    print(f'  Odometry:  {odom_err_pos:.4f} m')
    print(f'  EKF:       {ekf_err_pos:.4f} m')
    print(f'  Improvement: {(1 - ekf_err_pos/max(odom_err_pos, 1e-6))*100:.1f}%')
    print()
    print(f'  --- Final Yaw Error ---')
    print(f'  Odometry:  {math.degrees(odom_err_yaw):.2f}°')
    print(f'  EKF:       {math.degrees(ekf_err_yaw):.2f}°')
    print()
    print(f'  --- Overall RMSE ---')
    print(f'  Odometry:  {odom_rmse:.4f} m')
    print(f'  EKF:       {ekf_rmse:.4f} m')
    print(f'  Improvement: {(1 - ekf_rmse/max(odom_rmse, 1e-6))*100:.1f}%')
    print()
    print(f'  --- Final Uncertainty ---')
    print(f'  σ_x:  {h["sigma_x"][-1]:.4f} m')
    print(f'  σ_y:  {h["sigma_y"][-1]:.4f} m')
    print(f'  σ_θ:  {h["sigma_theta"][-1]:.2f}°')
    print('=' * 60)






def check_ekf_health(ekf):
    """EKF 건강 상태 확인."""
    P = ekf.get_covariance()
    state = ekf.get_state()

    issues = []

    # 1. 상태 범위 확인
    if abs(state[0]) > 100 or abs(state[1]) > 100:
        issues.append(f'State out of range: x={state[0]:.1f}, y={state[1]:.1f}')

    if abs(state[2]) > math.pi + 0.01:
        issues.append(f'θ not normalized: {state[2]:.4f}')

    # 2. P 양의 정부호 확인
    eigvals = np.linalg.eigvalsh(P)
    if np.any(eigvals < -1e-8):
        issues.append(f'P not positive definite! min eigval: {min(eigvals):.2e}')

    # 3. P 대칭 확인
    asym = np.max(np.abs(P - P.T))
    if asym > 1e-6:
        issues.append(f'P not symmetric! max asymmetry: {asym:.2e}')

    # 4. P 대각 원소가 합리적 범위?
    for i, label in enumerate(['x', 'y', 'θ']):
        if P[i, i] < 0:
            issues.append(f'P[{label}] negative: {P[i,i]:.2e}')
        if P[i, i] > 100:
            issues.append(f'P[{label}] very large: {P[i,i]:.2e}')

    # 5. NaN/Inf 확인
    if np.any(np.isnan(state)) or np.any(np.isinf(state)):
        issues.append('State contains NaN/Inf!')
    if np.any(np.isnan(P)) or np.any(np.isinf(P)):
        issues.append('P contains NaN/Inf!')

    return issues








# ═══════════════════════════════════════
# 메인 실행
# ═══════════════════════════════════════
if __name__ == '__main__':
    np.random.seed(42)  # 재현성!

    print('Running EKF simulation...')
    history, ekf = run_simulation()


    # EKF Health Check 추가
    issues = check_ekf_health(ekf)   # ekf 객체를 run_simulation 안에서 global로 반환하거나, run_simulation이 ekf를 함께 반환하도록 수정 필요
    if issues:
        print("EKF Health Issues Detected:")
        for msg in issues:
            print(" -", msg)
    else:
        print("EKF Health Check Passed ✅")


    print_final_report(history)

    fig = plot_results(history)
    plt.savefig('ekf_test_result.png', dpi=150, bbox_inches='tight')
    plt.show()
    print('Plot saved: ekf_test_result.png')
