#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# test_pure_pursuit.py
# Day 76 — Pure Pursuit 시뮬레이션 검증
# ════════════════════════════════════════════════════════════════

import numpy as np
import math
import matplotlib.pyplot as plt
from pure_pursuit import PurePursuitController, Pose2D
from path_generator import (generate_straight, generate_circle,
                            generate_square, generate_slalom,
                            generate_figure8)


class DiffDriveSimulator:
    """
    차동 구동 로봇 시뮬레이터.

    ω 명령 → 1차 지연 → θ 변화
    v 명령 → 1차 지연 → 위치 변화
    """

    def __init__(self, tau_v=0.05, tau_omega=0.05,
                 noise_xy=0.001, noise_theta=0.002):
        self.tau_v = tau_v
        self.tau_omega = tau_omega
        self.noise_xy = noise_xy
        self.noise_theta = noise_theta

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_actual = 0.0
        self.omega_actual = 0.0

    def step(self, v_cmd: float, omega_cmd: float,
             dt: float) -> Pose2D:
        """1스텝 시뮬레이션."""
        # 1차 지연
        self.v_actual += (v_cmd - self.v_actual) / self.tau_v * dt
        self.omega_actual += (omega_cmd - self.omega_actual) / self.tau_omega * dt

        # 운동학 (RK2)
        theta_mid = self.theta + self.omega_actual * dt / 2
        self.x += self.v_actual * math.cos(theta_mid) * dt
        self.y += self.v_actual * math.sin(theta_mid) * dt
        self.theta += self.omega_actual * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # 노이즈
        x_meas = self.x + np.random.normal(0, self.noise_xy)
        y_meas = self.y + np.random.normal(0, self.noise_xy)
        th_meas = self.theta + np.random.normal(0, self.noise_theta)

        return Pose2D(x_meas, y_meas, th_meas)


def run_pursuit_test(path, path_name, pp_config=None, max_time=30.0):
    """경로 추종 테스트 실행."""

    dt = 0.02  # 50Hz
    max_steps = int(max_time / dt)

    # Pure Pursuit 설정
    if pp_config is None:
        pp_config = {}

    pp = PurePursuitController(
        lookahead_distance=pp_config.get('l_d', 0.4),
        min_lookahead=pp_config.get('l_d_min', 0.15),
        max_lookahead=pp_config.get('l_d_max', 1.0),
        adaptive_lookahead=pp_config.get('adaptive', True),
        lookahead_gain=pp_config.get('l_d_gain', 0.5),
        desired_velocity=pp_config.get('v', 0.3),
        max_omega=pp_config.get('max_omega', 1.5),
        goal_tolerance=pp_config.get('goal_tol', 0.1),
        velocity_curvature_gain=pp_config.get('v_curv_gain', 0.3),
    )
    pp.set_path(path)

    sim = DiffDriveSimulator()
    np.random.seed(42)

    # 기록
    robot_x, robot_y, robot_theta = [], [], []
    lookahead_xs, lookahead_ys = [], []
    cte_log, v_log, omega_log, curv_log = [], [], [], []
    time_log = []

    for step in range(max_steps):
        t = step * dt

        # 현재 위치 (EKF 출력!)
        pose = Pose2D(sim.x, sim.y, sim.theta)
        # 테스트에서는 GT 사용 (노이즈 추가는 sim.step에서!)

        # Pure Pursuit
        cmd = pp.compute(pose)

        if cmd.at_goal:
            break

        # 로봇 시뮬
        measured = sim.step(cmd.linear_velocity,
                            cmd.angular_velocity, dt)

        # 기록
        time_log.append(t)
        robot_x.append(sim.x)
        robot_y.append(sim.y)
        robot_theta.append(sim.theta)
        lookahead_xs.append(cmd.lookahead_x)
        lookahead_ys.append(cmd.lookahead_y)
        cte_log.append(cmd.cross_track_error)
        v_log.append(cmd.linear_velocity)
        omega_log.append(cmd.angular_velocity)
        curv_log.append(cmd.curvature)

    # 메트릭 계산
    cte_arr = np.array(cte_log)
    cte_rmse = float(np.sqrt(np.mean(cte_arr ** 2)))
    cte_max = float(np.max(np.abs(cte_arr)))
    completion_time = time_log[-1] if time_log else 0

    metrics = {
        'cte_rmse': cte_rmse,
        'cte_max': cte_max,
        'time': completion_time,
        'path_name': path_name,
    }

    data = {
        'path': path,
        'robot_x': robot_x, 'robot_y': robot_y,
        'robot_theta': robot_theta,
        'lookahead_x': lookahead_xs, 'lookahead_y': lookahead_ys,
        'cte': cte_log, 'v': v_log, 'omega': omega_log,
        'curvature': curv_log, 'time': time_log,
        'metrics': metrics,
    }

    return data


def plot_result(data: dict) -> plt.Figure:
    """4패널 결과 시각화."""

    m = data['metrics']
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(
        f'Pure Pursuit — {m["path_name"]} | '
        f'CTE_RMSE={m["cte_rmse"]*100:.1f}cm, '
        f'CTE_Max={m["cte_max"]*100:.1f}cm, '
        f'Time={m["time"]:.1f}s',
        fontsize=12, fontweight='bold'
    )

    path = data['path']
    px = [p[0] for p in path]
    py = [p[1] for p in path]

    # Panel 1: XY 궤적
    ax = axes[0, 0]
    ax.plot(px, py, 'k--', linewidth=1.5, alpha=0.5, label='Path')
    ax.plot(data['robot_x'], data['robot_y'], 'b-',
            linewidth=1.2, label='Robot')
    # Lookahead 표시 (매 50스텝)
    for i in range(0, len(data['lookahead_x']), 50):
        ax.plot([data['robot_x'][i], data['lookahead_x'][i]],
                [data['robot_y'][i], data['lookahead_y'][i]],
                'r-', alpha=0.2, linewidth=0.5)
        ax.plot(data['lookahead_x'][i], data['lookahead_y'][i],
                'r.', markersize=3)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Trajectory')
    ax.legend(fontsize=8)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # Panel 2: CTE
    ax = axes[0, 1]
    ax.plot(data['time'], [c * 100 for c in data['cte']],
            'b-', linewidth=1)
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax.fill_between(data['time'],
                     [-5] * len(data['time']),
                     [5] * len(data['time']),
                     alpha=0.05, color='green')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('CTE (cm)')
    ax.set_title(f'Cross-Track Error (RMSE={m["cte_rmse"]*100:.1f}cm)')
    ax.grid(True, alpha=0.3)

    # Panel 3: 속도
    ax = axes[1, 0]
    ax.plot(data['time'], data['v'], 'g-', linewidth=1, label='v (m/s)')
    ax.plot(data['time'], data['omega'], 'r-', linewidth=1,
            alpha=0.7, label='ω (rad/s)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity')
    ax.set_title('Velocity Commands')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel 4: 곡률
    ax = axes[1, 1]
    ax.plot(data['time'], data['curvature'], 'm-', linewidth=1)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Curvature κ (1/m)')
    ax.set_title('Path Curvature')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def test_all_paths():
    """전체 경로 테스트."""

    tests = [
        ('Straight 3m', generate_straight(3.0)),
        ('Circle R=1m', generate_circle(1.0)),
        ('Square 2m', generate_square(2.0)),
        ('Slalom', generate_slalom(0.5, 2.0, 6.0)),
        ('Figure-8', generate_figure8(1.0)),
    ]

    all_metrics = []

    for name, path in tests:
        print(f'\n▶ Testing: {name}')
        data = run_pursuit_test(path, name)
        m = data['metrics']
        all_metrics.append(m)

        fig = plot_result(data)
        filename = f'pp_{name.lower().replace(" ", "_").replace("-", "")}.png'
        fig.savefig(filename, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f'  CTE RMSE={m["cte_rmse"]*100:.1f}cm, '
              f'Max={m["cte_max"]*100:.1f}cm, '
              f'Time={m["time"]:.1f}s → {filename}')

    # 요약
    print('\n' + '═' * 60)
    print('  PURE PURSUIT PERFORMANCE SUMMARY')
    print('═' * 60)
    print(f'  {"Path":<18} {"CTE RMSE":>10} {"CTE Max":>10} {"Time":>8}')
    print('  ' + '─' * 48)
    for m in all_metrics:
        print(f'  {m["path_name"]:<18} '
              f'{m["cte_rmse"]*100:>8.1f}cm '
              f'{m["cte_max"]*100:>8.1f}cm '
              f'{m["time"]:>6.1f}s')
    print('═' * 60)

    return all_metrics


def test_lookahead_comparison():
    """Lookahead Distance 비교."""

    path = generate_slalom(0.5, 2.0, 6.0)
    l_d_values = [0.15, 0.3, 0.5, 0.8, 1.2]

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    px = [p[0] for p in path]
    py = [p[1] for p in path]

    ax1 = axes[0]
    ax1.plot(px, py, 'k--', linewidth=2, alpha=0.5, label='Path')

    ax2 = axes[1]

    for l_d in l_d_values:
        data = run_pursuit_test(
            path, f'l_d={l_d}',
            pp_config={'l_d': l_d, 'adaptive': False}
        )
        m = data['metrics']
        ax1.plot(data['robot_x'], data['robot_y'],
                 linewidth=1.2,
                 label=f'l_d={l_d}m (CTE={m["cte_rmse"]*100:.1f}cm)')
        ax2.plot(data['time'],
                 [abs(c) * 100 for c in data['cte']],
                 linewidth=1, label=f'l_d={l_d}m')

    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Trajectory — Lookahead Comparison')
    ax1.legend(fontsize=7)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)

    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('|CTE| (cm)')
    ax2.set_title('Absolute CTE')
    ax2.legend(fontsize=7)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


if __name__ == '__main__':
    np.random.seed(42)

    print('═' * 60)
    print('  Pure Pursuit Test Suite — Day 76')
    print('═' * 60)

    print('\n[Test 1] All Paths')
    test_all_paths()

    print('\n[Test 2] Lookahead Comparison')
    fig = test_lookahead_comparison()
    fig.savefig('pp_lookahead_comparison.png', dpi=150, bbox_inches='tight')
    print('  Saved: pp_lookahead_comparison.png')

    plt.show()
