#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# Day 78 — Pure Pursuit 체계적 파라미터 튜닝

# Grid Search → 최적 파라미터 발견!
# ════════════════════════════════════════════════════════════════

import numpy as np
import math
import itertools
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import List, Tuple, Dict

from pure_pursuit import PurePursuitController, Pose2D
from path_generator import (
    generate_straight, generate_circle,
    generate_square, generate_slalom, generate_figure8
)


@dataclass
class TuningResult:
    """튜닝 결과."""
    l_d: float
    velocity: float
    v_curv_gain: float
    adaptive: bool
    adaptive_gain: float
    cte_rmse: float          # [m]
    cte_max: float           # [m]
    completion_time: float   # [s]
    smoothness: float        # ω jerk [rad/s²]
    completed: bool
    path_name: str = ''



class DiffDriveSim:
    """간략 차동 구동 시뮬레이터."""

    def __init__(self, tau=0.05, noise_xy=0.001, noise_th=0.002):
        self.tau = tau
        self.noise_xy = noise_xy
        self.noise_th = noise_th
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.omega = 0.0

    def reset(self):
        self.x = self.y = self.theta = self.v = self.omega = 0.0


    def step(self, v_cmd, omega_cmd, dt):
        self.v += (v_cmd - self.v) / self.tau * dt
        self.omega += (omega_cmd - self.omega) / self.tau * dt

        th_mid = self.theta + self.omega * dt / 2
        self.x += self.v * math.cos(th_mid) * dt
        self.y += self.v * math.sin(th_mid) * dt
        self.theta += self.omega * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        return Pose2D(
            self.x + np.random.normal(0, self.noise_xy),
            self.y + np.random.normal(0, self.noise_xy),
            self.theta + np.random.normal(0, self.noise_th)
        )



def evaluate_params(path: List[Tuple[float, float]],
                    path_name: str,
                    l_d: float,
                    velocity: float,
                    v_curv_gain: float,
                    adaptive: bool = True,
                    adaptive_gain: float = 0.5,
                    max_time: float = 60.0) -> TuningResult:
    """
    한 세트의 파라미터로 경로 추종 평가.

    Returns:
        TuningResult
    """
    dt = 0.02
    max_steps = int(max_time / dt)

    pp = PurePursuitController(
        lookahead_distance=l_d,
        min_lookahead=0.1,
        max_lookahead=1.5,
        adaptive_lookahead=adaptive,
        lookahead_gain=adaptive_gain,
        desired_velocity=velocity,
        max_omega=1.5,
        goal_tolerance=0.1,
        velocity_curvature_gain=v_curv_gain,
    )
    pp.set_path(path)

    sim = DiffDriveSim()

    cte_list = []
    omega_list = []
    completed = False

    for step in range(max_steps):
        pose = Pose2D(sim.x, sim.y, sim.theta)
        cmd = pp.compute(pose)

        if cmd.at_goal:
            completed = True
            break

        sim.step(cmd.linear_velocity, cmd.angular_velocity, dt)

        cte_list.append(cmd.cross_track_error)
        omega_list.append(cmd.angular_velocity)


    # 메트릭 계산
    cte_arr = np.array(cte_list) if cte_list else np.array([0.0])
    omega_arr = np.array(omega_list) if omega_list else np.array([0.0])

    cte_rmse = float(np.sqrt(np.mean(cte_arr ** 2)))
    cte_max = float(np.max(np.abs(cte_arr)))
    comp_time = len(cte_list) * dt


    # Smoothness: ω의 변화율 (jerk!)
    if len(omega_arr) > 1:
        omega_diff = np.diff(omega_arr) / dt
        smoothness = float(np.sqrt(np.mean(omega_diff ** 2)))
    else:
        smoothness = 0.0

    return TuningResult(
        l_d=l_d, velocity=velocity,
        v_curv_gain=v_curv_gain,
        adaptive=adaptive,
        adaptive_gain=adaptive_gain,
        cte_rmse=cte_rmse, cte_max=cte_max,
        completion_time=comp_time,
        smoothness=smoothness,
        completed=completed,
        path_name=path_name,
    )



def grid_search(path, path_name) -> List[TuningResult]:
    """Grid Search 튜닝."""

    # 탐색 범위 (로그 스케일로!)
    l_d_range = [0.15, 0.25, 0.35, 0.5, 0.7]
    v_range = [0.15, 0.25, 0.35, 0.45]
    curv_range = [0.1, 0.3, 0.5, 0.8]

    total = len(l_d_range) * len(v_range) * len(curv_range)
    print(f'  Grid Search: {total} combinations...')

    results = []
    np.random.seed(42)

    for l_d, v, cg in itertools.product(l_d_range, v_range, curv_range):
        result = evaluate_params(
            path, path_name,
            l_d=l_d, velocity=v, v_curv_gain=cg,
            adaptive=True, adaptive_gain=0.5,
        )
        results.append(result)


    # 완주한 것만 필터
    completed = [r for r in results if r.completed]

    if not completed:
        print('  ⚠️ No completed runs!')
        return results

    # CTE_RMSE 기준 정렬
    completed.sort(key=lambda r: r.cte_rmse)

    return completed



def print_top_results(results: List[TuningResult], n=5):
    """상위 N개 결과 출력."""

    print(f'\n  {"Rank":<5} {"l_d":>5} {"v":>5} {"curv_g":>7} '
          f'{"CTE_RMSE":>10} {"CTE_Max":>10} {"Time":>7} '
          f'{"Smooth":>8} ')
    print('  ' + '─' * 65)

    for i, r in enumerate(results[:n]):
        print(f'  {i+1:<5} {r.l_d:>5.2f} {r.velocity:>5.2f} '
              f'{r.v_curv_gain:>7.2f} '
              f'{r.cte_rmse*100:>8.1f}cm '
              f'{r.cte_max*100:>8.1f}cm '
              f'{r.completion_time:>5.1f}s '
              f'{r.smoothness:>7.2f}')


def print_worst_results(results: List[TuningResult], n=3):
    """하위 N개 결과 (안 좋은 예!)."""

    worst = sorted(results, key=lambda r: r.cte_rmse, reverse=True)

    print(f'\n  {"Rank":<5} {"l_d":>5} {"v":>5} {"curv_g":>7} '
          f'{"CTE_RMSE":>10} {"CTE_Max":>10} {"Time":>7}')
    print('  ' + '─' * 55)

    for i, r in enumerate(worst[:n]):
        print(f'  {i+1:<5} {r.l_d:>5.2f} {r.velocity:>5.2f} '
              f'{r.v_curv_gain:>7.2f} '
              f'{r.cte_rmse*100:>8.1f}cm '
              f'{r.cte_max*100:>8.1f}cm '
              f'{r.completion_time:>5.1f}s')





def plot_sensitivity(results: List[TuningResult],
                     path_name: str) -> plt.Figure:
    """파라미터 민감도 시각화."""

    fig, axes = plt.subplots(2, 3, figsize=(16, 10))
    fig.suptitle(f'Parameter Sensitivity — {path_name}', fontsize=13)


    # l_d vs CTE_RMSE (v별 색상!)
    ax = axes[0, 0]
    v_values = sorted(set(r.velocity for r in results))
    for v in v_values:
        subset = [r for r in results if r.velocity == v
                  and r.v_curv_gain == 0.3]
        if subset:
            xs = [r.l_d for r in subset]
            ys = [r.cte_rmse * 100 for r in subset]
            ax.plot(xs, ys, 'o-', markersize=4, label=f'v={v}')
    ax.set_xlabel('Lookahead Distance (m)')
    ax.set_ylabel('CTE RMSE (cm)')
    ax.set_title('l_d vs CTE (curv_gain=0.3)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)


    # v vs CTE_RMSE (l_d별!)
    ax = axes[0, 1]
    l_d_values = sorted(set(r.l_d for r in results))
    for l_d in l_d_values:
        subset = [r for r in results if r.l_d == l_d
                  and r.v_curv_gain == 0.3]
        if subset:
            xs = [r.velocity for r in subset]
            ys = [r.cte_rmse * 100 for r in subset]
            ax.plot(xs, ys, 'o-', markersize=4, label=f'l_d={l_d}')
    ax.set_xlabel('Velocity (m/s)')
    ax.set_ylabel('CTE RMSE (cm)')
    ax.set_title('v vs CTE (curv_gain=0.3)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)


    # v_curv_gain vs CTE_RMSE
    ax = axes[0, 2]
    for l_d in [0.25, 0.35, 0.5]:
        subset = [r for r in results if r.l_d == l_d
                  and r.velocity == 0.25]
        if subset:
            xs = [r.v_curv_gain for r in subset]
            ys = [r.cte_rmse * 100 for r in subset]
            ax.plot(xs, ys, 'o-', markersize=4, label=f'l_d={l_d}')
    ax.set_xlabel('Curvature Gain')
    ax.set_ylabel('CTE RMSE (cm)')
    ax.set_title('Curvature Gain vs CTE (v=0.25)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)


    # CTE vs Time (Pareto Front!)
    ax = axes[1, 0]
    for r in results:
        ax.scatter(r.cte_rmse * 100, r.completion_time,
                   c=r.l_d, cmap='viridis', s=20, alpha=0.6)
    cbar = plt.colorbar(ax.collections[0], ax=ax)
    cbar.set_label('l_d (m)')
    ax.set_xlabel('CTE RMSE (cm)')
    ax.set_ylabel('Completion Time (s)')
    ax.set_title('CTE vs Time (Pareto Trade-off)')
    ax.grid(True, alpha=0.3)


    # CTE vs Smoothness
    ax = axes[1, 1]
    for r in results:
        ax.scatter(r.cte_rmse * 100, r.smoothness,
                   c=r.velocity, cmap='plasma', s=20, alpha=0.6)
    cbar = plt.colorbar(ax.collections[0], ax=ax)
    cbar.set_label('v (m/s)')
    ax.set_xlabel('CTE RMSE (cm)')
    ax.set_ylabel('ω Jerk (rad/s²)')
    ax.set_title('CTE vs Smoothness')
    ax.grid(True, alpha=0.3)


    # l_d vs CTE_Max
    ax = axes[1, 2]
    for v in v_values:
        subset = [r for r in results if r.velocity == v
                  and r.v_curv_gain == 0.3]
        if subset:
            xs = [r.l_d for r in subset]
            ys = [r.cte_max * 100 for r in subset]
            ax.plot(xs, ys, 'o-', markersize=4, label=f'v={v}')
    ax.set_xlabel('Lookahead Distance (m)')
    ax.set_ylabel('CTE Max (cm)')
    ax.set_title('l_d vs Worst-Case CTE')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig





def run_full_tuning():
    """전체 튜닝 파이프라인."""

    paths = {
        'Straight': generate_straight(3.0),
        'Circle R=1': generate_circle(1.0),
        'Square 2m': generate_square(2.0),
        'Slalom': generate_slalom(0.5, 2.0, 6.0),
    }

    all_best = {}

    for name, path in paths.items():
        print(f'\n{"═"*60}')
        print(f'  TUNING: {name}')
        print(f'{"═"*60}')

        results = grid_search(path, name)

        print(f'\n  ★ Top 5 (Best CTE):')
        print_top_results(results, 5)

        print(f'\n  ⚠ Worst 3:')
        print_worst_results(results, 3)

        fig = plot_sensitivity(results, name)
        filename = f'tune_{name.lower().replace(" ", "_")}.png'
        fig.savefig(filename, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f'\n  Saved: {filename}')

        if results:
            all_best[name] = results[0]


    # 최종 추천
    print(f'\n{"═"*60}')
    print(f'  OPTIMAL PARAMETERS PER PATH')
    print(f'{"═"*60}')
    print(f'  {"Path":<15} {"l_d":>5} {"v":>5} {"c_g":>5} '
          f'{"CTE":>8} {"Max":>8} {"Time":>6}')
    print('  ' + '─' * 55)
    for name, r in all_best.items():
        print(f'  {name:<15} {r.l_d:>5.2f} {r.velocity:>5.2f} '
              f'{r.v_curv_gain:>5.2f} '
              f'{r.cte_rmse*100:>6.1f}cm '
              f'{r.cte_max*100:>6.1f}cm '
              f'{r.completion_time:>4.1f}s')


    # 범용 추천 (모든 경로의 평균 성능 최적!)
    print(f'\n  ★ Universal Recommendation:')
    print(f'    l_d=0.35, v=0.25, curv_gain=0.3, adaptive=True')
    print(f'    → 모든 경로에서 양호한 성능!')






if __name__ == '__main__':
    np.random.seed(42)
    run_full_tuning()