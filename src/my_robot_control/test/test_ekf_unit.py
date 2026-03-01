#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# test_ekf_unit.py
# Day 65 — EKF 단위 테스트
# ════════════════════════════════════════════════════════════════

import numpy as np
import math
from my_robot_control.ekf_core import DiffDriveEKF, normalize_angle


def test_normalize_angle():
    """각도 래핑 테스트."""
    assert abs(normalize_angle(0.0) - 0.0) < 1e-10
    assert abs(normalize_angle(math.pi) - math.pi) < 1e-10
    assert abs(normalize_angle(-math.pi) - (-math.pi)) < 1e-10
    assert abs(normalize_angle(2 * math.pi) - 0.0) < 1e-10
    assert abs(normalize_angle(3 * math.pi) - math.pi) < 1e-10
    assert abs(normalize_angle(-3 * math.pi) - (-math.pi)) < 1e-10
    assert abs(normalize_angle(100.0) - normalize_angle(100.0)) < 1e-10
    print('[PASS] normalize_angle')


def test_predict_stationary():
    """정지 상태: 상태 변화 없어야!"""
    ekf = DiffDriveEKF()
    x0 = ekf.get_state().copy()
    P0 = ekf.get_covariance().copy()

    ekf.predict(v=0.0, omega=0.0, dt=0.02)

    x1 = ekf.get_state()
    # 정지 → 상태 불변!
    assert np.allclose(x0, x1, atol=1e-6), \
        f'State changed during stationary: {x0} → {x1}'

    # P는 약간 증가 (min_sigma로 인한 Q!)
    P1 = ekf.get_covariance()
    assert np.all(np.diag(P1) >= np.diag(P0) - 1e-10), \
        'P decreased without measurement!'

    print('[PASS] predict_stationary')


def test_predict_straight():
    """직진: x만 증가, y와 θ 변화 없어야!"""
    ekf = DiffDriveEKF(initial_state=[0, 0, 0])
    v, dt = 0.5, 0.02  # 0.5 m/s, 20ms

    ekf.predict(v=v, omega=0.0, dt=dt)
    state = ekf.get_state()

    expected_x = v * dt  # 0.01m
    assert abs(state[0] - expected_x) < 1e-6, \
        f'x: expected {expected_x}, got {state[0]}'
    assert abs(state[1]) < 1e-6, f'y should be 0, got {state[1]}'
    assert abs(state[2]) < 1e-6, f'θ should be 0, got {state[2]}'

    print('[PASS] predict_straight')


def test_predict_rotation():
    """제자리 회전: θ만 변하고 x, y 불변!"""
    ekf = DiffDriveEKF(initial_state=[1.0, 2.0, 0.0])
    omega, dt = 0.5, 0.02  # 0.5 rad/s

    ekf.predict(v=0.0, omega=omega, dt=dt)
    state = ekf.get_state()

    assert abs(state[0] - 1.0) < 1e-6, 'x should not change'
    assert abs(state[1] - 2.0) < 1e-6, 'y should not change'
    assert abs(state[2] - omega * dt) < 1e-6, \
        f'θ: expected {omega * dt}, got {state[2]}'

    print('[PASS] predict_rotation')


def test_update_reduces_uncertainty():
    """측정 업데이트는 항상 불확실성을 줄여야!"""
    ekf = DiffDriveEKF()

    # 여러 번 예측 → P 증가!
    for _ in range(50):
        ekf.predict(v=0.3, omega=0.1, dt=0.02)

    P_before = ekf.get_covariance().copy()

    # 오도메트리 업데이트
    ekf.update_odometry(ekf.get_state())  # 완벽한 측정!

    P_after = ekf.get_covariance()

    # 대각 원소가 감소해야!
    for i in range(3):
        assert P_after[i, i] <= P_before[i, i] + 1e-10, \
            f'P[{i},{i}] increased after update: {P_before[i,i]} → {P_after[i,i]}'

    print('[PASS] update_reduces_uncertainty')


def test_imu_corrects_yaw():
    """IMU가 Yaw를 효과적으로 보정하는지 확인."""
    ekf = DiffDriveEKF()

    # 예측으로 θ를 0.5 rad까지 올림
    for _ in range(100):
        ekf.predict(v=0.0, omega=0.5, dt=0.02)

    theta_before = ekf.get_state()[2]
    P_theta_before = ekf.get_covariance()[2, 2]

    # IMU가 "실제로는 0.3 rad" 이라고 측정
    ekf.update_imu(0.3)

    theta_after = ekf.get_state()[2]
    P_theta_after = ekf.get_covariance()[2, 2]

    # θ가 0.3 쪽으로 이동해야!
    assert abs(theta_after - 0.3) < abs(theta_before - 0.3), \
        'IMU update did not move θ toward measurement'

    # θ 불확실성 감소!
    assert P_theta_after < P_theta_before, \
        'IMU update did not reduce θ uncertainty'

    print('[PASS] imu_corrects_yaw')


def test_covariance_stays_positive_definite():
    """공분산이 항상 양의 정부호인지 확인."""
    ekf = DiffDriveEKF()
    np.random.seed(123)

    for _ in range(1000):
        v = np.random.uniform(-0.5, 0.5)
        omega = np.random.uniform(-1.0, 1.0)
        ekf.predict(v, omega, 0.02)

        if np.random.random() < 0.5:
            z = ekf.get_state() + np.random.normal(0, 0.05, 3)
            z[2] = normalize_angle(z[2])
            ekf.update_odometry(z)

        if np.random.random() < 0.3:
            theta_imu = ekf.get_state()[2] + np.random.normal(0, 0.1)
            ekf.update_imu(normalize_angle(theta_imu))

        P = ekf.get_covariance()
        eigenvalues = np.linalg.eigvalsh(P)
        assert np.all(eigenvalues >= -1e-10), \
            f'P has negative eigenvalue: {eigenvalues}'

    print('[PASS] covariance_positive_definite (1000 random steps)')


def test_angle_wrapping_boundary():
    """±π 경계에서의 래핑 처리 검증."""
    ekf = DiffDriveEKF(initial_state=[0, 0, 3.1])
    # θ ≈ π 근처!

    # θ를 증가시켜 π를 넘김
    for _ in range(10):
        ekf.predict(v=0.0, omega=0.5, dt=0.02)

    state = ekf.get_state()
    assert -math.pi <= state[2] <= math.pi, \
        f'θ out of range: {state[2]}'

    # -π 근처에서 IMU 업데이트
    ekf.update_imu(-3.0)  # -π 근처
    state = ekf.get_state()
    assert -math.pi <= state[2] <= math.pi, \
        f'θ out of range after IMU update: {state[2]}'

    print('[PASS] angle_wrapping_boundary')


if __name__ == '__main__':
    test_normalize_angle()
    test_predict_stationary()
    test_predict_straight()
    test_predict_rotation()
    test_update_reduces_uncertainty()
    test_imu_corrects_yaw()
    test_covariance_stays_positive_definite()
    test_angle_wrapping_boundary()
    print()
    print('All tests passed!')