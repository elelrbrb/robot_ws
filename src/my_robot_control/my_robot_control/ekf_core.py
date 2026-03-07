#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# Day 65 — 차동 구동 로봇 EKF (Extended Kalman Filter) 핵심 엔진

# 상태: x = [x, y, θ]ᵀ
# 입력: u = [v, ω]ᵀ
# 측정: z_odom = [x, y, θ]ᵀ, z_imu = [θ]

# 모션 모델: RK2 (Midpoint) 적분
# 야코비안: Day 64에서 해석적으로 유도
# Process Noise: Thrun의 속도 의존 모델
# ════════════════════════════════════════════════════════════════

import numpy as np
import math


def normalize_angle(angle: float) -> float:
    """
        각도를 [-π, +π] 범위로 정규화.
    """
    return math.atan2(math.sin(angle), math.cos(angle))



class DiffDriveEKF:
    def __init__(self,
                 initial_state: np.ndarray = None,
                 initial_covariance: np.ndarray = None,
                 motion_noise_alpha: list = None,
                 odom_noise: np.ndarray = None,
                 imu_noise: float = None):
        """
            initial_state: [x, y, θ] 초기 상태 (기본: 원점)
            initial_covariance: 3×3 초기 공분산 (기본: 0.01×I)
            motion_noise_alpha: [α₁, α₂, α₃, α₄] Thrun 노이즈 파라미터
                α₁: 속도 → 직진 오차 (바퀴 슬립)
                α₂: 회전 → 직진 오차 (차동 비대칭)
                α₃: 속도 → 회전 오차 (비대칭 마찰)
                α₄: 회전 → 회전 오차 (캐스터 저항)
            odom_noise: [σ²_x, σ²_y, σ²_θ] 오도메트리 노이즈
            imu_noise: σ²_θ IMU Yaw 노이즈
        """

        # ════════════════════════════════════════
        # 상태 벡터: [x, y, θ]ᵀ
        # ════════════════════════════════════════
        if initial_state is not None:
            self.x = np.array(initial_state, dtype=np.float64)
        else:
            self.x = np.zeros(3, dtype=np.float64)
            # 시작: 원점, 전방 θ=0

        # ════════════════════════════════════════
        # 공분산 행렬: 3×3
        # ════════════════════════════════════════
        if initial_covariance is not None:
            self.P = np.array(initial_covariance, dtype=np.float64)
        else:
            self.P = np.diag([0.01, 0.01, 0.01])
            # 초기 불확실성:
                # σ_x = σ_y = 0.1m (10cm)
                # σ_θ = 0.1rad (5.7°)
                    # → 너무 작으면 첫 측정에 과반응!
                    # → 너무 크면 수렴 느림!

        # ════════════════════════════════════════
        # Motion Noise: Thrun 모델 α 파라미터
        # ════════════════════════════════════════
        if motion_noise_alpha is not None:
            self.alpha = motion_noise_alpha
        else:
            self.alpha = [0.05, 0.01, 0.01, 0.05]
            # α₁=0.05: 0.5 m/s에서 σ_v ≈ 0.025 m/s (5%)
            # α₄=0.05: 1.0 rad/s에서 σ_ω ≈ 0.05 rad/s (5%)
                # → 실내 차동 구동 로봇의 일반적 범위

        # ════════════════════════════════════════
        # Measurement Noise
        # ════════════════════════════════════════
        if odom_noise is not None:
            self.R_odom = np.diag(odom_noise)
        else:
            self.R_odom = np.diag([0.001, 0.001, 0.005])
            # σ_x = σ_y = 0.032m (3.2cm)
            # σ_θ = 0.071rad (4.1°)
                # → Day 53 Allan Variance 기반 추정

        if imu_noise is not None:
            self.R_imu = np.array([[imu_noise]])
        else:
            self.R_imu = np.array([[0.01]])
            # σ_θ = 0.1rad (5.7°)
                # → BNO055 Day 53 기반: 자기장 간섭 포함

        # ════════════════════════════════════════
        # 최소 Process Noise (정지 시에도 적용)
        # ════════════════════════════════════════
        self.min_sigma_v = 0.001       # 1mm/s
        self.min_sigma_omega = 0.001   # 0.001 rad/s

        # ════════════════════════════════════════
        # 디버깅/로깅용
        # ════════════════════════════════════════
        self.step_count = 0
        self.last_innovation_odom = None
        self.last_innovation_imu = None
        self.last_kalman_gain_odom = None
        self.last_kalman_gain_imu = None



    # ═══════════════════════════════════════════════════════
    #  PREDICT — 모션 모델 + 야코비안 + Process Noise
    # ═══════════════════════════════════════════════════════

    def predict(self, v: float, omega: float, dt: float) -> None:
        if dt <= 0.0:
            return
            # dt가 0이하이면 아무것도 하지 않음 → 센서 타이밍 오류 방어

        theta = self.x[2]

        # ────────────────────────────────────
        # ① 중간 각도 (RK2 Midpoint)
        # ────────────────────────────────────
        theta_mid = theta + omega * dt / 2.0
        # 왜 midpoint?
            # Euler: cos(θ_k) → 시작 각도만 사용 → 곡선 주행 시 부정확
            # RK2: cos(θ_k + δθ/2) → 중간 각도 → 2차 정확도!

        # ────────────────────────────────────
        # ② 상태 예측: x⁻ = f(x, u)
        # ────────────────────────────────────
        cos_tm = math.cos(theta_mid)
        sin_tm = math.sin(theta_mid)

        # f(x, u):
            # x_new = x + v × cos(θ_mid) × Δt
            # y_new = y + v × sin(θ_mid) × Δt
            # θ_new = θ + ω × Δt
        self.x[0] += v * cos_tm * dt
        self.x[1] += v * sin_tm * dt
        self.x[2] += omega * dt
        self.x[2] = normalize_angle(self.x[2])

        # ────────────────────────────────────
        # ③ 상태 전이 야코비안: F = ∂f/∂x
        # ────────────────────────────────────
        # Day 64 Section C에서 유도:
            # F = ┌ 1  0  -v×Δt×sin(θ_mid) ┐
            #     │ 0  1   v×Δt×cos(θ_mid) │
            #     └ 0  0   1                ┘
        ds = v * dt  # 이동 거리

        F = np.array([
            [1.0,  0.0,  -ds * sin_tm],
            [0.0,  1.0,   ds * cos_tm],
            [0.0,  0.0,   1.0]
        ])

        # ────────────────────────────────────
        # ④ Process Noise: Q = W × Q_input × Wᵀ
        # ────────────────────────────────────
        Q = self._compute_process_noise(v, omega, dt,
                                         cos_tm, sin_tm)

        # ────────────────────────────────────
        # ⑤ 공분산 예측: P⁻ = F × P × Fᵀ + Q
        # ────────────────────────────────────
        self.P = F @ self.P @ F.T + Q
        # F @ P @ F.T:
        # 현재 불확실성이 모션 모델을 통해 전파! → θ 불확실성이 x, y 불확실성으로 전파
        
        # + Q: 모션 자체의 불확실성 추가 → 움직일수록 불확실해짐!

        # ────────────────────────────────────
        # 공분산 대칭 보장 (수치 안정성)
        # ────────────────────────────────────
        self.P = (self.P + self.P.T) / 2.0
        # 부동소수점 오차로 P가 미세하게 비대칭 될 수 있음!
            # 비대칭 → 역행렬 불안정 → EKF 발산!
            # 강제 대칭화로 방지!

        self.step_count += 1



    # ═══════════════════════════════════════════════════════
    #  UPDATE — 오도메트리 측정
    # ═══════════════════════════════════════════════════════

    def update_odometry(self, z_odom: np.ndarray) -> None:
        z = np.array(z_odom, dtype=np.float64)

        # ────────────────────────────────────
        # ① 측정 야코비안 H
        # ────────────────────────────────────
        H = np.eye(3)
            # 오도메트리는 상태를 직접 관측!
            # h(x) = x → H = ∂h/∂x = I

        # ────────────────────────────────────
        # ② 혁신 (Innovation): ν = z - h(x⁻)
        # ────────────────────────────────────
        nu = z - self.x
        nu[2] = normalize_angle(nu[2])

        self.last_innovation_odom = nu.copy()

        # ────────────────────────────────────
        # ③ 혁신 공분산: S = H P⁻ Hᵀ + R
        # ────────────────────────────────────
        S = H @ self.P @ H.T + self.R_odom
        # H = I이므로 S = P⁻ + R_odom
        
        # S의 물리적 의미: "예측의 불확실성(P⁻)과 측정의 불확실성(R)의 합"
            # → S가 크면: 총 불확실성 높음
            # → K = P⁻/S → P⁻가 크면(예측 불확실): K ≈ 1 → 측정 신뢰!
            # → R이 크면(측정 불확실): K ≈ 0 → 예측 신뢰!

        # ────────────────────────────────────
        # ④ 칼만 이득: K = P⁻ Hᵀ S⁻¹
        # ────────────────────────────────────
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            # S가 특이행렬! (매우 드묾, 하지만 방어)
            return

        K = self.P @ H.T @ S_inv
        # K: 3×3 행렬
            # K[i,j]: "j번째 측정 혁신이 i번째 상태를 얼마나 보정?"
        
        # 예: K[0,2] → θ 혁신이 x를 보정하는 양
            # → P의 비대각 항(σ_xθ)이 클수록 → K[0,2]도 큼!
            # → "θ를 고치면 x도 같이 고쳐야" — 상관관계!

        self.last_kalman_gain_odom = K.copy()


        # ────────────────────────────────────
        # ⑤ 상태 업데이트: x⁺ = x⁻ + K × ν
        # ────────────────────────────────────
        self.x = self.x + K @ nu
        self.x[2] = normalize_angle(self.x[2])

        # ────────────────────────────────────
        # ⑥ 공분산 업데이트: Joseph Form
        # ────────────────────────────────────
        I_KH = np.eye(3) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R_odom @ K.T

        # 대칭 보장
        self.P = (self.P + self.P.T) / 2.0



    # ═══════════════════════════════════════════════════════
    #  UPDATE — IMU Yaw 측정
    # ═══════════════════════════════════════════════════════

    def update_imu(self, theta_imu: float) -> None:
    
        # ────────────────────────────────────
        # ① 측정 야코비안 H
        # ────────────────────────────────────
        H = np.array([[0.0, 0.0, 1.0]])
        # 1×3: IMU는 θ만 관측
            # h(x) = θ → H = [∂θ/∂x, ∂θ/∂y, ∂θ/∂θ] = [0, 0, 1]

        # ────────────────────────────────────
        # ② 혁신 (스칼라)
        # ────────────────────────────────────
        nu = normalize_angle(theta_imu - self.x[2])

        self.last_innovation_imu = nu

        # ────────────────────────────────────
        # ③ 혁신 공분산 (스칼라!)
        # ────────────────────────────────────
        S = H @ self.P @ H.T + self.R_imu
            # S = P[2,2] + R_imu → 1×1 행렬 (실질 스칼라!)
        S_val = S[0, 0]
        # 왜 스칼라?
            # 측정이 1차원(θ만)이므로 → 3×3 역행렬 대신 스칼라 나눗셈! → 매우 빠름!

        if abs(S_val) < 1e-12:
            return  # S ≈ 0: 분산이 0 = 완벽한 측정? 불가능 → 스킵

        # ────────────────────────────────────
        # ④ 칼만 이득 (3×1 벡터)
        # ────────────────────────────────────
        K = (self.P @ H.T) / S_val
        # P @ H.T = P @ [0,0,1]ᵀ = P의 3번째 열 = [P[0,2], P[1,2], P[2,2]]ᵀ
        # / S_val → 3×1 벡터
        
        # K[0]: IMU θ 혁신이 x를 보정하는 양 → P[0,2](x-θ 상관)이 크면 → x도 보정!
        # K[1]: IMU θ 혁신이 y를 보정하는 양
        # K[2]: IMU θ 혁신이 θ를 보정하는 양 ← 보통 가장 큼!

        self.last_kalman_gain_imu = K.copy()


        # ────────────────────────────────────
        # ⑤ 상태 업데이트
        # ────────────────────────────────────
        self.x = self.x + (K * nu).flatten()
            # K(3×1) × nu(스칼라) = 3×1 → flatten → 3,
        self.x[2] = normalize_angle(self.x[2])

        # ────────────────────────────────────
        # ⑥ 공분산 업데이트
        # ────────────────────────────────────
        I_KH = np.eye(3) - K @ H
        # K: 3×1, H: 1×3 → K@H: 3×3 
        # self.P = I_KH @ self.P
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R_imu @ K.T

        # 대칭 보장
        self.P = (self.P + self.P.T) / 2.0



    # ═══════════════════════════════════════════════════════
    #  INTERNAL — Process Noise 계산
    # ═══════════════════════════════════════════════════════

    def _compute_process_noise(self, v: float, omega: float,
                                dt: float,
                                cos_tm: float, sin_tm: float
                                ) -> np.ndarray:

        a1, a2, a3, a4 = self.alpha

        # 입력 노이즈 표준편차
        sigma_v = a1 * abs(v) + a2 * abs(omega)
        sigma_omega = a3 * abs(v) + a4 * abs(omega)

        # 최소값 보장 (정지 시에도 미세한 노이즈!)
        sigma_v = max(sigma_v, self.min_sigma_v)
        sigma_omega = max(sigma_omega, self.min_sigma_omega)

        # 입력 노이즈 공분산 (2×2)
        Q_input = np.array([
            [sigma_v ** 2,  0.0],
            [0.0,           sigma_omega ** 2]
        ])

        # 입력→상태 야코비안 W (3×2)
        half_vdt2 = v * dt * dt / 2.0
        W = np.array([
            [dt * cos_tm,   -half_vdt2 * sin_tm],
            [dt * sin_tm,    half_vdt2 * cos_tm],
            [0.0,            dt]
        ])

        # 상태 공간으로 전파: Q = W × Q_input × Wᵀ
        Q = W @ Q_input @ W.T
        # 결과: 3×3

        # 물리적 의미:
            # v에 노이즈가 있으면 → x, y에 cos/sin 방향으로 전파!
            # ω에 노이즈가 있으면 → θ에 직접 + x, y에 간접 전파!
                # → v가 클수록: x, y 노이즈 증가!
                # → ω가 클수록: θ 노이즈 증가!
                # → 정지 시: Q ≈ 매우 작음! (min_sigma만)

        return Q



    # ═══════════════════════════════════════════════════════
    #  ACCESSOR
    # ═══════════════════════════════════════════════════════

    def get_state(self) -> np.ndarray:
        """현재 상태 [x, y, θ] 반환."""
        return self.x.copy()

    def get_covariance(self) -> np.ndarray:
        """현재 3×3 공분산 행렬 반환."""
        return self.P.copy()

    def get_uncertainty(self) -> dict:
        """
        불확실성을 직관적 형태로 반환.

        Returns:
            dict: {
                'sigma_x': σ_x [m],
                'sigma_y': σ_y [m],
                'sigma_theta': σ_θ [rad],
                'sigma_theta_deg': σ_θ [°],
            }
        """
        return {
            'sigma_x': math.sqrt(max(self.P[0, 0], 0)),
            'sigma_y': math.sqrt(max(self.P[1, 1], 0)),
            'sigma_theta': math.sqrt(max(self.P[2, 2], 0)),
            'sigma_theta_deg': math.degrees(
                math.sqrt(max(self.P[2, 2], 0))
            ),
        }

    def get_diagnostics(self) -> dict:
        """디버깅용 진단 정보."""
        diag = {
            'step': self.step_count,
            'state': self.x.copy(),
            'P_diag': np.diag(self.P).copy(),
            'P_det': np.linalg.det(self.P),
        }
        if self.last_innovation_odom is not None:
            diag['innovation_odom'] = self.last_innovation_odom.copy()
        if self.last_innovation_imu is not None:
            diag['innovation_imu'] = self.last_innovation_imu
        return diag