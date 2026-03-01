import numpy as np
import matplotlib.pyplot as plt

class KalmanFilter1D:
    def __init__(self, dt, x_init, p_init, q_var, r_var):
        # 1. 시스템 모델 설정
        self.dt = dt
        self.A = np.array([[1, dt], [0, 1]])  # 상태 전이 행렬 (등속 모델)
        self.H = np.array([[1, 0]])           # 관측 행렬 (위치만 측정)
        
        # 2. 노이즈 설정
        self.Q = np.array([[q_var, 0], [0, q_var]])  # 프로세스 노이즈
        self.R = np.array([[r_var]])                  # 측정 노이즈
        
        # 3. 초기 상태 및 공분산
        self.x = x_init
        self.P = p_init

    def predict(self):
        # [Equation 1] 상태 예측: x⁻ = Ax
        self.x = self.A @ self.x
        # [Equation 2] 공분산 예측: P⁻ = APAᵀ + Q
        self.P = self.A @ self.P @ self.A.T + self.Q
        return self.x, self.P

    def update(self, z):
        # [Equation 3] 칼만 게인 계산: K = P⁻Hᵀ(HP⁻Hᵀ + R)⁻¹
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # [Equation 4] 상태 보정: x = x⁻ + K(z - Hx⁻)
        y_tilde = z - self.H @ self.x  # Innovation (잔차)
        self.x = self.x + K @ y_tilde
        
        # [Equation 5] 공분산 보정: P = (I - KH)P⁻
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ self.H) @ self.P
        
        return self.x, self.P, float(y_tilde.T @ np.linalg.inv(S) @ y_tilde) # NIS 반환
    

    

# 1. 환경 설정
dt = 0.1
steps = 100
true_v = 1.0
q_var = 0.01  # 모델은 꽤 정확함
r_var = 0.5   # 센서는 노이즈가 심함 (0.5m 분산)

# 2. 데이터 생성 (Ground Truth & Noisy Measurements)
gt_pos = [i * dt * true_v for i in range(steps)]
measurements = [p + np.random.normal(0, np.sqrt(r_var)) for p in gt_pos]

# 3. 칼만 필터 초기화
kf = KalmanFilter1D(dt, np.array([[0.0], [true_v]]), np.eye(2), q_var, r_var)

# 4. 루프 실행
estimates = []
covariances = []
nis_log = []

for z in measurements:
    kf.predict()
    x_hat, p_hat, nis = kf.update(np.array([[z]]))
    estimates.append(x_hat[0, 0])
    covariances.append(p_hat[0, 0])
    nis_log.append(nis)




# 시각화 코드
plt.figure(figsize=(12, 6))
plt.scatter(range(steps), measurements, color='red', alpha=0.3, label='Measurements (Noisy)')
plt.plot(gt_pos, 'g--', label='Ground Truth (Ideal)')
plt.plot(estimates, 'b-', linewidth=2, label='Kalman Filter Estimate')

# 공분산 밴드 (±2-sigma) 표시: 로봇의 확신도 시각화
lower_bound = np.array(estimates) - 2 * np.sqrt(covariances)
upper_bound = np.array(estimates) + 2 * np.sqrt(covariances)
plt.fill_between(range(steps), lower_bound, upper_bound, color='blue', alpha=0.1, label='Confidence Interval (2σ)')

plt.title('1D Kalman Filter Localization Performance')
plt.legend()
plt.grid(True)
plt.show()



