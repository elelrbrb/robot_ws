import numpy as np
import matplotlib.pyplot as plt

class KalmanPredictor1D:
    def __init__(self, dt=0.1):
        # 1. 상태 전이 행렬 (A)
        self.A = np.array([[1, dt],
                           [0, 1]])
        
        # 2. 프로세스 노이즈 공분산 (Q)
        # 위치 노이즈는 작게(0.001), 속도 노이즈는 약간 크게(0.01) 설정
        self.Q = np.array([[0.001, 0],
                           [0, 0.01]])
        
        # 3. 초기 상태 추정치 (x_hat) - 위치 0, 속도 1m/s
        self.x = np.array([[0.0], 
                           [1.0]])
        
        # 4. 초기 오차 공분산 (P) - 단위 행렬로 시작
        self.P = np.eye(2)

    def predict(self):
        # [Step 1] 상태 예측: x_hat = A * x
        self.x = self.A @ self.x
        
        # [Step 2] 공분산 예측: P = A * P * A.T + Q
        self.P = self.A @ self.P @ self.A.T + self.Q
        
        return self.x, self.P

# --- 시뮬레이션 실행 ---
predictor = KalmanPredictor1D(dt=0.1)
history_p = []  # 위치 분산 기록
history_v = []  # 속도 분산 기록

for i in range(50): # 5초간 예측 (50 steps)
    x_hat, P_hat = predictor.predict()
    
    # P의 대각 성분(Variance) 추출
    history_p.append(P_hat[0, 0])
    history_v.append(P_hat[1, 1])

# 결과 시각화
plt.figure(figsize=(10, 5))
plt.plot(history_p, label='Position Variance ($P_{pp}$)')
plt.plot(history_v, label='Velocity Variance ($P_{vv}$)')
plt.title('Uncertainty Growth over Time (Prediction Only)')
plt.xlabel('Time Steps')
plt.ylabel('Variance (Uncertainty)')
plt.legend()
plt.grid(True)
plt.show()