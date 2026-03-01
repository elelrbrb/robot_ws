import numpy as np
import matplotlib.pyplot as plt

class HistogramFilter1D:
    """
    설계 의도: 1차원 격자 공간에서 베이즈 필터의 
    Predict-Update 사이클을 시각적으로 구현함.
    """
    def __init__(self, num_bins=100, world_size=10.0):
        self.num_bins = num_bins
        self.world_size = world_size
        self.dx = world_size / num_bins
        # 1. 초기 상태: 완전 무지 (Uniform Distribution)
        self.belief = np.full(num_bins, 1.0 / num_bins)
        
    def predict(self, move_steps, kernel_sigma=2):
        """
        예측 단계: 확률 분포 이동 + 확산(Convolution)
        move_steps: 이동할 칸 수
        kernel_sigma: 이동 오차(불확실성)의 크기
        """
        # 1. 확률 이동 (Shift)
        prior = np.roll(self.belief, move_steps)
        
        # 2. 확산 커널 생성 (Gaussian Kernel)
        # 로봇이 이동하며 발생하는 불확실성을 묘사함
        size = int(kernel_sigma * 3)
        x = np.arange(-size, size + 1)
        kernel = np.exp(-(x**2) / (2 * kernel_sigma**2))
        kernel /= kernel.sum()
        
        # 3. 합성곱(Convolution)을 통한 확률 확산
        self.belief = np.convolve(prior, kernel, mode='same')
        print(f"Prediction: Moved {move_steps} steps. Entropy increased.")

    def update(self, sensor_pos, sensor_sigma=3):
        """
        보정 단계: 센서 관측을 통한 믿음 수정
        sensor_pos: 센서가 감지한 위치(칸 인덱스)
        sensor_sigma: 센서의 정밀도
        """
        # 1. 우도(Likelihood) 함수 생성
        # 센서가 감지한 위치 근처일 확률이 높음
        likelihood = np.zeros(self.num_bins)
        for i in range(self.num_bins):
            dist = abs(i - sensor_pos)
            likelihood[i] = np.exp(-(dist**2) / (2 * sensor_sigma**2))
            
        # 2. 베이지안 업데이트: 사후확률 = 우도 * 사전확률
        self.belief = self.belief * likelihood
        
        # 3. 정규화 (Normalization): 합을 1로 만듦
        self.belief /= self.belief.sum()
        print(f"Update: Sensed at bin {sensor_pos}. Entropy decreased.")



# --- 시뮬레이션 실행 ---
hf = HistogramFilter1D()

def plot_belief(step_name):
    plt.bar(range(hf.num_bins), hf.belief, color='blue', alpha=0.6)
    plt.title(step_name)
    plt.xlabel('Grid Index')
    plt.ylabel('Probability')
    plt.ylim(0, 0.2)
    plt.show()

# 1. 초기 믿음 시각화
plot_belief("Initial Belief (Uniform)")

# 2. 첫 번째 문 발견 (20번 칸 근처)
hf.update(sensor_pos=20)
plot_belief("After 1st Update (Sensed Gate at 20)")

# 3. 로봇 주행 (30칸 이동)
hf.predict(move_steps=30)
plot_belief("After Prediction (Moved 30 steps)")

# 4. 두 번째 문 발견 (50번 칸 근처)
hf.update(sensor_pos=50)
plot_belief("After 2nd Update (Sensed Gate at 50 - Converged!)")