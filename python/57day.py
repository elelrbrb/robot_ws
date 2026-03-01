import numpy as np
import matplotlib.pyplot as plt

def gaussian_pdf(x, mu, sigma):
    """
    1D 가우시안 PDF 구현
    설계 의도: 수식의 각 항을 코드에 직접 매핑하여 수학적 이해를 도모함.
    """
    variance = sigma**2
    # 분모: 정규화 계수
    denominator = np.sqrt(2 * np.pi * variance)
    # 분자: 지수부 (오차의 제곱 / 2*분산)
    exponent = np.exp(-((x - mu)**2) / (2 * variance))
    
    return exponent / denominator

# 시각화 데이터 생성
x_range = np.linspace(-5, 5, 500)
params = [
    (0, 0.5, "Sharp (Low Uncertainty)"), # 정밀한 센서
    (0, 1.0, "Standard"),
    (0, 2.0, "Flat (High Uncertainty)")  # 노이즈 심한 센서
]

plt.figure(figsize=(10, 6))
for mu, sigma, label in params:
    y = gaussian_pdf(x_range, mu, sigma)
    plt.plot(x_range, y, label=f"{label}: μ={mu}, σ={sigma}")

plt.title("1D Gaussian Distributions: Visualization of Uncertainty")
plt.xlabel("State (x)")
plt.ylabel("Probability Density")
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend()
plt.show()