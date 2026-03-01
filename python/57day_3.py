import numpy as np

def calculate_mahalanobis(x, mu, cov):
    """
    설계 의도: 공식 그대로를 구현하여 통계적 거리의 산출 과정을 이해함.
    """
    x_minus_mu = x - mu
    # 역행렬 계산 (정밀도 행렬)
    inv_cov = np.linalg.inv(cov)
    
    # 거리 제곱 계산: (x-mu).T * inv_S * (x-mu)
    left_term = np.dot(x_minus_mu.T, inv_cov)
    mahalanobis_sq = np.dot(left_term, x_minus_mu)
    
    return np.sqrt(mahalanobis_sq)

# --- 테스트 시나리오 ---
mu = np.array([0, 0])
# X축으로는 너그럽고(1.0), Y축으로는 엄격한(0.01) 공분산
cov = np.array([[1.0, 0], [0, 0.01]])

pointA = np.array([0.5, 0])  # X로 0.5 이동
pointB = np.array([0, 0.5])  # Y로 0.5 이동

distA = calculate_mahalanobis(pointA, mu, cov)
distB = calculate_mahalanobis(pointB, mu, cov)

print(f"Distance A (X-drift): {distA:.2f} sigma") # 결과: 0.50 sigma (너그러움)
print(f"Distance B (Y-drift): {distB:.2f} sigma") # 결과: 5.00 sigma (매우 엄격!)