import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

class GaussianFusionVisualizer:
    """
    설계 의도: 두 가우시안의 곱을 시각화하여 
    센서 융합의 기하학적 의미를 전달하는 전문 클래스.
    """
    def __init__(self, x_range: Tuple[float, float], num_points: int = 1000):
        # 시각화할 x축 범위 설정
        self.x = np.linspace(x_range[0], x_range[1], num_points)

    def pdf(self, mu: float, sigma: float) -> np.ndarray:
        """가우시안 확률 밀도 함수 계산 (Vectorized by NumPy)"""
        variance = sigma**2
        return (1.0 / np.sqrt(2.0 * np.pi * variance)) * \
               np.exp(-((self.x - mu)**2) / (2.0 * variance))

    def fuse(self, mu1: float, sigma1: float, mu2: float, sigma2: float) -> Tuple[float, float]:
        """두 가우시안의 곱셈 공식 (The heart of Kalman Filter)"""
        v1, v2 = sigma1**2, sigma2**2
        
        # 새로운 분산: 1/V_f = 1/V1 + 1/V2
        v_fused = (v1 * v2) / (v1 + v2)
        s_fused = np.sqrt(v_fused)
        
        # 새로운 평균: 가중 평균
        mu_fused = (mu1 * v2 + mu2 * v1) / (v1 + v2)
        
        return mu_fused, s_fused

    def visualize(self, m1: float, s1: float, m2: float, s2: float):
        # 1. 융합 결과 계산
        mf, sf = self.fuse(m1, s1, m2, s2)
        
        # 2. PDF 데이터 생성
        p1 = self.pdf(m1, s1)
        p2 = self.pdf(m2, s2)
        pf = self.pdf(mf, sf)

        # 3. 플로팅 아키텍처 구성
        plt.figure(figsize=(12, 7))
        
        # 센서 A (넓고 낮음)
        plt.plot(self.x, p1, 'b--', label=f'Sensor A: μ={m1}, σ={s1}', linewidth=1.5)
        plt.fill_between(self.x, p1, alpha=0.1, color='blue')
        
        # 센서 B (중간)
        plt.plot(self.x, p2, 'g--', label=f'Sensor B: μ={m2}, σ={s2}', linewidth=1.5)
        plt.fill_between(self.x, p2, alpha=0.1, color='green')
        
        # 융합 결과 (좁고 높음 - ✨ 융합의 힘!)
        plt.plot(self.x, pf, 'r-', label=f'Fused Result: μ={mf:.3f}, σ={sf:.3f}', linewidth=3)
        plt.fill_between(self.x, pf, alpha=0.2, color='red')

        # 데코레이션
        plt.axvline(mf, color='red', linestyle=':', alpha=0.5)
        plt.title('The Magic of Gaussian Multiplication (Sensor Fusion)', fontsize=16)
        plt.xlabel('State (Position [m])', fontsize=12)
        plt.ylabel('Probability Density', fontsize=12)
        plt.legend(loc='upper right')
        plt.grid(True, which='both', linestyle='--', alpha=0.5)
        
        print(f"📊 [Fusion Analysis Report]")
        print(f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        print(f"▶ Sensor A StdDev: {s1}")
        print(f"▶ Sensor B StdDev: {s2}")
        print(f"▶ Fused StdDev   : {sf:.4f} (Reduction: {((min(s1, s2)-sf)/min(s1, s2)*100):.1f}%)")
        print(f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        
        plt.show()

# --- 주행 테스트 실행 ---
viz = GaussianFusionVisualizer(x_range=(1.5, 5.0))
viz.visualize(m1=3.0, s1=0.5, m2=3.2, s2=0.3)