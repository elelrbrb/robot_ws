import math
import matplotlib.pyplot as plt

# ========================================================
# 1. FK 엔진 클래스 (Section E의 핵심 로직 통합)
# ========================================================
class DifferentialDriveFK:
    """
    차동 구동 로봇의 정운동학 엔진.
    L=0.345m, r=0.0325m 스펙 준수.
    """
    def __init__(self, L=0.345):
        self.L = L
        self.reset()

    def reset(self):
        """로봇의 상태를 원점(0,0,0)으로 초기화"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def step(self, v_L, v_R, dt):
        """한 스텝 진행 및 상태 반환"""
        v = (v_R + v_L) / 2.0
        w = (v_R - v_L) / self.L
        
        # 월드 좌표계 변환
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt
        
        # 각도 정규화 (-pi to pi)
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        return self.x, self.y, self.theta

# ========================================================
# 2. 시뮬레이션 시나리오 실행 함수
# ========================================================
def run_scenario(v_L_func, v_R_func, duration=10.0, dt=0.05):
    """
    주어진 속도 함수에 따라 시나리오를 실행하고 궤적 데이터를 반환합니다.
    """
    robot = DifferentialDriveFK()
    path_x, path_y, path_th = [0.0], [0.0], [0.0]
    
    steps = int(duration / dt)
    for i in range(steps):
        t = i * dt
        # 시간에 따른 속도 결정 (가변 속도 대응)
        v_L = v_L_func(t)
        v_R = v_R_func(t)
        
        x, y, th = robot.step(v_L, v_R, dt)
        path_x.append(x)
        path_y.append(y)
        path_th.append(th)
        
    return path_x, path_y, path_th

# ========================================================
# 3. 메인 시각화 로직
# ========================================================
def main():
    # --- 시나리오 1: 직진 (Straight) ---
    # v_L = 0.3, v_R = 0.3 (5초 주행)
    s1_x, s1_y, s1_th = run_scenario(lambda t: 0.3, lambda t: 0.3, duration=5.0)

    # --- 시나리오 2: 원형 주행 (Circle) ---
    # v_L = 0.2, v_R = 0.4 (10초 주행, L=0.345 대입 시 R=0.5175m)
    s2_x, s2_y, s2_th = run_scenario(lambda t: 0.2, lambda t: 0.4, duration=10.0)

    # --- 시나리오 3: S자 주행 (S-Curve) ---
    # 0~5초: 좌회전(v_L=0.2, v_R=0.4), 5~10초: 우회전(v_L=0.4, v_R=0.2)
    def s_curve_L(t): return 0.2 if t < 5.0 else 0.4
    def s_curve_R(t): return 0.4 if t < 5.0 else 0.2
    s3_x, s3_y, s3_th = run_scenario(s_curve_L, s_curve_R, duration=10.0)

    # --- 그래프 그리기 ---
    plt.figure(figsize=(12, 10))
    
    # 궤적 플롯
    plt.plot(s1_x, s1_y, 'g-', label='Straight (vL=0.3, vR=0.3)')
    plt.plot(s2_x, s2_y, 'b-', label='Circle (vL=0.2, vR=0.4)')
    plt.plot(s3_x, s3_y, 'r-', label='S-Curve (Reverse direction at 5s)')

    # 로봇 방향 표시 (Quiver plot)
    # 1초마다 화살표 하나씩 표시 (dt=0.05이므로 20스텝 간격)
    def draw_arrows(x, y, th, color):
        skip = 20
        plt.quiver(x[::skip], y[::skip], 
                   [math.cos(a) for a in th[::skip]], 
                   [math.sin(a) for a in th[::skip]], 
                   color=color, scale=15, width=0.005)

    draw_arrows(s1_x, s1_y, s1_th, 'green')
    draw_arrows(s2_x, s2_y, s2_th, 'blue')
    draw_arrows(s3_x, s3_y, s3_th, 'red')

    # 그래프 설정
    plt.axis('equal') # ⭐ 매우 중요: X축과 Y축의 비율을 1:1로 맞춤 (안 하면 원이 타원으로 보임)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xlabel('X Position (meters)', fontsize=12)
    plt.ylabel('Y Position (meters)', fontsize=12)
    plt.title(f'Differential Drive Trajectories (L={0.345}m)', fontsize=14)
    plt.legend()
    
    # 텍스트 정보 추가
    plt.text(0.1, -0.5, "Arrow indicates Robot Heading (Theta)", fontsize=10, bbox=dict(facecolor='white', alpha=0.5))

    plt.show()

if __name__ == "__main__":
    main()