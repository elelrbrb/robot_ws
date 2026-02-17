import math

class DifferentialDriveFK:
    """
    차동 구동 로봇의 정운동학(Forward Kinematics) 클래스.
    바퀴의 속도를 입력받아 월드 좌표계상의 위치 변화를 계산합니다.
    """
    
    def __init__(self, wheel_separation: float = 0.345, wheel_radius: float = 0.0325):
        """
        로봇의 하드웨어 스펙을 초기화합니다.
        
        :param wheel_separation: 바퀴 축간 거리 (L), 우리 로봇은 0.345m
        :param wheel_radius: 바퀴 반지름 (r), 우리 로봇은 0.0325m
        """
        self.L = wheel_separation # 바퀴 간격 저장 (m)
        self.r = wheel_radius     # 바퀴 반지름 저장 (m)
        
        # 로봇의 초기 상태 (x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def step(self, v_L: float, v_R: float, dt: float):
        """
        한 스텝(dt) 동안의 바퀴 속도를 입력받아 로봇의 위치를 업데이트합니다.
        
        :param v_L: 왼쪽 바퀴 선속도 (m/s)
        :param v_R: 오른쪽 바퀴 선속도 (m/s)
        :param dt: 샘플링 주기 (sec)
        :return: (x, y, theta) 업데이트된 상태
        """
        
        # 1. 로봇 본체의 선속도(v)와 각속도(omega) 유도 (Section B 공식)
        # v = (v_R + v_L) / 2
        linear_v = (v_R + v_L) / 2.0
        
        # omega = (v_R - v_L) / L
        angular_w = (v_R - v_L) / self.L
        
        # 2. 월드 좌표계로의 속도 사영 (Section C 공식)
        # dx/dt = v * cos(theta)
        vx = linear_v * math.cos(self.theta)
        
        # dy/dt = v * sin(theta)
        vy = linear_v * math.sin(self.theta)
        
        # 3. 오일러 적분을 통한 상태 업데이트 (수학 보충 공식)
        # x_new = x_old + vx * dt
        self.x += vx * dt
        
        # y_new = y_old + vy * dt
        self.y += vy * dt
        
        # theta_new = theta_old + omega * dt
        self.theta += angular_w * dt
        
        # 4. 각도 정규화 (Normalization)
        # theta 값을 -pi ~ pi 범위로 유지하여 수치적 안정성 확보
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        return self.x, self.y, self.theta

# --- 시뮬레이션 테스트 실행 ---
if __name__ == "__main__":
    # 로봇 객체 생성 (L=0.345m)
    robot = DifferentialDriveFK()
    
    # 5초 동안 0.1초 간격으로 원형 궤적 시뮬레이션
    # v_L = 0.2 m/s, v_R = 0.4 m/s (Section D의 완만한 회전 케이스)
    dt = 0.1
    print(f"{'Time':<5} | {'X':<8} | {'Y':<8} | {'Theta (deg)':<10}")
    print("-" * 45)
    
    for i in range(51):
        t = i * dt
        x, y, th = robot.step(v_L=0.2, v_R=0.4, dt=dt)
        if i % 10 == 0: # 1초마다 출력
            print(f"{t:<5.1f} | {x:<8.3f} | {y:<8.3f} | {math.degrees(th):<10.2f}")