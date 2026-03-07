import matplotlib.pyplot as plt
import numpy as np
import time


class PIDController:
    """
    산업용 표준 기능을 포함한 고성능 PID 제어기 클래스
        - Anti-windup (Clamping 방식)
        - Derivative Filtering (저주파 통과 필터)
        - Output Saturation (출력 제한)
    """
    def __init__(self, kp, ki, kd, output_limits=(None, None), lpf_alpha=0.7):
        # 제어 이득(Gain) 설정
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # 출력 제한 설정 (min, max)
        self.min_output, self.max_output = output_limits
        
        # 미분 필터 계수 (0.0 ~ 1.0, 클수록 필터링 강화)
        self.alpha = lpf_alpha
        
        # 내부 상태 변수 초기화
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_derivative = 0.0
        self._last_time = None



    def reset(self):
        """제어기 상태 초기화 (로봇이 멈추거나 재시작할 때 호출)"""
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_derivative = 0.0
        self._last_time = None



    def compute(self, setpoint, measurement, dt=None):
        """
        제어 입력을 계산하는 메인 함수
            :param setpoint: 목표값 (Reference)
            :param measurement: 현재 측정값 (Process Variable)
            :param dt: 샘플링 타임 (None일 경우 시스템 시간 계산)
            :return: 제어 출력 (Control Output)
        """
        # 1. 시간 간격(dt) 계산
        now = time.time()
        if dt is None:
            if self._last_time is None:
                self._last_time = now
                return 0.0
            dt = now - self._last_time
        
        # dt가 0이거나 음수인 경우 에러 방지
        if dt <= 0: return 0.0
        

        # 2. 오차 계산 (Error)
        error = setpoint - measurement
        

        # 3. P항 계산 (Proportional)
        p_term = self.kp * error
        

        # 4. D항 계산 (Derivative) + 저주파 필터(LPF) 적용
        raw_derivative = (error - self._prev_error) / dt
        d_term_filtered = (self.alpha * self._prev_derivative) + \
                          ((1 - self.alpha) * self.kd * raw_derivative)
        

        # 5. I항 계산 (Integral) 및 안티 와인드업(Anti-windup)
        temp_integral = self._integral + (error * dt)
        i_term = self.ki * temp_integral
        

        # 6. 최종 출력 합산
        output = p_term + i_term + d_term_filtered
        

        # 7. 출력 클램핑(Clamping) 및 적분값 업데이트
        if self.max_output is not None and output > self.max_output:
            output = self.max_output
            # 출력이 최대치면 적분값 업데이트 중단 (Anti-windup)
        elif self.min_output is not None and output < self.min_output:
            output = self.min_output
            # 출력이 최소치면 적분값 업데이트 중단 (Anti-windup)
        else:
            # 정상 범위일 때만 적분값을 실제로 누적
            self._integral = temp_integral


        # 8. 다음 스텝을 위한 상태 저장
        self._prev_error = error
        self._prev_derivative = d_term_filtered / self.kd if self.kd != 0 else 0
        self._last_time = now
        
        return output





# 1. 플랜트(모터) 파라미터 설정
K_PLANT = 1.0   # 모터 이득
TAU = 0.5       # 시정수 (0.5초)
DT = 0.01       # 시뮬레이션 주기 (100Hz)
SIM_TIME = 10.0 # 총 시뮬레이션 시간 (초)

def simulate_pid(kp, ki, kd):
    # 제어기 초기화 (출력 제한 0~1.0)
    pid = PIDController(kp, ki, kd, output_limits=(0, 1.2), lpf_alpha=0.5)
    
    # 상태 변수 초기화
    time_history = np.arange(0, SIM_TIME, DT)
    y_history = []
    u_history = []
    current_y = 0.0 # 현재 속도
    setpoint = 1.0  # 목표 속도
    
    for t in time_history:
        # (1) 제어 출력 계산 (PID)
        u = pid.compute(setpoint, current_y, DT)
        
        # (2) 플랜트 반응 계산 (물리 법칙)
        dy_dt = (-current_y + K_PLANT * u) / TAU
        current_y += dy_dt * DT
        
        # 데이터 저장
        y_history.append(current_y)
        u_history.append(u)
        
    return time_history, y_history

# 3. 다양한 게인 조합 실험
results = [
    (2.0, 0.0, 0.0, 'P-only (Low Kp)'),
    (10.0, 0.0, 0.0, 'P-only (High Kp)'),
    (10.0, 20.0, 0.5, 'PID Optimized')
]

plt.figure(figsize=(10, 6))
for kp, ki, kd, label in results:
    t, y = simulate_pid(kp, ki, kd)
    plt.plot(t, y, label=label)

plt.axhline(1.0, color='r', linestyle='--', label='Target')
plt.title('PID Simulation: 1st Order Plant Response')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.legend()
plt.grid(True)
plt.show()

