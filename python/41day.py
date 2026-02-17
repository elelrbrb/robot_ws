class MotorMapper:
    def __init__(self, deadzone=15.0, v_nom=12.0):
        self.deadzone = deadzone  # 최소 기동 PWM (%)
        self.v_nom = v_nom        # 기준 전압 (V)

    def get_pwm(self, target_ratio, current_v_batt):
        """
        target_ratio: 0.0 ~ 1.0 (목표 속도 비율)
        current_v_batt: 현재 배터리 전압 (V)
        """
        if abs(target_ratio) < 0.01:
            return 0.0

        # 1. 전압 보정 계수 계산
        v_corr = self.v_nom / current_v_batt
        
        # 2. 전압 보정 적용
        corrected_ratio = target_ratio * v_corr
        
        # 3. 데드존 기반 매핑 (Scaling)
        pwm = self.deadzone + (100.0 - self.deadzone) * abs(corrected_ratio)
        
        # 4. 클램핑 (100% 초과 방지)
        return min(max(pwm, 0.0), 100.0)

# 사용 예시
mapper = MotorMapper(deadzone=18.0)
final_pwm = mapper.get_pwm(target_ratio=0.85, current_v_batt=11.1)
print(f"Final PWM Output: {final_pwm:.2f}%")