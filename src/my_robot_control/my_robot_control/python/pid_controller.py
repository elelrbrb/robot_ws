#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# Day 73 — 범용 PID 컨트롤러 (순수 Python, 라이브러리 금지!)

# PID 수식 (Day 71 유도): u(t) = Kp × e(t) + Ki × ∫e(τ)dτ + Kd × de(t)/dt
# 이산화 (Day 71): u_k = Kp × e_k + Ki × Σe_j×Δt + Kd × (e_k - e_{k-1})/Δt

# 추가 기능:
    # ① Anti-Windup (적분 포화 방지!)
    # ② 미분 필터 (노이즈 증폭 방지!)
    # ③ 출력 클램핑 (액추에이터 한계!)
    # ④ 미분 Kick 방지 (Setpoint 변화 시!)
# ════════════════════════════════════════════════════════════════


class PIDController:

    def __init__(self,
                 kp: float = 0.0,
                 ki: float = 0.0,
                 kd: float = 0.0,
                 output_min: float = -255.0,
                 output_max: float = 255.0,
                 integral_max: float = 100.0,
                 derivative_filter_coeff: float = 0.1):
        """
            output_min/max: 출력 클램핑 범위
                → PWM 범위: -255~+255 (방향 포함!)

            integral_max: 적분항 절대값 상한
                → Anti-Windup! (적분 포화 방지!)

            derivative_filter_coeff: 미분 저역통과 필터 계수 (0~1)
                → 0.0: 필터 없음 (노이즈 그대로!)
                → 1.0: 완전 필터 (미분항 = 0!)
                → 0.1~0.3: 일반적 범위
        """

        # ── PID 이득 ──
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # ── 출력 제한 ──
        self.output_min = output_min
        self.output_max = output_max


        # ── Anti-Windup ──
        self.integral_max = integral_max
            # 적분항이 이 값을 넘으면 더 이상 누적하지 않음

        # ── 미분 필터 ──
        self.alpha = derivative_filter_coeff
            # 1차 저역통과: d_filtered = α × d_prev + (1-α) × d_raw
            # 인코더 노이즈가 미분항에서 증폭되는 것을 방지!

        # ── 내부 상태 ──
        self.integral = 0.0
            # 적분 누적값 = Σ(e_j × Δt)

        self.prev_error = 0.0
            # 이전 오차 (미분 계산용)

        self.prev_derivative = 0.0
            # 이전 필터링된 미분값 (필터 상태)

        self.prev_measurement = None
        # 이전 측정값 (미분 Kick 방지용)
            # 미분 Kick: setpoint가 갑자기 변하면 e_k - e_{k-1} 이 거대! → 출력 스파이크
            # → measurement의 미분을 쓰면 방지!
            # → d/dt(error) = d/dt(setpoint - measurement) ≈ -d/dt(measurement) (setpoint이 일정할 때)

        self.first_call = True
        # 첫 호출 플래그 (미분 초기화!)



    def compute(self,
                setpoint: float,
                measurement: float,
                dt: float) -> float:
        """
        PID 출력 계산.

        Args:
            setpoint: 목표값 (RPM_desired)
            measurement: 현재 측정값 (RPM_actual)
            dt: 시간 간격 [s]

        Returns:
            output: 제어 출력 (PWM 값!)
        """
        if dt <= 0.0:
            return 0.0

        # ────────────────────────────────────
        # ① 오차 계산
        # ────────────────────────────────────
        error = setpoint - measurement
            # error > 0: 속도 부족! → PWM 증가!
            # error < 0: 속도 초과! → PWM 감소!


        # ────────────────────────────────────
        # ② P항: 비례 출력
        # ────────────────────────────────────
        p_term = self.kp * error
            # 오차에 바로 비례!

        # ────────────────────────────────────
        # ③ I항: 적분 출력 + Anti-Windup
        # ────────────────────────────────────
        self.integral += error * dt
            # 오차 × 시간 = 누적!
            # 단위: [RPM × s] = [rev/min × s]

        # Anti-Windup: 클램핑!
        if self.integral > self.integral_max:
            self.integral = self.integral_max
        elif self.integral < -self.integral_max:
            self.integral = -self.integral_max

        i_term = self.ki * self.integral

        # ────────────────────────────────────
        # ④ D항: 미분 출력 (측정값 기반!)
        # ────────────────────────────────────
        if self.first_call:
            derivative_raw = 0.0
            self.first_call = False
        else:
        # 미분 Kick 방지!
            # setpoint 미분 대신 measurement 미분 사용!
            # d(error)/dt = d(sp)/dt - d(meas)/dt
        
            # sp이 일정하면: = -d(meas)/dt
            # sp이 변해도: measurement는 점진적 → 스파이크 없음!
            derivative_raw = -(measurement - self.prev_measurement) / dt
                # 부호: measurement 증가 → 오차 감소 → 출력 감소!
                # 그래서 음수!

        # 저역통과 필터
        derivative_filtered = (self.alpha * self.prev_derivative +
                               (1.0 - self.alpha) * derivative_raw)
            # α=0.1: 10% 이전값 + 90% 새값 → 약한 필터!
            # α=0.5: 50% 이전값 + 50% 새값 → 강한 필터!

        self.prev_derivative = derivative_filtered

        d_term = self.kd * derivative_filtered


        # ────────────────────────────────────
        # ⑤ 합산 + 출력 클램핑
        # ────────────────────────────────────
        output = p_term + i_term + d_term

        # 출력 제한!
        if output > self.output_max:
            output = self.output_max
            # Back-calculation Anti-Windup (선택적): 출력이 포화되면 적분도 멈춤!
            self.integral -= error * dt
        elif output < self.output_min:
            output = self.output_min
            self.integral -= error * dt


        # ────────────────────────────────────
        # ⑥ 상태 갱신
        # ────────────────────────────────────
        self.prev_error = error
        self.prev_measurement = measurement

        return output


    def reset(self):
        """PID 상태 초기화."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.prev_measurement = None
        self.first_call = True

    def set_gains(self, kp: float, ki: float, kd: float):
        """실시간 이득 변경 (튜닝용!)."""
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def get_terms(self) -> dict:
        """각 항의 기여도 반환 (디버깅!)."""
        return {
            'p': self.kp * self.prev_error,
            'i': self.ki * self.integral,
            'd': self.kd * self.prev_derivative,
            'integral_accum': self.integral,
            'error': self.prev_error,
        }