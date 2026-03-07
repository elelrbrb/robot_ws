#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# Day 73 — 차동 구동 모터 속도 컨트롤러

# 구조:
    # cmd_vel → IK → RPM_desired_L, RPM_desired_R
    # Encoder → RPM_actual_L, RPM_actual_R
    # PID_L(RPM_desired_L, RPM_actual_L) → PWM_L
    # PID_R(RPM_desired_R, RPM_actual_R) → PWM_R
# ════════════════════════════════════════════════════════════════

import math
from pid_controller import PIDController


class DiffDriveMotorController:
    """
    차동 구동 모터 컨트롤러.

    cmd_vel (v, ω) → 역운동학 → RPM → PID → PWM
    """

    def __init__(self,
                 wheel_radius: float = 0.0325,
                 wheel_separation: float = 0.345,
                 max_rpm: float = 330.0,
                 ticks_per_rev: int = 2244,
                 pid_kp: float = 1.0,
                 pid_ki: float = 5.0,
                 pid_kd: float = 0.01):

        # ── 로봇 파라미터 ──
        self.r = wheel_radius
        self.L = wheel_separation
        self.max_rpm = max_rpm
        self.ticks_per_rev = ticks_per_rev

        # ── 좌/우 독립 PID ──
        self.pid_left = PIDController(
            kp=pid_kp, ki=pid_ki, kd=pid_kd,
            output_min=-255.0, output_max=255.0,
            integral_max=150.0,
            derivative_filter_coeff=0.15,
        )
        self.pid_right = PIDController(
            kp=pid_kp, ki=pid_ki, kd=pid_kd,
            output_min=-255.0, output_max=255.0,
            integral_max=150.0,
            derivative_filter_coeff=0.15,
        )
        
        # ── 인코더 상태 ──
        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        # ── RPM 상태 ──
        self.rpm_left = 0.0
        self.rpm_right = 0.0
        self.rpm_desired_left = 0.0
        self.rpm_desired_right = 0.0



    # ═══════════════════════════════════════════════════════
    #  역운동학: cmd_vel → RPM
    # ═══════════════════════════════════════════════════════

    def cmd_vel_to_rpm(self, v: float, omega: float) -> tuple:
        """
        cmd_vel → 좌/우 바퀴 RPM.

        Day 36 역운동학:
          v_L = v - ω × L/2
          v_R = v + ω × L/2

        속도 → RPM:
          RPM = v_wheel × 60 / (2π × r)

        Args:
            v: 선속도 [m/s]
            omega: 각속도 [rad/s] (반시계=+)

        Returns:
            (rpm_left, rpm_right)
        """
        # 바퀴 선속도 [m/s]
        v_left = v - omega * self.L / 2.0
        v_right = v + omega * self.L / 2.0

        # 속도 → RPM
        rpm_left = v_left * 60.0 / (2.0 * math.pi * self.r)
        rpm_right = v_right * 60.0 / (2.0 * math.pi * self.r)

        # RPM 제한
        rpm_left = max(-self.max_rpm, min(self.max_rpm, rpm_left))
        rpm_right = max(-self.max_rpm, min(self.max_rpm, rpm_right))

        return rpm_left, rpm_right



    # ═══════════════════════════════════════════════════════
    #  인코더 → RPM 변환
    # ═══════════════════════════════════════════════════════

    def update_encoders(self, left_ticks: int, right_ticks: int,
                        dt: float):
        """
        인코더 틱으로 현재 RPM 계산.

        Args:
            left_ticks: 좌측 인코더 누적 틱
            right_ticks: 우측 인코더 누적 틱
            dt: 측정 간격 [s]
        """
        if dt <= 0.0:
            return

        # 틱 변화량
        delta_left = left_ticks - self.prev_left_ticks
        delta_right = right_ticks - self.prev_right_ticks

        # RPM 계산
            # RPM = (delta_ticks / ticks_per_rev) × (60 / dt)
        self.rpm_left = (delta_left / self.ticks_per_rev) * (60.0 / dt)
        self.rpm_right = (delta_right / self.ticks_per_rev) * (60.0 / dt)

        # 상태 업데이트
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks



    # ═══════════════════════════════════════════════════════
    #  PID 업데이트: RPM → PWM
    # ═══════════════════════════════════════════════════════

    def compute(self, v_cmd: float, omega_cmd: float,
                left_ticks: int, right_ticks: int,
                dt: float) -> tuple:
        """
        전체 제어 루프: cmd_vel + encoder → PWM.

        Args:
            v_cmd: 명령 선속도 [m/s]
            omega_cmd: 명령 각속도 [rad/s]
            left_ticks: 좌측 인코더 틱
            right_ticks: 우측 인코더 틱
            dt: 시간 간격 [s]

        Returns:
            (pwm_left, pwm_right): 각각 -255 ~ +255 정수!
        """
        # ① IK: cmd_vel → RPM 목표
        self.rpm_desired_left, self.rpm_desired_right = \
            self.cmd_vel_to_rpm(v_cmd, omega_cmd)

        # ② 인코더 → RPM 실제
        self.update_encoders(left_ticks, right_ticks, dt)

        # ③ PID: RPM 오차 → PWM
        pwm_left_raw = self.pid_left.compute(
            setpoint=self.rpm_desired_left,
            measurement=self.rpm_left,
            dt=dt
        )
        pwm_right_raw = self.pid_right.compute(
            setpoint=self.rpm_desired_right,
            measurement=self.rpm_right,
            dt=dt
        )

        # ④ 정수 변환 (PWM은 정수!)
        pwm_left = int(round(pwm_left_raw))
        pwm_right = int(round(pwm_right_raw))

        # ⑤ 데드밴드 보상
            # 매우 낮은 PWM에서는 모터가 아예 안 돌림 → 최소 PWM 이하이면 0으로
        MIN_PWM = 30  # 모터가 움직이기 시작하는 최소 PWM
        if abs(pwm_left) < MIN_PWM and abs(self.rpm_desired_left) > 0.5:
            pwm_left = MIN_PWM if self.rpm_desired_left > 0 else -MIN_PWM
        if abs(pwm_right) < MIN_PWM and abs(self.rpm_desired_right) > 0.5:
            pwm_right = MIN_PWM if self.rpm_desired_right > 0 else -MIN_PWM

        # ⑥ 정지 명령
        if abs(v_cmd) < 0.001 and abs(omega_cmd) < 0.001:
            pwm_left = 0
            pwm_right = 0
            self.pid_left.reset()
            self.pid_right.reset()
            # 정지 시 PID 초기화!
            # → 적분 누적 제거! (다음 출발 시 잔류 방지!)

        return pwm_left, pwm_right



    # ═══════════════════════════════════════════════════════
    #  디버깅
    # ═══════════════════════════════════════════════════════

    def get_status(self) -> dict:
        """현재 상태 반환."""
        left_terms = self.pid_left.get_terms()
        right_terms = self.pid_right.get_terms()

        return {
            'rpm_desired_L': self.rpm_desired_left,
            'rpm_desired_R': self.rpm_desired_right,
            'rpm_actual_L': self.rpm_left,
            'rpm_actual_R': self.rpm_right,
            'error_L': left_terms['error'],
            'error_R': right_terms['error'],
            'pid_P_L': left_terms['p'],
            'pid_I_L': left_terms['i'],
            'pid_D_L': left_terms['d'],
            'pid_P_R': right_terms['p'],
            'pid_I_R': right_terms['i'],
            'pid_D_R': right_terms['d'],
        }