#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# Day 74 — 로봇 방향(θ) PID 컨트롤러

# 특징:
    # ① 각도 래핑 처리 (오차, 적분, 미분 모두!)
    # ② 도달 판정 (threshold 이내이면 "도착!")
    # ③ 각속도 제한 (로봇 하드웨어 한계!)
    # ④ 감속 프로파일 (목표 근처에서 부드럽게!)

# 입력: θ_current (EKF에서!), θ_goal
# 출력: ω (각속도 명령) → cmd_vel.angular.z
# ════════════════════════════════════════════════════════════════

import math


def normalize_angle(angle: float) -> float:
    """각도를 [-π, +π]로 정규화."""
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_difference(target: float, current: float) -> float:
    """
    두 각도 사이의 최단 호(Shortest Arc) 계산.

    항상 [-π, +π] 범위의 결과!
        양수: 반시계 회전 필요
        음수: 시계 회전 필요

    수학:
      diff = target - current
      result = atan2(sin(diff), cos(diff))

    이것은 normalize(target - current)과 동일하지만, 의도를 명확히 하기 위해 별도 함수로
    """
    diff = target - current
    return math.atan2(math.sin(diff), math.cos(diff))



class HeadingPIDController:
    """
    로봇 방향 PID 컨트롤러.

    ┌──────────────────────────────────────────────┐
    │                                              │
    │  θ_goal ──→ angle_diff ──→ e ──→ PID ──→ ω  │
    │               ↑                              │
    │               │                              │
    │  θ_current ───┘ (EKF /odometry/filtered)     │
    │                                              │
    └──────────────────────────────────────────────┘
    """

    def __init__(self,
                 kp: float = 2.0,
                 ki: float = 0.1,
                 kd: float = 0.5,
                 max_omega: float = 1.0,
                 min_omega: float = 0.05,
                 goal_tolerance: float = 0.02,
                 integral_max: float = 1.0,
                 derivative_filter: float = 0.15,
                 deceleration_zone: float = 0.3):
        """
            goal_tolerance: 도달 판정 임계값 [rad]

            integral_max: 적분 상한 [rad⋅s]

            derivative_filter: 미분 필터 계수 (0~1)

            deceleration_zone: 감속 영역 [rad]
                → 이 각도 이내이면 점진적 감속
        """

        # ── 이득 ──
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # ── 제한 ──
        self.max_omega = max_omega
        self.min_omega = min_omega
        self.goal_tolerance = goal_tolerance
        self.integral_max = integral_max
        self.decel_zone = deceleration_zone

        # ── 필터 ──
        self.alpha = derivative_filter

        # ── 내부 상태 ──
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.prev_theta = None
        self.first_call = True

        # ── 상태 ──
        self.at_goal = False
        self.error = 0.0



    def compute(self, theta_current: float,
                theta_goal: float,
                dt: float) -> float:
        """
        방향 PID 출력 계산.

        Args:
            theta_current: 현재 Yaw [rad]
            theta_goal: 목표 Yaw [rad]
            dt: 시간 간격 [s]

        Returns:
            omega: 각속도 명령 [rad/s]
                   양수 = 반시계, 음수 = 시계
        """
        if dt <= 0.0:
            return 0.0

        # ────────────────────────────────────
        # ① 각도 오차 (래핑!)
        # ────────────────────────────────────
        self.error = angle_difference(theta_goal, theta_current)

        # ────────────────────────────────────
        # ② 도달 판정
        # ────────────────────────────────────
        if abs(self.error) < self.goal_tolerance:
            self.at_goal = True
            self.integral = 0.0
            # 도달하면 적분 초기화 → 다음 목표에서 잔류 적분 없이 시작!
            return 0.0
        else:
            self.at_goal = False


        # ────────────────────────────────────
        # ③ P항
        # ────────────────────────────────────
        p_term = self.kp * self.error

        # ────────────────────────────────────
        # ④ I항 + Anti-Windup
        # ────────────────────────────────────
        self.integral += self.error * dt
        # self.error는 이미 래핑 → 적분에 래핑된 오차만 누적

        # 클램핑
        self.integral = max(-self.integral_max,
                        min(self.integral_max, self.integral))

        i_term = self.ki * self.integral

        # ────────────────────────────────────
        # ⑤ D항 (측정 기반 + 래핑!)
        # ────────────────────────────────────
        if self.first_call or self.prev_theta is None:
            derivative_raw = 0.0
            self.first_call = False
        else:
            # θ 변화량도 래핑
            delta_theta = angle_difference(theta_current, self.prev_theta)
                # delta_theta: θ가 실제로 변한 양 (래핑된!)
            derivative_raw = -delta_theta / dt
                # 음수: measurement 증가 → 오차 감소 → 출력 감소

        # 필터
        derivative_filtered = (self.alpha * self.prev_derivative +
                               (1.0 - self.alpha) * derivative_raw)
        self.prev_derivative = derivative_filtered

        d_term = self.kd * derivative_filtered


        # ────────────────────────────────────
        # ⑥ 합산
        # ────────────────────────────────────
        omega = p_term + i_term + d_term


        # ────────────────────────────────────
        # ⑦ 감속 프로파일
        # ────────────────────────────────────
        if abs(self.error) < self.decel_zone:
            # 목표 근처 → 속도 제한!
            # 비례 감속: 오차가 줄수록 최대 속도도 줄임!
            scale = abs(self.error) / self.decel_zone
                # scale: 0(목표) ~ 1(감속 영역 경계)
            max_allowed = self.max_omega * scale
            max_allowed = max(max_allowed, self.min_omega * 2)
                # 최소한 min_omega의 2배는 허용!
            omega = max(-max_allowed, min(max_allowed, omega))


        # ────────────────────────────────────
        # ⑧ 최종 클램핑
        # ────────────────────────────────────
        omega = max(-self.max_omega, min(self.max_omega, omega))


        # ────────────────────────────────────
        # ⑨ 데드밴드: 최소 속도 이하이면 최소값으로!
        # ────────────────────────────────────
        if 0 < abs(omega) < self.min_omega:
            omega = self.min_omega * (1 if omega > 0 else -1)
            # 모터가 매우 낮은 ω에서는 안 움직임 → 최소 ω 보장!

        # ────────────────────────────────────
        # ⑩ 상태 갱신
        # ────────────────────────────────────
        self.prev_error = self.error
        self.prev_theta = theta_current

        return omega


    def reset(self):
        """상태 초기화."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.prev_theta = None
        self.first_call = True
        self.at_goal = False

    def is_at_goal(self) -> bool:
        """목표 도달 여부."""
        return self.at_goal

    def get_diagnostics(self) -> dict:
        """디버깅 정보."""
        return {
            'error_rad': self.error,
            'error_deg': math.degrees(self.error),
            'integral': self.integral,
            'derivative': self.prev_derivative,
            'at_goal': self.at_goal,
            'p_term': self.kp * self.error,
            'i_term': self.ki * self.integral,
            'd_term': self.kd * self.prev_derivative,
        }