#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# Day 76 — Pure Pursuit 경로 추종 알고리즘 완전 구현

# 알고리즘:
    # ① 최근접 경로 점 탐색
    # ② Lookahead Point 탐색 (원-선분 교차!)
    # ③ 월드→로봇 좌표 변환
    # ④ 곡률 κ = 2×l_y / l_d²
    # ⑤ ω = v × κ
# ════════════════════════════════════════════════════════════════

import math
import numpy as np
from dataclasses import dataclass
from typing import List, Optional, Tuple


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


@dataclass
class PurePursuitCommand:
    """Pure Pursuit 출력."""
    linear_velocity: float = 0.0    # v [m/s]
    angular_velocity: float = 0.0   # ω [rad/s]
    curvature: float = 0.0          # κ [1/m]
    lookahead_x: float = 0.0        # Lookahead point (world)
    lookahead_y: float = 0.0
    cross_track_error: float = 0.0  # CTE [m]
    nearest_index: int = 0          # 최근접 경로 인덱스
    at_goal: bool = False           # 경로 끝 도달?



class PurePursuitController:
    def __init__(self,
                 lookahead_distance: float = 0.5,
                 min_lookahead: float = 0.2,
                 max_lookahead: float = 1.5,
                 adaptive_lookahead: bool = True,
                 lookahead_gain: float = 0.5,
                 desired_velocity: float = 0.3,
                 min_velocity: float = 0.05,
                 max_velocity: float = 0.5,
                 max_omega: float = 1.5,
                 goal_tolerance: float = 0.1,
                 velocity_curvature_gain: float = 0.3):
        """
            lookahead_distance: 기본 Lookahead 거리 [m]
                → 크게: 부드러운 추종! 하지만 코너 절삭!
                → 작게: 정밀 추종! 하지만 진동!

            min/max_lookahead: Adaptive 범위 [m]

            adaptive_lookahead: True이면 속도에 비례!
                → l_d = l_d_base + gain × v
                    → 빠르면 더 멀리 봐! (안정!)
                    → 느리면 가까이 봐! (정밀!)

            lookahead_gain: Adaptive 이득 [s]
                → l_d += gain × v

                
            desired_velocity: 기본 선속도 [m/s]

            velocity_curvature_gain: 곡률에 따른 감속 이득
                → 급커브에서 자동 감속!
                → v = v_desired / (1 + gain × |κ|)
        """

        # ── Lookahead ──
        self.l_d_base = lookahead_distance
        self.l_d_min = min_lookahead
        self.l_d_max = max_lookahead
        self.adaptive = adaptive_lookahead
        self.l_d_gain = lookahead_gain

        # ── 속도 ──
        self.v_desired = desired_velocity
        self.v_min = min_velocity
        self.v_max = max_velocity
        self.max_omega = max_omega
        self.v_curv_gain = velocity_curvature_gain

        # ── 목표 ──
        self.goal_tol = goal_tolerance

        # ── 경로 ──
        self.path: List[Tuple[float, float]] = []
        self.nearest_idx = 0



    # ═══════════════════════════════════════════════════════
    #  경로 설정
    # ═══════════════════════════════════════════════════════

    def set_path(self, path: List[Tuple[float, float]]):
        """
        경로 설정.

        path: [(x₀,y₀), (x₁,y₁), ..., (xₙ,yₙ)]
        최소 2점 이상!
        """
        self.path = path
        self.nearest_idx = 0



    # ═══════════════════════════════════════════════════════
    #  Step 1: 최근접 경로 점
    # ═══════════════════════════════════════════════════════
    def _find_nearest_point(self, robot: Pose2D) -> int:

        if len(self.path) == 0:
            return 0

        # 탐색 윈도우: 현재 ± 20점
        window = 20
        start = max(0, self.nearest_idx - 5)
        # 뒤로는 약간만! (이미 지나간 점으로 되돌아가지 않기 위해!)
        end = min(len(self.path), self.nearest_idx + window)

        min_dist = float('inf')
        min_idx = self.nearest_idx

        for i in range(start, end):
            dx = self.path[i][0] - robot.x
            dy = self.path[i][1] - robot.y
            dist = dx * dx + dy * dy
            # sqrt 생략! (비교만 하므로!)
            if dist < min_dist:
                min_dist = dist
                min_idx = i

        # 인덱스는 절대 뒤로 가지 않음! (단방향!)
        if min_idx >= self.nearest_idx:
            self.nearest_idx = min_idx

        return self.nearest_idx



    # ═══════════════════════════════════════════════════════
    #  Step 2: Lookahead Point (원-선분 교차!)
    # ═══════════════════════════════════════════════════════
    def _find_lookahead_point(self, robot: Pose2D,
                               l_d: float) -> Optional[Tuple[float, float]]:

        # nearest_idx부터 앞으로 탐색!
        for i in range(self.nearest_idx, len(self.path) - 1):
            # 선분 A → B
            ax, ay = self.path[i]
            bx, by = self.path[i + 1]

            # ── 원-선분 교차 ──
            point = self._circle_line_intersection(
                robot.x, robot.y, l_d,
                ax, ay, bx, by
            )

            if point is not None:
                return point

        # 교점 없음! → 경로 마지막 점 반환!
        if len(self.path) > 0:
            return self.path[-1]
        return None


    @staticmethod
    def _circle_line_intersection(
            cx: float, cy: float, radius: float,
            x1: float, y1: float,
            x2: float, y2: float) -> Optional[Tuple[float, float]]:

        # 방향 벡터
        dx = x2 - x1
        dy = y2 - y1

        # A에서 원 중심까지
        fx = x1 - cx
        fy = y1 - cy

        # 이차방정식 계수
        a = dx * dx + dy * dy
        # a = |B-A|² = 선분 길이의 제곱!
        # a = 0이면 A=B (길이 0 선분!) → 무시!
        if a < 1e-12:
            return None

        b = 2.0 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - radius * radius

        # 판별식
        discriminant = b * b - 4.0 * a * c

        if discriminant < 0:
            # 교점 없음!
            return None

        sqrt_disc = math.sqrt(discriminant)

        # 두 해
        t1 = (-b - sqrt_disc) / (2.0 * a)
        t2 = (-b + sqrt_disc) / (2.0 * a)

        # 유효한 t: [0, 1] 범위!
        # t2를 우선! (경로 진행 방향의 더 먼 점!)
        if 0.0 <= t2 <= 1.0:
            return (x1 + t2 * dx, y1 + t2 * dy)
        elif 0.0 <= t1 <= 1.0:
            return (x1 + t1 * dx, y1 + t1 * dy)

        return None



    # ═══════════════════════════════════════════════════════
    #  Step 3~5: 곡률 + 속도 명령
    # ═══════════════════════════════════════════════════════
    def compute(self, robot: Pose2D) -> PurePursuitCommand:
        """
        Pure Pursuit 메인 계산.

        Args:
            robot: 현재 로봇 포즈 (EKF 출력!)

        Returns:
            PurePursuitCommand: v, ω, κ, lookahead, CTE, ...
        """
        cmd = PurePursuitCommand()

        if len(self.path) < 2:
            cmd.at_goal = True
            return cmd


        # ── Step 1: 최근접 점 ──
        nearest_idx = self._find_nearest_point(robot)
        cmd.nearest_index = nearest_idx

        # CTE (Cross-Track Error)
        cmd.cross_track_error = self._compute_cte(robot, nearest_idx)


        # ── ✅ 경로 끝 도달 체크 (수정됨: 진행률 95% 추가!) ──
        goal_x, goal_y = self.path[-1]
        dist_to_goal = math.sqrt(
            (robot.x - goal_x)**2 + (robot.y - goal_y)**2
        )

        # 진행률 계산!
        progress = self.get_progress()

        # 거리 + 진행률 모두 만족해야 도착 → 시작점 근처에서 조기 도착 방지!
        if dist_to_goal < self.goal_tol and progress > 0.95:
            cmd.at_goal = True
            return cmd



        # ── Adaptive Lookahead ──
        if self.adaptive:
            # 기본: 속도 비례
            l_d = self.l_d_base + self.l_d_gain * abs(self.v_desired)

            # 개선 ①: 경로 곡률 기반 → 앞으로의 경로가 급커브이면 l_d 줄임
            path_curvature = self._estimate_path_curvature(self.nearest_idx)
            if abs(path_curvature) > 0.5:
                # 곡률 > 0.5/m (반지름 < 2m) → 급커브
                curv_factor = 1.0 / (1.0 + 2.0 * abs(path_curvature))
                # curv_factor: 1.0 (직선) ~ 0.33 (급커브)
                l_d *= curv_factor

            # 개선 ②: CTE 기반 → CTE가 크면 l_d 줄여서 빨리 복귀
            if abs(cmd.cross_track_error) > 0.05:  # 5cm 초과
                cte_factor = 1.0 / (1.0 + 5.0 * abs(cmd.cross_track_error))
                l_d *= cte_factor

            l_d = max(self.l_d_min, min(self.l_d_max, l_d))
        else:
            l_d = self.l_d_base

        # l_d가 goal보다 멀면 goal까지만!
        l_d = min(l_d, dist_to_goal + 0.1)
        l_d = max(l_d, self.l_d_min)



        # ── Step 2: Lookahead Point ──
        lookahead = self._find_lookahead_point(robot, l_d)
        if lookahead is None:
            cmd.at_goal = True
            return cmd

        cmd.lookahead_x = lookahead[0]
        cmd.lookahead_y = lookahead[1]



        # ── Step 3: 월드 → 로봇 좌표 변환 ──
        dx = lookahead[0] - robot.x
        dy = lookahead[1] - robot.y

        # 회전 변환 (월드 → 로봇 프레임!)
        cos_th = math.cos(robot.theta)
        sin_th = math.sin(robot.theta)

        local_x = cos_th * dx + sin_th * dy
        # local_x: 로봇 전방 방향 거리
        #   > 0: 앞에 있음!
        #   < 0: 뒤에 있음! (비정상!)

        local_y = -sin_th * dx + cos_th * dy
        # local_y: 로봇 좌측 방향 거리
        #   > 0: 왼쪽! → 좌회전!
        #   < 0: 오른쪽! → 우회전!



        # ── Step 4: 곡률 κ ──
        l_d_actual = math.sqrt(local_x**2 + local_y**2)

        if l_d_actual < 1e-6:
            # Lookahead가 로봇 위! → 직진!
            cmd.curvature = 0.0
        else:
            cmd.curvature = 2.0 * local_y / (l_d_actual ** 2)
            # κ = 2 × l_y / l_d²
            # 부호:
            #   l_y > 0 → κ > 0 → 좌회전!
            #   l_y < 0 → κ < 0 → 우회전!



        # ── Step 5: 속도 명령 ──

        # 5a: 곡률 기반 감속!
        # 급커브에서 자동으로 속도 줄임!
        v = self.v_desired / (1.0 + self.v_curv_gain * abs(cmd.curvature))
        v = max(self.v_min, min(self.v_max, v))

        # 5b: 목표 근처 감속!
        if dist_to_goal < self.l_d_base:
            decel_scale = dist_to_goal / self.l_d_base
            decel_scale = max(0.2, decel_scale)
            # 최소 20% 속도 유지 (정지 방지!)
            v *= decel_scale

        # 5c: Lookahead가 뒤에 있으면? (비정상!)
        if local_x < 0:
            # 로봇이 경로를 지나쳤음!
            # → 속도를 매우 낮추고 회전 강조!
            v = self.v_min

        # 5d: ω 계산!
        omega = v * cmd.curvature

        # 5e: ω 클램핑!
        omega = max(-self.max_omega, min(self.max_omega, omega))

        # ω가 클램핑되면 v도 조정!
        # (κ 관계 유지를 위해!)
        if abs(cmd.curvature) > 1e-6 and abs(omega) >= self.max_omega:
            v = abs(omega / cmd.curvature)
            v = min(v, self.v_max)

        cmd.linear_velocity = v
        cmd.angular_velocity = omega

        return cmd



    def _compute_cte(self, robot: Pose2D, nearest_idx: int) -> float:
        """
        Cross-Track Error 계산.

        로봇에서 경로까지의 수직 거리!
        부호:
          양수: 경로 왼쪽에 있음
          음수: 경로 오른쪽에 있음
        """
        if nearest_idx >= len(self.path) - 1:
            idx = len(self.path) - 2
        else:
            idx = nearest_idx

        ax, ay = self.path[idx]
        bx, by = self.path[idx + 1]

        # 선분 AB와 점 R의 부호 있는 거리
        # d = ((b-a) × (a-r)) / |b-a|
        # × = cross product (2D)

        abx = bx - ax
        aby = by - ay
        arx = robot.x - ax
        ary = robot.y - ay

        ab_len = math.sqrt(abx**2 + aby**2)
        if ab_len < 1e-9:
            return math.sqrt(arx**2 + ary**2)

        # 2D cross product: ab × ar = abx*ary - aby*arx
        cross = abx * ary - aby * arx
        # 부호: 양수이면 R이 AB의 왼쪽!
        #       음수이면 R이 AB의 오른쪽!

        return cross / ab_len



    def is_at_goal(self, robot: Pose2D) -> bool:
        """경로 끝 도달 확인."""
        if len(self.path) == 0:
            return True
        goal_x, goal_y = self.path[-1]
        dist = math.sqrt((robot.x - goal_x)**2 + (robot.y - goal_y)**2)
        return dist < self.goal_tol

    def get_progress(self) -> float:
        """경로 진행률 (0.0~1.0)."""
        if len(self.path) <= 1:
            return 1.0
        return self.nearest_idx / (len(self.path) - 1)
    


    def _estimate_path_curvature(self, idx: int) -> float:
        '''
            경로의 국소 곡률 추정.
            
            3점을 이용한 곡률 계산!
            κ = 2 × |AB × AC| / (|AB| × |BC| × |CA|)

            Menger Curvature 공식!
        '''
        if idx < 1 or idx >= len(self.path) - 1:
            return 0.0

        # 3점
        ax, ay = self.path[idx - 1]
        bx, by = self.path[idx]
        cx, cy = self.path[idx + 1]

        # 변 길이
        ab = math.sqrt((bx-ax)**2 + (by-ay)**2)
        bc = math.sqrt((cx-bx)**2 + (cy-by)**2)
        ca = math.sqrt((ax-cx)**2 + (ay-cy)**2)

        if ab < 1e-9 or bc < 1e-9 or ca < 1e-9:
            return 0.0

        # 외적 (2D → 스칼라!)
        cross = (bx-ax)*(cy-ay) - (by-ay)*(cx-ax)

        # Menger Curvature
        curvature = 2.0 * cross / (ab * bc * ca)

        return curvature
