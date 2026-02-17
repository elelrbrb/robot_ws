#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════
# Day 43 — 안전 계층 노드
    # 역할:
        # twist_mux의 출력을 받아 안전 검증 후 최종 /cmd_vel 발행.
        # 속도 클램핑, 가속도 제한, 장애물 감속, 타임아웃 감시.
    # 토픽:
        # 구독: /cmd_vel_mux (twist_mux 출력)
                # /scan (LiDAR)
        # 발행: /cmd_vel (최종, diff_drive로)
            # /safety_status (진단 정보)
# ════════════════════════════════════════════════════════════

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import math
import time


class SafetyLayerNode(Node):
    """
    안전 계층 노드: 모든 cmd_vel 명령이 이 노드를 거쳐야만 로봇에 도달
        4중 안전 장치:
            1) 속도 클램핑 — 물리적 한계 초과 방지
            2) 가속도 제한 — 급가속/급제동 방지
            3) 장애물 감속 — LiDAR 기반 충돌 방지
            4) 타임아웃 감시 — 통신 단절 시 자동 정지
    """

    def __init__(self):
        super().__init__('safety_layer_node')

        # ════════════════════════════════════════
        # 파라미터 선언
        # ════════════════════════════════════════

        # 속도 제한
        self.declare_parameter('max_linear_vel', 0.5)
            # 최대 선속도 [m/s]
            # 우리 로봇: 0.8 m/s 가능하지만, 실내 안전을 위해 0.5로 제한
        self.declare_parameter('max_angular_vel', 1.5)
            # 최대 각속도 [rad/s]
            # 1.5 rad/s ≈ 86°/s — 빠르게 회전하되 불안정하지 않게

        # 가속도 제한
        self.declare_parameter('max_linear_accel', 0.5)
            # 최대 선가속도 [m/s²]
            # 0에서 0.5 m/s까지 1초 걸림 — 부드러운 출발
        self.declare_parameter('max_angular_accel', 1.0)
            # 최대 각가속도 [rad/s²]

        # 장애물 감속
        self.declare_parameter('slowdown_distance', 1.0)
            # 이 거리 이내에 장애물 → 감속 시작 [m]
        self.declare_parameter('stop_distance', 0.3)
            # 이 거리 이내에 장애물 → 강제 정지! [m]
            # 로봇 반경(~0.15m) + 여유(0.15m)
        self.declare_parameter('obstacle_angle_range', 30.0)
            # 전방 ±30° 범위만 검사 [deg]
            # 측면 장애물은 주행에 영향 없음!

        # 타임아웃
        self.declare_parameter('cmd_timeout', 0.5)
            # 0.5초 이상 명령 없으면 → 정지
        self.declare_parameter('scan_timeout', 2.0)
            # 2초 이상 LiDAR 데이터 없으면 → 감속 모드

        # 파라미터 읽기
        self.max_v = self.get_parameter('max_linear_vel').value
        self.max_w = self.get_parameter('max_angular_vel').value
        self.max_a = self.get_parameter('max_linear_accel').value
        self.max_alpha = self.get_parameter('max_angular_accel').value
        self.slowdown_dist = self.get_parameter('slowdown_distance').value
        self.stop_dist = self.get_parameter('stop_distance').value
        self.obstacle_angle = self.get_parameter('obstacle_angle_range').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.scan_timeout = self.get_parameter('scan_timeout').value


        # ════════════════════════════════════════
        # 상태 변수
        # ════════════════════════════════════════
        self.current_v = 0.0
            # 현재 출력 중인 선속도 (가속도 제한용)
        self.current_w = 0.0
            # 현재 출력 중인 각속도
        self.min_obstacle_dist = float('inf')
            # LiDAR에서 감지된 전방 최소 거리
        self.last_cmd_time = self.get_clock().now()
            # 마지막 cmd_vel_mux 수신 시간
        self.last_scan_time = self.get_clock().now()
            # 마지막 /scan 수신 시간
        self.last_target_v = 0.0
        self.last_target_w = 0.0

        # ════════════════════════════════════════
        # 구독자
        # ════════════════════════════════════════
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel_mux',
                # twist_mux의 출력 토픽
            self.cmd_callback,
            10
        )

        # LiDAR는 Best Effort QoS가 필요할 수 있음
        scan_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT
                # 센서 데이터는 Best Effort가 일반적
                # Reliable로 하면 지연 발생 가능
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            scan_qos
        )

        # ════════════════════════════════════════
        # 발행자
        # ════════════════════════════════════════
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            # 최종 출력! diff_drive 플러그인이 이것을 구독!

        self.status_pub = self.create_publisher(String, '/safety_status', 10)
            # 디버깅용 상태 정보


        # ════════════════════════════════════════
        # 주기적 실행 타이머 (20Hz)
        # ════════════════════════════════════════
        self.timer = self.create_timer(0.05, self.timer_callback)
            # 0.05초 = 50ms = 20Hz
            
            # 왜 콜백에서 직접 publish하지 않고 타이머를 쓰는가?
                # 이유 1: 타임아웃 감지
                    # → cmd_vel_mux가 안 오더라도 타이머는 계속 돌아감
                    # → "0.5초 동안 안 왔네? → 정지!"
                # 이유 2: 일정한 출력 주기
                    # → 입력이 불규칙해도 출력은 정확히 20Hz
                    # → diff_drive가 안정적으로 동작

        self.get_logger().info(
            f'[SafetyLayer] Started. max_v={self.max_v}, max_w={self.max_w}, '
            f'stop_dist={self.stop_dist}'
        )


    # ════════════════════════════════════════
    # 콜백: cmd_vel_mux 수신
    # ════════════════════════════════════════
    def cmd_callback(self, msg: Twist):
        """twist_mux에서 선택된 명령을 저장."""
        self.last_target_v = msg.linear.x
        self.last_target_w = msg.angular.z
        self.last_cmd_time = self.get_clock().now()     
            # 타이머에서 이 값을 읽어서 처리
            # 콜백에서 직접 publish하지 않음! (타이밍 제어를 타이머에 위임)

    # ════════════════════════════════════════
    # 콜백: LiDAR /scan 수신
    # ════════════════════════════════════════
    def scan_callback(self, msg: LaserScan):
        """전방 장애물까지의 최소 거리를 계산."""
        self.last_scan_time = self.get_clock().now()    # 마지막으로 스캔 데이터를 받은 시간을 기록

        # ---- 전방 ±obstacle_angle° 범위의 인덱스 계산 ----
        angle_min = msg.angle_min
            # 라이다가 첫 번째 빔을 쏜 각도 [rad] (보통 -π 또는 0)    
        angle_increment = msg.angle_increment
            # 빔 사이의 각도 간격. [rad/sample]
        num_readings = len(msg.ranges)
            # 전체 샘플 수 (RPLiDAR A1: ~360개)

        # 전방(0°) 기준 ± obstacle_angle 범위
        angle_range_rad = math.radians(self.obstacle_angle)
            # 30° → 0.5236 rad

        min_dist = float('inf')     # 최소 거리를 저장할 변수.

        for i in range(num_readings):
            angle = angle_min + i * angle_increment
                # 이 레이저 빔의 각도

            # 전방 ±30° 범위인지 확인
            # 각도 래핑 고려: -π ~ +π
            if abs(angle) <= angle_range_rad:
                r = msg.ranges[i]
                    # 이 방향의 거리 [m]

                # 유효한 값만 사용 - 유효하다면 현재까지의 최소 거리와 비교해서 더 작은 값이면 갱신
                if msg.range_min < r < msg.range_max:
                    if r < min_dist:
                        min_dist = r

        self.min_obstacle_dist = min_dist



    # ════════════════════════════════════════
    # 타이머: 안전 검증 + 최종 cmd_vel 발행
    # ════════════════════════════════════════
    def timer_callback(self):
        """20Hz로 호출. 4중 안전 검증 후 /cmd_vel 발행."""

        dt = 0.05  # 타이머 주기 [s]
        now = self.get_clock().now()
        status_parts = []

        target_v = self.last_target_v
        target_w = self.last_target_w

        # ════════════════════════════════════════
        # 안전 장치 #4: 타임아웃 감시 (Watchdog)
        # ════════════════════════════════════════
        cmd_elapsed = (now - self.last_cmd_time).nanoseconds / 1e9
        if cmd_elapsed > self.cmd_timeout:
            target_v = 0.0
            target_w = 0.0
            status_parts.append('CMD_TIMEOUT')
            # cmd_vel_mux가 0.5초 이상 침묵
                # → 통신 단절, 노드 크래시, 또는 모든 소스 비활성
                # → 안전을 위해 정지!


        # ════════════════════════════════════════
        # 안전 장치 #1: 속도 클램핑 - 속도가 최대값을 넘지 못하게
        # ════════════════════════════════════════
        target_v = max(-self.max_v, min(self.max_v, target_v))
            # Clamp: -max_v ≤ target_v ≤ +max_v
            # 누군가 v=10.0을 보내도 → 0.5로 제한!
        target_w = max(-self.max_w, min(self.max_w, target_w))


        # ════════════════════════════════════════
        # 안전 장치 #3: 장애물 감속 (전진 시에만)
        # ════════════════════════════════════════
        if target_v > 0.0:
            # 전진 중에만 전방 장애물 검사
            # 후진 시에는 후방 센서가 없으므로 검사 안 함 (추후 추가 가능)

            # LiDAR 타임아웃 체크
            scan_elapsed = (now - self.last_scan_time).nanoseconds / 1e9
            if scan_elapsed > self.scan_timeout:
                    # LiDAR 데이터가 2초 이상 없음
                    # → 센서 고장? → 안전을 위해 50% 감속
                target_v *= 0.5
                status_parts.append('SCAN_TIMEOUT')

            elif self.min_obstacle_dist < self.stop_dist:
                    # 장애물이 0.3m 이내 → 강제 정지!
                target_v = 0.0
                status_parts.append(f'ESTOP(d={self.min_obstacle_dist:.2f})')

            elif self.min_obstacle_dist < self.slowdown_dist:
                    # 장애물이 0.3~1.0m → 비례 감속
                    # ratio: 0.3m에서 0, 1.0m에서 1
                ratio = (self.min_obstacle_dist - self.stop_dist) / \
                        (self.slowdown_dist - self.stop_dist)
                    # 예: dist=0.5m → ratio = (0.5-0.3)/(1.0-0.3) = 0.286
                    # → 속도를 28.6%로 줄임!
                target_v *= ratio
                status_parts.append(f'SLOW(d={self.min_obstacle_dist:.2f},r={ratio:.2f})')


        # ════════════════════════════════════════
        # 안전 장치 #2: 가속도 제한
        # ════════════════════════════════════════
        # 선속도 가속도 제한
        max_dv = self.max_a * dt
            # 1스텝(50ms)에 허용되는 최대 속도 변화
            # 0.5 × 0.05 = 0.025 m/s → 50ms 동안 최대 0.025 m/s까지만 속도가 변할 수 있음

            # 지금 속도에서 목표 속도까지 얼마나 차이가 나는가?
        dv = target_v - self.current_v 
            # 만약 속도 변화량(dv)이 허용치(max_dv)보다 크면 → 제한
        if abs(dv) > max_dv:    
            target_v = self.current_v + math.copysign(max_dv, dv)
                # copysign(magnitude, sign):
                # dv > 0 → +max_dv (가속)
                # dv < 0 → -max_dv (감속)

        # 각속도 가속도 제한
        max_dw = self.max_alpha * dt 
        dw = target_w - self.current_w
        if abs(dw) > max_dw:
            target_w = self.current_w + math.copysign(max_dw, dw)

        # 현재 값 갱신
        self.current_v = target_v
        self.current_w = target_w


        # ════════════════════════════════════════
        # 최종 발행
        # ════════════════════════════════════════
        cmd_msg = Twist()
        cmd_msg.linear.x = target_v
        cmd_msg.angular.z = target_w
        self.cmd_pub.publish(cmd_msg)

        # 상태 발행
        if status_parts:
            status_msg = String()
            status_msg.data = ' | '.join(status_parts)
            self.status_pub.publish(status_msg)



def main(args=None):
    rclpy.init(args=args)
    node = SafetyLayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 정지 명령!
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
            # Ctrl+C로 종료해도 마지막에 정지 명령을 보냄!
            # 없으면 → 마지막 속도로 계속 주행!
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
