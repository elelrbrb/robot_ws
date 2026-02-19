#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════
# Day 46 — LaserScan 메시지를 구독하고 분석하는 노드
    # 기능:
        # 1. 전방/좌/우/후 영역별 최소 거리 계산
        # 2. 유효/무효 데이터 비율 분석
        # 3. Polar → Cartesian 변환
        # 4. 간단한 장애물 감지
# ════════════════════════════════════════════════════════════

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan

import math


class LaserScanAnalyzer(Node):
    def __init__(self):
        super().__init__('laser_scan_analyzer')

        scan_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            scan_qos
        )

        self.scan_count = 0
        self.get_logger().info('[LaserScanAnalyzer] Waiting for /scan...')



    def scan_callback(self, msg: LaserScan):
        """LaserScan 수신 시 분석."""
        self.scan_count += 1

        # ════════════════════════════════════════
        # 1. 메타데이터 출력 (첫 번째 스캔만)
        # ════════════════════════════════════════
        if self.scan_count == 1:
            self.get_logger().info('=== LaserScan Metadata ===')
            self.get_logger().info(f'  frame_id:        {msg.header.frame_id}')
            self.get_logger().info(f'  angle_min:       {msg.angle_min:.4f} rad '
                                   f'({math.degrees(msg.angle_min):.1f}°)')
            self.get_logger().info(f'  angle_max:       {msg.angle_max:.4f} rad '
                                   f'({math.degrees(msg.angle_max):.1f}°)')
            self.get_logger().info(f'  angle_increment: {msg.angle_increment:.6f} rad '
                                   f'({math.degrees(msg.angle_increment):.2f}°)')
            self.get_logger().info(f'  num_readings:    {len(msg.ranges)}')        


            self.get_logger().info(f'  time_increment:  {msg.time_increment:.6f} s')
            self.get_logger().info(f'  range_min:       {msg.range_min:.2f} m')
            self.get_logger().info(f'  range_max:       {msg.range_max:.2f} m')
            self.get_logger().info('=' * 40)


        # ════════════════════════════════════════
        # 2. 유효 데이터 분석
        # ════════════════════════════════════════
        total = len(msg.ranges)
        valid_count = 0
        inf_count = 0
        nan_count = 0
        zero_count = 0

        for r in msg.ranges:
            if math.isnan(r):
                nan_count += 1
            elif math.isinf(r):
                inf_count += 1
            elif r <= 0.0:
                zero_count += 1
            elif msg.range_min <= r <= msg.range_max:
                valid_count += 1


        # ════════════════════════════════════════
        # 3. 영역별 최소 거리 계산
        # ════════════════════════════════════════
        # 영역 정의 (각도 범위):
            # 전방: -30° ~ +30°
            # 좌측: +30° ~ +90°
            # 우측: -90° ~ -30°
            # 후방: +150° ~ -150° (뒤쪽)

        # 각도 범위(min~max)와 현재까지 발견된 최소 거리(dist)를 저장
        # 처음에는 dist = ∞로 설정해서 "아직 아무것도 발견되지 않음"을 의미
        regions = {
            'front': {'min': math.radians(-30), 'max': math.radians(30), 'dist': float('inf')},
            'left':  {'min': math.radians(30),  'max': math.radians(90), 'dist': float('inf')},
            'right': {'min': math.radians(-90), 'max': math.radians(-30), 'dist': float('inf')},
            'back':  {'min': math.radians(150), 'max': math.radians(-150), 'dist': float('inf')},
        }

        for i, r in enumerate(msg.ranges):
            # 이 빔의 각도 계산
            # enumerate를 쓰면 i는 인덱스(몇 번째 빔인지), r은 해당 빔의 거리값(mm 단위)
            angle = msg.angle_min + i * msg.angle_increment
                # angle_min부터 시작해서 i번째 빔의 각도

            # 유효한 값인지 확인
            if math.isnan(r) or math.isinf(r) or r <= 0.0:
                continue
            if not (msg.range_min <= r <= msg.range_max):
                continue

            # 어느 영역에 속하는지 확인
            # region은 해당 영역의 정보(min, max, dist)를 담은 딕셔너리
            for name, region in regions.items():
                if name == 'back':
                    # 후방: 각도가 ±150° 이상 (래핑 필요)
                    if angle > region['min'] or angle < region['max']:
                        # 현재 거리 r가 지금까지 기록된 최소 거리보다 더 작을 경우 갱신
                        if r < region['dist']:
                            region['dist'] = r
                else:
                    if region['min'] <= angle <= region['max']:
                        if r < region['dist']:
                            region['dist'] = r


        # ════════════════════════════════════════
        # 4. 결과 출력 (10스캔마다)
        # ════════════════════════════════════════
        if self.scan_count % 10 == 0:
            self.get_logger().info(
                f'[Scan #{self.scan_count}] '
                f'Valid: {valid_count}/{total} ({100*valid_count/total:.0f}%) | '
                f'inf: {inf_count} | nan: {nan_count} | zero: {zero_count}'
            )
            self.get_logger().info(
                f'  Distances → '
                f'Front: {regions["front"]["dist"]:.2f}m | '
                f'Left: {regions["left"]["dist"]:.2f}m | '
                f'Right: {regions["right"]["dist"]:.2f}m | '
                f'Back: {regions["back"]["dist"]:.2f}m'
            )


    # ════════════════════════════════════════
    # 유틸리티: Polar → Cartesian 변환 (LaserScan 메시지의 극좌표 데이터를 직교좌표(XY)로 변환)
    # ════════════════════════════════════════
    @staticmethod
    def polar_to_cartesian(ranges, angle_min, angle_increment, range_min, range_max):
        """
        LaserScan의 ranges[]를 XY 좌표 리스트로 변환.

        반환: [(x1, y1), (x2, y2), ...] — LiDAR 프레임 기준
        """
        points = []
        for i, r in enumerate(ranges):
            # 유효성 검사
            if math.isnan(r) or math.isinf(r) or r <= 0.0:
                continue
            if not (range_min <= r <= range_max):
                continue

            angle = angle_min + i * angle_increment
            x = r * math.cos(angle)
                # 전방이 +X
            y = r * math.sin(angle)
                # 좌측이 +Y (ROS 좌표계 표준!)
            points.append((x, y))

        return points


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
