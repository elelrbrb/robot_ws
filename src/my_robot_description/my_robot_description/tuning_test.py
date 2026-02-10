"""
    물리 튜닝의 효과를 정량적으로 측정하는 테스트 스크립트.

    테스트 항목:
        1. 직선 주행 정확도 (1m 주행 후 오도메트리 오차)
        2. 회전 정확도 (360° 회전 후 Yaw 오차)
        3. 정지 거리 (0.5m/s에서 정지 명령 후 미끄러진 거리)
        4. 센서 노이즈 측정 (정지 시 LiDAR/IMU 표준편차)
"""

import rclpy
from rclpy.node import Node
import math
import time
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from tf_transformations import euler_from_quaternion


class TuningTest(Node):
    """
        물리 튜닝 품질을 정량 측정하는 노드.
        각 테스트를 순차적으로 실행하고 결과를 터미널에 리포트로 출력
    """
    
    def __init__(self):
        super().__init__('tuning_test')
        
        # ---- Publishers ---- # cmd_vel 발행: 로봇에 속도 명령!
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            
        # ---- State Variables ---- # 오도메트리에서 읽은 현재 상태
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_vx = 0.0
            
        # LiDAR 거리 데이터 (노이즈 측정용)
        self.lidar_ranges = []
        # IMU Z 가속도 데이터 (노이즈 측정용)
        self.imu_az = []
        
        # ---- Subscribers ---- # 오도메트리 구독: 로봇 위치/속도 추적
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
                                # LiDAR 구독: 노이즈 수집
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
                                # IMU 구독: 노이즈 수집
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
        
        # ---- 테스트 시작 (3초 후) ----
        self.create_timer(3.0, self.run_tests)
            # 노드 시작 후 3초 대기 (센서 안정화)
        
        self.test_done = False
    

    def odom_callback(self, msg):
        """오도메트리 데이터를 상태 변수에 저장."""
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        
        # 쿼터니언 → Yaw 변환
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        self.odom_yaw = yaw
        self.odom_vx = msg.twist.twist.linear.x
    

    def scan_callback(self, msg):
        """LiDAR의 정면(0°) 부근 10개 데이터 수집."""
        # 전방 = 배열 중간
        mid = len(msg.ranges) // 2

        valid = [
            r for r in msg.ranges[mid-5:mid+5]
            if not math.isinf(r) and not math.isnan(r)
        ]
        # inf, nan 제거
        if valid:
            self.lidar_ranges.extend(valid)
    

    def imu_callback(self, msg):
        """IMU Z축 가속도 수집."""
        self.imu_az.append(msg.linear_acceleration.z)
        
    

    def send_cmd(self, vx, wz):
        """속도 명령 발행 유틸리티."""
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.angular.z = float(wz)
        self.cmd_pub.publish(cmd)
    

    def stop(self):
        """정지 명령."""
        self.send_cmd(0.0, 0.0)
    

    def wait(self, seconds):
        """지정 시간 대기 (rclpy.spin 허용)."""
        start = time.time()
        while time.time() - start < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)
            # 50ms마다 콜백 처리 → 대기 중에도 odom/lidar/imu 데이터 수신!
    

    def run_tests(self):
        """모든 테스트를 순차 실행."""
        
        if self.test_done:
            return
        self.test_done = True
        
        self.get_logger().info('='*50)
        self.get_logger().info('  PHYSICS TUNING TEST REPORT')
        self.get_logger().info('='*50)
        

        # ================================================
        # Test 1: 직선 주행 정확도
        # ================================================
        self.get_logger().info('\n--- Test 1: Linear Accuracy ---')
        
        # 현재 위치 기록
        start_x = self.odom_x
        start_y = self.odom_y
        
        # 0.3 m/s로 전진 (~3.3초 → 약 1m)
        self.send_cmd(0.3, 0.0)
        self.wait(3.33)
        self.stop()
        self.wait(2.0)  # 완전 정지 대기
        
        # 결과 계산
        dx = self.odom_x - start_x
        dy = self.odom_y - start_y
        distance = math.sqrt(dx**2 + dy**2)
        lateral_error = abs(dy)
            # dy: 좌우 편차 (직진이면 0이어야!)
        
        target = 1.0  # 목표: 1m
        linear_error = abs(distance - target)
        
        self.get_logger().info(f'  Target: {target:.2f}m, Actual: {distance:.3f}m')
        self.get_logger().info(f'  Distance Error: {linear_error:.4f}m ({linear_error*100:.1f}cm)')
        self.get_logger().info(f'  Lateral Drift: {lateral_error:.4f}m ({lateral_error*100:.1f}cm)')
        self.get_logger().info(f'  Grade: {"PASS" if linear_error < 0.05 else "NEEDS TUNING"}')
        # PASS 기준: 5cm 이내
        

        # ================================================
        # Test 2: 회전 정확도
        # ================================================
        self.get_logger().info('\n--- Test 2: Rotation Accuracy ---')
        
        start_yaw = self.odom_yaw
        
        # 0.5 rad/s로 회전 (2π/0.5 ≈ 12.57초 = 360°)
        self.send_cmd(0.0, 0.5)
        self.wait(12.57)
        self.stop()
        self.wait(2.0)
        
        end_yaw = self.odom_yaw
        
        yaw_diff = end_yaw - start_yaw
        # 각도 래핑: -π~+π 범위
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        # 360° 회전 후 원래 자리 → yaw_diff ≈ 0이어야!
        yaw_error_deg = abs(math.degrees(yaw_diff))
        
        self.get_logger().info(f'  Target: 360.0°, Residual: {yaw_error_deg:.2f}°')
        self.get_logger().info(f'  Grade: {"PASS" if yaw_error_deg < 5.0 else "NEEDS TUNING"}')
        # PASS 기준: 5° 이내
        
        
        # ================================================
        # Test 3: 정지 거리
        # ================================================
        self.get_logger().info('\n--- Test 3: Stopping Distance ---')
        
        # 0.5 m/s로 가속
        self.send_cmd(0.5, 0.0)
        self.wait(3.0)
        
        stop_x = self.odom_x
        
        # 정지!
        self.stop()
        self.wait(3.0)  # 충분히 대기
        
        final_x = self.odom_x
        stopping_distance = abs(final_x - stop_x)
        
        self.get_logger().info(f'  Speed: 0.5 m/s')
        self.get_logger().info(f'  Stopping Distance: {stopping_distance:.4f}m ({stopping_distance*100:.1f}cm)')
        self.get_logger().info(f'  Grade: {"PASS" if stopping_distance < 0.3 else "NEEDS TUNING"}')
        # PASS 기준: 30cm 이내 (실제 로봇도 이 정도)
        

        # ================================================
        # Test 4: 센서 노이즈 측정
        # ================================================
        self.get_logger().info('\n--- Test 4: Sensor Noise ---')
        
        self.stop()
        # 정지 상태에서 데이터 수집
        self.lidar_ranges = []
        self.imu_az = []
        self.wait(5.0)  # 5초간 데이터 수집
        
        if self.lidar_ranges:
            lidar_std = np.std(self.lidar_ranges)
            lidar_mean = np.mean(self.lidar_ranges)
            self.get_logger().info(
                f'  LiDAR (front): mean={lidar_mean:.3f}m, '
                f'std={lidar_std:.4f}m ({lidar_std*1000:.1f}mm)'
            )
        
        if self.imu_az:
            imu_std = np.std(self.imu_az)
            imu_mean = np.mean(self.imu_az)
            self.get_logger().info(
                f'  IMU (Z accel): mean={imu_mean:.3f} m/s², '
                f'std={imu_std:.4f} m/s²'
            )
            self.get_logger().info(
                f'  Gravity check: {imu_mean:.2f} (expected ~9.81)'
            )
        

        # ================================================
        # 종합 리포트
        # ================================================
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('  TEST COMPLETE')
        self.get_logger().info('='*50)


def main():
    rclpy.init()
    node = TuningTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
