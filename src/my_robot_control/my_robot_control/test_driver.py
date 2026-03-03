#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
from tf_transformations import euler_from_quaternion


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower_node')
        
        # 파라미터 선언
        self.declare_parameter('wait_time', 3.0)
        self.declare_parameter('linear_tolerance', 0.1)
        self.declare_parameter('angular_tolerance', 0.03)  # ✅ 더 정밀하게
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.4)      
        self.declare_parameter('use_imu', True)
        self.declare_parameter('reposition_time', 2.0)
        
        self.wait_time = self.get_parameter('wait_time').get_parameter_value().double_value
        self.linear_tolerance = self.get_parameter('linear_tolerance').get_parameter_value().double_value
        self.angular_tolerance = self.get_parameter('angular_tolerance').get_parameter_value().double_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        self.use_imu = self.get_parameter('use_imu').get_parameter_value().bool_value
        self.reposition_time = self.get_parameter('reposition_time').get_parameter_value().double_value

        # 경로 시퀀스
        self.path_sequence = ['straight', 'square', 'figure8']
        self.current_path_index = 0
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_key', 10)
        
        # Subscriber - EKF 오도메트리
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )
        
        # Subscriber - IMU
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        # 타이머 (20Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # 경로 세그먼트
        self.segments = []
        self.current_path_name = ''
        self.load_path(self.path_sequence[self.current_path_index])
        
        # 상태 변수
        self.current_segment_index = 0
        self.state = 'IDLE'
        self.wait_start_time = None
        self.segment_start_time = None  # ✅ 세그먼트 시작 시간
        
        # 오도메트리 데이터
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        self.imu_yaw = None
        
        # 세그먼트 기준
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.target_yaw = 0.0
        self.target_distance = 0.0

        # 원 주행용 누적 각도 추적
        self.cumulative_yaw = 0.0
        self.prev_yaw = 0.0
        
        self.get_logger().info("=== Path Follower Initialized (Figure8 Fixed) ===")
        self.get_logger().info(f"Path Sequence: {self.path_sequence}")
        self.get_logger().info(f"Angular Tolerance: {math.degrees(self.angular_tolerance):.2f}°")
        self.get_logger().info(f"Reposition Time: {self.reposition_time}s")

    def load_path(self, path_name):
        self.current_path_name = path_name
        self.segments = []
        self.current_segment_index = 0
        self.cumulative_yaw = 0.0
        self.prev_yaw = 0.0
        
        if path_name == 'straight':
            self.segments.append(('LINEAR', 3.0))

        elif path_name == 'square':
            for i in range(4):
                self.segments.append(('LINEAR', 2.0))
                self.segments.append(('ANGULAR', math.pi/2)) 

        elif path_name == 'figure8':
            # ✅ 8 자: 왼쪽 원 → 재보정 → 오른쪽 원 (대칭 보장)
            self.segments.append(('CIRCLE_LEFT', 2*math.pi))
            self.segments.append(('REPOSITION', self.reposition_time))
            self.segments.append(('CIRCLE_RIGHT', 2*math.pi))
        else:
            self.segments.append(('LINEAR', 2.0))
        
        self.get_logger().info(f"Loaded Path: {path_name} (Segments: {len(self.segments)})")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, odom_yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        self.current_yaw = odom_yaw
        self.odom_received = True

    def imu_callback(self, msg):
        orientation = msg.orientation
        _, _, imu_yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        self.imu_yaw = imu_yaw

    def get_current_yaw(self):
        """IMU 또는 EKF 에서 yaw 획득"""
        if self.use_imu and self.imu_yaw is not None:
            return self.imu_yaw
        return self.current_yaw
    
    def update_cumulative_yaw(self, current_yaw):
        """누적 각도 계산 (360 도 이상 추적용)"""
        if not hasattr(self, 'prev_yaw_initialized'):
            self.prev_yaw_initialized = True
            self.prev_yaw = current_yaw
            self.cumulative_yaw = 0.0
            return
        
        # yaw 변화량 계산 (정규화)
        delta_yaw = self.normalize_angle(current_yaw - self.prev_yaw)
        self.cumulative_yaw += delta_yaw
        self.prev_yaw = current_yaw

    def timer_callback(self):
        if not self.odom_received:
            return
        
        current_time = self.get_clock().now()
        current_yaw = self.get_current_yaw()

        # 누적 각도 업데이트 (원 주행용)
        self.update_cumulative_yaw(current_yaw)
        
        if self.state == 'IDLE':
            self.state = 'DRIVING'
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_yaw = current_yaw
            self.prev_yaw = current_yaw
            self.cumulative_yaw = 0.0
            self.setup_segment_target(current_yaw)
            self.segment_start_time = current_time  # ✅ 초기화
            self.get_logger().info(f"=== Starting Path: {self.current_path_name} ===")
        
        elif self.state == 'DRIVING':
            segment_type, target_value = self.segments[self.current_segment_index]
            
            if segment_type == 'LINEAR':
                self.drive_linear(target_value)
            elif segment_type == 'ANGULAR':
                self.drive_angular_precise(target_value, current_yaw)
            elif segment_type == 'CIRCLE_LEFT':
                self.drive_circle(target_value, 1, current_yaw)
            elif segment_type == 'CIRCLE_RIGHT':
                self.drive_circle(target_value, -1, current_yaw)
            elif segment_type == 'REPOSITION':
                self.handle_reposition(current_time, target_value)
        
        elif self.state == 'WAITING':
            wait_elapsed = (current_time - self.wait_start_time).nanoseconds / 1e9
            if wait_elapsed >= self.wait_time:
                self.current_path_index += 1
                if self.current_path_index >= len(self.path_sequence):
                    self.stop_robot()
                    self.state = 'FINISHED'
                    self.get_logger().info("=== All Paths Completed ===")
                    return
                else:
                    self.load_path(self.path_sequence[self.current_path_index])
                    self.state = 'DRIVING'
                    self.start_x = self.current_x
                    self.start_y = self.current_y
                    self.start_yaw = current_yaw
                    self.prev_yaw = current_yaw
                    self.cumulative_yaw = 0.0
                    self.setup_segment_target(current_yaw)
                    self.segment_start_time = current_time
                    self.get_logger().info(f"=== Starting Path: {self.current_path_name} ===")
        
        elif self.state == 'FINISHED':
            pass

    def handle_reposition(self, current_time, wait_duration):
        """✅ 재보정 구간: 정지하여 EKF 보정 대기"""
        # ✅ 중요: segment_start_time 이 None 이면 현재 시간으로 설정
        if self.segment_start_time is None:
            self.segment_start_time = current_time
        
        elapsed = (current_time - self.segment_start_time).nanoseconds / 1e9
        
        if elapsed >= wait_duration:
            self.get_logger().info(f"✓ Reposition Complete ({wait_duration}s)")
            # ✅ 중요: 현재 위치를 새로운 기준으로 설정 (대칭 보장)
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_yaw = self.get_current_yaw()
            self.prev_yaw = self.start_yaw
            self.cumulative_yaw = 0.0
            self.segment_start_time = None  # ✅ 리셋
            self.next_segment()
        # 정지 상태 유지

    def setup_segment_target(self, current_yaw):
        if len(self.segments) == 0:
            return
        segment_type, target_value = self.segments[self.current_segment_index]
        if segment_type == 'ANGULAR':
            self.target_yaw = self.start_yaw + target_value

    def drive_linear(self, target_distance):
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        distance_traveled = math.sqrt(dx**2 + dy**2)
        
        if distance_traveled >= target_distance - self.linear_tolerance:
            self.get_logger().info(f"✓ Linear: {distance_traveled:.3f}m / {target_distance:.1f}m")
            self.next_segment()
        else:
            remaining = target_distance - distance_traveled
            speed = min(self.linear_speed, remaining * 2.0)
            if speed < 0.1:
                speed = 0.1
            msg = Twist()
            msg.linear.x = speed
            self.cmd_pub.publish(msg)

    def drive_angular_precise(self, target_angle, current_yaw):
        """정밀 회전 제어 (P 제어 + 감속)"""
        target_absolute_yaw = self.start_yaw + target_angle
        yaw_diff = self.normalize_angle(target_absolute_yaw - current_yaw)
        
        if abs(yaw_diff) > 0.1:
            self.get_logger().debug(f"Rotate: {math.degrees(yaw_diff):.1f}° remaining")
        
        if abs(yaw_diff) <= self.angular_tolerance:
            self.get_logger().info(f"✓ Angular: {math.degrees(target_angle):.1f}° (error: {math.degrees(yaw_diff):.2f}°)")
            self.next_segment()
        else:
            k_p = 1.5
            angular_velocity = k_p * yaw_diff
            max_speed = self.angular_speed
            angular_velocity = max(-max_speed, min(max_speed, angular_velocity))
            if abs(angular_velocity) < 0.1:
                angular_velocity = 0.1 if angular_velocity > 0 else -0.1
            
            msg = Twist()
            msg.angular.z = angular_velocity
            self.cmd_pub.publish(msg)

    def drive_circle(self, total_angle, direction, current_yaw):
        """원 주행 - 누적 각도 사용 (360 도 이상 추적)"""
        traveled_angle = abs(self.cumulative_yaw)
        
        # 90 도 단위 로그
        if int(math.degrees(traveled_angle)) % 90 < 10:
            self.get_logger().info(f"🔄 Circle: {math.degrees(traveled_angle):.1f}° / {math.degrees(total_angle):.1f}°")
        
        if traveled_angle >= total_angle - self.angular_tolerance:
            self.get_logger().info(f"✓ Circle Complete: {math.degrees(traveled_angle):.1f}°")
            self.next_segment()
        else:
            msg = Twist()
            msg.linear.x = self.linear_speed
            msg.angular.z = direction * self.angular_speed
            self.cmd_pub.publish(msg)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def next_segment(self):
        self.stop_robot()
        self.current_segment_index += 1
        
        if self.current_segment_index >= len(self.segments):
            self.state = 'WAITING'
            self.wait_start_time = self.get_clock().now()
            self.get_logger().info(f"=== Path '{self.current_path_name}' Complete. Waiting {self.wait_time}s... ===")
        else:
            current_yaw = self.get_current_yaw()
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_yaw = current_yaw
            self.prev_yaw = current_yaw
            self.cumulative_yaw = 0.0
            
            # ✅ REPOSITION 세그먼트면 시간 기록
            segment_type, _ = self.segments[self.current_segment_index]
            if segment_type == 'REPOSITION':
                self.segment_start_time = self.get_clock().now()
                self.get_logger().info(f"⏸ Repositioning for {self.reposition_time}s...")
            
            self.setup_segment_target(current_yaw)
            self.get_logger().info(f"Segment {self.current_segment_index + 1}/{len(self.segments)} Started")

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()