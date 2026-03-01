#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════
# Day 52 — IMU 데이터 처리 및 Covariance 설정 노드
    # 기능:
        # 1. /imu/data_raw (Gazebo/하드웨어) 구독
        # 2. Covariance 행렬 채우기
        # 3. Quaternion 검증
        # 4. Euler 각도 추출 및 표시
        # 5. 정제된 /imu/data 발행 (EKF 입력용!)

    # 왜 이 노드가 필요?
        # → Gazebo IMU 플러그인은 covariance를 0으로 발행!
        # → robot_localization(EKF)은 적절한 covariance를 기대!
        # → 이 노드가 covariance를 채워서 EKF 입력을 준비!
# ════════════════════════════════════════════════════════════

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import String

import math
import copy


class ImuProcessorNode(Node):
    """IMU 데이터 처리 + Covariance 설정 노드."""

    def __init__(self):
        super().__init__('imu_processor_node')

        # ════════════════════════════════════════
        # 파라미터: Covariance 값
        # ════════════════════════════════════════
       
        # Orientation
        self.declare_parameter('orientation_cov_roll', 5e-5)
        self.declare_parameter('orientation_cov_pitch', 5e-5)
        self.declare_parameter('orientation_cov_yaw', 2e-4)

        # Angular Velocity
        self.declare_parameter('angular_vel_cov', 3.74e-12)
        # Linear Acceleration
        self.declare_parameter('linear_accel_cov', 2.84e-08)
        


        # 중력 제거 여부
        self.declare_parameter('remove_gravity', False)
            # Gazebo: True로 하면 중력을 빼서 linear_acceleration 제공
            # BNO055: 이미 중력 제거됨 → False

        # 읽기
        self.ori_cov_r = self.get_parameter('orientation_cov_roll').value
        self.ori_cov_p = self.get_parameter('orientation_cov_pitch').value
        self.ori_cov_y = self.get_parameter('orientation_cov_yaw').value
        self.ang_cov = self.get_parameter('angular_vel_cov').value
        self.acc_cov = self.get_parameter('linear_accel_cov').value
        self.remove_gravity = self.get_parameter('remove_gravity').value


        # ════════════════════════════════════════
        # 구독 / 발행
        # ════════════════════════════════════════
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10
        )
            # /imu/data_raw: Gazebo 또는 하드웨어 직접 출력 (covariance 미설정)

        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
            # /imu/data: covariance 설정된 깨끗한 출력 → robot_localization(EKF)이 이것을 구독!

        self.info_pub = self.create_publisher(String, '/imu/info', 10)

        self.msg_count = 0
        self.prev_yaw = None

        self.get_logger().info('[ImuProcessor] Ready. Waiting for /imu/data_raw...')



    def imu_callback(self, msg: Imu):
        """IMU 메시지 처리."""
        self.msg_count += 1

        # ════════════════════════════════════════
        # 1. Quaternion 검증
        # ════════════════════════════════════════
        q = msg.orientation
        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)

        if abs(norm - 1.0) > 0.1:
            self.get_logger().warn(
                f'[IMU] Invalid quaternion! norm={norm:.4f}. Skipping.'
            )
            return

        # 정규화 (미세한 오차 보정) -> 이렇게 하면 길이가 정확히 1이 되도록 정규화(normalization)
        if abs(norm - 1.0) > 0.001:
            q.x /= norm
            q.y /= norm
            q.z /= norm
            q.w /= norm


        # ════════════════════════════════════════
        # 2. Euler 각도 추출
        # ════════════════════════════════════════
        roll, pitch, yaw = self._quaternion_to_euler(q.x, q.y, q.z, q.w)


        # ════════════════════════════════════════
        # 3. 중력 제거 (선택) -> 센서가 측정한 값에서 중력 성분을 제거하고, 순수한 움직임(가속도)
        # ════════════════════════════════════════
        out_msg = copy.deepcopy(msg) # 복사본을 만들어 안전하게 수정

        if self.remove_gravity:
            # 중력 벡터 계산 (현재 자세 기준)
                # 월드 중력: (0, 0, -9.81) in world frame
                # 센서 프레임에서의 중력 = R^T × (0, 0, -9.81)
                # R = quaternion_to_rotation_matrix(q)
            # 간소화 (정확한 방법):
                
            gx, gy, gz = self._gravity_in_body(q.x, q.y, q.z, q.w)
            out_msg.linear_acceleration.x -= gx
            out_msg.linear_acceleration.y -= gy
            out_msg.linear_acceleration.z -= gz


        # ════════════════════════════════════════
        # 4. Covariance 행렬 채우기
        # ════════════════════════════════════════

        # Orientation Covariance (3×3)
        out_msg.orientation_covariance = [0.0] * 9
        out_msg.orientation_covariance[0] = self.ori_cov_r    # σ²_roll
        out_msg.orientation_covariance[4] = self.ori_cov_p    # σ²_pitch
        out_msg.orientation_covariance[8] = self.ori_cov_y    # σ²_yaw

        # Angular Velocity Covariance (3×3)
        out_msg.angular_velocity_covariance = [0.0] * 9
        out_msg.angular_velocity_covariance[0] = self.ang_cov  # σ²_ωx
        out_msg.angular_velocity_covariance[4] = self.ang_cov  # σ²_ωy
        out_msg.angular_velocity_covariance[8] = self.ang_cov  # σ²_ωz

        # Linear Acceleration Covariance (3×3)
        out_msg.linear_acceleration_covariance = [0.0] * 9
        out_msg.linear_acceleration_covariance[0] = self.acc_cov  # σ²_ax
        out_msg.linear_acceleration_covariance[4] = self.acc_cov  # σ²_ay
        out_msg.linear_acceleration_covariance[8] = self.acc_cov  # σ²_az


        # ════════════════════════════════════════
        # 5. 발행
        # ════════════════════════════════════════
        self.imu_pub.publish(out_msg)

        # ════════════════════════════════════════
        # 6. 디버깅 출력 (50 프레임마다 = 0.5초)
        # ════════════════════════════════════════
        '''
        if self.msg_count % 50 == 0:
            # Yaw 변화율
            yaw_rate = 0.0
            if self.prev_yaw is not None:
                dyaw = yaw - self.prev_yaw
                # 래핑 처리
                if dyaw > math.pi:
                    dyaw -= 2 * math.pi
                elif dyaw < -math.pi:
                    dyaw += 2 * math.pi
                yaw_rate = dyaw / 0.5  # 0.5초 간격 (50 frames @ 100Hz)

            info = (
                f'[IMU #{self.msg_count}] '
                f'RPY: ({math.degrees(roll):+6.1f}°, '
                f'{math.degrees(pitch):+6.1f}°, '
                f'{math.degrees(yaw):+6.1f}°) | '
                f'ωz: {msg.angular_velocity.z:+.4f} rad/s | '
                f'az: {msg.linear_acceleration.z:.2f} m/s²'
            )

            self.get_logger().info(info)

            status = String()
            status.data = info
            self.info_pub.publish(status)

            self.prev_yaw = yaw
        '''



    # @staticmethod: 이 함수는 클래스 안에 있지만, 클래스의 객체(self)와는 상관없이 독립적으로 쓸 수 있다는 표시
    @staticmethod
    def _quaternion_to_euler(x, y, z, w):
        """Quaternion → (Roll, Pitch, Yaw)."""
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


    @staticmethod
    def _gravity_in_body(qx, qy, qz, qw):
        """
        Quaternion으로 월드 중력을 센서 프레임으로 변환.

        월드 중력: g = (0, 0, 9.81) in world frame (위쪽 양수)
            → Gazebo에서 az ≈ +9.81 (정지 시)

        센서 프레임의 중력: g_body = R^T × g_world
            R = quaternion_to_rotation_matrix

        최적화된 공식 (행렬 곱 없이 직접 계산):
        """
        g = 9.81

        # Rotation matrix 요소 (3열만 필요: 월드 Z축의 센서 표현)
            # 수학적으로 쿼터니언을 행렬로 바꿀 때 생기는 계수(두 배)
            # 중력 크기(9.81 m/s²)를 곱해서 실제 물리량으로

        gx = 2.0 * (qx * qz - qw * qy) * g
            # 회전 행렬에서 월드 Z축이 센서 X축으로 어떻게 보이는지 계산
        gy = 2.0 * (qy * qz + qw * qx) * g
            # 회전 행렬에서 월드 Z축이 센서 Y축으로 어떻게 보이는지 계산
        gz = (1.0 - 2.0 * (qx * qx + qy * qy)) * g
            # 회전 행렬에서 월드 Z축이 센서 Z축으로 어떻게 보이는지 계산

        return gx, gy, gz




def main(args=None):
    rclpy.init(args=args)
    node = ImuProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()