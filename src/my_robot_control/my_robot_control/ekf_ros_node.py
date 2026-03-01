#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# ekf_ros_node.py
# Day 66 — 차동 구동 로봇 EKF ROS 2 노드
#
# 기능:
#   1. /odom 구독 → predict + update_odometry
#   2. /imu/data 구독 → update_imu
#   3. /odometry/filtered 발행 (퓨전된 상태!)
#   4. TF 브로드캐스팅 (odom → base_footprint)
#   5. /ekf/diagnostics 발행 (디버깅!)
#   6. ROS 파라미터로 노이즈 값 동적 설정
#
# 입력 토픽:
#   /odom           nav_msgs/Odometry    (50Hz, diff_drive)
#   /imu/data       sensor_msgs/Imu      (100Hz, BNO055)
#
# 출력 토픽:
#   /odometry/filtered  nav_msgs/Odometry  (odom 주기에 동기)
#   /ekf/diagnostics    diagnostic_msgs/DiagnosticArray
#   /tf                 odom → base_footprint
# ════════════════════════════════════════════════════════════════

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import (
    TransformStamped, Quaternion, PoseWithCovariance, TwistWithCovariance
)
from std_msgs.msg import String

import tf2_ros

import numpy as np
import math

# Day 65 EKF 엔진
from my_robot_control.ekf_core import DiffDriveEKF, normalize_angle


class EKFNode(Node):
    """
    차동 구동 로봇 EKF ROS 2 노드.

    /odom + /imu/data → 센서 퓨전 → /odometry/filtered + TF
    """

    def __init__(self):
        super().__init__('ekf_node')

        # ════════════════════════════════════════
        # ROS 파라미터 선언 + 읽기
        # ════════════════════════════════════════
        self._declare_parameters()
        self._read_parameters()

        # ════════════════════════════════════════
        # EKF 엔진 초기화 (Day 65)
        # ════════════════════════════════════════
        self.ekf = DiffDriveEKF(
            initial_state=np.zeros(3),
            initial_covariance=np.diag([
                self.initial_cov_x,
                self.initial_cov_y,
                self.initial_cov_theta
            ]),
            motion_noise_alpha=self.motion_alpha,
            odom_noise=np.array([
                self.odom_noise_x,
                self.odom_noise_y,
                self.odom_noise_theta
            ]),
            imu_noise=self.imu_noise_theta,
        )

        # ════════════════════════════════════════
        # 시간 추적
        # ════════════════════════════════════════
        self.last_odom_time = None
        # 첫 오도메트리의 타임스탬프를 기록하여
        # 이후 dt = current - last를 계산!

        self.odom_received = False
        self.imu_received = False

        # ════════════════════════════════════════
        # 구독자 (Subscribers)
        # ════════════════════════════════════════
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        # 오도메트리: Reliable (손실 방지!)
        # diff_drive 플러그인은 Reliable로 발행

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            odom_qos
        )

        imu_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # IMU: Best Effort
        # → 고주파(100Hz)에서는 1~2개 손실 허용!
        # → Reliable이면 지연 발생 가능!

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            imu_qos
        )

        # ════════════════════════════════════════
        # 발행자 (Publishers)
        # ════════════════════════════════════════
        self.filtered_pub = self.create_publisher(
            Odometry,
            '/odometry/filtered_set',
            10
        )

        self.diag_pub = self.create_publisher(
            String,
            '/ekf/diagnostics',
            10
        )

        # ════════════════════════════════════════
        # TF 브로드캐스터
        # ════════════════════════════════════════
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # Dynamic TF: odom → base_footprint
        # → 매 odom 콜백마다 갱신!

        # ════════════════════════════════════════
        # 프레임 ID
        # ════════════════════════════════════════
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # ════════════════════════════════════════
        # 디버깅 카운터
        # ════════════════════════════════════════
        self.odom_count = 0
        self.imu_count = 0
        self.publish_count = 0

        self.get_logger().info(
            f'[EKF] Initialized. '
            f'Waiting for /odom and /imu/data...'
        )
        self.get_logger().info(
            f'[EKF] Frames: {self.odom_frame} → {self.base_frame}'
        )

    # ═══════════════════════════════════════════════════════
    #  파라미터 관리
    # ═══════════════════════════════════════════════════════

    def _declare_parameters(self):
        """ROS 파라미터 선언."""

        # 초기 공분산
        self.declare_parameter('initial_cov_x', 0.05)
        self.declare_parameter('initial_cov_y', 0.05)
        self.declare_parameter('initial_cov_theta', 0.02)

        # Motion Noise (Thrun α 파라미터)
        self.declare_parameter('alpha1', 0.01)
        self.declare_parameter('alpha2', 0.005)
        self.declare_parameter('alpha3', 0.005)
        self.declare_parameter('alpha4', 0.01)

        # Odometry Measurement Noise
        self.declare_parameter('odom_noise_x', 0.0005)
        self.declare_parameter('odom_noise_y', 0.0005)
        self.declare_parameter('odom_noise_theta', 0.002)

        # IMU Measurement Noise
        self.declare_parameter('imu_noise_theta', 2e-4)

        # 프레임 ID
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        # use_sim_time은 launch에서 설정!

    def _read_parameters(self):
        """파라미터 읽기."""
        self.initial_cov_x = self.get_parameter('initial_cov_x').value
        self.initial_cov_y = self.get_parameter('initial_cov_y').value
        self.initial_cov_theta = self.get_parameter('initial_cov_theta').value

        self.motion_alpha = [
            self.get_parameter('alpha1').value,
            self.get_parameter('alpha2').value,
            self.get_parameter('alpha3').value,
            self.get_parameter('alpha4').value,
        ]

        self.odom_noise_x = self.get_parameter('odom_noise_x').value
        self.odom_noise_y = self.get_parameter('odom_noise_y').value
        self.odom_noise_theta = self.get_parameter('odom_noise_theta').value

        self.imu_noise_theta = self.get_parameter('imu_noise_theta').value

    # ═══════════════════════════════════════════════════════
    #  콜백: 오도메트리
    # ═══════════════════════════════════════════════════════

    def odom_callback(self, msg: Odometry):
        """
        /odom 콜백 — EKF의 메인 루프!

        처리 순서:
          ① dt 계산 (이전 odom 이후 경과 시간)
          ② 속도 추출 (v, ω)
          ③ EKF predict(v, ω, dt)
          ④ 오도메트리 위치 추출
          ⑤ EKF update_odometry(z)
          ⑥ /odometry/filtered 발행
          ⑦ TF 브로드캐스팅
        """
        self.odom_count += 1

        # ────────────────────────────────────
        # ① dt 계산
        # ────────────────────────────────────
        current_time = Time.from_msg(msg.header.stamp)
        # use_sim_time=true이면 Gazebo 시뮬 시간!
        # use_sim_time=false이면 시스템 시간!

        if self.last_odom_time is None:
            self.last_odom_time = current_time
            self.odom_received = True
            self.get_logger().info(
                '[EKF] First odom received. EKF started!'
            )
            return
            # 첫 메시지: dt 계산 불가 → 기준 시간만 설정!

        dt = (current_time - self.last_odom_time).nanoseconds * 1e-9
        # 나노초 → 초 변환
        # 50Hz: dt ≈ 0.02s

        self.last_odom_time = current_time

        if dt <= 0.0 or dt > 1.0:
            # dt가 0 이하: 시간 역전 (시뮬 리셋?)
            # dt > 1초: 토픽 드롭 또는 일시정지
            # → 이상 상황 → 스킵!
            self.get_logger().warn(
                f'[EKF] Invalid dt: {dt:.4f}s. Skipping.'
            )
            return

        # ────────────────────────────────────
        # ② 속도 추출
        # ────────────────────────────────────
        v = msg.twist.twist.linear.x
        # 로봇 전방 선속도 [m/s]
        # twist는 base_footprint 프레임!

        omega = msg.twist.twist.angular.z
        # Z축 각속도 [rad/s]
        # 반시계 = 양수 (REP-103)

        # ────────────────────────────────────
        # ③ EKF Predict
        # ────────────────────────────────────
        self.ekf.predict(v, omega, dt)

        # ────────────────────────────────────
        # ④ 오도메트리 위치 추출
        # ────────────────────────────────────
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        odom_theta = self._quaternion_to_yaw(msg.pose.pose.orientation)

        z_odom = np.array([odom_x, odom_y, odom_theta])

        # ────────────────────────────────────
        # ⑤ EKF Update (Odometry)
        # ────────────────────────────────────
        self.ekf.update_odometry(z_odom)

        # ────────────────────────────────────
        # ⑥ 결과 발행
        # ────────────────────────────────────
        self._publish_filtered_odom(msg.header.stamp, v, omega)

        # ────────────────────────────────────
        # ⑦ TF 브로드캐스팅
        # ────────────────────────────────────
        self._broadcast_tf(msg.header.stamp)

        # ────────────────────────────────────
        # 디버깅 (100 메시지마다 = 2초)
        # ────────────────────────────────────
        if self.odom_count % 100 == 0:
            self._publish_diagnostics(dt)

    # ═══════════════════════════════════════════════════════
    #  콜백: IMU
    # ═══════════════════════════════════════════════════════

    def imu_callback(self, msg: Imu):
        """
        /imu/data 콜백 — Yaw 보정!

        predict는 하지 않음! (odom_callback에서만!)
        update_imu만 수행!

        왜?
          → predict는 "시간 전파" — odom의 속도 정보 필요!
          → IMU는 속도 정보가 없음 (각속도는 있지만
            선속도가 없으므로 완전한 predict 불가)
          → IMU는 "추가 관측"으로만 사용!
        """
        if not self.odom_received:
            return
            # odom이 아직 안 왔으면 IMU도 무시!
            # → EKF가 초기화되지 않은 상태에서 update 방지!

        self.imu_count += 1

        # ────────────────────────────────────
        # Quaternion → Yaw 추출
        # ────────────────────────────────────
        theta_imu = self._quaternion_to_yaw(msg.orientation)

        # ────────────────────────────────────
        # Quaternion 유효성 검사
        # ────────────────────────────────────
        q = msg.orientation
        norm_sq = q.x**2 + q.y**2 + q.z**2 + q.w**2
        if abs(norm_sq - 1.0) > 0.1:
            self.get_logger().warn(
                f'[EKF] Invalid IMU quaternion! |q|²={norm_sq:.4f}'
            )
            return

        # ────────────────────────────────────
        # orientation_covariance 확인
        # ────────────────────────────────────
        if msg.orientation_covariance[0] < 0:
            # covariance[0] = -1 → "orientation 제공 안 함!"
            # → MPU6050 같은 Raw IMU → 스킵!
            return

        # ────────────────────────────────────
        # EKF Update (IMU)
        # ────────────────────────────────────
        self.ekf.update_imu(theta_imu)

        if not self.imu_received:
            self.imu_received = True
            self.get_logger().info(
                '[EKF] First IMU received. Yaw fusion active!'
            )

    # ═══════════════════════════════════════════════════════
    #  발행: /odometry/filtered
    # ═══════════════════════════════════════════════════════

    def _publish_filtered_odom(self, stamp, v, omega):
        """
        퓨전된 오도메트리를 nav_msgs/Odometry로 발행.

        nav_msgs/Odometry 구조:
          header.stamp: 타임스탬프
          header.frame_id: odom (부모 프레임)
          child_frame_id: base_footprint (자식 프레임)
          pose.pose: 위치 + 자세
          pose.covariance: 36개 float (6×6, row-major)
          twist.twist: 선속도 + 각속도
          twist.covariance: 36개 float (6×6)
        """
        state = self.ekf.get_state()
        P = self.ekf.get_covariance()

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        # ── Pose ──
        msg.pose.pose.position.x = state[0]
        msg.pose.pose.position.y = state[1]
        msg.pose.pose.position.z = 0.0
        # 2D 로봇 → z = 0!

        msg.pose.pose.orientation = self._yaw_to_quaternion(state[2])
        # EKF 상태의 θ → Quaternion 변환

        # ── Pose Covariance (6×6) ──
        # ROS Odometry의 covariance는 6×6!
        # [x, y, z, roll, pitch, yaw]
        # 우리 EKF는 3×3 [x, y, θ] → 6×6로 매핑!
        cov_36 = [0.0] * 36
        #
        # 인덱스 매핑 (6×6 row-major):
        #   (0,0)=0: x-x    (0,5)=5: x-yaw
        #   (1,1)=7: y-y    (1,5)=11: y-yaw
        #   (5,5)=35: yaw-yaw
        #   (5,0)=30: yaw-x  (5,1)=31: yaw-y
        #
        cov_36[0] = P[0, 0]      # σ²_x
        cov_36[1] = P[0, 1]      # σ_xy
        cov_36[5] = P[0, 2]      # σ_xθ → x-yaw
        cov_36[6] = P[1, 0]      # σ_yx
        cov_36[7] = P[1, 1]      # σ²_y
        cov_36[11] = P[1, 2]     # σ_yθ → y-yaw
        cov_36[30] = P[2, 0]     # σ_θx → yaw-x
        cov_36[31] = P[2, 1]     # σ_θy → yaw-y
        cov_36[35] = P[2, 2]     # σ²_θ → yaw-yaw

        # z, roll, pitch는 관측하지 않으므로 큰 분산!
        cov_36[14] = 1e6         # z: "모른다!" (2D 로봇)
        cov_36[21] = 1e6         # roll: "모른다!"
        cov_36[28] = 1e6         # pitch: "모른다!"
        # 1e6 = "이 값은 완전히 불확실" 신호!
        # Nav2, SLAM이 이 값을 보고 z/roll/pitch를 무시!

        msg.pose.covariance = cov_36

        # ── Twist ──
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = omega
        # 속도는 EKF 상태에 없으므로 입력 그대로 전달!
        # (robot_localization은 속도도 추정 — Day 67)

        # Twist Covariance (간단 설정)
        twist_cov = [0.0] * 36
        twist_cov[0] = 0.01     # σ²_vx
        twist_cov[35] = 0.01    # σ²_ωz
        twist_cov[7] = 1e6      # vy (차동 구동에서 0)
        twist_cov[14] = 1e6     # vz
        twist_cov[21] = 1e6     # ω_roll
        twist_cov[28] = 1e6     # ω_pitch
        msg.twist.covariance = twist_cov

        self.filtered_pub.publish(msg)
        self.publish_count += 1

    # ═══════════════════════════════════════════════════════
    #  TF 브로드캐스팅
    # ═══════════════════════════════════════════════════════

    def _broadcast_tf(self, stamp):
        """
        odom → base_footprint TF 발행.

        왜 EKF가 TF를 발행?
          → 기존: diff_drive 플러그인이 odom → base_footprint TF 발행
          → 문제: diff_drive의 오도메트리는 노이즈 포함!
          → 해결: EKF가 퓨전된 위치로 TF를 덮어씀!

        ⚠️ 충돌 방지:
          → diff_drive의 publish_odom_tf=false로 설정!
          → 또는 EKF 노드에서만 TF 발행!
          → 두 노드가 같은 TF를 발행하면 → 진동!
        """
        state = self.ekf.get_state()

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        # 부모 프레임: odom

        t.child_frame_id = self.base_frame
        # 자식 프레임: base_footprint

        t.transform.translation.x = state[0]
        t.transform.translation.y = state[1]
        t.transform.translation.z = 0.0

        t.transform.rotation = self._yaw_to_quaternion(state[2])

        self.tf_broadcaster.sendTransform(t)

    # ═══════════════════════════════════════════════════════
    #  진단 발행
    # ═══════════════════════════════════════════════════════

    def _publish_diagnostics(self, dt):
        """디버깅 정보 발행."""
        '''
        state = self.ekf.get_state()
        unc = self.ekf.get_uncertainty()
        diag = self.ekf.get_diagnostics()

        
        info = (
            f'[EKF #{self.odom_count}] '
            f'dt={dt*1000:.1f}ms | '
            f'x={state[0]:+.3f} y={state[1]:+.3f} '
            f'θ={math.degrees(state[2]):+.1f}° | '
            f'σx={unc["sigma_x"]:.3f}m '
            f'σy={unc["sigma_y"]:.3f}m '
            f'σθ={unc["sigma_theta_deg"]:.1f}° | '
            f'odom:{self.odom_count} imu:{self.imu_count}'
        )

        self.get_logger().info(info)

        msg = String()
        msg.data = info
        self.diag_pub.publish(msg)
        '''

    # ═══════════════════════════════════════════════════════
    #  유틸리티
    # ═══════════════════════════════════════════════════════

    @staticmethod
    def _quaternion_to_yaw(q) -> float:
        """
        geometry_msgs/Quaternion → Yaw [rad].

        Day 52에서 유도한 공식:
          yaw = atan2(2(wz + xy), 1 - 2(y² + z²))
        """
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        """
        Yaw [rad] → geometry_msgs/Quaternion.

        2D 전용 (Roll=Pitch=0):
          x = 0, y = 0
          z = sin(yaw/2)
          w = cos(yaw/2)
        """
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f'[EKF] Shutdown. '
            f'Total: odom={node.odom_count}, imu={node.imu_count}, '
            f'published={node.publish_count}'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()