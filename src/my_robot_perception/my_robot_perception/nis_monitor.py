#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# nis_monitor.py
# Day 68 — NIS (Normalized Innovation Squared) 실시간 모니터
#
# EKF의 tuning 상태를 정량적으로 평가!
# NIS가 χ² 분포를 따르면 → 잘 튜닝됨!
# ════════════════════════════════════════════════════════════════

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import numpy as np
import math


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class NISMonitor(Node):
    """
    NIS 모니터: EKF 출력과 센서 측정을 비교하여 NIS 계산.

    ★ 이것은 EKF 내부가 아니라 외부에서 관찰하는 도구!
    → EKF의 "건강 상태"를 진단!
    """

    def __init__(self):
        super().__init__('nis_monitor')

        # EKF 출력 구독
        self.ekf_sub = self.create_subscription(
            Odometry, '/odometry/filtered',
            self.ekf_callback, 10
        )

        # 센서 측정 구독
        self.odom_sub = self.create_subscription(
            Odometry, '/odom',
            self.odom_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data',
            self.imu_callback, 10
        )

        # NIS 발행 (시각화용)
        self.nis_odom_pub = self.create_publisher(
            Float64, '/ekf/nis_odom', 10
        )
        self.nis_imu_pub = self.create_publisher(
            Float64, '/ekf/nis_imu', 10
        )

        # 최신 EKF 상태
        self.ekf_state = None
        self.ekf_cov = None

        # NIS 히스토리 (통계용)
        self.nis_odom_history = []
        self.nis_imu_history = []

        # 타이머 (10초마다 보고)
        self.timer = self.create_timer(10.0, self.report)

        # χ² 임계값
        self.chi2_3_95 = 3.841   # 3 DOF, 95%
        self.chi2_1_95 = 5.991   # 1 DOF, 95%

        self.get_logger().info('[NIS] Monitor started.')



    def ekf_callback(self, msg):
        """EKF 상태 저장."""
        self.ekf_state = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            quaternion_to_yaw(msg.pose.pose.orientation)
        ])

        # 6×6 → 3×3 추출
        c = msg.pose.covariance
        self.ekf_cov = np.array([
            [c[0],  c[1],  c[5]],
            [c[6],  c[7],  c[11]],
            [c[30], c[31], c[35]]
        ])



    def odom_callback(self, msg):
        """오도메트리 NIS 계산."""
        if self.ekf_state is None:
            return

        z = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            quaternion_to_yaw(msg.pose.pose.orientation)
        ])

        # 오도메트리 R (메시지의 covariance에서 추출)
        c = msg.pose.covariance
        R = np.array([
            [c[0],  c[1],  c[5]],
            [c[6],  c[7],  c[11]],
            [c[30], c[31], c[35]]
        ])
        # 0이면 기본값 사용
        if R[0, 0] == 0:
            R = np.diag([0.001, 0.001, 0.005])

        # Innovation
        nu = z - self.ekf_state
        nu[2] = normalize_angle(nu[2])

        # S = P + R (H=I)
        S = self.ekf_cov + R

        # NIS = νᵀ S⁻¹ ν
        try:
            S_inv = np.linalg.inv(S)
            nis = float(nu.T @ S_inv @ nu)
        except np.linalg.LinAlgError:
            return

        self.nis_odom_history.append(nis)

        # 발행
        msg_out = Float64()
        msg_out.data = nis
        self.nis_odom_pub.publish(msg_out)



    def imu_callback(self, msg):
        """IMU Yaw NIS 계산."""
        if self.ekf_state is None:
            return

        theta_imu = quaternion_to_yaw(msg.orientation)

        # R
        R_yaw = msg.orientation_covariance[8]
        if R_yaw <= 0:
            R_yaw = 0.01

        # Innovation
        nu = normalize_angle(theta_imu - self.ekf_state[2])

        # S = P[2,2] + R
        S = self.ekf_cov[2, 2] + R_yaw

        # NIS = ν² / S
        nis = nu ** 2 / S if S > 1e-12 else 0.0

        self.nis_imu_history.append(nis)

        msg_out = Float64()
        msg_out.data = nis
        self.nis_imu_pub.publish(msg_out)



    def report(self):
        """10초마다 NIS 통계 보고."""
        if len(self.nis_odom_history) < 3:
            return

        odom_nis = np.array(self.nis_odom_history[-500:])
        imu_nis = np.array(self.nis_imu_history[-500:])

        odom_mean = np.mean(odom_nis)
        odom_exceed = np.sum(odom_nis > self.chi2_3_95) / len(odom_nis) * 100

        imu_mean = np.mean(imu_nis) if len(imu_nis) > 0 else 0
        imu_exceed = (np.sum(imu_nis > self.chi2_1_95) / len(imu_nis) * 100
                      if len(imu_nis) > 0 else 0)

        # 진단
        odom_status = self._diagnose(odom_mean, 1.0, odom_exceed)
        imu_status = self._diagnose(imu_mean, 2.0, imu_exceed)

        self.get_logger().info(
            f'[NIS Report]\n'
            f'  Odom: mean={odom_mean:.2f} (ideal=1.0) | '
            f'exceed 95%: {odom_exceed:.1f}% | {odom_status}\n'
            f'  IMU:  mean={imu_mean:.2f} (ideal=2.0) | '
            f'exceed 95%: {imu_exceed:.1f}% | {imu_status}'
        )

    @staticmethod
    def _diagnose(mean, ideal, exceed_pct):
        """NIS 기반 진단."""
        if exceed_pct > 20:
            return '⚠️ UNDER-CONFIDENT! → R 증가 or Q 증가!'
        elif mean < ideal * 0.3:
            return '⚠️ OVER-CONFIDENT! → R 감소 or Q 감소!'
        elif 0.5 * ideal <= mean <= 2.0 * ideal and exceed_pct < 10:
            return '✅ WELL TUNED!'
        else:
            return '🔧 NEEDS ADJUSTMENT'





def main(args=None):
    rclpy.init(args=args)
    node = NISMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()