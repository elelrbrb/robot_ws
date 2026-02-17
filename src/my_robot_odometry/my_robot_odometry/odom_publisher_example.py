#!/usr/bin/env python3
"""
——————————————————————————————————————————
ROS 2 오도메트리 노드
    기능:
        1. /joint_states 구독 → 엔코더 position 읽기
        2. 오도메트리 계산 (DifferentialDriveOdometry 클래스)
        3. /odom 토픽 발행 (nav_msgs/Odometry)
        4. odom → base_footprint TF 브로드캐스트
        5. /odom_path 발행 (RViz 궤적 시각화)
——————————————————————————————————————————
"""

import math
import rclpy
    # ROS 2 Python 클라이언트 라이브러리
from rclpy.node import Node
    # 모든 ROS 2 노드의 부모 클래스

from sensor_msgs.msg import JointState
    # /joint_states 메시지 타입
    # Gazebo diff_drive 플러그인이 바퀴 Joint의 position을 여기에 발행
from nav_msgs.msg import Odometry, Path
    # Odometry: 우리가 발행할 메시지
    # Path: RViz에서 궤적을 그리기 위한 메시지
from geometry_msgs.msg import (
    TransformStamped,
    PoseStamped
)
    # TransformStamped: TF 브로드캐스트용
    # PoseStamped: Path에 넣을 개별 포즈
from tf2_ros import TransformBroadcaster
    # odom → base_footprint TF를 발행하는 도구

import time


# ================================================================
# 오도메트리 계산 클래스 (Section A에서 만든 것을 그대로 사용!)
# ================================================================
class DifferentialDriveOdometry:
    """
    차동 구동 오도메트리 계산기 -> Section A의 코드와 동일. 노드와 분리하여 재사용성 확보.
    """

    def __init__(
        self,
        wheel_radius: float = 0.0325,
        wheel_separation: float = 0.345,
        ticks_per_rev: int = 2244
    ) -> None:
        self._wheel_radius = wheel_radius
        self._wheel_separation = wheel_separation
        self._ticks_per_rev = ticks_per_rev
        self._meters_per_tick = (2.0 * math.pi * wheel_radius) / ticks_per_rev

        self._x: float = 0.0
        self._y: float = 0.0
        self._theta: float = 0.0
        self._v: float = 0.0
        self._omega: float = 0.0



    def update_from_radians(
        self,
        delta_rad_left: float,
        delta_rad_right: float,
        dt: float
    ) -> tuple[float, float, float]:
        
        """
        엔코더 각도 변화(rad)로부터 오도메트리를 갱신
        
        Gazebo의 /joint_states는 position을 라디안(누적 회전 각도)으로 제공
        따라서 ticks가 아닌 radians로 직접 계산하는 메서드가 필요

        변환: Δdistance = Δrad × wheel_radius
            (틱 기반: Δdistance = Δticks × meters_per_tick와 동치)
        """

        # 좌/우 바퀴 이동 거리 [m]
        dl: float = delta_rad_left * self._wheel_radius
            # Δrad × r = 호의 길이 (호도법의 정의!)
            # 예: Δrad = 1.0 → dl = 1.0 × 0.0325 = 0.0325 m
        dr: float = delta_rad_right * self._wheel_radius

        # 중심 이동 거리, 방향 변화
        dc: float = (dr + dl) / 2.0
        dtheta: float = (dr - dl) / self._wheel_separation

        # Runge-Kutta 2차 적분
        theta_mid: float = self._theta + dtheta / 2.0
        self._x += dc * math.cos(theta_mid)
        self._y += dc * math.sin(theta_mid)
        self._theta += dtheta

        # 각도 래핑
        self._theta = math.atan2(
            math.sin(self._theta),
            math.cos(self._theta)
        )

        # 속도 계산
        if dt > 1e-6:
            self._v = dc / dt
            self._omega = dtheta / dt
        else:
            self._v = 0.0
            self._omega = 0.0

        return (self._x, self._y, self._theta)


    def get_pose(self) -> tuple[float, float, float]:
        return (self._x, self._y, self._theta)


    def get_velocity(self) -> tuple[float, float]:
        return (self._v, self._omega)


    def reset(self) -> None:
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._v = 0.0
        self._omega = 0.0



# ================================================================
# ROS 2 노드
# ================================================================
class OdometryPublisherNode(Node):
    """
    ROS 2 오도메트리 발행 노드.
        역할:
            1. Gazebo diff_drive의 /joint_states에서 바퀴 position(rad) 구독
            2. DifferentialDriveOdometry로 (x, y, θ, v, ω) 계산
            3. /odom (nav_msgs/Odometry) 발행
            4. odom → base_footprint TF 브로드캐스트
            5. /odom_path (nav_msgs/Path) 발행 (RViz 궤적 시각화)
    """

    def __init__(self) -> None:
        super().__init__('odom_publisher_node')

        # ---- 파라미터 선언 (외부에서 변경 가능!) ----
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_separation', 0.345)
        self.declare_parameter('ticks_per_rev', 2244)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', False)
            # publish_tf를 false로 하면 TF를 발행하지 않음
            # → diff_drive 플러그인의 TF와 충돌 방지!
            # → 또는 robot_localization이 TF를 발행할 때

        # 파라미터 읽기
        wr = self.get_parameter('wheel_radius').value
        ws = self.get_parameter('wheel_separation').value
        tpr = self.get_parameter('ticks_per_rev').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._publish_tf = self.get_parameter('publish_tf').value


        # ---- 오도메트리 계산기 인스턴스 ----
        self._odom_calc = DifferentialDriveOdometry(
            wheel_radius=wr,
            wheel_separation=ws,
            ticks_per_rev=tpr
        )


        # ---- 이전 Joint Position 저장 (Δ 계산용) ----
        self._prev_left_pos: float | None = None
            # None: 아직 첫 메시지를 받지 않았음
            # → 첫 메시지에서는 Δ를 계산할 수 없으므로 초기화만!
        self._prev_right_pos: float | None = None
        self._prev_time = None
            # 이전 메시지의 타임스탬프 → dt 계산용


        # ---- Joint 이름 → 인덱스 매핑 ----
        self._left_joint_name = 'left_wheel_joint'
        self._right_joint_name = 'right_wheel_joint'
            # URDF에서 정의한 Joint 이름과 정확히 일치해야!
            # 틀리면 → 엔코더 데이터를 못 찾음 → 오도메트리 = 0


        # ---- 구독자 ----
        self._joint_sub = self.create_subscription(
            JointState,                     # 메시지 타입
            '/joint_states',                # 토픽 이름
            self._joint_state_callback,     # 콜백 함수
            10                              # QoS 큐 크기
        )
            # Gazebo diff_drive 플러그인이 /joint_states를 50Hz로 발행
            # 매 메시지마다 _joint_state_callback이 호출됨

        # ---- 발행자 ----
        self._odom_pub = self.create_publisher(
            Odometry,       # 메시지 타입
            '/odom',        # 토픽 이름 (ROS 2 규약)
            10              # QoS
        )

        self._path_pub = self.create_publisher(
            Path,
            '/odom_path',   # RViz에서 궤적을 그릴 토픽
            10
        )


        # ---- TF 브로드캐스터 ----
        self._tf_broadcaster = TransformBroadcaster(self)
            # odom → base_footprint 변환을 /tf에 발행


        # ---- Path 메시지 (궤적 누적) ----
        self._path_msg = Path()
        self._path_msg.header.frame_id = self._odom_frame
            # 궤적의 좌표계 = "odom"

        self.get_logger().info(
            f'[OdometryNode] Started. '
            f'wheel_r={wr}, wheel_sep={ws}, tpr={tpr}'
        )



    # ================================================================
    # 콜백: /joint_states 수신 시 호출
    # ================================================================
    def _joint_state_callback(self, msg: JointState) -> None:
        """
        /joint_states 메시지가 올 때마다 호출.
        바퀴 Joint의 position(rad) 변화량으로 오도메트리를 업데이트

        JointState 메시지 구조:
            name: ['left_wheel_joint', 'right_wheel_joint']
            position: [2.345, 2.567]  ← 누적 회전 각도 (rad)
            velocity: [1.23, 1.45]    ← 각속도 (rad/s)
        """

        # ---- Step 1: Joint 이름으로 인덱스 찾기 ----
        try:
            left_idx = msg.name.index(self._left_joint_name)
            right_idx = msg.name.index(self._right_joint_name)
        except ValueError:
            # Joint 이름이 메시지에 없으면 → 경고 후 무시
            self.get_logger().warn(
                f'Joint names not found in /joint_states! '
                f'Expected: {self._left_joint_name}, {self._right_joint_name}. '
                f'Got: {msg.name}',
                throttle_duration_sec=5.0
                    # 5초에 1번만 경고 출력 → 로그 폭주 방지!
            )
            return

        # ---- Step 2: 현재 position 읽기 ----
        current_left_pos: float = msg.position[left_idx]
            # 좌바퀴 누적 회전 각도 [rad]
            # 예: 바퀴가 2바퀴 돌았으면 → 4π ≈ 12.566 rad
        current_right_pos: float = msg.position[right_idx]


        # ---- Step 3: 타임스탬프 ----
        current_time = msg.header.stamp
            # ROS 2 Time 객체 (sec + nanosec)

        # ---- Step 4: 첫 메시지 처리 ----
        if self._prev_left_pos is None:
            # 첫 메시지 → Δ를 계산할 수 없으므로 초기값만 저장
            self._prev_left_pos = current_left_pos
            self._prev_right_pos = current_right_pos
            self._prev_time = current_time
            self.get_logger().info(
                '[OdometryNode] First joint_state received. Initialized.'
            )
            return


        # ---- Step 5: Δ(변화량) 계산 ----
        delta_left: float = current_left_pos - self._prev_left_pos
            # 좌바퀴 회전 변화량 [rad]
            # 양수 = 전진 방향 회전
        delta_right: float = current_right_pos - self._prev_right_pos

        # dt 계산 (초 단위)
        dt: float = (
            (current_time.sec - self._prev_time.sec) +
            (current_time.nanosec - self._prev_time.nanosec) * 1e-9
        )
            # sec 차이 + nanosec 차이를 합산
            # 50Hz라면 dt ≈ 0.02초

        if dt <= 0.0:
            # 타임스탬프가 뒤로 갔거나 동일 → 비정상 → 건너뜀
            self._prev_left_pos = current_left_pos
            self._prev_right_pos = current_right_pos
            self._prev_time = current_time
            return

        # ---- Step 6: 오도메트리 갱신 ----
        x, y, theta = self._odom_calc.update_from_radians(
            delta_left, delta_right, dt
        )
        v, omega = self._odom_calc.get_velocity()



        # ---- Step 7: Odometry 메시지 조립 + 발행 ----
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
            # ⚠️ 반드시 Joint 메시지의 타임스탬프를 사용!
            # self.get_clock().now()를 쓰면 → 약간의 지연이 포함
            # → TF와 불일치 → RViz 깜박임!
        odom_msg.header.frame_id = self._odom_frame    # "odom"
        odom_msg.child_frame_id = self._base_frame     # "base_footprint"

        # 위치 (글로벌 좌표)
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
            # 2D → Z = 0 (지면 위)

        # 방향 (Quaternion)
        qx, qy, qz, qw = yaw_to_quaternion(theta)
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Pose Covariance (6×6 = 36개, 행 우선)
        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.pose.covariance[0]  = 0.01    # σ²_x  [m²]
        odom_msg.pose.covariance[7]  = 0.01    # σ²_y  [m²]
        odom_msg.pose.covariance[14] = 1e6     # σ²_z  → "모름" (2D)
        odom_msg.pose.covariance[21] = 1e6     # σ²_roll → "모름"
        odom_msg.pose.covariance[28] = 1e6     # σ²_pitch → "모름"
        odom_msg.pose.covariance[35] = 0.005   # σ²_yaw [rad²]


        # 속도 (Body Frame!)
        odom_msg.twist.twist.linear.x = v
            # ⚠️ Body Frame! X = 전방 속도
        odom_msg.twist.twist.linear.y = 0.0
            # 차동 구동 → 옆으로 못 감 → 0
        odom_msg.twist.twist.angular.z = omega
            # Z축 회전 속도

        # Twist Covariance
        odom_msg.twist.covariance = [0.0] * 36
        odom_msg.twist.covariance[0]  = 0.005  # σ²_vx [m²/s²]
        odom_msg.twist.covariance[7]  = 1e6    # σ²_vy → "모름"
        odom_msg.twist.covariance[14] = 1e6    # σ²_vz → "모름"
        odom_msg.twist.covariance[21] = 1e6    # σ²_ωroll → "모름"
        odom_msg.twist.covariance[28] = 1e6    # σ²_ωpitch → "모름"
        odom_msg.twist.covariance[35] = 0.003  # σ²_ωyaw [rad²/s²]

        self._odom_pub.publish(odom_msg)



        # ---- Step 8: TF 브로드캐스트 ----
        if self._publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = current_time
                # ⚠️ Odometry와 동일한 타임스탬프 필수!
                # 다르면 → "TF extrapolation into the future" 에러!
            tf_msg.header.frame_id = self._odom_frame     # 부모: "odom"
            tf_msg.child_frame_id = self._base_frame      # 자식: "base_footprint"
            tf_msg.transform.translation.x = x
            tf_msg.transform.translation.y = y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(tf_msg)


        # ---- Step 9: Path 궤적 갱신 ----
        pose_stamped = PoseStamped()
        pose_stamped.header = odom_msg.header
        pose_stamped.pose = odom_msg.pose.pose
        self._path_msg.poses.append(pose_stamped)
            # 매 업데이트마다 현재 포즈를 Path에 추가
            # → RViz에서 궤적이 점점 길어짐!

        # 최대 경로 길이 제한 (메모리 방지)
        max_path_length = 5000
        if len(self._path_msg.poses) > max_path_length:
            self._path_msg.poses = self._path_msg.poses[-max_path_length:]
                # 오래된 포즈 삭제, 최근 5000개만 유지

        self._path_msg.header.stamp = current_time
        self._path_pub.publish(self._path_msg)


        # ---- Step 10: 이전 값 갱신 (다음 콜백을 위해) ----
        self._prev_left_pos = current_left_pos
        self._prev_right_pos = current_right_pos
        self._prev_time = current_time



# ================================================================
# Yaw ↔ Quaternion 변환 (Section B)
# ================================================================
def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))

def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)



# ================================================================
# 메인 함수
# ================================================================
def main(args=None):
    rclpy.init(args=args)
        # ROS 2 초기화 (DDS 통신 시작)

    node = OdometryPublisherNode()

    try:
        rclpy.spin(node)
            # 무한 루프: 콜백 대기 → 메시지 올 때마다 처리
            # Ctrl+C로 종료
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
            # DDS 통신 종료, 자원 해제


if __name__ == '__main__':
    main()