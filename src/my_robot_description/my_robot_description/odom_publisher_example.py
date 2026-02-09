"""
    실제 로봇에서 오도메트리를 발행하는 패턴.
        → 엔코더 데이터로 위치 추정
        → odom → base_footprint TF 발행

    ⚠️ 시뮬에서는 diff_drive 플러그인이 이걸 해줌
        실제 로봇에서는 직접 구현 필요
        Day 38-40에서 자세히 다룹니다. 여기서는 TF 발행 패턴만 이해!
"""

import rclpy
from rclpy.node import Node
import math

from tf2_ros import TransformBroadcaster
# TransformBroadcaster: /tf에 Dynamic TF를 발행하는 클래스
    # StaticTransformBroadcaster와 달리 매 프레임마다 발행해야 함

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler


class OdomPublisher(Node):
    """
        간단한 원운동 오도메트리 시뮬레이터.
        실제로는 엔코더 데이터를 읽어서 계산하지만 여기서는 TF 발행 패턴을 보여주기 위해 가상의 원운동을 생성
    """
    
    def __init__(self):
        super().__init__('odom_publisher')
        
        # ---- TF Broadcaster ----
        self.tf_broadcaster = TransformBroadcaster(self)
            # Dynamic TF 발행자 → /tf 토픽에 발행
        
        # ---- Odometry Publisher ----
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
            # nav_msgs/Odometry 메시지도 함께 발행
            # → TF만으로는 속도 정보를 전달할 수 없으므로 별도 토픽으로 속도(twist) 정보 발행!
            # → robot_localization (EKF)이 이 토픽도 사용
        
        # ---- 상태 변수 ----
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.time = 0.0
        
        # ---- 타이머 (50Hz) ----
        self.timer = self.create_timer(0.02, self.update)
        # 50Hz = 20ms 주기
        # → diff_drive 플러그인의 update_rate와 동일!
    

    def update(self):
        """
            50Hz로 호출: 위치 업데이트 + TF 발행
        """
        
        dt = 0.02  # 20ms
        self.time += dt
        
        # 가상 원운동 (반지름 1m, 속도 0.3 m/s)
        v = 0.3        # 선속도 (m/s)
        omega = 0.3    # 각속도 (rad/s)
        
        # 위치 업데이트 (Euler 적분)
        self.theta += omega * dt
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        

        # TF 발행: odom → base_footprint
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
            # ⭐ 현재 시간!
                # → 다른 노드가 이 시간으로 TF를 조회!
                # → 시간이 틀리면 센서 데이터와 TF가 불일치!
        
        # 부모: odom (출발점)
        t.header.frame_id = 'odom'
        # 자식: base_footprint (로봇)
        t.child_frame_id = 'base_footprint'
        # Z = 0: 2D 주행이므로 높이 변화 없음
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        
        q = quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)
        # /tf에 발행! → 50Hz로 계속 갱신!
        

        # Odometry 메시지 발행        
        odom_msg = Odometry()
        odom_msg.header.stamp = t.header.stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        # 위치 (Pose)
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        # 속도 (Twist) — base_footprint 기준!
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = omega
        
        self.odom_pub.publish(odom_msg)


def main():
    rclpy.init()
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()