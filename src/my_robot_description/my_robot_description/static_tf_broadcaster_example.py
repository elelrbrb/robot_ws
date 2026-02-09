"""
    Static TF를 코드에서 직접 발행
        URDF에 포함되지 않은 추가 Static TF를 발행할 때 사용
            예: 외부 카메라, 추가 센서, 기준 마커 등

        ⚠️ 우리 로봇의 센서 TF는 URDF/Xacro에 정의되어 있어서 robot_state_publisher가 자동으로 발행
            이 예제는 "추가적인" Static TF가 필요할 때 참고
"""

import rclpy
from rclpy.node import Node

from tf2_ros import StaticTransformBroadcaster
    # StaticTransformBroadcaster: /tf_static에 변환을 발행하는 클래스
        # TransformBroadcaster와 비슷하지만 Transient Local QoS 사용

from geometry_msgs.msg import TransformStamped
    # TransformStamped: TF 메시지 타입
        # header (stamp, frame_id) + child_frame_id + transform

from tf_transformations import quaternion_from_euler
    # RPY → Quaternion 변환 유틸리티
        # pip install tf-transformations (ROS 2 Humble)



class StaticTFExample(Node):
    """
        외부 기준 마커와 로봇 사이의 고정 변환을 발행하는 노드

        실험실 벽에 ArUco 마커를 붙이고 그 마커의 위치를 map에 대해 고정으로 등록
    """
    
    def __init__(self):
        super().__init__('static_tf_example')
        
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        # StaticTransformBroadcaster 생성 → /tf_static 퍼블리셔 내부 생성
    
        self.make_transforms()
        # 노드 생성 시 한번만 호출 → Static이니까 한번이면 충분
    

    
    def make_transforms(self):
        """
            Static TF를 생성하고 발행
        """
        
        t = TransformStamped()
            # 빈 TransformStamped 메시지 생성
        

        # ---- Header ----
        t.header.stamp = self.get_clock().now().to_msg()
            # 현재 시간
            # Static TF는 시간에 무관하지만 stamp는 필수 필드
        
        t.header.frame_id = 'map'
            # 부모 프레임: map
        
        t.child_frame_id = 'aruco_marker_0'
            # 자식 프레임: ArUco 마커 #0
        

        # ---- Translation ----
        t.transform.translation.x = 3.0    # 전방 3m
        t.transform.translation.y = 1.5    # 왼쪽 1.5m
        t.transform.translation.z = 1.2    # 높이 1.2m
        
        # ---- Rotation ----
        q = quaternion_from_euler(0, 0, 1.5708)
            # RPY = (0, 0, π/2) → Z축 90° 회전 (마커가 왼쪽 벽에 붙어있어서 90° 회전)
        
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        

        # ---- 발행! ----
        self.tf_static_broadcaster.sendTransform(t)
            # sendTransform: /tf_static에 발행
            # 한번 발행하면 Transient Local QoS로 유지됨

        
        self.get_logger().info(
            f'Published static TF: {t.header.frame_id} → {t.child_frame_id}'
        )



def main():
    rclpy.init()
    node = StaticTFExample()
    try:
        rclpy.spin(node)
        # spin: 노드를 계속 실행
        # Static TF는 한번 발행이지만 노드가 살아있어야 다른 노드가 조회 가능
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()