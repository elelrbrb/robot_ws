"""
    TF Listener — 다른 노드에서 TF 조회하기

    실제 사용 사례: YOLO가 카메라 이미지에서 물체를 감지
        → camera_optical_frame 기준으로 (x, y, z) 좌표 → base_link 기준으로 변환해서 로봇팔에 전달!
        이때 TF Listener가 필요!
"""

import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
    # Buffer: TF 데이터를 저장하는 버퍼
        # 최근 10초간의 모든 TF를 캐시 → 과거 시점의 변환도 조회 가능
    # TransformListener: /tf, /tf_static 토픽을 구독하고 받은 데이터를 Buffer에 저장 → 사용자는 Buffer에서 조회

from tf2_ros import TransformException
    # TF 조회 실패 시 발생하는 예외 → try-except로 처리!

from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
    # 좌표 변환 유틸리티 → PointStamped를 다른 프레임으로 변환!

from rclpy.duration import Duration
    # 시간 간격을 나타내는 클래스 → TF 조회 timeout에 사용



class TFListenerExample(Node):
    """
        LiDAR에서 감지한 장애물의 좌표를 base_link 기준으로 변환하는 예제 노드
    
        이 패턴은 로봇 소프트웨어에서 수없이 반복 → 센서 데이터(센서 프레임) → 로봇 프레임 변환
    """
    
    def __init__(self):
        super().__init__('tf_listener_example')
        
        # ---- TF Buffer + Listener 생성 ----
        
        self.tf_buffer = Buffer()
        # Buffer 생성
            # → 내부에 TF 데이터를 시간별로 저장
            # → 기본 캐시 시간: 10초
        
        self.tf_listener = TransformListener(self.tf_buffer, self)
            # TransformListener 생성
                # → /tf, /tf_static 구독 시작!
                # → 받은 데이터를 self.tf_buffer에 자동 저장
                # → 이후 self.tf_buffer에서 조회하면 됨
        

        # ---- 주기적 실행 타이머 ----
        
        self.timer = self.create_timer(1.0, self.timer_callback)
            # 1초마다 timer_callback 호출 → TF 조회 예제를 주기적으로 실행
    



    def timer_callback(self):
        """
            1초마다 호출: lidar_link → base_link 변환을 조회하고 출력
        """
        
        # 방법 1: 변환 행렬(Transform) 자체를 조회
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',        # 목표 프레임 (to)
                'lidar_link',       # 원본 프레임 (from)
                rclpy.time.Time(),  # 시간 (0 = 최신!)
                timeout=Duration(seconds=1.0)
                # timeout: TF가 아직 없으면 최대 1초 대기 (1초 안에 안 오면 예외 발생)
            )
            # lookup_transform(target, source, time): "source 프레임에서 target 프레임으로의 변환"   
                # ⚠️ 순서 주의!
                # lookup_transform('base_link', 'lidar_link', ...)
                    # = "lidar_link → base_link" 변환
                    # = lidar_link에 있는 점을 base_link 기준으로 표현
            
                # rclpy.time.Time() = 시간 0 = "최신 TF 사용"
                # 특정 시점: rclpy.time.Time(seconds=1.5)
            
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not get transform: {ex}'
            )
            return
        # TF 조회 실패 시:
            # 1. 프레임이 아직 존재하지 않음
            # 2. 트리가 연결되지 않음
            # 3. 요청한 시간의 TF가 없음
                # → 에러 로그 출력 후 다음 주기에 재시도!
        

        # 결과 출력:
        t = transform.transform.translation
        r = transform.transform.rotation
        
        self.get_logger().info(
            f'lidar_link → base_link:\n'
            f'  Translation: x={t.x:.3f}, y={t.y:.3f}, z={t.z:.3f}\n'
            f'  Rotation: x={r.x:.3f}, y={r.y:.3f}, z={r.z:.3f}, w={r.w:.3f}'
        )
        # 예상 출력:
            # Translation: x=0.130, y=0.000, z=-0.424
            # Rotation: x=0.000, y=0.000, z=0.000, w=1.000
        
            # ⚠️ URDF에서 base_link → lidar_link = (-0.130, 0, +0.424)
                # 역변환: lidar_link → base_link = (+0.130, 0, -0.424)
                # → 부호가 반대! 역변환이니까!




        # 방법 2: 특정 점(Point)을 변환
        
        # LiDAR가 감지한 장애물: lidar_link 기준 (2.0, 0.5, 0)
        obstacle_in_lidar = PointStamped()
        obstacle_in_lidar.header.frame_id = 'lidar_link'
        obstacle_in_lidar.header.stamp = self.get_clock().now().to_msg()
        obstacle_in_lidar.point.x = 2.0   # 전방 2m
        obstacle_in_lidar.point.y = 0.5   # 왼쪽 0.5m
        obstacle_in_lidar.point.z = 0.0   # 같은 높이
        
        try:
            obstacle_in_base = self.tf_buffer.transform(
                obstacle_in_lidar,
                'base_link',
                timeout=Duration(seconds=1.0)
            )
            # tf_buffer.transform(point, target_frame): point를 target_frame 기준으로 변환!
                # 내부적으로 lookup_transform + 행렬 곱 수행
            
        except TransformException as ex:
            self.get_logger().warn(f'Transform failed: {ex}')
            return
        

        p = obstacle_in_base.point
        self.get_logger().info(
            f'Obstacle in base_link: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}'
        )
        # 예상 결과:
            # LiDAR 위치: (-0.130, 0, +0.424) from base_link
            # 장애물 in LiDAR: (2.0, 0.5, 0)
            # 장애물 in base_link: 
                # x = 2.0 + (-0.130) = 1.870
                # y = 0.5 + 0 = 0.500
                # z = 0.0 + 0.424 = 0.424
        
                # → "base_link 기준 전방 1.87m, 왼쪽 0.5m, 높이 0.424m"
                # → 이 좌표로 Nav2가 장애물 회피!



def main():
    rclpy.init()
    node = TFListenerExample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()