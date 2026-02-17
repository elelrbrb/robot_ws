#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════
# Day 43 — 비상 정지 노드
    # 키보드 스페이스바 또는 특정 조건에서 /e_stop 토글.
    # twist_mux의 lock 기능과 연동하여 모든 cmd_vel을 차단.
# ════════════════════════════════════════════════════════════

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class EStopNode(Node):
    """
    비상 정지 토글 노드.

    /e_stop 토픽에 Bool 메시지를 발행:
      True  = 비상 정지 활성 (로봇 정지!)
      False = 비상 정지 해제 (정상 운행)

    twist_mux의 locks 설정과 연동:
      locks:
        e_stop:
          topic: /e_stop
          timeout: 0.0      ← 영구 잠금!
          priority: 255      ← 최고 우선순위!
    """

    def __init__(self):
        super().__init__('e_stop_node')

        self.e_stop_active = False
            # 초기 상태: 비상 정지 해제

        self.pub = self.create_publisher(Bool, '/e_stop', 10)

        # 주기적으로 상태 발행 (10Hz)
        self.timer = self.create_timer(0.1, self.publish_state)
            # 왜 주기적으로 발행?
            # → twist_mux의 timeout=0.0이므로 한 번만 보내도 되지만,
            #   안전을 위해 지속적으로 발행!
            # → 메시지 유실에 대비!

        self.get_logger().info('[E-Stop] Node started. e_stop=False')
        self.get_logger().info('[E-Stop] Call: ros2 topic pub /e_stop_toggle ...')

        # E-Stop 토글 구독 (외부에서 토글 가능)
        self.toggle_sub = self.create_subscription(
            Bool, '/e_stop_toggle', self.toggle_callback, 10
        )

    def toggle_callback(self, msg: Bool):
        """외부에서 비상 정지 토글."""
        self.e_stop_active = msg.data
        state = "ACTIVE (로봇 정지!)" if self.e_stop_active else "RELEASED (정상 운행)"
        self.get_logger().warn(f'[E-Stop] {state}')

    def publish_state(self):
        """현재 E-Stop 상태를 주기적으로 발행."""
        msg = Bool()
        msg.data = self.e_stop_active
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()