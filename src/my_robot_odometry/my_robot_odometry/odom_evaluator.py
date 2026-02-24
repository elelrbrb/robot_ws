#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════
# Odometry를 Ground Truth와 비교하여 RMSE 계산
# ════════════════════════════════════════════════════════════

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import numpy as np


class OdomEvaluator(Node):
    """오도메트리 정확도 평가 노드."""

    def __init__(self):
        super().__init__('odom_evaluator')


        self.gt_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.gt_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )


        self.latest_gt = None
        self.latest_odom = None
        self.errors_x = []
        self.errors_y = []
        self.errors_yaw = []

        self.timer = self.create_timer(1.0, self.evaluate)



    def gt_callback(self, msg):
        self.latest_gt = msg


    def odom_callback(self, msg):
        self.latest_odom = msg



    def evaluate(self):
        if self.latest_gt is None or self.latest_odom is None:
            return

        # 위치 오차
        gt_x = self.latest_gt.pose.pose.position.x
        gt_y = self.latest_gt.pose.pose.position.y
        od_x = self.latest_odom.pose.pose.position.x
        od_y = self.latest_odom.pose.pose.position.y

        err_x = gt_x - od_x
        err_y = gt_y - od_y
        err_pos = math.sqrt(err_x**2 + err_y**2)


        # Yaw 오차
        gt_yaw = self._quat_to_yaw(self.latest_gt.pose.pose.orientation)
        od_yaw = self._quat_to_yaw(self.latest_odom.pose.pose.orientation)
        err_yaw = gt_yaw - od_yaw
        # 래핑 (각도 차이를 -π ~ +π 범위로)
        while err_yaw > math.pi: err_yaw -= 2*math.pi
        while err_yaw < -math.pi: err_yaw += 2*math.pi

        self.errors_x.append(err_x)
        self.errors_y.append(err_y)
        self.errors_yaw.append(err_yaw)


        # RMSE: Root Mean Square Error(평균 제곱근 오차) 계산
        if len(self.errors_x) > 5:
            rmse_pos = math.sqrt(
                np.mean(np.array(self.errors_x)**2 +
                        np.array(self.errors_y)**2)
            )
            rmse_yaw = math.sqrt(
                np.mean(np.array(self.errors_yaw)**2)
            )

            self.get_logger().info(
                f'[Eval] Pos err: {err_pos:.4f}m | '
                f'Yaw err: {math.degrees(err_yaw):.2f}° | '
                f'RMSE pos: {rmse_pos:.4f}m | '
                f'RMSE yaw: {math.degrees(rmse_yaw):.2f}°'
            )




    @staticmethod
    def _quat_to_yaw(q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)




def main(args=None):
    rclpy.init(args=args)
    node = OdomEvaluator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        # 종료 처리 (Ctrl+C)
        # 만약 오차 데이터가 있다면, 마지막 RMSE를 계산해서 출력
        if node.errors_x:
            rmse_final = math.sqrt(
                np.mean(np.array(node.errors_x)**2 +
                        np.array(node.errors_y)**2)
            )
            print(f'\nFinal RMSE Position: {rmse_final:.4f} m')
            print(f'Final RMSE Yaw: {math.degrees(math.sqrt(np.mean(np.array(node.errors_yaw)**2))):.2f}°')
            print(f'Samples: {len(node.errors_x)}')
    
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
