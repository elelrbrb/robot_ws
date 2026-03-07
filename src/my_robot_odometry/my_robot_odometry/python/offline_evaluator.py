#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# offline_evaluator.py
# Day 80 — ros2 bag 기반 오프라인 평가
# ════════════════════════════════════════════════════════════════

import sys
import math
import numpy as np

from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)



def load_bag(bag_path: str) -> dict:
    """ros2 bag에서 데이터 추출."""
    reader = SequentialReader()
    storage = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage, converter)

    data = {
        'gt': [],      # (t, x, y, theta, vx, wz)
        'ekf': [],
        'cmd': [],     # (t, v, omega)
        'path': [],    # [(x, y), ...]
    }

    t0 = None

    while reader.has_next():
        topic, raw, timestamp = reader.read_next()
        t_sec = timestamp * 1e-9
        if t0 is None:
            t0 = t_sec
        t = t_sec - t0

        if topic == '/ground_truth/odom':
            msg = deserialize_message(raw, Odometry)
            p = msg.pose.pose.position
            yaw = quat_to_yaw(msg.pose.pose.orientation)
            data['gt'].append((
                t, p.x, p.y, yaw,
                msg.twist.twist.linear.x,
                msg.twist.twist.angular.z
            ))

        elif topic == '/odometry/filtered':
            msg = deserialize_message(raw, Odometry)
            p = msg.pose.pose.position
            yaw = quat_to_yaw(msg.pose.pose.orientation)
            data['ekf'].append((
                t, p.x, p.y, yaw,
                msg.twist.twist.linear.x,
                msg.twist.twist.angular.z
            ))

        elif topic == '/cmd_vel_key':
            msg = deserialize_message(raw, Twist)
            data['cmd'].append((t, msg.linear.x, msg.angular.z))

        elif topic == '/path':
            msg = deserialize_message(raw, Path)
            data['path'] = [
                (ps.pose.position.x, ps.pose.position.y)
                for ps in msg.poses
            ]

    return data



def evaluate_bag(bag_path: str):
    """Bag 파일 평가."""
    print(f'\nLoading: {bag_path}')
    data = load_bag(bag_path)

    print(f'  GT points: {len(data["gt"])}')
    print(f'  EKF points: {len(data["ekf"])}')
    print(f'  CMD points: {len(data["cmd"])}')
    print(f'  Path points: {len(data["path"])}')

    if len(data['gt']) < 10 or len(data['path']) < 2:
        print('  ⚠️ Insufficient data!')
        return

    # CTE 계산 (GT 기준!)
    path = data['path']
    cte_list = []

    for gt_point in data['gt']:
        _, gx, gy, _, _, _ = gt_point
        min_cte = float('inf')
        for i in range(len(path) - 1):
            ax, ay = path[i]
            bx, by = path[i + 1]
            abx, aby = bx - ax, by - ay
            ab_sq = abx**2 + aby**2
            if ab_sq < 1e-12:
                continue
            arx, ary = gx - ax, gy - ay
            t = max(0, min(1, (arx*abx + ary*aby) / ab_sq))
            px, py = ax + t*abx, ay + t*aby
            dist = math.sqrt((gx-px)**2 + (gy-py)**2)
            if dist < abs(min_cte):
                cross = abx*ary - aby*arx
                min_cte = cross / math.sqrt(ab_sq)
        cte_list.append(min_cte)

    cte = np.array(cte_list)



    # 리포트
    total_time = data['gt'][-1][0]

    print()
    print('═' * 50)
    print('  OFFLINE EVALUATION REPORT')
    print('═' * 50)
    print(f'  CTE RMSE:    {np.sqrt(np.mean(cte**2))*100:.2f} cm')
    print(f'  CTE Max:     {np.max(np.abs(cte))*100:.2f} cm')
    print(f'  CTE Mean:    {np.mean(np.abs(cte))*100:.2f} cm')
    print(f'  Total Time:  {total_time:.1f} s')
    print('═' * 50)




if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: python3 offline_evaluator.py <bag_path>')
        sys.exit(1)
    evaluate_bag(sys.argv[1])