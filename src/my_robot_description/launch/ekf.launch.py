#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# Day 66 — EKF 노드 Launch (Gazebo 시뮬레이션 통합)
# ═══════════════════════════════════════════════════════════════

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node



def generate_launch_description():
    # ============================================================
    # 1️⃣ 패키지 경로
    # ============================================================
    pkg_desc = get_package_share_directory('my_robot_description')
    pkg_control = get_package_share_directory('my_robot_control')


    # ============================================================
    # 2️⃣ Launch 인자
    # ============================================================
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )


    # ============================================================
    # 3️⃣ Gazebo + 센서 통합 Launch 재사용
    # ============================================================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_desc, 'launch',
                         'gazebo_full_sensors.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # ============================================================
    # 3️⃣ Teleop Stack (Day 43 Launch 재사용)
    # ============================================================
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_control, 'launch',
                         'teleop.launch.py') 
        )
    )



    # ============================================================
    # 4️⃣ EKF Node (Allan 기반 반영)
    # ============================================================
    ekf_node = Node(
        package='my_robot_control',
        executable='ekf_ros_node',
        name='ekf_node',
        output='screen',
        parameters=[{

            'use_sim_time': LaunchConfiguration('use_sim_time'),

            # ───────────── 초기 공분산 ─────────────
            'initial_cov_x': 0.1,
            'initial_cov_y': 0.1,
            'initial_cov_theta': 0.05,

            # ───────────── Motion Noise (Thrun α) ─────────────
            # 실제 로봇은 완벽하게 제어되지 않기 때문에
            # 너무 작게 주면 필터가 비현실적으로 움직임을 믿음
            'alpha1': 0.02,
            'alpha2': 0.01,
            'alpha3': 0.02,
            'alpha4': 0.01,            
            
            # ───────────── Odometry Measurement Noise ─────────────
            # Odom은 IMU보다 보통 신뢰도가 낮음
            'odom_noise_x': 0.001,
            'odom_noise_y': 0.001,
            'odom_noise_theta': 10.0,

            # ───────────── IMU Measurement Noise (Yaw) ─────────────
            # 반드시 orientation_cov_yaw 값과 일관성 유지
            'imu_noise_theta': 0.01,
            
            

            # ───────────── Frame 설정 ─────────────
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
        }],
    )



    # ============================================================
    # 6️⃣ RViz
    # ============================================================
    rviz_config = os.path.join(pkg_desc, 'rviz', 'ekf.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )



    # ============================================================
    # 반환
    # ============================================================
    return LaunchDescription([
        declare_use_sim_time,
        gazebo_launch,
        teleop_launch,
        ekf_node,
        rviz,
    ])