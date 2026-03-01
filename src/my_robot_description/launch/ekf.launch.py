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
            'initial_cov_x': 0.05,
            'initial_cov_y': 0.05,
            'initial_cov_theta': 0.02,

            # ───────────── Motion Noise (Thrun α) ─────────────
            'alpha1': 0.01,
            'alpha2': 0.005,
            'alpha3': 0.005,
            'alpha4': 0.01,

            # ───────────── Odometry Measurement Noise ─────────────
            'odom_noise_x': 0.0005,
            'odom_noise_y': 0.0005,
            'odom_noise_theta': 0.002,

            # ───────────── IMU Measurement Noise (Yaw) ─────────────
            # IMU Processor의 orientation_cov_yaw와 동일하게!
            'imu_noise_theta': 2e-4,

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