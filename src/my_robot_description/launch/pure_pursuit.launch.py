#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# launch/pure_pursuit.launch.py
# Day 77 — Pure Pursuit 전체 스택 Launch (EKF 스타일 구조화)
# ════════════════════════════════════════════════════════════════

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ============================================================
    # 1️⃣ 패키지 경로
    # ============================================================
    pkg_desc = get_package_share_directory('my_robot_description')
    pkg_ctrl = get_package_share_directory('my_robot_control')


    # ============================================================
    # 2️⃣ Launch 인자
    # ============================================================
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    
    path_type_arg = DeclareLaunchArgument(
        'path_type',
        default_value='square',
        description='Test path: straight, circle, square, slalom, figure8'
    )

    velocity_arg = DeclareLaunchArgument(
        'velocity',
        default_value='0.3',
        description='Desired velocity [m/s]'
    )

    lookahead_arg = DeclareLaunchArgument(
        'lookahead',
        default_value='0.4',
        description='Lookahead distance [m]'
    )


    # ============================================================
    # 3️⃣ Gazebo + 센서 통합 Launch
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
    # 5️⃣ robot_localization EKF (Day 67)
    # ============================================================
    ekf_config = os.path.join(pkg_ctrl, 'config', 'ekf.yaml')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ekf_config
        ],
        output='screen'
    )

    # ============================================================
    # 3️⃣ Teleop Stack (Day 43 Launch 재사용)
    # ============================================================
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ctrl, 'launch',
                         'teleop.launch.py') 
        )
    )


    # ============================================================
    # 6️⃣ Path Publisher
    # ============================================================
    path_publisher = Node(
        package='my_robot_control',
        executable='path_publisher_node',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'path_type': LaunchConfiguration('path_type'),
            'delay': 5.0,
            'path_frame': 'odom',
        }],
        output='screen'
    )

    # ============================================================
    # 7️⃣ Pure Pursuit Node
    # ============================================================
    pure_pursuit = Node(
        package='my_robot_control',
        executable='pure_pursuit_node',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'lookahead_distance': LaunchConfiguration('lookahead'),
            'min_lookahead': 0.15,
            'max_lookahead': 1.0,
            'adaptive_lookahead': True,
            'lookahead_gain': 0.5,
            'desired_velocity': LaunchConfiguration('velocity'),
            'min_velocity': 0.05,
            'max_velocity': 0.5,
            'max_omega': 1.5,
            'goal_tolerance': 0.1,
            'velocity_curvature_gain': 0.3,
            'control_rate': 50.0,
            'path_frame': 'odom',
        }],
        output='screen'
    )


    # ============================================================
    # 8️⃣ RViz
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
        path_type_arg,
        velocity_arg,
        lookahead_arg,
        gazebo_launch,
        ekf_node,
        teleop_launch,
        path_publisher,
        pure_pursuit,
        rviz,
    ])
