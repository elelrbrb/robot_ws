# ════════════════════════════════════════════════════════════
# Day 43 — 텔레오퍼레이션 스택 통합 Launch
# ════════════════════════════════════════════════════════════

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ---- 패키지 경로 ----
    pkg_control = get_package_share_directory('my_robot_control')


    # ════════════════════════════════════════
    # 1. twist_mux
    # ════════════════════════════════════════
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[
            os.path.join(pkg_control, 'config', 'twist_mux.yaml')
        ],
        remappings=[
            ('cmd_vel_out', '/cmd_vel_mux')
                # twist_mux 출력 → /cmd_vel_mux
                # (Safety Layer의 입력으로!)
        ]
    )

    # ════════════════════════════════════════
    # 2. Safety Layer
    # ════════════════════════════════════════
    safety_node = Node(
        package='my_robot_control',
        executable='safety_layer_node',
        name='safety_layer',
        parameters=[{
            'max_linear_vel': 0.5,
            'max_angular_vel': 1.5,
            'max_linear_accel': 0.5,
            'max_angular_accel': 1.0,
            'slowdown_distance': 1.0,
            'stop_distance': 0.3,
            'obstacle_angle_range': 30.0,
            'cmd_timeout': 0.5,
        }]
    )

    # ════════════════════════════════════════
    # 3. E-Stop
    # ════════════════════════════════════════
    e_stop_node = Node(
        package='my_robot_control',
        executable='e_stop_node',
        name='e_stop'
    )

    # ════════════════════════════════════════
    # 4. 키보드 텔레오퍼레이션
    # ════════════════════════════════════════
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        prefix='xterm -e',
            # 별도 터미널 창에서 실행!
            # 키보드 입력을 받아야 하므로
        remappings=[
            ('cmd_vel', '/cmd_vel_key')
                # /cmd_vel → /cmd_vel_key로 리매핑
        ]
    )

    return LaunchDescription([
        twist_mux_node,
        safety_node,
        e_stop_node,
        teleop_keyboard,
    ])