#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# gazebo_full_sensors.launch.py
#
# ✔ Gazebo + Robot + IMU + LiDAR + RViz 통합 실행
# ✔ 확장성 좋은 정석 구조
# ✔ use_sim_time 완전 동기화
# ✔ Gazebo model path 자동 설정
# ═══════════════════════════════════════════════════════════════

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    AppendEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ============================================================
    # 1️⃣ 패키지 경로 설정
    # ============================================================
    pkg_share = get_package_share_directory('my_robot_description')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    gazebo_params_path = os.path.join(pkg_share,'config','gazebo_params.yaml')

    default_model_path = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    default_world_path = os.path.join(pkg_share, 'worlds', 'test_room.world')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'full_sensors.rviz')

    # Gazebo가 package:// 경로를 찾도록 환경변수 설정
    install_dir = os.path.join(pkg_share, '..', '..')
    set_gazebo_model_path = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=install_dir
    )


    # ============================================================
    # 2️⃣ Launch 인자 선언
    # ============================================================
    declare_world = DeclareLaunchArgument(
        name='world',
        default_value=default_world_path,
        description='Path to world file'
    )

    declare_model = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Path to URDF/Xacro file'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock'
    )

    declare_rviz = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Launch RViz'
    )


    # ============================================================
    # 3️⃣ Xacro → robot_description
    # ============================================================
    robot_description_content = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    robot_description = {
        'robot_description': robot_description_content
    }





    # ============================================================
    # 1. Robot State Publisher
    # ============================================================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # ============================================================
    # 2. Gazebo 실행
    # ============================================================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'params_file': gazebo_params_path
            
        }.items()
    )

    # ============================================================
    # 3. 로봇 스폰
    # ============================================================
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', 'my_mobile_robot',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-Y', '0.0',
        ]
    )

    # ============================================================
    # 4. RViz
    # ============================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )



    # ============================================================
    # 5. IMU Processor Node
    # ============================================================
    imu_processor = Node(
        package='my_robot_odometry',
        executable='imu_processor_node',
        name='imu_processor',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),

            # 🔹 Orientation covariance
            'orientation_cov_roll': 0.0025,
            'orientation_cov_pitch': 0.0025,
            'orientation_cov_yaw': 0.01,
            
            # 🔹 Allan 결과 반영
            '''
            'angular_vel_cov': 3.97e-08,
            'linear_accel_cov': 2.28e-04,
            '''
            'angular_vel_cov': 1.20e-07,
            'linear_accel_cov': 8.85e-04,

            'remove_gravity': False,
        }]
    )

    # ============================================================
    # 6. LiDAR Processor Node
    # ============================================================
    lidar_processor = Node(
        package='my_robot_perception',
        executable='lidar_processor_node',
        name='lidar_processor',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'median_kernel_size': 5,
            'min_cluster_size': 3,
            'cluster_threshold_factor': 2.5,
        }]
    )


    # ============================================================
    # 7. Depth Camera 32FC1 → 16UC1 변환 노드
    # ============================================================
    depth_converter_node = Node(
        package='my_robot_perception',          
        executable='depth_converter_node',      
        name='depth_converter',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    

    # ============================================================
    # 🔟 LaunchDescription 반환
    # ============================================================
    return LaunchDescription([
        set_gazebo_model_path,

        declare_world,
        declare_model,
        declare_use_sim_time,
        declare_rviz,

        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot,
        #rviz_node,

        imu_processor,
        lidar_processor,

        depth_converter_node,
    ])