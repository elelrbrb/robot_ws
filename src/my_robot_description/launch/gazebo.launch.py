import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    ExecuteProcess,
    AppendEnvironmentVariable  # [NEW] 환경 변수 추가 액션 임포트
)
    # IncludeLaunchDescription: 다른 Launch 파일을 포함 실행 → Gazebo 자체의 Launch 파일을 가져와서 실행
    # ExecuteProcess: 임의의 프로세스(명령어)를 실행 → 필요 시 사용

from launch.launch_description_sources import PythonLaunchDescriptionSource
    # 다른 Python Launch 파일을 소스로 지정

from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition



def generate_launch_description():
    
    # ================================================
    # 경로 설정
    # ================================================
    pkg_share = get_package_share_directory('my_robot_description')
    default_model_path = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    default_world_path = os.path.join(pkg_share, 'worlds', 'my_world.world')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'robot_view.rviz')
    
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
        # gazebo_ros 패키지의 share 디렉토리 → gazebo_ros의 Launch 파일이 여기에 있음
    
    # [NEW] Gazebo 모델 경로 설정 
        # install/my_robot_description/share/my_robot_description 경로에서 상위 두 단계(../..)로 올라가면 'install/share'가 됩니다.
        # 이 경로를 Gazebo에게 알려주어야 'package://' 경로를 찾을 수 있습니다.
    install_dir = os.path.join(pkg_share, '..', '..')

    set_gazebo_model_path = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=install_dir
    )


    # ================================================
    # Launch 인자
    # ================================================
    declare_world = DeclareLaunchArgument(
        name='world',
        default_value=default_world_path,
        description='Full path to the world file'
    )
    
    declare_model = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Path to robot URDF/Xacro'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation clock from Gazebo'
    )
    # ⭐ use_sim_time = true: 모든 ROS 2 노드가 Gazebo의 시뮬 시간을 사용!

        # 왜 중요한가?
            # → Gazebo가 일시정지(Pause)되면 시간도 멈춤
            # → 실제 시간(wall clock)을 쓰면 일시정지 중에도 TF가 계속 발행 → 꼬임!
            # → 시뮬 시간을 쓰면 멈추면 같이 멈춤 ✅
    
            # Gazebo가 /clock 토픽에 시뮬 시간을 발행 → use_sim_time=true인 노드들이 이 시간 사용
    
    declare_rviz = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    


    # ================================================
    # Xacro → URDF
    # ================================================
    robot_description_content = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)
    
    robot_description = {'robot_description': robot_description_content}
    

    
    # ================================================
    # 노드 1: robot_state_publisher
    # ================================================
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
    # ⚠️ use_sim_time 파라미터 추가!
        # → Gazebo 시뮬 시간과 동기화
        # → 이게 없으면 TF timestamp이 꼬임!
    

    # ================================================
    # 노드 2: Gazebo Server + Client
    # ================================================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
        }.items()
    )
    # gazebo_ros의 gazebo.launch.py를 포함 실행! 
        # 이 Launch 파일이 하는 일:
            # 1. gzserver 실행 (물리 엔진 + 센서 엔진)
            # 2. gzclient 실행 (3D 렌더링 GUI)
            # 3. /clock 토픽 발행 시작
            # 4. /spawn_entity 서비스 활성화
    
        # launch_arguments로 world 파일 경로 전달!
    

    # ================================================
    # 노드 3: 로봇 스폰
    # ================================================
    
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', 'my_mobile_robot',
            # -entity: Gazebo 안에서 이 모델의 이름 → 여러 로봇을 스폰할 때 이름으로 구분
            '-topic', '/robot_description',
            # -topic: URDF를 어디서 가져올지
                # → robot_state_publisher가 발행하는 토픽!
                # → spawn_entity가 이 토픽에서 URDF를 읽어서 Gazebo에 전달!   
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.1',
            # 초기 스폰 위치 (XYZ)
                # Z=0.1: 지면보다 약간 위에서 스폰
                # → 떨어지면서 자연스럽게 착지!
                # → Z=0으로 하면 바퀴가 바닥에 끼일 수 있음!
            '-Y', '0.0',
            # 초기 Yaw (방향)
                # 0 = X+ 방향(전방) 바라봄
                # 1.5708 = Y+ 방향(왼쪽) 바라봄
        ]
    )
    # spawn_entity.py: gazebo_ros 패키지의 유틸리티 스크립트
        # Gazebo의 /spawn_entity 서비스를 호출!
    
        # 다른 소스 옵션:
            # -file /path/to/model.urdf → 파일에서 직접 읽기
            # -database model_name → Gazebo 모델 데이터베이스
            # -topic /robot_description → ROS 2 토픽에서 읽기 ✅
    

    # ================================================
    # 노드 4: RViz (선택)
    # ================================================
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
    

    # ================================================
    # 반환
    # ================================================
    
    return LaunchDescription([
        # [NEW] 환경변수 설정이 가장 먼저 실행되어야 합니다.
        set_gazebo_model_path,
        
        declare_world,
        declare_model,
        declare_use_sim_time,
        declare_rviz,
    
        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot,
        rviz_node,
    ])



