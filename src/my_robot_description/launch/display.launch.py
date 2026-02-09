"""
    목적: RViz에서 로봇 모델을 시각화
        → Xacro → URDF 변환
        → robot_state_publisher 실행
        → joint_state_publisher_gui 실행
        → RViz 실행
"""

import os
from ament_index_python.packages import get_package_share_directory
    # get_package_share_directory: 패키지의 share 디렉토리 경로를 반환
        # 예: '/home/user/robot_ws/install/my_robot_description/share/my_robot_description'
            # 여기에 URDF, 메시, Launch 등이 설치되어 있음!
            # setup.py의 data_files에서 설치한 파일들이 이 경로 아래에 위치함

from launch import LaunchDescription
    # LaunchDescription: Launch 파일의 핵심 클래스
        # "이 Launch에서 무엇을 실행할지" 목록을 담음 → return LaunchDescription([...]) 형태로 반환

from launch.actions import DeclareLaunchArgument
    # DeclareLaunchArgument: Launch 인자(Argument) 선언
        # 명령줄에서 파라미터를 전달받을 수 있게 함
            # 예: ros2 launch ... use_gui:=false

from launch.substitutions import LaunchConfiguration, Command
    # LaunchConfiguration: 선언된 Launch 인자의 값을 참조 -> DeclareLaunchArgument로 선언한 값을 가져옴
    # Command: 셸 명령을 실행하고 그 출력을 문자열로 사용 → xacro 명령 실행에 사용!

from launch.conditions import IfCondition, UnlessCondition
    # IfCondition: 조건부 실행 → 특정 인자가 true일 때만 노드 실행

from launch_ros.actions import Node
    # Node: ROS 2 노드를 실행하는 Launch 액션 → 패키지명, 실행파일명, 파라미터 등 지정

from launch_ros.parameter_descriptions import ParameterValue




def generate_launch_description():
    """
    Launch 파일의 진입점.
    
    ROS 2 Launch 시스템은 이 함수를 찾아서 호출
        → 함수 이름이 반드시 generate_launch_description이어야 함
        → LaunchDescription 객체를 반환해야 함
    """
    
    # 1. 경로 설정    
    pkg_share = get_package_share_directory('my_robot_description')
        # 결과 예: '/home/user/robot_ws/install/my_robot_description/share/my_robot_description'
    default_model_path = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
        # 결과 예: '.../share/my_robot_description/urdf/my_robot.urdf.xacro'
            # os.path.join: 경로를 안전하게 조합
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'robot_view.rviz')
        # RViz 설정 파일 경로
    


    # 2. Launch 인자 선언
    declare_model_path = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot URDF/Xacro file'
    )
        # 'model' 인자:
            # 기본값: 우리 Xacro 파일 경로
            # 오버라이드: ros2 launch ... model:=/other/path/robot.urdf
    
    declare_rviz_config = DeclareLaunchArgument(
        name='rviz_config',
        default_value=default_rviz_config_path,
        description='Absolute path to RViz config file'
    )
    
    declare_use_gui = DeclareLaunchArgument(
        name='use_gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )
        # 'use_gui' 인자:
            # true: GUI 슬라이더 표시 (디버깅용)
            # false: GUI 없이 실행 (Gazebo 연동 시)
                # → ros2 launch ... use_gui:=false
    


    # 3. Xacro → URDF 변환
    #robot_description_content = Command(['xacro ', LaunchConfiguration('model')])
        # Command(['xacro ', ...]): 셸에서 'xacro /path/to/my_robot.urdf.xacro' 실행!
            # → Xacro 매크로를 펼쳐서 순수 URDF XML 생성
            # → 그 결과(XML 문자열)를 robot_description_content에 저장

    robot_description_content = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)
        # ROS2 Launch는 기본적으로 파라미터 = YAML이라고 가정 하지만
            # URDF = XML
            # Xacro 결과 = 순수 문자열 -> 그래서 명시적으로 str 타입을 선언


    robot_description = {'robot_description': robot_description_content}
        # 딕셔너리 형태로 파라미터 준비
            # → robot_state_publisher 노드에 전달!
            # → 'robot_description' 파라미터 이름은 ROS 2 규약!
    


    # 4. 노드 정의
    
    # ---- (1) robot_state_publisher ----
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
        # 역할:
            # 1. URDF를 파싱해서 로봇 구조 파악
            # 2. /joint_states 토픽을 구독
            # 3. 각 Link의 TF를 계산해서 /tf에 발행
            # 4. /robot_description 파라미터로 URDF 제공
    
        # 입력: /joint_states (sensor_msgs/JointState)
        # 출력: /tf, /tf_static (geometry_msgs/TransformStamped)
    
        # output='screen': 노드의 로그를 터미널에 출력 → 에러 메시지를 바로 볼 수 있음!
    

    # ---- (2) joint_state_publisher_gui ----
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )
        # 역할:
            # 1. non-fixed Joint의 상태(각도)를 발행
            # 2. GUI 슬라이더로 수동 조작 가능
            # 3. /joint_states 토픽에 발행

        # condition=IfCondition(...): use_gui가 'true'일 때만 실행
            # → Gazebo 연동 시에는 Gazebo가 joint_states를 발행하므로 이 노드가 필요 없음 → use_gui:=false로 끔
    
        # 발행하는 Joint들 (우리 로봇):
            # left_wheel_joint  → 슬라이더로 바퀴 회전!
            # right_wheel_joint → 슬라이더로 바퀴 회전!

    
    # ---- (3) joint_state_publisher (GUI 없는 버전) ----
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('use_gui'))
    )
        # use_gui가 false일 때만 실행
            # → GUI 없이 기본값(0)으로 joint_states 발행
            # ROS2 Launch에서 IfCondition은: 파이썬 표현식 (not, and, or) 지원 안 함
            

    # ---- (4) RViz ----
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )
        # arguments=['-d', ...]:
            # -d: RViz 설정 파일을 지정
            # → 미리 저장한 설정으로 RViz 열기!
            # → RobotModel, TF, Grid 등이 이미 추가된 상태!
        
            # ⚠️ RViz 설정 파일이 없으면?
                # → 빈 RViz가 열림 → 수동으로 디스플레이 추가 필요
            
    

    # 5. LaunchDescription 조립 + 반환
    return LaunchDescription([
        # 인자 선언
        declare_model_path,
        declare_rviz_config,
        declare_use_gui,
        
        # 노드 실행
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        rviz_node,
    ])


    # 순서:
        # 1. 인자 선언 → 값이 결정됨
        # 2. 노드 실행 → 인자 값을 사용
    
    # 노드들은 거의 동시에 실행됨 (순서 보장 아님!)
        # → 하지만 robot_state_publisher가 먼저 떠야 RViz가 로봇 모델을 표시할 수 있음
        # → 실제로는 RViz가 잠시 대기하다가 데이터 오면 표시




'''
    [robot_state_publisher가 하는 일 — 상세]

    ┌─────────────────────────┐
    │  URDF (robot_description) 
    │  파라미터로 전달           │
    └───────────┬─────────────┘
                │ 파싱
                ▼
    ┌─────────────────────────┐
    │  로봇 구조 파악           │
    │  Link-Joint 트리 구축     │
    └───────────┬─────────────┘
                │
    ┌───────────┴─────────────┐
    │                         │
    ▼                         ▼
  Fixed Joints           Non-fixed Joints
  (base→lidar 등)        (base→wheel 등)
    │                         │
    ▼                         ▼
  /tf_static              /joint_states 구독
  (한번만 발행)            (매 프레임 수신)
                              │
                              ▼
                          FK 계산
                          (Forward Kinematics)
                              │
                              ▼
                            /tf
                          (매 프레임 발행)

    정리:
    ● fixed Joint → /tf_static (한번 발행, 변하지 않으니까)
    ● non-fixed Joint → /tf (매번 새로 계산해서 발행)
    
    /tf_static 예시:
        base_link → lidar_link (항상 같은 위치)
        base_link → camera_link (항상 같은 위치)
        base_link → imu_link (항상 같은 위치)
        base_footprint → base_link (항상 같은 높이)
        
    /tf 예시:
        base_link → left_wheel (바퀴 회전 각도에 따라 변함!)
        base_link → right_wheel (바퀴 회전 각도에 따라 변함!)
'''

'''
    [/joint_states 메시지 구조]

    sensor_msgs/JointState:
        header:
            stamp: {sec: 1234, nanosec: 567890}
            frame_id: ''
        name: ['left_wheel_joint', 'right_wheel_joint']
        position: [0.785, 1.571]      # 라디안! (45°, 90°)
        velocity: [2.0, 2.0]          # rad/s
        effort: [0.1, 0.1]            # N·m (토크)

    → joint_state_publisher_gui: 슬라이더에서 position을 설정
    → Gazebo: 시뮬레이션 결과로 position/velocity/effort 발행
    → 실제 로봇: 엔코더에서 읽은 값으로 position/velocity 발행
'''