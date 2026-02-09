"""
    역할:
        1. Python 패키지 빌드/설치 설정
        2. 데이터 파일(URDF, 메시, Launch 등) 설치 위치 지정
        3. colcon build가 이 파일을 읽어서 패키지를 설치

    핵심:
        data_files 리스트!
            → URDF, STL, Launch 파일은 Python 코드가 아님
            → "데이터 파일"로 지정해서 설치 경로에 복사해야 함
            → 이걸 빠뜨리면 Launch 시 "파일 못 찾음" 에러!
"""

import os
from glob import glob
from setuptools import setup

package_name = 'my_robot_description'

setup(
    name=package_name,
    version='0.1.0',
    
    packages=[package_name],
    # Python 패키지 (my_robot_description/ 디렉토리)
    # __init__.py가 있는 디렉토리를 Python 모듈로 인식
    
    data_files=[
        # (1) ament index 등록 (필수 보일러플레이트)
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
            # → colcon이 이 패키지를 찾을 수 있게 등록
            # → 이게 없으면 ros2 pkg list에 안 나옴
        
        # (2) package.xml (필수)
        ('share/' + package_name, ['package.xml']),
            # → 패키지 메타데이터

        # (3) URDF/Xacro 파일들
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro') + glob('urdf/*.urdf')),
            # glob('urdf/*.xacro'): urdf/ 디렉토리의 모든 .xacro 파일을 찾음
                # 설치 위치: install/share/my_robot_description/urdf/
                # → Launch에서 $(find my_robot_description)/urdf/ 로 접근
        
        # (4) 메시(STL) 파일들
        ('share/' + package_name + '/meshes', glob('meshes/*.stl') + glob('meshes/*.dae')),
            # STL (바이너리 메시)와 DAE (Collada 컬러 메시) 모두 포함
                # 설치 위치: install/share/my_robot_description/meshes/
                # → URDF의 package://my_robot_description/meshes/ 가 여기를 참조
        
        # (5) Launch 파일들
        ('share/' + package_name + '/launch', glob('launch/*.launch.py') + glob('launch/*.launch.xml')),
            # Python Launch 파일 (.launch.py)
            # XML Launch 파일 (.launch.xml) → 선택적
        
        # (6) RViz 설정 파일
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        # RViz 레이아웃/디스플레이 설정
        
        # (7) Config 파일 (YAML 등)
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    
    install_requires=['setuptools'],
    
    zip_safe=True,
    
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='URDF description for my mobile robot',
    license='MIT',
    
    tests_require=['pytest'],
    
    entry_points={
        'console_scripts': [
            # 실행 가능한 Python 노드가 있으면 여기에 등록
                # 예: 'my_node = my_robot_description.my_node:main',
            'static_tf_broadcaster_example = my_robot_description.static_tf_broadcaster_example:main',    
            'tf_listener_example = my_robot_description.tf_listener_example:main',    
            'odom_publisher_example = my_robot_description.odom_publisher_example:main',    
        ],
    },
)