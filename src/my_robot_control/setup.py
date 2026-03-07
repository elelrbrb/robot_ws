import os
from glob import glob
from setuptools import setup

package_name = 'my_robot_control'

setup(
    name=package_name,
    version='0.1.0',
    
    packages=[package_name],

    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
            
        # Launch 파일들
        ('share/' + package_name + '/launch', glob('launch/*.launch.py') + glob('launch/*.launch.xml')),
                  
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
            'safety_layer_node = my_robot_control.safety_layer_node:main',
            'e_stop_node = my_robot_control.e_stop_node:main',
            'ekf_ros_node = my_robot_control.ekf_ros_node:main',

            'test_driver= my_robot_control.test_driver:main', 

            'path_publisher_node= my_robot_control.path_publisher_node:main', 
            'pure_pursuit_node= my_robot_control.pure_pursuit_node:main', 

            'figure8_challenge= my_robot_control.figure8_challenge:main', 


 
            

                

        ],
    },
)