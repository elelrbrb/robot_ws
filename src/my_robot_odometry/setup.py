import os
from glob import glob
from setuptools import setup

package_name = 'my_robot_odometry'

setup(
    name=package_name,
    version='0.1.0',
    
    packages=[package_name],
    
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

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
            'odom_publisher_example = my_robot_odometry.odom_publisher_example:main',
                
        ],
    },
)