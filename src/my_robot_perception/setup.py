from setuptools import find_packages, setup

package_name = 'my_robot_perception'

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
            'laser_scan_analyzer = my_robot_perception.laser_scan_analyzer:main',
            'lidar_processor_node = my_robot_perception.lidar_processor_node:main',
            'depth_processor_node = my_robot_perception.depth_processor_node:main',
                
        ],
    },
)