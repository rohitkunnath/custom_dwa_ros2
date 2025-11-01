from setuptools import find_packages, setup
import os

package_name = 'custom_dwa'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch and RViz config files
        (os.path.join('share', package_name, 'launch'), ['launch/dwa_planner_launch.py']),
        (os.path.join('share', package_name, 'rviz_config'), ['rviz_config/rviz_config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ashish',
    maintainer_email='ashish@todo.todo',
    description='Custom DWA local planner for TurtleBot3 in ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dwa_planner = custom_dwa.dwa_planner_node:main',
        ],
    },
)
