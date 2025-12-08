from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'drone_path_planning_px4'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models'), glob('models/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='belal212',
    maintainer_email='belal212@example.com',
    description='PX4 drone path planning with A* and APF',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_controller_px4 = drone_path_planning_px4.drone_controller_px4:main',
            'path_planner_node = drone_path_planning_px4.path_planner_node:main',
            'obstacle_manager_node = drone_path_planning_px4.obstacle_manager_node:main',
            'visualization_node = drone_path_planning_px4.visualization_node:main',
            'gazebo_pose_bridge = drone_path_planning_px4.gazebo_pose_bridge:main',
            'obstacle_generator = drone_path_planning_px4.obstacle_generator:main',
            'spawn_gazebo_obstacles = drone_path_planning_px4.spawn_gazebo_obstacles:main',
        ],
    },
)
