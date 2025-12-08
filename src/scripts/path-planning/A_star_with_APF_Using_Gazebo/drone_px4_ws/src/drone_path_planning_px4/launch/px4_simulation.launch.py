#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('drone_path_planning_px4')
    world_file = os.path.join(pkg_dir, 'worlds', 'obstacle_world.sdf')
    
    # PX4 SITL
    px4_dir = os.path.expanduser('~/PX4-Autopilot')
    
    px4_sitl = ExecuteProcess(
        cmd=[
            'make', 'px4_sitl',
            f'gz_x500__obstacle_world'
        ],
        cwd=px4_dir,
        output='screen',
        shell=True,
        env={'GZ_SIM_RESOURCE_PATH': f'{pkg_dir}/worlds:{px4_dir}/Tools/simulation/gz/worlds'}
    )
    
    # MicroXRCEAgent for PX4-ROS2 communication
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )
    
    return LaunchDescription([
        px4_sitl,
        micro_xrce_agent,
    ])
