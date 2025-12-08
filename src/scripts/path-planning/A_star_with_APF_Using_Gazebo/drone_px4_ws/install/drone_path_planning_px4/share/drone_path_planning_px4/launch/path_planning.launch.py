#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('drone_path_planning_px4')
    config_file = os.path.join(pkg_dir, 'config', 'planner_params.yaml')
    
    # Nodes
    path_planner = Node(
        package='drone_path_planning_px4',
        executable='path_planner',
        name='path_planner_node',
        output='screen',
        parameters=[config_file]
    )
    
    drone_controller = Node(
        package='drone_path_planning_px4',
        executable='drone_controller',
        name='drone_controller_px4',
        output='screen',
        parameters=[config_file]
    )
    
    obstacle_manager = Node(
        package='drone_path_planning_px4',
        executable='obstacle_manager',
        name='obstacle_manager_node',
        output='screen'
    )
    
    visualizer = Node(
        package='drone_path_planning_px4',
        executable='visualizer',
        name='visualization_node',
        output='screen'
    )
    
    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world']
    )
    
    return LaunchDescription([
        path_planner,
        drone_controller,
        obstacle_manager,
        visualizer,
        static_tf,
    ])
