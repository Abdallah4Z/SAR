from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Generate obstacles
        Node(
            package='drone_path_planning_px4',
            executable='obstacle_generator',
            name='obstacle_generator',
            output='screen',
            parameters=[{
                'num_obstacles': 6,
                'min_height': 6.0,
                'max_height': 10.0,
            }]
        ),
        
        # 2. Spawn in Gazebo
        Node(
            package='drone_path_planning_px4',
            executable='spawn_gazebo_obstacles',
            name='gazebo_spawner',
            output='screen'
        ),
        
        # 3. Path planner
        Node(
            package='drone_path_planning_px4',
            executable='path_planner_node',
            name='path_planner',
            output='screen',
            parameters=[{
                'grid_resolution': 0.5,
                'grid_size_x': 50.0,
                'grid_size_y': 50.0,
                'safety_distance': 3.0,
            }]
        ),
        
        # 4. Drone controller
        Node(
            package='drone_path_planning_px4',
            executable='drone_controller_px4',
            name='drone_controller',
            output='screen',
            parameters=[{
                'flight_altitude': 7.0,
                'goal_threshold': 1.5,
                'obstacle_detection_range': 6.0,      # Increased
                'apf_activation_distance': 6.0, 
            }]
        ),
        
        # 5. Visualization
        Node(
            package='drone_path_planning_px4',
            executable='visualization_node',
            name='visualization',
            output='screen'
        ),
    ])
