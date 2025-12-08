#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32MultiArray
import numpy as np
from .utils.astar import AStarPlanner


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        
        self.declare_parameter('grid_resolution', 0.5)
        self.declare_parameter('grid_size_x', 50.0)
        self.declare_parameter('grid_size_y', 50.0)
        self.declare_parameter('safety_distance', 3.0)
        
        self.grid_resolution = self.get_parameter('grid_resolution').value
        self.grid_size_x = self.get_parameter('grid_size_x').value
        self.grid_size_y = self.get_parameter('grid_size_y').value
        self.safety_distance = self.get_parameter('safety_distance').value
        
        self.obstacles = []
        self.start = (0.0, 0.0)
        self.goal = (20.0, 10.0)
        self.planner = AStarPlanner(self.grid_resolution, self.safety_distance)
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/drone/astar_path', 10)
        
        # Subscribers
        self.obstacle_sub = self.create_subscription(
            Float32MultiArray, '/drone/obstacles', self.obstacle_callback, 10)
        
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        self.start_planning_sub = self.create_subscription(
            Bool, '/drone/start_planning', self.start_planning_callback, 10)
        
        self.get_logger().info('🗺️  Path Planner Node initialized')
        self.get_logger().info(f'Grid: {self.grid_size_x}x{self.grid_size_y}m, resolution: {self.grid_resolution}m')
        
    def obstacle_callback(self, msg):
        """Receive obstacles: [x, y, radius, height, ...]"""
        data = msg.data
        self.obstacles = []
        
        for i in range(0, len(data), 4):
            obs = {
                'x': data[i],
                'y': data[i+1],
                'radius': data[i+2],
                'height': data[i+3]
            }
            self.obstacles.append(obs)
        
        self.get_logger().info(f'📍 Received {len(self.obstacles)} obstacles', once=True)
        
    def goal_callback(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'🎯 Goal updated: {self.goal}')
        
    def start_planning_callback(self, msg):
        if not msg.data:
            return
        
        if len(self.obstacles) == 0:
            self.get_logger().warn('⚠️  No obstacles received yet!')
            return
        
        self.get_logger().info(f'🚀 Planning path from {self.start} to {self.goal}')
        self.get_logger().info(f'🧱 Avoiding {len(self.obstacles)} obstacles')
        
        # Plan path with A*
        path = self.planner.plan(self.start, self.goal, self.obstacles)
        
        if path is None or len(path) == 0:
            self.get_logger().error('❌ No path found!')
            return
        
        self.get_logger().info(f'✅ Path found with {len(path)} waypoints')
        
        # Publish path
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for waypoint in path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(waypoint[0])
            pose.pose.position.y = float(waypoint[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        self.get_logger().info('📡 Published A* path')


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
