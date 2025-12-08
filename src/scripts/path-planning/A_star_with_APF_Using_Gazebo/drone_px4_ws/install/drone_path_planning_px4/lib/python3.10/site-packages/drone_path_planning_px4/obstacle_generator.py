#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import json
import random

class ObstacleGenerator(Node):
    def __init__(self):
        super().__init__('obstacle_generator')
        
        self.declare_parameter('num_obstacles', 8)
        self.declare_parameter('min_height', 5.0)
        self.declare_parameter('max_height', 12.0)
        
        num_obs = self.get_parameter('num_obstacles').value
        min_h = self.get_parameter('min_height').value
        max_h = self.get_parameter('max_height').value
        
        # Publish to BOTH topics
        self.obstacle_info_pub = self.create_publisher(String, '/drone/obstacle_info', 10)
        self.obstacle_list_pub = self.create_publisher(Float32MultiArray, '/drone/obstacles', 10)
        
        # Generate obstacles
        self.obstacles = []
        for i in range(num_obs):
            obs = {
                'x': random.uniform(5.0, 20.0),
                'y': random.uniform(-5.0, 15.0),
                'z': 0.0,
                'radius': random.uniform(1.5, 3.0),
                'height': random.uniform(min_h, max_h)
            }
            self.obstacles.append(obs)
        
        # Timer to publish
        self.timer = self.create_timer(1.0, self.publish_obstacles)
        
        self.get_logger().info(f'🧱 Generated {num_obs} obstacles')
        for i, obs in enumerate(self.obstacles):
            self.get_logger().info(f'  Obstacle {i}: pos=({obs["x"]:.1f}, {obs["y"]:.1f}), r={obs["radius"]:.1f}m, h={obs["height"]:.1f}m')
        
    def publish_obstacles(self):
        # Publish as JSON for obstacle_manager
        info_msg = String()
        info_msg.data = json.dumps({'obstacles': self.obstacles})
        self.obstacle_info_pub.publish(info_msg)
        
        # Publish as float array for path_planner
        list_msg = Float32MultiArray()
        flat_list = []
        for obs in self.obstacles:
            flat_list.extend([obs['x'], obs['y'], obs['radius'], obs['height']])
        list_msg.data = flat_list
        self.obstacle_list_pub.publish(list_msg)
        
        self.get_logger().info(f'📡 Publishing {len(self.obstacles)} obstacles', once=True)

def main():
    rclpy.init()
    node = ObstacleGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
