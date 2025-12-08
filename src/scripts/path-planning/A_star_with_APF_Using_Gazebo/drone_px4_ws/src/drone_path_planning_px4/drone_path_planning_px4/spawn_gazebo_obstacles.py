#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import subprocess
import time
import os

class GazeboObstacleSpawner(Node):
    def __init__(self):
        super().__init__('gazebo_obstacle_spawner')
        
        self.spawned = False
        
        self.obstacle_sub = self.create_subscription(
            Float32MultiArray, '/drone/obstacles', self.spawn_obstacles, 10)
        
        self.get_logger().info('🧱 Waiting for obstacles to spawn in Gazebo...')
        
    def spawn_obstacles(self, msg):
        if self.spawned:
            return
        
        obstacles = msg.data
        num_obs = len(obstacles) // 4
        
        self.get_logger().info(f'Spawning {num_obs} red cylinders in Gazebo...')
        
        for i in range(num_obs):
            x = obstacles[i*4]
            y = obstacles[i*4 + 1]
            radius = obstacles[i*4 + 2]
            height = obstacles[i*4 + 3]
            
            # Create SDF model
            sdf_content = f'''<?xml version="1.0"?>
<sdf version="1.8">
  <model name="obstacle_{i}">
    <static>true</static>
    <pose>{x} {y} {height/2} 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <specular>0.5 0.5 0.5 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''
            
            # Save SDF file
            sdf_path = f'/tmp/obstacle_{i}.sdf'
            with open(sdf_path, 'w') as f:
                f.write(sdf_content)
            
            # Spawn using gz service
            cmd = [
                'gz', 'service', '-s', '/world/default/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req', f'sdf_filename: "{sdf_path}"'
            ]
            
            try:
                subprocess.run(cmd, check=True, capture_output=True)
                self.get_logger().info(f'✅ Spawned obstacle {i} at ({x:.1f}, {y:.1f}, h={height:.1f}m)')
            except Exception as e:
                self.get_logger().warn(f'⚠️  Failed to spawn obstacle {i}: {e}')
            
            time.sleep(0.2)
        
        self.spawned = True
        self.get_logger().info('🎉 All obstacles spawned!')

def main():
    rclpy.init()
    node = GazeboObstacleSpawner()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
