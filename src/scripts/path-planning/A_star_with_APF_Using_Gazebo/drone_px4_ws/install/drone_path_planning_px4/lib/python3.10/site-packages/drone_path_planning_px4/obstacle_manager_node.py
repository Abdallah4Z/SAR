#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # ✅ CORRECT
from visualization_msgs.msg import Marker, MarkerArray
import json
import subprocess
import time


class ObstacleManagerNode(Node):
    def __init__(self):
        super().__init__('obstacle_manager_node')
        
        self.obstacles_spawned = False
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/drone/obstacle_markers', 10)
        
        # Subscribers
        self.obstacle_info_sub = self.create_subscription(
            String, '/drone/obstacle_info', self.obstacle_info_callback, 10)
        
        self.get_logger().info('Obstacle Manager Node initialized')
        
    def obstacle_info_callback(self, msg):
        """Receive obstacle info and spawn in Gazebo"""
        if self.obstacles_spawned:
            return
        
        try:
            data = json.loads(msg.data)
            obstacles = data['obstacles']
            
            self.get_logger().info(f'Spawning {len(obstacles)} obstacles in Gazebo...')
            
            for i, obs in enumerate(obstacles):
                self.spawn_obstacle(i, obs)
                time.sleep(0.5)  # Wait between spawns
            
            self.obstacles_spawned = True
            self.get_logger().info('All obstacles spawned!')
            
            # Publish markers for RViz
            self.publish_markers(obstacles)
            
        except Exception as e:
            self.get_logger().error(f'Error spawning obstacles: {e}')
    
    def spawn_obstacle(self, index, obs):
        """Spawn single obstacle in Gazebo"""
        name = f"obstacle_{index}"
        x, y, z = obs['x'], obs['y'], obs['z']
        width, height, depth = obs['width'], obs['height'], obs['depth']
        obs_type = obs['type']
        
        if obs_type == 'box':
            sdf_content = f"""
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>{width} {height} {depth}</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{width} {height} {depth}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.7 0.7 0.7 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
        else:  # cylinder
            radius = width / 2
            sdf_content = f"""
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{depth}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{depth}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.5 0.5 0.5 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
        
        # Write SDF to temp file
        sdf_path = f"/tmp/{name}.sdf"
        with open(sdf_path, 'w') as f:
            f.write(sdf_content)
        
        # Spawn using gz service
        z_adjusted = z + depth/2  # Center the model
        cmd = [
            'gz', 'service', '-s', '/world/default/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000',
            '--req',
            f'sdf_filename: "{sdf_path}", name: "{name}", pose: {{position: {{x: {x}, y: {y}, z: {z_adjusted}}}}}'
        ]
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                self.get_logger().info(f'Spawned {name} at ({x}, {y}, {z})')
            else:
                self.get_logger().error(f'Failed to spawn {name}: {result.stderr}')
        except Exception as e:
            self.get_logger().error(f'Error running spawn command: {e}')
    
    def publish_markers(self, obstacles):
        """Publish RViz markers for obstacles"""
        marker_array = MarkerArray()
        
        for i, obs in enumerate(obstacles):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'obstacles'
            marker.id = i
            marker.type = Marker.CUBE if obs['type'] == 'box' else Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = obs['x']
            marker.pose.position.y = obs['y']
            marker.pose.position.z = obs['z'] + obs['depth']/2
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = obs['width']
            marker.scale.y = obs['height']
            marker.scale.z = obs['depth']
            
            marker.color.r = 0.7
            marker.color.g = 0.7
            marker.color.b = 0.7
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
