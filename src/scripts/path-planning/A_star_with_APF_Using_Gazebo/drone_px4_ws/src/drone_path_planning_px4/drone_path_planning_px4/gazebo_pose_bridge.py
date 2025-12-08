#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleVisualOdometry
from geometry_msgs.msg import PoseStamped
import subprocess
import re

class GazeboPoseBridge(Node):
    def __init__(self):
        super().__init__('gazebo_pose_bridge')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.visual_odom_pub = self.create_publisher(
            VehicleVisualOdometry, '/fmu/in/vehicle_visual_odometry', qos)
        
        self.timer = self.create_timer(0.02, self.publish_pose)  # 50 Hz
        self.get_logger().info('Gazebo Pose Bridge started')
        
    def get_gazebo_pose(self):
        try:
            # Get pose from Gazebo
            result = subprocess.run(
                ['gz', 'model', '-m', 'x500_0', '-p'],
                capture_output=True, text=True, timeout=0.1
            )
            
            if result.returncode == 0:
                # Parse position
                pos_match = re.search(r'Pose: ([-\d.]+) ([-\d.]+) ([-\d.]+)', result.stdout)
                if pos_match:
                    x = float(pos_match.group(1))
                    y = float(pos_match.group(2))
                    z = float(pos_match.group(3))
                    return [x, y, z]
        except:
            pass
        return None
        
    def publish_pose(self):
        pose = self.get_gazebo_pose()
        
        if pose is not None:
            msg = VehicleVisualOdometry()
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            msg.timestamp_sample = msg.timestamp
            msg.position = pose
            msg.velocity = [0.0, 0.0, 0.0]
            msg.q = [1.0, 0.0, 0.0, 0.0]
            msg.position_variance = [0.01, 0.01, 0.01]
            msg.orientation_variance = [0.01, 0.01, 0.01]
            msg.velocity_variance = [0.1, 0.1, 0.1]
            
            self.visual_odom_pub.publish(msg)

def main():
    rclpy.init()
    node = GazeboPoseBridge()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
