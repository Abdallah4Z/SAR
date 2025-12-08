#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition
import math

class VisionBridge(Node):
    def __init__(self):
        super().__init__('vision_bridge')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to PX4 position (to get current state)
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.pos_callback,
            qos)
        
        # Publish vision position to EKF2
        self.vision_pub = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            qos)
        
        self.timer = self.create_timer(0.02, self.publish_vision)
        self.current_pos = [0.0, 0.0, 0.0]
        self.get_logger().info('Vision position bridge started')
        
    def pos_callback(self, msg):
        self.current_pos = [msg.x, msg.y, msg.z]
        
    def publish_vision(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        msg = VehicleOdometry()
        msg.timestamp = timestamp
        msg.timestamp_sample = timestamp
        
        # Position
        msg.position = self.current_pos
        msg.q = [1.0, 0.0, 0.0, 0.0]
        
        # Velocity
        msg.velocity = [0.0, 0.0, 0.0]
        
        # Variance (low = high confidence)
        msg.position_variance = [0.001, 0.001, 0.001]
        msg.orientation_variance = [0.001, 0.001, 0.001]
        msg.velocity_variance = [0.1, 0.1, 0.1]
        
        # Quality
        msg.quality = 100
        
        self.vision_pub.publish(msg)

rclpy.init()
node = VisionBridge()
rclpy.spin(node)
