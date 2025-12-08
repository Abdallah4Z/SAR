#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        
        self.timer = self.create_timer(0.02, self.publish)  # 50 Hz
        self.get_logger().info('Publishing velocity commands at 50 Hz')
        
    def publish(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Offboard control mode - VELOCITY
        offboard = OffboardControlMode()
        offboard.timestamp = timestamp
        offboard.position = False
        offboard.velocity = True
        offboard.acceleration = False
        offboard.attitude = False
        offboard.body_rate = False
        self.offboard_pub.publish(offboard)
        
        # Velocity setpoint - climb at 1 m/s
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = timestamp
        setpoint.velocity = [0.0, 0.0, -1.0]  # NED: negative Z = up
        self.setpoint_pub.publish(setpoint)

rclpy.init()
node = VelocityPublisher()
rclpy.spin(node)
