#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleAttitudeSetpoint

class AttitudePublisher(Node):
    def __init__(self):
        super().__init__('attitude_publisher')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos)
        
        self.timer = self.create_timer(0.02, self.publish)
        self.get_logger().info('Publishing attitude/thrust commands at 50 Hz')
        
    def publish(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Offboard control mode - ATTITUDE
        offboard = OffboardControlMode()
        offboard.timestamp = timestamp
        offboard.position = False
        offboard.velocity = False
        offboard.acceleration = False
        offboard.attitude = True
        offboard.body_rate = False
        self.offboard_pub.publish(offboard)
        
        # Attitude setpoint - level hover with thrust
        setpoint = VehicleAttitudeSetpoint()
        setpoint.timestamp = timestamp
        setpoint.q_d = [1.0, 0.0, 0.0, 0.0]  # Level attitude (quaternion)
        setpoint.thrust_body = [0.0, 0.0, -0.7]  # 70% thrust downward (NED)
        self.setpoint_pub.publish(setpoint)

rclpy.init()
node = AttitudePublisher()
rclpy.spin(node)
