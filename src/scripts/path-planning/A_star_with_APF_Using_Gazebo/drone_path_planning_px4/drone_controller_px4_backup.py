#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition
import numpy as np
import json

from .utils.apf import APFController


class DroneControllerPX4(Node):
    def __init__(self):
        super().__init__('drone_controller_px4')
        
        self.declare_parameter('flight_altitude', 5.0)
        self.declare_parameter('goal_threshold', 2.0)
        
        self.flight_altitude = self.get_parameter('flight_altitude').value
        self.goal_threshold = self.get_parameter('goal_threshold').value
        
        self.apf = APFController()
        
        self.current_position = None
        self.goal = None
        self.a_star_path = []
        self.obstacles = []
        self.is_navigating = False
        self.path_index = 0
        self.offboard_setpoint_counter = 0
        
        # PX4 QoS profile - THIS IS THE KEY FIX!
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers for PX4 (use QoS)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        # ROS2 Publishers (standard QoS)
        self.hybrid_path_pub = self.create_publisher(Path, '/drone/hybrid_path', 10)
        self.status_pub = self.create_publisher(String, '/drone/status', 10)
        
        # Subscribers (use PX4 QoS for position!)
        self.position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.position_callback, qos_profile)
        self.target_sub = self.create_subscription(
            PoseStamped, '/drone/target_pose', self.target_callback, 10)
        self.astar_path_sub = self.create_subscription(
            Path, '/drone/astar_path', self.astar_path_callback, 10)
        self.start_nav_sub = self.create_subscription(
            Bool, '/drone/start_navigation', self.start_navigation_callback, 10)
        
        # Timers
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('PX4 Drone Controller initialized')
        self.get_logger().info('Waiting for position data from PX4...')
        
    def position_callback(self, msg):
        """Update current position from PX4"""
        self.current_position = np.array([msg.x, msg.y, msg.z])
        if self.current_position is not None:
            self.get_logger().info(f'✅ Position received: {self.current_position}', once=True)
        
    def target_callback(self, msg):
        """Receive goal position"""
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])
        self.get_logger().info(f'Goal updated: {self.goal}')
        
    def astar_path_callback(self, msg):
        """Receive A* reference path"""
        self.a_star_path = [
            np.array([pose.pose.position.x, pose.pose.position.y])
            for pose in msg.poses
        ]
        self.get_logger().info(f'Received A* path: {len(self.a_star_path)} waypoints')
        
    def start_navigation_callback(self, msg):
        """Start/stop navigation"""
        if msg.data and not self.is_navigating:
            self.get_logger().info('🚀 Starting navigation sequence!')
            self.is_navigating = True
            self.offboard_setpoint_counter = 0
            self.arm()
        elif not msg.data:
            self.is_navigating = False
            self.land()
            
    def arm(self):
        """Send arm command to PX4"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('✅ Arm command sent')
        
    def disarm(self):
        """Send disarm command to PX4"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('Disarm command sent')
        
    def land(self):
        """Send land command to PX4"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('🛬 Land command sent')
        
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Publish a vehicle command"""
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)
        
    def publish_offboard_control_mode(self):
        """Publish offboard control mode"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)
        
    def publish_trajectory_setpoint(self, x, y, z, yaw=0.0):
        """Publish position setpoint"""
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(-z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)
        
    def control_loop(self):
        """Main control loop"""
        if not self.is_navigating or self.current_position is None:
            return
        
        self.publish_offboard_control_mode()
        
        if self.offboard_setpoint_counter == 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.get_logger().info('🚁 Offboard mode enabled')
        
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1
            self.publish_trajectory_setpoint(
                self.current_position[0],
                self.current_position[1],
                self.flight_altitude
            )
            return
        
        if self.goal is not None:
            current_2d = self.current_position[:2]
            goal_dist = np.linalg.norm(current_2d - self.goal)
            
            if goal_dist < self.goal_threshold:
                self.get_logger().info('🎯 Goal reached! Landing...')
                self.is_navigating = False
                self.land()
                return
        
        if len(self.a_star_path) > 0 and self.goal is not None:
            current_2d = self.current_position[:2]
            
            target_idx = min(self.path_index + 5, len(self.a_star_path) - 1)
            target = self.a_star_path[target_idx]
            
            force = self.apf.calculate_force(current_2d, target, self.obstacles)
            
            next_pos = current_2d + 0.5 * force
            
            self.publish_trajectory_setpoint(
                float(next_pos[0]),
                float(next_pos[1]),
                self.flight_altitude,
                0.0
            )
            
    def publish_status(self):
        """Publish status"""
        connected = self.current_position is not None
        status = {
            'connected': connected,
            'navigating': self.is_navigating,
            'position': self.current_position.tolist() if self.current_position is not None else None,
            'goal': self.goal.tolist() if self.goal is not None else None
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneControllerPX4()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
