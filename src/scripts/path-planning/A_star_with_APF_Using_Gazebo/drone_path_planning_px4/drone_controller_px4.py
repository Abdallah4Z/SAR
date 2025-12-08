#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String, Float32MultiArray
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition
import numpy as np
import json

from .utils.apf import APFController


class DroneControllerPX4(Node):
    def __init__(self):
        super().__init__('drone_controller_px4')
        
        self.declare_parameter('flight_altitude', 7.0)
        self.declare_parameter('goal_threshold', 1.5)
        self.declare_parameter('obstacle_detection_range', 4.0)
        self.declare_parameter('apf_activation_distance', 6.0)
        
        self.flight_altitude = self.get_parameter('flight_altitude').value
        self.goal_threshold = self.get_parameter('goal_threshold').value
        self.obstacle_detection_range = self.get_parameter('obstacle_detection_range').value
        self.apf_activation_distance = self.get_parameter('apf_activation_distance').value
        
        # APF Controller
        self.apf = APFController()
        
        # State
        self.current_position = None
        self.goal = np.array([20.0, 10.0])
        self.astar_path = []
        self.obstacles = []
        self.path_index = 0
        self.is_navigating = False
        self.offboard_setpoint_counter = 0
        self.using_apf = False
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # PX4 Publishers
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        self.status_pub = self.create_publisher(String, '/drone/status', 10)
        
        # Subscribers
        self.position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.position_callback, qos_profile)
        
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        self.astar_path_sub = self.create_subscription(
            Path, '/drone/astar_path', self.astar_path_callback, 10)
        
        self.obstacle_sub = self.create_subscription(
            Float32MultiArray, '/drone/obstacles', self.obstacle_callback, 10)
        
        self.start_nav_sub = self.create_subscription(
            Bool, '/drone/start_navigation', self.start_navigation_callback, 10)
        
        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('🚁 Hybrid A* + APF Drone Controller initialized')
        self.get_logger().info(f'📍 APF activation distance: {self.apf_activation_distance}m')
        
    def position_callback(self, msg):
        self.current_position = np.array([msg.x, msg.y, msg.z])
        
    def goal_callback(self, msg):
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])
        self.path_index = 0
        self.get_logger().info(f'🎯 New goal: {self.goal}')
        
    def astar_path_callback(self, msg):
        """Receive A* path"""
        self.astar_path = [
            np.array([pose.pose.position.x, pose.pose.position.y])
            for pose in msg.poses
        ]
        self.path_index = 0
        self.get_logger().info(f'📍 Received A* path with {len(self.astar_path)} waypoints')
        
    def obstacle_callback(self, msg):
        """Receive obstacle positions"""
        data = msg.data
        self.obstacles = []
        
        for i in range(0, len(data), 4):
            obs = {
                'position': np.array([data[i], data[i+1]]),
                'radius': data[i+2],
                'height': data[i+3]
            }
            self.obstacles.append(obs)
        
        self.get_logger().info(f'🧱 Received {len(self.obstacles)} obstacles', once=True)
        
    def start_navigation_callback(self, msg):
        if msg.data and not self.is_navigating:
            self.get_logger().info('🚀 Starting hybrid navigation!')
            self.is_navigating = True
            self.offboard_setpoint_counter = 0
        elif not msg.data:
            self.is_navigating = False
            self.land()
            
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
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
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)
        
    def publish_trajectory_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(-z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)
        
    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('🛬 Landing...')
        
    def detect_nearby_obstacles(self):
        """Check if any obstacles are within detection range"""
        if self.current_position is None or len(self.obstacles) == 0:
            return []
        
        current_2d = self.current_position[:2]
        nearby = []
        
        for obs in self.obstacles:
            dist = np.linalg.norm(current_2d - obs['position'])
            if dist < self.apf_activation_distance:
                nearby.append(obs)
        
        return nearby
        
    def control_loop(self):
        if not self.is_navigating or self.current_position is None:
            return
        
        self.publish_offboard_control_mode()
        
        # Enable offboard mode
        if self.offboard_setpoint_counter == 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.get_logger().info('✅ Offboard mode enabled')
        
        # Send initial setpoints
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1
            self.publish_trajectory_setpoint(
                self.current_position[0],
                self.current_position[1],
                self.flight_altitude
            )
            return
        
        current_2d = self.current_position[:2]
        goal_dist = np.linalg.norm(current_2d - self.goal)
        
        # Check if reached goal
        if goal_dist < self.goal_threshold:
            self.get_logger().info(f'🎯 Goal reached! Distance: {goal_dist:.2f}m')
            self.is_navigating = False
            self.land()
            return
        
        # HYBRID ALGORITHM: A* + APF
        nearby_obstacles = self.detect_nearby_obstacles()
        
        # Get current target from A* path
        if len(self.astar_path) > 0 and self.path_index < len(self.astar_path):
            target = self.astar_path[min(self.path_index + 1, len(self.astar_path) - 1)]
        else:
            target = self.goal
        
        # Check if path to target is blocked
        path_blocked = False
        for obs in nearby_obstacles:
            # Vector from current to target
            to_target = target - current_2d
            to_target_dist = np.linalg.norm(to_target)
            
            if to_target_dist > 0.1:
                to_target_normalized = to_target / to_target_dist
                
                # Vector from current to obstacle
                to_obs = obs['position'] - current_2d
                
                # Project obstacle onto path
                projection = np.dot(to_obs, to_target_normalized)
                
                # If obstacle is ahead on our path
                if 0 < projection < to_target_dist:
                    closest_point = current_2d + projection * to_target_normalized
                    dist_to_path = np.linalg.norm(obs['position'] - closest_point)
                    
                    if dist_to_path < (obs['radius'] + 2.0):
                        path_blocked = True
                        break
        
        if path_blocked and len(nearby_obstacles) > 0:
            # PATH IS BLOCKED - USE APF
            if not self.using_apf:
                self.get_logger().info(f'⚠️  Path blocked by {len(nearby_obstacles)} obstacles! Using APF')
                self.using_apf = True
            
            obstacle_positions = [obs['position'] for obs in nearby_obstacles]
            force = self.apf.calculate_force(current_2d, target, obstacle_positions)
            
            force_mag = np.linalg.norm(force)
            if force_mag > 0.1:
                step_size = 0.8
                next_pos = current_2d + step_size * force / force_mag
            else:
                # Move perpendicular to nearest obstacle
                nearest_obs = min(nearby_obstacles, key=lambda o: np.linalg.norm(o['position'] - current_2d))
                to_obs = current_2d - nearest_obs['position']
                perpendicular = np.array([-to_obs[1], to_obs[0]])
                next_pos = current_2d + 1.0 * perpendicular / (np.linalg.norm(perpendicular) + 1e-6)
            
            self.publish_trajectory_setpoint(
                float(next_pos[0]),
                float(next_pos[1]),
                self.flight_altitude,
                0.0
            )
            
            if self.offboard_setpoint_counter % 20 == 0:
                self.get_logger().info(f'🔄 APF MODE: Avoiding obstacles')
        
        else:
            # PATH IS CLEAR - FOLLOW A* 
            if self.using_apf:
                self.get_logger().info('✅ Path clear! Returning to A* path')
                self.using_apf = False
            
            if len(self.astar_path) > 0:
                # Advance through waypoints
                while self.path_index < len(self.astar_path):
                    waypoint = self.astar_path[self.path_index]
                    dist_to_waypoint = np.linalg.norm(current_2d - waypoint)
                    
                    if dist_to_waypoint < 2.0:
                        self.path_index += 1
                        self.get_logger().info(f'✅ Waypoint {self.path_index}/{len(self.astar_path)}')
                    else:
                        break
                
                target_idx = min(self.path_index, len(self.astar_path) - 1)
                target = self.astar_path[target_idx]
                
                self.publish_trajectory_setpoint(
                    float(target[0]),
                    float(target[1]),
                    self.flight_altitude,
                    0.0
                )
                
                if self.offboard_setpoint_counter % 20 == 0:
                    self.get_logger().info(f'📍 A* MODE: Waypoint {self.path_index}/{len(self.astar_path)}')
            else:
                self.publish_trajectory_setpoint(
                    float(self.goal[0]),
                    float(self.goal[1]),
                    self.flight_altitude,
                    0.0
                )
            
    def publish_status(self):
        connected = self.current_position is not None
        current_2d = self.current_position[:2] if connected else None
        goal_dist = np.linalg.norm(current_2d - self.goal) if current_2d is not None else None
        
        status = {
            'connected': connected,
            'navigating': self.is_navigating,
            'mode': 'APF' if self.using_apf else 'A*',
            'position': self.current_position.tolist() if connected else None,
            'goal': self.goal.tolist(),
            'distance_to_goal': float(goal_dist) if goal_dist is not None else None,
            'path_waypoints': len(self.astar_path),
            'current_waypoint': self.path_index
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
