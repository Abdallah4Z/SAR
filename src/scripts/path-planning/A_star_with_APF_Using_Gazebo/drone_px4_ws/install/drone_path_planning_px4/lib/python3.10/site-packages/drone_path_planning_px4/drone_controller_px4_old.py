#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, String
import numpy as np
import asyncio
import json
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

from .utils.apf import APFController


class DroneControllerPX4(Node):
    def __init__(self):
        super().__init__('drone_controller_px4')
        
        # Parameters
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('flight_altitude', 5.0)
        self.declare_parameter('goal_threshold', 2.0)
        self.declare_parameter('control_rate', 10.0)
        
        self.max_speed = self.get_parameter('max_speed').value
        self.flight_altitude = self.get_parameter('flight_altitude').value
        self.goal_threshold = self.get_parameter('goal_threshold').value
        control_rate = self.get_parameter('control_rate').value
        
        # APF controller
        self.apf = APFController()
        
        # State
        self.current_position = None
        self.goal = None
        self.a_star_path = []
        self.obstacles = []
        self.is_navigating = False
        self.path_index = 0
        
        # MAVSDK drone
        self.drone = System()
        self.connected = False
        
        # Publishers
        self.hybrid_path_pub = self.create_publisher(Path, '/drone/hybrid_path', 10)
        self.status_pub = self.create_publisher(String, '/drone/status', 10)
        
        # Subscribers
        self.target_sub = self.create_subscription(
            PoseStamped, '/drone/target_pose', self.target_callback, 10)
        self.astar_path_sub = self.create_subscription(
            Path, '/drone/astar_path', self.astar_path_callback, 10)
        self.start_nav_sub = self.create_subscription(
            Bool, '/drone/start_navigation', self.start_navigation_callback, 10)
        
        # Timers
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Start async tasks
        self.loop = asyncio.get_event_loop()
        self.loop.create_task(self.connect_drone())
        
        self.get_logger().info('PX4 Drone Controller initialized')
        
    async def connect_drone(self):
        """Connect to PX4 via MAVSDK"""
        self.get_logger().info('Connecting to PX4...')
        await self.drone.connect(system_address="udp://:14540")
        
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info('PX4 Connected!')
                self.connected = True
                break
        
        # Start telemetry
        self.loop.create_task(self.get_position())
        
    async def get_position(self):
        """Get drone position from PX4"""
        async for position in self.drone.telemetry.position():
            self.current_position = np.array([
                position.latitude_deg,   # Convert to local coordinates in real implementation
                position.longitude_deg,
                position.absolute_altitude_m
            ])
            
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
        self.is_navigating = msg.data
        if self.is_navigating and self.connected:
            self.get_logger().info('Starting navigation!')
            self.loop.create_task(self.navigate())
        else:
            self.get_logger().info('Navigation stopped')
            
    async def navigate(self):
        """Main navigation loop using PX4 offboard mode"""
        # Arm and takeoff
        self.get_logger().info('Arming and taking off...')
        await self.drone.action.arm()
        await self.drone.action.set_takeoff_altitude(self.flight_altitude)
        await self.drone.action.takeoff()
        
        # Wait for takeoff
        await asyncio.sleep(5)
        
        # Start offboard mode
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -self.flight_altitude, 0.0))
        
        try:
            await self.drone.offboard.start()
        except OffboardError as error:
            self.get_logger().error(f'Offboard mode failed: {error}')
            return
        
        # Navigation loop
        while self.is_navigating and rclpy.ok():
            if self.current_position is None or self.goal is None:
                await asyncio.sleep(0.1)
                continue
            
            current_2d = self.current_position[:2]
            
            # Check if goal reached
            goal_dist = np.linalg.norm(current_2d - self.goal)
            if goal_dist < self.goal_threshold:
                self.get_logger().info('Goal reached!')
                break
            
            # Calculate next position using APF
            if len(self.a_star_path) > 0:
                # Find target on path
                target_idx = min(self.path_index + 5, len(self.a_star_path) - 1)
                target = self.a_star_path[target_idx]
                
                # Calculate force
                force = self.apf.calculate_force(current_2d, target, self.obstacles)
                
                # Calculate next position
                next_pos = current_2d + 0.1 * force
                
                # Send to PX4 (NED coordinates)
                await self.drone.offboard.set_position_ned(
                    PositionNedYaw(
                        float(next_pos[0]),
                        float(next_pos[1]),
                        -self.flight_altitude,
                        0.0
                    ))
            
            await asyncio.sleep(0.1)
        
        # Land
        self.get_logger().info('Landing...')
        await self.drone.action.land()
        
    def publish_status(self):
        """Publish status"""
        status = {
            'connected': self.connected,
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
