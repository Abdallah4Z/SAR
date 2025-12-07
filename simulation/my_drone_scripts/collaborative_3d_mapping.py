#!/usr/bin/env python3
"""
Collaborative 3D Voxel Grid Mapping with Multi-Drone Swarm
5 drones fly coordinated patterns collecting LiDAR data
Builds clean 3D voxel map with colored cubes
Real-time visualization in RViz with structured occupancy grid
"""

import airsim
import numpy as np
import time
import threading
import sys
import struct
from collections import defaultdict

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Header, String, ColorRGBA
from nav_msgs.msg import Odometry
import struct


class Collaborative3DMapper(Node):
    def __init__(self):
        super().__init__('collaborative_3d_mapper')
        
        # Connect to AirSim - main client for setup
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.get_logger().info("✅ Connected to AirSim")
        
        # Create separate clients for each drone to avoid IOLoop conflicts
        self.drone_clients = {}
        
        # Drone configuration
        self.drone_names = ["Drone0", "Drone1", "Drone2", "Drone3", "Drone4"]
        self.lidar_name = "Lidar1"
        
        # Flight parameters
        self.altitude = -30.0  # 30m above ground (NED: negative is up)
        self.velocity = 5.0  # m/s
        
        # Voxel grid parameters for clean visualization
        self.voxel_size = 1.0  # 1 meter voxels
        self.voxel_grid = {}  # Dict of (x,y,z) -> occupancy count
        self.voxel_lock = threading.Lock()  # Protect voxel grid access
        self.voxel_publish_event = threading.Event()  # Signal voxel updates
        
        # Occupancy grid parameters (2D map with SLAM)
        self.grid_resolution = 1.0  # 1 meter per cell for better performance
        self.grid_width = 300  # 300m total width
        self.grid_height = 300  # 300m total height  
        self.grid_origin_x = -150.0  # Center at 0,0
        self.grid_origin_y = -150.0
        
        # Use log-odds for probabilistic mapping (persistent, doesn't disappear)
        self.log_odds_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.float32)
        self.log_odds_occupied = 2.0  # Strong update when obstacle detected
        self.log_odds_free = -0.5  # Moderate update for free space
        self.log_odds_max = 20.0  # Clamp to prevent overflow
        self.log_odds_min = -20.0
        
        # Mapping data structures (keep for compatibility)
        self.global_point_cloud = []
        self.point_cloud_lock = threading.Lock()
        self.max_points = 1000000  # Limit total points for performance
        
        # Per-drone statistics
        self.drone_stats = {
            name: {
                'points_collected': 0,
                'voxels_discovered': 0,
                'position': [0, 0, 0],
                'status': 'initializing',
                'coverage_area': 0.0
            } for name in self.drone_names
        }
        
        # ROS2 Publishers
        self.combined_lidar_pub = self.create_publisher(
            PointCloud2,
            '/collaborative_map/combined_lidar',
            10
        )
        self.occupancy_grid_pub = self.create_publisher(
            OccupancyGrid,
            '/collaborative_map/occupancy_grid',
            10
        )
        self.drone_markers_pub = self.create_publisher(
            MarkerArray,
            '/collaborative_map/drone_positions',
            10
        )
        self.coverage_markers_pub = self.create_publisher(
            MarkerArray,
            '/collaborative_map/coverage_zones',
            10
        )
        self.stats_pub = self.create_publisher(
            String,
            '/collaborative_map/stats',
            10
        )
        
        # ROS2 Subscribers - Listen to each drone's ROS2 bridge LiDAR topic
        self.lidar_subscribers = []
        self.accumulated_points = []  # Store all points from all drones
        self.accumulated_lock = threading.Lock()
        
        for drone_name in self.drone_names:
            topic = f'/airsim_node/{drone_name}/lidar/Lidar1'
            sub = self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, name=drone_name: self.lidar_callback(msg, name),
                10
            )
            self.lidar_subscribers.append(sub)
            self.get_logger().info(f"📡 Subscribed to {topic}")
        
        # Odometry publishers for each drone
        self.odom_pubs = {}
        for drone_name in self.drone_names:
            self.odom_pubs[drone_name] = self.create_publisher(
                Odometry,
                f'/{drone_name}/odometry',
                10
            )
        
        # Simple point cloud publishing timer (like step-by-step demo)
        self.create_timer(1.0, self.publish_point_cloud_simple)  # 1 Hz
        self.create_timer(2.0, self.publish_stats)  # 0.5 Hz
        self.create_timer(2.0, self.publish_drone_poses)  # 0.5 Hz
        # Simple combined LiDAR publishing timer (slower for RViz performance)
        self.create_timer(2.0, self.publish_combined_lidar)  # 0.5 Hz - every 2 seconds
        self.create_timer(3.0, self.publish_occupancy_grid)  # 0.33 Hz - occupancy grid map
        self.create_timer(2.0, self.publish_stats)  # 0.5 Hz
        self.create_timer(1.0, self.publish_drone_markers)  # 1 Hz - drone position markers
        self.create_timer(5.0, self.publish_coverage_zones)  # 0.2 Hz - coverage zones
        
        # Control flags
        self.mapping_active = False
        self.threads = []
        
        self.get_logger().info("✅ Collaborative 3D Mapper initialized")
        self.get_logger().info(f"📡 Subscribed to {len(self.lidar_subscribers)} LiDAR topics")
        self.get_logger().info(f"📤 Publishing to /collaborative_map/combined_lidar")
        self.get_logger().info(f"📤 Publishing to /collaborative_map/drone_positions")
        self.get_logger().info(f"📤 Publishing to /collaborative_map/coverage_zones")
        self.get_logger().info(f"📤 Publishing to /collaborative_map/occupancy_grid")
        
        # Publish initial markers immediately
        self.publish_drone_markers()
        self.publish_coverage_zones()
        
        # Publish empty occupancy grid immediately so RViz sees the topic
        self.publish_occupancy_grid()
    
    def lidar_callback(self, msg, drone_name):
        """Receive LiDAR from ROS2 bridge and accumulate into combined map"""
        try:
            # Log first time we receive data
            self.get_logger().info(f"📡 Received LiDAR from {drone_name}: {len(msg.data)} bytes", throttle_duration_sec=5.0)
            
            # Extract points from PointCloud2 message
            point_step = msg.point_step
            num_points = len(msg.data) // point_step
            
            self.get_logger().info(f"   → {num_points} points from {drone_name}", throttle_duration_sec=5.0)
            
            # Downsample incoming data - only keep every 5th point
            with self.accumulated_lock:
                # Limit total accumulated points to 20k for RViz performance
                if len(self.accumulated_points) >= 20000:
                    # Remove oldest 5000 points
                    self.accumulated_points = self.accumulated_points[5000:]
                
            # Downsample incoming data moderately
            with self.accumulated_lock:
                # Limit total accumulated points to 100k for good detail
                if len(self.accumulated_points) >= 100000:
                    # Remove oldest 20k to make room
                    self.accumulated_points = self.accumulated_points[20000:]
                
                # Only add every 2nd point (keep more detail)
                for i in range(0, len(msg.data), point_step * 2):
                    # Extract x, y, z (first 12 bytes = 3 floats)
                    x, y, z = struct.unpack('fff', msg.data[i:i+12])
                    self.accumulated_points.append((x, y, z))
                    
                    # Update occupancy grid with ray tracing
                    # Use actual drone position from stats
                    if drone_name in self.drone_stats:
                        drone_pos = self.drone_stats[drone_name]['position']
                        self.update_occupancy_grid_with_ray(drone_pos[0], drone_pos[1], x, y, z)
                
                # Update stats
                if drone_name in self.drone_stats:
                    self.drone_stats[drone_name]['points_collected'] = len(self.accumulated_points)
            
        except Exception as e:
            self.get_logger().error(f"LiDAR callback error for {drone_name}: {e}")
    
    def publish_combined_lidar(self):
        """Publish the accumulated combined LiDAR from all drones"""
        with self.accumulated_lock:
            if len(self.accumulated_points) == 0:
                self.get_logger().info("⏳ No accumulated points yet...", throttle_duration_sec=5.0)
                return
            
            self.get_logger().info(f"📤 Publishing {len(self.accumulated_points):,} points to RViz", throttle_duration_sec=3.0)
            
            # Create PointCloud2 message with intensity for better performance
            header = Header()
            header.frame_id = "world_ned"
            header.stamp = self.get_clock().now().to_msg()
            
            # Convert points to bytes (x, y, z, intensity)
            points_bytes = b''
            for x, y, z in self.accumulated_points:
                intensity = abs(z) / 50.0  # Use height as intensity
                points_bytes += struct.pack('ffff', x, y, z, intensity)
            
            msg = PointCloud2()
            msg.header = header
            msg.height = 1
            msg.width = len(self.accumulated_points)
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            msg.is_bigendian = False
            msg.point_step = 16  # 4 floats * 4 bytes
            msg.row_step = 16 * len(self.accumulated_points)
            msg.data = points_bytes
            msg.is_dense = True
            
            self.combined_lidar_pub.publish(msg)
            msg.row_step = 12 * len(self.accumulated_points)
            msg.data = points_bytes
            msg.is_dense = True
            
            self.combined_lidar_pub.publish(msg)
            self.get_logger().info(f"🗺️  Combined map: {len(self.accumulated_points):,} points", throttle_duration_sec=2.0)
    
    def update_occupancy_grid_with_ray(self, drone_x, drone_y, point_x, point_y, point_z):
        """Update occupancy grid using ray tracing from drone to LiDAR hit"""
        # Skip if drone position is at origin (not updated yet)
        if abs(drone_x) < 0.1 and abs(drone_y) < 0.1:
            return
        
        # Convert drone position to grid
        drone_grid_x = int((drone_x - self.grid_origin_x) / self.grid_resolution)
        drone_grid_y = int((drone_y - self.grid_origin_y) / self.grid_resolution)
        
        # Convert point position to grid
        point_grid_x = int((point_x - self.grid_origin_x) / self.grid_resolution)
        point_grid_y = int((point_y - self.grid_origin_y) / self.grid_resolution)
        
        # Skip if out of bounds
        if not (0 <= drone_grid_x < self.grid_width and 0 <= drone_grid_y < self.grid_height):
            return
        if not (0 <= point_grid_x < self.grid_width and 0 <= point_grid_y < self.grid_height):
            return
        
        # Bresenham's line algorithm for ray tracing
        cells = self.bresenham_line(drone_grid_x, drone_grid_y, point_grid_x, point_grid_y)
        
        # Mark all cells along ray as FREE (except last one)
        for i, (gx, gy) in enumerate(cells[:-1]):
            if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                # Update log-odds for free space (persistent)
                self.log_odds_grid[gy, gx] += self.log_odds_free
                # Clamp
                self.log_odds_grid[gy, gx] = max(self.log_odds_min, self.log_odds_grid[gy, gx])
        
        # Mark endpoint as OCCUPIED if it's an obstacle (not ground)
        if point_z > -40:  # Above ground level (adjusted for AirSimNH)
            if 0 <= point_grid_x < self.grid_width and 0 <= point_grid_y < self.grid_height:
                self.log_odds_grid[point_grid_y, point_grid_x] += self.log_odds_occupied
                # Clamp
                self.log_odds_grid[point_grid_y, point_grid_x] = min(self.log_odds_max, 
                                                                       self.log_odds_grid[point_grid_y, point_grid_x])
    
    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm for ray tracing"""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return cells
    
    def publish_occupancy_grid(self):
        """Publish 2D occupancy grid map (top-down view with persistent SLAM)"""
        try:
            self.get_logger().info("🗺️  Publishing occupancy grid...", throttle_duration_sec=10.0)
            
            msg = OccupancyGrid()
            
            # Header
            msg.header.frame_id = "world_ned"
            msg.header.stamp = self.get_clock().now().to_msg()
            
            # Metadata
            msg.info.resolution = float(self.grid_resolution)
            msg.info.width = int(self.grid_width)
            msg.info.height = int(self.grid_height)
            msg.info.origin.position.x = float(self.grid_origin_x)
            msg.info.origin.position.y = float(self.grid_origin_y)
            msg.info.origin.position.z = 0.0
            msg.info.origin.orientation.w = 1.0
            
            # Convert log-odds to probability [0-100]
            # Probability = 1 / (1 + exp(-log_odds))
            prob_grid = 1.0 / (1.0 + np.exp(-self.log_odds_grid))
            
            # Convert to ROS occupancy grid format:
            # -1 = unknown, 0 = free, 100 = occupied
            occupancy_grid = np.full_like(prob_grid, -1, dtype=np.int8)
            
            # Threshold for classification (more sensitive)
            occupancy_grid[prob_grid < 0.4] = 0    # Free space (white)
            occupancy_grid[prob_grid > 0.7] = 100  # Occupied (black)
            # Between 0.4-0.7 stays unknown (gray)
            
            # Flatten grid data (row-major order)
            msg.data = occupancy_grid.flatten().tolist()
            
            self.occupancy_grid_pub.publish(msg)
            
            occupied_cells = np.sum(occupancy_grid == 100)
            free_cells = np.sum(occupancy_grid == 0)
            unknown_cells = np.sum(occupancy_grid == -1)
            self.get_logger().info(
                f"🗺️  Map published: {occupied_cells:,} occupied, {free_cells:,} free, {unknown_cells:,} unknown", 
                throttle_duration_sec=5.0
            )
            
        except Exception as e:
            self.get_logger().error(f"❌ Error publishing occupancy grid: {e}")
            import traceback
            traceback.print_exc()
    
    def voxel_publisher_thread(self):
        """Dedicated thread for publishing voxels - prevents RViz overload"""
        self.get_logger().info("🧵 Voxel publisher thread started")
        published_voxels = set()  # Track what we've published
        
        while rclpy.ok():
            try:
                # Wait 5 seconds between updates (VERY slow to prevent crash)
                time.sleep(5.0)
                
                # Publish voxels if we have any (even before mapping fully active)
                with self.voxel_lock:
                    if len(self.voxel_grid) == 0:
                        self.get_logger().info("⏳ No voxels yet...")
                        continue
                    
                    self.get_logger().info(f"🗺️  Voxel grid has {len(self.voxel_grid)} voxels")
                    
                    # Get top 100 most confident voxels only
                    sorted_voxels = sorted(
                        self.voxel_grid.items(),
                        key=lambda x: x[1],  # Sort by count
                        reverse=True
                    )[:100]  # TOP 100 ONLY
                    
                    marker_array = MarkerArray()
                    current_voxels = set()
                    
                    # Only add/update markers - NEVER DELETE
                    for i, (voxel, count) in enumerate(sorted_voxels):
                        # Skip low confidence (< 5 hits)
                        if count < 5:
                            continue
                        
                        current_voxels.add(voxel)
                        
                        # Create stable marker with fixed ID based on voxel coords
                        marker = Marker()
                        marker.header.frame_id = "world_ned"
                        marker.header.stamp = self.get_clock().now().to_msg()
                        marker.ns = "voxels"
                        # Use hash of voxel coords for stable ID (prevents flicker)
                        marker.id = hash(voxel) % 100000
                        marker.type = Marker.CUBE
                        marker.action = Marker.ADD  # Always ADD, never DELETE
                        
                        # Position
                        marker.pose.position.x = float(voxel[0] * self.voxel_size)
                        marker.pose.position.y = float(voxel[1] * self.voxel_size)
                        marker.pose.position.z = float(voxel[2] * self.voxel_size)
                        marker.pose.orientation.w = 1.0
                        
                        # Size
                        marker.scale.x = float(self.voxel_size * 0.8)
                        marker.scale.y = float(self.voxel_size * 0.8)
                        marker.scale.z = float(self.voxel_size * 0.8)
                        
                        # Color by height
                        z = voxel[2] * self.voxel_size
                        if z > -10:
                            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)  # Red=high
                        elif z > -20:
                            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9)  # Green=mid
                        else:
                            marker.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.9)  # Blue=low
                        
                        # No lifetime - persist forever (prevents crash on zoom)
                        marker.lifetime.sec = 0
                        marker.lifetime.nanosec = 0
                        
                        marker_array.markers.append(marker)
                    
                    published_voxels.update(current_voxels)
                    
                    # Publish in thread-safe way
                    if len(marker_array.markers) > 0:
                        self.voxel_pub.publish(marker_array)
                        self.get_logger().info(f"📦 Published {len(marker_array.markers)} persistent voxels")
                    
            except Exception as e:
                self.get_logger().error(f"Voxel thread error: {e}")
                time.sleep(1.0)
    
    def publish_point_cloud_simple(self):
        """Publish accumulated point cloud - simple like step-by-step demo"""
        if not self.mapping_active:
            return
        
        with self.point_cloud_lock:
            if len(self.global_point_cloud) == 0:
                return
            
            # Downsample if too many points (every 10th point for performance)
            if len(self.global_point_cloud) > 50000:
                points_to_show = self.global_point_cloud[::10]
            else:
                points_to_show = self.global_point_cloud
            
            try:
                msg = self.create_point_cloud_msg(points_to_show)
                self.map_pub.publish(msg)
                self.get_logger().info(f"🗺️  Published {len(points_to_show)} points", throttle_duration_sec=3.0)
            except Exception as e:
                self.get_logger().error(f"Error publishing point cloud: {e}")
    
    def publish_test_marker(self):
        """Publish a simple test marker at origin to verify RViz is working"""
        marker = Marker()
        marker.header.frame_id = "world_ned"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "test"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # At origin
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = -5.0  # 5m above ground
        marker.pose.orientation.w = 1.0
        
        # Large bright marker
        marker.scale.x = 5.0
        marker.scale.y = 5.0
        marker.scale.z = 5.0
        
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.lifetime.sec = 0  # Persist forever
        
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.drone_markers_pub.publish(marker_array)
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return (x, y, z, w)
    
    def publish_drone_poses(self):
        """Publish odometry for each drone for ROS2 localization"""
        # Skip pose publishing - causes IOLoop conflicts with AirSim's internal loop
        # The ROS2 AirSim bridge already publishes TF and odometry
        return
        
        for drone_name in self.drone_names:
            try:
                state = self.client.getMultirotorState(vehicle_name=drone_name)
                pos = state.kinematics_estimated.position
                orientation = state.kinematics_estimated.orientation
                linear_vel = state.kinematics_estimated.linear_velocity
                angular_vel = state.kinematics_estimated.angular_velocity
                
                # Create Odometry message
                odom = Odometry()
                odom.header.stamp = self.get_clock().now().to_msg()
                odom.header.frame_id = "world_ned"
                odom.child_frame_id = f"{drone_name}_base_link"
                
                # Position
                odom.pose.pose.position.x = pos.x_val
                odom.pose.pose.position.y = pos.y_val
                odom.pose.pose.position.z = pos.z_val
                
                # Orientation
                odom.pose.pose.orientation.x = orientation.x_val
                odom.pose.pose.orientation.y = orientation.y_val
                odom.pose.pose.orientation.z = orientation.z_val
                odom.pose.pose.orientation.w = orientation.w_val
                
                # Velocity
                odom.twist.twist.linear.x = linear_vel.x_val
                odom.twist.twist.linear.y = linear_vel.y_val
                odom.twist.twist.linear.z = linear_vel.z_val
                odom.twist.twist.angular.x = angular_vel.x_val
                odom.twist.twist.angular.y = angular_vel.y_val
                odom.twist.twist.angular.z = angular_vel.z_val
                
                self.odom_pubs[drone_name].publish(odom)
                
                # Update stats
                self.drone_stats[drone_name]['position'] = [pos.x_val, pos.y_val, pos.z_val]
                
            except Exception as e:
                self.get_logger().error(f"Error publishing pose for {drone_name}: {e}")
    
    def create_point_cloud_msg(self, points):
        """Convert point cloud to ROS2 PointCloud2 message"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "world_ned"
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Pack points into binary data - ensure all values are float
        cloud_data = []
        for point in points:
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])
            cloud_data.append(struct.pack('ffff', x, y, z, 1.0))
        
        msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=16,
            row_step=16 * len(points),
            data=b''.join(cloud_data)
        )
        
        return msg
    
    def publish_map(self):
        """Publish voxel grid only (prevent RViz crash from too much data)"""
        if not self.mapping_active:
            return
        
        try:
            # Only publish voxel grid - much lighter than raw point cloud
            self.publish_voxel_grid()
            
            # Publish drone position markers
            self.publish_drone_markers()
            
            # Skip coverage zones to reduce load
            # self.publish_coverage_zones()
            
        except Exception as e:
            self.get_logger().error(f"Error publishing map: {e}")
    
    def publish_drone_markers(self):
        """Publish markers showing drone positions and status"""
        marker_array = MarkerArray()
        
        colors = [
            (1.0, 0.0, 0.0),  # Red - Drone0
            (0.0, 1.0, 0.0),  # Green - Drone1
            (0.0, 0.0, 1.0),  # Blue - Drone2
            (1.0, 1.0, 0.0),  # Yellow - Drone3
            (1.0, 0.0, 1.0),  # Magenta - Drone4
        ]
        
        for i, drone_name in enumerate(self.drone_names):
            # ALWAYS update position from AirSim for accurate ray tracing
            try:
                state = self.client.getMultirotorState(vehicle_name=drone_name)
                pos_obj = state.kinematics_estimated.position
                # Explicitly convert to float and update immediately
                self.drone_stats[drone_name]['position'] = [
                    float(pos_obj.x_val), 
                    float(pos_obj.y_val), 
                    float(pos_obj.z_val)
                ]
            except Exception as e:
                self.get_logger().warning(f"Could not get position for {drone_name}: {e}")
            
            pos = self.drone_stats[drone_name]['position']
            status = self.drone_stats[drone_name]['status']
            points = self.drone_stats[drone_name]['points_collected']
            
            # Drone marker (sphere)
            marker = Marker()
            marker.header.frame_id = "world_ned"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "drones"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Ensure float types for ROS2
            marker.pose.position.x = float(pos[0])
            marker.pose.position.y = float(pos[1])
            marker.pose.position.z = float(pos[2])
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 2.0
            marker.scale.y = 2.0
            marker.scale.z = 2.0
            
            marker.color.r = colors[i][0]
            marker.color.g = colors[i][1]
            marker.color.b = colors[i][2]
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
            
            # Text label
            text_marker = Marker()
            text_marker.header.frame_id = "world_ned"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "drone_labels"
            text_marker.id = i + 100
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Ensure float types
            text_marker.pose.position.x = float(pos[0])
            text_marker.pose.position.y = float(pos[1])
            text_marker.pose.position.z = float(pos[2]) - 3.0  # Above drone
            text_marker.pose.orientation.w = 1.0
            
            text_marker.text = f"{drone_name}\n{status}\n{points} pts"
            text_marker.scale.z = 1.5
            
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            marker_array.markers.append(text_marker)
        
        if len(marker_array.markers) > 0:
            self.drone_markers_pub.publish(marker_array)
            self.get_logger().info(f"🚁 Published {len(marker_array.markers)} drone markers", throttle_duration_sec=5.0)
    
    def publish_coverage_zones(self):
        """Publish visualization of coverage zones for each drone"""
        marker_array = MarkerArray()
        
        # Define sector boundaries (5 sectors radiating from center)
        sectors = self.get_sector_assignments()
        
        for i, (drone_name, sector) in enumerate(sectors.items()):
            marker = Marker()
            marker.header.frame_id = "world_ned"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "coverage_zones"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Create sector outline
            for point in sector['boundary']:
                p = Point()
                p.x = float(point[0])
                p.y = float(point[1])
                p.z = -2.0  # Ground level
                marker.points.append(p)
            
            # Close the loop
            p = Point()
            p.x = float(sector['boundary'][0][0])
            p.y = float(sector['boundary'][0][1])
            p.z = -2.0
            marker.points.append(p)
            
            marker.scale.x = 0.5  # Line width
            
            # Semi-transparent color matching drone
            colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), 
                     (1.0, 1.0, 0.0), (1.0, 0.0, 1.0)]
            marker.color.r = colors[i][0]
            marker.color.g = colors[i][1]
            marker.color.b = colors[i][2]
            marker.color.a = 0.5
            
            marker_array.markers.append(marker)
        
        self.coverage_markers_pub.publish(marker_array)
    
    def publish_stats(self):
        """Publish mapping statistics"""
        if not self.mapping_active:
            return
        
        total_points = len(self.global_point_cloud)
        
        stats_lines = [
            "=== COLLABORATIVE 3D MAPPING STATS ===",
            f"Total Points: {total_points:,}",
            ""
        ]
        
        for drone_name in self.drone_names:
            stats = self.drone_stats[drone_name]
            stats_lines.append(
                f"{drone_name}: {stats['status']} | "
                f"{stats['points_collected']:,} pts | "
                f"Pos: ({stats['position'][0]:.1f}, {stats['position'][1]:.1f}, {stats['position'][2]:.1f})"
            )
        
        stats_msg = String()
        stats_msg.data = "\n".join(stats_lines)
        self.stats_pub.publish(stats_msg)
        
        self.get_logger().info(f"📊 Total map points: {total_points:,}")
    
    def get_sector_assignments(self):
        """Divide map into 5 sectors for parallel coverage"""
        # Radial sectors from center (0,0)
        # Each drone gets a 72-degree slice
        radius = 150.0  # 150m radius coverage
        
        sectors = {}
        for i, drone_name in enumerate(self.drone_names):
            angle_start = i * 72.0 * np.pi / 180.0
            angle_end = (i + 1) * 72.0 * np.pi / 180.0
            
            # Create sector boundary points
            boundary = [(0, 0)]  # Center
            for angle in np.linspace(angle_start, angle_end, 10):
                x = radius * np.cos(angle)
                y = radius * np.sin(angle)
                boundary.append((x, y))
            boundary.append((0, 0))  # Back to center
            
            sectors[drone_name] = {
                'angle_start': angle_start,
                'angle_end': angle_end,
                'radius': radius,
                'boundary': boundary
            }
        
        return sectors
    
    def get_waypoints_for_sector(self, sector, altitude):
        """Generate spiral waypoints within a sector"""
        waypoints = []
        
        angle_mid = (sector['angle_start'] + sector['angle_end']) / 2
        angle_range = sector['angle_end'] - sector['angle_start']
        
        # Spiral outward from center
        for r in np.linspace(20, sector['radius'], 8):
            # Sweep across the sector
            for angle_offset in np.linspace(-angle_range/2, angle_range/2, 6):
                angle = angle_mid + angle_offset
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                waypoints.append((x, y, altitude))
        
        return waypoints
    
    def collect_lidar_data(self, drone_name, drone_client=None):
        """Collect LiDAR point cloud data from a drone"""
        try:
            # Use provided client or fall back to main client
            client = drone_client if drone_client else self.client
            
            lidar_data = client.getLidarData(
                lidar_name=self.lidar_name,
                vehicle_name=drone_name
            )
            
            if len(lidar_data.point_cloud) < 3:
                self.get_logger().warning(f"{drone_name}: No LiDAR data (empty point cloud)")
                return []
            
            self.get_logger().info(f"{drone_name}: Received {len(lidar_data.point_cloud)//3} LiDAR points")
            
            # Get sensor pose
            pos = lidar_data.pose.position
            orientation = lidar_data.pose.orientation
            
            # Convert quaternion to rotation matrix
            q0, q1, q2, q3 = orientation.w_val, orientation.x_val, orientation.y_val, orientation.z_val
            
            R = np.array([
                [1 - 2*(q2**2 + q3**2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
                [2*(q1*q2 + q0*q3), 1 - 2*(q1**2 + q3**2), 2*(q2*q3 - q0*q1)],
                [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1**2 + q2**2)]
            ])
            
            # Transform points to world frame and voxelize
            points_local = np.array(lidar_data.point_cloud).reshape(-1, 3)
            world_points = []
            new_voxels = 0
            
            for point_local in points_local:
                # Rotate
                point_rotated = R @ point_local
                # Translate
                point_world = point_rotated + np.array([pos.x_val, pos.y_val, pos.z_val])
                # Convert to list of floats explicitly
                world_points.append([float(point_world[0]), float(point_world[1]), float(point_world[2])])
                
                # Voxelize with thread-safe access
                voxel = self.voxelize_point(point_world[0], point_world[1], point_world[2])
                with self.voxel_lock:
                    if voxel not in self.voxel_grid:
                        self.voxel_grid[voxel] = 0
                        new_voxels += 1
                    self.voxel_grid[voxel] += 1
            
            return world_points
            
        except Exception as e:
            self.get_logger().error(f"Error collecting LiDAR from {drone_name}: {e}")
            return []
    
    def voxelize_point(self, x, y, z):
        """Convert world coordinate to voxel grid coordinate"""
        vx = int(np.floor(x / self.voxel_size))
        vy = int(np.floor(y / self.voxel_size))
        vz = int(np.floor(z / self.voxel_size))
        return (vx, vy, vz)
    
    def publish_voxel_grid(self):
        """Legacy - now handled by dedicated thread"""
        pass
    
    def drone_mapping_task(self, drone_name, sector):
        """Individual drone mapping task"""
        try:
            self.get_logger().info(f"🚁 {drone_name} starting mapping task")
            
            # Create dedicated client for this drone to avoid IOLoop conflicts
            drone_client = airsim.MultirotorClient()
            drone_client.confirmConnection()
            self.drone_clients[drone_name] = drone_client
            
            # Enable API control
            drone_client.enableApiControl(True, drone_name)
            drone_client.armDisarm(True, drone_name)
            
            self.drone_stats[drone_name]['status'] = 'taking_off'
            
            # Takeoff
            drone_client.takeoffAsync(vehicle_name=drone_name).join()
            time.sleep(3)
            
            # Move to altitude
            drone_client.moveToZAsync(self.altitude, self.velocity, vehicle_name=drone_name).join()
            time.sleep(2)
            
            self.drone_stats[drone_name]['status'] = 'mapping'
            
            # Get waypoints for this sector
            waypoints = self.get_waypoints_for_sector(sector, self.altitude)
            
            self.get_logger().info(f"🗺️  {drone_name} visiting {len(waypoints)} waypoints")
            
            # Visit each waypoint and collect data
            for i, waypoint in enumerate(waypoints):
                if not self.mapping_active:
                    break
                
                # Move to waypoint
                drone_client.moveToPositionAsync(
                    waypoint[0], waypoint[1], waypoint[2],
                    self.velocity,
                    vehicle_name=drone_name
                ).join()
                
                # Collect LiDAR data using the drone's client
                points = self.collect_lidar_data(drone_name, drone_client)
                
                if points:
                    with self.point_cloud_lock:
                        self.global_point_cloud.extend(points)
                        self.drone_stats[drone_name]['points_collected'] += len(points)
                    
                    self.get_logger().info(
                        f"📍 {drone_name} waypoint {i+1}/{len(waypoints)}: "
                        f"collected {len(points)} points"
                    )
                
                time.sleep(0.5)  # Brief pause for data collection
            
            self.drone_stats[drone_name]['status'] = 'returning'
            
            # Return to home
            drone_client.moveToPositionAsync(0, 0, self.altitude, self.velocity, vehicle_name=drone_name).join()
            
            self.drone_stats[drone_name]['status'] = 'landing'
            drone_client.landAsync(vehicle_name=drone_name).join()
            
            self.drone_stats[drone_name]['status'] = 'complete'
            self.get_logger().info(f"✅ {drone_name} mapping complete!")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error in {drone_name} mapping task: {e}")
            self.drone_stats[drone_name]['status'] = 'error'
    
    def start_collaborative_mapping(self):
        """Launch all drones for collaborative mapping"""
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("  🗺️  STARTING COLLABORATIVE 3D MAPPING")
        self.get_logger().info("="*70)
        
        self.mapping_active = True
        
        # Get sector assignments
        sectors = self.get_sector_assignments()
        
        # Launch each drone in its own thread
        for drone_name in self.drone_names:
            sector = sectors[drone_name]
            thread = threading.Thread(
                target=self.drone_mapping_task,
                args=(drone_name, sector)
            )
            thread.daemon = True
            thread.start()
            self.threads.append(thread)
            time.sleep(1)  # Stagger launches
        
        self.get_logger().info("🚀 All drones launched!")
    
    def wait_for_completion(self):
        """Wait for all drones to complete mapping"""
        for thread in self.threads:
            thread.join()
        
        self.mapping_active = False
        
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("  ✅ COLLABORATIVE MAPPING COMPLETE")
        self.get_logger().info("="*70)
        self.get_logger().info(f"Total points collected: {len(self.global_point_cloud):,}")
        
        # Final stats
        for drone_name in self.drone_names:
            stats = self.drone_stats[drone_name]
            self.get_logger().info(
                f"  {drone_name}: {stats['points_collected']:,} points | {stats['status']}"
            )
    
    def save_point_cloud(self, filename="/tmp/collaborative_map.ply"):
        """Save the final point cloud to PLY file"""
        try:
            with self.point_cloud_lock:
                points = self.global_point_cloud
            
            with open(filename, 'w') as f:
                f.write("ply\n")
                f.write("format ascii 1.0\n")
                f.write(f"element vertex {len(points)}\n")
                f.write("property float x\n")
                f.write("property float y\n")
                f.write("property float z\n")
                f.write("end_header\n")
                
                for point in points:
                    f.write(f"{point[0]} {point[1]} {point[2]}\n")
            
            self.get_logger().info(f"💾 Point cloud saved to {filename}")
            
        except Exception as e:
            self.get_logger().error(f"Error saving point cloud: {e}")


def main(args=None):
    print("\n" + "="*70)
    print("  🗺️  COLLABORATIVE 3D MAPPING WITH DRONE SWARM")
    print("="*70)
    print("  5 drones will map the environment in parallel sectors")
    print("  Real-time 3D point cloud published to ROS2")
    print("  Visualization in RViz2")
    print("="*70 + "\n")
    
    rclpy.init(args=args)
    
    mapper = Collaborative3DMapper()
    
    try:
        # Start mapping
        mapper.start_collaborative_mapping()
        
        # Spin ROS2 while mapping
        while mapper.mapping_active and rclpy.ok():
            rclpy.spin_once(mapper, timeout_sec=0.1)
        
        # Wait for completion
        mapper.wait_for_completion()
        
        # Save map
        mapper.save_point_cloud()
        
        # Keep publishing for visualization
        print("\n📊 Mapping complete. Press Ctrl+C to exit...")
        rclpy.spin(mapper)
        
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        mapper.mapping_active = False
        mapper.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
