#!/usr/bin/env python3
"""
3D Voxel Grid Mapping with Single Drone
Builds a clean 3D occupancy grid map layer by layer
Real-time visualization in RViz with proper map structure
"""

import airsim
import numpy as np
import time
import sys
from collections import defaultdict

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA
import struct


class VoxelGridMapper(Node):
    def __init__(self):
        super().__init__('voxel_grid_mapper')
        
        # Connect to AirSim
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.get_logger().info("✅ Connected to AirSim")
        
        # Mapping parameters
        self.drone_name = "Drone0"
        self.lidar_name = "Lidar1"
        
        # Voxel grid parameters
        self.voxel_size = 1.0  # 1 meter voxels
        self.voxel_grid = {}  # Dict of (x,y,z) -> occupancy count
        
        # Flight parameters
        self.scan_altitude = -20.0  # 20m above ground
        self.scan_radius = 50  # Scan 50m radius
        self.grid_step = 10  # Visit waypoints every 10m
        
        # ROS2 Publishers
        self.voxel_pub = self.create_publisher(
            MarkerArray,
            '/voxel_map/occupied_cells',
            10
        )
        self.scan_progress_pub = self.create_publisher(
            Marker,
            '/voxel_map/scan_progress',
            10
        )
        
        # Timer for visualization updates
        self.create_timer(1.0, self.publish_voxel_map)
        
        self.mapping_active = False
        
    def voxelize_point(self, x, y, z):
        """Convert world coordinate to voxel grid coordinate"""
        vx = int(np.floor(x / self.voxel_size))
        vy = int(np.floor(y / self.voxel_size))
        vz = int(np.floor(z / self.voxel_size))
        return (vx, vy, vz)
    
    def collect_and_voxelize_lidar(self):
        """Collect LiDAR data and add to voxel grid"""
        try:
            lidar_data = self.client.getLidarData(
                lidar_name=self.lidar_name,
                vehicle_name=self.drone_name
            )
            
            if len(lidar_data.point_cloud) < 3:
                return 0
            
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
            new_voxels = 0
            
            for point_local in points_local:
                # Transform to world coordinates
                point_rotated = R @ point_local
                point_world = point_rotated + np.array([pos.x_val, pos.y_val, pos.z_val])
                
                # Convert to voxel coordinates
                voxel = self.voxelize_point(point_world[0], point_world[1], point_world[2])
                
                # Add to grid (count occupancy)
                if voxel not in self.voxel_grid:
                    self.voxel_grid[voxel] = 0
                    new_voxels += 1
                self.voxel_grid[voxel] += 1
            
            return new_voxels
            
        except Exception as e:
            self.get_logger().error(f"Error collecting LiDAR: {e}")
            return 0
    
    def publish_voxel_map(self):
        """Publish voxel grid as colored cubes"""
        if not self.mapping_active or len(self.voxel_grid) == 0:
            return
        
        marker_array = MarkerArray()
        
        # Create a cube marker for each occupied voxel
        for i, (voxel, count) in enumerate(self.voxel_grid.items()):
            # Only show voxels with multiple hits (filter noise)
            if count < 2:
                continue
            
            marker = Marker()
            marker.header.frame_id = "world_ned"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "occupied_voxels"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Position at voxel center
            marker.pose.position.x = float(voxel[0] * self.voxel_size + self.voxel_size/2)
            marker.pose.position.y = float(voxel[1] * self.voxel_size + self.voxel_size/2)
            marker.pose.position.z = float(voxel[2] * self.voxel_size + self.voxel_size/2)
            marker.pose.orientation.w = 1.0
            
            # Voxel size
            marker.scale.x = float(self.voxel_size * 0.95)  # Slight gap between voxels
            marker.scale.y = float(self.voxel_size * 0.95)
            marker.scale.z = float(self.voxel_size * 0.95)
            
            # Color by height (blue=low, green=mid, red=high)
            height = voxel[2] * self.voxel_size
            if height > -10:  # High objects
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            elif height > -20:  # Mid-level
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
            else:  # Ground level
                marker.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)
            
            marker_array.markers.append(marker)
            
            # Limit to 10000 markers for performance
            if len(marker_array.markers) >= 10000:
                break
        
        self.voxel_pub.publish(marker_array)
        self.get_logger().info(f"📦 Published {len(marker_array.markers)} voxels")
    
    def scan_layer(self, altitude):
        """Scan a horizontal layer at given altitude"""
        self.get_logger().info(f"📍 Scanning layer at altitude {altitude}m")
        
        # Move to altitude
        self.client.moveToZAsync(altitude, 5.0, vehicle_name=self.drone_name).join()
        time.sleep(2)
        
        # Create grid of waypoints
        waypoints = []
        for x in range(-self.scan_radius, self.scan_radius + 1, self.grid_step):
            for y in range(-self.scan_radius, self.scan_radius + 1, self.grid_step):
                # Skip if outside circular boundary
                if x*x + y*y <= self.scan_radius * self.scan_radius:
                    waypoints.append((x, y, altitude))
        
        self.get_logger().info(f"   Visiting {len(waypoints)} waypoints...")
        
        total_voxels = 0
        # Visit each waypoint
        for i, (x, y, z) in enumerate(waypoints):
            if not self.mapping_active:
                break
            
            # Move to waypoint
            self.client.moveToPositionAsync(x, y, z, 5.0, vehicle_name=self.drone_name).join()
            time.sleep(0.3)
            
            # Collect LiDAR and voxelize
            new_voxels = self.collect_and_voxelize_lidar()
            total_voxels += new_voxels
            
            if (i + 1) % 10 == 0:
                self.get_logger().info(
                    f"   Progress: {i+1}/{len(waypoints)} waypoints | "
                    f"Total voxels: {len(self.voxel_grid):,} | "
                    f"Layer voxels: {total_voxels}"
                )
        
        self.get_logger().info(f"✅ Layer complete: {total_voxels} new voxels discovered")
    
    def start_mapping(self):
        """Start the voxel mapping process"""
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("  📦 STARTING 3D VOXEL GRID MAPPING")
        self.get_logger().info("="*70)
        
        self.mapping_active = True
        
        # Enable API control
        self.client.enableApiControl(True, self.drone_name)
        self.client.armDisarm(True, self.drone_name)
        
        # Takeoff
        self.get_logger().info("🛫 Taking off...")
        self.client.takeoffAsync(vehicle_name=self.drone_name).join()
        time.sleep(3)
        
        # Scan multiple layers (top to bottom)
        layers = [-15, -20, -25, -30]  # Heights to scan
        
        for layer_alt in layers:
            self.scan_layer(layer_alt)
            time.sleep(2)
        
        # Return home
        self.get_logger().info("🏠 Returning to home...")
        self.client.moveToPositionAsync(0, 0, self.scan_altitude, 5.0, vehicle_name=self.drone_name).join()
        
        # Land
        self.get_logger().info("🛬 Landing...")
        self.client.landAsync(vehicle_name=self.drone_name).join()
        
        self.mapping_active = False
        
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("  ✅ MAPPING COMPLETE")
        self.get_logger().info("="*70)
        self.get_logger().info(f"Total unique voxels mapped: {len(self.voxel_grid):,}")
        self.get_logger().info(f"Voxel size: {self.voxel_size}m")
        self.get_logger().info(f"Map volume: ~{len(self.voxel_grid) * self.voxel_size**3:.1f} m³")


def main(args=None):
    print("\n" + "="*70)
    print("  📦 3D VOXEL GRID MAPPING")
    print("="*70)
    print("  Building structured 3D map with LiDAR")
    print("  Visualization: Colored cubes in RViz")
    print("="*70 + "\n")
    
    rclpy.init(args=args)
    
    mapper = VoxelGridMapper()
    
    try:
        # Start mapping in background
        import threading
        mapping_thread = threading.Thread(target=mapper.start_mapping)
        mapping_thread.start()
        
        # Spin ROS2 for visualization
        rclpy.spin(mapper)
        
        mapping_thread.join()
        
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    finally:
        mapper.mapping_active = False
        mapper.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
