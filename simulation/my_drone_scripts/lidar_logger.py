#!/usr/bin/env python3
"""
Lidar Data Logger
=================
Logs lidar data to files for analysis
- Saves point clouds to .ply files
- Logs statistics to CSV
- Real-time distance measurements
"""

import airsim
import numpy as np
import time
import os
from datetime import datetime

def save_point_cloud_ply(points, filename):
    """Save point cloud in PLY format (viewable in MeshLab/CloudCompare)"""
    with open(filename, 'w') as f:
        # Write header
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        
        # Write points
        for point in points:
            f.write(f"{point[0]} {point[1]} {point[2]}\n")

def log_lidar_data(duration_seconds=60, drone="Drone1"):
    """Log lidar data for specified duration"""
    
    print("📊 Lidar Logger Starting...")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    # Create output directory
    output_dir = os.path.expanduser("~/AirSim/my_drone_scripts/lidar_logs")
    os.makedirs(output_dir, exist_ok=True)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_file = os.path.join(output_dir, f"lidar_stats_{timestamp}.csv")
    
    print(f"📁 Saving to: {output_dir}")
    print(f"📝 CSV log: {csv_file}")
    print(f"⏱️  Duration: {duration_seconds} seconds")
    print("\nLogging started...\n")
    
    # CSV header
    with open(csv_file, 'w') as f:
        f.write("timestamp,num_points,min_dist,max_dist,avg_dist,std_dist\n")
    
    start_time = time.time()
    frame_count = 0
    
    try:
        while (time.time() - start_time) < duration_seconds:
            # Get lidar data
            lidar = client.getLidarData(lidar_name="Lidar1", vehicle_name=drone)
            
            if len(lidar.point_cloud) >= 3:
                points = np.array(lidar.point_cloud, dtype=np.float32)
                points = points.reshape(-1, 3)
                
                # Calculate statistics
                distances = np.linalg.norm(points, axis=1)
                min_dist = distances.min()
                max_dist = distances.max()
                avg_dist = distances.mean()
                std_dist = distances.std()
                
                # Log to CSV
                timestamp_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                with open(csv_file, 'a') as f:
                    f.write(f"{timestamp_str},{len(points)},{min_dist:.3f},{max_dist:.3f},{avg_dist:.3f},{std_dist:.3f}\n")
                
                # Save point cloud every 10 frames
                if frame_count % 10 == 0:
                    ply_file = os.path.join(output_dir, f"pointcloud_frame_{frame_count:04d}.ply")
                    save_point_cloud_ply(points, ply_file)
                    print(f"💾 Saved: {ply_file} ({len(points)} points)")
                
                # Display stats
                print(f"\rFrame {frame_count}: {len(points)} pts | "
                      f"Dist: min={min_dist:.2f}m avg={avg_dist:.2f}m max={max_dist:.2f}m", end="")
                
                frame_count += 1
            
            time.sleep(0.1)  # 10Hz
            
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    
    print(f"\n\n✅ Logging complete!")
    print(f"   Frames captured: {frame_count}")
    print(f"   CSV file: {csv_file}")
    print(f"   Point clouds: {output_dir}")
    print(f"\n💡 View .ply files with MeshLab or CloudCompare")

if __name__ == "__main__":
    import sys
    
    duration = 60
    drone = "Drone1"
    
    if len(sys.argv) > 1:
        duration = int(sys.argv[1])
    if len(sys.argv) > 2:
        drone = sys.argv[2]
    
    print("=" * 60)
    print("  📊 LIDAR DATA LOGGER")
    print("=" * 60)
    print(f"\nDrone: {drone}")
    print(f"Duration: {duration} seconds")
    print("\nPress Ctrl+C to stop early\n")
    
    log_lidar_data(duration, drone)
