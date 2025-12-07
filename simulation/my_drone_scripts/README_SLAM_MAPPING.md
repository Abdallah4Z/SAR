# 🗺️ Collaborative SLAM Mapping with RTAB-Map

Complete 3D SLAM mapping system using 5 drones with RTAB-Map for proper localization and mapping.

## 🎯 What You Get

- **Proper 3D SLAM** - Real-time appearance-based mapping with loop closure
- **2D Occupancy Grid** - Top-down map like your reference image
- **3D Point Cloud** - Dense reconstructed environment
- **Multi-drone collaboration** - All 5 drones contribute to one map
- **Persistent mapping** - Map stays even when drones leave areas

## 📋 Prerequisites

### 1. Install RTAB-Map

```bash
sudo apt update
sudo apt install -y ros-humble-rtabmap-ros
```

### 2. Verify Installation

```bash
rtabmap --version
```

Should show: `RTAB-Map version X.X.X`

## 🚀 Quick Start

### 1. Make script executable

```bash
chmod +x ~/AirSim/my_drone_scripts/launch_slam_mapping.sh
```

### 2. Start AirSim first

```bash
cd ~/Downloads/AirSimNH/LinuxNoEditor
./AirSimNH.sh -windowed -ResX=1280 -ResY=720
```

### 3. Launch SLAM system

```bash
cd ~/AirSim/my_drone_scripts
./launch_slam_mapping.sh
```

## 🎨 RViz2 Visualization Setup

Once RViz opens, configure these displays:

### Essential Displays

1. **Set Fixed Frame**: `world_ned` or `map`

2. **3D Reconstructed Map**
   - Add → `/rtabmap/cloud_map` → PointCloud2
   - Color Transformer: RGB or Intensity
   - Size (m): 0.05
   - **This is your main 3D map!**

3. **2D Occupancy Grid** (like your reference image)
   - Add → `/rtabmap/grid_map` → Map
   - Color Scheme: map or costmap
   - Alpha: 0.7
   - **This is the top-down 2D map!**

4. **3D Occupancy (Optional)**
   - Add → `/rtabmap/octomap_occupied_space` → PointCloud2
   - Shows 3D voxel occupancy

### Optional Displays

5. **Drone Positions**
   - Add → `/collaborative_map/drone_positions` → MarkerArray
   - Shows where all 5 drones are

6. **Coverage Zones**
   - Add → `/collaborative_map/coverage_zones` → MarkerArray
   - Shows exploration sectors

## 🔑 Key ROS2 Topics

### RTAB-Map Topics (SLAM output)
- `/rtabmap/cloud_map` - Full 3D reconstructed point cloud
- `/rtabmap/grid_map` - 2D occupancy grid map
- `/rtabmap/octomap_grid` - 3D OctoMap
- `/rtabmap/mapData` - SLAM graph data
- `/rtabmap/info` - Loop closures and statistics

### Drone Topics (Input data)
- `/airsim_node/Drone0-4/lidar/Lidar1` - LiDAR scans
- `/airsim_node/Drone0-4/front_center/Scene` - Camera images
- `/collaborative_map/combined_lidar` - Merged point cloud

## 📊 What You'll See

1. **First 10-20 seconds**: Map initializes, starts collecting data
2. **30-60 seconds**: 3D map begins to form, showing buildings/trees
3. **1-2 minutes**: 2D grid map shows clear free space (white) vs obstacles (black)
4. **5+ minutes**: Complete detailed map with loop closures

## 🎛️ Advanced Configuration

### Change Map Resolution

Edit `launch_slam_mapping.sh` and modify:
```bash
rtabmap_args:="--Grid/CellSize 0.5"  # Cell size in meters
```

### Enable Loop Closure

Loop closure is enabled by default. RTAB-Map will recognize previously visited areas and correct drift.

### Save the Map

RTAB-Map automatically saves to: `~/.ros/rtabmap.db`

To export as point cloud:
```bash
rtabmap-export ~/.ros/rtabmap.db
```

## 🐛 Troubleshooting

### "RTAB-Map not found"
```bash
sudo apt install -y ros-humble-rtabmap-ros
source /opt/ros/humble/setup.bash
```

### "No map appearing"
- Wait 30-60 seconds for initialization
- Check that drones are flying: `rostopic echo /airsim_node/Drone0/lidar/Lidar1`
- Check RTAB-Map log: `tail -f /tmp/rtabmap.log`

### "Map quality is poor"
- Increase flight time (longer = better map)
- Add more features to environment
- Reduce drone speed for better scanning

### "Topics not showing in RViz"
```bash
ros2 topic list | grep rtabmap
```
Should show multiple /rtabmap/* topics

## 📁 Log Files

- `/tmp/rtabmap.log` - RTAB-Map SLAM output
- `/tmp/rviz2_slam.log` - RViz visualization log
- `/tmp/airsim_ros2_bridge.log` - ROS2 bridge log

## 🎓 How It Works

1. **Data Collection**: 5 drones fly in assigned sectors
2. **ROS2 Bridge**: Publishes LiDAR + camera data to ROS2
3. **RTAB-Map**: 
   - Processes LiDAR scans
   - Extracts visual features from cameras
   - Performs odometry estimation
   - Builds 3D map with loop closure
   - Projects to 2D occupancy grid
4. **Visualization**: RViz shows all layers in real-time

## 🎯 Expected Results

You should see maps similar to your reference image:
- **3D point cloud**: Colored by height showing full environment
- **2D occupancy grid**: Black=obstacles, white=free, gray=unknown
- **Persistent**: Map doesn't disappear when drones move away

## 🔄 Restarting with Fresh Map

To clear the map and start over:
```bash
rm ~/.ros/rtabmap.db
./launch_slam_mapping.sh
```

The `--delete_db_on_start` flag in the script already does this automatically.

## 📚 Learn More

- [RTAB-Map Documentation](http://wiki.ros.org/rtabmap_ros)
- [RTAB-Map ROS2](https://github.com/introlab/rtabmap_ros)
- [RTAB-Map Paper](http://introlab.3it.usherbrooke.ca/mediawiki-introlab/images/7/7a/Labbe14-IROS.pdf)
