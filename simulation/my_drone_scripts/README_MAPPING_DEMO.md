# 🗺️ Collaborative 3D Mapping + YOLO Person/Animal Detection Demo

## Overview

This demo showcases a multi-drone system that performs collaborative 3D mapping using LiDAR while simultaneously detecting people and animals using YOLO computer vision.

**Key Features:**
- ✅ 5 drones mapping environment in parallel sectors
- ✅ Real-time 3D point cloud generation and merging
- ✅ YOLO-based person and animal detection
- ✅ ROS2 localization with odometry publishing
- ✅ Live RViz2 visualization
- ✅ Automatic entity spawning in environment

## Files Created

### 1. `spawn_entities.py`
Spawns people and animals throughout the AirSimNH environment for detection.
- **Features:**
  - Automatic asset detection with fallback to markers
  - Grid + clustered distribution (parks, farms)
  - Saves entity manifest for reference
  - Supports 8+ entity types (person, dog, cat, horse, cow, sheep, bird, elephant)

### 2. `yolo_person_animal_detector.py`
YOLO detection running on Drone0 (scout drone).
- **Features:**
  - YOLOv8 nano model (real-time performance)
  - Detects 11 COCO classes (person + animals)
  - 3D position estimation from 2D detections
  - ROS2 publishers for images and 3D markers
  - Real-time statistics overlay

### 3. `collaborative_3d_mapping.py`
Main mapping script coordinating all 5 drones.
- **Features:**
  - Sector-based parallel coverage (5 x 72° sectors)
  - LiDAR data collection and merging
  - Real-time point cloud publishing
  - Per-drone statistics tracking
  - Odometry publishing for each drone
  - PLY file export

### 4. `launch_mapping_demo.sh`
Complete launch script for the entire demo.
- **Features:**
  - Automatic prerequisite checking
  - Sequential component launching
  - RViz2 configuration guidance
  - Automatic cleanup on exit

### 5. Updated `settings.json`
All 5 drones now equipped with:
- Front-facing cameras (1280x720, 90° FOV)
- LiDAR sensors (16 channels, 100k points/sec, 200m range)

## Prerequisites

### Required
- **AirSim**: [AirSimNH environment](https://github.com/microsoft/AirSim/releases)
- **ROS2 Humble**: `sudo apt install ros-humble-desktop`
- **AirSim ROS2 Bridge**: Built in `~/AirSim/ros2/`
- **Python packages**:
  ```bash
  pip install airsim opencv-python numpy ultralytics
  ```

### Optional
- **YOLO**: `pip install ultralytics` (for person/animal detection)
- **CloudCompare**: For viewing saved .ply point clouds

## Setup

1. **Copy settings.json**:
   ```bash
   cp ~/Documents/AirSim/settings.json ~/Documents/AirSim/settings.json.backup
   # (The demo will use the updated settings.json already in place)
   ```

2. **Make scripts executable**:
   ```bash
   cd ~/AirSim/my_drone_scripts
   chmod +x launch_mapping_demo.sh
   chmod +x spawn_entities.py
   chmod +x yolo_person_animal_detector.py
   chmod +x collaborative_3d_mapping.py
   ```

3. **Build ROS2 workspace** (if not already done):
   ```bash
   cd ~/AirSim/ros2
   colcon build
   source install/setup.bash
   ```

## Running the Demo

### Quick Start
```bash
cd ~/AirSim/my_drone_scripts
./launch_mapping_demo.sh
```

### Step-by-Step
1. **Start AirSim**:
   ```bash
   cd ~/Downloads/AirSimNH/LinuxNoEditor
   ./AirSimNH.sh -windowed -ResX=1280 -ResY=720
   ```

2. **Run the demo script**:
   ```bash
   cd ~/AirSim/my_drone_scripts
   ./launch_mapping_demo.sh
   ```

3. **Configure RViz2** (when prompted):
   - Fixed Frame: `world_ned`
   - Add: `/collaborative_map/point_cloud` (PointCloud2)
   - Add: `/collaborative_map/drone_positions` (MarkerArray)
   - Add: `/collaborative_map/coverage_zones` (MarkerArray)
   - Add: `/Drone0/detections_3d` (MarkerArray)
   - Add: `/Drone0/detection_image` (Image)

4. **Watch the magic happen!** 🎉

## What to Expect

### Timeline
- **0:00-0:30**: System checks, component launching
- **0:30-1:00**: Entity spawning, RViz2 setup
- **1:00-1:30**: Drones take off and reach altitude
- **1:30-8:00**: Drones map their sectors (spiral patterns)
- **8:00-9:00**: Drones return home and land
- **9:00+**: View final 3D map in RViz2

### Visualization in RViz2

**Point Cloud**:
- Real-time 3D reconstruction of the environment
- Colors represent sensor intensity or axis position
- Adjust size slider for better visibility

**Drone Markers**:
- Colored spheres showing drone positions
- Red: Drone0 (scout with YOLO)
- Green: Drone1
- Blue: Drone2
- Yellow: Drone3
- Magenta: Drone4

**Coverage Zones**:
- Semi-transparent sector boundaries
- Shows area responsibility for each drone

**Detection Markers**:
- Colored spheres at detected person/animal locations
- Text labels showing class and confidence

### Detection Classes
YOLO will detect:
- 👤 person
- 🐕 dog
- 🐈 cat
- 🐴 horse
- 🐄 cow
- 🐑 sheep
- 🐦 bird
- 🐘 elephant
- 🐻 bear
- 🦓 zebra
- 🦒 giraffe

## Output Files

After completion:
- **`/tmp/collaborative_map.ply`**: Full 3D point cloud (can be opened in CloudCompare, MeshLab)
- **`/tmp/airsim_entity_manifest.txt`**: List of spawned entities with positions

## Troubleshooting

### "ROS2 not found"
```bash
source /opt/ros/humble/setup.bash
# Or if you have ROS2 workspace:
source ~/AirSim/ros2/install/setup.bash
```

### "airsim module not found"
```bash
pip install airsim
```

### "YOLO not available"
```bash
pip install ultralytics
```

### Drones not moving
- Check AirSim is running
- Verify settings.json is in `~/Documents/AirSim/`
- Restart AirSim and try again

### Low framerate in RViz2
- Reduce PointCloud2 point size
- Close Image viewer if not needed
- Reduce number of displayed MarkerArrays

### No detections appearing
- Make sure entities spawned successfully
- Check `/Drone0/detection_image` topic for camera feed
- Verify YOLO model downloaded (first run may take time)

## Customization

### Change mapping area
Edit `collaborative_3d_mapping.py`:
```python
radius = 150.0  # Change coverage radius (meters)
```

### Change drone altitude
```python
self.altitude = -30.0  # 30m above ground (negative in NED)
```

### Change drone speed
```python
self.velocity = 5.0  # m/s
```

### Change detection confidence threshold
Edit `yolo_person_animal_detector.py`:
```python
results = self.model(img_rgb, verbose=False, conf=0.4)  # Change 0.4 to desired threshold
```

### Add more entities
Edit `spawn_entities.py`:
```python
grid_size = 60  # Reduce for more entities
```

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        AirSim (Unreal Engine)                │
│  ┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐              │
│  │Drone0│ │Drone1│ │Drone2│ │Drone3│ │Drone4│  + Entities  │
│  └───┬──┘ └───┬──┘ └───┬──┘ └───┬──┘ └───┬──┘              │
└──────┼────────┼────────┼────────┼────────┼─────────────────┘
       │        │        │        │        │
       │        └────────┴────────┴────────┼─── LiDAR Data
       │                                   │
       └─── Camera Images ─────────────────┘
                   │                       │
                   ▼                       ▼
       ┌───────────────────┐   ┌─────────────────────┐
       │  YOLO Detector    │   │  3D Mapper          │
       │  (Drone0)         │   │  (All Drones)       │
       └─────────┬─────────┘   └──────────┬──────────┘
                 │                        │
                 └────────────┬───────────┘
                              ▼
                    ┌──────────────────┐
                    │   ROS2 Bridge    │
                    └─────────┬────────┘
                              │
                              ▼
                    ┌──────────────────┐
                    │      RViz2       │
                    │  (Visualization) │
                    └──────────────────┘
```

## Performance Tips

- **Real-time stats**: Watch terminal for per-drone statistics
- **Point cloud quality**: Higher altitude = faster coverage, but less detail
- **Detection accuracy**: Lower flight = better detection, but slower mapping
- **ROS2 topics**: Use `ros2 topic list` to see all available data streams

## Credits

Created for AirSim collaborative drone demonstration.
- Uses Microsoft AirSim simulator
- YOLOv8 from Ultralytics
- ROS2 Humble
- Python 3.10+

## 🍪 Cookie Time!

If everything works on the first try, you've earned your cookie! 🎉

Enjoy watching 5 drones autonomously map an environment while detecting people and animals in real-time!
