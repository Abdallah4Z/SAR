#!/bin/bash
# Complete 3D Mapping + Person/Animal Detection Demo Launch Script
# ================================================================
# Launches collaborative 3D mapping with 5 drones + YOLO detection
# Real-time visualization in RViz2 with localization

echo "═══════════════════════════════════════════════════════════════════════════"
echo "  🗺️  COLLABORATIVE 3D MAPPING + YOLO DETECTION DEMO"
echo "═══════════════════════════════════════════════════════════════════════════"
echo ""

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check prerequisites
echo "📋 Checking prerequisites..."
echo ""

# Check ROS2
if command_exists ros2; then
    echo -e "${GREEN}✅ ROS2 found${NC}"
else
    echo -e "${RED}❌ ROS2 not found. Please install ROS2 Humble.${NC}"
    exit 1
fi

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}⚠️  ROS2 not sourced. Sourcing now...${NC}"
    if [ -f ~/AirSim/ros2/install/setup.bash ]; then
        source ~/AirSim/ros2/install/setup.bash
    elif [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    else
        echo -e "${RED}❌ Could not find ROS2 setup.bash${NC}"
        exit 1
    fi
fi

# Check Python packages
echo ""
echo "📦 Checking Python packages..."
python3 -c "import airsim" 2>/dev/null && echo -e "${GREEN}✅ airsim${NC}" || echo -e "${RED}❌ airsim (pip install airsim)${NC}"
python3 -c "import cv2" 2>/dev/null && echo -e "${GREEN}✅ opencv-python${NC}" || echo -e "${RED}❌ opencv-python (pip install opencv-python)${NC}"
python3 -c "import numpy" 2>/dev/null && echo -e "${GREEN}✅ numpy${NC}" || echo -e "${RED}❌ numpy${NC}"
python3 -c "import rclpy" 2>/dev/null && echo -e "${GREEN}✅ rclpy${NC}" || echo -e "${RED}❌ rclpy${NC}"

# Check YOLO
if python3 -c "import ultralytics" 2>/dev/null; then
    echo -e "${GREEN}✅ ultralytics (YOLO)${NC}"
    YOLO_AVAILABLE=true
else
    echo -e "${YELLOW}⚠️  ultralytics not installed. YOLO detection will be limited.${NC}"
    echo -e "${YELLOW}   Install with: pip install ultralytics${NC}"
    YOLO_AVAILABLE=false
fi

echo ""
echo "═══════════════════════════════════════════════════════════════════════════"
echo "  📝 DEMO OVERVIEW"
echo "═══════════════════════════════════════════════════════════════════════════"
echo ""
echo "This demo showcases:"
echo "  🗺️  Collaborative 3D Mapping - 5 drones map environment in parallel"
echo "  🎯 Person/Animal Detection - YOLO detects people and animals"
echo "  📍 ROS2 Localization - Real-time odometry and TF transforms"
echo "  📊 RViz2 Visualization - Live 3D point cloud and detections"
echo ""
echo "Drone Configuration:"
echo "  • Drone0: 🔍 Scout with YOLO detection (Red sector)"
echo "  • Drone1: 🗺️  Mapper (Green sector)"
echo "  • Drone2: 🗺️  Mapper (Blue sector)"
echo "  • Drone3: 🗺️  Mapper (Yellow sector)"
echo "  • Drone4: 🗺️  Mapper (Magenta sector)"
echo ""
echo "Each drone covers a 72° sector radiating from center"
echo "LiDAR data merged into unified 3D point cloud in real-time"
echo ""
echo "═══════════════════════════════════════════════════════════════════════════"
echo ""

# Ask user to start AirSim
echo -e "${YELLOW}⚠️  IMPORTANT: Make sure AirSim is running!${NC}"
echo ""
echo "If not started, run in another terminal:"
echo "  cd ~/Downloads/AirSimNH/LinuxNoEditor"
echo "  ./AirSimNH.sh -windowed -ResX=1280 -ResY=720"
echo ""
echo -e "${CYAN}Make sure you copied the updated settings.json to ~/Documents/AirSim/${NC}"
echo ""
read -p "Press ENTER when AirSim is ready..."

echo ""
echo "═══════════════════════════════════════════════════════════════════════════"
echo "  🚀 LAUNCHING DEMO COMPONENTS"
echo "═══════════════════════════════════════════════════════════════════════════"
echo ""

# Change to scripts directory
cd ~/AirSim/my_drone_scripts

# Launch ROS2 AirSim bridge in background
echo "1️⃣  Starting ROS2 AirSim Bridge..."
cd ~/AirSim/ros2
source install/setup.bash
ros2 launch airsim_ros_pkgs airsim_node.launch.py &
BRIDGE_PID=$!
sleep 4
echo -e "${GREEN}   ✅ ROS2 Bridge started (PID: $BRIDGE_PID)${NC}"
echo ""

# Spawn entities (people and animals)
echo "2️⃣  Spawning People and Animals in Environment..."
cd ~/AirSim/my_drone_scripts
python3 spawn_entities.py &
SPAWN_PID=$!
sleep 5
echo -e "${GREEN}   ✅ Entities spawned (PID: $SPAWN_PID)${NC}"
echo ""

# Launch RTAB-Map SLAM (if available)
echo "3️⃣  Checking for RTAB-Map SLAM..."
echo ""

if command -v rtabmap &> /dev/null; then
    echo -e "${GREEN}   ✅ RTAB-Map found - launching SLAM!${NC}"
    echo ""
    
    # Launch RTAB-Map with Drone0 as primary
    ros2 launch rtabmap_launch rtabmap.launch.py \
        rtabmap_args:="--delete_db_on_start" \
        depth_topic:=/airsim_node/Drone0/lidar/Lidar1 \
        rgb_topic:=/airsim_node/Drone0/front_center/Scene \
        camera_info_topic:=/airsim_node/Drone0/front_center/Scene/camera_info \
        frame_id:=world_ned \
        approx_sync:=true \
        wait_imu_to_init:=false \
        icp_odometry:=true \
        odom_frame_id:=world_ned \
        publish_tf:=true \
        qos:=2 > /tmp/rtabmap.log 2>&1 &
    RTABMAP_PID=$!
    
    echo -e "${GREEN}   ✅ RTAB-Map SLAM started (PID: $RTABMAP_PID)${NC}"
    echo ""
    sleep 5
else
    echo -e "${YELLOW}   ⚠️  RTAB-Map not installed (optional)${NC}"
    echo -e "${CYAN}   To install: sudo apt install ros-humble-rtabmap-ros${NC}"
    echo ""
    RTABMAP_PID=""
fi

# Launch RViz2
echo "4️⃣  Starting RViz2 Visualization..."
echo ""

rviz2 > /tmp/rviz2_mapping.log 2>&1 &
RVIZ_PID=$!
sleep 3
echo -e "${GREEN}   ✅ RViz2 started (PID: $RVIZ_PID)${NC}"
echo ""

if [ -n "$RTABMAP_PID" ]; then
    echo -e "${CYAN}   📊 RViz2 Setup for RTAB-Map SLAM:${NC}"
    echo "      1. Set Fixed Frame to: 'world_ned' or 'map'"
    echo ""
    echo "      2. Add 3D reconstructed map (BEST):"
    echo "         Add -> By topic -> /rtabmap/cloud_map -> PointCloud2"
    echo "         - Color: RGB or Intensity"
    echo "         - Size (m): 0.05"
    echo ""
    echo "      3. Add 2D occupancy grid (like your reference image):"
    echo "         Add -> By topic -> /rtabmap/grid_map -> Map"
    echo ""
    echo "      4. Optional - Add drone positions:"
    echo "         Add -> By topic -> /collaborative_map/drone_positions -> MarkerArray"
    echo ""
    echo -e "${GREEN}   🎯 RTAB-Map builds professional SLAM maps!${NC}"
else
    echo -e "${CYAN}   📊 RViz2 Setup for Point Cloud Mapping:${NC}"
    echo "      1. Set Fixed Frame to: 'world_ned'"
    echo "      2. Add -> /collaborative_map/combined_lidar -> PointCloud2"
    echo "         - Size (m): 0.1"
    echo "         - Color Transformer: 'AxisColor'"
    echo "      3. Optional -> /collaborative_map/drone_positions -> MarkerArray"
fi

echo ""
sleep 2
echo ""

# ═══════════════════════════════════════════════════════════════════════════════
# Step 5: Launch Python Mapping Script
# ═══════════════════════════════════════════════════════════════════════════════

# Launch YOLO detection if available
if [ "$YOLO_AVAILABLE" = true ]; then
    echo "5️⃣  Starting YOLO Person/Animal Detection on Drone0..."
    cd ~/AirSim/my_drone_scripts
    python3 yolo_person_animal_detector.py Drone0 front_center &
    YOLO_PID=$!
    sleep 3
    echo -e "${GREEN}   ✅ YOLO Detection started (PID: $YOLO_PID)${NC}"
    echo ""
else
    echo "5️⃣  ${YELLOW}Skipping YOLO detection (not installed)${NC}"
    YOLO_PID=""
    echo ""
fi

# Launch collaborative 3D mapping (main script)
echo "6️⃣  Starting Collaborative 3D Mapping Mission..."
echo -e "${CYAN}   🚁 All 5 drones will take off and map their sectors${NC}"
echo -e "${CYAN}   ⏱️  This will take several minutes to complete${NC}"
echo ""
sleep 2

cd ~/AirSim/my_drone_scripts
python3 collaborative_3d_mapping.py

echo ""
echo "═══════════════════════════════════════════════════════════════════════════"
echo "  🏁 MAPPING MISSION COMPLETED"
echo "═══════════════════════════════════════════════════════════════════════════"
echo ""

# Show results
if [ -f /tmp/collaborative_map.ply ]; then
    echo -e "${GREEN}✅ 3D Map saved to: /tmp/collaborative_map.ply${NC}"
    POINT_COUNT=$(grep -c "^[0-9-]" /tmp/collaborative_map.ply)
    echo -e "${GREEN}   Total points: $POINT_COUNT${NC}"
fi

if [ -f /tmp/airsim_entity_manifest.txt ]; then
    echo -e "${GREEN}✅ Entity manifest saved to: /tmp/airsim_entity_manifest.txt${NC}"
    ENTITY_COUNT=$(grep -c "^[a-z]" /tmp/airsim_entity_manifest.txt)
    echo -e "${GREEN}   Total entities: $ENTITY_COUNT${NC}"
fi

echo ""
echo "═══════════════════════════════════════════════════════════════════════════"
echo ""
echo "📊 You can keep RViz2 open to view the final map."
echo "💾 Point cloud file can be viewed in CloudCompare, MeshLab, or similar tools."
echo ""
echo "Press Ctrl+C to cleanup and exit..."
echo ""

# Keep alive to show final visualization
while true; do
    sleep 1
done

# Cleanup function (called on exit)
cleanup() {
    echo ""
    echo "═══════════════════════════════════════════════════════════════════════════"
    echo "  🧹 CLEANING UP"
    echo "═══════════════════════════════════════════════════════════════════════════"
    echo ""
    
    [ ! -z "$YOLO_PID" ] && kill $YOLO_PID 2>/dev/null && echo "   ✅ Stopped YOLO detection"
    [ ! -z "$SPAWN_PID" ] && kill $SPAWN_PID 2>/dev/null && echo "   ✅ Stopped entity spawner"
    [ ! -z "$RVIZ_PID" ] && kill $RVIZ_PID 2>/dev/null && echo "   ✅ Stopped RViz2"
    [ ! -z "$BRIDGE_PID" ] && kill $BRIDGE_PID 2>/dev/null && echo "   ✅ Stopped ROS2 bridge"
    
    echo ""
    echo -e "${GREEN}✅ All processes terminated${NC}"
    echo ""
    echo "═══════════════════════════════════════════════════════════════════════════"
    echo "  🎉 THANK YOU FOR RUNNING THE DEMO!"
    echo "═══════════════════════════════════════════════════════════════════════════"
    echo ""
    echo "Results saved:"
    echo "  • 3D Map: /tmp/collaborative_map.ply"
    echo "  • Entities: /tmp/airsim_entity_manifest.txt"
    echo ""
    echo "🍪 Here's your cookie! You've successfully demonstrated:"
    echo "   ✅ Multi-drone collaborative mapping"
    echo "   ✅ Real-time 3D point cloud generation"
    echo "   ✅ YOLO person/animal detection"
    echo "   ✅ ROS2 localization and visualization"
    echo ""
    exit 0
}

# Trap Ctrl+C and call cleanup
trap cleanup SIGINT SIGTERM

# Wait indefinitely
wait
