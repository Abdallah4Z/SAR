#!/bin/bash

# ═══════════════════════════════════════════════════════════════════════════════
# 🗺️  COLLABORATIVE SLAM MAPPING WITH RTAB-MAP
# ═══════════════════════════════════════════════════════════════════════════════
# Uses RTAB-Map for proper 3D SLAM reconstruction
# All 5 drones contribute to building one unified 3D map
# ═══════════════════════════════════════════════════════════════════════════════

set -e

echo ""
echo "═══════════════════════════════════════════════════════════════════════════════"
echo "🗺️  AIRSIM COLLABORATIVE SLAM MAPPING"
echo "═══════════════════════════════════════════════════════════════════════════════"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# ═══════════════════════════════════════════════════════════════════════════════
# Step 1: Check if RTAB-Map is installed
# ═══════════════════════════════════════════════════════════════════════════════

echo -e "${BLUE}1️⃣  Checking RTAB-Map installation...${NC}"
echo ""

if ! command -v rtabmap &> /dev/null; then
    echo -e "${YELLOW}   ⚠️  RTAB-Map not found!${NC}"
    echo ""
    echo -e "${CYAN}   Installing RTAB-Map (this may take a few minutes)...${NC}"
    echo ""
    sudo apt update
    sudo apt install -y ros-humble-rtabmap-ros
    echo ""
    echo -e "${GREEN}   ✅ RTAB-Map installed!${NC}"
    echo ""
else
    echo -e "${GREEN}   ✅ RTAB-Map is installed${NC}"
    echo ""
fi

# ═══════════════════════════════════════════════════════════════════════════════
# Step 2: Check AirSim
# ═══════════════════════════════════════════════════════════════════════════════

echo -e "${BLUE}2️⃣  Checking AirSim...${NC}"
echo ""

if ! pgrep -f "AirSimNH" > /dev/null; then
    echo -e "${YELLOW}   ⚠️  AirSim is NOT running!${NC}"
    echo ""
    echo -e "${YELLOW}   Please start AirSim first:${NC}"
    echo ""
    echo -e "   ${GREEN}cd ~/Downloads/AirSimNH/LinuxNoEditor${NC}"
    echo -e "   ${GREEN}./AirSimNH.sh -windowed -ResX=1280 -ResY=720${NC}"
    echo ""
    read -p "   Press ENTER when AirSim is running..."
    echo ""
fi

echo -e "${GREEN}   ✅ AirSim is running${NC}"
echo ""
sleep 2

# ═══════════════════════════════════════════════════════════════════════════════
# Step 3: Launch ROS2 Bridge
# ═══════════════════════════════════════════════════════════════════════════════

echo -e "${BLUE}3️⃣  Starting ROS2 AirSim Bridge...${NC}"
echo ""

if pgrep -f "airsim_node" > /dev/null; then
    echo -e "${GREEN}   ✅ ROS2 bridge already running${NC}"
    echo ""
else
    cd ~/AirSim/ros2
    source install/setup.bash
    ros2 launch airsim_ros_pkgs airsim_node.launch.py > /tmp/airsim_ros2_bridge.log 2>&1 &
    ROS2_PID=$!
    echo -e "${GREEN}   ✅ ROS2 bridge started (PID: $ROS2_PID)${NC}"
    echo ""
    sleep 5
fi

# ═══════════════════════════════════════════════════════════════════════════════
# Step 4: Launch RTAB-Map SLAM
# ═══════════════════════════════════════════════════════════════════════════════

echo -e "${BLUE}4️⃣  Starting RTAB-Map SLAM...${NC}"
echo ""

cd ~/AirSim/ros2
source install/setup.bash

# Launch RTAB-Map with Drone0 as primary
# Will subscribe to all drone LiDAR topics
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

echo -e "${GREEN}   ✅ RTAB-Map started (PID: $RTABMAP_PID)${NC}"
echo ""
echo -e "${CYAN}   📝 RTAB-Map log: /tmp/rtabmap.log${NC}"
echo ""
sleep 5

# ═══════════════════════════════════════════════════════════════════════════════
# Step 5: Launch RViz with RTAB-Map visualization
# ═══════════════════════════════════════════════════════════════════════════════

echo -e "${BLUE}5️⃣  Starting RViz2 with RTAB-Map visualization...${NC}"
echo ""

rviz2 > /tmp/rviz2_slam.log 2>&1 &
RVIZ_PID=$!

echo -e "${GREEN}   ✅ RViz2 started (PID: $RVIZ_PID)${NC}"
echo ""

echo -e "${CYAN}   📊 RViz2 Setup for SLAM:${NC}"
echo "      1. Set Fixed Frame to: 'world_ned' or 'map'"
echo ""
echo "      2. Add RTAB-Map visualization:"
echo "         Add -> By topic -> /rtabmap/cloud_map -> PointCloud2"
echo "         - Color: RGB or Intensity"
echo "         - Size: 0.05"
echo ""
echo "      3. Add 3D occupancy grid:"
echo "         Add -> By topic -> /rtabmap/octomap_occupied_space -> PointCloud2"
echo ""
echo "      4. Add 2D map projection:"
echo "         Add -> By topic -> /rtabmap/grid_map -> OccupancyGrid or Map"
echo ""
echo "      5. Optional - Add drone markers:"
echo "         Add -> By topic -> /collaborative_map/drone_positions -> MarkerArray"
echo ""
echo -e "${YELLOW}   💡 RTAB-Map builds a persistent 3D map with loop closure!${NC}"
echo ""
sleep 3

# ═══════════════════════════════════════════════════════════════════════════════
# Step 6: Launch Collaborative Mapping Script
# ═══════════════════════════════════════════════════════════════════════════════

echo -e "${BLUE}6️⃣  Starting collaborative drone mapping...${NC}"
echo ""

cd ~/AirSim/my_drone_scripts
source ~/AirSim/ros2/install/setup.bash

python3 collaborative_3d_mapping.py &
MAPPER_PID=$!

echo -e "${GREEN}   ✅ Collaborative mapper started (PID: $MAPPER_PID)${NC}"
echo ""

# ═══════════════════════════════════════════════════════════════════════════════
# Summary
# ═══════════════════════════════════════════════════════════════════════════════

echo ""
echo "═══════════════════════════════════════════════════════════════════════════════"
echo "  🎉 SLAM MAPPING SYSTEM LAUNCHED!"
echo "═══════════════════════════════════════════════════════════════════════════════"
echo ""
echo -e "${GREEN}Running processes:${NC}"
echo "  • ROS2 Bridge:       PID $ROS2_PID"
echo "  • RTAB-Map SLAM:     PID $RTABMAP_PID"
echo "  • RViz2:             PID $RVIZ_PID"
echo "  • Collaborative Map: PID $MAPPER_PID"
echo ""
echo -e "${CYAN}Key ROS2 Topics:${NC}"
echo "  • /rtabmap/cloud_map          - Full 3D reconstructed map"
echo "  • /rtabmap/grid_map           - 2D occupancy grid"
echo "  • /rtabmap/octomap_*          - 3D occupancy (OctoMap)"
echo "  • /collaborative_map/*        - Drone positions & combined LiDAR"
echo ""
echo -e "${CYAN}Logs:${NC}"
echo "  • RTAB-Map: /tmp/rtabmap.log"
echo "  • RViz2:    /tmp/rviz2_slam.log"
echo "  • ROS2:     /tmp/airsim_ros2_bridge.log"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop all processes${NC}"
echo ""
echo "═══════════════════════════════════════════════════════════════════════════════"
echo ""

# Wait for user interrupt
trap "echo ''; echo 'Stopping all processes...'; kill $ROS2_PID $RTABMAP_PID $RVIZ_PID $MAPPER_PID 2>/dev/null; exit 0" INT

# Keep script running
wait
