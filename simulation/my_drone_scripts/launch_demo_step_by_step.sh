#!/bin/bash

# ═══════════════════════════════════════════════════════════════════════════════
# 🚁 AIRSIM ADVANCED DEMO - STEP-BY-STEP LAUNCHER
# ═══════════════════════════════════════════════════════════════════════════════
# This script launches each component with proper waiting times
# Use this if the automated launcher fails
# ═══════════════════════════════════════════════════════════════════════════════

set -e  # Exit on any error

echo ""
echo "═══════════════════════════════════════════════════════════════════════════════"
echo "🚁 AIRSIM ADVANCED DEMO - STEP-BY-STEP LAUNCHER"
echo "═══════════════════════════════════════════════════════════════════════════════"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ═══════════════════════════════════════════════════════════════════════════════
# Step 0: Check AirSim Settings
# ═══════════════════════════════════════════════════════════════════════════════

echo -e "${BLUE}0️⃣  Checking AirSim settings...${NC}"
echo ""

# Check if settings.json has the correct drone names (Drone0-Drone4)
if grep -q '"Drone0"' ~/Documents/AirSim/settings.json 2>/dev/null; then
    echo -e "${GREEN}   ✅ Settings are correct (Drone0-Drone4)${NC}"
    echo ""
else
    echo -e "${YELLOW}   ⚠️  Settings need to be updated!${NC}"
    echo ""
    echo -e "${YELLOW}   The fire detection demo requires drones named: Drone0, Drone1, Drone2, Drone3, Drone4${NC}"
    echo -e "${YELLOW}   Your current settings use different names.${NC}"
    echo ""
    read -p "   Apply correct settings now? (Y/n): " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Nn]$ ]]; then
        echo -e "${BLUE}   Applying fire detection demo settings...${NC}"
        ~/AirSim/my_drone_scripts/apply_fire_demo_settings.sh
        echo ""
        echo -e "${RED}   ⚠️  You MUST restart AirSim if it's already running!${NC}"
        echo ""
    else
        echo -e "${RED}   ⚠️  Warning: Demo may not work with incorrect drone names!${NC}"
        echo ""
    fi
fi

# ═══════════════════════════════════════════════════════════════════════════════
# Step 1: Check if AirSim is running
# ═══════════════════════════════════════════════════════════════════════════════

echo -e "${BLUE}1️⃣  Checking if AirSim is running...${NC}"
echo ""

if pgrep -f "AirSimNH" > /dev/null; then
    echo -e "${GREEN}   ✅ AirSim is already running${NC}"
    echo ""
    
    # Verify drones are available
    echo -e "${BLUE}   Verifying drone availability...${NC}"
    DRONES=$(python3 -c "import airsim; c = airsim.MultirotorClient(); c.confirmConnection(); print(' '.join(c.listVehicles()))" 2>/dev/null || echo "ERROR")
    
    if [[ "$DRONES" == *"Drone0"* ]]; then
        echo -e "${GREEN}   ✅ Drones detected: $DRONES${NC}"
        echo ""
    else
        echo -e "${RED}   ❌ Drone0 not found! Available: $DRONES${NC}"
        echo ""
        echo -e "${YELLOW}   You need to restart AirSim with the correct settings!${NC}"
        echo ""
        read -p "   Press ENTER to continue anyway (may fail) or Ctrl+C to exit..."
        echo ""
    fi
else
    echo -e "${YELLOW}   ⚠️  AirSim is NOT running!${NC}"
    echo ""
    echo -e "${YELLOW}   Please start AirSim FIRST in a separate terminal:${NC}"
    echo ""
    echo -e "   ${GREEN}cd ~/Downloads/AirSimNH/LinuxNoEditor${NC}"
    echo -e "   ${GREEN}./AirSimNH.sh -windowed -ResX=1280 -ResY=720${NC}"
    echo ""
    echo -e "${YELLOW}   Wait until you see the drones spawn in the environment.${NC}"
    echo ""
    read -p "   Press ENTER when AirSim is fully loaded and drones are visible..."
    echo ""
fi

# Wait a bit more to ensure AirSim API is ready
echo -e "${BLUE}   Waiting 5 seconds for AirSim API to be ready...${NC}"
sleep 5
echo ""

# ═══════════════════════════════════════════════════════════════════════════════
# Step 2: Launch ROS2 Bridge
# ═══════════════════════════════════════════════════════════════════════════════

echo -e "${BLUE}2️⃣  Starting ROS2 AirSim Bridge...${NC}"
echo ""

# Check if ROS2 bridge is already running
if pgrep -f "airsim_node" > /dev/null; then
    echo -e "${GREEN}   ✅ ROS2 bridge is already running${NC}"
    echo ""
else
    cd ~/AirSim/ros2
    source install/setup.bash
    ros2 launch airsim_ros_pkgs airsim_node.launch.py > /tmp/airsim_ros2_bridge.log 2>&1 &
    ROS2_PID=$!
    echo -e "${GREEN}   ✅ ROS2 bridge started (PID: $ROS2_PID)${NC}"
    echo ""
    echo -e "${BLUE}   Waiting 5 seconds for ROS2 topics to initialize...${NC}"
    sleep 5
    echo ""
fi

# ═══════════════════════════════════════════════════════════════════════════════
# Step 3: Launch RViz2
# ═══════════════════════════════════════════════════════════════════════════════

echo -e "${BLUE}3️⃣  Starting RViz2...${NC}"
echo ""

# Check if RViz2 is already running
if pgrep -f "rviz2" > /dev/null; then
    echo -e "${GREEN}   ✅ RViz2 is already running${NC}"
    echo ""
else
    cd ~/AirSim/ros2
    source install/setup.bash
    rviz2 > /tmp/rviz2.log 2>&1 &
    RVIZ_PID=$!
    echo -e "${GREEN}   ✅ RViz2 started (PID: $RVIZ_PID)${NC}"
    echo ""
fi

echo -e "${YELLOW}   ⚠️  Configure RViz2:${NC}"
echo "      1. Set Fixed Frame to: 'world_ned' (top-left dropdown)"
echo "      2. Add -> By topic -> /fire_markers -> MarkerArray"
echo "      3. Add -> By topic -> /task_markers -> MarkerArray"
echo "      4. Add -> By topic -> /Drone0/detections_3d -> MarkerArray"
echo "      5. Add -> By topic -> /airsim_node/Drone1/lidar/Lidar1 -> PointCloud2"
echo ""
read -p "   Press ENTER when RViz2 is configured..."
echo ""

# ═══════════════════════════════════════════════════════════════════════════════
# Step 4: Launch YOLO Detection (Optional)
# ═══════════════════════════════════════════════════════════════════════════════

echo -e "${BLUE}4️⃣  YOLO Object Detection (Optional)${NC}"
echo ""
read -p "   Do you want to enable YOLO object detection? (y/N): " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo -e "${BLUE}   Starting YOLO Detection on Drone0...${NC}"
    echo ""
    cd ~/AirSim/my_drone_scripts
    python3 yolo_detection_ros2.py --drone_name Drone0 --camera 0 > /tmp/yolo_detection.log 2>&1 &
    YOLO_PID=$!
    echo -e "${GREEN}   ✅ YOLO Detection started (PID: $YOLO_PID)${NC}"
    echo ""
    echo -e "${BLUE}   Waiting 3 seconds for YOLO to initialize...${NC}"
    sleep 3
    echo ""
else
    echo -e "${YELLOW}   ⏭️  Skipping YOLO detection${NC}"
    echo ""
fi

# ═══════════════════════════════════════════════════════════════════════════════
# Step 5: Launch Fire Detection Swarm
# ═══════════════════════════════════════════════════════════════════════════════

echo -e "${BLUE}5️⃣  Starting Fire Detection Swarm with Task Allocation...${NC}"
echo ""
echo -e "${YELLOW}   ℹ️  Sourcing ROS2 environment for fire detection script...${NC}"

cd ~/AirSim/ros2
source install/setup.bash
cd ~/AirSim/my_drone_scripts
python3 fire_detection_swarm.py

# ═══════════════════════════════════════════════════════════════════════════════
# Cleanup on exit
# ═══════════════════════════════════════════════════════════════════════════════

echo ""
echo -e "${YELLOW}═══════════════════════════════════════════════════════════════════════════════${NC}"
echo -e "${YELLOW}Demo finished or interrupted${NC}"
echo -e "${YELLOW}═══════════════════════════════════════════════════════════════════════════════${NC}"
echo ""
echo "Logs saved to:"
echo "  - /tmp/airsim_ros2_bridge.log"
echo "  - /tmp/rviz2.log"
if [[ $YOLO_PID ]]; then
    echo "  - /tmp/yolo_detection.log"
fi
echo ""
echo "Background processes are still running."
echo "To stop everything, run:"
echo "  pkill -f 'airsim_node|rviz2|yolo_detection'"
echo ""
