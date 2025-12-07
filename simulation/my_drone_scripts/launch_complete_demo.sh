#!/bin/bash
# Complete Demo Launch Script for Fire Detection + YOLO + RViz2
# ==============================================================

echo "═══════════════════════════════════════════════════════════════════════════"
echo "  🚁 COMPLETE SWARM DEMO: Fire Detection + YOLO + Task Allocation"
echo "═══════════════════════════════════════════════════════════════════════════"
echo ""

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
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
    source ~/AirSim/ros2/install/setup.bash
fi

# Check Python packages
echo ""
echo "📦 Checking Python packages..."
python3 -c "import airsim" 2>/dev/null && echo -e "${GREEN}✅ airsim${NC}" || echo -e "${RED}❌ airsim (pip install airsim)${NC}"
python3 -c "import cv2" 2>/dev/null && echo -e "${GREEN}✅ opencv-python${NC}" || echo -e "${RED}❌ opencv-python (pip install opencv-python)${NC}"
python3 -c "import rclpy" 2>/dev/null && echo -e "${GREEN}✅ rclpy${NC}" || echo -e "${RED}❌ rclpy${NC}"

# Check YOLO (optional)
if python3 -c "import ultralytics" 2>/dev/null; then
    echo -e "${GREEN}✅ ultralytics (YOLO)${NC}"
    YOLO_AVAILABLE=true
else
    echo -e "${YELLOW}⚠️  ultralytics not installed. YOLO detection will be skipped.${NC}"
    echo -e "${YELLOW}   Install with: pip install ultralytics${NC}"
    YOLO_AVAILABLE=false
fi

echo ""
echo "═══════════════════════════════════════════════════════════════════════════"
echo "  📝 DEMO OVERVIEW"
echo "═══════════════════════════════════════════════════════════════════════════"
echo ""
echo "This demo will showcase:"
echo "  1. 🔥 Fire Detection - Virtual fire locations with real-time monitoring"
echo "  2. 📋 Task Allocation - Different roles for each drone in the swarm"
echo "  3. 🎯 YOLO Detection - Object detection on the scout drone (if available)"
echo "  4. 📊 RViz2 Visualization - Real-time 3D visualization of everything"
echo ""
echo "Drone Roles:"
echo "  • Drone0: 🔍 Scout with YOLO object detection"
echo "  • Drone1: 🔥 Fire Monitor #1"
echo "  • Drone2: 🔥 Fire Monitor #2"
echo "  • Drone3: 🛡️  Perimeter Patrol"
echo "  • Drone4: 🛡️  Perimeter Patrol"
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
read -p "Press ENTER when AirSim is ready..."

echo ""
echo "═══════════════════════════════════════════════════════════════════════════"
echo "  🚀 LAUNCHING DEMO COMPONENTS"
echo "═══════════════════════════════════════════════════════════════════════════"
echo ""

# Launch ROS2 AirSim bridge in background
echo "1️⃣  Starting ROS2 AirSim Bridge..."
cd ~/AirSim/ros2
source install/setup.bash
ros2 launch airsim_ros_pkgs airsim_node.launch.py &
BRIDGE_PID=$!
sleep 3
echo -e "${GREEN}   ✅ ROS2 Bridge started (PID: $BRIDGE_PID)${NC}"
echo ""

# Launch RViz2 in background
echo "2️⃣  Starting RViz2..."
rviz2 &
RVIZ_PID=$!
sleep 2
echo -e "${GREEN}   ✅ RViz2 started (PID: $RVIZ_PID)${NC}"
echo ""
echo -e "${YELLOW}   ⚠️  Configure RViz2:${NC}"
echo "      - Set Fixed Frame: 'world_ned'"
echo "      - Add -> By topic -> /fire_markers -> MarkerArray"
echo "      - Add -> By topic -> /task_markers -> MarkerArray"
echo "      - Add -> By topic -> /Drone0/detections_3d -> MarkerArray (if YOLO enabled)"
echo "      - Add -> By topic -> /airsim_node/*/lidar/* -> PointCloud2"
echo ""
read -p "Press ENTER when RViz2 is configured..."
echo ""

# Launch YOLO detection if available
if [ "$YOLO_AVAILABLE" = true ]; then
    echo "3️⃣  Starting YOLO Detection on Drone0..."
    cd ~/AirSim/my_drone_scripts
    python3 yolo_detection_ros2.py Drone0 0 &
    YOLO_PID=$!
    sleep 2
    echo -e "${GREEN}   ✅ YOLO Detection started (PID: $YOLO_PID)${NC}"
    echo ""
else
    echo "3️⃣  ${YELLOW}Skipping YOLO detection (not installed)${NC}"
    echo ""
fi

# Launch main fire detection swarm script
echo "4️⃣  Starting Fire Detection Swarm with Task Allocation..."
cd ~/AirSim/my_drone_scripts
sleep 2
python3 fire_detection_swarm.py

echo ""
echo "═══════════════════════════════════════════════════════════════════════════"
echo "  🏁 DEMO COMPLETED"
echo "═══════════════════════════════════════════════════════════════════════════"
echo ""

# Cleanup
echo "🧹 Cleaning up..."
[ ! -z "$YOLO_PID" ] && kill $YOLO_PID 2>/dev/null
kill $RVIZ_PID 2>/dev/null
kill $BRIDGE_PID 2>/dev/null

echo ""
echo -e "${GREEN}✅ All processes terminated${NC}"
echo ""
echo "Thank you for watching the demo! 🎉"
echo ""
