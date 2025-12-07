#!/bin/bash

# Standalone Fire Detection Demo Runner
# This script properly sources ROS2 before running the fire detection demo

echo "═══════════════════════════════════════════════════════════════════════════════"
echo "🔥 Fire Detection Swarm Demo - Standalone Runner"
echo "═══════════════════════════════════════════════════════════════════════════════"
echo ""

# Check if AirSim is running
if ! pgrep -f "AirSimNH" > /dev/null; then
    echo "❌ AirSim is NOT running!"
    echo ""
    echo "Please start AirSim first in a separate terminal:"
    echo "  cd ~/Downloads/AirSimNH/LinuxNoEditor"
    echo "  ./AirSimNH.sh -windowed -ResX=1280 -ResY=720"
    echo ""
    exit 1
fi

echo "✅ AirSim is running"
echo ""

# Source ROS2 environment
echo "📦 Sourcing ROS2 environment..."
cd ~/AirSim/ros2
source install/setup.bash

# Check if ROS2 bridge is running
if ! pgrep -f "airsim_node" > /dev/null; then
    echo "⚠️  ROS2 AirSim bridge is not running"
    echo "   Starting it in the background..."
    ros2 launch airsim_ros_pkgs airsim_node.launch.py > /tmp/airsim_ros2_bridge.log 2>&1 &
    sleep 3
    echo "   ✅ ROS2 bridge started"
fi

# Check if RViz2 is running
if ! pgrep -f "rviz2" > /dev/null; then
    echo "⚠️  RViz2 is not running"
    echo "   You may want to start it in another terminal:"
    echo "   cd ~/AirSim/ros2 && source install/setup.bash && rviz2"
    echo ""
fi

echo ""
echo "🚀 Launching Fire Detection Demo..."
echo "═══════════════════════════════════════════════════════════════════════════════"
echo ""

# Run the fire detection script with ROS2 environment sourced
cd ~/AirSim/my_drone_scripts
python3 fire_detection_swarm.py

echo ""
echo "═══════════════════════════════════════════════════════════════════════════════"
echo "Demo completed!"
echo "═══════════════════════════════════════════════════════════════════════════════"
