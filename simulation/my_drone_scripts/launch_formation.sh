#!/bin/bash

# Launch Formation Flight Controller

echo "=========================================="
echo "  🚁 Formation Flight Launcher"
echo "=========================================="
echo ""

# Check if AirSim is running
if ! pgrep -x "AirSimNH" > /dev/null; then
    echo "❌ AirSimNH is not running!"
    echo ""
    echo "Please start AirSimNH first:"
    echo "  cd ~/Downloads/AirSimNH/LinuxNoEditor/"
    echo "  ./AirSimNH.sh -ResX=1920 -ResY=1080 -windowed"
    echo ""
    exit 1
fi

echo "✅ AirSimNH detected"
echo ""
echo "🚀 Launching formation flight controller..."
echo "   • Press V for 3D lidar visualization"
echo "   • Press B to toggle formation"
echo "   • Improved controls (8 m/s, 45°/s yaw)"
echo ""

cd ~/AirSim/my_drone_scripts
python3 formation_flight_no_viz.py
