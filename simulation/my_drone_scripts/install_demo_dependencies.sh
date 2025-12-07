#!/bin/bash
# Install Dependencies for Advanced Swarm Demo
# ============================================

echo "═══════════════════════════════════════════════════════════════════════════"
echo "  📦 Installing Dependencies for Advanced Swarm Demo"
echo "═══════════════════════════════════════════════════════════════════════════"
echo ""

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Function to check command
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

echo "1️⃣  Checking Python version..."
python3 --version
echo ""

echo "2️⃣  Installing Python packages..."
echo ""

# Core packages
echo "   Installing AirSim Python client..."
pip install airsim || pip3 install airsim
echo ""

echo "   Installing OpenCV..."
pip install opencv-python || pip3 install opencv-python
echo ""

echo "   Installing NumPy..."
pip install numpy || pip3 install numpy
echo ""

# YOLO (optional)
echo "3️⃣  Installing YOLO (optional, but recommended)..."
echo "   This may take a few minutes on first run..."
pip install ultralytics || pip3 install ultralytics
echo ""

# Check if in ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}⚠️  ROS2 environment not detected${NC}"
    echo "   Please source your ROS2 workspace:"
    echo "   source ~/AirSim/ros2/install/setup.bash"
    echo ""
else
    echo -e "${GREEN}✅ ROS2 $ROS_DISTRO detected${NC}"
    echo ""
    
    echo "4️⃣  Installing ROS2 packages..."
    
    # CV Bridge
    if dpkg -l | grep -q ros-$ROS_DISTRO-cv-bridge; then
        echo -e "   ${GREEN}✅ cv-bridge already installed${NC}"
    else
        echo "   Installing cv-bridge..."
        sudo apt update
        sudo apt install -y ros-$ROS_DISTRO-cv-bridge
    fi
    
    # Vision OpenCV
    if dpkg -l | grep -q ros-$ROS_DISTRO-vision-opencv; then
        echo -e "   ${GREEN}✅ vision-opencv already installed${NC}"
    else
        echo "   Installing vision-opencv..."
        sudo apt install -y ros-$ROS_DISTRO-vision-opencv
    fi
    
    echo ""
fi

# Pre-download YOLO model
echo "5️⃣  Pre-downloading YOLO model..."
python3 << EOF
try:
    from ultralytics import YOLO
    print("   Downloading YOLOv8n model...")
    model = YOLO('yolov8n.pt')
    print("   ✅ YOLO model ready!")
except Exception as e:
    print(f"   ⚠️  Could not download YOLO model: {e}")
    print("   It will download automatically on first run.")
EOF
echo ""

echo "═══════════════════════════════════════════════════════════════════════════"
echo "  ✅ INSTALLATION COMPLETE"
echo "═══════════════════════════════════════════════════════════════════════════"
echo ""
echo "📋 Installed packages:"
echo "   • airsim"
echo "   • opencv-python"
echo "   • numpy"
echo "   • ultralytics (YOLO)"
echo ""
echo "🚀 Next steps:"
echo "   1. Make sure AirSim is running"
echo "   2. Run the demo: ./launch_complete_demo.sh"
echo ""
echo "📚 Documentation:"
echo "   Read ADVANCED_DEMO_README.md for full details"
echo ""
