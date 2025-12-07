#!/bin/bash
#
# Complete AirSim Setup and Launch Script
# This script guides you through the entire process from building to flying
#

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

AIRSIM_DIR="/home/ahmed/AirSim"
BLOCKS_DIR="$AIRSIM_DIR/Unreal/Environments/Blocks"

clear
echo -e "${CYAN}"
cat << "EOF"
╔═══════════════════════════════════════════════════════════════╗
║                                                               ║
║     🚁  AirSim Complete Setup & Launch Wizard  🚁            ║
║                                                               ║
║     This will guide you through:                             ║
║     1. Building AirSim and Blocks environment                ║
║     2. Installing Python dependencies                        ║
║     3. Launching Unreal Engine with Blocks                   ║
║     4. Running your first drone flight!                      ║
║                                                               ║
╚═══════════════════════════════════════════════════════════════╝
EOF
echo -e "${NC}"

# Function to check if build is needed
check_build_status() {
    if [ -d "$AIRSIM_DIR/build_release" ] && [ -f "$AIRSIM_DIR/build_release/output/lib/libAirSim.a" ]; then
        return 0  # Build exists
    else
        return 1  # Build needed
    fi
}

# Function to check if Blocks is set up
check_blocks_status() {
    if [ -f "$BLOCKS_DIR/Blocks.uproject" ]; then
        return 0  # Blocks exists
    else
        return 1  # Blocks needs setup
    fi
}

echo ""
echo -e "${YELLOW}═══════════════════════════════════════════════${NC}"
echo -e "${YELLOW}PHASE 1: Build Check${NC}"
echo -e "${YELLOW}═══════════════════════════════════════════════${NC}"
echo ""

if check_build_status; then
    echo -e "${GREEN}✓ AirSim is already built${NC}"
else
    echo -e "${YELLOW}⚠ AirSim needs to be built${NC}"
    echo ""
    echo "This will compile AirSim libraries and tools."
    echo "This process may take 30-60 minutes."
    echo ""
    read -p "Do you want to build AirSim now? (y/n): " build_now
    
    if [ "$build_now" = "y" ] || [ "$build_now" = "Y" ]; then
        echo ""
        echo -e "${BLUE}Building AirSim...${NC}"
        cd "$AIRSIM_DIR"
        ./setup.sh
        ./build.sh
        
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}✓ AirSim built successfully!${NC}"
        else
            echo -e "${RED}✗ Build failed. Please check error messages above.${NC}"
            exit 1
        fi
    else
        echo -e "${YELLOW}Please build AirSim first using: cd $AIRSIM_DIR && ./build.sh${NC}"
        exit 0
    fi
fi

echo ""
echo -e "${YELLOW}═══════════════════════════════════════════════${NC}"
echo -e "${YELLOW}PHASE 2: Blocks Environment Setup${NC}"
echo -e "${YELLOW}═══════════════════════════════════════════════${NC}"
echo ""

if check_blocks_status; then
    echo -e "${GREEN}✓ Blocks environment is already set up${NC}"
    echo -e "  Location: $BLOCKS_DIR"
else
    echo -e "${YELLOW}⚠ Blocks environment needs setup${NC}"
    echo ""
    echo "The Blocks environment is a simple test environment for AirSim."
    echo "It provides a basic scene with blocks/buildings to fly around."
    echo ""
    read -p "Do you want to set up Blocks now? (y/n): " setup_blocks
    
    if [ "$setup_blocks" = "y" ] || [ "$setup_blocks" = "Y" ]; then
        echo ""
        echo -e "${BLUE}Setting up Blocks environment...${NC}"
        cd "$BLOCKS_DIR"
        
        if [ -f "./update_from_git.sh" ]; then
            ./update_from_git.sh
            echo -e "${GREEN}✓ Blocks environment created!${NC}"
        else
            echo -e "${RED}✗ update_from_git.sh not found. Check your AirSim installation.${NC}"
            exit 1
        fi
        
        echo ""
        read -p "Do you want to package Blocks for faster startup? (y/n): " package_blocks
        if [ "$package_blocks" = "y" ] || [ "$package_blocks" = "Y" ]; then
            echo -e "${BLUE}Packaging Blocks (this will take a while)...${NC}"
            ./package.sh
        fi
    else
        echo -e "${YELLOW}Skipping Blocks setup. You can set it up later.${NC}"
    fi
fi

echo ""
echo -e "${YELLOW}═══════════════════════════════════════════════${NC}"
echo -e "${YELLOW}PHASE 3: Python Setup${NC}"
echo -e "${YELLOW}═══════════════════════════════════════════════${NC}"
echo ""

# Check Python
if command -v python3 &> /dev/null; then
    PYTHON_VERSION=$(python3 --version)
    echo -e "${GREEN}✓ $PYTHON_VERSION${NC}"
else
    echo -e "${RED}✗ Python 3 not found. Please install Python 3.${NC}"
    exit 1
fi

# Install AirSim package
echo ""
echo -e "${BLUE}Installing AirSim Python package...${NC}"
if python3 -c "import airsim" 2>/dev/null; then
    echo -e "${GREEN}✓ AirSim package already installed${NC}"
else
    pip install airsim tornado msgpack-rpc-python --user
    if python3 -c "import airsim" 2>/dev/null; then
        echo -e "${GREEN}✓ AirSim package installed successfully${NC}"
    else
        echo -e "${RED}✗ Failed to install AirSim package${NC}"
        exit 1
    fi
fi

# Create settings.json
echo ""
echo -e "${BLUE}Setting up AirSim configuration...${NC}"
AIRSIM_SETTINGS_DIR=~/Documents/AirSim
mkdir -p "$AIRSIM_SETTINGS_DIR"

if [ -f "$AIRSIM_SETTINGS_DIR/settings.json" ]; then
    echo -e "${GREEN}✓ settings.json already exists${NC}"
else
    cat > "$AIRSIM_SETTINGS_DIR/settings.json" << 'EOF'
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "ClockSpeed": 1,
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "X": 0,
      "Y": 0,
      "Z": 0,
      "Yaw": 0
    }
  },
  "CameraDefaults": {
    "CaptureSettings": [
      {
        "ImageType": 0,
        "Width": 1920,
        "Height": 1080,
        "FOV_Degrees": 90
      }
    ]
  }
}
EOF
    echo -e "${GREEN}✓ settings.json created${NC}"
fi

echo ""
echo -e "${YELLOW}═══════════════════════════════════════════════${NC}"
echo -e "${YELLOW}PHASE 4: Launch Unreal Engine${NC}"
echo -e "${YELLOW}═══════════════════════════════════════════════${NC}"
echo ""

echo "How do you want to launch the Blocks environment?"
echo ""
echo "  1) Packaged Binary (Fast - if you packaged earlier)"
echo "  2) Unreal Editor (Flexible - can modify environment)"
echo "  3) I'll launch manually (show me instructions)"
echo "  4) Skip - Unreal is already running"
echo ""
read -p "Enter choice [1-4]: " launch_choice

case $launch_choice in
    1)
        BINARY_PATH="$BLOCKS_DIR/LinuxNoEditor/Blocks.sh"
        if [ -f "$BINARY_PATH" ]; then
            echo ""
            echo -e "${GREEN}🎮 Launching Blocks binary...${NC}"
            cd "$BLOCKS_DIR/LinuxNoEditor"
            ./Blocks.sh -ResX=1280 -ResY=720 -windowed &
            UNREAL_PID=$!
            echo -e "${GREEN}✓ Blocks launched (PID: $UNREAL_PID)${NC}"
            echo -e "${BLUE}  Waiting 20 seconds for environment to fully load...${NC}"
            sleep 20
        else
            echo -e "${RED}✗ Packaged binary not found at: $BINARY_PATH${NC}"
            echo "Please package it first using: cd $BLOCKS_DIR && ./package.sh"
            exit 1
        fi
        ;;
    2)
        echo ""
        echo -e "${CYAN}📖 Unreal Editor Launch Instructions:${NC}"
        echo "=================================================="
        echo ""
        echo "Run this command in a new terminal:"
        echo ""
        echo -e "${YELLOW}cd $BLOCKS_DIR${NC}"
        echo -e "${YELLOW}~/UnrealEngine/Engine/Binaries/Linux/UE4Editor \$(pwd)/Blocks.uproject${NC}"
        echo ""
        echo "Then in the Unreal Editor:"
        echo "  1. Wait for the editor to fully load (1-2 minutes)"
        echo "  2. Click the green 'Play' button (▶) at the top"
        echo "  3. Or press Alt+P"
        echo ""
        echo "=================================================="
        echo ""
        read -p "Press Enter when you have launched Unreal and pressed Play..."
        ;;
    3)
        echo ""
        echo -e "${CYAN}📖 Manual Launch Options:${NC}"
        echo "=================================================="
        echo ""
        echo "Method 1 - Packaged Binary:"
        echo -e "${YELLOW}  cd $BLOCKS_DIR/LinuxNoEditor${NC}"
        echo -e "${YELLOW}  ./Blocks.sh -ResX=1280 -ResY=720 -windowed${NC}"
        echo ""
        echo "Method 2 - Unreal Editor:"
        echo -e "${YELLOW}  cd $BLOCKS_DIR${NC}"
        echo -e "${YELLOW}  ~/UnrealEngine/Engine/Binaries/Linux/UE4Editor Blocks.uproject${NC}"
        echo "  Then click Play button"
        echo ""
        echo "=================================================="
        echo ""
        read -p "Press Enter when Unreal is running and environment is loaded..."
        ;;
    4)
        echo -e "${GREEN}✓ Assuming Unreal is already running${NC}"
        ;;
    *)
        echo -e "${RED}Invalid choice${NC}"
        exit 1
        ;;
esac

echo ""
echo -e "${YELLOW}═══════════════════════════════════════════════${NC}"
echo -e "${YELLOW}PHASE 5: Test Connection${NC}"
echo -e "${YELLOW}═══════════════════════════════════════════════${NC}"
echo ""

cd "$AIRSIM_DIR/my_drone_scripts"

echo -e "${BLUE}Testing connection to AirSim...${NC}"
python3 << 'PYEOF'
import sys
try:
    import airsim
    print("✓ AirSim package loaded")
    
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("✓ Connected to AirSim simulator!")
    
    state = client.getMultirotorState()
    print(f"✓ Drone detected - State: {state.landed_state}")
    
    pos = state.kinematics_estimated.position
    print(f"✓ Drone position: X={pos.x_val:.2f}, Y={pos.y_val:.2f}, Z={pos.z_val:.2f}")
    
except Exception as e:
    print(f"✗ Connection failed: {e}")
    print("\nTroubleshooting:")
    print("  1. Make sure Unreal is running (you should see the environment)")
    print("  2. Make sure you clicked 'Play' in Unreal Editor")
    print("  3. Check that ~/Documents/AirSim/settings.json exists")
    print("  4. Try restarting Unreal")
    sys.exit(1)
PYEOF

if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}═══════════════════════════════════════════════${NC}"
    echo -e "${GREEN}   ✓✓✓ All Systems Ready! ✓✓✓${NC}"
    echo -e "${GREEN}═══════════════════════════════════════════════${NC}"
    echo ""
    echo -e "${YELLOW}═══════════════════════════════════════════════${NC}"
    echo -e "${YELLOW}PHASE 6: Fly Your Drone! 🚁${NC}"
    echo -e "${YELLOW}═══════════════════════════════════════════════${NC}"
    echo ""
    echo "Available flight scripts:"
    echo ""
    echo "  1) basic_takeoff_land.py    - Simple test (RECOMMENDED FIRST)"
    echo "  2) simple_flight.py          - Square pattern flight"
    echo "  3) advanced_flight.py        - Camera + sensors + circle"
    echo "  4) Exit (run scripts manually later)"
    echo ""
    read -p "Choose a script to run [1-4]: " script_choice
    
    case $script_choice in
        1)
            echo ""
            echo -e "${CYAN}🚁 Running: Basic Takeoff & Land${NC}"
            echo "=================================================="
            python3 basic_takeoff_land.py
            ;;
        2)
            echo ""
            echo -e "${CYAN}🚁 Running: Square Flight Pattern${NC}"
            echo "=================================================="
            python3 simple_flight.py
            ;;
        3)
            echo ""
            echo -e "${CYAN}🚁 Running: Advanced Flight${NC}"
            echo "=================================================="
            python3 advanced_flight.py
            ;;
        4)
            echo -e "${YELLOW}Exiting. You can run scripts manually from:${NC}"
            echo "  cd $AIRSIM_DIR/my_drone_scripts"
            echo "  python3 basic_takeoff_land.py"
            exit 0
            ;;
        *)
            echo -e "${RED}Invalid choice${NC}"
            ;;
    esac
    
    echo ""
    echo -e "${GREEN}═══════════════════════════════════════════════${NC}"
    echo -e "${GREEN}   🎉 Flight Complete! 🎉${NC}"
    echo -e "${GREEN}═══════════════════════════════════════════════${NC}"
    echo ""
    echo -e "${BLUE}Next Steps:${NC}"
    echo "  • Run more scripts: cd $AIRSIM_DIR/my_drone_scripts"
    echo "  • Modify scripts to create custom flight patterns"
    echo "  • Read the guide: $AIRSIM_DIR/QUICK_START_GUIDE.md"
    echo "  • Explore examples: $AIRSIM_DIR/PythonClient/multirotor/"
    echo ""
else
    echo ""
    echo -e "${RED}═══════════════════════════════════════════════${NC}"
    echo -e "${RED}   ✗ Connection Failed${NC}"
    echo -e "${RED}═══════════════════════════════════════════════${NC}"
    echo ""
    echo "Please check that:"
    echo "  1. Unreal Engine is running"
    echo "  2. You pressed Play in the editor"
    echo "  3. The environment has fully loaded"
    echo "  4. No error messages in the Unreal console"
    echo ""
    echo "Then run this script again or manually run:"
    echo "  cd $AIRSIM_DIR/my_drone_scripts"
    echo "  python3 basic_takeoff_land.py"
fi
