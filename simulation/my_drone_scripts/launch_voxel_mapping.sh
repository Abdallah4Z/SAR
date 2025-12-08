#!/bin/bash
# Simple Voxel Grid Mapping Demo

echo "📦 3D VOXEL GRID MAPPING DEMO"
echo "=============================="
echo ""
echo "This builds a clean 3D voxel map (colored cubes)"
echo "Instead of raw point clouds"
echo ""

# Launch RViz with voxel config
echo "1️⃣ Starting RViz..."
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
rviz2 -d "$SCRIPT_DIR/voxel_rviz_config.rviz" &
RVIZ_PID=$!
sleep 3
echo "✅ RViz started"
echo ""

# Start mapping
echo "2️⃣ Starting voxel mapping..."
echo "   Drone will scan 4 horizontal layers"
echo "   Map appears as colored cubes in RViz"
echo ""
python3 "$SCRIPT_DIR/voxel_grid_mapping.py"

# Cleanup
echo ""
echo "🧹 Cleaning up..."
kill $RVIZ_PID 2>/dev/null
echo "✅ Done!"
