#!/bin/bash
# Quick Fix and Test Script
# Fixes common issues and runs simple spawn test

echo "🔧 QUICK FIX AND TEST"
echo "===================="
echo ""

cd ~/AirSim/my_drone_scripts

# Make everything executable
chmod +x *.sh *.py

echo "✅ Made all scripts executable"
echo ""

# Test entity spawning first
echo "🧪 Running simple spawn test..."
echo "   This will spawn 5 visible objects in front of the drones"
echo "   Check your AirSim window to see if spheres/cubes appear"
echo ""

python3 test_spawn_simple.py

echo ""
echo "If spawn test worked, run the full demo with:"
echo "  ./launch_mapping_demo.sh"
echo ""
