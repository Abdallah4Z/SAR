# AirSim Neighborhood in Unreal Engine - Setup Guide

## Problem
The AirSimNH binary doesn't support spawning because it's pre-compiled without editor access.

## Solution Options

### Option 1: Use Blocks Environment (You Already Have This!)
Open your existing Blocks project in Unreal Engine and add animals/people:

```bash
cd ~/AirSim/Unreal/Environments/Blocks
# Open Blocks.uproject with Unreal Engine 4.27
```

**In Unreal Editor:**
1. Open Content Browser → Add animal/character blueprints from Marketplace
2. Enable spawning in Project Settings → Physics → Allow Kinematic to Static
3. Add `DeerBP`, `CharacterBP` or similar assets
4. Save and run!

### Option 2: Download Pre-made Neighborhood Source

The AirSim Neighborhood **source project** isn't publicly available, but you can:

**A) Use Community Environments:**
- Africa environment: https://github.com/Microsoft/AirSim/releases
- LandscapeMountains: Download from Unreal Marketplace
- City Park environments: Community contributions

**B) Create Custom Environment from Blocks:**

```bash
cd ~/AirSim/Unreal/Environments
cp -r "Blocks" "MyCity"
cd MyCity
mv Blocks.uproject MyCity.uproject
```

Then open `MyCity.uproject` in UE4.27 and:
1. Add landscape (Landscape Mode)
2. Import/create buildings
3. Add character blueprints from Marketplace (free)
4. Configure spawning points

### Option 3: Download Assets for Blocks

**Free Unreal Marketplace Assets with Animals/People:**
1. **Animal Variety Pack** (Free)
2. **City People Pack** (Free/Paid options)
3. **Wildlife Pack**

**To add to Blocks environment:**
```bash
# Open Blocks.uproject in Unreal Engine
# File → Add Feature or Content Pack
# Search for "Character" or "Animal"
# Import to project
```

## Quick Fix: Make Blocks Support Your Demo

Since spawning doesn't work in binary, let's use **Blocks environment** with Unreal Engine:

```bash
cd ~/AirSim/Unreal/Environments/Blocks
unreal-engine ~/AirSim/Unreal/Environments/Blocks/Blocks.uproject
```

Or if you have the Unreal Engine 4.27 installed:
```bash
~/UnrealEngine/Engine/Binaries/Linux/UE4Editor ~/AirSim/Unreal/Environments/Blocks/Blocks.uproject
```

### What to do in Unreal Editor:
1. Place some **StaticMesh cubes/spheres** at ground level (these will be "animals")
2. Scale them large (3-5 meters)
3. Apply different materials (brown for "deer", skin tone for "person")
4. Enable collision
5. Play in Editor - now your YOLO detector has targets!

## Recommended Solution for Your Demo

**Use Blocks + Manual Asset Placement:**

1. Open Blocks.uproject in Unreal Engine
2. Drag in some StaticMesh actors (cubes, spheres)
3. Position them around the map at ground level
4. Run AirSim in this environment
5. Your YOLO detector will see them!

This is **much faster** than trying to get the Neighborhood source or fixing spawning issues.

## Command to Launch
```bash
# After modifying Blocks in Unreal Editor, run it:
cd ~/AirSim/Unreal/Environments/Blocks
./LinuxBlocks.sh -windowed
```

Would you like me to create a script to help you modify the Blocks environment?
