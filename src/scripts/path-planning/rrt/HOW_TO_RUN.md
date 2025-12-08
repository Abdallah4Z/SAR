# 🚁 HOW TO RUN RRT★ WITH VISIBLE OBSTACLES

## ⚠️ IMPORTANT: READ THIS CAREFULLY!

Your obstacles **ARE** being spawned in the code, but you need to follow these steps **IN ORDER**:

---

## 📋 STEP-BY-STEP INSTRUCTIONS

### 1️⃣ START UNREAL ENGINE
```bash
cd /home/belal/MyFiles/UNReal_Projects/MyProject2
./MyProject2.uproject
```
**WAIT** for Unreal to fully load!

---

### 2️⃣ PRESS PLAY IN UNREAL ENGINE
- Click the **PLAY** button (▶️) at the top of Unreal Editor
- Wait for the simulation to start
- You should see Drone1 appear in the scene

---

### 3️⃣ RUN THE PYTHON SCRIPT
Open a **NEW terminal** and run:
```bash
cd /home/belal/MyFiles/UNReal_Projects/MyProject2/rrt
python3 airsim_rrt_star_drone.py
```

---

## ✅ WHAT YOU SHOULD SEE

### In Terminal:
```
Connecting to AirSim...
✓ Connected to AirSim successfully

[1/6] Generating random obstacles...
[2/6] Spawning REAL obstacles in Unreal Engine...
  Spawning 30 obstacles in Unreal Engine...
  Spawned 10/30 obstacles...
  Spawned 20/30 obstacles...
  Spawned 30/30 obstacles...
✓ Successfully spawned 30 visible obstacles!

[3/6] Planning path with RRT★...
...
```

### In Unreal Engine:
- ✅ **Spheres or Cubes appearing** (the obstacles)
- ✅ **Green line** (the path)
- ✅ **Drone flying** avoiding obstacles

---

## ❌ TROUBLESHOOTING

### Problem: "Could not spawn physical obstacles"
**Reason**: Your Unreal project doesn't have basic shapes (Sphere/Cube meshes)

**Solution**: The code tries multiple mesh types:
- Sphere
- SM_Sphere
- DefaultSphere
- Cube
- SM_Cube
- DefaultCube
- Cylinder

If NONE exist, obstacles will be virtual (invisible but still work for collision avoidance).

**To add basic shapes to Unreal:**
1. Open Unreal Engine
2. Window → Content Browser
3. Add Basic Shapes plugin if not enabled
4. Or import basic mesh assets

---

### Problem: "Unreal Engine crashed"
**Reason**: Running script BEFORE pressing Play

**Solution**: Always press PLAY in Unreal FIRST, THEN run Python script!

---

### Problem: "Address already in use"
**Reason**: AirSim API server already running

**Solution**: This is just a warning - ignore it! The script will still work.

---

## 🎯 WHAT CHANGES EACH RUN

✅ **Random Goal Position** - Goal changes each time (60-120m from start)
✅ **Random Obstacles** - 30 new obstacles generated each run
✅ **Fixed Start** - Drone starts at (0, 0, -10) always

---

## 🔧 CURRENT CONFIGURATION

```python
Start Position: [0, 0, -10]  (FIXED)
Goal Position: RANDOM (60-120m away from start)
Obstacles: 30 (RANDOM positions and sizes)
Obstacle Radius: 3-8 meters
```

---

## 🚀 QUICK START

**ONE-LINE SUMMARY:**
1. Open Unreal → Press Play → Run `python3 airsim_rrt_star_drone.py`

That's it!

---

## 📝 NOTES

- If obstacles don't appear, they're virtual (still work for collision detection)
- Green path line will ALWAYS appear
- Drone will ALWAYS fly avoiding obstacles
- Each run is different (random goal + random obstacles)

---

## 💡 TIP

Want to see if obstacles spawned?
- Look at terminal output for "✓ Successfully spawned X visible obstacles!"
- If it says 0, obstacles are virtual
- If it says >0, obstacles ARE visible in Unreal

---

**REMEMBER: Unreal MUST be running with PLAY active BEFORE running Python script!**
