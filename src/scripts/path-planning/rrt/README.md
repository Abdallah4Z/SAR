# 3D RRT★ Path Planning with Obstacle Visualization

## 🎯 Features

✅ Complete RRT★ algorithm with performance tracking  
✅ **Visible obstacles in Unreal Engine** (red wireframe spheres)  
✅ **Visible planned path** (green line in Unreal Engine)  
✅ Real-time performance metrics (time, memory, CPU, power)  
✅ Autonomous drone navigation  

---

## 📁 Files

- **`rrt_star_3d.py`** - Core RRT★ algorithm with performance metrics
- **`airsim_rrt_star_drone.py`** - Main script for AirSim integration
- **`visualize_obstacles.py`** - Obstacle visualization in Unreal Engine
- **`performance_analyzer.py`** - Benchmark tool for different scenarios
- **`requirements.txt`** - Python dependencies

---

## 🚀 Quick Start

### 1. Install Dependencies
```bash
cd /home/belal/MyFiles/UNReal_Projects/MyProject2/rrt
pip3 install -r requirements.txt --user
```

### 2. Start Unreal Engine with AirSim
```bash
~/MyFiles/UnrealEngine/Engine/Binaries/Linux/UE4Editor ~/MyFiles/UNReal_Projects/MyProject2/MyProject2.uproject
```

### 3. Run the Path Planner
```bash
cd /home/belal/MyFiles/UNReal_Projects/MyProject2/rrt
python3 airsim_rrt_star_drone.py
```

---

## 👁️ What You'll See in Unreal Engine

### **Red Wireframe Spheres** = Obstacles
- Randomly generated around the environment
- Drone must avoid these during navigation
- Visible as 3D wireframe circles

### **Green Line** = Planned Path
- Optimal path computed by RRT★
- Shows the route the drone will follow
- Connects start to goal while avoiding obstacles

### **Blue Drone** = Your Drone 1
- Will autonomously navigate the green path
- Avoids all red obstacles

---

## 📊 Performance Metrics

After the mission, you'll see:

```
⏱️  TIMING ANALYSIS:
  Planning Time:          3.456 seconds
  Execution Time:         45.123 seconds
  Total Time:             48.579 seconds

💾 MEMORY USAGE:
  Peak Memory:            87.34 MB
  Memory Increase:        12.56 MB

🔋 CPU UTILIZATION:
  Average CPU:            34.56%
  Peak CPU:               78.90%

🧮 ALGORITHM STATISTICS:
  Iterations:             673
  Nodes Generated:        645
  Collision Checks:       1234
  Rewiring Operations:    89

🛤️  PATH QUALITY:
  Path Length:            156.78 meters
  Waypoints:              31
  Optimal Cost:           156.78

🎯 SUITABILITY: ✅ EXCELLENT - Suitable for real-time drone navigation
```

Detailed report saved to `performance_report.txt`

---

## ⚙️ Configuration

Edit in `airsim_rrt_star_drone.py` main() function:

```python
# Change start and goal
start_pos = [0, 0, -10]        # Start position
goal_pos = [80, 80, -20]       # Goal position

# Adjust difficulty
obstacle_count=30              # More obstacles = harder
max_iter=1000                  # More iterations = better path
velocity=5.0                   # Flight speed (m/s)

# Environment size
boundary=((-100, 100),         # X range
          (-100, 100),         # Y range  
          (-30, -5))           # Z range (altitude)
```

---

## 🔧 Troubleshooting

### **Obstacles not visible?**
- The script uses wireframe visualization (works with all AirSim versions)
- Look for red circular wireframes in 3D space
- Objects persist for 10 minutes
- Run `clear_all_plots()` to remove old visualizations

### **Can't see the path?**
- Green line shows the planned route
- Make sure visualization=True in the script
- Path is drawn before drone starts flying

### **"Failed to connect to AirSim"**
- Ensure Unreal Engine is running
- Wait for environment to fully load
- Check AirSim plugin is active

### **"No path found"**
- Increase `max_iter` (try 1500 or 2000)
- Reduce `obstacle_count`
- Make goal closer to start
- Check that goal isn't inside an obstacle

---

## 🎨 Visualization Colors

| Color | Meaning |
|-------|---------|
| 🔴 Red wireframes | Obstacles (avoid these) |
| 🟢 Green line | Planned optimal path |
| 🔵 Blue lines | Tree exploration (in offline plot only) |

---

## 📈 Performance Analysis

Run benchmark tests:
```bash
python3 performance_analyzer.py
```

This will:
- Test Easy, Medium, and Hard scenarios
- Compare planning time, memory, CPU usage
- Generate performance graphs
- Estimate battery/power consumption
- Save results to `benchmark_results.json`

---

## 💡 Tips

1. **Start Simple**: Use fewer obstacles (10-15) for first tests
2. **Adjust Speed**: Lower velocity for better accuracy
3. **Monitor**: Watch performance metrics to optimize settings
4. **Battery**: Check estimated power consumption before long missions
5. **Clear Old Plots**: Obstacles visualization persists - restart if needed

---

## 🐛 Known Limitations

- Obstacles are virtual (for planning) but visualized as wireframes
- Single drone control (Drone 1 only)
- Static obstacles (no dynamic updates during flight)
- Path computed before flight (no real-time replanning)

---

## 📞 Need Help?

Check `QUICKSTART.md` for detailed setup instructions!

---

**Happy Flying! 🚁✨**
