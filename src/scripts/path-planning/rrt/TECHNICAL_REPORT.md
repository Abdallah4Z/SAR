# Technical Report: 3D RRT★ Path Planning for UAV Navigation in AirSim

**Project:** Autonomous Drone Navigation with Obstacle Avoidance  
**Algorithm:** Rapidly-Exploring Random Tree Star (RRT★)  
**Environment:** AirSim/Unreal Engine 4.27  
**Date:** November 12, 2025

---

## Executive Summary

This project implements an autonomous navigation system for unmanned aerial vehicles (UAVs) using the RRT★ path planning algorithm in a simulated 3D environment. The system generates collision-free trajectories from a fixed start position to randomly generated goal locations while navigating around dynamically placed obstacles. The implementation has been optimized through parameter tuning to improve path-finding success rates, with real-time obstacle visualization in the AirSim simulator.

---

## 1. Introduction

### 1.1 Motivation

Autonomous drone navigation in complex environments requires robust path planning algorithms capable of:
- Finding optimal collision-free paths in 3D space
- Handling dynamic obstacle configurations
- Computing paths efficiently for real-time operation
- Adapting to varying mission objectives

### 1.2 Objectives

1. Implement RRT★ algorithm for 3D space with obstacle avoidance
2. Integrate path planning with AirSim drone control
3. Generate realistic obstacle scenarios along potential flight paths
4. Visualize planned paths and obstacles in real-time
5. Execute computed trajectories autonomously

---

## 2. System Architecture

### 2.1 Component Overview

The system consists of three primary modules:

**Path Planning Module (`rrt_star_3d.py`)**
- RRT★ algorithm implementation with 3D spatial sampling
- Collision detection using line-sphere intersection
- Path optimization through tree rewiring
- Performance metrics tracking

**Drone Control Module (`airsim_rrt_star_drone.py`)**
- AirSim API integration for drone control
- Obstacle visualization using persistent markers
- Path execution with waypoint navigation
- Safe mode operation to prevent simulation crashes

**Visualization Module (`visualize_obstacles.py`)**
- Real-time path rendering in AirSim
- Obstacle sphere generation with 288-point density
- 3D trajectory visualization

### 2.2 System Workflow

```
1. Initialize AirSim Connection
2. Takeoff & Position Acquisition
3. Generate Random Goal (0-300m range)
4. Create Obstacle Field (20 obstacles, 3-6m radius)
   ├── 40% placed along direct path (8-20m offset)
   └── 60% placed randomly in environment
5. Execute RRT★ Path Planning
   ├── Tree expansion with step_size=8m
   ├── Goal biasing (15% probability)
   ├── Near-neighbor rewiring (20m radius)
   └── Collision checking with 2m safety margin
6. Path Smoothing (optional)
7. Visualize Path in AirSim (green line)
8. Execute Trajectory (5 m/s velocity)
9. Performance Reporting
```

---

## 3. RRT★ Algorithm Implementation

### 3.1 Core Algorithm

The Rapidly-Exploring Random Tree Star (RRT★) is an optimal sampling-based path planning algorithm that improves upon the basic RRT by continuously refining the path cost through tree rewiring.

**Key Features:**
- **Asymptotic Optimality:** Converges to optimal solution as iterations increase
- **Anytime Algorithm:** Provides valid solution early, improves over time
- **Probabilistic Completeness:** Guaranteed to find path if one exists

### 3.2 Algorithm Parameters

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| `step_size` | 8 meters | Balances exploration speed vs. path granularity |
| `goal_sample_rate` | 0.15 | 15% bias toward goal accelerates convergence |
| `max_iterations` | 2000 | Increased from 1000 to allow more search time |
| `search_radius` | 20 meters | Enables effective rewiring without excessive computation |
| `drone_radius` | 2 meters | Safety margin accounting for UAV dimensions + GPS uncertainty |

### 3.3 Collision Detection Enhancement

**Problem:** Standard RRT★ checks collision with obstacle centers only, ignoring physical obstacle size and drone dimensions.

**Solution:** Implemented augmented collision detection:

```python
effective_radius = obstacle_radius + drone_radius
```

This ensures the planned path maintains minimum 2-meter clearance from obstacle surfaces, accounting for:
- Physical drone body dimensions (~1m)
- GPS/positioning uncertainty (~0.5m)
- Safety buffer for real-world deployment (~0.5m)

**Mathematical Formulation:**

For line segment *L* from *p₁* to *p₂* and sphere obstacle *S* with center *c* and radius *r*:

```
Collision if: distance(L, c) < (r + r_drone)
```

Where distance is computed using line-sphere intersection with parametric form.

---

## 4. Obstacle Generation Strategy

### 4.1 Path-Based Obstacle Placement

To create realistic navigation scenarios that require intelligent path planning, obstacles are strategically placed relative to the direct line-of-sight path:

**Distribution:**
- **40% On-Path Obstacles:** Placed along middle 50% of direct path, offset 8-20m perpendicular
- **60% Random Obstacles:** Uniformly distributed throughout environment boundaries

**Benefits:**
- Forces drone to navigate around obstacles (prevents straight-line solutions)
- Maintains solvability (doesn't completely block all paths)
- Creates realistic urban/forest-like environments

### 4.2 Obstacle Characteristics

- **Count:** 20 obstacles per scenario
- **Radius Range:** 3-6 meters (reduced from initial 3-8m for higher success rate)
- **Clearance:** 10-meter minimum distance from start/goal positions
- **Altitude Distribution:** Random within z = -35 to -5 meters

---

## 5. Visualization & Simulation Integration

### 5.1 Safe Mode Operation

Physical object spawning in Unreal Engine 4.27 with AirSim caused segmentation faults (Exit Code 139). The system implements "Safe Mode" visualization:

**Visual Obstacles:**
- 288-point spherical point clouds (12 latitude × 24 longitude divisions)
- Red color coding (RGBA: [1.0, 0.0, 0.0, 1.0])
- Persistent rendering (duration = -1)
- Equator and meridian circle outlines for enhanced definition

**Advantages:**
- Zero crash risk
- Identical algorithmic treatment as physical obstacles
- Real-time rendering performance
- Easy to clear/regenerate

### 5.2 Path Visualization

- **Color:** Green ([0, 1, 0]) for planned trajectory
- **Thickness:** 5.0 units for visibility
- **Rendering:** Persistent line strips connecting waypoints

---

## 6. Performance Metrics

### 6.1 Algorithm Performance

The system tracks comprehensive performance metrics:

**Timing Metrics:**
- Planning Time: Time to compute RRT★ path (typically 2-5 seconds)
- Execution Time: Time to physically fly the path (varies by distance)
- Total Mission Time: End-to-end operation duration

**Resource Metrics:**
- Memory Usage: Initial, peak, and final memory consumption
- CPU Utilization: Average and peak CPU percentage
- Node Count: Number of RRT★ tree nodes generated

**Path Quality Metrics:**
- Path Length: Total trajectory distance in meters
- Waypoint Count: Number of discrete waypoints
- Optimal Cost: RRT★ cost function value
- Collision Checks: Number of collision detection calls

### 6.2 Parameter Optimization Results

Through iterative parameter tuning, the system was optimized to:
- **Iteration Capacity:** Increased from 1000 to 2000 maximum iterations
- **Search Efficiency:** Larger step size (8m) and search radius (20m)
- **Obstacle Complexity:** Reduced from 30 to 20 obstacles with smaller radii (3-6m vs 3-8m)
- **Goal Convergence:** Increased goal sample rate from 0.05 to 0.15

**Note:** Formal statistical testing with multiple trial runs would be required to establish empirical success rates. The current configuration represents optimized parameters based on algorithmic analysis and initial testing.

---

## 7. System Requirements

### 7.1 Software Dependencies

```
Python >= 3.7
airsim >= 1.8.0
numpy >= 1.19.0
scipy >= 1.5.0
matplotlib >= 3.3.0
psutil >= 5.7.0
```

### 7.2 Hardware Requirements

- **Minimum:** 8GB RAM, 4-core CPU, integrated graphics
- **Recommended:** 16GB RAM, 8-core CPU, dedicated GPU (for Unreal Engine)
- **Storage:** 50GB+ for Unreal Engine + AirSim assets

### 7.3 Environment Setup

- Unreal Engine 4.27.2
- AirSim Plugin (compiled for Linux)
- NED Coordinate System (X=North, Y=East, Z=Down)
- Simulation Map: Untitled (empty environment)

---

## 8. Experimental Results

### 8.1 Test Configuration

**Mission Parameters:**
- Start Position: Fixed at current drone location (typically [0, 0, -10])
- Goal Position: Random within 0-300m X, 0-300m Y, -25 to -15m Z
- Environment Boundary: (0, 300) × (0, 300) × (-35, -5) meters
- Flight Velocity: 5 m/s
- Path Smoothing: Enabled (α=0.3, 50 iterations)

### 8.2 Key Findings

**Path Finding Performance:**
- Preliminary testing shows path finding capability with optimized parameters
- Typical iteration count when path found: Variable (implementation allows up to 2000 iterations)
- Average path length: 80-150 meters (depending on goal distance and obstacle placement)
- Typical node count: 800-1200 nodes

**Collision Avoidance:**
- Drone safety radius (2m) successfully integrated into collision detection
- Paths maintain theoretical clearance from all obstacle surfaces
- Algorithm correctly identifies and avoids obstacles during planning phase

**Computational Efficiency:**
- Planning time scales with obstacle count and environment complexity
- 20 obstacles: Typically several seconds of planning time
- Performance varies based on random obstacle configuration and goal location

**Important Note:** The values reported here are based on initial implementation testing and algorithmic design. Comprehensive statistical evaluation with multiple trial runs across diverse scenarios would be necessary to establish robust performance metrics and success rate statistics.

---

## 9. Challenges & Solutions

### 9.1 Simulation Stability

**Challenge:** Unreal Engine crashes when spawning physics-enabled objects via AirSim API.

**Solution:** Implemented visual-only obstacle rendering using persistent plot markers. Obstacles treated identically by path planning algorithm but rendered as point clouds rather than physics meshes.

### 9.2 Path Finding Failures

**Challenge:** Initial configuration (30 obstacles, radius 3-8m, 1000 iterations) resulted in frequent path finding failures.

**Solution:** 
- Reduced obstacle count to 20
- Limited max radius to 6m
- Increased iterations to 2000
- Increased step size to 8m for faster exploration

### 9.3 Straight-Line Paths

**Challenge:** Random obstacle placement often left direct line-of-sight clear, resulting in trivial straight paths.

**Solution:** Implemented path-based obstacle placement strategy, forcing 40% of obstacles near the direct route to create realistic navigation challenges.

---

## 10. Conclusion

### 10.1 Achievements

This project successfully demonstrates:
1. ✓ Robust 3D RRT★ implementation with collision detection
2. ✓ Seamless AirSim integration for autonomous flight
3. ✓ Real-time obstacle visualization
4. ✓ Optimized parameters for improved path planning performance
5. ✓ Comprehensive performance monitoring

### 10.2 Applications

The developed system is applicable to:
- UAV delivery and logistics planning
- Search and rescue mission planning
- Agricultural drone navigation
- Infrastructure inspection path generation
- Autonomous aerial cinematography

### 10.3 Future Work

**Algorithm Enhancements:**
- Implement dynamic replanning for moving obstacles
- Add wind/weather considerations to path optimization
- Integrate terrain following for low-altitude flight
- Multi-drone coordination and collision avoidance

**Performance Optimization:**
- GPU acceleration for collision detection
- Parallel tree exploration
- Machine learning for obstacle prediction
- Adaptive parameter tuning based on environment density

**Real-World Deployment:**
- Hardware testing on physical UAV platforms
- Integration with vision-based obstacle detection
- GPS-denied navigation using SLAM
- Regulatory compliance (FAA Part 107)

---

## 11. References

### Academic Publications

1. Karaman, S., & Frazzoli, E. (2011). "Sampling-based algorithms for optimal motion planning." *The International Journal of Robotics Research*, 30(7), 846-894.

2. LaValle, S. M. (2006). *Planning Algorithms*. Cambridge University Press.

3. Kuffner, J. J., & LaValle, S. M. (2000). "RRT-connect: An efficient approach to single-query path planning." *IEEE International Conference on Robotics and Automation*.

### Software Documentation

4. Microsoft AirSim Documentation: https://microsoft.github.io/AirSim/
5. Unreal Engine 4 Documentation: https://docs.unrealengine.com/4.27/
6. Python NumPy/SciPy Scientific Computing Libraries

---

## Appendix A: Quick Start Guide

### Installation

```bash
# Clone repository
cd /home/belal/MyFiles/UNReal_Projects/MyProject2/rrt

# Install dependencies
pip install -r requirements.txt
```

### Execution

```bash
# 1. Start Unreal Engine with AirSim
cd ~/MyFiles/UNReal_Projects/MyProject2
~/MyFiles/UnrealEngine/Engine/Binaries/Linux/UE4Editor MyProject2.uproject

# 2. Press PLAY in Unreal Editor

# 3. Run path planning script
cd rrt
python3 airsim_rrt_star_drone.py
```

### Expected Output

```
Connecting to AirSim...
✓ Connected to AirSim successfully
Taking off Drone1...
Goal position (RANDOM): [234.56, 178.23, -18.45]
  Distance from start: 127.34 meters

[1/6] Generating random obstacles...
  Placing 8 obstacles along the path...
  Placing 12 random obstacles...
Generated 20 obstacles

[2/6] Spawning REAL obstacles in Unreal Engine...
✓ Drew 20 beautiful sphere obstacles (red)

[3/6] Planning path with RRT★...
Iteration 1247/2000
✓ Found path to goal!
Path length: 134.23 meters
Planning time: 3.89 seconds

[4/6] Smoothing path...
Path smoothed: 127 → 45 waypoints

[5/6] Visualizing path...
✓ Path visualization complete

[6/6] Executing path in AirSim...
Flying to waypoint 1/45: [0.00, 0.00, -10.00]
...
✓ Path execution completed successfully!

======================================================================
✓ MISSION COMPLETED SUCCESSFULLY!
======================================================================
```

---

## Appendix B: File Structure

```
rrt/
├── airsim_rrt_star_drone.py    # Main execution script (543 lines)
├── rrt_star_3d.py               # RRT★ algorithm (805 lines)
├── visualize_obstacles.py       # Visualization helpers
├── requirements.txt             # Python dependencies
├── README.md                    # Project overview
├── HOW_TO_RUN.md               # Detailed execution guide
├── COLLISION_DETECTION_FIX.md  # Safety margin documentation
└── TECHNICAL_REPORT.md         # This document
```

---

**Report Prepared By:** Autonomous Systems Development Team  
**Contact:** See project repository for details  
**Version:** 1.0  
**Last Updated:** November 12, 2025
