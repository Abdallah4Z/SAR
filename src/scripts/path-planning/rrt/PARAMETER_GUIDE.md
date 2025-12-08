# RRT★ Parameter Guide

## Complete Parameter Explanation

This document explains every parameter used in the RRT★ path planning system and how it affects the algorithm's behavior.

---

## Core RRT★ Algorithm Parameters

### 1. `step_size` (Default: 8 meters)

**What it does:**
- Controls the **maximum distance** the tree can grow in a single step
- When RRT★ generates a random point and finds the nearest node, it extends toward that point by exactly `step_size` meters (or less if the target is closer)

**Effect on performance:**
- **Larger values (e.g., 8-10m):**
  - ✅ Faster exploration of space
  - ✅ Fewer nodes needed to reach goal
  - ✅ Quicker planning time
  - ❌ Less precise paths (more jagged)
  - ❌ May miss narrow passages
  
- **Smaller values (e.g., 2-5m):**
  - ✅ More precise, smoother paths
  - ✅ Better at finding narrow passages
  - ❌ Slower exploration
  - ❌ More iterations needed
  - ❌ Longer planning time

**Current value:** `8 meters`
- Good balance for medium-to-large environments (0-300m range)
- Allows fast exploration while maintaining reasonable path quality

---

### 2. `goal_sample_rate` (Default: 0.15)

**What it does:**
- Probability (0.0 to 1.0) that the algorithm will **sample the goal** instead of a random point
- Every iteration, RRT★ decides: "Should I try to grow toward the goal (15% chance) or explore randomly (85% chance)?"

**Effect on performance:**
- **Higher values (e.g., 0.2-0.5):**
  - ✅ Finds goal faster (more direct attempts)
  - ✅ Fewer iterations needed
  - ❌ May get stuck in local minima
  - ❌ Less exploration of alternative routes
  - ❌ Lower path quality (not optimal)
  
- **Lower values (e.g., 0.01-0.1):**
  - ✅ Better exploration of entire space
  - ✅ Higher quality paths (more optimal)
  - ❌ Takes longer to find goal
  - ❌ More iterations needed

**Current value:** `0.15` (15%)
- Balances exploration vs exploitation
- Still explores 85% of the time, but has regular goal attempts
- Prevents excessive wandering while maintaining path quality

---

### 3. `max_iter` (Default: 1000 → Should be 2000)

**What it does:**
- **Maximum number of iterations** the algorithm will run before giving up
- Each iteration = one attempt to grow the tree (sample point → extend → rewire)

**Effect on performance:**
- **Higher values (e.g., 2000-5000):**
  - ✅ Higher success rate (more chances to find path)
  - ✅ Better path quality (more rewiring opportunities)
  - ❌ Longer planning time if path is hard to find
  
- **Lower values (e.g., 500-1000):**
  - ✅ Faster timeout if no path exists
  - ✅ Quicker results in simple scenarios
  - ❌ More failures in complex obstacle fields
  - ❌ May timeout before finding optimal path

**Current value:** `1000` (in your code) ⚠️
- **WARNING:** Comment says "Increased from 1000" but value is still 1000!
- **Recommended:** Change to `2000` for better success rate
- With 20 obstacles in 0-300m space, 2000 iterations provides sufficient search time

---

### 4. `search_radius` (Default: 20 meters)

**What it does:**
- Defines the **neighborhood radius** for rewiring operations
- After adding a new node, RRT★ looks for all nodes within this radius and checks if routing through the new node would give them a shorter path

**Effect on performance:**
- **Larger values (e.g., 20-30m):**
  - ✅ More rewiring opportunities
  - ✅ Better path optimization
  - ✅ Higher quality (more optimal) paths
  - ❌ Slower per-iteration (more neighbor checks)
  - ❌ Higher computational cost
  
- **Smaller values (e.g., 10-15m):**
  - ✅ Faster iterations
  - ✅ Lower computational cost
  - ❌ Less path optimization
  - ❌ May produce suboptimal paths

**Current value:** `20 meters`
- Allows significant rewiring in medium-density environments
- Good for obstacle fields with 3-6m radius obstacles
- Balances optimization quality with computation time

---

### 5. `drone_radius` (Default: 2.0 meters)

**What it does:**
- **Safety margin** added to every obstacle radius during collision detection
- Accounts for physical drone size + GPS uncertainty + safety buffer
- Formula: `effective_radius = obstacle_radius + drone_radius`

**Effect on safety:**
- **Larger values (e.g., 2.5-3m):**
  - ✅ Safer paths (more clearance)
  - ✅ Better for real-world deployment
  - ✅ Accounts for positioning errors
  - ❌ Harder to find paths (more space blocked)
  - ❌ May fail in tight spaces
  
- **Smaller values (e.g., 1-1.5m):**
  - ✅ Easier to find paths
  - ✅ Can navigate tighter spaces
  - ❌ Less safe (minimal clearance)
  - ❌ Risk of collision with positioning errors

**Current value:** `2.0 meters`
- Breakdown:
  - ~1.0m: Physical drone body dimensions
  - ~0.5m: GPS/positioning uncertainty
  - ~0.5m: Safety buffer
- Good balance for simulation and real-world deployment

---

## Environment Parameters

### 6. `obstacle_count` (Default: 20)

**What it does:**
- **Number of random obstacles** generated in the environment
- Each obstacle is a sphere with random position and radius

**Effect on difficulty:**
- **More obstacles (e.g., 30-50):**
  - ✅ More realistic/challenging scenarios
  - ✅ Tests algorithm robustness
  - ❌ Much higher failure rate
  - ❌ Longer planning time
  - ❌ May completely block some paths
  
- **Fewer obstacles (e.g., 10-15):**
  - ✅ Higher success rate
  - ✅ Faster planning
  - ✅ Easier path finding
  - ❌ Less challenging scenarios
  - ❌ May have trivial straight-line solutions

**Current value:** `20 obstacles`
- Good for demonstrations (challenging but solvable)
- With current parameters, allows ~85-95% success rate
- Reduced from 30 to improve reliability

---

### 7. `obstacle_radius_range` (Default: (3, 6) meters)

**What it does:**
- Range for **random obstacle sizes**
- Each obstacle gets a radius randomly chosen between min and max
- Tuple format: `(minimum_radius, maximum_radius)`

**Effect on difficulty:**
- **Larger obstacles (e.g., 5-10m):**
  - ✅ More visible/realistic
  - ❌ Block more space
  - ❌ Harder to find paths
  - ❌ Higher failure rate
  
- **Smaller obstacles (e.g., 2-4m):**
  - ✅ Easier path finding
  - ✅ More navigable space
  - ❌ May be too easy
  - ❌ Less realistic challenges

**Current value:** `(3, 6)` meters
- Minimum 3m: Ensures obstacles are visible
- Maximum 6m: Reduced from 8m to improve success rate
- With 2m drone radius, effective sizes are 5-8m

---

### 8. `boundary` (Default: ((0, 300), (0, 300), (-35, -5)))

**What it does:**
- Defines the **3D space boundaries** for random sampling and obstacle placement
- Format: `((x_min, x_max), (y_min, y_max), (z_min, z_max))`

**Components:**
- **X boundary (0, 300):** North-South extent (300 meters)
- **Y boundary (0, 300):** East-West extent (300 meters)  
- **Z boundary (-35, -5):** Altitude range (30 meters vertical space)
  - Note: NED coordinates (Z negative is UP)
  - -5m = 5 meters altitude
  - -35m = 35 meters altitude

**Effect on planning:**
- **Larger boundaries:**
  - ✅ More exploration space
  - ✅ Can accommodate longer paths
  - ❌ More iterations needed
  - ❌ Longer planning time
  
- **Smaller boundaries:**
  - ✅ Faster planning
  - ✅ More constrained exploration
  - ❌ May not fit entire mission
  - ❌ Less realistic for long-distance flights

**Current value:** `((0, 300), (0, 300), (-35, -5))`
- 300m × 300m × 30m volume
- Suitable for medium-range UAV missions
- Positive X,Y for easier coordinate understanding

---

## Execution Parameters

### 9. `velocity` (Default: 5.0 m/s)

**What it does:**
- Flight speed when executing the planned path
- Controls how fast the drone moves between waypoints

**Effect on execution:**
- **Faster (e.g., 10-15 m/s):**
  - ✅ Quicker mission completion
  - ✅ More realistic for racing/delivery
  - ❌ Less time to react to errors
  - ❌ Harder to follow tight curves
  - ❌ More overshoot at waypoints
  
- **Slower (e.g., 2-5 m/s):**
  - ✅ Smoother path following
  - ✅ Better accuracy
  - ✅ Safer for testing
  - ❌ Longer mission time
  - ❌ Less realistic for practical use

**Current value:** `5.0 m/s`
- Conservative safe speed
- Good for simulation testing and visualization
- Allows smooth path following with minimal overshoot

---

### 10. `smooth_path_flag` (Default: True)

**What it does:**
- Enables/disables **path smoothing** after RRT★ planning
- Smoothing reduces waypoints and creates more natural curves

**Effect on path quality:**
- **Enabled (True):**
  - ✅ Smoother, more natural paths
  - ✅ Fewer waypoints (easier to follow)
  - ✅ Removes unnecessary zigzags
  - ❌ Small additional computation time
  - ❌ May slightly increase path length
  
- **Disabled (False):**
  - ✅ Faster (no post-processing)
  - ✅ Preserves exact RRT★ output
  - ❌ Jagged, unnatural paths
  - ❌ Many unnecessary waypoints
  - ❌ Harder for drone to follow

**Current value:** `True`
- Recommended to keep enabled
- Improves path aesthetics and followability
- Uses alpha=0.3, 50 iterations (good defaults)

---

### 11. `visualize` (Default: True)

**What it does:**
- Controls whether to **visualize the path** in AirSim and save plots
- Includes both 3D matplotlib plots and green line rendering in simulator

**Effect on operation:**
- **Enabled (True):**
  - ✅ See the planned path in real-time
  - ✅ Saves visualization image (rrt_star_path_3d.png)
  - ✅ Green path line in AirSim simulator
  - ❌ Slight performance overhead
  
- **Disabled (False):**
  - ✅ Faster execution (no rendering)
  - ✅ Useful for batch testing
  - ❌ No visual feedback
  - ❌ Harder to debug issues

**Current value:** `True`
- Essential for demonstrations and debugging
- Minimal performance impact
- Helps verify path quality before execution

---

## Position Parameters

### 12. `start_pos` (Dynamic - current drone position)

**What it does:**
- **Starting position** for path planning [x, y, z]
- Typically the current drone position after takeoff
- Fixed for each planning run (doesn't change)

**Current behavior:**
- Gets actual drone position: `controller.get_drone_position()`
- Fallback default: `[0, 0, -10]` if position unavailable
- Stays fixed while goal is randomized

---

### 13. `goal_pos` (Dynamic - random each run)

**What it does:**
- **Target destination** for the drone [x, y, z]
- Randomized each run for varied scenarios

**Current generation:**
```python
goal_x = np.random.uniform(0, 300)    # Random X: 0-300m
goal_y = np.random.uniform(0, 300)    # Random Y: 0-300m  
goal_z = np.random.uniform(-25, -15)  # Random Z: 15-25m altitude
```

**Range:**
- Horizontal: Anywhere in 300m × 300m space
- Vertical: Between 15-25 meters altitude
- Ensures goal is within boundary

---

## Obstacle Generation Parameters

### 14. `min_clearance` (Default: 10.0 meters)

**What it does:**
- **Minimum distance** obstacles must maintain from start and goal positions
- Ensures start and goal are not blocked

**Effect on planning:**
- **Larger clearance (e.g., 15-20m):**
  - ✅ Guarantees obstacle-free zones at endpoints
  - ✅ Higher success rate
  - ❌ Less challenging scenarios
  - ❌ Unrealistic (real world may have obstacles near start/goal)
  
- **Smaller clearance (e.g., 5-8m):**
  - ✅ More realistic scenarios
  - ✅ Tests local planning capability
  - ❌ May block start/goal access
  - ❌ Higher failure rate

**Current value:** `10.0 meters`
- With obstacles sized 3-6m and drone radius 2m, this ensures:
  - Minimum 10m from obstacle center to start/goal
  - Effective clearance: 10m - 6m - 2m = 2m minimum
- Balances realism with reliability

---

## Parameter Interactions

### Critical Relationships:

1. **`step_size` vs `search_radius`:**
   - search_radius should be ≥ 2 × step_size
   - Current: 20m radius, 8m step ✓ (ratio = 2.5)
   - Allows effective rewiring

2. **`obstacle_radius` + `drone_radius` = effective blocking:**
   - Max obstacle: 6m
   - Drone safety: 2m
   - Total: 8m effective radius
   - Influences `search_radius` needed

3. **`max_iter` vs environment complexity:**
   - Formula (rough): `max_iter ≈ (boundary_volume / step_size³) × obstacle_density`
   - Current: 2000 iterations for 300×300×30m space = sufficient
   - More obstacles → need more iterations

4. **`goal_sample_rate` vs `max_iter`:**
   - Higher sample rate → fewer iterations needed
   - Lower sample rate → need more iterations for optimal paths
   - Current: 0.15 rate with 2000 iter = good balance

---

## Recommended Tuning Guide

### For Higher Success Rate:
```python
max_iter=3000              # More search time
obstacle_count=15          # Fewer obstacles
obstacle_radius_range=(3, 5)  # Smaller obstacles
step_size=10               # Faster exploration
```

### For Better Path Quality:
```python
step_size=5                # More precise
search_radius=25           # More rewiring
goal_sample_rate=0.10      # More exploration
smooth_path_flag=True      # Always smooth
```

### For Challenging Scenarios:
```python
obstacle_count=30          # More obstacles
obstacle_radius_range=(4, 8)   # Larger obstacles
min_clearance=5            # Less safe zones
drone_radius=1.5           # Tighter constraints
```

### For Fast Planning:
```python
max_iter=1000              # Quick timeout
step_size=12               # Large steps
search_radius=15           # Less rewiring
obstacle_count=10          # Fewer obstacles
```

---

## Current Configuration Summary

```python
step_size = 8              # Balanced exploration
goal_sample_rate = 0.15    # 15% goal bias
max_iter = 1000            # ⚠️ SHOULD BE 2000
search_radius = 20         # Good rewiring radius
drone_radius = 2.0         # Safe margin
obstacle_count = 20        # Moderate difficulty
obstacle_radius_range = (3, 6)  # Medium-sized obstacles
boundary = ((0,300), (0,300), (-35,-5))  # 300×300×30m space
velocity = 5.0             # Safe flight speed
smooth_path_flag = True    # Enabled
visualize = True           # Enabled
min_clearance = 10.0       # Obstacle-free zones
```

### Overall Assessment:
✅ Well-balanced for demonstrations  
✅ Good success/quality tradeoff  
⚠️ Increase `max_iter` to 2000 for better reliability  
✅ Safe for simulation testing  
✅ Ready for real-world parameter adaptation  

---

**Last Updated:** November 12, 2025  
**Configuration Version:** Optimized v1.0
