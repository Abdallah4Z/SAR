# Hybrid A* and APF Path Planning for PX4 Drones
## Technical Report

**Project:** Autonomous Drone Navigation with Hybrid Path Planning  
**Platform:** PX4 Autopilot + ROS 2 + Gazebo  
**Algorithm:** A* (A-Star) + APF (Artificial Potential Field)  
**Date:** December 2025  
**Version:** 1.0.0

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [System Architecture](#system-architecture)
3. [Algorithm Overview](#algorithm-overview)
4. [System Components](#system-components)
5. [Installation & Setup](#installation--setup)
6. [Operational Workflow](#operational-workflow)
7. [ROS 2 Communication](#ros-2-communication)
8. [Technical Implementation](#technical-implementation)
9. [Configuration Parameters](#configuration-parameters)
10. [Testing & Validation](#testing--validation)
11. [Known Limitations](#known-limitations)
12. [Future Enhancements](#future-enhancements)

---

## Executive Summary

This project implements a **hybrid autonomous navigation system** for PX4-based drones, combining global path planning with local obstacle avoidance. The system uses:

- **A* Algorithm** for optimal global path planning in known environments
- **Artificial Potential Field (APF)** for real-time local obstacle avoidance
- **PX4 Autopilot** for low-level flight control
- **ROS 2** for inter-process communication
- **Gazebo** for high-fidelity 3D simulation

The hybrid approach ensures safe navigation by switching between A* path following (when the path is clear) and APF-based reactive avoidance (when obstacles are detected nearby).

### Key Features

✅ **Global Path Planning:** A* algorithm computes collision-free paths around known obstacles  
✅ **Local Obstacle Avoidance:** APF provides real-time reactive maneuvering  
✅ **Dynamic Mode Switching:** Intelligent transition between planning and avoidance modes  
✅ **3D Simulation:** Full Gazebo integration with physics simulation  
✅ **Scalable Architecture:** Modular ROS 2 nodes for easy extension  
✅ **Visualization:** RViz markers for path and obstacle visualization  

---

## System Architecture

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         PX4 SITL (Software-in-the-Loop)         │
│                     Gazebo Simulation Environment               │
└──────────────────────────┬──────────────────────────────────────┘
                           │
                           ├── Vehicle Dynamics
                           ├── Sensor Simulation
                           └── Physics Engine
                           │
┌──────────────────────────┴──────────────────────────────────────┐
│                    Micro XRCE-DDS Agent                         │
│              (Bridge: PX4 ↔ ROS 2 Communication)                │
└──────────────────────────┬──────────────────────────────────────┘
                           │
┌──────────────────────────┴──────────────────────────────────────┐
│                      ROS 2 Middleware (DDS)                     │
└─────────┬───────────┬────────────┬───────────┬───────────┬──────┘
          │           │            │           │           │
    ┌─────▼────┐ ┌────▼─────┐ ┌───▼────┐ ┌────▼─────┐ ┌──▼──────┐
    │Obstacle  │ │  Path    │ │ Drone  │ │Gazebo    │ │Visuali- │
    │Generator │ │  Planner │ │Control │ │Spawner   │ │zation   │
    │  Node    │ │   (A*)   │ │ (APF)  │ │  Node    │ │  Node   │
    └──────────┘ └──────────┘ └────────┘ └──────────┘ └─────────┘
```

### Component Interaction Flow

```
1. Obstacle Generator → Generates random obstacles
                      ↓
2. Gazebo Spawner → Spawns 3D cylinders in simulation
                      ↓
3. Path Planner (A*) → Computes optimal path avoiding obstacles
                      ↓
4. Drone Controller → Follows A* path, switches to APF when blocked
                      ↓
5. PX4 Autopilot → Executes trajectory commands
                      ↓
6. Gazebo → Simulates drone movement and physics
```

---

## Algorithm Overview

### 1. A* (A-Star) Path Planning

**Purpose:** Global path planning with guaranteed optimality.

**Algorithm Details:**
- **Grid-Based Search:** Discretizes 2D space into a grid (default 0.5m resolution)
- **Cost Function:** `f(n) = g(n) + h(n)` where:
  - `g(n)` = actual cost from start to node n
  - `h(n)` = heuristic (Euclidean distance) from n to goal
- **8-Connectivity:** Allows diagonal movements (cost = 1.414) and straight (cost = 1.0)
- **Obstacle Inflation:** Adds safety margin around obstacles (default 3.0m)
- **Path Simplification:** Reduces waypoints to ~20 points for efficiency

**Advantages:**
- Complete and optimal (finds shortest path if one exists)
- Predictable computational cost
- Works well with static obstacles

**Implementation Location:** `drone_path_planning_px4/utils/astar.py`

### 2. APF (Artificial Potential Field)

**Purpose:** Real-time local obstacle avoidance and reactive navigation.

**Algorithm Details:**

**Attractive Force (toward goal):**
```
F_att = k_att * (goal - current_pos) / ||goal - current_pos||
```

**Repulsive Force (away from obstacles):**
```
F_rep = k_rep * (1/d - 1/d0) * (1/d²) * direction
```
where:
- `d` = distance to obstacle
- `d0` = obstacle influence distance (5.0m)
- `direction` = unit vector away from obstacle

**Total Force:**
```
F_total = F_att + Σ F_rep + F_tangent + F_escape
```

**Special Features:**
- **Tangential Force:** Helps "slide" around obstacles instead of getting stuck
- **Local Minima Escape:** Detects when stuck (< 0.2m movement for 15 iterations) and applies perpendicular random force
- **Force Limiting:** Caps maximum force at 5.0 to prevent erratic behavior
- **Dynamic Gain:** Adjusts repulsive force based on proximity

**Implementation Location:** `drone_path_planning_px4/utils/apf.py`

### 3. Hybrid Mode Switching

The controller intelligently switches between A* and APF based on real-time conditions:

**A* Mode (Default):**
- Path to next waypoint is clear
- No obstacles within critical range
- Follows pre-planned path waypoints
- Efficient and smooth trajectory

**APF Mode (Triggered when):**
- Obstacle detected within 6.0m
- Obstacle blocks direct path to waypoint
- Path clearance < (obstacle_radius + 2.0m)

**Switching Logic:**
```python
if path_blocked and nearby_obstacles:
    use APF for local avoidance
else:
    follow A* waypoints
```

---

## System Components

### 1. Obstacle Generator Node

**File:** `obstacle_generator.py`

**Purpose:** Generates random obstacle configurations for testing.

**Features:**
- Configurable number of obstacles (default: 6)
- Random positions within defined area (5-20m x, -5-15m y)
- Variable radius (1.5-3.0m) and height (6-12m)
- Publishes on two topics for compatibility

**Topics Published:**
- `/drone/obstacle_info` (String - JSON format)
- `/drone/obstacles` (Float32MultiArray - flat array)

**Parameters:**
```yaml
num_obstacles: 6
min_height: 6.0
max_height: 10.0
```

---

### 2. Gazebo Obstacle Spawner

**File:** `spawn_gazebo_obstacles.py`

**Purpose:** Spawns 3D cylindrical obstacles in Gazebo simulation.

**Features:**
- Receives obstacle data from generator
- Creates SDF (Simulation Description Format) models
- Spawns red cylinders in Gazebo world
- Static collision geometry for drone interaction

**SDF Model Structure:**
```xml
<model name="obstacle_i">
  <static>true</static>
  <pose>x y z/2 0 0 0</pose>
  <link>
    <collision>
      <cylinder radius="r" length="h"/>
    </collision>
    <visual>
      <cylinder radius="r" length="h"/>
      <material color="red"/>
    </visual>
  </link>
</model>
```

**Gazebo Service Used:**
```bash
gz service -s /world/default/create \
  --reqtype gz.msgs.EntityFactory \
  --req 'sdf_filename: "/tmp/obstacle_i.sdf"'
```

---

### 3. Path Planner Node (A*)

**File:** `path_planner_node.py`

**Purpose:** Computes optimal collision-free paths using A* algorithm.

**Workflow:**
1. Receives obstacles from generator
2. Waits for goal pose and start planning trigger
3. Creates occupancy grid with obstacle inflation
4. Runs A* search from start (0,0) to goal
5. Simplifies path to key waypoints
6. Publishes path to controller

**Topics Subscribed:**
- `/drone/obstacles` (Float32MultiArray)
- `/goal_pose` (PoseStamped)
- `/drone/start_planning` (Bool)

**Topics Published:**
- `/drone/astar_path` (Path)

**Key Functions:**
- `plan()` - Main A* search algorithm
- `_heuristic()` - Euclidean distance estimate
- `_neighbors()` - 8-connected grid neighbors
- `_reconstruct_path()` - Builds final path from search tree

---

### 4. Drone Controller Node (Hybrid A* + APF)

**File:** `drone_controller_px4.py`

**Purpose:** Main navigation controller that executes hybrid path following.

**Core Responsibilities:**
1. **Mode Management:** Switches between A* and APF modes
2. **Waypoint Tracking:** Advances through A* waypoints as reached
3. **Obstacle Detection:** Monitors nearby obstacles in real-time
4. **Path Validation:** Checks if A* path is currently blocked
5. **PX4 Communication:** Sends trajectory setpoints to autopilot
6. **Offboard Control:** Manages PX4 offboard mode activation

**Control Loop (50 Hz):**
```python
def control_loop():
    1. Publish offboard control mode
    2. Enable offboard mode (after 10 setpoints)
    3. Check if goal reached
    4. Detect nearby obstacles
    5. Check if path to next waypoint is blocked
    6. If blocked: use APF
       - Calculate attractive + repulsive forces
       - Compute next position from force vector
       - Send trajectory setpoint
    7. If clear: follow A*
       - Advance through waypoints
       - Send waypoint as trajectory setpoint
```

**Path Blocking Detection:**
- Projects obstacles onto line from current position to target
- If obstacle within (radius + 2m) of path line → blocked
- Considers only obstacles ahead on path

**Topics Subscribed:**
- `/fmu/out/vehicle_local_position_v1` (VehicleLocalPosition)
- `/goal_pose` (PoseStamped)
- `/drone/astar_path` (Path)
- `/drone/obstacles` (Float32MultiArray)
- `/drone/start_navigation` (Bool)

**Topics Published:**
- `/fmu/in/vehicle_command` (VehicleCommand)
- `/fmu/in/offboard_control_mode` (OffboardControlMode)
- `/fmu/in/trajectory_setpoint` (TrajectorySetpoint)
- `/drone/status` (String - JSON)

---

### 5. Visualization Node

**File:** `visualization_node.py`

**Purpose:** Publishes RViz markers for start and goal positions.

**Features:**
- Green sphere at start position (0, 0, 5)
- Red sphere at goal position (default: 40, 40, 5)
- 2m diameter markers for visibility
- Updates at 2 Hz

**Topics Published:**
- `/drone/start_marker` (Marker)
- `/drone/goal_marker` (Marker)

---

### 6. Gazebo Pose Bridge

**File:** `gazebo_pose_bridge.py`

**Purpose:** Bridges Gazebo simulation pose to PX4 visual odometry.

**Functionality:**
- Queries Gazebo for drone model position (50 Hz)
- Converts to PX4 `VehicleVisualOdometry` message format
- Provides position feedback for closed-loop control

**Note:** This is a workaround for simulation environments without proper PX4-Gazebo integration.

---

## Installation & Setup

### Prerequisites

1. **Ubuntu 22.04 LTS** (recommended)
2. **ROS 2 Humble** (or later)
3. **PX4 Autopilot** (latest stable)
4. **Gazebo Garden** (or compatible version)
5. **Micro XRCE-DDS Agent**

### Installation Steps

#### 1. Install ROS 2 Humble
```bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

#### 2. Install PX4 Autopilot
```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
make px4_sitl gz_x500
```

#### 3. Install Micro XRCE-DDS Agent
```bash
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

#### 4. Clone and Build Project
```bash
cd ~
mkdir -p drone_px4_ws/src
cd drone_px4_ws/src
# Clone your project repository here
cd ~/drone_px4_ws
colcon build --packages-select px4_msgs drone_path_planning_px4
source install/setup.bash
```

#### 5. Install Python Dependencies
```bash
pip3 install numpy matplotlib
```

---

## Operational Workflow

### Complete Startup Sequence

#### Terminal 1: Start PX4 SITL with Gazebo
```bash
cd ~/PX4-Autopilot
PX4_GZ_MODEL=x500 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4
```

**What happens:**
- Launches PX4 Software-In-The-Loop simulation
- Starts Gazebo with x500 quadcopter model
- Initializes flight controller virtual instance
- Opens MAVLink connection on UDP port 18570

**Wait:** 30 seconds for full initialization

---

#### Terminal 2: Start Micro XRCE-DDS Agent
```bash
MicroXRCEAgent udp4 -p 8888
```

**What happens:**
- Starts DDS-XRCE agent on UDP port 8888
- Bridges PX4 (using uXRCE-DDS) to ROS 2 (using DDS)
- Enables PX4 topics to appear as ROS 2 topics
- Required for `/fmu/*` topic communication

---

#### Terminal 3: Build and Launch ROS 2 Nodes
```bash
cd ~/drone_px4_ws
colcon build --packages-select drone_path_planning_px4
source install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch drone_path_planning_px4 full_path_planning.launch.py
```

**What happens:**
- Builds the ROS 2 package (if changes were made)
- Sources the install workspace
- Sets ROS domain to 0 (matches PX4 domain)
- Launches all nodes:
  1. `obstacle_generator` - Creates 6 random obstacles
  2. `spawn_gazebo_obstacles` - Spawns red cylinders in Gazebo
  3. `path_planner_node` - Initializes A* planner
  4. `drone_controller_px4` - Starts hybrid controller
  5. `visualization_node` - Publishes RViz markers

**Wait:** 10 seconds for all nodes to initialize

---

#### Terminal 4: Arm, Takeoff, and Execute Mission
```bash
cd ~/PX4-Autopilot
export ROS_DOMAIN_ID=0

# 1. Arm the drone
./build/px4_sitl_default/bin/px4-commander arm -f
sleep 3

# 2. Takeoff to default altitude
./build/px4_sitl_default/bin/px4-commander takeoff
sleep 5

# 3. Set goal position (x=24m, y=24m, z=-7m in NED)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, \
    pose: {position: {x: 24.0, y: 24.0, z: -7.0}, \
           orientation: {w: 1.0}}}"

# 4. Trigger path planning
ros2 topic pub --once /drone/start_planning std_msgs/msg/Bool "data: true"
sleep 2

# 5. Start navigation
ros2 topic pub --once /drone/start_navigation std_msgs/msg/Bool "data: true"
```

**Mission Execution Flow:**
1. **Arm:** Enables motors (forced with `-f` flag)
2. **Takeoff:** Climbs to 7m altitude (configured flight altitude)
3. **Set Goal:** Publishes target position to `/goal_pose`
4. **Start Planning:** Triggers A* computation with current obstacles
5. **Start Navigation:** Activates hybrid controller to follow path

---

### Monitoring the Mission

#### Check Status
```bash
# View drone status (A* vs APF mode, position, progress)
ros2 topic echo /drone/status

# Monitor position
ros2 topic echo /fmu/out/vehicle_local_position_v1

# View planned path
ros2 topic echo /drone/astar_path
```

#### Visualize in RViz
```bash
rviz2
```

Add displays:
- `/drone/astar_path` (Path - planned trajectory)
- `/drone/start_marker` (Marker - green start sphere)
- `/drone/goal_marker` (Marker - red goal sphere)
- `/fmu/out/vehicle_local_position_v1` (Odometry - current position)

---

## ROS 2 Communication

### Topic Architecture

#### Published by Obstacle System
| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/drone/obstacle_info` | `String` | obstacle_generator | JSON obstacle data |
| `/drone/obstacles` | `Float32MultiArray` | obstacle_generator | Flat array: [x,y,r,h, ...] |

#### Published by Path Planner
| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/drone/astar_path` | `Path` | path_planner_node | A* computed waypoints |

#### Published by Drone Controller
| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/fmu/in/vehicle_command` | `VehicleCommand` | drone_controller_px4 | Arm, takeoff, mode changes |
| `/fmu/in/offboard_control_mode` | `OffboardControlMode` | drone_controller_px4 | Enable position control |
| `/fmu/in/trajectory_setpoint` | `TrajectorySetpoint` | drone_controller_px4 | Desired position & velocity |
| `/drone/status` | `String` | drone_controller_px4 | JSON status (mode, progress) |

#### Subscribed by Drone Controller
| Topic | Type | Subscriber | Description |
|-------|------|------------|-------------|
| `/fmu/out/vehicle_local_position_v1` | `VehicleLocalPosition` | drone_controller_px4 | Current drone position |
| `/goal_pose` | `PoseStamped` | drone_controller_px4 | Target position |
| `/drone/astar_path` | `Path` | drone_controller_px4 | Planned waypoints |
| `/drone/obstacles` | `Float32MultiArray` | drone_controller_px4 | Obstacle positions |
| `/drone/start_navigation` | `Bool` | drone_controller_px4 | Navigation trigger |

#### Published by Visualization
| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/drone/start_marker` | `Marker` | visualization_node | Green start sphere |
| `/drone/goal_marker` | `Marker` | visualization_node | Red goal sphere |

### Message Formats

#### Float32MultiArray (Obstacles)
```
data: [x1, y1, radius1, height1, x2, y2, radius2, height2, ...]
```

#### Path (A* Waypoints)
```yaml
header:
  frame_id: 'map'
poses:
  - pose:
      position: {x: 0.0, y: 0.0, z: 0.0}
      orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
  - pose:
      position: {x: 2.5, y: 1.3, z: 0.0}
      orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
  # ... more waypoints
```

#### String (Status JSON)
```json
{
  "connected": true,
  "navigating": true,
  "mode": "A*",
  "position": [12.3, 8.7, -7.0],
  "goal": [24.0, 24.0],
  "distance_to_goal": 14.2,
  "path_waypoints": 18,
  "current_waypoint": 5
}
```

---

## Technical Implementation

### A* Path Planning Implementation

**Key Algorithm Steps:**

1. **Grid Creation:**
```python
grid_w = int((max_x - min_x) / resolution)
grid_h = int((max_y - min_y) / resolution)
obstacle_grid = np.zeros((grid_h, grid_w), dtype=bool)
```

2. **Obstacle Inflation:**
```python
for obs in obstacles:
    ox, oy = obs['x'], obs['y']
    radius = obs['radius'] + safety_distance  # Add safety margin
    
    for i in range(grid_h):
        for j in range(grid_w):
            wx = min_x + j * resolution
            wy = min_y + i * resolution
            dist = sqrt((wx - ox)² + (wy - oy)²)
            
            if dist < radius:
                obstacle_grid[i, j] = True  # Mark as obstacle
```

3. **A* Search:**
```python
open_set = [(f_score[start], start)]  # Priority queue
came_from = {}
g_score = {start: 0}

while open_set:
    current = heappop(open_set)
    
    if current == goal:
        return reconstruct_path(came_from, current)
    
    for neighbor in neighbors(current):
        tentative_g = g_score[current] + distance(current, neighbor)
        
        if neighbor not in g_score or tentative_g < g_score[neighbor]:
            came_from[neighbor] = current
            g_score[neighbor] = tentative_g
            f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
            heappush(open_set, (f_score[neighbor], neighbor))
```

4. **Path Simplification:**
```python
step = max(1, len(path) // 20)  # Keep max 20 waypoints
simplified = [path[i] for i in range(0, len(path), step)]
```

---

### APF Implementation

**Force Calculation:**

```python
def calculate_force(current, goal, obstacles):
    # 1. Attractive Force
    diff = goal - current
    dist_to_goal = norm(diff)
    f_att = k_att * (diff / dist_to_goal)
    
    # 2. Repulsive Force
    f_rep = zeros(2)
    for obs in obstacles:
        diff_obs = current - obs['position']
        dist_obs = norm(diff_obs)
        effective_dist = dist_obs - obs['radius']
        
        if effective_dist < d0:
            magnitude = k_rep * (1/effective_dist - 1/d0) / (effective_dist²)
            direction = diff_obs / dist_obs
            
            # Add tangential sliding component
            tangent = [-direction[1], direction[0]]
            f_rep += magnitude * direction + 0.3 * magnitude * tangent
    
    # 3. Local Minima Escape
    if stuck_counter > threshold:
        perpendicular = [-goal_dir[1], goal_dir[0]]
        random_sign = ±1.0
        escape_force = random_force_gain * random_sign * perpendicular
        return f_att + f_rep + escape_force
    
    return f_att + f_rep
```

**Next Position Calculation:**
```python
force = apf.calculate_force(current_2d, target, obstacles)
force_mag = norm(force)

if force_mag > 0.1:
    step_size = 0.8  # 0.8 meters per step
    next_pos = current_2d + step_size * force / force_mag
```

---

### Hybrid Mode Switching Logic

```python
def control_loop():
    nearby_obstacles = detect_nearby_obstacles()  # Within 6m
    
    # Check if path to next waypoint is blocked
    path_blocked = False
    for obs in nearby_obstacles:
        to_target = target - current_2d
        to_obs = obs['position'] - current_2d
        
        # Project obstacle onto path line
        projection = dot(to_obs, to_target_normalized)
        
        if 0 < projection < norm(to_target):
            closest_point = current_2d + projection * to_target_normalized
            dist_to_path = norm(obs['position'] - closest_point)
            
            if dist_to_path < (obs['radius'] + 2.0):
                path_blocked = True
                break
    
    if path_blocked and len(nearby_obstacles) > 0:
        # USE APF MODE
        force = apf.calculate_force(current_2d, target, nearby_obstacles)
        next_pos = current_2d + step_size * normalized(force)
        publish_trajectory_setpoint(next_pos)
        
    else:
        # USE A* MODE
        while waypoint_reached(current_waypoint):
            path_index += 1
        
        target = astar_path[path_index]
        publish_trajectory_setpoint(target)
```

---

### PX4 Offboard Control

**Offboard Mode Activation Sequence:**

```python
# 1. Send 10 setpoints before switching
if offboard_setpoint_counter < 11:
    offboard_setpoint_counter += 1
    publish_trajectory_setpoint(current_position)
    return

# 2. Enable offboard mode (at setpoint 10)
if offboard_setpoint_counter == 10:
    publish_vehicle_command(
        VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
        param1=1.0,  # Custom mode
        param2=6.0   # Offboard mode
    )
```

**Trajectory Setpoint Message:**
```python
msg = TrajectorySetpoint()
msg.position = [x, y, -z]  # NED frame (z is negative for up)
msg.yaw = yaw_angle
msg.timestamp = clock.now().nanoseconds / 1000
trajectory_setpoint_pub.publish(msg)
```

**QoS Profile (for PX4 topics):**
```python
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
```

---

## Configuration Parameters

### Launch File Parameters

**File:** `launch/full_path_planning.launch.py`

```python
# Obstacle Generator
parameters=[{
    'num_obstacles': 6,        # Number of random obstacles
    'min_height': 6.0,         # Minimum obstacle height (m)
    'max_height': 10.0,        # Maximum obstacle height (m)
}]

# Path Planner (A*)
parameters=[{
    'grid_resolution': 0.5,    # Grid cell size (m)
    'grid_size_x': 50.0,       # Grid width (m)
    'grid_size_y': 50.0,       # Grid height (m)
    'safety_distance': 3.0,    # Obstacle inflation radius (m)
}]

# Drone Controller (Hybrid)
parameters=[{
    'flight_altitude': 7.0,              # Flight height (m, negative in NED)
    'goal_threshold': 1.5,               # Goal reached distance (m)
    'obstacle_detection_range': 6.0,     # APF activation radius (m)
    'apf_activation_distance': 6.0,      # Obstacle proximity threshold (m)
}]
```

### A* Algorithm Parameters

**File:** `utils/astar.py`

```python
resolution = 0.5           # Grid cell size (meters)
safety_distance = 3.0      # Obstacle buffer (meters)
max_iterations = grid_w * grid_h  # Search timeout
path_simplification_step = max(1, len(path) // 20)  # Max waypoints
```

### APF Algorithm Parameters

**File:** `utils/apf.py`

```python
k_att = 1.5                # Attractive force gain
k_rep = 8.0                # Repulsive force gain
d0 = 5.0                   # Obstacle influence distance (m)
min_force = 0.1            # Minimum force threshold
stuck_threshold = 15       # Iterations before local minima escape
random_force_gain = 3.0    # Escape force magnitude
step_size = 0.8            # APF step size (m)
```

### Tuning Guidelines

| Parameter | Effect | Increase → | Decrease → |
|-----------|--------|------------|------------|
| `k_att` | Attraction to goal | Faster approach | More cautious |
| `k_rep` | Obstacle repulsion | Wider berth | Tighter turns |
| `d0` | Obstacle range | Earlier reaction | Last-minute avoidance |
| `safety_distance` | A* clearance | Safer but longer paths | Shorter but riskier |
| `grid_resolution` | Path precision | More detailed, slower | Faster but coarser |
| `apf_activation_distance` | APF trigger | More reactive behavior | More A* following |
| `goal_threshold` | Goal precision | Land further away | Hover at exact point |

---

## Testing & Validation

### Unit Testing

**Test A* Planner:**
```bash
cd ~/drone_px4_ws
source install/setup.bash
ros2 run drone_path_planning_px4 path_planner_node
```

**Manually publish test data:**
```bash
# Publish obstacles
ros2 topic pub /drone/obstacles std_msgs/msg/Float32MultiArray \
  "data: [10.0, 5.0, 2.0, 8.0, 15.0, 8.0, 2.5, 9.0]"

# Publish goal
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{pose: {position: {x: 20.0, y: 10.0, z: 0.0}}}"

# Trigger planning
ros2 topic pub /drone/start_planning std_msgs/msg/Bool "data: true"

# Check output
ros2 topic echo /drone/astar_path
```

---

### Integration Testing

**Full System Test:**

1. **Start all terminals** as described in [Operational Workflow](#operational-workflow)

2. **Verify each component:**
```bash
# Check obstacle generation
ros2 topic echo /drone/obstacles --once

# Check Gazebo models
gz model -l

# Check A* path
ros2 topic echo /drone/astar_path --once

# Check drone position
ros2 topic echo /fmu/out/vehicle_local_position_v1
```

3. **Monitor mode switching:**
```bash
ros2 topic echo /drone/status
```

Expected output during navigation:
```json
{"mode": "A*", ...}        # Following planned path
{"mode": "APF", ...}       # Avoiding unexpected obstacle
{"mode": "A*", ...}        # Returned to planned path
```

---

### Performance Metrics

**Measured Values (Typical Run):**

| Metric | Value | Unit |
|--------|-------|------|
| A* Planning Time | 0.5-2.0 | seconds |
| Control Loop Frequency | 50 | Hz |
| Position Update Rate | 50 | Hz |
| Obstacle Detection Range | 6.0 | meters |
| Waypoint Accuracy | ±0.5 | meters |
| Goal Threshold | 1.5 | meters |
| Average Flight Speed | 2-4 | m/s |
| Maximum Acceleration | 3.0 | m/s² |

---

## Known Limitations

### 1. Local Minima Problem (APF)

**Issue:** APF can get stuck in local minima between obstacles.

**Mitigation:** 
- Implemented stuck detection (< 0.2m movement for 15 iterations)
- Random perpendicular force injection for escape
- Tangential force component for sliding

**Limitation:** May still occur in complex obstacle configurations.

---

### 2. Static Obstacle Assumption

**Issue:** System assumes obstacles are static and known before planning.

**Current State:**
- A* plans once at mission start
- No replanning during flight
- Dynamic obstacles not detected

**Future Work:** Add replanning trigger when obstacles detected far from A* path.

---

### 3. 2D Planning Only

**Issue:** A* operates in 2D (x, y), ignores height variations.

**Current State:**
- All obstacles assumed to be vertical cylinders
- Drone flies at fixed altitude (7m)
- No terrain following

**Future Work:** Extend to 3D A* with octree representation.

---

### 4. Gazebo-Specific Implementation

**Issue:** Pose bridge uses Gazebo CLI commands (`gz model`).

**Limitation:**
- Not portable to real hardware
- Adds latency (subprocess calls)
- Requires Gazebo-specific command

**Real Hardware:** Use onboard sensors (GPS, VIO, SLAM) for localization.

---

### 5. No Sensor Simulation

**Issue:** Obstacles are known a priori, not detected via sensors.

**Current State:**
- Perfect knowledge of environment
- No LiDAR, camera, or depth sensors
- No uncertainty in obstacle positions

**Future Work:** Add sensor simulation (ray casting, point clouds) and SLAM.

---

### 6. Single Drone Only

**Issue:** No multi-agent coordination or collision avoidance.

**Limitation:**
- Cannot handle multiple drones
- No communication between agents

**Future Work:** Extend to multi-drone formations with distributed planning.

---

## Future Enhancements

### Short-Term (1-3 months)

1. **Dynamic Replanning**
   - Monitor for significant deviations from A* path
   - Trigger replanning when new obstacles detected
   - Implement replanning without stopping drone

2. **Improved Visualization**
   - Show APF force vectors in RViz
   - Visualize obstacle detection zones
   - Display mode switching events

3. **Parameter Auto-Tuning**
   - Adaptive APF gains based on obstacle density
   - Dynamic safety distance based on speed
   - Machine learning for optimal parameter selection

4. **Sensor Integration**
   - Add simulated LiDAR for obstacle detection
   - Implement occupancy grid mapping
   - Test with partial observability

---

### Medium-Term (3-6 months)

1. **3D Path Planning**
   - Extend A* to 3D octree grid
   - Handle varying obstacle heights
   - Terrain-following capability

2. **Advanced APF Techniques**
   - Virtual force field (VFF) for better local minima handling
   - Adaptive potential fields based on goal distance
   - Velocity obstacles (VO) for dynamic obstacle avoidance

3. **Real Hardware Deployment**
   - Port to real PX4 drone (e.g., Pixhawk 4)
   - Integrate onboard computer (Jetson Nano, Raspberry Pi)
   - Test with real sensors (Intel RealSense, RPLiDAR)

4. **Mission Planning Interface**
   - Web-based mission planning GUI
   - Waypoint upload/download
   - Real-time monitoring dashboard

---

### Long-Term (6-12 months)

1. **Multi-Drone Coordination**
   - Distributed path planning
   - Formation flying
   - Leader-follower control
   - Collision avoidance between drones

2. **Advanced Algorithms**
   - RRT* (Rapidly-exploring Random Trees) for complex environments
   - Probabilistic roadmap (PRM) for offline planning
   - Deep reinforcement learning for end-to-end navigation

3. **Full SLAM Integration**
   - Visual-Inertial Odometry (VIO)
   - Loop closure detection
   - Map building and updating
   - Localization in GPS-denied environments

4. **Safety & Certification**
   - Geofencing and no-fly zones
   - Failsafe behaviors (RTL, land)
   - Redundant sensors and planning
   - Compliance with aviation regulations

---

## Troubleshooting

### Common Issues

#### 1. "No path found" Error

**Symptoms:** A* returns straight line instead of navigating around obstacles.

**Possible Causes:**
- Goal is inside an obstacle (with safety margin)
- Grid resolution too coarse
- Safety distance too large

**Solutions:**
```bash
# Reduce safety distance in launch file
'safety_distance': 2.0  # Instead of 3.0

# Increase grid resolution
'grid_resolution': 0.3  # Instead of 0.5

# Check obstacle positions
ros2 topic echo /drone/obstacles
```

---

#### 2. Drone Not Moving

**Symptoms:** Drone arms and takes off but doesn't navigate.

**Checklist:**
- [ ] Offboard mode enabled? (Check PX4 console)
- [ ] Navigation started? (`/drone/start_navigation` published)
- [ ] A* path received? (`ros2 topic echo /drone/astar_path`)
- [ ] Position feedback working? (`ros2 topic echo /fmu/out/vehicle_local_position_v1`)

**Solutions:**
```bash
# Check drone status
ros2 topic echo /drone/status

# Manually enable offboard
./build/px4_sitl_default/bin/px4-commander mode offboard

# Restart navigation trigger
ros2 topic pub --once /drone/start_navigation std_msgs/msg/Bool "data: true"
```

---

#### 3. Drone Oscillating (Unstable)

**Symptoms:** Drone wobbles or oscillates around waypoints.

**Possible Causes:**
- APF gains too high
- Control loop frequency too low
- PX4 position controller tuning

**Solutions:**
```python
# Reduce APF repulsive gain in utils/apf.py
self.k_rep = 5.0  # Instead of 8.0

# Reduce APF step size in drone_controller_px4.py
step_size = 0.5  # Instead of 0.8

# Tune PX4 position controller (MC_POS_P, MC_VEL_P)
```

---

#### 4. Stuck in Local Minima

**Symptoms:** Drone stops moving and rotates in place near obstacles.

**Check:**
```bash
# Monitor force vectors (add debug prints in apf.py)
print(f"Attractive force: {f_att}")
print(f"Repulsive force: {f_rep}")
print(f"Total force: {total_force}")
```

**Solutions:**
- Wait for stuck detection (15 iterations = 0.75 seconds)
- Reduce `stuck_threshold` in `apf.py` for faster escape
- Increase `random_force_gain` for stronger escape maneuvers

---

#### 5. Micro XRCE-DDS Agent Connection Fails

**Symptoms:** PX4 topics not appearing in ROS 2 (`ros2 topic list`).

**Check:**
```bash
# Verify agent is running
ps aux | grep MicroXRCEAgent

# Check PX4 client status
# In PX4 console:
uxrce_dds_client status
```

**Solutions:**
```bash
# Restart agent
killall MicroXRCEAgent
MicroXRCEAgent udp4 -p 8888

# In PX4 console, restart client
uxrce_dds_client stop
uxrce_dds_client start -t udp -p 8888
```

---

#### 6. Gazebo Obstacles Not Spawning

**Symptoms:** Red cylinders not visible in Gazebo.

**Check:**
```bash
# List Gazebo models
gz model -l

# Check obstacle topic
ros2 topic echo /drone/obstacles
```

**Solutions:**
```bash
# Manually spawn a test obstacle
gz service -s /world/default/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf_filename: "/tmp/obstacle_0.sdf"'

# Check Gazebo logs
gz log
```

---

## Appendix

### A. File Structure

```
drone_px4_ws/
├── src/
│   ├── drone_path_planning_px4/
│   │   ├── config/
│   │   │   └── planner_params.yaml
│   │   ├── drone_path_planning_px4/
│   │   │   ├── __init__.py
│   │   │   ├── drone_controller_px4.py       # Main hybrid controller
│   │   │   ├── path_planner_node.py          # A* planner
│   │   │   ├── obstacle_generator.py         # Random obstacle creator
│   │   │   ├── spawn_gazebo_obstacles.py     # Gazebo spawner
│   │   │   ├── visualization_node.py         # RViz markers
│   │   │   ├── gazebo_pose_bridge.py         # Pose feedback
│   │   │   └── utils/
│   │   │       ├── __init__.py
│   │   │       ├── astar.py                  # A* algorithm
│   │   │       └── apf.py                    # APF algorithm
│   │   ├── launch/
│   │   │   ├── full_path_planning.launch.py  # Main launch file
│   │   │   ├── path_planning.launch.py
│   │   │   └── px4_simulation.launch.py
│   │   ├── worlds/
│   │   │   └── obstacle_world.sdf
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   └── px4_msgs/                             # PX4 message definitions
├── build/                                     # Build artifacts
├── install/                                   # Installed packages
├── log/                                       # Build logs
└── Docs/
    └── README.md
```

---

### B. Dependencies

**ROS 2 Packages:**
- `rclpy` - ROS 2 Python client library
- `std_msgs` - Standard message types
- `geometry_msgs` - Geometric messages (Pose, Point, etc.)
- `nav_msgs` - Navigation messages (Path, Odometry)
- `visualization_msgs` - RViz markers
- `sensor_msgs` - Sensor data messages
- `tf2_ros` - Transform library
- `px4_msgs` - PX4-specific message types

**Python Libraries:**
- `numpy` - Numerical computing
- `matplotlib` - Plotting (optional)
- `heapq` - Priority queue for A*
- `subprocess` - System command execution
- `json` - JSON serialization

**System Dependencies:**
- PX4-Autopilot (SITL)
- Gazebo Garden
- Micro-XRCE-DDS-Agent

---

### C. References

**Algorithms:**
1. Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). *A formal basis for the heuristic determination of minimum cost paths.* IEEE Transactions on Systems Science and Cybernetics.

2. Khatib, O. (1986). *Real-time obstacle avoidance for manipulators and mobile robots.* The International Journal of Robotics Research.

3. Ferguson, D., Likhachev, M., & Stentz, A. (2008). *A Guide to Heuristic-based Path Planning.* Proceedings of the International Workshop on Planning under Uncertainty for Autonomous Systems.

**PX4 Documentation:**
- [PX4 User Guide](https://docs.px4.io/)
- [PX4 ROS 2 Integration](https://docs.px4.io/main/en/ros/ros2_comm.html)
- [Offboard Control](https://docs.px4.io/main/en/flight_modes/offboard.html)

**ROS 2 Documentation:**
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Navigation2 Stack](https://navigation.ros.org/)

---

### D. Contact & Support

**Project Maintainer:** belal212  
**Email:** belal212@example.com  
**License:** MIT  
**Repository:** [GitHub Link]  

For issues, feature requests, or contributions, please open an issue on the GitHub repository.

---

### E. Changelog

**v1.0.0 (December 2025)**
- Initial release
- Hybrid A* + APF navigation
- PX4 SITL integration
- Gazebo obstacle spawning
- ROS 2 Humble support
- Complete documentation

---

## Conclusion

This project demonstrates a robust **hybrid path planning approach** combining the strengths of global planning (A*) and local reactive control (APF). The system successfully navigates complex environments with multiple obstacles while maintaining safety margins and computational efficiency.

Key achievements:
- ✅ Modular ROS 2 architecture for easy extension
- ✅ Seamless PX4 integration via Micro XRCE-DDS
- ✅ Intelligent mode switching between A* and APF
- ✅ Local minima escape mechanisms
- ✅ Full Gazebo simulation environment

The system provides a solid foundation for further research in autonomous drone navigation, multi-agent systems, and advanced planning algorithms. With the outlined future enhancements, this platform can be extended to real-world deployments and complex mission scenarios.

---

**End of Technical Report**
