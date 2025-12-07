#!/usr/bin/env python3
"""
Fire Detection and Swarm Task Allocation Demo
============================================
This script demonstrates:
1. Fire detection simulation with multiple fire locations
2. Task allocation - different drones get different missions
3. ROS2 integration for visualization in RViz2
4. Swarm coordination with role-based behavior

Drone Roles:
- Drone0: Scout/Search drone with YOLO detection
- Drone1-2: Fire monitoring drones
- Drone3-4: Perimeter patrol drones
"""

import airsim
import numpy as np
import time
import math
from threading import Thread, Lock, Semaphore
import cv2

# Try importing ROS2 (will work if ROS2 is sourced)
try:
    import rclpy
    from rclpy.node import Node
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA, String
    ROS2_AVAILABLE = True
except ImportError:
    print("⚠️  ROS2 not available. Running without RViz2 visualization.")
    ROS2_AVAILABLE = False


class FireLocation:
    """Represents a fire location in the environment"""
    def __init__(self, x, y, z, intensity=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.intensity = intensity  # 0.0 to 1.0
        self.detected = False
        self.monitoring_drone = None
        
    def get_position(self):
        return airsim.Vector3r(self.x, self.y, self.z)


class DroneTask:
    """Base class for drone tasks"""
    def __init__(self, task_name, priority=1):
        self.task_name = task_name
        self.priority = priority
        self.status = "assigned"  # assigned, in_progress, completed, failed
        
    def __str__(self):
        return f"{self.task_name} (Priority: {self.priority}, Status: {self.status})"


class SearchTask(DroneTask):
    """Search pattern task"""
    def __init__(self, search_area, altitude):
        super().__init__("Search Pattern", priority=3)
        self.search_area = search_area
        self.altitude = altitude


class MonitorFireTask(DroneTask):
    """Monitor a specific fire location"""
    def __init__(self, fire_location):
        super().__init__("Monitor Fire", priority=5)
        self.fire_location = fire_location


class PerimeterPatrolTask(DroneTask):
    """Patrol a perimeter"""
    def __init__(self, center, radius, altitude):
        super().__init__("Perimeter Patrol", priority=2)
        self.center = center
        self.radius = radius
        self.altitude = altitude


class ROS2Publisher(Node):
    """ROS2 node for publishing visualization markers"""
    def __init__(self):
        print("🔧 Initializing ROS2Publisher node...")
        super().__init__('fire_detection_swarm')
        
        print("📡 Creating publishers...")
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/fire_markers', 10)
        self.task_pub = self.create_publisher(MarkerArray, '/task_markers', 10)
        self.detection_pub = self.create_publisher(String, '/detection_events', 10)
        
        print("⏰ Creating timer for periodic updates...")
        # Timer for periodic updates
        self.timer = self.create_timer(0.5, self.publish_markers)
        
        self.fires = []
        self.tasks = {}
        self.drone_positions = {}
        self.lock = Lock()
        
        self.get_logger().info('🔥 Fire Detection ROS2 Node Started')
        print("✅ ROS2Publisher initialization complete!")
    
    def update_fires(self, fires):
        """Update fire locations"""
        with self.lock:
            self.fires = fires
    
    def update_task(self, drone_name, task, position):
        """Update drone task and position"""
        with self.lock:
            self.tasks[drone_name] = task
            self.drone_positions[drone_name] = position
    
    def publish_markers(self):
        """Publish visualization markers to RViz2"""
        try:
            with self.lock:
                # Publish fire markers
                fire_array = MarkerArray()
                for i, fire in enumerate(self.fires):
                    # Fire marker (red sphere)
                    marker = Marker()
                    marker.header.frame_id = "world_ned"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "fires"
                    marker.id = i
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    
                    # Convert to float to avoid numpy type issues
                    marker.pose.position.x = float(fire.x)
                    marker.pose.position.y = float(fire.y)
                    marker.pose.position.z = float(fire.z)
                    marker.pose.orientation.w = 1.0
                    
                    # Size based on intensity
                    size = float(2.0 + (fire.intensity * 3.0))
                    marker.scale.x = size
                    marker.scale.y = size
                    marker.scale.z = size
                    
                    # Color: red with pulsing alpha
                    marker.color.r = 1.0
                    marker.color.g = 0.3 if fire.detected else 0.0
                    marker.color.b = 0.0
                    marker.color.a = 0.6 + 0.4 * math.sin(time.time() * 2.0)  # Pulsing effect
                    
                    marker.lifetime.sec = 1
                    fire_array.markers.append(marker)
                    
                    # Text label
                    text_marker = Marker()
                    text_marker.header = marker.header
                    text_marker.ns = "fire_labels"
                    text_marker.id = i + 1000
                    text_marker.type = Marker.TEXT_VIEW_FACING
                    text_marker.action = Marker.ADD
                    
                    text_marker.pose.position.x = float(fire.x)
                    text_marker.pose.position.y = float(fire.y)
                    text_marker.pose.position.z = float(fire.z + 3.0)
                    
                    text_marker.scale.z = 1.5
                    text_marker.color.r = 1.0
                    text_marker.color.g = 1.0
                    text_marker.color.b = 1.0
                    text_marker.color.a = 1.0
                    
                    status = "🔥 DETECTED" if fire.detected else "🔥 FIRE"
                    text_marker.text = f"{status}\nIntensity: {fire.intensity:.1f}"
                    text_marker.lifetime.sec = 1
                    
                    fire_array.markers.append(text_marker)
                
                self.marker_pub.publish(fire_array)
                
                # Publish task markers
                task_array = MarkerArray()
                for i, (drone_name, task) in enumerate(self.tasks.items()):
                    if drone_name not in self.drone_positions:
                        continue
                        
                    pos = self.drone_positions[drone_name]
                    
                    # Task text marker above drone
                    marker = Marker()
                    marker.header.frame_id = "world_ned"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "tasks"
                    marker.id = i
                    marker.type = Marker.TEXT_VIEW_FACING
                    marker.action = Marker.ADD
                    
                    marker.pose.position.x = float(pos[0])
                    marker.pose.position.y = float(pos[1])
                    marker.pose.position.z = float(pos[2] - 3.0)
                    
                    marker.scale.z = 1.0
                    
                    # Color based on task status
                    if task.status == "in_progress":
                        marker.color.r = 0.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0
                    elif task.status == "completed":
                        marker.color.r = 0.0
                        marker.color.g = 0.0
                        marker.color.b = 1.0
                    else:
                        marker.color.r = 1.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0
                    marker.color.a = 1.0
                    
                    marker.text = f"{drone_name}\n{task}"
                    marker.lifetime.sec = 1
                    
                    task_array.markers.append(marker)
                
                self.task_pub.publish(task_array)
        except Exception as e:
            self.get_logger().error(f'Error publishing markers: {e}')
            import traceback
            traceback.print_exc()


class FireDetectionSwarm:
    """Main swarm controller with task allocation"""
    
    def __init__(self, num_drones=5, use_ros2=True):
        self.num_drones = num_drones
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        
        self.drone_names = [f"Drone{i}" for i in range(num_drones)]
        self.drone_tasks = {}
        self.fires = []
        
        # Thread-safe lock for AirSim API calls
        self.airsim_lock = Lock()
        
        # Initialize ROS2 if available
        self.ros2_node = None
        if use_ros2 and ROS2_AVAILABLE:
            try:
                # Try to initialize rclpy (may already be initialized)
                if not rclpy.ok():
                    rclpy.init()
                
                self.ros2_node = ROS2Publisher()
                
                # Start ROS2 spin in separate thread
                self.ros2_thread = Thread(target=self._ros2_spin, daemon=True)
                self.ros2_thread.start()
                print("✅ ROS2 visualization enabled - check RViz2!")
                print(f"   Publishing to: /fire_markers, /task_markers, /detection_events")
            except Exception as e:
                print(f"⚠️  ROS2 initialization failed: {e}")
                print("   Continuing without ROS2 visualization...")
                self.ros2_node = None
        else:
            print("⚠️  Running without ROS2 visualization")
        
        # Initialize fires
        self._initialize_fires()
        
        print(f"\n{'='*70}")
        print(f"🔥 FIRE DETECTION AND SWARM TASK ALLOCATION DEMO")
        print(f"{'='*70}")
        print(f"Number of drones: {num_drones}")
        print(f"Number of fires: {len(self.fires)}")
        print(f"{'='*70}\n")
    
    def _ros2_spin(self):
        """Spin ROS2 node in separate thread"""
        try:
            rclpy.spin(self.ros2_node)
        except Exception as e:
            print(f"⚠️  ROS2 spin thread error: {e}")
            import traceback
            traceback.print_exc()
    
    def _initialize_fires(self):
        """Initialize fire locations"""
        # Create fire locations in the environment
        fire_configs = [
            (50, 30, -10, 0.8),   # Fire 1: Medium intensity
            (-40, -50, -8, 1.0),  # Fire 2: High intensity
            (80, -30, -12, 0.6),  # Fire 3: Low intensity
            (-70, 60, -9, 0.9),   # Fire 4: High intensity
        ]
        
        for x, y, z, intensity in fire_configs:
            self.fires.append(FireLocation(x, y, z, intensity))
        
        if self.ros2_node:
            self.ros2_node.update_fires(self.fires)
    
    def allocate_tasks(self):
        """Allocate tasks to drones based on their role"""
        print("\n📋 Task Allocation:")
        print("-" * 70)
        
        for i, drone_name in enumerate(self.drone_names):
            if i == 0:
                # Drone 0: Scout with search pattern
                task = SearchTask(search_area=(100, 100), altitude=-15)
                print(f"  {drone_name}: 🔍 Scout - Search for fires and objects")
            elif i < 3 and i <= len(self.fires):
                # Drones 1-2: Monitor specific fires
                fire_idx = i - 1
                if fire_idx < len(self.fires):
                    task = MonitorFireTask(self.fires[fire_idx])
                    self.fires[fire_idx].monitoring_drone = drone_name
                    print(f"  {drone_name}: 🔥 Monitor Fire {fire_idx + 1} at ({self.fires[fire_idx].x:.1f}, {self.fires[fire_idx].y:.1f})")
                else:
                    task = PerimeterPatrolTask(center=(0, 0), radius=60, altitude=-20)
                    print(f"  {drone_name}: 🛡️  Perimeter Patrol")
            else:
                # Remaining drones: Perimeter patrol
                task = PerimeterPatrolTask(center=(0, 0), radius=70, altitude=-18)
                print(f"  {drone_name}: 🛡️  Perimeter Patrol")
            
            self.drone_tasks[drone_name] = task
        
        print("-" * 70)
    
    def arm_and_takeoff(self):
        """Arm and takeoff all drones"""
        print("\n🚁 Arming and taking off...")
        
        for drone_name in self.drone_names:
            with self.airsim_lock:
                self.client.enableApiControl(True, drone_name)
                self.client.armDisarm(True, drone_name)
            time.sleep(0.2)
        
        # Staggered takeoff
        for i, drone_name in enumerate(self.drone_names):
            time.sleep(0.5)
            with self.airsim_lock:
                self.client.takeoffAsync(timeout_sec=10, vehicle_name=drone_name).join()
            print(f"  ✅ {drone_name} airborne")
        
        time.sleep(2)
    
    def execute_tasks(self, duration=60):
        """Execute assigned tasks"""
        print(f"\n▶️  Executing tasks for {duration} seconds...")
        print("  Watch RViz2 for real-time visualization!\n")
        
        start_time = time.time()
        
        # Start task execution threads
        threads = []
        for drone_name in self.drone_names:
            task = self.drone_tasks[drone_name]
            task.status = "in_progress"
            
            thread = Thread(target=self._execute_drone_task, 
                          args=(drone_name, task), 
                          daemon=True)
            thread.start()
            threads.append(thread)
        
        # Monitor progress
        while time.time() - start_time < duration:
            self._update_visualizations()
            self._print_status()
            time.sleep(5)
        
        print("\n⏹️  Task execution completed")
    
    def _execute_drone_task(self, drone_name, task):
        """Execute task for a specific drone"""
        try:
            if isinstance(task, SearchTask):
                self._execute_search_pattern(drone_name, task)
            elif isinstance(task, MonitorFireTask):
                self._execute_monitor_fire(drone_name, task)
            elif isinstance(task, PerimeterPatrolTask):
                self._execute_perimeter_patrol(drone_name, task)
        except Exception as e:
            print(f"  ⚠️  {drone_name} task error: {e}")
            task.status = "failed"
    
    def _execute_search_pattern(self, drone_name, task):
        """Execute search pattern"""
        alt = task.altitude
        pattern_points = [
            (30, 30, alt), (30, -30, alt), (-30, -30, alt), (-30, 30, alt)
        ]
        
        while task.status == "in_progress":
            for x, y, z in pattern_points:
                with self.airsim_lock:
                    self.client.moveToPositionAsync(x, y, z, 8, vehicle_name=drone_name).join()
                # Check for fires
                self._check_fire_detection(drone_name)
                time.sleep(0.5)
    
    def _execute_monitor_fire(self, drone_name, task):
        """Monitor fire location"""
        fire = task.fire_location
        
        # Orbit positions around fire
        orbit_radius = 8
        orbit_alt = fire.z - 5
        
        angles = np.linspace(0, 2*np.pi, 8, endpoint=False)
        orbit_points = [
            (fire.x + orbit_radius * np.cos(a),
             fire.y + orbit_radius * np.sin(a),
             orbit_alt)
            for a in angles
        ]
        
        fire.detected = True
        
        while task.status == "in_progress":
            for x, y, z in orbit_points:
                with self.airsim_lock:
                    self.client.moveToPositionAsync(x, y, z, 5, vehicle_name=drone_name).join()
                time.sleep(1)
    
    def _execute_perimeter_patrol(self, drone_name, task):
        """Execute perimeter patrol"""
        center_x, center_y = task.center
        radius = task.radius
        alt = task.altitude
        
        angles = np.linspace(0, 2*np.pi, 12, endpoint=False)
        patrol_points = [
            (center_x + radius * np.cos(a),
             center_y + radius * np.sin(a),
             alt)
            for a in angles
        ]
        
        while task.status == "in_progress":
            for x, y, z in patrol_points:
                with self.airsim_lock:
                    self.client.moveToPositionAsync(x, y, z, 10, vehicle_name=drone_name).join()
                time.sleep(0.5)
    
    def _check_fire_detection(self, drone_name):
        """Check if drone detected any fires"""
        with self.airsim_lock:
            state = self.client.getMultirotorState(vehicle_name=drone_name)
        pos = state.kinematics_estimated.position
        
        for fire in self.fires:
            dist = np.sqrt((pos.x_val - fire.x)**2 + 
                          (pos.y_val - fire.y)**2 + 
                          (pos.z_val - fire.z)**2)
            
            if dist < 20 and not fire.detected:
                fire.detected = True
                print(f"\n  🔥 {drone_name} DETECTED FIRE at ({fire.x:.1f}, {fire.y:.1f})!")
                if self.ros2_node:
                    msg = String()
                    msg.data = f"{drone_name} detected fire at ({fire.x:.1f}, {fire.y:.1f})"
                    self.ros2_node.detection_pub.publish(msg)
    
    def _update_visualizations(self):
        """Update ROS2 visualizations"""
        if not self.ros2_node:
            return
        
        # Update drone positions and tasks
        for drone_name in self.drone_names:
            try:
                with self.airsim_lock:
                    state = self.client.getMultirotorState(vehicle_name=drone_name)
                pos = state.kinematics_estimated.position
                position = (pos.x_val, pos.y_val, pos.z_val)
                
                task = self.drone_tasks[drone_name]
                self.ros2_node.update_task(drone_name, task, position)
            except:
                pass
        
        self.ros2_node.update_fires(self.fires)
    
    def _print_status(self):
        """Print current status"""
        detected_fires = sum(1 for f in self.fires if f.detected)
        print(f"  📊 Status: {detected_fires}/{len(self.fires)} fires detected")
    
    def land_all(self):
        """Land all drones"""
        print("\n🛬 Landing all drones...")
        
        land_futures = []
        for drone_name in self.drone_names:
            future = self.client.landAsync(vehicle_name=drone_name)
            land_futures.append((drone_name, future))
        
        for drone_name, future in land_futures:
            future.join()
            self.client.armDisarm(False, drone_name)
            print(f"  ✅ {drone_name} landed")
    
    def cleanup(self):
        """Cleanup resources"""
        if self.ros2_node:
            self.ros2_node.destroy_node()
            rclpy.shutdown()


def main():
    """Main function"""
    swarm = None
    
    try:
        # Create swarm controller
        swarm = FireDetectionSwarm(num_drones=5, use_ros2=True)
        
        # Allocate tasks
        swarm.allocate_tasks()
        
        # Arm and takeoff
        swarm.arm_and_takeoff()
        
        # Execute tasks
        swarm.execute_tasks(duration=120)  # Run for 2 minutes
        
        # Land all
        swarm.land_all()
        
        print("\n✅ Demo completed successfully!")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
        if swarm:
            swarm.land_all()
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        if swarm:
            swarm.cleanup()


if __name__ == "__main__":
    main()
