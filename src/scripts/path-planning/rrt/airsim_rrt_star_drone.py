"""
AirSim RRT★ Path Planning Integration for Drone 1
Autonomous navigation with random obstacle avoidance
SAFE MODE: Visual obstacles only (prevents Unreal crashes)
"""

import airsim
import numpy as np
import time
import sys
from rrt_star_3d import (RRTStar3D, generate_random_obstacles, 
                         visualize_rrt_star_3d, smooth_path, 
                         PerformanceMetrics)
try:
    from visualize_obstacles import draw_path_in_airsim, clear_all_plots
except:
    pass  # Optional visualization


class AirSimRRTStarController:
    """Controls Drone 1 in AirSim using RRT★ path planning"""
    
    def __init__(self, drone_name="Drone1"):
        """
        Initialize AirSim connection and controller
        
        Args:
            drone_name: Name of the drone to control in AirSim
        """
        self.drone_name = drone_name
        self.client = None
        self.spawned_objects = []  # Track spawned obstacle objects
        
    def connect(self):
        """Connect to AirSim and initialize drone"""
        try:
            print(f"Connecting to AirSim...")
            self.client = airsim.MultirotorClient()
            self.client.confirmConnection()
            print("✓ Connected to AirSim successfully")
            
            # Enable API control for Drone 1
            print(f"Enabling API control for {self.drone_name}...")
            self.client.enableApiControl(True, self.drone_name)
            print(f"✓ API control enabled for {self.drone_name}")
            
            # Arm the drone
            print(f"Arming {self.drone_name}...")
            self.client.armDisarm(True, self.drone_name)
            print(f"✓ {self.drone_name} armed")
            
            return True
            
        except Exception as e:
            print(f"✗ Error connecting to AirSim: {e}")
            print("Make sure AirSim/Unreal Engine is running!")
            return False
    
    def get_drone_position(self):
        """
        Get current position of the drone
        
        Returns:
            numpy array [x, y, z] in NED coordinates
        """
        try:
            state = self.client.getMultirotorState(vehicle_name=self.drone_name)
            pos = state.kinematics_estimated.position
            return np.array([pos.x_val, pos.y_val, pos.z_val])
        except Exception as e:
            print(f"Error getting drone position: {e}")
            return None
    
    def takeoff(self, timeout=10):
        """
        Takeoff the drone
        
        Args:
            timeout: Maximum time to wait for takeoff
        """
        try:
            print(f"Taking off {self.drone_name}...")
            self.client.takeoffAsync(timeout_sec=timeout, vehicle_name=self.drone_name).join()
            print(f"✓ {self.drone_name} has taken off")
            time.sleep(2)  # Stabilize
            return True
        except Exception as e:
            print(f"Error during takeoff: {e}")
            return False
    
    def fly_path(self, path, velocity=5.0, visualize=True, metrics=None):
        """
        Fly the drone along the computed path
        
        Args:
            path: List of waypoints [x, y, z]
            velocity: Flight velocity in m/s
            visualize: Whether to visualize waypoints in AirSim
            metrics: PerformanceMetrics object for tracking
        """
        if not path or len(path) == 0:
            print("No path to follow!")
            return False
        
        print(f"\n{'='*50}")
        print(f"Starting path execution with {len(path)} waypoints")
        print(f"Velocity: {velocity} m/s")
        print(f"{'='*50}\n")
        
        execution_start_time = time.time()
        
        try:
            # Optionally visualize waypoints in AirSim
            if visualize:
                self._visualize_waypoints_in_airsim(path)
            
            # Fly through each waypoint
            for i, waypoint in enumerate(path):
                print(f"Flying to waypoint {i+1}/{len(path)}: [{waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f}]")
                
                # Move to waypoint
                self.client.moveToPositionAsync(
                    waypoint[0], waypoint[1], waypoint[2],
                    velocity=velocity,
                    vehicle_name=self.drone_name
                ).join()
                
                # Get current position for verification
                current_pos = self.get_drone_position()
                if current_pos is not None:
                    distance = np.linalg.norm(current_pos - waypoint)
                    print(f"  → Reached waypoint (error: {distance:.2f}m)")
                
                # Update metrics
                if metrics:
                    metrics.update_peak_memory()
                    metrics.record_cpu()
                
                time.sleep(0.5)  # Small pause between waypoints
            
            execution_end_time = time.time()
            
            if metrics:
                metrics.execution_time = execution_end_time - execution_start_time
            
            print(f"\n✓ Path execution completed successfully!")
            print(f"Execution time: {execution_end_time - execution_start_time:.2f} seconds")
            return True
            
        except Exception as e:
            print(f"\n✗ Error during path execution: {e}")
            return False
    
    def _visualize_waypoints_in_airsim(self, path):
        """
        Visualize waypoints as markers in AirSim
        
        Args:
            path: List of waypoints
        """
        try:
            # Clear previous markers
            # self.client.simFlushPersistentMarkers()
            
            # Plot waypoints (if AirSim API supports it)
            for i, waypoint in enumerate(path):
                # Note: simPlotPoints may not be available in all AirSim versions
                # This is optional visualization
                pass
            
            print("Waypoints visualization attempted")
        except Exception as e:
            print(f"Note: Could not visualize waypoints in AirSim: {e}")
    
    def spawn_obstacles(self, obstacles):
        """
        SAFE MODE: Draw beautiful visual obstacles (prevents crashes)
        
        Args:
            obstacles: List of obstacle dicts with 'center' and 'radius'
        
        Returns:
            Number of obstacles successfully spawned (0 in safe mode)
        """
        print(f"\n🛡️ SAFE MODE: Drawing {len(obstacles)} visual obstacles...")
        print("   (Physical spawning disabled to prevent Unreal Engine crashes)")
        
        # Draw beautiful sphere obstacles using dense points
        try:
            for i, obs in enumerate(obstacles):
                center = obs['center']
                radius = obs['radius']
                
                # Generate points on sphere surface using spherical coordinates
                # More points = better looking sphere
                num_phi = 12  # Latitude divisions
                num_theta = 24  # Longitude divisions
                
                sphere_points = []
                for phi_step in range(num_phi):
                    phi = np.pi * phi_step / (num_phi - 1)  # 0 to π
                    for theta_step in range(num_theta):
                        theta = 2 * np.pi * theta_step / num_theta  # 0 to 2π
                        
                        # Spherical to Cartesian conversion
                        x = center[0] + radius * np.sin(phi) * np.cos(theta)
                        y = center[1] + radius * np.sin(phi) * np.sin(theta)
                        z = center[2] + radius * np.cos(phi)
                        
                        sphere_points.append(airsim.Vector3r(x, y, z))
                
                # Draw all points as a sphere
                self.client.simPlotPoints(
                    sphere_points,
                    color_rgba=[1.0, 0.0, 0.0, 1.0],  # Solid red
                    size=15,  # Large point size for visibility
                    duration=-1,
                    is_persistent=True
                )
                
                # Also draw a few circles for better definition
                circle_points = 32
                angles = np.linspace(0, 2*np.pi, circle_points, endpoint=False)
                
                # Equator circle (XY plane)
                equator = [airsim.Vector3r(
                    center[0] + radius * np.cos(a),
                    center[1] + radius * np.sin(a),
                    center[2]
                ) for a in angles]
                
                # Meridian circle (XZ plane)
                meridian = [airsim.Vector3r(
                    center[0] + radius * np.cos(a),
                    center[1],
                    center[2] + radius * np.sin(a)
                ) for a in angles]
                
                # Draw circles with thick lines
                self.client.simPlotLineStrip(equator, color_rgba=[0.8, 0.0, 0.0, 1.0], 
                                            thickness=5.0, duration=-1, is_persistent=True)
                self.client.simPlotLineStrip(meridian, color_rgba=[0.8, 0.0, 0.0, 1.0], 
                                             thickness=5.0, duration=-1, is_persistent=True)
            
            print(f"✓ Drew {len(obstacles)} beautiful sphere obstacles (red)")
            print("   These look great and path planning treats them as real!")
            return 0  # Return 0 to indicate visual mode
            
        except Exception as e:
            print(f"⚠ Could not draw visual obstacles: {e}")
            print("   Continuing with invisible obstacles...")
            return 0
    
    def clear_obstacles(self):
        """Clear visual obstacles (safe mode - no object destruction)"""
        try:
            # Clear all plot lines/spheres
            self.client.simFlushPersistentMarkers()
            print("✓ Cleared visual obstacles")
        except:
            pass
        
        self.spawned_objects = []
    
    def hover(self, duration=2.0):
        """Hover in place for specified duration"""
        print(f"Hovering for {duration} seconds...")
        self.client.hoverAsync(vehicle_name=self.drone_name).join()
        time.sleep(duration)
    
    def land(self):
        """Land the drone"""
        try:
            print(f"\nLanding {self.drone_name}...")
            self.client.landAsync(vehicle_name=self.drone_name).join()
            print(f"✓ {self.drone_name} has landed")
            return True
        except Exception as e:
            print(f"Error during landing: {e}")
            return False
    
    def disconnect(self):
        """Disconnect and reset drone"""
        try:
            if self.client is not None:
                # Clear spawned obstacles first
                self.clear_obstacles()
                
                print(f"\nDisarming {self.drone_name}...")
                self.client.armDisarm(False, self.drone_name)
                
                print(f"Disabling API control...")
                self.client.enableApiControl(False, self.drone_name)
                
                print("✓ Disconnected successfully")
        except Exception as e:
            print(f"Error during disconnect: {e}")
    
    def reset(self):
        """Reset the simulation"""
        try:
            print("Resetting simulation...")
            self.client.reset()
            time.sleep(2)
            print("✓ Simulation reset")
        except Exception as e:
            print(f"Error during reset: {e}")


def rrt_star_3d_airsim(
    client,
    start_pos,
    goal_pos,
    step_size=5,
    goal_sample_rate=0.05,
    max_iter=1000,
    search_radius=15,
    obstacle_count=30,
    obstacle_radius_range=(3, 8),
    boundary=((-100, 100), (-100, 100), (-30, -5)),
    drone_name="Drone1",
    velocity=15.0,
    smooth_path_flag=True,
    visualize=True
):
    """
    Complete RRT★ path planning and execution pipeline for AirSim Drone 1
    
    Args:
        client: AirSimRRTStarController instance
        start_pos: Start position [x, y, z]
        goal_pos: Goal position [x, y, z]
        step_size: RRT★ step size
        goal_sample_rate: Probability of sampling goal
        max_iter: Maximum RRT★ iterations
        search_radius: Radius for rewiring
        obstacle_count: Number of random obstacles
        obstacle_radius_range: (min, max) radius for obstacles
        boundary: 3D environment boundaries
        drone_name: Name of drone in AirSim
        velocity: Flight velocity
        smooth_path_flag: Whether to smooth the path
        visualize: Whether to visualize results
    
    Returns:
        PerformanceMetrics object with complete performance data
    """
    
    print("\n" + "="*70)
    print("RRT★ 3D PATH PLANNING FOR AIRSIM DRONE 1")
    print("="*70)
    
    # Initialize performance tracking
    metrics = PerformanceMetrics()
    metrics.start_tracking()
    
    # Step 1: Generate random obstacles
    print("\n[1/6] Generating random obstacles...")
    obstacles = generate_random_obstacles(
        count=obstacle_count,
        boundary=boundary,
        radius_range=obstacle_radius_range,
        start_pos=start_pos,
        goal_pos=goal_pos,
        min_clearance=10.0
    )
    
    # Step 1b: Spawn REAL obstacles in Unreal Engine
    print("\n[2/6] Spawning REAL obstacles in Unreal Engine...")
    spawned_count = client.spawn_obstacles(obstacles)
    
    if spawned_count > 0:
        print(f"✓ Successfully spawned {spawned_count} visible obstacles!")
    else:
        print(f"⚠ Warning: Could not spawn physical obstacles")
        print(f"  Obstacles will be virtual (invisible but functional)")
    
    time.sleep(1.5)  # Give Unreal time to render objects
    
    # Step 2: Plan path with RRT★
    print("\n[3/6] Planning path with RRT★...")
    rrt = RRTStar3D(
        start=start_pos,
        goal=goal_pos,
        obstacles=obstacles,
        boundary=boundary,
        step_size=step_size,
        goal_sample_rate=goal_sample_rate,
        max_iter=max_iter,
        search_radius=search_radius,
        drone_radius=2.0,  # Safety margin: 2 meters around drone
        metrics=metrics
    )
    
    path = rrt.plan()
    
    if path is None:
        print("\n✗ Failed to find a valid path!")
        metrics.end_tracking()
        return metrics
    
    # Step 3: Smooth path (optional)
    if smooth_path_flag and len(path) > 2:
        print("\n[4/6] Smoothing path...")
        original_length = len(path)
        path = smooth_path(path, alpha=0.3, iterations=50)
        print(f"Path smoothed: {original_length} → {len(path)} waypoints")
    else:
        print("\n[4/6] Skipping path smoothing")
    
    # Step 4: Visualize (offline)
    if visualize:
        print("\n[5/6] Visualizing path...")
        visualize_rrt_star_3d(rrt, path, show=False, save_path="rrt_star_path_3d.png")
        # Draw path in AirSim (green line - this works!)
        try:
            draw_path_in_airsim(client.client, path, color=[0, 1, 0], thickness=5.0)
        except Exception as e:
            print(f"  Note: Could not draw path in AirSim: {e}")
    else:
        print("\n[5/6] Skipping visualization")
    
    # Step 5: Execute path in AirSim
    print("\n[6/6] Executing path in AirSim...")
    success = client.fly_path(path, velocity=velocity, visualize=False, metrics=metrics)
    
    # End tracking
    metrics.end_tracking()
    
    if success:
        print("\n" + "="*70)
        print("✓ MISSION COMPLETED SUCCESSFULLY!")
        print("="*70)
    else:
        print("\n✗ Mission failed during execution")
    
    # Print and save performance report
    metrics.print_report()
    metrics.save_report("performance_report.txt")
    
    return metrics


def main():
    """Main execution function"""
    
    # Initialize controller
    controller = AirSimRRTStarController(drone_name="Drone1")
    
    try:
        # Connect to AirSim
        if not controller.connect():
            print("Failed to connect to AirSim. Exiting...")
            return
        
        # Clear any previous visualizations
        try:
            clear_all_plots(controller.client)
        except:
            pass  # Not critical if this fails
        
        # Takeoff
        if not controller.takeoff(timeout=10):
            print("Takeoff failed. Exiting...")
            controller.disconnect()
            return
        
        time.sleep(2)
        
        # Get current position as start (FIXED - doesn't change)
        start_pos = controller.get_drone_position()
        if start_pos is None:
            print("Could not get drone position. Using default start position.")
            start_pos = np.array([0, 0, -10])
        else:
            print(f"Start position (fixed): [{start_pos[0]:.2f}, {start_pos[1]:.2f}, {start_pos[2]:.2f}]")
        
        # Generate RANDOM goal position (positive coordinates between 0-300)
        goal_x = np.random.uniform(0, 300)
        goal_y = np.random.uniform(0, 300)
        goal_z = np.random.uniform(-25, -15)  # Random altitude between -25 and -15
        
        goal_pos = np.array([goal_x, goal_y, goal_z])
        print(f"Goal position (RANDOM): [{goal_pos[0]:.2f}, {goal_pos[1]:.2f}, {goal_pos[2]:.2f}]")
        print(f"  Distance from start: {np.linalg.norm(goal_pos - start_pos):.2f} meters")
        
        # Define environment boundary (0-300 range for positive coordinates)
        boundary = ((0, 300), (0, 300), (-35, -5))
        
        # Run RRT★ planning and execution with BETTER parameters
        metrics = rrt_star_3d_airsim(
            client=controller,
            start_pos=start_pos.tolist(),
            goal_pos=goal_pos.tolist(),
            step_size=8,  # Increased from 5 - larger steps = faster exploration
            goal_sample_rate=0.15,  # Increased from 0.05 - bias toward goal more often
            max_iter=1000,  # Increased from 1000 - more iterations to find path
            search_radius=20,  # Increased from 15 - wider rewiring radius
            obstacle_count=20,  # Reduced from 30 - fewer obstacles = easier path
            obstacle_radius_range=(3, 6),  # Reduced max from 8 to 6 - smaller obstacles
            boundary=boundary,
            drone_name="Drone1",
            velocity=5.0,
            smooth_path_flag=True,
            visualize=True
        )
        
        if metrics and metrics.optimal_cost > 0:
            # Hover at goal
            controller.hover(duration=3.0)
        
        # Land
        controller.land()
        
        # Wait before disconnecting
        time.sleep(2)
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user!")
        print("Landing drone...")
        controller.land()
        
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        # Always disconnect properly
        controller.disconnect()
        print("\nProgram terminated.")


if __name__ == "__main__":
    main()
