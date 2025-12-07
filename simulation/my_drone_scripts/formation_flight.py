#!/usr/bin/env python3
"""
5-Drone Formation Flight with Improved Controls
================================================
- Drone1: Keyboard controlled (leader)
- Drones 2-5: Follow in formation
- Real-time Lidar visualization
- Improved steering responsiveness
"""

import airsim
import numpy as np
import time
import os
from pynput import keyboard
from threading import Thread, Lock
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class FormationController:
    def __init__(self):
        print("🚁 Initializing 5-Drone Formation Controller...")
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        
        # Drone configuration
        self.drones = ["Drone1", "Drone2", "Drone3", "Drone4", "Drone5"]
        self.leader = "Drone1"
        
        # Formation offsets (relative to leader) - X, Y, Z in meters
        self.formation = {
            "Drone2": (5, 0, 0),    # 5m to the right
            "Drone3": (-5, 0, 0),   # 5m to the left
            "Drone4": (0, 5, 0),    # 5m behind
            "Drone5": (0, -5, 0)    # 5m ahead
        }
        
        # Enable API control for all drones
        for drone in self.drones:
            self.client.enableApiControl(True, drone)
            self.client.armDisarm(True, drone)
            print(f"✅ {drone} armed and ready")
        
        # Control state - IMPROVED RESPONSIVENESS
        self.control_lock = Lock()
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0
        self.speed = 8.0  # Increased from 5.0 for better response
        self.yaw_speed = 45.0  # Increased from 30.0 for sharper turns
        
        # Running flags
        self.running = True
        self.flying = False
        self.formation_enabled = True
        
        # Lidar data storage
        self.lidar_points = []
        self.show_lidar = False
        
    def takeoff_all(self):
        """Takeoff all drones"""
        print("\n🛫 Taking off all drones...")
        tasks = []
        for drone in self.drones:
            tasks.append(self.client.takeoffAsync(timeout_sec=10, vehicle_name=drone))
        
        for task in tasks:
            task.join()
        
        self.flying = True
        print("✅ All drones airborne in formation!\n")
        
    def land_all(self):
        """Land all drones"""
        print("\n🛬 Landing all drones...")
        tasks = []
        for drone in self.drones:
            tasks.append(self.client.landAsync(timeout_sec=10, vehicle_name=drone))
        
        for task in tasks:
            task.join()
        
        self.flying = False
        print("✅ All drones landed!\n")
        
    def emergency_stop(self):
        """Emergency stop - hover in place"""
        print("\n⚠️  EMERGENCY STOP!")
        with self.control_lock:
            self.vx = self.vy = self.vz = self.yaw_rate = 0
        
        for drone in self.drones:
            self.client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=drone)
    
    def on_press(self, key):
        """Handle key press events"""
        try:
            if hasattr(key, 'char') and key.char:
                k = key.char.lower()
                
                with self.control_lock:
                    if k == 'w':
                        self.vx = self.speed
                        print(f"⬆️  Forward: {self.speed} m/s")
                    elif k == 's':
                        self.vx = -self.speed
                        print(f"⬇️  Backward: {-self.speed} m/s")
                    elif k == 'a':
                        self.vy = -self.speed
                        print(f"⬅️  Left: {-self.speed} m/s")
                    elif k == 'd':
                        self.vy = self.speed
                        print(f"➡️  Right: {self.speed} m/s")
                    elif k == 'r':
                        self.vz = -self.speed
                        print(f"🔼 Up: {-self.speed} m/s")
                    elif k == 'f':
                        self.vz = self.speed
                        print(f"🔽 Down: {self.speed} m/s")
                    elif k == 'q':
                        self.yaw_rate = -self.yaw_speed
                        print(f"↪️  Yaw Left: {-self.yaw_speed} deg/s")
                    elif k == 'e':
                        self.yaw_rate = self.yaw_speed
                        print(f"↩️  Yaw Right: {self.yaw_speed} deg/s")
                    elif k == 't' and not self.flying:
                        Thread(target=self.takeoff_all, daemon=True).start()
                    elif k == 'l' and self.flying:
                        Thread(target=self.land_all, daemon=True).start()
                    elif k == 'v':  # Toggle lidar visualization
                        self.show_lidar = not self.show_lidar
                        print(f"📊 Lidar visualization: {'ON' if self.show_lidar else 'OFF'}")
                    elif k == 'b':  # Toggle formation
                        self.formation_enabled = not self.formation_enabled
                        print(f"🛸 Formation: {'ENABLED' if self.formation_enabled else 'DISABLED'}")
                        
            if key == keyboard.Key.space:
                self.emergency_stop()
            elif key == keyboard.Key.esc:
                print("\n🔚 ESC pressed - Shutting down...")
                self.running = False
                
        except Exception as e:
            print(f"Key press error: {e}")
    
    def on_release(self, key):
        """Handle key release events"""
        try:
            if hasattr(key, 'char') and key.char:
                k = key.char.lower()
                
                with self.control_lock:
                    if k in ['w', 's']:
                        self.vx = 0
                    elif k in ['a', 'd']:
                        self.vy = 0
                    elif k in ['r', 'f']:
                        self.vz = 0
                    elif k in ['q', 'e']:
                        self.yaw_rate = 0
                        
        except Exception as e:
            pass
    
    def leader_control_loop(self):
        """Control loop for leader drone"""
        print("🎮 Leader control loop started")
        
        while self.running:
            if self.flying:
                with self.control_lock:
                    vx, vy, vz, yaw_rate = self.vx, self.vy, self.vz, self.yaw_rate
                
                # Send velocity command with improved responsiveness
                try:
                    self.client.moveByVelocityAsync(
                        vx, vy, vz, 0.5,  # Reduced duration for faster response
                        airsim.DrivetrainType.MaxDegreeOfFreedom,
                        airsim.YawMode(True, yaw_rate),
                        vehicle_name=self.leader
                    )
                except Exception as e:
                    pass
            
            time.sleep(0.05)  # 20Hz for better responsiveness
    
    def formation_control_loop(self):
        """Control loop for follower drones"""
        print("🛸 Formation control loop started")
        
        while self.running:
            if self.flying and self.formation_enabled:
                try:
                    # Get leader position
                    leader_state = self.client.getMultirotorState(vehicle_name=self.leader)
                    leader_pos = leader_state.kinematics_estimated.position
                    leader_orient = leader_state.kinematics_estimated.orientation
                    
                    # Move followers to formation positions
                    for drone in self.drones[1:]:  # Skip leader
                        offset_x, offset_y, offset_z = self.formation[drone]
                        
                        # Calculate target position (offset from leader)
                        target_x = leader_pos.x_val + offset_x
                        target_y = leader_pos.y_val + offset_y
                        target_z = leader_pos.z_val + offset_z
                        
                        # Move to position smoothly
                        self.client.moveToPositionAsync(
                            target_x, target_y, target_z,
                            5,  # velocity
                            vehicle_name=drone
                        )
                        
                except Exception as e:
                    pass
            
            time.sleep(0.2)  # 5Hz for formation updates
    
    def lidar_visualization_loop(self):
        """Visualize lidar data in 3D"""
        print("📊 Lidar visualization ready (press 'V' to toggle)")
        
        # Setup matplotlib for interactive plotting
        plt.ion()
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        while self.running:
            if self.show_lidar and self.flying:
                try:
                    # Get lidar data from leader
                    lidar = self.client.getLidarData(lidar_name="Lidar1", vehicle_name=self.leader)
                    
                    if len(lidar.point_cloud) >= 3:
                        points = np.array(lidar.point_cloud, dtype=np.float32)
                        points = points.reshape(-1, 3)
                        
                        # Limit points for performance
                        if len(points) > 5000:
                            indices = np.random.choice(len(points), 5000, replace=False)
                            points = points[indices]
                        
                        # Calculate colors based on distance
                        distances = np.linalg.norm(points, axis=1)
                        colors = plt.cm.viridis(distances / distances.max())
                        
                        # Clear and plot
                        ax.clear()
                        ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
                                 c=colors, s=1, alpha=0.6)
                        
                        ax.set_xlabel('X (m)')
                        ax.set_ylabel('Y (m)')
                        ax.set_zlabel('Z (m)')
                        ax.set_title(f'Lidar Point Cloud - {len(points)} points')
                        ax.set_xlim([-20, 20])
                        ax.set_ylim([-20, 20])
                        ax.set_zlim([-20, 20])
                        
                        plt.draw()
                        plt.pause(0.1)
                        
                except Exception as e:
                    pass
            
            time.sleep(0.2)
        
        plt.close()
    
    def sensor_display_loop(self):
        """Display sensor data in terminal"""
        print("📊 Sensor display started")
        
        while self.running:
            time.sleep(2)
            
            if not self.flying:
                continue
            
            try:
                os.system('clear' if os.name == 'posix' else 'cls')
                
                print("=" * 80)
                print("  🚁 5-DRONE FORMATION FLIGHT - DASHBOARD")
                print("=" * 80)
                print(f"Leader: {self.leader} | Formation: {'✅' if self.formation_enabled else '❌'} | Lidar Viz: {'✅' if self.show_lidar else '❌'}")
                print(f"Control: vx={self.vx:.1f}, vy={self.vy:.1f}, vz={self.vz:.1f}, yaw={self.yaw_rate:.1f}")
                print("-" * 80)
                
                # Display all drones
                for i, drone in enumerate(self.drones):
                    try:
                        state = self.client.getMultirotorState(vehicle_name=drone)
                        pos = state.kinematics_estimated.position
                        
                        # Get lidar data
                        lidar = self.client.getLidarData(lidar_name="Lidar1", vehicle_name=drone)
                        num_points = len(lidar.point_cloud) // 3 if len(lidar.point_cloud) >= 3 else 0
                        
                        # Get GPS
                        gps = self.client.getGpsData(gps_name="Gps", vehicle_name=drone)
                        
                        role = "🎮 LEADER" if drone == self.leader else f"🛸 FOLLOWER"
                        print(f"\n[{drone}] {role}")
                        print(f"  Position: X={pos.x_val:.2f}m, Y={pos.y_val:.2f}m, Z={pos.z_val:.2f}m")
                        print(f"  GPS: {gps.gnss.geo_point.latitude:.6f}, {gps.gnss.geo_point.longitude:.6f}")
                        print(f"  Lidar: {num_points} points")
                        
                    except Exception as e:
                        print(f"\n[{drone}] ⚠️  Error: {e}")
                
                print("\n" + "-" * 80)
                print("W/A/S/D=Move | R/F=Up/Down | Q/E=Yaw | V=Lidar Viz | B=Formation Toggle")
                print("T=Takeoff | L=Land | SPACE=Stop | ESC=Exit")
                print("=" * 80)
                
            except Exception as e:
                pass
    
    def run(self):
        """Main run method"""
        print("\n" + "=" * 80)
        print("  🚁 5-DRONE FORMATION FLIGHT - IMPROVED CONTROLS")
        print("=" * 80)
        print("\n✨ NEW FEATURES:")
        print("  • Faster response (8 m/s speed, 45°/s yaw)")
        print("  • Formation flight (drones follow leader)")
        print("  • 3D Lidar visualization (press V)")
        print("  • Toggle formation on/off (press B)")
        print("\n🎮 CONTROLS:")
        print("  W/S/A/D  - Move forward/back/left/right")
        print("  R/F      - Up/Down")
        print("  Q/E      - Yaw left/right")
        print("  V        - Toggle lidar 3D visualization")
        print("  B        - Toggle formation following")
        print("  T        - Takeoff all")
        print("  L        - Land all")
        print("  SPACE    - Emergency stop")
        print("  ESC      - Exit")
        print("\n" + "=" * 80)
        print("\nPress 'T' to takeoff all drones in formation...\n")
        
        # Start keyboard listener
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        listener.start()
        
        # Start control threads
        leader_thread = Thread(target=self.leader_control_loop, daemon=True)
        leader_thread.start()
        
        formation_thread = Thread(target=self.formation_control_loop, daemon=True)
        formation_thread.start()
        
        sensor_thread = Thread(target=self.sensor_display_loop, daemon=True)
        sensor_thread.start()
        
        lidar_thread = Thread(target=self.lidar_visualization_loop, daemon=True)
        lidar_thread.start()
        
        # Keep main thread alive
        try:
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.running = False
        
        # Cleanup
        print("\n🔄 Cleaning up...")
        if self.flying:
            self.land_all()
        
        for drone in self.drones:
            self.client.enableApiControl(False, drone)
            self.client.armDisarm(False, drone)
        
        listener.stop()
        print("✅ Shutdown complete!")

if __name__ == "__main__":
    try:
        controller = FormationController()
        controller.run()
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
