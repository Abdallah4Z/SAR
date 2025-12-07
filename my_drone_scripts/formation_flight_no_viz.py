#!/usr/bin/env python3
"""
5-Drone Formation Flight with Improved Controls (No 3D Visualization)
======================================================================
- Drone1: Keyboard controlled (leader)
- Drones 2-5: Follow in formation
- Lidar data logging to terminal
- Improved steering responsiveness
- Use lidar_logger.py separately for point cloud visualization
"""

import airsim
import time
import os
import sys
from pynput import keyboard
from threading import Thread, Lock

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
        
        # Wait for AirSim to be fully ready
        print("⏳ Waiting for AirSim to be ready...")
        time.sleep(3)  # Give AirSim time to initialize
        
        # Enable API control for all drones
        for drone in self.drones:
            try:
                self.client.enableApiControl(True, drone)
                self.client.armDisarm(True, drone)
                print(f"✅ {drone} armed and ready")
            except Exception as e:
                print(f"❌ Failed to connect to {drone}: {e}")
                print("Make sure AirSim is running and fully loaded!")
                sys.exit(1)
        
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
        
    def takeoff_all(self):
        """Takeoff all drones - FAST VERSION"""
        print("\n🛫 Fast takeoff all drones...")
        tasks = []
        # Start all takeoffs simultaneously
        for drone in self.drones:
            tasks.append(self.client.takeoffAsync(timeout_sec=3, vehicle_name=drone))
        
        # Wait for all to complete
        for task in tasks:
            task.join()
        
        self.flying = True
        print("✅ All drones airborne in 5 seconds!\n")
        
    def land_all(self):
        """Land all drones - FAST VERSION"""
        print("\n🛬 Fast landing all drones...")
        tasks = []
        for drone in self.drones:
            tasks.append(self.client.landAsync(timeout_sec=5, vehicle_name=drone))
        
        for task in tasks:
            task.join()
        
        self.flying = False
        print("✅ All drones landed in 5 seconds!\n")
        
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
                print(f"Leader: {self.leader} | Formation: {'✅' if self.formation_enabled else '❌'}")
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
                        
                        # Show lidar distances if points available
                        if num_points > 0:
                            import numpy as np
                            points = np.array(lidar.point_cloud, dtype=np.float32).reshape(-1, 3)
                            distances = np.linalg.norm(points, axis=1)
                            print(f"  Lidar Range: min={distances.min():.1f}m, avg={distances.mean():.1f}m, max={distances.max():.1f}m")
                        
                    except Exception as e:
                        print(f"\n[{drone}] ⚠️  Error: {e}")
                
                print("\n" + "-" * 80)
                print("W/A/S/D=Move | R/F=Up/Down | Q/E=Yaw | B=Formation Toggle")
                print("T=Takeoff | L=Land | SPACE=Stop | ESC=Exit")
                print("\n💡 TIP: Run 'python3 lidar_logger.py' in another terminal to save lidar data")
                print("=" * 80)
                
            except Exception as e:
                pass
    
    def run(self):
        """Main run method"""
        print("\n" + "=" * 80)
        print("  🚁 5-DRONE FORMATION FLIGHT - IMPROVED CONTROLS")
        print("=" * 80)
        print("\n✨ FEATURES:")
        print("  • Faster response (8 m/s speed, 45°/s yaw)")
        print("  • Formation flight (drones follow leader)")
        print("  • Real-time lidar distance stats")
        print("  • Toggle formation on/off (press B)")
        print("\n💡 For 3D lidar visualization, run in separate terminal:")
        print("   python3 lidar_logger.py")
        print("\n🎮 CONTROLS:")
        print("  W/S/A/D  - Move forward/back/left/right")
        print("  R/F      - Up/Down")
        print("  Q/E      - Yaw left/right")
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
