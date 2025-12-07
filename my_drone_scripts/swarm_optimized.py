#!/usr/bin/env python3
"""
5-Drone Swarm - OPTIMIZED VERSION (No Camera Windows)
======================================================
Controls work smoothly, sensor data in terminal only
Camera windows removed to fix performance issues
"""

import airsim
import numpy as np
import time
import os
from pynput import keyboard
from threading import Thread, Lock
from collections import defaultdict

class SwarmController:
    def __init__(self):
        print("🚁 Initializing 5-Drone Swarm Controller...")
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        
        # Drone names
        self.drones = ["Drone1", "Drone2", "Drone3", "Drone4", "Drone5"]
        self.active_drone = "Drone1"
        
        # Enable API control for all drones
        for drone in self.drones:
            self.client.enableApiControl(True, drone)
            self.client.armDisarm(True, drone)
            print(f"✅ {drone} armed and ready")
        
        # Control state
        self.control_lock = Lock()
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0
        self.speed = 5.0
        self.yaw_speed = 30.0
        
        # Running flag
        self.running = True
        self.flying = False
        
        # Sensor data storage
        self.sensor_data = defaultdict(dict)
        
    def takeoff_all(self):
        """Takeoff all drones"""
        print("\n🛫 Taking off all drones...")
        tasks = []
        for drone in self.drones:
            print(f"   {drone} taking off...")
            tasks.append(self.client.takeoffAsync(timeout_sec=10, vehicle_name=drone))
        
        for task in tasks:
            task.join()
        
        self.flying = True
        print("✅ All drones airborne!\n")
        
    def land_all(self):
        """Land all drones"""
        print("\n🛬 Landing all drones...")
        tasks = []
        for drone in self.drones:
            print(f"   {drone} landing...")
            tasks.append(self.client.landAsync(timeout_sec=10, vehicle_name=drone))
        
        for task in tasks:
            task.join()
        
        self.flying = False
        print("✅ All drones landed!\n")
        
    def emergency_stop(self):
        """Emergency stop - hover in place"""
        print("\n⚠️  EMERGENCY STOP - All drones hovering!")
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
                        Thread(target=self.takeoff_all).start()
                    elif k == 'l' and self.flying:
                        Thread(target=self.land_all).start()
                        
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
            print(f"Key release error: {e}")
    
    def control_loop(self):
        """Main control loop - sends velocity commands"""
        print("🎮 Control loop started")
        loop_count = 0
        
        while self.running:
            if self.flying:
                with self.control_lock:
                    vx, vy, vz, yaw_rate = self.vx, self.vy, self.vz, self.yaw_rate
                
                # Send velocity command to active drone
                try:
                    self.client.moveByVelocityAsync(
                        vx, vy, vz, 1.0,
                        airsim.DrivetrainType.MaxDegreeOfFreedom,
                        airsim.YawMode(True, yaw_rate),
                        vehicle_name=self.active_drone
                    )
                except Exception as e:
                    if loop_count % 50 == 0:
                        print(f"⚠️  Control error: {e}")
                
                loop_count += 1
            
            time.sleep(0.1)
    
    def sensor_display_loop(self):
        """Display sensor data in terminal (lightweight)"""
        print("📊 Sensor display started")
        
        while self.running:
            time.sleep(2)  # Update every 2 seconds
            
            if not self.flying:
                continue
            
            try:
                os.system('clear' if os.name == 'posix' else 'cls')
                
                print("=" * 80)
                print("  🚁 5-DRONE SWARM - SENSOR DASHBOARD")
                print("=" * 80)
                print(f"Active Drone: {self.active_drone} | Status: {'✈️  AIRBORNE' if self.flying else '🛬 GROUNDED'}")
                print(f"Velocity: vx={self.vx:.1f}, vy={self.vy:.1f}, vz={self.vz:.1f}, yaw={self.yaw_rate:.1f}")
                print("-" * 80)
                
                # Get sensor data for all drones
                for drone in self.drones:
                    try:
                        # Position
                        state = self.client.getMultirotorState(vehicle_name=drone)
                        pos = state.kinematics_estimated.position
                        
                        # GPS
                        gps = self.client.getGpsData(gps_name="Gps", vehicle_name=drone)
                        
                        # Lidar
                        lidar = self.client.getLidarData(lidar_name="Lidar1", vehicle_name=drone)
                        num_points = len(lidar.point_cloud) // 3 if len(lidar.point_cloud) >= 3 else 0
                        
                        print(f"\n[{drone}]")
                        print(f"  Position: X={pos.x_val:.2f}m, Y={pos.y_val:.2f}m, Z={pos.z_val:.2f}m")
                        print(f"  GPS: Lat={gps.gnss.geo_point.latitude:.6f}, Lon={gps.gnss.geo_point.longitude:.6f}")
                        print(f"  Lidar: {num_points} points")
                        
                    except Exception as e:
                        print(f"\n[{drone}] ⚠️  Sensor error: {e}")
                
                print("\n" + "-" * 80)
                print("CONTROLS: W/A/S/D=Move | R/F=Up/Down | Q/E=Yaw | T=Takeoff | L=Land | SPACE=Stop | ESC=Exit")
                print("=" * 80)
                
            except Exception as e:
                print(f"Display error: {e}")
    
    def run(self):
        """Main run method"""
        print("\n" + "=" * 80)
        print("  🚁 5-DRONE SWARM CONTROL (OPTIMIZED)")
        print("=" * 80)
        print("\nKeyboard Controls:")
        print("  W/S      - Forward/Backward")
        print("  A/D      - Left/Right")
        print("  Q/E      - Yaw Left/Right")
        print("  R/F      - Up/Down")
        print("  T        - Takeoff all drones")
        print("  L        - Land all drones")
        print("  SPACE    - Emergency stop")
        print("  ESC      - Exit")
        print("\n" + "=" * 80)
        print("\nPress 'T' to takeoff all drones...\n")
        
        # Start keyboard listener
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        listener.start()
        
        # Start control thread
        control_thread = Thread(target=self.control_loop, daemon=True)
        control_thread.start()
        
        # Start sensor display thread
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
        controller = SwarmController()
        controller.run()
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
