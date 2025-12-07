#!/usr/bin/env python3
"""
SIMPLE TEST - Just one drone, keyboard control, NO camera windows
This will help us debug the movement issue
"""

import airsim
import time
from pynput import keyboard
from threading import Lock

class SimpleController:
    def __init__(self):
        print("🚁 Connecting to AirSim...")
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        print("✅ Connected!")
        
        # Enable control
        self.client.enableApiControl(True, "Drone1")
        self.client.armDisarm(True, "Drone1")
        print("✅ Drone1 armed")
        
        # Control state
        self.lock = Lock()
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0
        self.speed = 3.0  # Reduced speed for testing
        
        self.running = True
        self.flying = False
        
    def on_press(self, key):
        try:
            if hasattr(key, 'char') and key.char:
                k = key.char.lower()
                
                with self.lock:
                    if k == 'w':
                        self.vx = self.speed
                        print(f"⬆️  W pressed - Forward {self.speed} m/s")
                    elif k == 's':
                        self.vx = -self.speed
                        print(f"⬇️  S pressed - Backward {-self.speed} m/s")
                    elif k == 'a':
                        self.vy = -self.speed
                        print(f"⬅️  A pressed - Left {-self.speed} m/s")
                    elif k == 'd':
                        self.vy = self.speed
                        print(f"➡️  D pressed - Right {self.speed} m/s")
                    elif k == 'r':
                        self.vz = -self.speed
                        print(f"🔼 R pressed - Up {-self.speed} m/s")
                    elif k == 'f':
                        self.vz = self.speed
                        print(f"🔽 F pressed - Down {self.speed} m/s")
                    elif k == 't':
                        print("🛫 T pressed - Taking off...")
                        self.client.takeoffAsync().join()
                        self.flying = True
                        print("✅ Airborne!")
                    elif k == 'l':
                        print("🛬 L pressed - Landing...")
                        self.flying = False
                        self.client.landAsync().join()
                        print("✅ Landed!")
                        
            if key == keyboard.Key.esc:
                print("🔚 ESC pressed - Exiting...")
                self.running = False
                
        except Exception as e:
            print(f"❌ Key press error: {e}")
    
    def on_release(self, key):
        try:
            if hasattr(key, 'char') and key.char:
                k = key.char.lower()
                
                with self.lock:
                    if k in ['w', 's']:
                        self.vx = 0
                        print("⏹️  Released W/S - Stopping forward/back")
                    elif k in ['a', 'd']:
                        self.vy = 0
                        print("⏹️  Released A/D - Stopping left/right")
                    elif k in ['r', 'f']:
                        self.vz = 0
                        print("⏹️  Released R/F - Stopping up/down")
                        
        except Exception as e:
            print(f"❌ Key release error: {e}")
    
    def run(self):
        print("\n" + "="*60)
        print("  🚁 SIMPLE DRONE TEST - Keyboard Control")
        print("="*60)
        print("\nControls:")
        print("  T - Takeoff")
        print("  W/S - Forward/Backward")
        print("  A/D - Left/Right")
        print("  R/F - Up/Down")
        print("  L - Land")
        print("  ESC - Exit")
        print("\nPress 'T' to takeoff first!")
        print("="*60 + "\n")
        
        # Start keyboard listener
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()
        
        # Main control loop
        print("🎮 Control loop running...")
        loop_count = 0
        
        while self.running:
            if self.flying:
                with self.lock:
                    vx, vy, vz = self.vx, self.vy, self.vz
                
                # Print velocities every 2 seconds if moving
                if loop_count % 20 == 0 and (vx != 0 or vy != 0 or vz != 0):
                    print(f"📍 Current velocity: vx={vx:.1f}, vy={vy:.1f}, vz={vz:.1f}")
                    
                    # Get position
                    state = self.client.getMultirotorState()
                    pos = state.kinematics_estimated.position
                    print(f"📍 Position: X={pos.x_val:.1f}m, Y={pos.y_val:.1f}m, Z={pos.z_val:.1f}m")
                
                # Send velocity command
                try:
                    self.client.moveByVelocityAsync(
                        vx, vy, vz, 1.0,
                        vehicle_name="Drone1"
                    )
                except Exception as e:
                    print(f"⚠️  Movement error: {e}")
                
                loop_count += 1
            
            time.sleep(0.1)  # 10Hz
        
        # Cleanup
        print("\n🔄 Cleaning up...")
        if self.flying:
            print("Landing...")
            self.client.landAsync().join()
        
        self.client.enableApiControl(False)
        self.client.armDisarm(False)
        listener.stop()
        print("✅ Done!")

if __name__ == "__main__":
    try:
        controller = SimpleController()
        controller.run()
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
