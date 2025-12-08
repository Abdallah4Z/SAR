import airsim
import numpy as np
import time
import math
import threading
from path_planning import PathPlanner
from task_allocator import TaskAllocator

class DroneSwarmSimulation:
    def __init__(self, spawn_offset=(0, 0, 0)):
        # Connect to AirSim
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        print("Connected to AirSim")
        
        self.spawn_offset = spawn_offset
        self.path_planner = PathPlanner(grid_size=100, alpha=0.5)
        self.task_allocator = TaskAllocator(w1=1.0, w2=0.5, w3=2.0, min_battery_threshold=15.0)
        
        base_x, base_y, base_z = spawn_offset
        
        # Separate AirSim clients for each drone (avoids IOLoop conflicts)
        self.drone_clients = {}
        for i in range(1, 5):
            self.drone_clients[f'Drone{i}'] = airsim.MultirotorClient()
            self.drone_clients[f'Drone{i}'].confirmConnection()
        
        self.drones = {
            'Drone1': {
                'position': (base_x + 0, base_y + 0, base_z + 0),
                'battery': 30.0,  # LOW battery - will complete fewer tasks
                'max_speed': 10.0,
                'payload_capacity': 2.0,
                'energy_rate': 0.15,
                'status': 'Idle',
                'home_position': (base_x + 0, base_y + 0, base_z + 0),
                'battery_capacity': 100.0,
                'distance_traveled': 0.0,
                'energy_used': 0.0,
                'tasks_completed': 0,
                'returns_to_home': 0,
                'is_busy': False,
                'current_task_id': None,
                'lock': threading.Lock()
            },
            'Drone2': {
                'position': (base_x + 10, base_y + 0, base_z + 0),
                'battery': 60.0,  # MEDIUM battery
                'max_speed': 10.0,
                'payload_capacity': 2.5,
                'energy_rate': 0.12,
                'status': 'Idle',
                'home_position': (base_x + 10, base_y + 0, base_z + 0),
                'battery_capacity': 100.0,
                'distance_traveled': 0.0,
                'energy_used': 0.0,
                'tasks_completed': 0,
                'returns_to_home': 0,
                'is_busy': False,
                'current_task_id': None,
                'lock': threading.Lock()
            },
            'Drone3': {
                'position': (base_x + 0, base_y + 10, base_z + 0),
                'battery': 80.0,  # HIGH battery
                'max_speed': 12.0,
                'payload_capacity': 1.5,
                'energy_rate': 0.10,
                'status': 'Idle',
                'home_position': (base_x + 0, base_y + 10, base_z + 0),
                'battery_capacity': 100.0,
                'distance_traveled': 0.0,
                'energy_used': 0.0,
                'tasks_completed': 0,
                'returns_to_home': 0,
                'is_busy': False,
                'current_task_id': None,
                'lock': threading.Lock()
            },
            'Drone4': {
                'position': (base_x + 10, base_y + 10, base_z + 0),
                'battery': 100.0,  # FULL battery - best performer
                'max_speed': 11.0,
                'payload_capacity': 3.0,
                'energy_rate': 0.14,
                'status': 'Idle',
                'home_position': (base_x + 10, base_y + 10, base_z + 0),
                'battery_capacity': 100.0,
                'distance_traveled': 0.0,
                'energy_used': 0.0,
                'tasks_completed': 0,
                'returns_to_home': 0,
                'is_busy': False,
                'current_task_id': None,
                'lock': threading.Lock()
            }
        }
        
        self.tasks = [
            {'task_id': 'T1', 'goal_position': (base_x + 50, base_y + 50, base_z - 10), 'importance': 8, 'payload': 1.5, 'status': 'Pending', 'assigned_to': None},
            {'task_id': 'T2', 'goal_position': (base_x + 30, base_y + 70, base_z - 15), 'importance': 10, 'payload': 2.0, 'status': 'Pending', 'assigned_to': None},
            {'task_id': 'T3', 'goal_position': (base_x + 80, base_y + 20, base_z - 12), 'importance': 5, 'payload': 1.0, 'status': 'Pending', 'assigned_to': None},
            {'task_id': 'T4', 'goal_position': (base_x + 60, base_y + 40, base_z - 8), 'importance': 7, 'payload': 1.8, 'status': 'Pending', 'assigned_to': None},
            {'task_id': 'T5', 'goal_position': (base_x + 20, base_y + 90, base_z - 20), 'importance': 9, 'payload': 2.5, 'status': 'Pending', 'assigned_to': None},
            {'task_id': 'T6', 'goal_position': (base_x + 90, base_y + 80, base_z - 18), 'importance': 6, 'payload': 1.2, 'status': 'Pending', 'assigned_to': None},
            {'task_id': 'T7', 'goal_position': (base_x + 40, base_y + 30, base_z - 10), 'importance': 4, 'payload': 0.8, 'status': 'Pending', 'assigned_to': None},
            {'task_id': 'T8', 'goal_position': (base_x + 70, base_y + 60, base_z - 14), 'importance': 8, 'payload': 2.2, 'status': 'Pending', 'assigned_to': None},
            {'task_id': 'T9', 'goal_position': (base_x + 25, base_y + 45, base_z - 16), 'importance': 7, 'payload': 1.5, 'status': 'Pending', 'assigned_to': None},
            {'task_id': 'T10', 'goal_position': (base_x + 85, base_y + 35, base_z - 11), 'importance': 5, 'payload': 1.3, 'status': 'Pending', 'assigned_to': None}
        ]
        
        self.task_lock = threading.Lock()
        self.active_threads = []
        self.start_time = None

    def initialize_drones(self):
        """Enable API control and arm all drones"""
        print("\nInitializing drones...")
        for drone_name in self.drones.keys():
            try:
                client = self.drone_clients[drone_name]
                client.enableApiControl(True, drone_name)
                client.armDisarm(True, drone_name)
                drone = self.drones[drone_name]
                print(f"  {drone_name} ready | Battery: {drone['battery']:.0f}% | Speed: {drone['max_speed']} m/s")
            except Exception as e:
                print(f"  [ERROR] {drone_name}: {e}")
        time.sleep(2)

    def takeoff_all(self):
        """Takeoff all drones simultaneously"""
        print("\nTaking off all drones...")
        
        threads = []
        for drone_name in self.drones.keys():
            thread = threading.Thread(target=self._takeoff_single, args=(drone_name,))
            thread.start()
            threads.append(thread)
        
        for thread in threads:
            thread.join()
        
        time.sleep(2)
        print("All drones airborne")

    def _takeoff_single(self, drone_name):
        """Takeoff single drone"""
        try:
            client = self.drone_clients[drone_name]
            client.takeoffAsync(vehicle_name=drone_name).join()
            print(f"  {drone_name} airborne")
        except Exception as e:
            print(f"  [ERROR] {drone_name} takeoff: {e}")

    def update_drone_position(self, drone_name):
        """Update drone position from AirSim"""
        try:
            client = self.drone_clients[drone_name]
            state = client.getMultirotorState(vehicle_name=drone_name)
            pos = state.kinematics_estimated.position
            
            drone = self.drones[drone_name]
            with drone['lock']:
                drone['position'] = (pos.x_val, pos.y_val, pos.z_val)
        except Exception as e:
            pass

    def execute_task_parallel(self, drone_name, task):
        """Execute a single task (runs in separate thread)"""
        drone = self.drones[drone_name]
        client = self.drone_clients[drone_name]
        
        with drone['lock']:
            drone['is_busy'] = True
            drone['status'] = 'Flying'
            drone['current_task_id'] = task['task_id']
        
        # Mark task as assigned IMMEDIATELY to prevent duplicates
        with self.task_lock:
            task['assigned_to'] = drone_name
            task['status'] = 'InProgress'
        
        print(f"\n[{drone_name}] >>> Task {task['task_id']} START")
        
        # Get current state
        with drone['lock']:
            battery = drone['battery']
            position = drone['position']
        
        print(f"  Battery: {battery:.1f}% | From: ({position[0]:.1f}, {position[1]:.1f}, {position[2]:.1f})")
        print(f"  To: ({task['goal_position'][0]:.1f}, {task['goal_position'][1]:.1f}, {task['goal_position'][2]:.1f})")
        
        # Plan path
        path, energy = self.path_planner.plan(
            position,
            task['goal_position'],
            drone['energy_rate']
        )
        
        print(f"  Path: {len(path)} waypoints | Energy needed: {energy:.2f}%")
        
        # Check battery
        if energy > battery:
            print(f"  [ABORT] Insufficient battery (need {energy:.1f}%, have {battery:.1f}%)")
            with drone['lock']:
                drone['is_busy'] = False
                drone['status'] = 'Idle'
                drone['current_task_id'] = None
            with self.task_lock:
                task['status'] = 'Pending'
                task['assigned_to'] = None
            return False
        
        # Simplify path (every 15th waypoint)
        simplified = path[::15]
        if path[-1] not in simplified:
            simplified.append(path[-1])
        
        # Execute movement
        for i, wp in enumerate(simplified):
            # Check critical battery level
            if battery <= 5.0:
                print(f"  [CRITICAL] {drone_name} battery at {battery:.1f}% - ABORTING!")
                with drone['lock']:
                    drone['is_busy'] = False
                    drone['status'] = 'LowBattery'
                    drone['current_task_id'] = None
                with self.task_lock:
                    task['status'] = 'Pending'
                    task['assigned_to'] = None
                return False
            
            try:
                client.moveToPositionAsync(
                    wp[0], wp[1], wp[2],
                    drone['max_speed'],
                    vehicle_name=drone_name
                ).join()
                
                prev_pos = position
                self.update_drone_position(drone_name)
                
                with drone['lock']:
                    position = drone['position']
                    dist = math.dist(prev_pos, position)
                    
                    # Correct energy calculation
                    energy_consumed = (dist * drone['energy_rate'] / drone['battery_capacity']) * 100
                    
                    battery -= energy_consumed
                    drone['battery'] = battery
                    drone['distance_traveled'] += dist
                    drone['energy_used'] += energy_consumed
                
                if (i + 1) % 2 == 0 or i == 0:
                    print(f"  [{drone_name}] Progress: {i+1}/{len(simplified)} | Battery: {battery:.1f}%")
                
            except Exception as e:
                print(f"  [ERROR] {drone_name}: {e}")
                with drone['lock']:
                    drone['is_busy'] = False
                    drone['status'] = 'Idle'
                    drone['current_task_id'] = None
                with self.task_lock:
                    task['status'] = 'Pending'
                    task['assigned_to'] = None
                return False
        
        # Task completed
        print(f"[{drone_name}] <<< Task {task['task_id']} COMPLETED! Battery remaining: {battery:.1f}%")
        
        with drone['lock']:
            drone['tasks_completed'] += 1
            drone['is_busy'] = False
            drone['status'] = 'Idle'
            drone['current_task_id'] = None
        
        with self.task_lock:
            task['status'] = 'Completed'
        
        return True

    def return_all_to_home(self):
        """Return all drones to home positions in parallel - ONLY AT END"""
        print("\n" + "="*70)
        print(" ALL TASKS COMPLETED - RETURNING ALL DRONES TO HOME")
        print("="*70)
        
        threads = []
        for drone_name in self.drones.keys():
            # Only return drones that have enough battery
            if self.drones[drone_name]['battery'] > 5.0:
                thread = threading.Thread(target=self._return_home_single, args=(drone_name,))
                thread.start()
                threads.append(thread)
            else:
                print(f"[{drone_name}] Battery too low ({self.drones[drone_name]['battery']:.1f}%) - staying at current location")
        
        for thread in threads:
            thread.join()
        
        print("\n" + "="*70)
        print(" RETURN HOME COMPLETE")
        print("="*70)

    def _return_home_single(self, drone_name):
        """Return single drone to home position"""
        drone = self.drones[drone_name]
        client = self.drone_clients[drone_name]
        
        with drone['lock']:
            current_pos = drone['position']
            home_pos = drone['home_position']
            battery = drone['battery']
        
        print(f"\n[{drone_name}] Returning to home...")
        print(f"  From: ({current_pos[0]:.1f}, {current_pos[1]:.1f}, {current_pos[2]:.1f})")
        print(f"  To: ({home_pos[0]:.1f}, {home_pos[1]:.1f}, {home_pos[2]:.1f})")
        
        # Check if enough battery to return
        distance_home = math.dist(current_pos, home_pos)
        energy_needed = (distance_home * drone['energy_rate'] / drone['battery_capacity']) * 100
        
        if energy_needed > battery:
            print(f"  [WARNING] Insufficient battery to return home (need {energy_needed:.1f}%, have {battery:.1f}%)")
            print(f"  [INFO] Drone will stay at current location")
            return
        
        # Plan path back home
        path, energy = self.path_planner.plan(
            current_pos,
            home_pos,
            drone['energy_rate']
        )
        
        # Simplify path
        simplified = path[::15]
        if path[-1] not in simplified:
            simplified.append(path[-1])
        
        # Fly back home
        for i, wp in enumerate(simplified):
            if battery <= 2.0:
                print(f"  [{drone_name}] Critical battery during return - stopping!")
                break
            
            try:
                client.moveToPositionAsync(
                    wp[0], wp[1], wp[2],
                    drone['max_speed'],
                    vehicle_name=drone_name
                ).join()
                
                prev_pos = current_pos
                self.update_drone_position(drone_name)
                
                with drone['lock']:
                    current_pos = drone['position']
                    dist = math.dist(prev_pos, current_pos)
                    
                    # Correct energy calculation
                    energy_consumed = (dist * drone['energy_rate'] / drone['battery_capacity']) * 100
                    
                    battery -= energy_consumed
                    drone['battery'] = battery
                    drone['distance_traveled'] += dist
                    drone['energy_used'] += energy_consumed
                
                if (i + 1) % 2 == 0:
                    print(f"  [{drone_name}] Return progress: {i+1}/{len(simplified)}")
                    
            except Exception as e:
                print(f"  [{drone_name}] Return error: {e}")
                break
        
        with drone['lock']:
            drone['returns_to_home'] += 1
            drone['position'] = home_pos
        
        print(f"[{drone_name}] Arrived at home | Battery: {battery:.1f}%")

    def get_pending_tasks(self):
        """Get truly pending tasks (not assigned or in progress)"""
        with self.task_lock:
            return [t for t in self.tasks if t['status'] == 'Pending' and t['assigned_to'] is None]

    def run_simulation_parallel(self):
        """Main simulation loop with TRUE parallel execution"""
        print("="*70)
        print(" ENERGY-AWARE MULTI-DRONE TASK ALLOCATION SIMULATION")
        print(" TRUE PARALLEL EXECUTION MODE")
        print(" Different Battery Levels per Drone")
        print(" NO Auto-Recharge - Realistic Battery Constraints")
        print(" Drones return home ONLY after all tasks complete")
        print("="*70)
        
        self.initialize_drones()
        self.takeoff_all()
        
        self.start_time = time.time()
        cycle = 0
        
        while True:
            cycle += 1
            
            # Get truly pending tasks
            pending = self.get_pending_tasks()
            
            # Check available drones (not low battery)
            available_drones = [name for name, d in self.drones.items() 
                               if not d['is_busy'] and d['battery'] > 15.0]
            
            # Check if simulation is complete
            if not pending or not available_drones:
                busy_drones = [name for name, d in self.drones.items() if d['is_busy']]
                if not busy_drones:
                    elapsed = time.time() - self.start_time
                    
                    # Check if there are still pending tasks
                    remaining = self.get_pending_tasks()
                    if remaining:
                        print("\n" + "="*70)
                        print(f" SIMULATION STOPPED - {len(remaining)} TASKS REMAINING")
                        print(" All drones have insufficient battery to continue")
                        print(f" Time elapsed: {elapsed:.1f} seconds ({elapsed/60:.1f} minutes)")
                        print("="*70)
                        
                        print("\nRemaining tasks:")
                        for task in remaining:
                            print(f"  - {task['task_id']} (Importance: {task['importance']})")
                    else:
                        print("\n" + "="*70)
                        print(f" ALL TASKS COMPLETED in {elapsed:.1f} seconds ({elapsed/60:.1f} minutes)")
                        print("="*70)
                    break
            
            print(f"\n{'='*70}")
            print(f" CYCLE {cycle} | Pending: {len(pending)} | Available Drones: {len(available_drones)}")
            
            # Show active drones and their tasks
            busy_drones = []
            for name, d in self.drones.items():
                if d['is_busy']:
                    busy_drones.append(f"{name}({d['current_task_id']}, {d['battery']:.0f}%)")
            
            if busy_drones:
                print(f" Active: {', '.join(busy_drones)}")
            
            # Show low battery drones
            low_battery = [f"{name}({d['battery']:.0f}%)" for name, d in self.drones.items() 
                          if not d['is_busy'] and d['battery'] <= 15.0]
            if low_battery:
                print(f" Low Battery (Idle): {', '.join(low_battery)}")
            
            print(f"{'='*70}")
            
            # Clean up finished threads
            self.active_threads = [t for t in self.active_threads if t.is_alive()]
            
            # Allocate new tasks to idle drones
            if pending and available_drones:
                assignments = self.task_allocator.assign_tasks(
                    self.drones,
                    pending,
                    self.path_planner
                )
                
                if assignments:
                    print("\nNEW ASSIGNMENTS:")
                    for name, task in assignments.items():
                        # Double-check task isn't already assigned
                        with self.task_lock:
                            if task['assigned_to'] is not None:
                                print(f"  [SKIP] {name} -> {task['task_id']} (already assigned to {task['assigned_to']})")
                                continue
                        
                        print(f"  {name} -> {task['task_id']} (Importance: {task['importance']}, Payload: {task['payload']}kg)")
                        
                        # Start task in new thread
                        thread = threading.Thread(
                            target=self.execute_task_parallel,
                            args=(name, task)
                        )
                        thread.daemon = True
                        thread.start()
                        self.active_threads.append(thread)
            
            # Sleep between cycles
            time.sleep(1)
        
        # Wait for all threads to complete
        print("\nWaiting for all drones to finish their final tasks...")
        for thread in self.active_threads:
            thread.join(timeout=30)
        
        # NOW return all drones to home (if they have battery)
        self.return_all_to_home()
        
        # Print final report
        self.print_final_report()

    def print_final_report(self):
        """Print simulation summary"""
        elapsed = time.time() - self.start_time
        
        print("\n" + "="*70)
        print(" SIMULATION COMPLETE - FINAL REPORT")
        print(f" Total Time: {elapsed:.1f} seconds ({elapsed/60:.1f} minutes)")
        print("="*70)
        
        total_distance = 0
        total_energy = 0
        total_tasks = 0
        
        for drone_name, drone in self.drones.items():
            print(f"\n{drone_name}:")
            print(f"  Initial Battery: {20 if drone_name == 'Drone1' else 60 if drone_name == 'Drone2' else 80 if drone_name == 'Drone3' else 100}%")
            print(f"  Final Battery: {drone['battery']:.1f}%")
            print(f"  Battery Used: {drone['energy_used']:.2f}%")
            print(f"  Tasks Completed: {drone['tasks_completed']}")
            print(f"  Distance Traveled: {drone['distance_traveled']:.2f} m")
            print(f"  Final Position: ({drone['position'][0]:.1f}, {drone['position'][1]:.1f}, {drone['position'][2]:.1f})")
            print(f"  Home Position: ({drone['home_position'][0]:.1f}, {drone['home_position'][1]:.1f}, {drone['home_position'][2]:.1f})")
            print(f"  Status: {drone['status']}")
            
            total_distance += drone['distance_traveled']
            total_energy += drone['energy_used']
            total_tasks += drone['tasks_completed']
        
        completed = sum(1 for t in self.tasks if t['status'] == 'Completed')
        pending = sum(1 for t in self.tasks if t['status'] == 'Pending')
        
        print(f"\n{'='*70}")
        print(f"TOTALS:")
        print(f"  Execution Time: {elapsed:.1f}s ({elapsed/60:.1f} min)")
        print(f"  Tasks Completed: {completed}/{len(self.tasks)}")
        
        if pending > 0:
            print(f"  Tasks Remaining: {pending} (insufficient battery)")
        
        print(f"  Total Distance: {total_distance:.2f} m")
        print(f"  Total Energy Used: {total_energy:.2f}%")
        print(f"  Average Tasks/Drone: {total_tasks/len(self.drones):.1f}")
        
        if completed > 0:
            print(f"  Average Time/Task: {elapsed/completed:.1f}s")
        
        print("="*70)

    def cleanup(self):
        """Land all drones at their positions and cleanup"""
        print("\n\nLanding all drones...")
        
        threads = []
        for drone_name in self.drones.keys():
            thread = threading.Thread(target=self._land_single, args=(drone_name,))
            thread.start()
            threads.append(thread)
        
        for thread in threads:
            thread.join()
        
        time.sleep(2)
        
        for drone_name, client in self.drone_clients.items():
            try:
                client.armDisarm(False, drone_name)
                client.enableApiControl(False, drone_name)
            except:
                pass
        
        print("Shutdown complete")

    def _land_single(self, drone_name):
        """Land single drone"""
        try:
            client = self.drone_clients[drone_name]
            client.landAsync(vehicle_name=drone_name).join()
            print(f"  {drone_name} landed")
        except Exception as e:
            print(f"  [ERROR] {drone_name} landing: {e}")


if __name__ == "__main__":
    SPAWN_OFFSET = (0, 0, 0)
    
    sim = DroneSwarmSimulation(spawn_offset=SPAWN_OFFSET)
    
    try:
        sim.run_simulation_parallel()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        sim.cleanup()