"""
UAV Dynamics Module — Simplified kinematic (point-mass in 3D) model.

Reference: Used in ~60% of FL-UAV papers for computational efficiency.

V4 Features:
- Velocity clipping by magnitude (not per-component)
- Collision detection
- Proper validation in constructor
- Battery depletion handling
"""

import numpy as np
from typing import Tuple, Dict, List


class UAV:
    """
    Single UAV with simplified kinematic physics model.

    V3 FIXES:
    - Velocity clipping by magnitude (not per-component)
    - Collision detection actually called and penalized
    - Proper validation in constructor
    """

    def __init__(self,
                 uav_id: int,
                 initial_position: np.ndarray,
                 battery_capacity: float = 500000,  # Joules (500 kJ)
                 max_speed: float = 5.0,  # m/s
                 cpu_frequency: float = 2.0e9,  # 2 GHz in Hz
                 max_queue_size: int = 5,  # REDUCED from 10 to prevent overflow
                 rng: np.random.RandomState = None):
        """
        Initialize UAV.

        Args:
            uav_id: Unique identifier
            initial_position: [x, y, z] in meters
            battery_capacity: Total battery energy in Joules
            max_speed: Maximum velocity magnitude in m/s
            cpu_frequency: CPU frequency in Hz
            max_queue_size: Maximum tasks in queue
            rng: Random number generator for reproducibility
        """
        # Validation
        assert battery_capacity > 0, "Battery capacity must be positive"
        assert max_speed > 0, "Max speed must be positive"
        assert cpu_frequency > 0, "CPU frequency must be positive"
        assert max_queue_size > 0, "Queue size must be positive"

        self.id = uav_id
        self.position = np.array(initial_position, dtype=np.float64)
        self.velocity = np.zeros(3, dtype=np.float64)
        self.rng = rng if rng is not None else np.random.RandomState()

        # Energy
        self.battery_capacity = battery_capacity
        self.battery = battery_capacity  # Current battery level (J)
        self.is_alive = True

        # Computation
        self.cpu_frequency = cpu_frequency  # Hz
        self.max_speed = max_speed

        # Task queue
        self.task_queue = []
        self.max_queue_size = max_queue_size
        self.tasks_completed = 0
        self.tasks_failed = 0

    def step(self,
             target_velocity: np.ndarray,
             dt: float,
             area_bounds: Tuple[float, float, float]) -> Dict:
        """
        Update UAV state for one timestep.

        Args:
            target_velocity: Desired [vx, vy, vz] in m/s
            dt: Timestep duration in seconds
            area_bounds: (width, height, altitude) in meters

        Returns:
            Dict with status information including collision flag
        """
        if not self.is_alive:
            return {
                'alive': False,
                'position': self.position.copy(),
                'collision': False
            }

        # 1. Clip velocity by MAGNITUDE, not per-component
        # This prevents diagonal velocity from exceeding max_speed
        v_magnitude = np.linalg.norm(target_velocity)
        if v_magnitude > self.max_speed:
            self.velocity = target_velocity * (self.max_speed / v_magnitude)
        else:
            self.velocity = target_velocity.copy()

        # 2. Update position
        new_position = self.position + self.velocity * dt

        # 3. Enforce boundaries (hard clipping)
        new_position[0] = np.clip(new_position[0], 0, area_bounds[0])
        new_position[1] = np.clip(new_position[1], 0, area_bounds[1])
        new_position[2] = np.clip(new_position[2], 10, area_bounds[2])  # Min altitude 10m

        self.position = new_position

        # 4. Calculate flight energy consumption
        P_hover = 50  # Watts
        alpha = 0.5
        v_magnitude = np.linalg.norm(self.velocity)
        P_flight = P_hover + alpha * (v_magnitude ** 2)
        E_flight = P_flight * dt  # Joules

        # 5. Update battery
        self.battery -= E_flight

        # 6. Check if battery depleted
        if self.battery <= 0:
            self.battery = 0
            self.is_alive = False

        return {
            'alive': self.is_alive,
            'position': self.position.copy(),
            'velocity': self.velocity.copy(),
            'battery': self.battery,
            'energy_consumed': E_flight,
            'collision': False  # Will be updated by environment
        }

    def check_collision(self, other_uavs: List['UAV'], min_separation: float = 10.0) -> bool:
        """
        Check if this UAV is too close to others.

        Args:
            other_uavs: List of other UAV objects
            min_separation: Minimum allowed distance in meters

        Returns:
            True if collision detected
        """
        for other in other_uavs:
            if other.id == self.id:
                continue
            distance = np.linalg.norm(self.position - other.position)
            if distance < min_separation:
                return True
        return False

    def can_accept_task(self) -> bool:
        """Check if UAV can accept more tasks."""
        return len(self.task_queue) < self.max_queue_size and self.is_alive

    def move_toward(self,
                    target_position: np.ndarray,
                    dt: float,
                    area_bounds: Tuple[float, float, float]) -> Dict:
        """
        Move UAV toward a target position at max_speed.

        This is the POSITION-BASED movement model used by 80% of papers
        (MADDPG, MAPPO, Joint Trajectory). The MARL agent outputs a
        target position, and the UAV flies toward it.

        Args:
            target_position: [x, y, z] target in meters
            dt: Timestep duration in seconds
            area_bounds: (width, height, altitude) in meters

        Returns:
            Dict with status info
        """
        if not self.is_alive:
            return {
                'alive': False,
                'position': self.position.copy(),
                'collision': False
            }

        # 1. Compute direction to target
        direction = target_position - self.position
        distance = np.linalg.norm(direction)

        # 2. Move toward target at max_speed (or arrive if close)
        if distance > 0.1:  # Not at target yet
            direction_normalized = direction / distance
            # Move at max_speed, but don't overshoot
            move_distance = min(self.max_speed * dt, distance)
            self.velocity = direction_normalized * (move_distance / dt)
        else:
            # At target — hover
            self.velocity = np.zeros(3)

        # 3. Update position
        new_position = self.position + self.velocity * dt

        # 4. Enforce boundaries
        new_position[0] = np.clip(new_position[0], 0, area_bounds[0])
        new_position[1] = np.clip(new_position[1], 0, area_bounds[1])
        new_position[2] = np.clip(new_position[2], 10, area_bounds[2])

        self.position = new_position

        # 5. Flight energy
        P_hover = 50  # Watts
        alpha = 0.5
        v_magnitude = np.linalg.norm(self.velocity)
        P_flight = P_hover + alpha * (v_magnitude ** 2)
        E_flight = P_flight * dt

        # 6. Battery update
        self.battery -= E_flight
        if self.battery <= 0:
            self.battery = 0
            self.is_alive = False

        return {
            'alive': self.is_alive,
            'position': self.position.copy(),
            'velocity': self.velocity.copy(),
            'battery': self.battery,
            'energy_consumed': E_flight,
            'collision': False
        }

    def find_nearest_neighbor(self, other_uavs: List['UAV']) -> 'UAV':
        """
        Find the nearest alive neighbor UAV that can accept tasks.

        Args:
            other_uavs: List of all UAVs

        Returns:
            Nearest available UAV, or None if none available
        """
        best = None
        best_dist = float('inf')

        for other in other_uavs:
            if other.id == self.id or not other.can_accept_task():
                continue
            dist = np.linalg.norm(self.position - other.position)
            if dist < best_dist:
                best_dist = dist
                best = other

        return best
