#!/usr/bin/env python3

import numpy as np

class APFController:
    def __init__(self):
        # APF parameters
        self.k_att = 1.5          # Attractive force gain
        self.k_rep = 8.0          # Repulsive force gain
        self.d0 = 5.0             # Obstacle influence distance
        self.min_force = 0.1      # Minimum force magnitude
        
        # Local minima escape
        self.stuck_counter = 0
        self.last_position = None
        self.stuck_threshold = 15  # If stuck for 15 iterations
        self.random_force_gain = 3.0
        
    def calculate_force(self, current, goal, obstacles):
        """
        Calculate total force with local minima escape
        
        current: [x, y]
        goal: [x, y]
        obstacles: list of [x, y] positions or dicts with 'position' key
        """
        current = np.array(current, dtype=float)
        goal = np.array(goal, dtype=float)
        
        # Check if stuck (not moving)
        if self.last_position is not None:
            movement = np.linalg.norm(current - self.last_position)
            if movement < 0.2:  # Moving less than 0.2m
                self.stuck_counter += 1
            else:
                self.stuck_counter = 0
        
        self.last_position = current.copy()
        
        # === ATTRACTIVE FORCE ===
        diff = goal - current
        dist_to_goal = np.linalg.norm(diff)
        
        if dist_to_goal > 0.1:
            f_att = self.k_att * (diff / dist_to_goal)
        else:
            f_att = np.zeros(2)
        
        # === REPULSIVE FORCE ===
        f_rep = np.zeros(2)
        
        for obs in obstacles:
            # Handle both dict and array formats
            if isinstance(obs, dict):
                obs_pos = np.array(obs['position'], dtype=float)
                obs_radius = obs.get('radius', 2.0)
            else:
                obs_pos = np.array(obs, dtype=float)
                obs_radius = 2.0
            
            diff_obs = current - obs_pos
            dist_obs = np.linalg.norm(diff_obs)
            
            # Effective distance (accounting for radius)
            effective_dist = dist_obs - obs_radius
            
            if effective_dist < self.d0 and effective_dist > 0.01:
                # Standard repulsive force
                magnitude = self.k_rep * (1.0 / effective_dist - 1.0 / self.d0) * (1.0 / (effective_dist ** 2))
                direction = diff_obs / dist_obs
                
                # Add tangential component to "slide" around obstacles
                tangent = np.array([-direction[1], direction[0]])  # Perpendicular
                
                f_rep += magnitude * direction + 0.3 * magnitude * tangent
            
            elif effective_dist <= 0.01:
                # Too close! Strong push away
                if dist_obs > 0.01:
                    direction = diff_obs / dist_obs
                else:
                    direction = np.array([1.0, 0.0])  # Default direction
                
                f_rep += 50.0 * direction
        
        # === TOTAL FORCE ===
        total_force = f_att + f_rep
        
        # === LOCAL MINIMA ESCAPE ===
        if self.stuck_counter > self.stuck_threshold:
            self.get_logger().info(f'🔄 STUCK! Applying random force to escape local minima')
            
            # Add random force perpendicular to goal direction
            if dist_to_goal > 0.1:
                goal_dir = (goal - current) / dist_to_goal
                perpendicular = np.array([-goal_dir[1], goal_dir[0]])
                
                # Random direction (left or right)
                random_sign = 1.0 if np.random.random() > 0.5 else -1.0
                escape_force = self.random_force_gain * random_sign * perpendicular
                
                total_force += escape_force
            
            self.stuck_counter = 0  # Reset counter
        
        # === NORMALIZE ===
        force_magnitude = np.linalg.norm(total_force)
        
        if force_magnitude > 0.01:
            # Normalize and apply reasonable limits
            normalized_force = total_force / force_magnitude
            
            # Limit maximum force
            max_force = 5.0
            limited_magnitude = min(force_magnitude, max_force)
            
            total_force = normalized_force * limited_magnitude
        else:
            # Very small force, add small movement toward goal
            if dist_to_goal > 0.1:
                total_force = 0.5 * (goal - current) / dist_to_goal
        
        return total_force
    
    def get_logger(self):
        """Dummy logger for when not used in ROS node"""
        class DummyLogger:
            def info(self, msg):
                print(f"[APF] {msg}")
        return DummyLogger()
