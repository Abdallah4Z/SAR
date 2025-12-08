#!/usr/bin/env python3

import numpy as np
import heapq

class AStarPlanner:
    def __init__(self, resolution=0.5, safety_distance=3.0):
        self.resolution = resolution
        self.safety_distance = safety_distance
    
    def plan(self, start, goal, obstacles):
        """A* pathfinding with obstacle avoidance"""
        
        # Determine grid bounds based on start, goal, and obstacles
        all_x = [start[0], goal[0]] + [obs['x'] for obs in obstacles]
        all_y = [start[1], goal[1]] + [obs['y'] for obs in obstacles]
        
        min_x = min(all_x) - 10
        max_x = max(all_x) + 10
        min_y = min(all_y) - 10
        max_y = max(all_y) + 10
        
        grid_w = int((max_x - min_x) / self.resolution)
        grid_h = int((max_y - min_y) / self.resolution)
        
        print(f"[A*] Grid: {grid_w}x{grid_h}, bounds: ({min_x:.1f},{min_y:.1f}) to ({max_x:.1f},{max_y:.1f})")
        
        # Create obstacle grid
        obstacle_grid = np.zeros((grid_h, grid_w), dtype=bool)
        
        for obs in obstacles:
            ox, oy = obs['x'], obs['y']
            radius = obs['radius'] + self.safety_distance
            
            print(f"[A*] Marking obstacle at ({ox:.1f}, {oy:.1f}) with radius {radius:.1f}m")
            
            # Mark all cells within radius
            for i in range(grid_h):
                for j in range(grid_w):
                    wx = min_x + j * self.resolution
                    wy = min_y + i * self.resolution
                    
                    dist = np.sqrt((wx - ox)**2 + (wy - oy)**2)
                    if dist < radius:
                        obstacle_grid[i, j] = True
        
        # Convert to grid coordinates
        def world_to_grid(pos):
            gx = int((pos[0] - min_x) / self.resolution)
            gy = int((pos[1] - min_y) / self.resolution)
            gx = max(0, min(gx, grid_w - 1))
            gy = max(0, min(gy, grid_h - 1))
            return (gx, gy)
        
        start_grid = world_to_grid(start)
        goal_grid = world_to_grid(goal)
        
        print(f"[A*] Start grid: {start_grid}, Goal grid: {goal_grid}")
        
        # Check if start or goal is in obstacle
        if obstacle_grid[start_grid[1], start_grid[0]]:
            print("[A*] WARNING: Start is in obstacle! Clearing...")
            obstacle_grid[start_grid[1], start_grid[0]] = False
        
        if obstacle_grid[goal_grid[1], goal_grid[0]]:
            print("[A*] WARNING: Goal is in obstacle! Clearing...")
            obstacle_grid[goal_grid[1], goal_grid[0]] = False
        
        # A* search
        open_set = []
        heapq.heappush(open_set, (0, start_grid))
        
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self._heuristic(start_grid, goal_grid)}
        
        iterations = 0
        max_iterations = grid_w * grid_h
        
        while open_set and iterations < max_iterations:
            iterations += 1
            _, current = heapq.heappop(open_set)
            
            if current == goal_grid:
                print(f"[A*] Path found after {iterations} iterations!")
                return self._reconstruct_path(came_from, current, min_x, min_y)
            
            for neighbor in self._neighbors(current, grid_w, grid_h, obstacle_grid):
                tentative_g = g_score[current] + self._distance(current, neighbor)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal_grid)
                    
                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        print(f"[A*] No path found after {iterations} iterations! Returning straight line")
        return [start, goal]
    
    def _heuristic(self, a, b):
        # Euclidean distance
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def _distance(self, a, b):
        # Actual distance between neighbors
        if a[0] != b[0] and a[1] != b[1]:
            return 1.414  # Diagonal
        return 1.0  # Straight
    
    def _neighbors(self, pos, w, h, obstacles):
        x, y = pos
        neighbors = []
        
        # 8-connected neighbors
        for dx, dy in [(0,1), (1,0), (0,-1), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]:
            nx, ny = x + dx, y + dy
            
            if 0 <= nx < w and 0 <= ny < h and not obstacles[ny, nx]:
                neighbors.append((nx, ny))
        
        return neighbors
    
    def _reconstruct_path(self, came_from, current, min_x, min_y):
        path = []
        
        while current in came_from:
            wx = min_x + current[0] * self.resolution
            wy = min_y + current[1] * self.resolution
            path.append((wx, wy))
            current = came_from[current]
        
        path.reverse()
        
        # Simplify path - take every Nth point
        step = max(1, len(path) // 20)  # Max 20 waypoints
        simplified = [path[i] for i in range(0, len(path), step)]
        
        if len(path) > 0 and simplified[-1] != path[-1]:
            simplified.append(path[-1])
        
        print(f"[A*] Path: {len(path)} points -> {len(simplified)} waypoints")
        
        return simplified if len(simplified) > 0 else path
