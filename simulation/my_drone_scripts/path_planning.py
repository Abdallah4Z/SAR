import numpy as np
import heapq
import math

class PathPlanner:
    def __init__(self, grid_size=50, alpha=0.5):
        self.grid_size = grid_size
        self.alpha = alpha  # Energy weight factor
        self.grid_resolution = 1.0  # 1 meter per grid cell

    def plan(self, start, goal, energy_rate=0.1, obstacles=None):
        """
        Energy-aware A* path planning
        :param start: (x, y, z) starting position
        :param goal: (x, y, z) goal position
        :param energy_rate: Energy consumption rate (W/m)
        :param obstacles: List of obstacle positions (optional)
        :return: List of waypoints and total energy cost
        """
        path = self._astar_3d(start, goal, energy_rate, obstacles)
        energy_cost = self._calculate_path_energy(path, energy_rate)
        return path, energy_cost

    def _astar_3d(self, start, goal, energy_rate, obstacles):
        """3D A* with energy awareness"""
        start_node = tuple(np.round(start).astype(int))
        goal_node = tuple(np.round(goal).astype(int))
        
        open_set = []
        heapq.heappush(open_set, (0, start_node))
        
        came_from = {}
        g_score = {start_node: 0}
        
        obstacle_set = set()
        if obstacles:
            obstacle_set = {tuple(np.round(obs).astype(int)) for obs in obstacles}
        
        while open_set:
            current_f, current = heapq.heappop(open_set)
            
            if current == goal_node:
                return self._reconstruct_path(came_from, current)
            
            for neighbor in self._get_neighbors(current):
                if neighbor in obstacle_set:
                    continue
                
                distance = self._euclidean_distance(current, neighbor)
                energy_cost = distance * energy_rate
                
                # Combined cost: distance + energy weight
                tentative_g = g_score[current] + distance + self.alpha * energy_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    
                    h = self._heuristic(neighbor, goal_node, energy_rate)
                    f = tentative_g + h
                    
                    heapq.heappush(open_set, (f, neighbor))
        
        # If no path found, return direct path
        return [start, goal]

    def _get_neighbors(self, node):
        """Get 26-connected neighbors in 3D space"""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    neighbor = (node[0] + dx, node[1] + dy, node[2] + dz)
                    neighbors.append(neighbor)
        return neighbors

    def _heuristic(self, node, goal, energy_rate):
        """Euclidean distance with energy factor"""
        dist = self._euclidean_distance(node, goal)
        return dist * (1 + self.alpha * energy_rate)

    def _euclidean_distance(self, a, b):
        """Calculate Euclidean distance between two points"""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2])**2)

    def _reconstruct_path(self, came_from, current):
        """Reconstruct path from came_from dictionary"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return [tuple(float(x) for x in node) for node in path]

    def _calculate_path_energy(self, path, energy_rate):
        """Calculate total energy for the path"""
        total_energy = 0
        for i in range(len(path) - 1):
            dist = self._euclidean_distance(path[i], path[i+1])
            total_energy += dist * energy_rate
        return total_energy