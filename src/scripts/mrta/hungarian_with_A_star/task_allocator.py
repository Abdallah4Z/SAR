import math
import numpy as np
from scipy.optimize import linear_sum_assignment

class TaskAllocator:
    def __init__(self, w1=1.0, w2=0.5, w3=2.0, min_battery_threshold=15.0):
        """
        Initialize task allocator with cost weights
        :param w1: Distance weight
        :param w2: Energy weight
        :param w3: Importance weight
        :param min_battery_threshold: Minimum battery to accept task
        """
        self.w1 = w1
        self.w2 = w2
        self.w3 = w3
        self.min_battery_threshold = min_battery_threshold
        self.assigned_tasks = {}
        self.task_history = []

    def assign_tasks(self, drones, tasks, path_planner):
        """
        Assign tasks to drones using Hungarian algorithm
        :param drones: dict of drone states
        :param tasks: list of task dictionaries
        :param path_planner: PathPlanner instance for cost calculation
        :return: dict of assignments {drone_name: task}
        """
        if not tasks or not drones:
            return {}
        
        available_drones = [name for name, drone in drones.items() 
                           if drone['status'] == 'Idle' and 
                           drone['battery'] > self.min_battery_threshold]
        
        if not available_drones:
            return {}
        
        # Build cost matrix
        cost_matrix = np.full((len(available_drones), len(tasks)), 1e9)
        
        for i, drone_name in enumerate(available_drones):
            drone = drones[drone_name]
            
            for j, task in enumerate(tasks):
                cost = self._calculate_cost(drone, task, path_planner)
                
                # Check if drone can complete task with current battery
                if cost['energy'] <= drone['battery']:
                    cost_matrix[i][j] = cost['total']
        
        # Use Hungarian algorithm for optimal assignment
        if cost_matrix.size > 0:
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            
            assignments = {}
            for i, j in zip(row_ind, col_ind):
                if cost_matrix[i][j] < 1e9:  # Valid assignment
                    drone_name = available_drones[i]
                    task = tasks[j]
                    
                    assignments[drone_name] = task
                    self.assigned_tasks[drone_name] = task
                    drones[drone_name]['status'] = 'Executing'
                    
                    self.task_history.append({
                        'drone': drone_name,
                        'task_id': task['task_id'],
                        'timestamp': None,
                        'cost': cost_matrix[i][j]
                    })
            
            return assignments
        
        return {}

    def _calculate_cost(self, drone, task, path_planner):
        """
        Calculate weighted cost for drone-task pair
        :return: dict with distance, energy, and total cost
        """
        start = drone['position']
        goal = task['goal_position']
        
        # Get path and energy from planner
        path, energy = path_planner.plan(start, goal, drone['energy_rate'])
        
        # Calculate distance
        distance = self._path_distance(path)
        
        # Importance factor (inverse)
        importance_factor = 1.0 / task['importance'] if task['importance'] > 0 else 10.0
        
        # Check payload capacity
        if task.get('payload', 0) > drone['payload_capacity']:
            return {'distance': distance, 'energy': 1e9, 'total': 1e9}
        
        # Weighted cost
        total_cost = (self.w1 * distance + 
                     self.w2 * energy + 
                     self.w3 * importance_factor)
        
        return {
            'distance': distance,
            'energy': energy,
            'total': total_cost
        }

    def _path_distance(self, path):
        """Calculate total path distance"""
        total_dist = 0
        for i in range(len(path) - 1):
            total_dist += math.dist(path[i], path[i+1])
        return total_dist

    def release_drone(self, drone_name, drones):
        """Release drone from task and mark as idle"""
        if drone_name in self.assigned_tasks:
            del self.assigned_tasks[drone_name]
        
        if drone_name in drones:
            drones[drone_name]['status'] = 'Idle'
        
        return True

    def get_next_task(self, drone_name, drones, remaining_tasks, path_planner):
        """
        Get next best task for a specific drone
        :return: task dict or None
        """
        if not remaining_tasks:
            return None
        
        drone = drones.get(drone_name)
        if not drone or drone['battery'] <= self.min_battery_threshold:
            return None
        
        best_task = None
        best_cost = float('inf')
        
        for task in remaining_tasks:
            cost = self._calculate_cost(drone, task, path_planner)
            
            if cost['energy'] <= drone['battery'] and cost['total'] < best_cost:
                best_cost = cost['total']
                best_task = task
        
        return best_task

    def get_task_history(self):
        """Return task assignment history"""
        return self.task_history
