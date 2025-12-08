"""
3D RRT★ Path Planning Algorithm for AirSim Drone Navigation
Implements optimal path planning with random obstacle avoidance
"""

import airsim
import numpy as np
import math
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import cKDTree
import time
import psutil
import os
import sys
from datetime import datetime


class PerformanceMetrics:
    """Track performance metrics for the algorithm"""
    def __init__(self):
        self.start_time = None
        self.end_time = None
        self.planning_time = 0.0
        self.execution_time = 0.0
        self.total_time = 0.0
        
        self.start_memory = 0.0
        self.peak_memory = 0.0
        self.end_memory = 0.0
        
        self.cpu_percent = []
        self.iterations_completed = 0
        self.nodes_generated = 0
        self.collision_checks = 0
        self.rewiring_operations = 0
        
        self.path_length = 0.0
        self.path_waypoints = 0
        self.optimal_cost = 0.0
        
        self.process = psutil.Process(os.getpid())
    
    def start_tracking(self):
        """Start performance tracking"""
        self.start_time = time.time()
        self.start_memory = self.process.memory_info().rss / 1024 / 1024  # MB
        
    def update_peak_memory(self):
        """Update peak memory usage"""
        current_memory = self.process.memory_info().rss / 1024 / 1024  # MB
        if current_memory > self.peak_memory:
            self.peak_memory = current_memory
    
    def record_cpu(self):
        """Record CPU usage"""
        self.cpu_percent.append(self.process.cpu_percent(interval=0.01))
    
    def end_tracking(self):
        """End performance tracking"""
        self.end_time = time.time()
        self.total_time = self.end_time - self.start_time
        self.end_memory = self.process.memory_info().rss / 1024 / 1024  # MB
    
    def get_report(self):
        """Generate performance report"""
        report = {
            'timing': {
                'planning_time': self.planning_time,
                'execution_time': self.execution_time,
                'total_time': self.total_time,
            },
            'memory': {
                'start_memory_mb': self.start_memory,
                'peak_memory_mb': self.peak_memory,
                'end_memory_mb': self.end_memory,
                'memory_increase_mb': self.end_memory - self.start_memory,
            },
            'cpu': {
                'avg_cpu_percent': np.mean(self.cpu_percent) if self.cpu_percent else 0,
                'max_cpu_percent': np.max(self.cpu_percent) if self.cpu_percent else 0,
            },
            'algorithm': {
                'iterations_completed': self.iterations_completed,
                'nodes_generated': self.nodes_generated,
                'collision_checks': self.collision_checks,
                'rewiring_operations': self.rewiring_operations,
            },
            'path': {
                'path_length_m': self.path_length,
                'waypoints': self.path_waypoints,
                'optimal_cost': self.optimal_cost,
            }
        }
        return report
    
    def print_report(self):
        """Print formatted performance report"""
        report = self.get_report()
        
        print("\n" + "="*70)
        print("PERFORMANCE METRICS REPORT")
        print("="*70)
        
        print("\n⏱️  TIMING ANALYSIS:")
        print(f"  Planning Time:          {report['timing']['planning_time']:.3f} seconds")
        print(f"  Execution Time:         {report['timing']['execution_time']:.3f} seconds")
        print(f"  Total Time:             {report['timing']['total_time']:.3f} seconds")
        
        print("\n💾 MEMORY USAGE:")
        print(f"  Initial Memory:         {report['memory']['start_memory_mb']:.2f} MB")
        print(f"  Peak Memory:            {report['memory']['peak_memory_mb']:.2f} MB")
        print(f"  Final Memory:           {report['memory']['end_memory_mb']:.2f} MB")
        print(f"  Memory Increase:        {report['memory']['memory_increase_mb']:.2f} MB")
        
        print("\n🔋 CPU UTILIZATION:")
        print(f"  Average CPU:            {report['cpu']['avg_cpu_percent']:.2f}%")
        print(f"  Peak CPU:               {report['cpu']['max_cpu_percent']:.2f}%")
        
        print("\n🧮 ALGORITHM STATISTICS:")
        print(f"  Iterations:             {report['algorithm']['iterations_completed']}")
        print(f"  Nodes Generated:        {report['algorithm']['nodes_generated']}")
        print(f"  Collision Checks:       {report['algorithm']['collision_checks']}")
        print(f"  Rewiring Operations:    {report['algorithm']['rewiring_operations']}")
        
        print("\n🛤️  PATH QUALITY:")
        print(f"  Path Length:            {report['path']['path_length_m']:.2f} meters")
        print(f"  Waypoints:              {report['path']['waypoints']}")
        print(f"  Optimal Cost:           {report['path']['optimal_cost']:.2f}")
        
        # Performance assessment
        print("\n📊 PERFORMANCE ASSESSMENT:")
        self._assess_performance(report)
        
        print("\n" + "="*70)
    
    def _assess_performance(self, report):
        """Assess if performance is suitable for real-time applications"""
        issues = []
        warnings = []
        good_points = []
        
        # Timing assessment
        if report['timing']['total_time'] < 5:
            good_points.append("✓ Very fast execution (< 5s)")
        elif report['timing']['total_time'] < 15:
            good_points.append("✓ Fast execution (< 15s)")
        elif report['timing']['total_time'] < 30:
            warnings.append("⚠ Moderate execution time (15-30s)")
        else:
            issues.append("✗ Slow execution (> 30s)")
        
        # Memory assessment
        if report['memory']['peak_memory_mb'] < 100:
            good_points.append("✓ Low memory usage (< 100 MB)")
        elif report['memory']['peak_memory_mb'] < 500:
            good_points.append("✓ Moderate memory usage (< 500 MB)")
        elif report['memory']['peak_memory_mb'] < 1000:
            warnings.append("⚠ High memory usage (500-1000 MB)")
        else:
            issues.append("✗ Very high memory usage (> 1 GB)")
        
        # CPU assessment
        avg_cpu = report['cpu']['avg_cpu_percent']
        if avg_cpu < 50:
            good_points.append("✓ Low CPU usage (< 50%)")
        elif avg_cpu < 80:
            good_points.append("✓ Moderate CPU usage (< 80%)")
        else:
            warnings.append("⚠ High CPU usage (> 80%)")
        
        # Algorithm efficiency
        if report['algorithm']['iterations_completed'] < 500:
            good_points.append("✓ Quick convergence (< 500 iterations)")
        elif report['algorithm']['iterations_completed'] < 1000:
            good_points.append("✓ Normal convergence (< 1000 iterations)")
        else:
            warnings.append("⚠ Slow convergence (> 1000 iterations)")
        
        # Print assessment
        for point in good_points:
            print(f"  {point}")
        for warning in warnings:
            print(f"  {warning}")
        for issue in issues:
            print(f"  {issue}")
        
        # Overall suitability
        print("\n🎯 SUITABILITY FOR REAL-TIME APPLICATIONS:")
        if len(issues) == 0 and len(warnings) <= 1:
            print("  ✅ EXCELLENT - Suitable for real-time drone navigation")
        elif len(issues) == 0:
            print("  ✓ GOOD - Suitable with minor optimizations")
        elif len(issues) <= 1:
            print("  ⚠ FAIR - May need optimization for real-time use")
        else:
            print("  ✗ POOR - Requires significant optimization")
    
    def save_report(self, filename="performance_report.txt"):
        """Save performance report to file"""
        report = self.get_report()
        
        with open(filename, 'w') as f:
            f.write("="*70 + "\n")
            f.write("RRT★ PERFORMANCE METRICS REPORT\n")
            f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("="*70 + "\n\n")
            
            f.write("TIMING ANALYSIS:\n")
            f.write(f"  Planning Time:          {report['timing']['planning_time']:.3f} seconds\n")
            f.write(f"  Execution Time:         {report['timing']['execution_time']:.3f} seconds\n")
            f.write(f"  Total Time:             {report['timing']['total_time']:.3f} seconds\n\n")
            
            f.write("MEMORY USAGE:\n")
            f.write(f"  Initial Memory:         {report['memory']['start_memory_mb']:.2f} MB\n")
            f.write(f"  Peak Memory:            {report['memory']['peak_memory_mb']:.2f} MB\n")
            f.write(f"  Final Memory:           {report['memory']['end_memory_mb']:.2f} MB\n")
            f.write(f"  Memory Increase:        {report['memory']['memory_increase_mb']:.2f} MB\n\n")
            
            f.write("CPU UTILIZATION:\n")
            f.write(f"  Average CPU:            {report['cpu']['avg_cpu_percent']:.2f}%\n")
            f.write(f"  Peak CPU:               {report['cpu']['max_cpu_percent']:.2f}%\n\n")
            
            f.write("ALGORITHM STATISTICS:\n")
            f.write(f"  Iterations:             {report['algorithm']['iterations_completed']}\n")
            f.write(f"  Nodes Generated:        {report['algorithm']['nodes_generated']}\n")
            f.write(f"  Collision Checks:       {report['algorithm']['collision_checks']}\n")
            f.write(f"  Rewiring Operations:    {report['algorithm']['rewiring_operations']}\n\n")
            
            f.write("PATH QUALITY:\n")
            f.write(f"  Path Length:            {report['path']['path_length_m']:.2f} meters\n")
            f.write(f"  Waypoints:              {report['path']['waypoints']}\n")
            f.write(f"  Optimal Cost:           {report['path']['optimal_cost']:.2f}\n\n")
        
        print(f"Performance report saved to: {filename}")


class Node:
    """Represents a node in the RRT★ tree"""
    def __init__(self, position):
        self.position = np.array(position, dtype=float)
        self.parent = None
        self.cost = 0.0  # Cost from start to this node
        
    def __repr__(self):
        return f"Node({self.position})"


class RRTStar3D:
    """3D RRT★ Path Planning Algorithm"""
    
    def __init__(self, start, goal, obstacles, boundary, 
                 step_size=5.0, goal_sample_rate=0.05, 
                 max_iter=1000, search_radius=15.0, drone_radius=2.0, metrics=None):
        """
        Initialize RRT★ planner
        
        Args:
            start: Start position [x, y, z]
            goal: Goal position [x, y, z]
            obstacles: List of obstacle dicts with 'center' and 'radius'
            boundary: Tuple of ((x_min, x_max), (y_min, y_max), (z_min, z_max))
            step_size: Maximum distance for tree expansion
            goal_sample_rate: Probability of sampling goal point
            max_iter: Maximum number of iterations
            search_radius: Radius for nearby node search and rewiring
            drone_radius: Safety radius around drone (meters) - added to obstacle collision
            metrics: PerformanceMetrics object for tracking
        """
        self.start = Node(start)
        self.goal = Node(goal)
        self.obstacles = obstacles
        self.boundary = boundary
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.search_radius = search_radius
        self.drone_radius = drone_radius  # Safety margin for collision detection
        
        self.node_list = [self.start]
        self.goal_node = None
        
        # Performance tracking
        self.metrics = metrics if metrics else PerformanceMetrics()
        
    def plan(self):
        """
        Execute RRT★ planning algorithm
        
        Returns:
            List of waypoints from start to goal, or None if no path found
        """
        print("Starting RRT★ path planning...")
        planning_start_time = time.time()
        
        for i in range(self.max_iter):
            # Update metrics
            self.metrics.update_peak_memory()
            if i % 50 == 0:
                self.metrics.record_cpu()
            
            # Sample random point
            if random.random() < self.goal_sample_rate:
                random_point = self.goal.position
            else:
                random_point = self._sample_random_point()
            
            # Find nearest node in tree
            nearest_node = self._get_nearest_node(random_point)
            
            # Steer toward random point
            new_position = self._steer(nearest_node.position, random_point)
            
            # Check collision
            self.metrics.collision_checks += 1
            if self._check_collision(nearest_node.position, new_position):
                continue
            
            # Create new node
            new_node = Node(new_position)
            self.metrics.nodes_generated += 1
            
            # Find nearby nodes
            nearby_nodes = self._find_nearby_nodes(new_node)
            
            # Choose best parent (minimum cost)
            best_parent = self._choose_parent(new_node, nearby_nodes, nearest_node)
            
            if best_parent is None:
                continue
                
            new_node.parent = best_parent
            new_node.cost = best_parent.cost + self._distance(best_parent.position, new_node.position)
            
            # Add node to tree
            self.node_list.append(new_node)
            
            # Rewire tree
            rewire_count = self._rewire(new_node, nearby_nodes)
            self.metrics.rewiring_operations += rewire_count
            
            # Check if goal is reached
            if self._distance(new_node.position, self.goal.position) <= self.step_size:
                if not self._check_collision(new_node.position, self.goal.position):
                    self.goal.parent = new_node
                    self.goal.cost = new_node.cost + self._distance(new_node.position, self.goal.position)
                    self.goal_node = self.goal
                    print(f"Goal reached at iteration {i+1}!")
                    # Continue to optimize
            
            if (i + 1) % 100 == 0:
                print(f"Iteration {i+1}/{self.max_iter}")
            
            self.metrics.iterations_completed = i + 1
        
        planning_end_time = time.time()
        self.metrics.planning_time = planning_end_time - planning_start_time
        
        if self.goal_node is None:
            print("Failed to find path to goal")
            return None
        
        # Extract path
        path = self._extract_path()
        
        # Update metrics
        self.metrics.optimal_cost = self.goal.cost
        self.metrics.path_waypoints = len(path)
        self.metrics.path_length = self._calculate_path_length(path)
        
        print(f"Path found with {len(path)} waypoints and cost {self.goal.cost:.2f}")
        return path
    
    def _sample_random_point(self):
        """Sample random point within boundary"""
        x = random.uniform(self.boundary[0][0], self.boundary[0][1])
        y = random.uniform(self.boundary[1][0], self.boundary[1][1])
        z = random.uniform(self.boundary[2][0], self.boundary[2][1])
        return np.array([x, y, z])
    
    def _get_nearest_node(self, point):
        """Find nearest node to given point using Euclidean distance"""
        positions = np.array([node.position for node in self.node_list])
        distances = np.linalg.norm(positions - point, axis=1)
        nearest_idx = np.argmin(distances)
        return self.node_list[nearest_idx]
    
    def _steer(self, from_pos, to_pos):
        """Steer from one position toward another by step_size"""
        direction = to_pos - from_pos
        distance = np.linalg.norm(direction)
        
        if distance <= self.step_size:
            return to_pos
        else:
            return from_pos + (direction / distance) * self.step_size
    
    def _check_collision(self, from_pos, to_pos):
        """
        Check if path between two positions collides with any obstacle
        Uses line-sphere intersection with safety margin (drone_radius)
        """
        for obstacle in self.obstacles:
            # Add drone_radius as safety margin to obstacle radius
            effective_radius = obstacle['radius'] + self.drone_radius
            if self._line_sphere_intersection(from_pos, to_pos, 
                                             obstacle['center'], 
                                             effective_radius):
                return True
        return False
    
    def _line_sphere_intersection(self, p1, p2, center, radius):
        """
        Check if line segment from p1 to p2 intersects with sphere
        
        Args:
            p1, p2: Line segment endpoints
            center: Sphere center
            radius: Sphere radius
        
        Returns:
            True if intersection exists, False otherwise
        """
        # Vector from p1 to p2
        d = p2 - p1
        # Vector from p1 to center
        f = p1 - center
        
        a = np.dot(d, d)
        b = 2 * np.dot(f, d)
        c = np.dot(f, f) - radius * radius
        
        discriminant = b * b - 4 * a * c
        
        if discriminant < 0:
            return False
        
        discriminant = math.sqrt(discriminant)
        
        t1 = (-b - discriminant) / (2 * a)
        t2 = (-b + discriminant) / (2 * a)
        
        # Check if intersection is within line segment
        if (0 <= t1 <= 1) or (0 <= t2 <= 1) or (t1 < 0 and t2 > 1):
            return True
        
        return False
    
    def _find_nearby_nodes(self, node):
        """Find all nodes within search_radius of given node"""
        nearby_nodes = []
        for n in self.node_list:
            if self._distance(n.position, node.position) <= self.search_radius:
                nearby_nodes.append(n)
        return nearby_nodes
    
    def _choose_parent(self, node, nearby_nodes, nearest_node):
        """
        Choose best parent for node from nearby nodes
        Best parent minimizes total cost from start
        """
        if not nearby_nodes:
            nearby_nodes = [nearest_node]
        
        min_cost = float('inf')
        best_parent = None
        
        for nearby_node in nearby_nodes:
            if self._check_collision(nearby_node.position, node.position):
                continue
            
            cost = nearby_node.cost + self._distance(nearby_node.position, node.position)
            
            if cost < min_cost:
                min_cost = cost
                best_parent = nearby_node
        
        return best_parent
    
    def _rewire(self, new_node, nearby_nodes):
        """
        Rewire tree: check if routing through new_node reduces cost for nearby nodes
        Returns number of rewiring operations performed
        """
        rewire_count = 0
        for nearby_node in nearby_nodes:
            if nearby_node == new_node.parent:
                continue
            
            new_cost = new_node.cost + self._distance(new_node.position, nearby_node.position)
            
            if new_cost < nearby_node.cost:
                if not self._check_collision(new_node.position, nearby_node.position):
                    nearby_node.parent = new_node
                    nearby_node.cost = new_cost
                    rewire_count += 1
                    # Propagate cost update to children
                    self._propagate_cost_to_children(nearby_node)
        
        return rewire_count
    
    def _propagate_cost_to_children(self, parent_node):
        """Recursively update costs of all children after rewiring"""
        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = parent_node.cost + self._distance(parent_node.position, node.position)
                self._propagate_cost_to_children(node)
    
    def _distance(self, pos1, pos2):
        """Calculate Euclidean distance between two positions"""
        return np.linalg.norm(pos1 - pos2)
    
    def _extract_path(self):
        """Extract path from start to goal by backtracking through parents"""
        if self.goal_node is None:
            return None
        
        path = [self.goal_node.position]
        current = self.goal_node
        
        while current.parent is not None:
            current = current.parent
            path.append(current.position)
        
        path.reverse()
        return path
    
    def _calculate_path_length(self, path):
        """Calculate total Euclidean length of path"""
        if not path or len(path) < 2:
            return 0.0
        
        length = 0.0
        for i in range(len(path) - 1):
            length += self._distance(path[i], path[i+1])
        return length


def generate_random_obstacles(count, boundary, radius_range=(3, 8), 
                              start_pos=None, goal_pos=None, 
                              min_clearance=10.0):
    """
    Generate random spherical obstacles within boundary
    50% placed along the direct path to force interesting navigation!
    
    Args:
        count: Number of obstacles to generate
        boundary: Environment boundaries ((x_min, x_max), (y_min, y_max), (z_min, z_max))
        radius_range: (min_radius, max_radius) for obstacles
        start_pos: Start position to avoid [x, y, z]
        goal_pos: Goal position to avoid [x, y, z]
        min_clearance: Minimum distance from start/goal positions
    
    Returns:
        List of obstacle dictionaries with 'center' and 'radius'
    """
    obstacles = []
    
    # Calculate how many obstacles to place along path vs random
    obstacles_on_path = int(count * 0.4)  # Reduced to 40% (was 50%)
    obstacles_random = count - obstacles_on_path
    
    # First, place obstacles ALONG the direct path (to make it interesting!)
    if start_pos is not None and goal_pos is not None:
        start = np.array(start_pos)
        goal = np.array(goal_pos)
        path_vector = goal - start
        path_length = np.linalg.norm(path_vector)
        path_direction = path_vector / path_length
        
        print(f"  Placing {obstacles_on_path} obstacles along the path...")
        
        for i in range(obstacles_on_path):
            attempts = 0
            while attempts < 50:
                # Position along path (skip start and goal areas)
                t = random.uniform(0.25, 0.75)  # Middle 50% of path (was 60%)
                base_pos = start + t * path_vector
                
                # Offset perpendicular to path (not directly blocking, but close!)
                offset_distance = random.uniform(8, 20)  # Increased from 3-15 to 8-20 meters
                
                # Random perpendicular offset
                perp_angle = random.uniform(0, 2*np.pi)
                offset_x = offset_distance * np.cos(perp_angle)
                offset_y = offset_distance * np.sin(perp_angle)
                
                center = base_pos + np.array([offset_x, offset_y, random.uniform(-3, 3)])
                
                # Random radius
                radius = random.uniform(radius_range[0], radius_range[1])
                
                # Check clearance from start and goal
                dist_to_start = np.linalg.norm(center - start)
                dist_to_goal = np.linalg.norm(center - goal)
                
                if (dist_to_start > (radius + min_clearance) and 
                    dist_to_goal > (radius + min_clearance)):
                    obstacles.append({'center': center, 'radius': radius})
                    break
                
                attempts += 1
    
    # Then place remaining obstacles randomly in the environment
    print(f"  Placing {obstacles_random} random obstacles...")
    for i in range(obstacles_random):
        attempts = 0
        max_attempts = 100
        
        while attempts < max_attempts:
            # Random position
            x = random.uniform(boundary[0][0], boundary[0][1])
            y = random.uniform(boundary[1][0], boundary[1][1])
            z = random.uniform(boundary[2][0], boundary[2][1])
            center = np.array([x, y, z])
            
            # Random radius
            radius = random.uniform(radius_range[0], radius_range[1])
            
            # Check clearance from start and goal
            valid = True
            
            if start_pos is not None:
                dist_to_start = np.linalg.norm(center - np.array(start_pos))
                if dist_to_start < (radius + min_clearance):
                    valid = False
            
            if goal_pos is not None and valid:
                dist_to_goal = np.linalg.norm(center - np.array(goal_pos))
                if dist_to_goal < (radius + min_clearance):
                    valid = False
            
            if valid:
                obstacles.append({
                    'center': center,
                    'radius': radius
                })
                break
            
            attempts += 1
        
        if attempts >= max_attempts:
            print(f"Warning: Could not place obstacle {i+1} after {max_attempts} attempts")
    
    print(f"Generated {len(obstacles)} obstacles")
    return obstacles


def visualize_rrt_star_3d(rrt_star, path=None, show=True, save_path=None):
    """
    Visualize RRT★ tree, obstacles, and path in 3D
    
    Args:
        rrt_star: RRTStar3D object with tree and obstacles
        path: Computed path waypoints (optional)
        show: Whether to display the plot
        save_path: Path to save figure (optional)
    """
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot obstacles
    for obs in rrt_star.obstacles:
        # Draw sphere
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        x = obs['center'][0] + obs['radius'] * np.outer(np.cos(u), np.sin(v))
        y = obs['center'][1] + obs['radius'] * np.outer(np.sin(u), np.sin(v))
        z = obs['center'][2] + obs['radius'] * np.outer(np.ones(np.size(u)), np.cos(v))
        ax.plot_surface(x, y, z, color='red', alpha=0.3)
    
    # Plot tree edges
    for node in rrt_star.node_list:
        if node.parent is not None:
            xs = [node.position[0], node.parent.position[0]]
            ys = [node.position[1], node.parent.position[1]]
            zs = [node.position[2], node.parent.position[2]]
            ax.plot(xs, ys, zs, 'gray', linewidth=0.5, alpha=0.5)
    
    # Plot nodes
    node_positions = np.array([node.position for node in rrt_star.node_list])
    ax.scatter(node_positions[:, 0], node_positions[:, 1], node_positions[:, 2],
               c='blue', marker='o', s=10, alpha=0.6, label='Tree Nodes')
    
    # Plot start and goal
    ax.scatter(*rrt_star.start.position, c='green', marker='o', s=200, 
               label='Start', edgecolors='black', linewidths=2)
    ax.scatter(*rrt_star.goal.position, c='red', marker='*', s=300, 
               label='Goal', edgecolors='black', linewidths=2)
    
    # Plot path if available
    if path is not None:
        path_array = np.array(path)
        ax.plot(path_array[:, 0], path_array[:, 1], path_array[:, 2],
                'g-', linewidth=3, label='Optimal Path', alpha=0.8)
        ax.scatter(path_array[:, 0], path_array[:, 1], path_array[:, 2],
                   c='yellow', marker='o', s=50, edgecolors='black', linewidths=1)
    
    # Set labels and limits
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_zlabel('Z (m)', fontsize=12)
    ax.set_title('RRT★ 3D Path Planning', fontsize=14, fontweight='bold')
    
    ax.set_xlim(rrt_star.boundary[0])
    ax.set_ylim(rrt_star.boundary[1])
    ax.set_zlim(rrt_star.boundary[2])
    
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Visualization saved to {save_path}")
    
    if show:
        plt.show()
    
    return fig, ax


def smooth_path(path, alpha=0.5, iterations=100):
    """
    Smooth path using iterative averaging
    
    Args:
        path: List of waypoints
        alpha: Smoothing factor (0-1)
        iterations: Number of smoothing iterations
    
    Returns:
        Smoothed path
    """
    if len(path) <= 2:
        return path
    
    smoothed = np.array(path, dtype=float)
    
    for _ in range(iterations):
        new_path = smoothed.copy()
        for i in range(1, len(smoothed) - 1):
            new_path[i] = smoothed[i] + alpha * (smoothed[i-1] + smoothed[i+1] - 2 * smoothed[i])
        smoothed = new_path
    
    return smoothed.tolist()


if __name__ == "__main__":
    # Test the algorithm without AirSim
    print("Testing RRT★ algorithm...")
    
    # Initialize performance metrics
    metrics = PerformanceMetrics()
    metrics.start_tracking()
    
    # Define environment
    boundary = ((-100, 100), (-100, 100), (-30, -5))
    start_pos = [0, 0, -10]
    goal_pos = [80, 80, -20]
    
    # Generate obstacles
    obstacles = generate_random_obstacles(
        count=30,
        boundary=boundary,
        radius_range=(3, 8),
        start_pos=start_pos,
        goal_pos=goal_pos,
        min_clearance=10.0
    )
    
    # Create and run RRT★
    rrt = RRTStar3D(
        start=start_pos,
        goal=goal_pos,
        obstacles=obstacles,
        boundary=boundary,
        step_size=5.0,
        goal_sample_rate=0.05,
        max_iter=1000,
        search_radius=15.0,
        metrics=metrics
    )
    
    path = rrt.plan()
    
    # End tracking
    metrics.end_tracking()
    
    if path:
        print(f"\nPath computed successfully!")
        print(f"Number of waypoints: {len(path)}")
        print(f"Path cost: {rrt.goal.cost:.2f}")
        
        # Print performance report
        metrics.print_report()
        metrics.save_report("performance_report_standalone.txt")
        
        # Visualize
        visualize_rrt_star_3d(rrt, path, show=True)
    else:
        print("\nFailed to find path")
        metrics.print_report()
        visualize_rrt_star_3d(rrt, show=True)
