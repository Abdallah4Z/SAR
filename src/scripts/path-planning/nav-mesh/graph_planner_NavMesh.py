# pathfinder.py
import heapq
import numpy as np
from itertools import count

class Pathfinder:
    def __init__(self, navmesh):
        self.navmesh = navmesh
        self.counter = count()  

    def heuristic(self, n1, n2):
        """Euclidean distance between two nodes as heuristic."""
        pos1 = self.navmesh.get_position(n1)
        pos2 = self.navmesh.get_position(n2)
        return np.linalg.norm(pos1 - pos2)

    def find_path(self, start_node, goal_node):
        """A* search on the NavMesh with tie-breaker for deterministic behavior."""
        open_set = []
        g_score = {start_node: 0}

        start_f = self.heuristic(start_node, goal_node)
        heapq.heappush(open_set, (start_f, 0, next(self.counter), start_node))

        came_from = {}

        while open_set:
            f_current, g_current, _, current = heapq.heappop(open_set)

            if g_current > g_score[current]:
                continue

            # Goal reached
            if current == goal_node:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]  # reverse path

            # Explore neighbors
            for neighbor in self.navmesh.graph.neighbors(current):
                tentative_g = g_score[current] + self.navmesh.graph[current][neighbor]['weight']

                # If this path to neighbor is better, record it
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal_node)
                    heapq.heappush(open_set, (f_score, tentative_g, next(self.counter), neighbor))
                    came_from[neighbor] = current

        # No path found
        return None
