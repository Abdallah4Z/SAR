import heapq
import math


def heuristic(a, b):
    """
    Calculates the estimated cost (h(n)) from point 'a' to point 'b'.
    Uses Euclidean distance for a more accurate estimate, which guides A* efficiently.
    """
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def a_star_pathfinding(start, goal, static_map):
    """
    The A* algorithm that finds the shortest path on the grid (simulated NavMesh).
    
    Args:
        start (tuple): The (row, col) grid coordinates of the SAR agent.
        goal (tuple): The (row, col) grid coordinates of the human target.
        static_map (list of lists): The environment map (0=walkable, 1=obstacle).
        
    Returns:
        list of tuples: The shortest path, or None if no path exists.
    """
    
    ROWS = len(static_map)
    COLS = len(static_map[0])

    # 1. Initialization
    # Priority Queue stores: (f_score, g_score, position, path_so_far)
    pq = [(0, 0, start, [start])] 
    # Visited tracks the lowest actual cost (g_score) found so far for each position
    visited = {start: 0} 

    # 2. Search Loop
    while pq:
        # Get the node with the lowest f_score (best candidate)
        f, g, current_pos, path = heapq.heappop(pq)
        
        # Check if the goal has been reached
        if current_pos == goal:
            return path 

        # 3. Explore Neighbors (8 directions: orthogonal and diagonal)
        for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0), 
                       (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            
            r, c = current_pos[0] + dr, current_pos[1] + dc
            neighbor = (r, c)
            
            # Check bounds (is it on the map?) and if it's walkable (not an obstacle)
            if 0 <= r < ROWS and 0 <= c < COLS and static_map[r][c] == 0:
                
                # Calculate the cost to move to this neighbor (g(n))
                # Step cost is 1 for orthogonal moves, sqrt(2) for diagonal moves
                step_cost = math.sqrt(dr**2 + dc**2) 
                new_g = g + step_cost
                
                # Check if this new path to the neighbor is better than any previous path
                if neighbor not in visited or new_g < visited[neighbor]:
                    
                    # Update costs and path for the neighbor
                    visited[neighbor] = new_g
                    h = heuristic(neighbor, goal)
                    new_f = new_g + h # f(n) = g(n) + h(n)
                    
                    new_path = path + [neighbor]
                    
                    # Add the neighbor to the priority queue
                    heapq.heappush(pq, (new_f, new_g, neighbor, new_path))
    
    # If the loop finishes without finding the goal
    return None 


# # Simplified Map (0=Walkable, 1=Obstacle)
# MAP = [
#     [0, 0, 0, 0, 0],
#     [0, 1, 1, 1, 0],
#     [0, 0, 0, 1, 0],
#     [1, 1, 0, 1, 0],
#     [0, 0, 0, 0, 0]
# ]

# start_pos = (0, 0) # Top-left
# goal_pos = (4, 4)  # Bottom-right

# # Find the path
# final_path = a_star_pathfinding(start_pos, goal_pos, MAP)

# if final_path:
#     print(f"\n A* Path Found from {start_pos} to {goal_pos}:")
#     print(final_path)
# else:
#     print(f"\nNo Path Found between {start_pos} and {goal_pos}.")