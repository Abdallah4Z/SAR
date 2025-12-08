"""
Helper script to visualize obstacles using AirSim plot functions
Works with most AirSim versions
"""

import airsim
import numpy as np
import time


def draw_sphere_wireframe(client, center, radius, color=[1, 0, 0], thickness=5.0, num_segments=16):
    """
    Draw a wireframe sphere using line segments
    
    Args:
        client: AirSim client
        center: [x, y, z] center position
        radius: Sphere radius
        color: RGB color [r, g, b] where each is 0-1
        thickness: Line thickness
        num_segments: Number of segments for circles
    """
    points = []
    
    # Draw 3 circles (XY, XZ, YZ planes)
    for angle in np.linspace(0, 2*np.pi, num_segments, endpoint=False):
        # XY plane circle
        x1 = center[0] + radius * np.cos(angle)
        y1 = center[1] + radius * np.sin(angle)
        z1 = center[2]
        
        x2 = center[0] + radius * np.cos(angle + 2*np.pi/num_segments)
        y2 = center[1] + radius * np.sin(angle + 2*np.pi/num_segments)
        z2 = center[2]
        
        points.append(airsim.Vector3r(x1, y1, z1))
        points.append(airsim.Vector3r(x2, y2, z2))
        
        # XZ plane circle
        x1 = center[0] + radius * np.cos(angle)
        y1 = center[1]
        z1 = center[2] + radius * np.sin(angle)
        
        x2 = center[0] + radius * np.cos(angle + 2*np.pi/num_segments)
        y2 = center[1]
        z2 = center[2] + radius * np.sin(angle + 2*np.pi/num_segments)
        
        points.append(airsim.Vector3r(x1, y1, z1))
        points.append(airsim.Vector3r(x2, y2, z2))
        
        # YZ plane circle
        x1 = center[0]
        y1 = center[1] + radius * np.cos(angle)
        z1 = center[2] + radius * np.sin(angle)
        
        x2 = center[0]
        y2 = center[1] + radius * np.cos(angle + 2*np.pi/num_segments)
        z2 = center[2] + radius * np.sin(angle + 2*np.pi/num_segments)
        
        points.append(airsim.Vector3r(x1, y1, z1))
        points.append(airsim.Vector3r(x2, y2, z2))
    
    # Draw all line segments
    try:
        client.simPlotLineList(
            points=points,
            color_rgba=color + [1.0],  # Add alpha channel
            thickness=thickness,
            duration=600.0,  # Last for 10 minutes
            is_persistent=True
        )
        return True
    except Exception as e:
        return False


def visualize_obstacles_as_lines(client, obstacles, color=[1, 0, 0]):
    """
    Visualize obstacles as wireframe spheres in AirSim
    
    Args:
        client: AirSim client
        obstacles: List of obstacle dicts with 'center' and 'radius'
        color: RGB color for obstacles [r, g, b]
    
    Returns:
        Number of obstacles visualized
    """
    print(f"\nDrawing {len(obstacles)} obstacles as wireframes...")
    
    success_count = 0
    failed_count = 0
    
    for i, obs in enumerate(obstacles):
        try:
            center = obs['center']
            radius = obs['radius']
            
            success = draw_sphere_wireframe(
                client,
                center,
                radius,
                color=color,
                thickness=5.0,  # Thicker lines for visibility
                num_segments=16  # More segments for better visibility
            )
            
            if success:
                success_count += 1
            else:
                failed_count += 1
                
            if (i + 1) % 10 == 0:
                print(f"  Drew {i + 1}/{len(obstacles)} obstacles...")
        
        except Exception as e:
            failed_count += 1
            if i == 0:
                print(f"  Warning: simPlotLineList may not be working: {e}")
            continue
    
    if success_count > 0:
        print(f"✓ Drew {success_count} obstacles as wireframes")
        if failed_count > 0:
            print(f"  ({failed_count} failed - this is normal)")
    else:
        print(f"⚠ simPlotLineList not supported in your AirSim version")
        print(f"  Obstacles are still used for path planning (virtual)")
    
    return success_count


def draw_path_in_airsim(client, path, color=[0, 1, 0], thickness=5.0):
    """
    Draw the planned path in AirSim
    
    Args:
        client: AirSim client
        path: List of waypoints [[x, y, z], ...]
        color: RGB color for path
        thickness: Line thickness
    """
    if not path or len(path) < 2:
        return False
    
    print(f"\nDrawing path with {len(path)} waypoints...")
    
    try:
        points = []
        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i + 1]
            points.append(airsim.Vector3r(p1[0], p1[1], p1[2]))
            points.append(airsim.Vector3r(p2[0], p2[1], p2[2]))
        
        client.simPlotLineList(
            points=points,
            color_rgba=color + [1.0],
            thickness=thickness,
            duration=600.0,
            is_persistent=True
        )
        
        print(f"✓ Path drawn in AirSim")
        return True
        
    except Exception as e:
        print(f"⚠ Could not draw path: {e}")
        return False


def clear_all_plots(client):
    """Clear all plotted lines in AirSim"""
    try:
        client.simFlushPersistentMarkers()
        print("✓ Cleared all plotted lines")
        return True
    except Exception as e:
        print(f"Note: Could not clear plots: {e}")
        return False


if __name__ == "__main__":
    # Test visualization
    print("Testing obstacle visualization...")
    
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    # Test obstacles
    test_obstacles = [
        {'center': np.array([10, 10, -10]), 'radius': 3},
        {'center': np.array([20, 20, -15]), 'radius': 4},
        {'center': np.array([30, 15, -12]), 'radius': 3.5},
    ]
    
    visualize_obstacles_as_lines(client, test_obstacles, color=[1, 0, 0])
    
    # Test path
    test_path = [
        [0, 0, -5],
        [10, 10, -8],
        [20, 20, -12],
        [30, 30, -15]
    ]
    
    draw_path_in_airsim(client, test_path, color=[0, 1, 0])
    
    print("\nVisualization complete! Check Unreal Engine window.")
    print("Lines will persist for 10 minutes.")
