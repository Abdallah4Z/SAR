"""
Environment configurations for different experiment scales.

V3 FIX: dt = 0.01 seconds (10ms) to handle 50ms deadlines.
"""

CONFIGS = {
    'small': {
        'num_uavs': 4,
        'num_ground_devices': 20,
        'area_size': [500, 500, 100],  # [width, height, altitude] in meters
        'lambda_rate': 0.5,  # tasks/sec/device
        'episode_length': 1000,  # timesteps (1000 * 0.01s = 10 seconds)
        'dt': 0.01,  # 0.01 seconds (10ms) per timestep
    },
    'medium': {
        'num_uavs': 8,
        'num_ground_devices': 40,
        'area_size': [1000, 1000, 100],
        'lambda_rate': 0.5,
        'episode_length': 1500,  # 15 seconds sim time
        'dt': 0.01,
    },
    'large': {
        'num_uavs': 12,
        'num_ground_devices': 60,
        'area_size': [1000, 1000, 100],
        'lambda_rate': 0.5,
        'episode_length': 2000,  # 20 seconds sim time
        'dt': 0.01,
    }
}


def get_config_for_swarm_size(num_drones, seed=None):
    """
    Generate environment config for a specific swarm size.
    
    Scales area and ground devices proportionally to swarm size.
    
    Args:
        num_drones: Number of UAVs (5, 10, 20, 50, 100)
        seed: Random seed (optional)
        
    Returns:
        Config dict for UAVSwarmEnv
    """
    # Base scaling: 5 ground devices per UAV
    num_ground_devices = num_drones * 5
    
    # Scale area based on swarm size (more drones = larger area)
    if num_drones <= 10:
        area_size = [500, 500, 100]
        episode_length = 1000  # 10 seconds sim time (1000 * 0.01s)
    elif num_drones <= 20:
        area_size = [750, 750, 100]
        episode_length = 1500  # 15 seconds sim time
    elif num_drones <= 50:
        area_size = [1000, 1000, 100]
        episode_length = 2000  # 20 seconds sim time
    else:  # 100 drones
        area_size = [1500, 1500, 100]
        episode_length = 3000  # 30 seconds sim time
    
    config = {
        'num_uavs': num_drones,
        'num_ground_devices': num_ground_devices,
        'area_size': area_size,
        'lambda_rate': 0.5,
        'episode_length': episode_length,
        'dt': 0.01,  # 10ms timesteps
    }
    
    if seed is not None:
        config['seed'] = seed
    
    return config

