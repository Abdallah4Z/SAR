from .uav_swarm_env import UAVSwarmEnv
from .uav_swarm_env_gpu import UAVSwarmEnvGPU
from .env_adapter import MADDPGEnvAdapter

def make_maddpg_env(config):
    """
    Create a UAVSwarmEnv wrapped for MADDPG compatibility.
    
    Args:
        config: Environment configuration dict
        
    Returns:
        MADDPGEnvAdapter wrapping UAVSwarmEnv
    """
    env = UAVSwarmEnv(config)
    return MADDPGEnvAdapter(env)

__all__ = [
    'UAVSwarmEnv',
    'UAVSwarmEnvGPU',
    'MADDPGEnvAdapter', 
    'make_maddpg_env'
]
