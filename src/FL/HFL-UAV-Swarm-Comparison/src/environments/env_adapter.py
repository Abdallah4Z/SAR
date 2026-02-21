"""
Environment Adapter for MADDPG Compatibility.

Converts Dict-based UAVSwarmEnv to list-based API expected by MADDPG.
"""

import numpy as np
from typing import List, Tuple, Dict, Any
from gymnasium import spaces


class MADDPGEnvAdapter:
    """
    Adapter that wraps UAVSwarmEnv to provide list-based API for MADDPG.
    
    Converts:
    - Dict observations → List of observations
    - Dict rewards → List of rewards  
    - Dict actions ← List of actions
    - Gymnasium v26+ API (5 returns) → Old gym API (4 returns)
    """
    
    def __init__(self, env):
        """
        Args:
            env: UAVSwarmEnv instance with Dict spaces
        """
        self.env = env
        self.num_agents = env.num_uavs
        
        # Convert Dict spaces to list of spaces
        self.observation_space = [
            env.observation_space[i] for i in range(self.num_agents)
        ]
        self.action_space = [
            env.action_space[i] for i in range(self.num_agents)
        ]
    
    def reset(self, seed=None, options=None) -> List[np.ndarray]:
        """
        Reset environment.
        
        Returns:
            List of observations (one per agent)
        """
        obs_dict, info = self.env.reset(seed=seed, options=options)
        obs_list = [obs_dict[i] for i in range(self.num_agents)]
        return obs_list
    
    def step(self, actions: List[np.ndarray]) -> Tuple[List, List, List, Dict]:
        """
        Execute one step.
        
        Args:
            actions: List of actions (one per agent)
            
        Returns:
            observations: List of observations
            rewards: List of rewards
            dones: List of done flags
            info: Dict with additional info
        """
        # Convert list to dict
        actions_dict = {i: actions[i] for i in range(self.num_agents)}
        
        # Call underlying environment
        obs_dict, rewards_dict, terminated, truncated, info = self.env.step(actions_dict)
        
        # Convert dict outputs to lists
        obs_list = [obs_dict[i] for i in range(self.num_agents)]
        rewards_list = [rewards_dict[i] for i in range(self.num_agents)]
        
        # Combine terminated and truncated into single done signal per agent
        # In MADDPG, all agents share the same episode termination
        done = terminated or truncated
        dones_list = [done] * self.num_agents
        
        return obs_list, rewards_list, dones_list, info
    
    def close(self):
        """Close the environment."""
        if hasattr(self.env, 'close'):
            self.env.close()
    
    def render(self, mode='human'):
        """Render the environment."""
        if hasattr(self.env, 'render'):
            return self.env.render(mode=mode)
    
    @property
    def unwrapped(self):
        """Get the underlying environment."""
        return self.env
