"""
Action Discretizer for QMIX

Converts continuous action space (5D) to discrete actions for QMIX.
QMIX requires discrete action spaces for value decomposition.

Action Space Discretization:
- Movement (x,y,z): 3D space discretized into 3 levels each = 3^3 = 27 bins
  * -1 to -0.33: Go down (-1)
  * -0.33 to 0.33: Stay (0)
  * 0.33 to 1: Go up (+1)
  
- Offload decision: 3 options
  * < -0.33: Process locally (0)
  * -0.33 to 0.33: Reject task (1)
  * > 0.33: Share with neighbor (2)
  
- CPU frequency: 3 levels
  * < -0.33: Low (1.0 GHz)
  * -0.33 to 0.33: Medium (2.0 GHz)
  * > 0.33: High (3.0 GHz)

Total discrete action space: 27 * 3 * 3 = 243 actions
Simplified to 9 actions: 3^2 for movement magnitude + offload + cpu
"""

import numpy as np
import torch
from typing import Dict, List, Tuple


class ActionDiscretizer:
    """Convert continuous [-1, 1] actions to discrete bins for QMIX."""
    
    # Discretization levels
    MOVEMENT_LEVELS = 3  # Each axis: -1, 0, +1
    OFFLOAD_LEVELS = 3   # Process, reject, share
    CPU_LEVELS = 3       # Low, medium, high
    
    # CPU frequency mappings
    CPU_FREQUENCIES = {
        0: 1.0,   # Low
        1: 2.0,   # Medium
        2: 3.0,   # High
    }
    
    def __init__(self, num_agents: int):
        """
        Initialize discretizer.
        
        Args:
            num_agents: Number of agents (for consistency)
        """
        self.num_agents = num_agents
        # Total action space: 9 (movement magnitude choices) * 3 (offload) * 3 (CPU)
        self.num_actions = 9 * self.OFFLOAD_LEVELS * self.CPU_LEVELS  # 81 total
        
    @property
    def action_space_size(self) -> int:
        """Get total number of discrete actions."""
        return self.num_actions
    
    def continuous_to_discrete(self, continuous_actions: Dict[int, np.ndarray]) -> Dict[int, int]:
        """
        Convert continuous actions to discrete action indices.
        
        Args:
            continuous_actions: Dict mapping agent_id -> action vector (5,)
                [target_x, target_y, target_z, offload_decision, cpu_frequency]
        
        Returns:
            Dict mapping agent_id -> discrete action index (0 to num_actions-1)
        """
        discrete_actions = {}
        
        for agent_id, action in continuous_actions.items():
            # Discretize movement (x, y, z)
            movement_bins = []
            for i in range(3):  # x, y, z
                if action[i] < -0.33:
                    movement_bins.append(0)  # Down/back
                elif action[i] > 0.33:
                    movement_bins.append(2)  # Up/forward
                else:
                    movement_bins.append(1)  # Stay
            
            # Convert movement bins to index (3^3 = 27 possibilities, but we use 9)
            # Simplified: only consider (x,y) movement, z is implicit
            movement_idx = movement_bins[0] * 3 + movement_bins[1]  # 0-8
            
            # Discretize offload decision
            if action[3] < -0.33:
                offload_idx = 0  # Process locally
            elif action[3] > 0.33:
                offload_idx = 2  # Share with neighbor
            else:
                offload_idx = 1  # Reject task
            
            # Discretize CPU frequency
            if action[4] < -0.33:
                cpu_idx = 0  # Low
            elif action[4] > 0.33:
                cpu_idx = 2  # High
            else:
                cpu_idx = 1  # Medium
            
            # Combine into single action index
            # Index = movement (9) + offload (3) + cpu (3)
            discrete_idx = (movement_idx * self.OFFLOAD_LEVELS * self.CPU_LEVELS +
                           offload_idx * self.CPU_LEVELS +
                           cpu_idx)
            
            discrete_actions[agent_id] = discrete_idx
        
        return discrete_actions
    
    def discrete_to_continuous(self, discrete_actions: Dict[int, int]) -> Dict[int, np.ndarray]:
        """
        Convert discrete actions back to continuous control signals.
        """
        continuous_actions = {}
        for agent_id, action_idx in discrete_actions.items():
            continuous_actions[agent_id] = self._single_discrete_to_continuous(action_idx)
        return continuous_actions

    def _single_discrete_to_continuous(self, action_idx: int) -> np.ndarray:
        cpu_idx = action_idx % self.CPU_LEVELS
        offload_idx = (action_idx // self.CPU_LEVELS) % self.OFFLOAD_LEVELS
        movement_idx = action_idx // (self.OFFLOAD_LEVELS * self.CPU_LEVELS)
        
        move_x = (movement_idx // 3) - 1
        move_y = (movement_idx % 3) - 1
        
        return np.array([
            float(move_x) * 0.7,
            float(move_y) * 0.7,
            0.0,
            float(offload_idx - 1) * 0.5,
            float(cpu_idx - 1) * 0.7,
        ], dtype=np.float32)

    def discrete_to_continuous_tensor(self, action_indices: torch.Tensor) -> torch.Tensor:
        """
        Vectorized conversion from discrete indices to continuous (N, 5) tensor.
        Args:
            action_indices: (N,) or (B, N) long tensor
        """
        dev = action_indices.device
        
        cpu_idx = action_indices % self.CPU_LEVELS
        offload_idx = (action_indices // self.CPU_LEVELS) % self.OFFLOAD_LEVELS
        movement_idx = action_indices // (self.OFFLOAD_LEVELS * self.CPU_LEVELS)
        
        move_x = (movement_idx // 3).float() - 1.0
        move_y = (movement_idx % 3).float() - 1.0
        
        # Reconstruct (..., 5)
        # We can use torch.stack to build the final tensor
        res = torch.stack([
            move_x * 0.7,
            move_y * 0.7,
            torch.zeros_like(move_x),
            (offload_idx.float() - 1.0) * 0.5,
            (cpu_idx.float() - 1.0) * 0.7
        ], dim=-1)
        
        return res
    
    def get_cpu_frequency(self, cpu_idx: int) -> float:
        """
        Get CPU frequency from discrete index.
        
        Args:
            cpu_idx: Discrete CPU level (0=low, 1=medium, 2=high)
        
        Returns:
            float: CPU frequency in GHz
        """
        return self.CPU_FREQUENCIES.get(cpu_idx, 2.0)
    
    def get_action_description(self, action_idx: int) -> str:
        """
        Get human-readable description of a discrete action.
        
        Args:
            action_idx: Discrete action index
        
        Returns:
            str: Description of the action
        """
        # Decode
        cpu_idx = action_idx % self.CPU_LEVELS
        offload_idx = (action_idx // self.CPU_LEVELS) % self.OFFLOAD_LEVELS
        movement_idx = action_idx // (self.OFFLOAD_LEVELS * self.CPU_LEVELS)
        
        # Descriptions
        move_x = (movement_idx // 3) - 1
        move_y = (movement_idx % 3) - 1
        
        movements = {-1: "back", 0: "stay", 1: "forward"}
        offloads = {0: "process", 1: "reject", 2: "share"}
        cpus = {0: "low(1GHz)", 1: "med(2GHz)", 2: "high(3GHz)"}
        
        desc = f"move({movements[move_x]},{movements[move_y]}) "
        desc += f"offload({offloads[offload_idx]}) "
        desc += f"cpu({cpus[cpu_idx]})"
        
        return desc


class UniformActionBatcher:
    """Batch uniform random actions for exploration."""
    
    def __init__(self, discretizer: ActionDiscretizer):
        self.discretizer = discretizer
    
    def sample_random_actions(self, num_agents: int) -> Dict[int, int]:
        """
        Sample random discrete actions for all agents.
        
        Args:
            num_agents: Number of agents
        
        Returns:
            Dict mapping agent_id -> random discrete action
        """
        return {
            agent_id: np.random.randint(0, self.discretizer.action_space_size)
            for agent_id in range(num_agents)
        }
    
    def batch_actions_for_env(self,
                             discrete_actions: Dict[int, int]) -> Dict[int, np.ndarray]:
        """
        Batch discrete actions and convert to continuous for environment.
        
        Args:
            discrete_actions: Dict of discrete actions
        
        Returns:
            Dict of continuous action vectors for environment
        """
        return self.discretizer.discrete_to_continuous(discrete_actions)
