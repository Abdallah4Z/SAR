"""
Base agent interface for all RL algorithms in the HFL-UAV-Swarm project.

This module provides the abstract base class that all MARL agents (MAPPO, MADDPG, 
QMIX, PPO, SAC) must inherit from, ensuring consistent API for federated learning.
"""

from abc import ABC, abstractmethod
import numpy as np
import torch
from typing import Dict, List, Optional, Any
import os


class BaseAgent(ABC):
    """
    Abstract base class for all RL agents in the federated learning framework.
    
    All MARL algorithms (MAPPO, MADDPG, QMIX, PPO, SAC) must inherit from this class
    and implement the required abstract methods. This ensures a consistent interface
    for federated learning aggregation and training.
    
    Key Design Principles:
        - Modular: Each agent can be combined with any FL aggregation algorithm
        - FL-Compatible: Provides get/set methods for model weight sharing
        - Flexible: Supports both on-policy and off-policy algorithms
        - Extensible: Easy to add new MARL algorithms by inheriting this class
    
    Attributes:
        agent_id (int): Unique identifier for this agent (UAV ID)
        obs_dim (int): Observation space dimension (40 for UAV environment)
        action_dim (int): Action space dimension (5 for UAV environment)
        config (dict): Algorithm-specific hyperparameters
        training (bool): Training mode flag
    """
    
    def __init__(
        self, 
        agent_id: int, 
        obs_dim: int, 
        action_dim: int, 
        config: dict
    ):
        """
        Initialize base agent.
        
        Args:
            agent_id: Unique identifier for this agent (UAV ID)
            obs_dim: Observation space dimension (40 for UAV environment)
            action_dim: Action space dimension (5 for UAV environment)
            config: Algorithm-specific configuration dictionary
        """
        self.agent_id = agent_id
        self.obs_dim = obs_dim
        self.action_dim = action_dim
        self.config = config if config is not None else {}
        self.training = True
        
    @abstractmethod
    def select_action(
        self, 
        obs: np.ndarray, 
        deterministic: bool = False
    ) -> np.ndarray:
        """
        Select action based on current observation.
        
        This is the core policy execution method. During training, it typically
        uses stochastic policies for exploration. During evaluation, it uses
        deterministic policies for performance measurement.
        
        Args:
            obs: Observation array of shape (obs_dim,) = (40,)
            deterministic: If True, return mean action (for evaluation)
                          If False, sample from action distribution (for training)
                          
        Returns:
            action: Action array of shape (action_dim,) = (5,) in [-1, 1]
                   [target_x, target_y, target_z, offload_decision, cpu_frequency]
        """
        pass
    
    @abstractmethod
    def update(self, batch: dict) -> dict:
        """
        Update policy and value networks from a batch of experience.
        
        This method implements the learning algorithm (PPO, DDPG, Q-learning, etc.)
        and updates network weights based on collected experience.
        
        Args:
            batch: Dictionary containing experience data with keys:
                  - 'obs': Observations (batch_size, obs_dim)
                  - 'actions': Actions (batch_size, action_dim)
                  - 'rewards': Rewards (batch_size,)
                  - 'next_obs': Next observations (batch_size, obs_dim)
                  - 'dones': Done flags (batch_size,)
                  - Additional algorithm-specific fields (log_probs, values, etc.)
                  
        Returns:
            dict: Dictionary of training losses and metrics for logging:
                 {'actor_loss': float, 'critic_loss': float, ...}
        """
        pass
    
    @abstractmethod
    def store_transition(
        self, 
        obs: np.ndarray,
        action: np.ndarray,
        reward: float,
        next_obs: np.ndarray,
        done: bool,
        **kwargs
    ):
        """
        Store a single transition in the agent's replay buffer.
        
        Additional keyword arguments may include:
            - log_prob: Log probability of action (for on-policy methods)
            - value: Value estimate (for advantage computation)
            - info: Additional environment information
        
        Args:
            obs: Current observation (obs_dim,)
            action: Action taken (action_dim,)
            reward: Reward received
            next_obs: Next observation (obs_dim,)
            done: Episode termination flag
            **kwargs: Algorithm-specific additional data
        """
        pass
    
    @abstractmethod
    def get_model_weights(self) -> dict:
        """
        Get actor network weights for federated learning aggregation.
        
        CRITICAL FOR FL: This method extracts ONLY the actor/policy network
        weights that will be shared across agents via federated aggregation.
        The critic/value network is NOT shared (it's centralized in MAPPO).
        
        Returns:
            dict: Actor network state_dict (PyTorch OrderedDict)
                 Compatible with torch.save() and torch.load()
        """
        pass
    
    @abstractmethod
    def set_model_weights(self, weights: dict):
        """
        Load aggregated weights from FL server into actor network.
        
        CRITICAL FOR FL: This method receives the globally aggregated weights
        from the federated server and loads them into the local actor/policy
        network. This synchronizes all agents after each FL round.
        
        Args:
            weights: Actor network state_dict from FL aggregation
        """
        pass
    
    def save_checkpoint(self, path: str):
        """
        Save full agent checkpoint (actor + critic + optimizers).
        
        This saves the complete agent state for later resumption of training.
        Use this for checkpointing during long training runs.
        
        Args:
            path: File path to save checkpoint (.pt or .pth)
        """
        os.makedirs(os.path.dirname(path), exist_ok=True)
        checkpoint = {
            'agent_id': self.agent_id,
            'config': self.config,
            'model_state': self.get_model_weights(),
        }
        torch.save(checkpoint, path)
        
    def load_checkpoint(self, path: str):
        """
        Load full agent checkpoint from file.
        
        Args:
            path: File path to load checkpoint from
        """
        if not os.path.exists(path):
            raise FileNotFoundError(f"Checkpoint not found: {path}")
        checkpoint = torch.load(path)
        self.set_model_weights(checkpoint['model_state'])
        
    def set_train_mode(self):
        """Set agent to training mode (enables exploration, dropout, etc.)"""
        self.training = True
        
    def set_eval_mode(self):
        """Set agent to evaluation mode (disables exploration, dropout, etc.)"""
        self.training = False
        
    def get_agent_id(self) -> int:
        """
        Get this agent's unique identifier.
        
        Returns:
            int: Agent ID (UAV ID in the swarm)
        """
        return self.agent_id


if __name__ == '__main__':
    """Smoke test for BaseAgent interface."""
    
    print("Testing BaseAgent interface...")
    
    # Create a minimal concrete implementation for testing
    class DummyAgent(BaseAgent):
        def __init__(self, agent_id, obs_dim, action_dim, config):
            super().__init__(agent_id, obs_dim, action_dim, config)
            self.model = {'dummy_weight': torch.randn(10, 10)}
            
        def select_action(self, obs, deterministic=False):
            return np.random.randn(self.action_dim).astype(np.float32)
        
        def update(self, batch):
            return {'loss': 0.5}
        
        def store_transition(self, obs, action, reward, next_obs, done, **kwargs):
            pass
        
        def get_model_weights(self):
            return self.model.copy()
        
        def set_model_weights(self, weights):
            self.model = weights.copy()
    
    # Test instantiation
    agent = DummyAgent(agent_id=0, obs_dim=40, action_dim=5, config={'lr': 0.001})
    assert agent.get_agent_id() == 0
    assert agent.obs_dim == 40
    assert agent.action_dim == 5
    print("✓ Agent initialization works")
    
    # Test action selection
    obs = np.random.randn(40).astype(np.float32)
    action = agent.select_action(obs)
    assert action.shape == (5,), f"Expected shape (5,), got {action.shape}"
    print("✓ Action selection works")
    
    # Test FL weight methods
    weights_1 = agent.get_model_weights()
    assert 'dummy_weight' in weights_1
    print("✓ Get model weights works")
    
    new_weights = {'dummy_weight': torch.randn(10, 10)}
    agent.set_model_weights(new_weights)
    weights_2 = agent.get_model_weights()
    assert torch.allclose(weights_2['dummy_weight'], new_weights['dummy_weight'])
    print("✓ Set model weights works")
    
    # Test training modes
    agent.set_train_mode()
    assert agent.training is True
    agent.set_eval_mode()
    assert agent.training is False
    print("✓ Training mode switching works")
    
    print("\n✅ All BaseAgent interface tests passed!")
