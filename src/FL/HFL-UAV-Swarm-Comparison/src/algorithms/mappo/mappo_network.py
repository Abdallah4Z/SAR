"""
MAPPO-specific neural networks for Multi-Agent PPO.

MAPPO uses:
- Decentralized actor: each agent has its own policy based on local observations
- Centralized critic: shares information across all agents during training
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from typing import Tuple, Optional

from src.algorithms.base.actor_critic import BaseActor, BaseCritic, orthogonal_init


class MAPPOActor(nn.Module):
    """
    MAPPO Actor network - decentralized policy.
    
    Each UAV has its own actor that only sees its local observation.
    During training, actions are sampled from a Gaussian distribution.
    During evaluation, the mean action is used deterministically.
    
    Architecture:
        obs (40,) → FC(256) → Tanh → FC(256) → Tanh → action_mean (5,) → Tanh
        Also outputs learnable log_std (5,) for Gaussian policy
    
    Args:
        obs_dim: Observation dimension (40 for UAV)
        action_dim: Action dimension (5 for UAV)
        hidden_dim: Hidden layer size (default: 256)
    """
    
    def __init__(
        self,
        obs_dim: int = 40,
        action_dim: int = 5,
        hidden_dim: int = 256
    ):
        super().__init__()
        
        self.obs_dim = obs_dim
        self.action_dim = action_dim
        self.hidden_dim = hidden_dim
        
        # Build actor network
        self.fc1 = nn.Linear(obs_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.mean_layer = nn.Linear(hidden_dim, action_dim)
        
        # Learnable log std
        self.log_std = nn.Parameter(
            torch.ones(action_dim) * -0.5,
            requires_grad=True
        )
        
        # Initialize with orthogonal weights
        orthogonal_init(self.fc1, gain=np.sqrt(2))
        orthogonal_init(self.fc2, gain=np.sqrt(2))
        orthogonal_init(self.mean_layer, gain=0.01)  # Small init for output
    
    def forward(self, obs: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Forward pass through actor network.
        
        Args:
            obs: Observation tensor (batch_size, obs_dim) or (obs_dim,)
            
        Returns:
            action_mean: Mean of action distribution (batch_size, action_dim)
            log_std: Log standard deviation (action_dim,) broadcasted
        """
        # Handle single observation
        if obs.dim() == 1:
            obs = obs.unsqueeze(0)
            squeeze = True
        else:
            squeeze = False
        
        # Forward pass
        x = torch.tanh(self.fc1(obs))
        x = torch.tanh(self.fc2(x))
        action_mean = torch.tanh(self.mean_layer(x))  # Bounded to [-1, 1]
        
        # Clamp log_std for stability: std ∈ [0.135, 1.0]
        log_std = torch.clamp(self.log_std, -2.0, 0.0)
        
        if squeeze:
            action_mean = action_mean.squeeze(0)
        
        return action_mean, log_std
    
    def get_action(
        self,
        obs: torch.Tensor,
        deterministic: bool = False
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Sample action from policy and compute log probability.
        
        Args:
            obs: Observation tensor (batch_size, obs_dim) or (obs_dim,)
            deterministic: If True, return mean action (evaluation mode)
            
        Returns:
            action: Sampled action (batch_size, action_dim)
            log_prob: Log probability of action (batch_size,)
            entropy: Entropy of distribution (batch_size,)
        """
        action_mean, log_std = self.forward(obs)
        
        # Handle single observation
        if action_mean.dim() == 1:
            action_mean = action_mean.unsqueeze(0)
            squeeze = True
        else:
            squeeze = False
        
        if deterministic:
            # Deterministic evaluation
            action = action_mean
            log_prob = torch.zeros(action.shape[0], device=obs.device)
            entropy = torch.zeros(action.shape[0], device=obs.device)
        else:
            # Stochastic sampling
            std = torch.exp(log_std)
            dist = torch.distributions.Normal(action_mean, std)
            action = dist.sample()
            
            # Clamp to [-1, 1] (shouldn't be needed with tanh, but safe)
            action = torch.clamp(action, -1.0, 1.0)
            
            # Compute log probability and entropy
            log_prob = dist.log_prob(action).sum(dim=-1)
            entropy = dist.entropy().sum(dim=-1)
        
        if squeeze:
            action = action.squeeze(0)
            log_prob = log_prob.squeeze(0)
            entropy = entropy.squeeze(0)
        
        return action, log_prob, entropy
    
    def evaluate_actions(
        self,
        obs: torch.Tensor,
        actions: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Evaluate log probability and entropy of given actions.
        
        Used during PPO updates to compute importance sampling ratio.
        
        Args:
            obs: Observation tensor (batch_size, obs_dim)
            actions: Action tensor (batch_size, action_dim)
            
        Returns:
            log_prob: Log probability of actions (batch_size,)
            entropy: Entropy of distribution (batch_size,)
        """
        action_mean, log_std = self.forward(obs)
        std = torch.exp(log_std)
        
        dist = torch.distributions.Normal(action_mean, std)
        log_prob = dist.log_prob(actions).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)
        
        return log_prob, entropy


class MAPPOCritic(nn.Module):
    """
    MAPPO Critic network - centralized value function.
    
    The critic sees the global state (all agents' observations) during training.
    This allows it to evaluate actions in the context of what other agents are doing,
    reducing variance in policy gradient estimates.
    
    Key Design: CTDE (Centralized Training, Decentralized Execution)
        - Training: Critic uses global state for accurate value estimates
        - Execution: Actor only needs local observations (decentralized)
    
    Architecture:
        global_obs (obs_dim * num_agents,) → FC(256) → Tanh → FC(256) → Tanh → V(s) (1,)
    
    Args:
        obs_dim: Single agent observation dimension (40)
        num_agents: Number of agents in the system (4, 10, 20, 50, 100)
        hidden_dim: Hidden layer size (default: 256)
    """
    
    def __init__(
        self,
        obs_dim: int = 40,
        num_agents: int = 4,
        hidden_dim: int = 256
    ):
        super().__init__()
        
        self.obs_dim = obs_dim
        self.num_agents = num_agents
        self.input_dim = obs_dim * num_agents  # Global state dimension
        self.hidden_dim = hidden_dim
        
        # Build critic network
        self.fc1 = nn.Linear(self.input_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.value_layer = nn.Linear(hidden_dim, 1)
        
        # Initialize with orthogonal weights
        orthogonal_init(self.fc1, gain=np.sqrt(2))
        orthogonal_init(self.fc2, gain=np.sqrt(2))
        orthogonal_init(self.value_layer, gain=1.0)
    
    def forward(self, global_obs: torch.Tensor) -> torch.Tensor:
        """
        Forward pass through critic network.
        
        Args:
            global_obs: Global state tensor (batch_size, obs_dim * num_agents)
                       Concatenation of all agents' observations
            
        Returns:
            value: State value (batch_size, 1)
        """
        x = torch.tanh(self.fc1(global_obs))
        x = torch.tanh(self.fc2(x))
        value = self.value_layer(x)
        return value
    
    def get_value(self, global_obs: torch.Tensor) -> torch.Tensor:
        """
        Get value estimate for global state.
        
        Args:
            global_obs: Global state tensor (batch_size, obs_dim * num_agents)
            
        Returns:
            value: State value (batch_size,) - squeezed for easier use
        """
        return self.forward(global_obs).squeeze(-1)


if __name__ == '__main__':
    """Smoke test for MAPPO networks."""
    
    print("Testing MAPPO networks...\n")
    
    # Test configuration
    obs_dim = 40
    action_dim = 5
    num_agents = 4
    batch_size = 16
    
    # Test MAPPOActor
    print("Testing MAPPOActor...")
    actor = MAPPOActor(obs_dim=obs_dim, action_dim=action_dim, hidden_dim=256)
    
    # Test forward pass
    obs = torch.randn(batch_size, obs_dim)
    action_mean, log_std = actor.forward(obs)
    assert action_mean.shape == (batch_size, action_dim), \
        f"Expected shape ({batch_size}, {action_dim}), got {action_mean.shape}"
    assert log_std.shape == (action_dim,), \
        f"Expected shape ({action_dim},), got {log_std.shape}"
    assert (action_mean >= -1.0).all() and (action_mean <= 1.0).all(), \
        "Action means not bounded to [-1, 1]"
    print(f"✓ MAPPOActor forward: {obs.shape} → mean {action_mean.shape}, log_std {log_std.shape}")
    print(f"  Action mean range: [{action_mean.min():.3f}, {action_mean.max():.3f}]")
    
    # Test get_action
    action, log_prob, entropy = actor.get_action(obs, deterministic=False)
    assert action.shape == (batch_size, action_dim), \
        f"Expected shape ({batch_size}, {action_dim}), got {action.shape}"
    assert log_prob.shape == (batch_size,), \
        f"Expected shape ({batch_size},), got {log_prob.shape}"
    assert entropy.shape == (batch_size,), \
        f"Expected shape ({batch_size},), got {entropy.shape}"
    print(f"✓ get_action: action {action.shape}, log_prob {log_prob.shape}, entropy {entropy.shape}")
    print(f"  Log prob range: [{log_prob.min():.3f}, {log_prob.max():.3f}]")
    print(f"  Entropy range: [{entropy.min():.3f}, {entropy.max():.3f}]")
    
    # Test deterministic action
    action_det, _, _ = actor.get_action(obs, deterministic=True)
    assert torch.allclose(action_det, action_mean, atol=1e-6), \
        "Deterministic action should match mean"
    print("✓ Deterministic action matches mean")
    
    # Test single observation
    single_obs = torch.randn(obs_dim)
    single_action, single_log_prob, single_entropy = actor.get_action(single_obs)
    assert single_action.shape == (action_dim,), \
        f"Expected shape ({action_dim},), got {single_action.shape}"
    print(f"✓ Single observation: {single_obs.shape} → {single_action.shape}")
    
    # Test evaluate_actions
    actions = torch.randn(batch_size, action_dim)
    log_prob_eval, entropy_eval = actor.evaluate_actions(obs, actions)
    assert log_prob_eval.shape == (batch_size,), \
        f"Expected shape ({batch_size},), got {log_prob_eval.shape}"
    assert entropy_eval.shape == (batch_size,), \
        f"Expected shape ({batch_size},), got {entropy_eval.shape}"
    print(f"✓ evaluate_actions: log_prob {log_prob_eval.shape}, entropy {entropy_eval.shape}")
    
    # Test MAPPOCritic
    print("\nTesting MAPPOCritic...")
    critic = MAPPOCritic(obs_dim=obs_dim, num_agents=num_agents, hidden_dim=256)
    
    # Create global observation (concatenate all agents)
    global_obs = torch.randn(batch_size, obs_dim * num_agents)
    value = critic.forward(global_obs)
    assert value.shape == (batch_size, 1), \
        f"Expected shape ({batch_size}, 1), got {value.shape}"
    print(f"✓ MAPPOCritic forward: {global_obs.shape} → {value.shape}")
    print(f"  Value range: [{value.min():.3f}, {value.max():.3f}]")
    
    # Test get_value
    value_squeezed = critic.get_value(global_obs)
    assert value_squeezed.shape == (batch_size,), \
        f"Expected shape ({batch_size},), got {value_squeezed.shape}"
    print(f"✓ get_value: {global_obs.shape} → {value_squeezed.shape}")
    
    # Test with different agent counts
    print("\nTesting scalability across different swarm sizes...")
    for n in [5, 10, 20, 50, 100]:
        critic_n = MAPPOCritic(obs_dim=obs_dim, num_agents=n, hidden_dim=256)
        global_obs_n = torch.randn(8, obs_dim * n)
        value_n = critic_n.get_value(global_obs_n)
        assert value_n.shape == (8,), f"Failed for {n} agents"
        print(f"✓ {n} agents: input ({obs_dim * n},) → value (8,)")
    
    print("\n✅ All MAPPO network tests passed!")
