"""
Reusable neural network components for all MARL algorithms.

This module provides base actor-critic architectures that can be used across
different algorithms (MAPPO, MADDPG, PPO, SAC, QMIX). All networks use:
- Orthogonal initialization for stable training
- Tanh activations for bounded action spaces
- Modular design for easy customization
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from typing import List, Tuple, Optional


def orthogonal_init(layer: nn.Module, gain: float = 1.0):
    """
    Apply orthogonal initialization to a neural network layer.
    
    Orthogonal initialization helps with gradient flow and stable training,
    especially important for RL where training can be unstable.
    
    Args:
        layer: PyTorch layer (nn.Linear or nn.Conv2d)
        gain: Scaling factor for the weights
              - Use sqrt(2) ≈ 1.414 for hidden layers with ReLU/Tanh
              - Use 0.01 for output layers (small initial policy changes)
              
    Reference:
        Exact solutions to the nonlinear dynamics of learning in deep linear
        neural networks - Saxe et al. 2013
    """
    if isinstance(layer, (nn.Linear, nn.Conv2d)):
        nn.init.orthogonal_(layer.weight, gain=gain)
        if layer.bias is not None:
            nn.init.constant_(layer.bias, 0.0)


class MLPNetwork(nn.Module):
    """
    Multi-Layer Perceptron with orthogonal initialization.
    
    Standard feedforward network used as building block for actors and critics.
    
    Args:
        input_dim: Input feature dimension
        output_dim: Output dimension
        hidden_dims: List of hidden layer sizes (default: [256, 256])
        activation: Activation function name ('tanh', 'relu', 'elu')
        output_activation: Optional output activation ('tanh', None)
    """
    
    def __init__(
        self,
        input_dim: int,
        output_dim: int,
        hidden_dims: List[int] = [256, 256],
        activation: str = 'tanh',
        output_activation: Optional[str] = None
    ):
        super().__init__()
        
        self.input_dim = input_dim
        self.output_dim = output_dim
        self.hidden_dims = hidden_dims
        
        # Select activation function
        if activation == 'tanh':
            self.activation = nn.Tanh()
        elif activation == 'relu':
            self.activation = nn.ReLU()
        elif activation == 'elu':
            self.activation = nn.ELU()
        else:
            raise ValueError(f"Unknown activation: {activation}")
        
        # Select output activation
        if output_activation == 'tanh':
            self.output_activation = nn.Tanh()
        elif output_activation is None:
            self.output_activation = nn.Identity()
        else:
            raise ValueError(f"Unknown output activation: {output_activation}")
        
        # Build layers
        layers = []
        prev_dim = input_dim
        
        for hidden_dim in hidden_dims:
            layer = nn.Linear(prev_dim, hidden_dim)
            orthogonal_init(layer, gain=np.sqrt(2))  # sqrt(2) for hidden layers
            layers.extend([layer, self.activation])
            prev_dim = hidden_dim
        
        # Output layer with small initialization
        output_layer = nn.Linear(prev_dim, output_dim)
        orthogonal_init(output_layer, gain=0.01)  # Small init for output
        layers.append(output_layer)
        
        if output_activation is not None:
            layers.append(self.output_activation)
        
        self.network = nn.Sequential(*layers)
    
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Forward pass through the network.
        
        Args:
            x: Input tensor of shape (batch_size, input_dim)
            
        Returns:
            Output tensor of shape (batch_size, output_dim)
        """
        return self.network(x)


class BaseActor(nn.Module):
    """
    Base actor network for continuous action spaces.
    
    Outputs a Gaussian policy: mean and log_std for each action dimension.
    The final action is sampled from N(mean, exp(log_std)).
    
    For UAV environment:
        - Input: observation (40,)
        - Output: action_mean (5,) with tanh activation → values in [-1, 1]
        - Also outputs: log_std (5,) as learnable parameters
    
    Args:
        obs_dim: Observation space dimension (40 for UAV)
        action_dim: Action space dimension (5 for UAV)
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
        
        # Mean network: obs → action_mean with tanh
        self.mean_net = MLPNetwork(
            input_dim=obs_dim,
            output_dim=action_dim,
            hidden_dims=[hidden_dim, hidden_dim],
            activation='tanh',
            output_activation='tanh'  # Enforce [-1, 1] bounds
        )
        
        # Log standard deviation as learnable parameter
        # Initialize to -0.5 → std ≈ 0.6 (reasonable exploration)
        self.log_std = nn.Parameter(
            torch.ones(action_dim) * -0.5,
            requires_grad=True
        )
    
    def forward(
        self, 
        obs: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Forward pass: compute action mean and log_std.
        
        Args:
            obs: Observation tensor (batch_size, obs_dim)
            
        Returns:
            action_mean: Mean of action distribution (batch_size, action_dim)
            log_std: Log standard deviation (action_dim,) broadcasted
        """
        action_mean = self.mean_net(obs)
        
        # Clamp log_std to [-2, 0] for stability
        # This means std ∈ [0.135, 1.0]
        log_std = torch.clamp(self.log_std, -2.0, 0.0)
        
        return action_mean, log_std
    
    def sample_action(
        self,
        obs: torch.Tensor,
        deterministic: bool = False
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Sample action from the policy distribution.
        
        Args:
            obs: Observation tensor (batch_size, obs_dim)
            deterministic: If True, return mean action without noise
            
        Returns:
            action: Sampled action (batch_size, action_dim)
            log_prob: Log probability of the action (batch_size,)
            entropy: Entropy of the distribution (batch_size,)
        """
        action_mean, log_std = self.forward(obs)
        
        if deterministic:
            # Return mean action during evaluation
            action = action_mean
            log_prob = torch.zeros(obs.shape[0], device=obs.device)
            entropy = torch.zeros(obs.shape[0], device=obs.device)
        else:
            # Sample from Gaussian distribution
            std = torch.exp(log_std)
            dist = torch.distributions.Normal(action_mean, std)
            action = dist.sample()
            
            # Clamp action to [-1, 1] (shouldn't be needed with tanh, but safe)
            action = torch.clamp(action, -1.0, 1.0)
            
            # Compute log probability and entropy
            log_prob = dist.log_prob(action).sum(dim=-1)
            entropy = dist.entropy().sum(dim=-1)
        
        return action, log_prob, entropy


class BaseCritic(nn.Module):
    """
    Base critic network for value function estimation.
    
    For centralized training (MAPPO):
        - Input: global state (obs_dim * num_agents)
        - Output: scalar value V(global_state)
    
    For decentralized training (PPO, SAC):
        - Input: local observation (obs_dim)
        - Output: scalar value V(obs)
    
    Args:
        input_dim: Input dimension (obs_dim for local, obs_dim * num_agents for global)
        hidden_dim: Hidden layer size (default: 256)
    """
    
    def __init__(
        self,
        input_dim: int,
        hidden_dim: int = 256
    ):
        super().__init__()
        
        self.input_dim = input_dim
        
        # Value network: input → scalar value
        self.value_net = MLPNetwork(
            input_dim=input_dim,
            output_dim=1,
            hidden_dims=[hidden_dim, hidden_dim],
            activation='tanh',
            output_activation=None  # No activation on value output
        )
    
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Forward pass: compute state value.
        
        Args:
            x: State tensor (batch_size, input_dim)
            
        Returns:
            value: State value (batch_size, 1)
        """
        return self.value_net(x)
    
    def get_value(self, x: torch.Tensor) -> torch.Tensor:
        """
        Get value estimate for a state.
        
        Args:
            x: State tensor (batch_size, input_dim)
            
        Returns:
            value: State value (batch_size,) - squeezed
        """
        return self.forward(x).squeeze(-1)


if __name__ == '__main__':
    """Smoke test for actor-critic networks."""
    
    print("Testing actor-critic networks...\n")
    
    # Test MLPNetwork
    print("Testing MLPNetwork...")
    mlp = MLPNetwork(input_dim=10, output_dim=5, hidden_dims=[64, 64])
    x = torch.randn(32, 10)
    y = mlp(x)
    assert y.shape == (32, 5), f"Expected shape (32, 5), got {y.shape}"
    print(f"✓ MLPNetwork: {x.shape} → {y.shape}")
    
    # Test BaseActor
    print("\nTesting BaseActor...")
    actor = BaseActor(obs_dim=40, action_dim=5, hidden_dim=256)
    obs = torch.randn(16, 40)
    
    # Test forward pass
    action_mean, log_std = actor.forward(obs)
    assert action_mean.shape == (16, 5), f"Expected shape (16, 5), got {action_mean.shape}"
    assert log_std.shape == (5,), f"Expected shape (5,), got {log_std.shape}"
    assert (action_mean >= -1.0).all() and (action_mean <= 1.0).all(), "Actions not in [-1, 1]"
    print(f"✓ BaseActor forward: {obs.shape} → mean {action_mean.shape}, log_std {log_std.shape}")
    print(f"  Action mean range: [{action_mean.min():.3f}, {action_mean.max():.3f}]")
    
    # Test action sampling
    action, log_prob, entropy = actor.sample_action(obs, deterministic=False)
    assert action.shape == (16, 5), f"Expected shape (16, 5), got {action.shape}"
    assert log_prob.shape == (16,), f"Expected shape (16,), got {log_prob.shape}"
    assert entropy.shape == (16,), f"Expected shape (16,), got {entropy.shape}"
    print(f"✓ BaseActor sampling: action {action.shape}, log_prob {log_prob.shape}, entropy {entropy.shape}")
    print(f"  Log prob range: [{log_prob.min():.3f}, {log_prob.max():.3f}]")
    print(f"  Entropy range: [{entropy.min():.3f}, {entropy.max():.3f}]")
    
    # Test deterministic sampling
    action_det, _, _ = actor.sample_action(obs, deterministic=True)
    assert torch.allclose(action_det, action_mean), "Deterministic action should match mean"
    print("✓ Deterministic sampling matches mean")
    
    # Test BaseCritic
    print("\nTesting BaseCritic...")
    critic = BaseCritic(input_dim=160, hidden_dim=256)  # 40 * 4 agents = 160
    global_obs = torch.randn(16, 160)
    value = critic(global_obs)
    assert value.shape == (16, 1), f"Expected shape (16, 1), got {value.shape}"
    print(f"✓ BaseCritic: {global_obs.shape} → {value.shape}")
    
    value_squeezed = critic.get_value(global_obs)
    assert value_squeezed.shape == (16,), f"Expected shape (16,), got {value_squeezed.shape}"
    print(f"✓ BaseCritic get_value: {global_obs.shape} → {value_squeezed.shape}")
    print(f"  Value range: [{value_squeezed.min():.3f}, {value_squeezed.max():.3f}]")
    
    # Test orthogonal initialization
    print("\nTesting orthogonal initialization...")
    layer = nn.Linear(10, 10)
    orthogonal_init(layer, gain=1.0)
    weights = layer.weight.data
    # Check if W @ W^T ≈ I (orthogonal property)
    product = weights @ weights.T
    identity = torch.eye(10)
    error = (product - identity).abs().max()
    print(f"✓ Orthogonal init error: {error:.6f} (should be < 0.01)")
    assert error < 0.01, "Orthogonal initialization failed"
    
    print("\n✅ All actor-critic network tests passed!")
