"""
QMIX Network Architecture

Implements the QMIX value decomposition network:
- Individual Q-networks (one per agent)
- Mixing network (combines individual Q-values with monotonicity constraint)

Key features:
- Dueling architecture for better value estimates
- Target networks for stability
- Monotonic mixing network for theoretical guarantees
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import Tuple, Dict


class IndividualQNetwork(nn.Module):
    """
    Individual agent Q-network for QMIX.
    
    Input: Agent's local observation (obs_dim,)
    Output: Q-values for each discrete action (num_actions,)
    """
    
    def __init__(self, obs_dim: int, num_actions: int, hidden_dim: int = 256):
        """
        Initialize individual Q-network.
        
        Args:
            obs_dim: Observation dimension
            num_actions: Number of discrete actions
            hidden_dim: Hidden layer dimension
        """
        super().__init__()
        
        self.obs_dim = obs_dim
        self.num_actions = num_actions
        self.hidden_dim = hidden_dim
        
        # Dueling network: value and advantage streams
        self.fc1 = nn.Linear(obs_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        
        # Value stream
        self.value_fc = nn.Linear(hidden_dim, 1)
        
        # Advantage stream
        self.advantage_fc = nn.Linear(hidden_dim, num_actions)
        
        # Initialize weights
        for layer in [self.fc1, self.fc2, self.value_fc, self.advantage_fc]:
            nn.init.orthogonal_(layer.weight, gain=torch.nn.init.calculate_gain('relu'))
            nn.init.constant_(layer.bias, 0)
    
    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        """
        Forward pass to compute Q-values.
        
        Args:
            obs: Observation tensor (batch_size, obs_dim)
        
        Returns:
            Q-values (batch_size, num_actions)
        """
        x = F.relu(self.fc1(obs))
        x = F.relu(self.fc2(x))
        
        # Dueling decomposition
        value = self.value_fc(x)  # (batch_size, 1)
        advantages = self.advantage_fc(x)  # (batch_size, num_actions)
        
        # Combine: Q = V + (A - mean(A))
        mean_advantage = advantages.mean(dim=1, keepdim=True)
        q_values = value + (advantages - mean_advantage)
        
        return q_values


class MixingNetwork(nn.Module):
    """
    QMIX Mixing Network for value decomposition.
    
    Combines individual agent Q-values into a joint Q-value.
    Constraint: Monotonicity - mixing weights must be non-negative.
    """
    
    def __init__(self,
                 num_agents: int,
                 num_actions: int,
                 state_dim: int,
                 hidden_dim: int = 32):
        """
        Initialize mixing network.
        
        Args:
            num_agents: Number of agents
            num_actions: Number of discrete actions per agent
            state_dim: Global state dimension (obs_dim * num_agents)
            hidden_dim: Hidden dimension for mixing network
        """
        super().__init__()
        
        self.num_agents = num_agents
        self.num_actions = num_actions
        self.state_dim = state_dim
        self.hidden_dim = hidden_dim
        
        # Hypernetwork for mixing weight generation
        # Input: global state, Output: weights for individual Q-values
        self.hypernetwork = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, num_agents)
        )
        
        # Initialize to output positive weights
        nn.init.orthogonal_(self.hypernetwork[0].weight)
        nn.init.constant_(self.hypernetwork[0].bias, 0)
        nn.init.orthogonal_(self.hypernetwork[2].weight, gain=0.01)
        nn.init.constant_(self.hypernetwork[2].bias, 0)
        
        # Bias term (learnable offset)
        self.bias = nn.Linear(state_dim, 1)
        nn.init.orthogonal_(self.bias.weight)
        nn.init.constant_(self.bias.bias, 0)
    
    def forward(self, chosen_action_q_values: torch.Tensor,
                global_state: torch.Tensor) -> torch.Tensor:
        """
        Mix individual chosen-action Q-values into joint Q_tot.

        QMIX monotonicity constraint: dQ_tot/dQ_i >= 0, enforced by
        using abs() on the mixing weights.

        Args:
            chosen_action_q_values: Per-agent Q(o_i, a_i) scalars
                                    (batch_size, num_agents)
            global_state: Global state (batch_size, state_dim)

        Returns:
            Q_tot: Joint Q-value scalar (batch_size, 1)
        """
        # Generate mixing weights (monotonic positive constraint)
        mixing_weights = torch.abs(self.hypernetwork(global_state))  # (batch_size, num_agents)

        # Weighted sum: Q_tot = sum_i (w_i * Q_i(o_i, a_i)) + bias
        weighted_q = (chosen_action_q_values * mixing_weights).sum(dim=1, keepdim=True)  # (batch_size, 1)

        # Add bias term
        bias = self.bias(global_state)  # (batch_size, 1)
        joint_q = weighted_q + bias  # (batch_size, 1)

        return joint_q


class QMIXNetwork(nn.Module):
    """
    Complete QMIX network combining individual Q-networks and mixing network.
    """
    
    def __init__(self,
                 num_agents: int,
                 obs_dim: int,
                 num_actions: int,
                 hidden_dim: int = 256,
                 mixing_hidden_dim: int = 32):
        """
        Initialize QMIX network.
        
        Args:
            num_agents: Number of agents
            obs_dim: Observation dimension per agent
            num_actions: Number of discrete actions
            hidden_dim: Hidden dimension for individual Q-networks
            mixing_hidden_dim: Hidden dimension for mixing network
        """
        super().__init__()
        
        self.num_agents = num_agents
        self.obs_dim = obs_dim
        self.num_actions = num_actions
        self.hidden_dim = hidden_dim
        
        # Individual Q-networks (one per agent)
        self.individual_q_networks = nn.ModuleList([
            IndividualQNetwork(obs_dim, num_actions, hidden_dim)
            for _ in range(num_agents)
        ])
        
        # Mixing network
        state_dim = obs_dim * num_agents
        self.mixing_network = MixingNetwork(
            num_agents, num_actions, state_dim, mixing_hidden_dim
        )
    
    def forward(self, observations: torch.Tensor,
                global_state: torch.Tensor,
                actions: torch.Tensor = None) -> tuple:
        """
        Forward pass for QMIX network.

        Args:
            observations: Local observations (batch_size, num_agents, obs_dim)
            global_state: Global state (batch_size, state_dim)
            actions: Joint actions (batch_size, num_agents) of type LongTensor
                     If None, returns individual Q-value matrices without mixing.

        Returns:
            If actions provided: (q_tot (batch_size, 1), individual_q_values (batch_size, num_agents, num_actions))
            If actions is None:  (None, individual_q_values (batch_size, num_agents, num_actions))
        """
        # Get individual Q-values from each agent's network
        individual_q_values = []
        for agent_id, q_network in enumerate(self.individual_q_networks):
            agent_obs = observations[:, agent_id, :]  # (batch_size, obs_dim)
            agent_q = q_network(agent_obs)  # (batch_size, num_actions)
            individual_q_values.append(agent_q)

        # Stack individual Q-values
        individual_q_values = torch.stack(individual_q_values, dim=1)  # (batch_size, num_agents, num_actions)

        if actions is not None:
            # Extract chosen-action Q-values for each agent: Q_i(o_i, a_i)
            chosen_q = individual_q_values.gather(
                2, actions.unsqueeze(2)
            ).squeeze(2)  # (batch_size, num_agents)

            # Mix into joint Q_tot
            q_tot = self.mixing_network(chosen_q, global_state)  # (batch_size, 1)
            return q_tot, individual_q_values
        else:
            return None, individual_q_values
    
    def get_q_values(self, observations: torch.Tensor) -> Dict[int, torch.Tensor]:
        """
        Get individual Q-values for each agent (for exploration).
        
        Args:
            observations: Local observations (batch_size, num_agents, obs_dim)
        
        Returns:
            Dict mapping agent_id -> Q-values
        """
        q_dict = {}
        for agent_id, q_network in enumerate(self.individual_q_networks):
            agent_obs = observations[:, agent_id, :]
            q_dict[agent_id] = q_network(agent_obs)
        return q_dict


def create_qmix_network(num_agents: int,
                       obs_dim: int,
                       num_actions: int,
                       hidden_dim: int = 256) -> Tuple[QMIXNetwork, QMIXNetwork]:
    """
    Create QMIX network and target network.
    
    Args:
        num_agents: Number of agents
        obs_dim: Observation dimension
        num_actions: Number of discrete actions
        hidden_dim: Hidden dimension
    
    Returns:
        Tuple of (network, target_network)
    """
    network = QMIXNetwork(num_agents, obs_dim, num_actions, hidden_dim)
    target_network = QMIXNetwork(num_agents, obs_dim, num_actions, hidden_dim)
    
    # Copy weights
    target_network.load_state_dict(network.state_dict())
    
    # Freeze target network
    for param in target_network.parameters():
        param.requires_grad = False
    
    return network, target_network
