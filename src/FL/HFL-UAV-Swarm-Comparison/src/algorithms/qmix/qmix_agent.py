"""
QMIX Agent Implementation

Implements a QMIX agent with Q-learning and value decomposition.
Uses discrete action space and experience replay for stable learning.
"""

import torch
import torch.optim as optim
import numpy as np
from typing import Dict, Tuple
from collections import deque

from src.algorithms.base.base_agent import BaseAgent
from src.algorithms.qmix.qmix_network import QMIXNetwork, create_qmix_network
from src.algorithms.qmix.action_discretizer import ActionDiscretizer
from src.algorithms.maddpg.replay_buffer import ReplayBuffer


class QMIXAgent(BaseAgent):
    """
    QMIX Agent with value decomposition and mixing network.
    
    Features:
    - Discrete action space via action discretizer
    - Off-policy learning with experience replay
    - Target networks for stability
    - Dueling architecture for better value estimates
    - FL-compatible weight methods
    """
    
    def __init__(self,
                 agent_id: int,
                 obs_dim: int,
                 action_dim: int = 5,
                 num_agents: int = 4,
                 num_actions: int = 81,
                 hidden_dim: int = 256,
                 lr: float = 5e-4,
                 gamma: float = 0.99,
                 tau: float = 0.005,
                 buffer_size: int = 100000,
                 batch_size: int = 32,
                 device: str = 'cuda'):
        """
        Initialize QMIX agent.
        
        Args:
            agent_id: Agent identifier
            obs_dim: Observation dimension
            action_dim: Continuous action dimension (for compatibility)
            num_agents: Total number of agents in swarm
            num_actions: Number of discrete actions (typically 81)
            hidden_dim: Hidden dimension for networks
            lr: Learning rate
            gamma: Discount factor
            tau: Target network update rate
            buffer_size: Experience replay buffer size
            batch_size: Mini-batch size for training
            device: Device to use ('cuda' or 'cpu')
        """
        # Build config dict for BaseAgent
        config = {
            'num_agents': num_agents,
            'num_actions': num_actions,
            'hidden_dim': hidden_dim,
            'lr': lr,
            'gamma': gamma,
            'tau': tau,
            'buffer_size': buffer_size,
            'batch_size': batch_size,
            'device': device,
        }
        
        super().__init__(agent_id, obs_dim, action_dim, config)
        
        self.num_actions = num_actions
        self.hidden_dim = hidden_dim
        self.lr = lr
        self.gamma = gamma
        self.tau = tau
        self.batch_size = batch_size
        self.device = device
        self.num_agents = num_agents
        
        # Action discretizer
        self.discretizer = ActionDiscretizer(num_agents)
        assert self.discretizer.action_space_size == num_actions, \
            f"Action space mismatch: {self.discretizer.action_space_size} vs {num_actions}"
        
        # Networks
        self.network, self.target_network = create_qmix_network(
            num_agents, obs_dim, num_actions, hidden_dim
        )
        self.network = self.network.to(device)
        self.target_network = self.target_network.to(device)
        
        # Optimizer
        self.optimizer = optim.Adam(self.network.parameters(), lr=lr)
        
        # Experience replay buffer (Modernized)
        # Note: Each QMIX agent has its own buffer storing only its transitions
        # So num_agents in the buffer should be 1, not the total swarm size
        # But global_obs_dim should still be obs_dim * total_num_agents for QMIX mixer
        self.replay_buffer = ReplayBuffer(
            capacity=buffer_size,
            obs_dim=obs_dim,
            act_dim=1,  # Discrete action
            num_agents=1,  # Individual agent buffer (not shared)
            global_obs_dim=obs_dim * num_agents,  # Full swarm observation for mixer
            device=device
        )
        
        # Epsilon-greedy exploration
        self.epsilon = 1.0
        self.epsilon_decay = 0.995
        self.epsilon_min = 0.05
        
        # Training step counter
        self.train_step = 0
    
    def select_action(self, obs: np.ndarray) -> int:
        # ── VECTORIZED EPSILON-GREEDY ──
        obs_tensor = torch.as_tensor(obs, dtype=torch.float32, device=self.device)
        is_batched = (obs_tensor.dim() > 1)
        batch_size = obs_tensor.shape[0] if is_batched else 1
        
        # 1. Generate random actions (Exploration)
        random_actions = torch.randint(0, self.num_actions, (batch_size,), device=self.device)
        
        # 2. Compute greedy actions (Exploitation)
        with torch.no_grad():
            inference_obs = obs_tensor if is_batched else obs_tensor.unsqueeze(0)
            # Use this agent's network for current policy
            q_values = self.network.individual_q_networks[self.agent_id](inference_obs)
            greedy_actions = q_values.argmax(dim=1)
            
        # 3. Combine using epsilon mask
        epsilon_mask = (torch.rand(batch_size, device=self.device) < self.epsilon)
        actions = torch.where(epsilon_mask, random_actions, greedy_actions)
        
        return actions if is_batched else actions.item()
    
    def select_action_with_value(self, obs: np.ndarray, global_obs: np.ndarray,
                                 deterministic: bool = False) -> Tuple[int, float, float]:
        """
        Select action and return Q-value (for compatibility with trainer).
        
        Args:
            obs: Local observation
            global_obs: Global observation (concatenated)
            deterministic: Whether to use greedy action (for evaluation)
        
        Returns:
            Tuple of (action, q_value, td_target_placeholder)
        """
        if deterministic:
            action = self.select_action_greedy(obs)
        else:
            action = self.select_action(obs)
        
        # Get Q-value
        with torch.no_grad():
            obs_tensor = torch.as_tensor(obs, dtype=torch.float32, device=self.device).unsqueeze(0)
            q_values = self.network.individual_q_networks[self.agent_id](obs_tensor)
            q_value = q_values[0, action].item()
        
        return action, q_value, 0.0
    
    def select_action_greedy(self, obs: np.ndarray) -> int:
        """
        Select best action (greedy, no exploration).
        
        Args:
            obs: Local observation
        
        Returns:
            int: Best discrete action
        """
        with torch.no_grad():
            obs_tensor = torch.as_tensor(obs, dtype=torch.float32, device=self.device).unsqueeze(0)
            q_values = self.network.individual_q_networks[self.agent_id](obs_tensor)
            action = q_values.argmax(dim=1).item()
        
        return action
    
    def store_transition(self, obs, action, reward, next_obs, done, global_obs, next_global_obs):
        """Store transition in replay buffer (Zero-copy)."""
        self.replay_buffer.push(
            obs=obs,
            action=action,
            reward=reward,
            next_obs=next_obs,
            done=done,
            global_obs=global_obs,
            next_global_obs=next_global_obs
        )
    
    def update(self, batch: Dict, is_fl_round: bool = False, global_weights: Dict = None, proximal_mu: float = 0.0) -> Dict:
        """
        Update QMIX agent using experience replay WITH MIXING NETWORK.
        
        This is the correct QMIX implementation that uses the mixing network
        to combine individual Q-values into Q_tot for value decomposition.
        
        Args:
            batch: Batch of transitions (not used, uses internal replay buffer)
            is_fl_round: Whether this is an FL aggregation round
            global_weights: Global model weights for FedProx proximal term
            proximal_mu: Proximal coefficient for FedProx (0 = no regularization)
        
        Returns:
            Dict with training metrics
        """
        if len(self.replay_buffer) < self.batch_size:
            return {'loss': 0.0, 'epsilon': self.epsilon, 'proximal_loss': 0.0}
        
        # Sample mini-batch (Modernized, Zero-copy)
        batch = self.replay_buffer.sample(self.batch_size)
        obs_batch = batch['obs'].squeeze(1)  # Remove num_agents=1 dim: [batch, 1, obs_dim] -> [batch, obs_dim]
        action_batch = batch['actions'].long().squeeze(-1).squeeze(-1)  # [batch, 1, 1] -> [batch]
        reward_batch = batch['rewards'].squeeze(-1).squeeze(-1)  # [batch, 1, 1] -> [batch]
        next_obs_batch = batch['next_obs'].squeeze(1)  # [batch, 1, obs_dim] -> [batch, obs_dim]
        done_batch = batch['dones'].squeeze(-1).squeeze(-1)  # [batch, 1, 1] -> [batch]
        global_obs_batch = batch['global_obs']  # [batch, global_obs_dim] - already correct
        next_global_obs_batch = batch['next_global_obs']  # [batch, global_obs_dim] - already correct
        
        batch_size = obs_batch.shape[0]
        
        # ═══════════════════════════════════════════════════════════════
        # CRITICAL FIX: Use QMIX Mixing Network for Value Decomposition
        # ═══════════════════════════════════════════════════════════════
        
        # Reshape global observations to (batch_size, num_agents, obs_dim)
        global_obs_reshaped = global_obs_batch.view(batch_size, self.num_agents, self.obs_dim)
        next_global_obs_reshaped = next_global_obs_batch.view(batch_size, self.num_agents, self.obs_dim)
        
        # Compute individual Q-values for ALL agents
        individual_q_values = []
        for agent_id in range(self.num_agents):
            agent_obs = global_obs_reshaped[:, agent_id, :]
            q_vals = self.network.individual_q_networks[agent_id](agent_obs)  # (batch_size, num_actions)
            individual_q_values.append(q_vals)
        
        # Stack into (batch_size, num_agents, num_actions)
        individual_q_values = torch.stack(individual_q_values, dim=1)
        
        # Get Q-values for actions taken (for this agent only)
        current_agent_q = individual_q_values[:, self.agent_id, :]  # (batch_size, num_actions)
        chosen_action_q = current_agent_q.gather(1, action_batch.unsqueeze(1)).squeeze(1)  # (batch_size,)
        
        # Get chosen Q-values for ALL agents (each picks its greedy action)
        # The mixing network expects (batch_size, num_agents) — scalar per agent
        all_agents_greedy_actions = individual_q_values.argmax(dim=2)  # (batch_size, num_agents)
        # Override current agent's action with the actual action taken
        all_agents_greedy_actions[:, self.agent_id] = action_batch
        # Gather chosen Q-values: (batch_size, num_agents)
        chosen_q_all = individual_q_values.gather(
            2, all_agents_greedy_actions.unsqueeze(2)
        ).squeeze(2)  # (batch_size, num_agents)
        
        # Use mixing network to compute Q_tot (joint Q-value)
        # This enforces monotonicity: ∂Q_tot/∂Q_i ≥ 0
        q_tot = self.network.mixing_network(chosen_q_all, global_obs_batch)  # (batch_size, 1)
        q_tot_chosen = q_tot.squeeze(1)  # (batch_size,)
        
        # Compute target Q_tot
        with torch.no_grad():
            # Next individual Q-values for ALL agents
            next_individual_q_values = []
            for agent_id in range(self.num_agents):
                next_agent_obs = next_global_obs_reshaped[:, agent_id, :]
                next_q_vals = self.target_network.individual_q_networks[agent_id](next_agent_obs)
                next_individual_q_values.append(next_q_vals)
            
            next_individual_q_values = torch.stack(next_individual_q_values, dim=1)
            
            # Get best next Q-values for all agents: pick greedy action per agent
            best_next_actions = next_individual_q_values.argmax(dim=2)  # (batch_size, num_agents)
            next_chosen_q_all = next_individual_q_values.gather(
                2, best_next_actions.unsqueeze(2)
            ).squeeze(2)  # (batch_size, num_agents)
            
            # Mix next Q-values using target mixing network
            next_q_tot = self.target_network.mixing_network(next_chosen_q_all, next_global_obs_batch)  # (batch_size, 1)
            next_q_tot_max = next_q_tot.squeeze(1)  # (batch_size,)
            
            # TD target
            target_q_tot = reward_batch + self.gamma * next_q_tot_max * (1 - done_batch)
        
        # TD loss (QMIX uses joint Q_tot, not individual Q)
        td_loss = torch.nn.functional.mse_loss(q_tot_chosen, target_q_tot)
        
        # ═══════════════════════════════════════════════════════════════
        # CRITICAL FIX: Add FedProx Proximal Term to Loss
        # ═══════════════════════════════════════════════════════════════
        proximal_loss = 0.0
        if proximal_mu > 0.0 and global_weights is not None:
            # Compute ||w - w_global||²
            for param_name, param in self.network.named_parameters():
                # Find corresponding global weight
                global_param = None
                if param_name in global_weights:
                    global_param = global_weights[param_name]
                else:
                    # Try with prefixes (for compatibility)
                    for prefix in ['individual_q.', 'mixing.']:
                        full_key = f'{prefix}{param_name}'
                        if full_key in global_weights:
                            global_param = global_weights[full_key]
                            break
                
                if global_param is not None:
                    if not isinstance(global_param, torch.Tensor):
                        global_param = torch.tensor(global_param, device=self.device)
                    proximal_loss += ((param - global_param) ** 2).sum()
            
            proximal_loss = (proximal_mu / 2.0) * proximal_loss
        
        # Total loss = TD loss + proximal term
        total_loss = td_loss + proximal_loss
        
        # Update network
        self.optimizer.zero_grad()
        total_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.network.parameters(), max_norm=10.0)
        self.optimizer.step()
        
        # Update target network (soft update)
        self._soft_update_target_network()
        
        # Decay epsilon
        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)
        
        self.train_step += 1
        
        return {
            'loss': total_loss.item(),
            'td_loss': td_loss.item(),
            'proximal_loss': proximal_loss.item() if isinstance(proximal_loss, torch.Tensor) else proximal_loss,
            'epsilon': self.epsilon,
            'train_step': self.train_step,
        }
    
    def _soft_update_target_network(self):
        """Soft update of target network parameters."""
        for param, target_param in zip(self.network.parameters(),
                                      self.target_network.parameters()):
            target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)
    
    def get_model_weights(self) -> Dict:
        """
        Get agent's network weights for FL.
        
        For QMIX, we share:
        - Individual Q-network for this agent
        - Shared mixing network weights
        
        Returns:
            Dict of flattened model weights (compatible with FL aggregators)
        """
        # Flatten the nested structure for compatibility with FL aggregators
        weights = {}
        
        # Individual Q-network weights
        individual_q_state = self.network.individual_q_networks[self.agent_id].state_dict()
        for key, value in individual_q_state.items():
            weights[f'individual_q.{key}'] = value
        
        # Mixing network weights (shared across all agents)
        mixing_state = self.network.mixing_network.state_dict()
        for key, value in mixing_state.items():
            weights[f'mixing.{key}'] = value
        
        return weights
    
    def set_model_weights(self, weights: Dict):
        """
        Set agent's network weights from FL.
        
        Args:
            weights: Dict of flattened model weights from FL aggregator
        """
        # Reconstruct nested dicts from flattened structure
        individual_q_state = {}
        mixing_state = {}
        
        for key, value in weights.items():
            if key.startswith('individual_q.'):
                param_key = key.replace('individual_q.', '')
                individual_q_state[param_key] = value
            elif key.startswith('mixing.'):
                param_key = key.replace('mixing.', '')
                mixing_state[param_key] = value
        
        # Load into networks
        if individual_q_state:
            self.network.individual_q_networks[self.agent_id].load_state_dict(individual_q_state)
            self.target_network.individual_q_networks[self.agent_id].load_state_dict(individual_q_state)
        
        if mixing_state:
            self.network.mixing_network.load_state_dict(mixing_state)
            self.target_network.mixing_network.load_state_dict(mixing_state)
    
    def save_checkpoint(self, filepath: str):
        """Save agent checkpoint."""
        checkpoint = {
            'network_state': self.network.state_dict(),
            'target_network_state': self.target_network.state_dict(),
            'optimizer_state': self.optimizer.state_dict(),
            'epsilon': self.epsilon,
            'train_step': self.train_step,
        }
        torch.save(checkpoint, filepath)
    
    def load_checkpoint(self, filepath: str):
        """Load agent checkpoint."""
        checkpoint = torch.load(filepath, map_location=self.device)
        self.network.load_state_dict(checkpoint['network_state'])
        self.target_network.load_state_dict(checkpoint['target_network_state'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state'])
        self.epsilon = checkpoint.get('epsilon', 0.05)
        self.train_step = checkpoint.get('train_step', 0)
