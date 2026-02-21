"""
MAPPO Agent implementation inheriting from BaseAgent.

This module implements a complete MAPPO agent that can be used in the
federated learning framework. The agent manages its own actor-critic networks,
experience buffer, and provides methods for FL weight aggregation.
"""

import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
from typing import Dict, Tuple, Optional
import os

from src.algorithms.base.base_agent import BaseAgent
from src.algorithms.base.buffer import BaseBuffer
from src.algorithms.mappo.mappo_network import MAPPOActor, MAPPOCritic


class MAPPOAgent(BaseAgent):
    """
    Multi-Agent Proximal Policy Optimization (MAPPO) agent.
    
    MAPPO extends PPO to multi-agent settings with centralized training
    and decentralized execution (CTDE):
        - Actor: Decentralized, uses only local observations
        - Critic: Centralized, uses global state during training
        - FL: Only actor weights are shared across agents
    
    Architecture:
        - Actor: obs (40,) → action (5,) in [-1, 1]
        - Critic: global_obs (40 * num_agents,) → value (scalar)
    
    Args:
        agent_id: Unique identifier for this agent (UAV ID)
        obs_dim: Observation dimension (40 for UAV)
        action_dim: Action dimension (5 for UAV)
        num_agents: Total number of agents in the system
        config: Configuration dictionary with hyperparameters
    """
    
    def __init__(
        self,
        agent_id: int,
        obs_dim: int = 40,
        action_dim: int = 5,
        num_agents: int = 4,
        config: Optional[Dict] = None
    ):
        super().__init__(agent_id, obs_dim, action_dim, config)
        
        self.num_agents = num_agents
        
        # Default configuration
        default_config = {
            'hidden_dim': 256,
            'lr_actor': 3e-4,
            'lr_critic': 1e-3,
            'gamma': 0.99,
            'gae_lambda': 0.95,
            'clip_param': 0.2,
            'ppo_epochs': 10,
            'batch_size': 64,
            'entropy_coef': 0.01,
            'value_loss_coef': 0.5,
            'max_grad_norm': 0.5,
        }
        default_config.update(self.config)
        self.config = default_config
        
        # Determine device
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        # Create networks
        self.actor = MAPPOActor(
            obs_dim=obs_dim,
            action_dim=action_dim,
            hidden_dim=self.config['hidden_dim']
        ).to(self.device)
        
        self.critic = MAPPOCritic(
            obs_dim=obs_dim,
            num_agents=num_agents,
            hidden_dim=self.config['hidden_dim']
        ).to(self.device)
        
        # Optimizers
        self.actor_optimizer = optim.Adam(
            self.actor.parameters(),
            lr=self.config['lr_actor']
        )
        self.critic_optimizer = optim.Adam(
            self.critic.parameters(),
            lr=self.config['lr_critic']
        )
        
        # Experience buffer (on-policy, Modernized)
        self.buffer = BaseBuffer(
            buffer_type='onpolicy', 
            max_size=100000,
            obs_dim=obs_dim,
            act_dim=action_dim,
            device=self.device
        )
        
        # Training statistics
        self.update_count = 0
        
    def select_action(
        self,
        obs: np.ndarray,
        deterministic: bool = False
    ) -> np.ndarray:
        """
        Select action based on current observation.
        
        Args:
            obs: Observation array (obs_dim,) = (40,)
            deterministic: If True, return mean action (evaluation)
                          If False, sample from distribution (training)
        
        Returns:
            action: Action array (action_dim,) = (5,) in [-1, 1]
        """
        with torch.no_grad():
            obs_tensor = torch.as_tensor(obs, dtype=torch.float32, device=self.device)
            action, log_prob, entropy = self.actor.get_action(
                obs_tensor, deterministic=deterministic
            )
            
            # Store log_prob for training (if not deterministic)
            if not deterministic and self.training:
                self._last_log_prob = log_prob.item()
            
        # Return GPU tensor - caller decides if numpy needed for logging
        return action
    
    def select_action_with_value(
        self,
        obs: np.ndarray,
        global_obs: np.ndarray,
        deterministic: bool = False
    ) -> Tuple[np.ndarray, float, float]:
        """
        Select action and compute value estimate.
        
        This method is used during rollout to get both action and value
        in a single forward pass for efficiency.
        
        Args:
            obs: Local observation (obs_dim,)
            global_obs: Global state (obs_dim * num_agents,)
            deterministic: If True, return mean action
        
        Returns:
            action: Action array (action_dim,)
            log_prob: Log probability of action
            value: Value estimate V(global_state)
        """
        with torch.no_grad():
            obs_tensor = torch.as_tensor(obs, dtype=torch.float32, device=self.device)
            global_obs_tensor = torch.as_tensor(global_obs, dtype=torch.float32, device=self.device)
            
            # Get action
            action, log_prob, entropy = self.actor.get_action(
                obs_tensor, deterministic=deterministic
            )
            
            # Get value
            value = self.critic.get_value(global_obs_tensor)
            
            log_prob_np = log_prob.item()
            value_np = value.item()
        
        # Return GPU tensor for action - caller decides if numpy needed
        return action, log_prob_np, value_np
    
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
        Store transition in experience buffer.
        
        Args:
            obs: Current observation (obs_dim,)
            action: Action taken (action_dim,)
            reward: Reward received
            next_obs: Next observation (obs_dim,)
            done: Episode termination flag
            **kwargs: Additional data (log_prob, value)
        """
        log_prob = kwargs.get('log_prob', None)
        value = kwargs.get('value', None)
        
        self.buffer.add(
            obs=obs,
            action=action,
            reward=reward,
            next_obs=next_obs,
            done=done,
            log_prob=log_prob,
            value=value
        )
    
    def update(self, batch: dict, global_weights: dict = None, proximal_mu: float = 0.0) -> dict:
        """
        Update actor and critic networks using PPO.

        This method implements the PPO update with clipped objective,
        value function loss, and entropy bonus. When proximal_mu > 0
        (FedProx), adds proximal regularization to prevent local models
        from diverging too far from the global model.

        Args:
            batch: Dictionary with keys:
                - 'obs': Local observations (batch_size, obs_dim)
                - 'global_obs': Global states (batch_size, obs_dim * num_agents)
                - 'actions': Actions (batch_size, action_dim)
                - 'old_log_probs': Log probs from rollout (batch_size,)
                - 'advantages': GAE advantages (batch_size,)
                - 'returns': Target returns (batch_size,)
            global_weights: Global model weights for FedProx proximal term
            proximal_mu: Proximal coefficient for FedProx (0 = no regularization)

        Returns:
            dict: Training metrics {'actor_loss', 'critic_loss', 'entropy', ...}
        """
        # Convert to tensors
        obs = torch.as_tensor(batch['obs'], dtype=torch.float32, device=self.device)
        global_obs = torch.as_tensor(batch['global_obs'], dtype=torch.float32, device=self.device)
        actions = torch.as_tensor(batch['actions'], dtype=torch.float32, device=self.device)
        old_log_probs = torch.as_tensor(batch['old_log_probs'], dtype=torch.float32, device=self.device)
        advantages = torch.as_tensor(batch['advantages'], dtype=torch.float32, device=self.device)
        returns = torch.as_tensor(batch['returns'], dtype=torch.float32, device=self.device)
        
        # Evaluate actions with current policy
        new_log_probs, entropy = self.actor.evaluate_actions(obs, actions)
        
        # Compute importance sampling ratio
        ratio = torch.exp(new_log_probs - old_log_probs)
        
        # Clipped surrogate objective
        surr1 = ratio * advantages
        surr2 = torch.clamp(
            ratio,
            1.0 - self.config['clip_param'],
            1.0 + self.config['clip_param']
        ) * advantages
        actor_loss = -torch.min(surr1, surr2).mean()
        
        # Entropy bonus (encourage exploration)
        entropy_loss = -entropy.mean()
        
        # Total actor loss
        total_actor_loss = actor_loss + self.config['entropy_coef'] * entropy_loss

        # FedProx proximal term: (mu/2) * ||w_local - w_global||^2
        proximal_loss = 0.0
        if proximal_mu > 0.0 and global_weights is not None:
            for name, param in self.actor.named_parameters():
                if name in global_weights:
                    global_param = global_weights[name]
                    if not isinstance(global_param, torch.Tensor):
                        global_param = torch.tensor(global_param, device=self.device)
                    else:
                        global_param = global_param.to(self.device)
                    proximal_loss += ((param - global_param.detach()) ** 2).sum()
            total_actor_loss = total_actor_loss + (proximal_mu / 2.0) * proximal_loss

        # Update actor
        self.actor_optimizer.zero_grad()
        total_actor_loss.backward()
        nn.utils.clip_grad_norm_(
            self.actor.parameters(),
            self.config['max_grad_norm']
        )
        self.actor_optimizer.step()
        
        # Critic update
        values = self.critic.get_value(global_obs)
        critic_loss = 0.5 * ((values - returns) ** 2).mean()
        
        # Update critic
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        nn.utils.clip_grad_norm_(
            self.critic.parameters(),
            self.config['max_grad_norm']
        )
        self.critic_optimizer.step()
        
        self.update_count += 1
        
        # Return metrics
        return {
            'actor_loss': actor_loss.item(),
            'critic_loss': critic_loss.item(),
            'entropy': entropy.mean().item(),
            'ratio_mean': ratio.mean().item(),
            'ratio_std': ratio.std().item(),
            'advantage_mean': advantages.mean().item(),
            'advantage_std': advantages.std().item(),
            'value_mean': values.mean().item(),
            'proximal_loss': proximal_loss.item() if isinstance(proximal_loss, torch.Tensor) else proximal_loss,
        }
    
    def get_model_weights(self) -> dict:
        """
        Get actor network weights for FL aggregation.
        
        CRITICAL: Only actor weights are shared in federated learning.
        The critic remains local (centralized training only).
        
        Returns:
            dict: Actor network state_dict
        """
        return self.actor.state_dict()
    
    def set_model_weights(self, weights: dict):
        """
        Load aggregated actor weights from FL server.
        
        Args:
            weights: Actor network state_dict from FL aggregation
        """
        self.actor.load_state_dict(weights)
    
    def get_critic_weights(self) -> dict:
        """
        Get critic network weights (separate from FL aggregation).
        
        Returns:
            dict: Critic network state_dict
        """
        return self.critic.state_dict()
    
    def set_critic_weights(self, weights: dict):
        """
        Load critic weights (for checkpoint restoration).
        
        Args:
            weights: Critic network state_dict
        """
        self.critic.load_state_dict(weights)
    
    def save_checkpoint(self, path: str):
        """
        Save complete agent checkpoint.
        
        Args:
            path: File path to save checkpoint
        """
        os.makedirs(os.path.dirname(path), exist_ok=True)
        checkpoint = {
            'agent_id': self.agent_id,
            'config': self.config,
            'actor_state': self.actor.state_dict(),
            'critic_state': self.critic.state_dict(),
            'actor_optimizer': self.actor_optimizer.state_dict(),
            'critic_optimizer': self.critic_optimizer.state_dict(),
            'update_count': self.update_count,
        }
        torch.save(checkpoint, path)
    
    def load_checkpoint(self, path: str):
        """
        Load complete agent checkpoint.
        
        Args:
            path: File path to load checkpoint from
        """
        if not os.path.exists(path):
            raise FileNotFoundError(f"Checkpoint not found: {path}")
        
        checkpoint = torch.load(path, map_location=self.device)
        self.actor.load_state_dict(checkpoint['actor_state'])
        self.critic.load_state_dict(checkpoint['critic_state'])
        self.actor_optimizer.load_state_dict(checkpoint['actor_optimizer'])
        self.critic_optimizer.load_state_dict(checkpoint['critic_optimizer'])
        self.update_count = checkpoint['update_count']
    
    def set_train_mode(self):
        """Set networks to training mode."""
        self.training = True
        self.actor.train()
        self.critic.train()
    
    def set_eval_mode(self):
        """Set networks to evaluation mode."""
        self.training = False
        self.actor.eval()
        self.critic.eval()


if __name__ == '__main__':
    """Smoke test for MAPPO agent."""
    
    print("Testing MAPPO agent...\n")
    
    # Configuration
    agent_id = 0
    obs_dim = 40
    action_dim = 5
    num_agents = 4
    config = {
        'hidden_dim': 128,
        'lr_actor': 3e-4,
        'lr_critic': 1e-3,
        'gamma': 0.99,
        'clip_param': 0.2,
    }
    
    # Create agent
    agent = MAPPOAgent(
        agent_id=agent_id,
        obs_dim=obs_dim,
        action_dim=action_dim,
        num_agents=num_agents,
        config=config
    )
    print(f"✓ Created MAPPO agent {agent_id}")
    print(f"  Device: {agent.device}")
    print(f"  Actor params: {sum(p.numel() for p in agent.actor.parameters())}")
    print(f"  Critic params: {sum(p.numel() for p in agent.critic.parameters())}")
    
    # Test action selection
    print("\nTesting action selection...")
    obs = np.random.randn(obs_dim).astype(np.float32)
    action = agent.select_action(obs, deterministic=False)
    assert action.shape == (action_dim,), f"Expected shape ({action_dim},), got {action.shape}"
    assert (action >= -1.0).all() and (action <= 1.0).all(), "Actions not in [-1, 1]"
    print(f"✓ Stochastic action: {action[:3]} ... (shape {action.shape})")
    
    action_det = agent.select_action(obs, deterministic=True)
    print(f"✓ Deterministic action: {action_det[:3]} ... (shape {action_det.shape})")
    
    # Test action with value
    print("\nTesting action with value...")
    global_obs = np.random.randn(obs_dim * num_agents).astype(np.float32)
    action, log_prob, value = agent.select_action_with_value(obs, global_obs)
    assert action.shape == (action_dim,), f"Expected shape ({action_dim},), got {action.shape}"
    assert isinstance(log_prob, float), f"log_prob should be float, got {type(log_prob)}"
    assert isinstance(value, float), f"value should be float, got {type(value)}"
    print(f"✓ Action with value: action shape {action.shape}, log_prob={log_prob:.3f}, value={value:.3f}")
    
    # Test storing transitions
    print("\nTesting transition storage...")
    for i in range(10):
        obs = np.random.randn(obs_dim).astype(np.float32)
        action = np.random.randn(action_dim).astype(np.float32)
        reward = np.random.rand()
        next_obs = np.random.randn(obs_dim).astype(np.float32)
        done = (i == 9)
        
        agent.store_transition(
            obs, action, reward, next_obs, done,
            log_prob=np.random.randn(), value=np.random.randn()
        )
    
    assert len(agent.buffer) == 10, f"Expected buffer size 10, got {len(agent.buffer)}"
    print(f"✓ Stored 10 transitions, buffer size: {len(agent.buffer)}")
    
    # Test update
    print("\nTesting PPO update...")
    batch = {
        'obs': np.random.randn(32, obs_dim).astype(np.float32),
        'global_obs': np.random.randn(32, obs_dim * num_agents).astype(np.float32),
        'actions': np.random.randn(32, action_dim).astype(np.float32),
        'old_log_probs': np.random.randn(32).astype(np.float32),
        'advantages': np.random.randn(32).astype(np.float32),
        'returns': np.random.randn(32).astype(np.float32),
    }
    
    metrics = agent.update(batch)
    assert 'actor_loss' in metrics, "actor_loss missing from metrics"
    assert 'critic_loss' in metrics, "critic_loss missing from metrics"
    assert 'entropy' in metrics, "entropy missing from metrics"
    print("✓ PPO update successful")
    print(f"  Metrics: actor_loss={metrics['actor_loss']:.3f}, "
          f"critic_loss={metrics['critic_loss']:.3f}, entropy={metrics['entropy']:.3f}")
    
    # Test FL weight methods
    print("\nTesting FL weight methods...")
    actor_weights_before = agent.get_model_weights()
    assert isinstance(actor_weights_before, dict), "Actor weights should be dict"
    assert len(actor_weights_before) > 0, "Actor weights should not be empty"
    print(f"✓ Got actor weights: {len(actor_weights_before)} tensors")
    
    # Store original weights for comparison
    original_weights = {k: v.clone().detach().cpu() for k, v in actor_weights_before.items()}
    
    # Create new weights with modifications, ensuring proper device/dtype
    new_weights = {}
    for k, v in actor_weights_before.items():
        new_weights[k] = (v.clone() + 0.01).detach()
    
    # Set the modified weights
    agent.set_model_weights(new_weights)
    
    # Get weights after setting
    actor_weights_after = agent.get_model_weights()
    
    # Check that at least one weight changed
    weights_changed = False
    max_diff = 0.0
    for k in original_weights.keys():
        after_cpu = actor_weights_after[k].clone().detach().cpu()
        diff = (after_cpu - original_weights[k]).abs().max().item()
        max_diff = max(max_diff, diff)
        if diff > 1e-5:
            weights_changed = True
    
    assert weights_changed and max_diff >= 0.009, \
        f"Weights should have changed (max diff: {max_diff:.6f}, expected ~0.01)"
    print(f"✓ Set model weights successfully (max diff: {max_diff:.6f})")
    
    # Test critic weights
    critic_weights = agent.get_critic_weights()
    assert isinstance(critic_weights, dict), "Critic weights should be dict"
    print(f"✓ Got critic weights: {len(critic_weights)} tensors")
    
    # Test checkpoint save/load
    print("\nTesting checkpoint save/load...")
    import tempfile
    with tempfile.TemporaryDirectory() as tmpdir:
        checkpoint_path = os.path.join(tmpdir, 'agent_0.pt')
        agent.save_checkpoint(checkpoint_path)
        assert os.path.exists(checkpoint_path), "Checkpoint file not created"
        print(f"✓ Saved checkpoint to {checkpoint_path}")
        
        # Create new agent and load checkpoint
        agent2 = MAPPOAgent(agent_id, obs_dim, action_dim, num_agents, config)
        agent2.load_checkpoint(checkpoint_path)
        print("✓ Loaded checkpoint into new agent")
        
        # Verify weights match
        w1 = agent.get_model_weights()
        w2 = agent2.get_model_weights()
        for k in w1.keys():
            assert torch.allclose(w1[k], w2[k]), f"Weights mismatch for {k}"
        print("✓ Checkpoint weights match")
    
    # Test training/eval modes
    print("\nTesting training/eval modes...")
    agent.set_train_mode()
    assert agent.training is True, "Should be in training mode"
    assert agent.actor.training is True, "Actor should be in training mode"
    
    agent.set_eval_mode()
    assert agent.training is False, "Should be in eval mode"
    assert agent.actor.training is False, "Actor should be in eval mode"
    print("✓ Training/eval mode switching works")
    
    print("\n✅ All MAPPO agent tests passed!")
