import numpy as np
import torch
import random

class ReplayBuffer:
    """
    Replay Buffer for storing experience tuples (Modernized, supports Global Obs).
    """
    def __init__(self, capacity, obs_dim, act_dim, num_agents, device='cuda', global_obs_dim=None):
        self.capacity = capacity
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.num_agents = num_agents
        self.device = device
        self.global_obs_dim = global_obs_dim or (obs_dim * num_agents)
        
        # Use torch.Tensor for faster handling
        self.obs = torch.zeros((capacity, num_agents, obs_dim), device=device)
        self.next_obs = torch.zeros((capacity, num_agents, obs_dim), device=device)
        self.actions = torch.zeros((capacity, num_agents, act_dim), device=device)
        self.rewards = torch.zeros((capacity, num_agents, 1), device=device)
        self.dones = torch.zeros((capacity, num_agents, 1), device=device)
        
        # Optional global observations (for QMIX/MADDPG centralized parts)
        self.global_obs = torch.zeros((capacity, self.global_obs_dim), device=device)
        self.next_global_obs = torch.zeros((capacity, self.global_obs_dim), device=device)
        
        self.idx = 0
        self.size = 0

    def push(self, obs, action, reward, next_obs, done, global_obs=None, next_global_obs=None):
        """Add transitions (Vectorized)."""
        idx = self.idx
        self.obs[idx] = torch.as_tensor(obs, device=self.device)
        self.actions[idx] = torch.as_tensor(action, device=self.device)
        
        # Handle both scalar (QMIX single agent) and vector (MADDPG all agents) rewards
        reward_tensor = torch.as_tensor(reward, device=self.device)
        if reward_tensor.dim() == 0:  # Scalar
            self.rewards[idx] = reward_tensor.view(1, 1)
        else:
            self.rewards[idx] = reward_tensor.reshape(self.num_agents, 1)
        
        self.next_obs[idx] = torch.as_tensor(next_obs, device=self.device)
        
        # Handle both scalar and vector dones
        done_tensor = torch.as_tensor(done, device=self.device)
        if done_tensor.dim() == 0:  # Scalar
            self.dones[idx] = done_tensor.view(1, 1)
        else:
            self.dones[idx] = done_tensor.reshape(self.num_agents, 1)
        
        if global_obs is not None:
            self.global_obs[idx] = torch.as_tensor(global_obs, device=self.device).reshape(-1)
        if next_global_obs is not None:
            self.next_global_obs[idx] = torch.as_tensor(next_global_obs, device=self.device).reshape(-1)
        
        self.idx = (self.idx + 1) % self.capacity
        self.size = min(self.size + 1, self.capacity)

    def sample(self, batch_size):
        """Sample batch (Zero-copy)."""
        indices = torch.randint(0, self.size, (batch_size,), device=self.device)
        
        return {
            'obs': self.obs[indices],
            'actions': self.actions[indices],
            'rewards': self.rewards[indices],
            'next_obs': self.next_obs[indices],
            'dones': self.dones[indices],
            'global_obs': self.global_obs[indices],
            'next_global_obs': self.next_global_obs[indices]
        }

    def __len__(self):
        return self.size
