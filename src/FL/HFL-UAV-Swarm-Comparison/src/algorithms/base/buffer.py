"""
Experience replay buffer for RL agents.

Supports both on-policy (PPO, MAPPO) and off-policy (DDPG, SAC) algorithms.
Stores transitions and provides batching functionality for training.
"""

import numpy as np
import torch
from typing import Dict, List, Optional, Tuple, Any
from collections import deque


class BaseBuffer:
    """
    Experience replay buffer for storing and sampling transitions (Modernized).
    
    Stored fields:
        - obs, actions, rewards, next_obs, dones, log_probs, values
    """
    
    def __init__(self, buffer_type: str = 'onpolicy', max_size: int = 10000, 
                 obs_dim: int = 44, act_dim: int = 5, device: str = 'cuda'):
        self.buffer_type = buffer_type
        self.max_size = max_size
        self.device = device
        self.ptr = 0
        self.size = 0
        
        # Pre-allocate tensors
        self.obs = torch.zeros((max_size, obs_dim), device=device)
        self.actions = torch.zeros((max_size, act_dim), device=device)
        self.rewards = torch.zeros(max_size, device=device)
        self.next_obs = torch.zeros((max_size, obs_dim), device=device)
        self.dones = torch.zeros(max_size, device=device)
        self.log_probs = torch.zeros(max_size, device=device)
        self.values = torch.zeros(max_size, device=device)
        
    def add(self, obs, action, reward, next_obs, done, log_prob=None, value=None):
        """Add a single transition (Zero-copy)."""
        idx = self.ptr
        self.obs[idx] = torch.as_tensor(obs, device=self.device)
        self.actions[idx] = torch.as_tensor(action, device=self.device)
        self.rewards[idx] = torch.as_tensor(reward, device=self.device)
        self.next_obs[idx] = torch.as_tensor(next_obs, device=self.device)
        self.dones[idx] = torch.as_tensor(done, device=self.device)
        
        if log_prob is not None:
            self.log_probs[idx] = torch.as_tensor(log_prob, device=self.device)
        if value is not None:
            self.values[idx] = torch.as_tensor(value, device=self.device)
            
        self.ptr = (self.ptr + 1) % self.max_size
        self.size = min(self.size + 1, self.max_size)
    
    def sample(self, batch_size: Optional[int] = None) -> Dict[str, torch.Tensor]:
        """Sample a batch. onpolicy returns ALL active data."""
        if self.buffer_type == 'onpolicy':
            # Return all data up to current size
            indices = torch.arange(self.size, device=self.device)
        else:
            assert batch_size is not None
            indices = torch.randint(0, self.size, (batch_size,), device=self.device)
            
        return {
            'obs': self.obs[indices],
            'actions': self.actions[indices],
            'rewards': self.rewards[indices],
            'next_obs': self.next_obs[indices],
            'dones': self.dones[indices],
            'log_probs': self.log_probs[indices],
            'values': self.values[indices],
        }
    
    def clear(self):
        """Reset buffer (pointers only)."""
        self.ptr = 0
        self.size = 0
    
    def is_ready(self, min_size: int) -> bool:
        return self.size >= min_size
    
    def __len__(self) -> int:
        return self.size
    
    def get_stats(self) -> Dict[str, Any]:
        return {
            'size': self.size,
            'capacity': self.max_size,
            'fill_ratio': self.size / self.max_size,
        }


if __name__ == '__main__':
    """Smoke test for experience buffer."""
    
    print("Testing BaseBuffer...\n")
    
    # Test on-policy buffer
    print("Testing on-policy buffer...")
    buffer_on = BaseBuffer(buffer_type='onpolicy', max_size=1000)
    
    # Add some transitions
    for i in range(10):
        obs = np.random.randn(40).astype(np.float32)
        action = np.random.randn(5).astype(np.float32)
        reward = np.random.rand()
        next_obs = np.random.randn(40).astype(np.float32)
        done = (i % 5 == 4)
        log_prob = np.random.randn()
        value = np.random.randn()
        
        buffer_on.add(obs, action, reward, next_obs, done, log_prob, value)
    
    assert len(buffer_on) == 10, f"Expected size 10, got {len(buffer_on)}"
    print(f"✓ Added 10 transitions, buffer size: {len(buffer_on)}")
    
    # Sample all data
    batch = buffer_on.sample()
    assert batch['obs'].shape == (10, 40), f"Expected shape (10, 40), got {batch['obs'].shape}"
    assert batch['actions'].shape == (10, 5), f"Expected shape (10, 5), got {batch['actions'].shape}"
    assert batch['rewards'].shape == (10,), f"Expected shape (10,), got {batch['rewards'].shape}"
    assert 'log_probs' in batch, "log_probs missing from batch"
    assert 'values' in batch, "values missing from batch"
    print(f"✓ Sampled batch: obs {batch['obs'].shape}, actions {batch['actions'].shape}")
    print(f"  Batch keys: {list(batch.keys())}")
    
    # Test clear
    buffer_on.clear()
    assert len(buffer_on) == 0, f"Expected size 0 after clear, got {len(buffer_on)}"
    print("✓ Buffer cleared successfully")
    
    # Test off-policy buffer
    print("\nTesting off-policy buffer...")
    buffer_off = BaseBuffer(buffer_type='offpolicy', max_size=100)
    
    # Fill buffer beyond capacity
    for i in range(150):
        obs = np.random.randn(40).astype(np.float32)
        action = np.random.randn(5).astype(np.float32)
        reward = np.random.rand()
        next_obs = np.random.randn(40).astype(np.float32)
        done = (i % 10 == 9)
        
        buffer_off.add(obs, action, reward, next_obs, done)
    
    assert len(buffer_off) == 100, f"Expected size 100 (max_size), got {len(buffer_off)}"
    print(f"✓ Added 150 transitions, buffer size capped at {len(buffer_off)}")
    
    # Test random sampling
    batch = buffer_off.sample(batch_size=32)
    assert batch['obs'].shape == (32, 40), f"Expected shape (32, 40), got {batch['obs'].shape}"
    assert batch['actions'].shape == (32, 5), f"Expected shape (32, 5), got {batch['actions'].shape}"
    print(f"✓ Random sampled batch: obs {batch['obs'].shape}, actions {batch['actions'].shape}")
    
    # Test is_ready
    assert buffer_off.is_ready(50) is True, "Buffer should be ready with 100 samples"
    assert buffer_off.is_ready(200) is False, "Buffer should not be ready for 200 samples"
    print("✓ is_ready() works correctly")
    
    # Test statistics
    stats = buffer_off.get_stats()
    print(f"\n✓ Buffer statistics:")
    for key, value in stats.items():
        if isinstance(value, float):
            print(f"  {key}: {value:.3f}")
        else:
            print(f"  {key}: {value}")
    
    print("\n✅ All buffer tests passed!")
