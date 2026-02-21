"""
Unit tests for MADDPG implementation.

Tests verify:
- Agent initialization with BaseAgent API
- Action selection (deterministic and stochastic)
- Network updates
- Replay buffer operations
- Federated learning weight sharing
"""

import torch
import numpy as np
import pytest
import sys
sys.path.append('/home/skyvision/HFL-UAV-Swarm-Comparison')

from src.algorithms.maddpg.maddpg_agent import MADDPGAgent
from src.algorithms.maddpg.maddpg_network import Actor, Critic
from src.algorithms.maddpg.replay_buffer import ReplayBuffer


def test_maddpg_agent_creation():
    """Test agent initialization with BaseAgent API."""
    config = {
        'num_agents': 5,
        'device': 'cpu',
        'hidden_dim': 128,
        'actor_lr': 0.001,
        'critic_lr': 0.001,
        'tau': 0.005,
        'gamma': 0.99,
        'noise_scale': 0.1
    }
    
    agent = MADDPGAgent(
        agent_id=0,
        obs_dim=40,
        action_dim=5,
        config=config
    )
    
    assert agent.agent_id == 0
    assert agent.obs_dim == 40
    assert agent.action_dim == 5
    assert agent.num_agents == 5
    print("✅ test_maddpg_agent_creation passed")


def test_select_action_deterministic():
    """Test deterministic action selection (no noise)."""
    config = {'num_agents': 5, 'device': 'cpu', 'hidden_dim': 64}
    agent = MADDPGAgent(0, 40, 5, config)
    
    obs = np.random.randn(40)
    action = agent.select_action(obs, deterministic=True)
    
    assert action.shape == (5,)
    assert np.all(action >= -1) and np.all(action <= 1)
    print("✅ test_select_action_deterministic passed")


def test_select_action_stochastic():
    """Test stochastic action selection (with noise)."""
    config = {'num_agents': 5, 'device': 'cpu', 'hidden_dim': 64, 'noise_scale': 0.1}
    agent = MADDPGAgent(0, 40, 5, config)
    
    obs = np.random.randn(40)
    action1 = agent.select_action(obs, deterministic=False)
    action2 = agent.select_action(obs, deterministic=False)
    
    # Actions should be different due to noise
    assert not np.allclose(action1, action2)
    assert np.all(action1 >= -1) and np.all(action1 <= 1)
    print("✅ test_select_action_stochastic passed")


def test_maddpg_update():
    """Test MADDPG update with batch dict."""
    config = {'num_agents': 3, 'device': 'cpu', 'hidden_dim': 64}
    agents = [MADDPGAgent(i, 40, 5, config) for i in range(3)]
    
    batch_size = 32
    batch = {
        'obs': torch.randn(batch_size, 3, 40),
        'actions': torch.randn(batch_size, 3, 5),
        'rewards': torch.randn(batch_size, 3),
        'next_obs': torch.randn(batch_size, 3, 40),
        'dones': torch.zeros(batch_size, 3),
        'agents': agents
    }
    
    metrics = agents[0].update(batch)
    
    assert 'critic_loss' in metrics
    assert 'actor_loss' in metrics
    assert 'q_value' in metrics
    assert isinstance(metrics['critic_loss'], float)
    assert isinstance(metrics['actor_loss'], float)
    print("✅ test_maddpg_update passed")


def test_replay_buffer():
    """Test replay buffer operations."""
    buffer = ReplayBuffer(
        capacity=1000,
        obs_dim=40,
        act_dim=5,
        num_agents=3,
        device='cpu'
    )
    
    # Add samples
    for _ in range(100):
        obs = np.random.randn(3, 40)
        actions = np.random.randn(3, 5)
        rewards = np.random.randn(3)
        next_obs = np.random.randn(3, 40)
        dones = np.zeros(3)
        
        buffer.push(obs, actions, rewards, next_obs, dones)
    
    assert len(buffer) == 100
    
    # Sample batch
    batch = buffer.sample(32)
    assert len(batch) == 5  # obs, actions, rewards, next_obs, dones
    assert batch[0].shape == (32, 3, 40)  # obs
    print("✅ test_replay_buffer passed")


def test_federated_weight_sharing():
    """Test get_weights and set_weights for FL."""
    config = {'num_agents': 5, 'device': 'cpu', 'hidden_dim': 64}
    agent1 = MADDPGAgent(0, 40, 5, config)
    agent2 = MADDPGAgent(1, 40, 5, config)
    
    # Get weights from agent1
    weights = agent1.get_weights()
    
    assert 'actor' in weights
    assert 'critic' in weights
    
    # Set weights to agent2
    agent2.set_weights(weights)
    
    # Verify weights match
    for key in weights['actor'].keys():
        assert torch.allclose(
            agent1.actor.state_dict()[key],
            agent2.actor.state_dict()[key]
        )
    print("✅ test_federated_weight_sharing passed")


def test_network_architectures():
    """Test Actor and Critic network architectures."""
    obs_dim = 40
    act_dim = 5
    num_agents = 3
    hidden_dim = 128
    batch_size = 16
    
    # Test Actor
    actor = Actor(obs_dim, act_dim, hidden_dim)
    obs = torch.randn(batch_size, obs_dim)
    actions = actor(obs)
    assert actions.shape == (batch_size, act_dim)
    assert torch.all(actions >= -1) and torch.all(actions <= 1)
    print("✅ Actor network test passed")
    
    # Test Critic
    critic = Critic(obs_dim, act_dim, num_agents, hidden_dim)
    obs_full = torch.randn(batch_size, num_agents * obs_dim)
    act_full = torch.randn(batch_size, num_agents * act_dim)
    q_values = critic(obs_full, act_full)
    assert q_values.shape == (batch_size, 1)
    print("✅ Critic network test passed")


if __name__ == '__main__':
    print("\n" + "="*60)
    print("Running MADDPG Unit Tests")
    print("="*60 + "\n")
    
    test_maddpg_agent_creation()
    test_select_action_deterministic()
    test_select_action_stochastic()
    test_maddpg_update()
    test_replay_buffer()
    test_federated_weight_sharing()
    test_network_architectures()
    
    print("\n" + "="*60)
    print("✅ All tests passed!")
    print("="*60 + "\n")
