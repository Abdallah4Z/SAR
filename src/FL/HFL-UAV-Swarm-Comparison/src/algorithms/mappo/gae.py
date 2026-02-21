"""
Generalized Advantage Estimation (GAE) for PPO and MAPPO.

GAE provides a way to compute advantages with reduced variance compared to
Monte Carlo returns, while maintaining lower bias than TD(0) methods.

Reference:
    High-Dimensional Continuous Control Using Generalized Advantage Estimation
    Schulman et al. 2016 (https://arxiv.org/abs/1506.02438)
"""

import torch
import numpy as np
from typing import Tuple, Union


def compute_gae(
    rewards: Union[torch.Tensor, np.ndarray],
    values: Union[torch.Tensor, np.ndarray],
    next_values: Union[torch.Tensor, np.ndarray],
    dones: Union[torch.Tensor, np.ndarray],
    gamma: float = 0.99,
    gae_lambda: float = 0.95
) -> Tuple[torch.Tensor, torch.Tensor]:
    """
    Compute Generalized Advantage Estimation (GAE) and returns.
    
    GAE formula:
        delta_t = r_t + gamma * V(s_{t+1}) * (1 - done_t) - V(s_t)
        A_t = delta_t + (gamma * lambda) * (1 - done_t) * A_{t+1}
    
    Returns formula:
        R_t = A_t + V(s_t)
    
    The advantages are normalized to have zero mean and unit variance,
    which stabilizes training.
    
    Args:
        rewards: Reward sequence, shape (T,) or (T, num_agents)
        values: Value estimates V(s_t), shape (T,) or (T, num_agents)
        next_values: Next value estimates V(s_{t+1}), shape (T,) or (T, num_agents)
        dones: Episode termination flags, shape (T,) or (T, num_agents)
        gamma: Discount factor (default: 0.99)
        gae_lambda: GAE lambda parameter (default: 0.95)
            - lambda=0: TD(0) (low variance, high bias)
            - lambda=1: Monte Carlo (high variance, low bias)
            - lambda∈(0,1): Trade-off between bias and variance
    
    Returns:
        advantages: Advantage estimates A_t, shape same as input
        returns: Target returns R_t = A_t + V(s_t), shape same as input
    
    Example:
        >>> rewards = torch.tensor([1.0, 2.0, 3.0, 4.0])
        >>> values = torch.tensor([0.5, 1.0, 1.5, 2.0])
        >>> next_values = torch.tensor([1.0, 1.5, 2.0, 0.0])
        >>> dones = torch.tensor([0.0, 0.0, 0.0, 1.0])
        >>> advantages, returns = compute_gae(rewards, values, next_values, dones)
    """
    # Convert to torch tensors if numpy
    if isinstance(rewards, np.ndarray):
        rewards = torch.from_numpy(rewards).float()
    if isinstance(values, np.ndarray):
        values = torch.from_numpy(values).float()
    if isinstance(next_values, np.ndarray):
        next_values = torch.from_numpy(next_values).float()
    if isinstance(dones, np.ndarray):
        dones = torch.from_numpy(dones).float()
    
    # Ensure same device
    device = rewards.device
    values = values.to(device)
    next_values = next_values.to(device)
    dones = dones.to(device)
    
    # Get sequence length
    T = rewards.shape[0]
    
    # Initialize advantages
    advantages = torch.zeros_like(rewards)
    
    # Compute TD errors: delta_t = r_t + gamma * V(s_{t+1}) * (1 - done) - V(s_t)
    deltas = rewards + gamma * next_values * (1.0 - dones) - values
    
    # Compute GAE backwards from T-1 to 0
    gae = 0.0
    for t in reversed(range(T)):
        gae = deltas[t] + gamma * gae_lambda * (1.0 - dones[t]) * gae
        advantages[t] = gae
    
    # Compute returns: R_t = A_t + V(s_t)
    returns = advantages + values
    
    # Normalize advantages: (A - mean(A)) / (std(A) + eps)
    # This stabilizes training by keeping gradient magnitudes consistent
    advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
    
    return advantages, returns


def compute_gae_trajectory(
    trajectory: dict,
    gamma: float = 0.99,
    gae_lambda: float = 0.95
) -> Tuple[torch.Tensor, torch.Tensor]:
    """
    Compute GAE for a trajectory dictionary.
    
    This is a convenience wrapper around compute_gae that extracts
    the necessary fields from a trajectory dictionary.
    
    Args:
        trajectory: Dictionary containing:
            - 'rewards': Reward sequence (T,)
            - 'values': Value estimates (T,)
            - 'dones': Done flags (T,)
            - 'next_obs': Next observations (T, obs_dim)
        gamma: Discount factor
        gae_lambda: GAE lambda parameter
    
    Returns:
        advantages: Advantage estimates (T,)
        returns: Target returns (T,)
    """
    rewards = trajectory['rewards']
    values = trajectory['values']
    dones = trajectory['dones']
    
    # Compute next values from trajectory
    # For the last timestep, next_value = 0 if done, else V(next_obs)
    next_values = torch.cat([
        values[1:],
        torch.zeros_like(values[:1])
    ])
    
    return compute_gae(rewards, values, next_values, dones, gamma, gae_lambda)


def compute_returns_monte_carlo(
    rewards: Union[torch.Tensor, np.ndarray],
    dones: Union[torch.Tensor, np.ndarray],
    gamma: float = 0.99
) -> torch.Tensor:
    """
    Compute Monte Carlo returns (no value function bootstrap).
    
    This is included for comparison with GAE. MC returns have high variance
    but zero bias, while GAE trades off some bias for lower variance.
    
    Args:
        rewards: Reward sequence (T,)
        dones: Done flags (T,)
        gamma: Discount factor
    
    Returns:
        returns: Discounted returns (T,)
    """
    if isinstance(rewards, np.ndarray):
        rewards = torch.from_numpy(rewards).float()
    if isinstance(dones, np.ndarray):
        dones = torch.from_numpy(dones).float()
    
    T = rewards.shape[0]
    returns = torch.zeros_like(rewards)
    
    G = 0.0
    for t in reversed(range(T)):
        G = rewards[t] + gamma * G * (1.0 - dones[t])
        returns[t] = G
    
    return returns


if __name__ == '__main__':
    """Smoke test for GAE computation."""
    
    print("Testing GAE computation...\n")
    
    # Test 1: Simple trajectory
    print("Test 1: Simple trajectory")
    T = 10
    rewards = torch.tensor([1.0] * T)
    values = torch.tensor([5.0] * T)
    next_values = torch.tensor([5.0] * T)
    dones = torch.zeros(T)
    dones[-1] = 1.0  # Last step is terminal
    
    advantages, returns = compute_gae(
        rewards, values, next_values, dones, gamma=0.99, gae_lambda=0.95
    )
    
    assert advantages.shape == (T,), f"Expected shape ({T},), got {advantages.shape}"
    assert returns.shape == (T,), f"Expected shape ({T},), got {returns.shape}"
    assert torch.abs(advantages.mean()) < 1e-6, \
        f"Advantages should be normalized (mean≈0), got {advantages.mean():.6f}"
    assert torch.abs(advantages.std() - 1.0) < 1e-6, \
        f"Advantages should be normalized (std≈1), got {advantages.std():.6f}"
    print(f"✓ Shape: advantages {advantages.shape}, returns {returns.shape}")
    print(f"✓ Normalized: mean={advantages.mean():.6f}, std={advantages.std():.6f}")
    print(f"  Advantage range: [{advantages.min():.3f}, {advantages.max():.3f}]")
    print(f"  Return range: [{returns.min():.3f}, {returns.max():.3f}]")
    
    # Test 2: Multi-agent trajectory
    print("\nTest 2: Multi-agent trajectory")
    num_agents = 4
    rewards_ma = torch.randn(T, num_agents)
    values_ma = torch.randn(T, num_agents)
    next_values_ma = torch.randn(T, num_agents)
    dones_ma = torch.zeros(T, num_agents)
    dones_ma[-1, :] = 1.0
    
    advantages_ma, returns_ma = compute_gae(
        rewards_ma, values_ma, next_values_ma, dones_ma
    )
    
    assert advantages_ma.shape == (T, num_agents), \
        f"Expected shape ({T}, {num_agents}), got {advantages_ma.shape}"
    print(f"✓ Multi-agent: {advantages_ma.shape}")
    print(f"  Mean per agent: {advantages_ma.mean(dim=0)}")
    
    # Test 3: Numpy input
    print("\nTest 3: Numpy input compatibility")
    rewards_np = np.random.randn(T).astype(np.float32)
    values_np = np.random.randn(T).astype(np.float32)
    next_values_np = np.random.randn(T).astype(np.float32)
    dones_np = np.zeros(T, dtype=np.float32)
    
    advantages_np, returns_np = compute_gae(
        rewards_np, values_np, next_values_np, dones_np
    )
    
    assert isinstance(advantages_np, torch.Tensor), "Should return torch tensor"
    assert isinstance(returns_np, torch.Tensor), "Should return torch tensor"
    print("✓ Numpy input converted to torch correctly")
    
    # Test 4: Monte Carlo returns
    print("\nTest 4: Monte Carlo returns")
    mc_returns = compute_returns_monte_carlo(rewards, dones, gamma=0.99)
    assert mc_returns.shape == (T,), f"Expected shape ({T},), got {mc_returns.shape}"
    print(f"✓ MC returns: {mc_returns.shape}")
    print(f"  MC return range: [{mc_returns.min():.3f}, {mc_returns.max():.3f}]")
    
    # Test 5: Different lambda values
    print("\nTest 5: Effect of GAE lambda")
    for lam in [0.0, 0.5, 0.95, 1.0]:
        adv, ret = compute_gae(rewards, values, next_values, dones, gae_lambda=lam)
        print(f"  λ={lam:.2f}: advantage std={adv.std():.3f}, return mean={ret.mean():.3f}")
    print("✓ Lambda parameter works correctly")
    
    # Test 6: Terminal state handling
    print("\nTest 6: Terminal state handling")
    rewards_term = torch.tensor([1.0, 2.0, 3.0])
    values_term = torch.tensor([1.0, 2.0, 3.0])
    next_values_term = torch.tensor([2.0, 3.0, 0.0])  # Terminal next_value = 0
    dones_term = torch.tensor([0.0, 0.0, 1.0])
    
    adv_term, ret_term = compute_gae(
        rewards_term, values_term, next_values_term, dones_term
    )
    
    # At terminal state, next_value should not contribute
    print(f"✓ Terminal state: last advantage={adv_term[-1]:.3f}, last return={ret_term[-1]:.3f}")
    
    print("\n✅ All GAE computation tests passed!")
