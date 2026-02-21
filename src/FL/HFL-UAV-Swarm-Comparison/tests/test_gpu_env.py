"""
Tests for the GPU-tensorized UAV swarm environment.

Verifies:
1. Environment can be created and reset on GPU
2. Environment step produces correct shapes and types
3. Observations, rewards, and metrics are valid
4. Multi-step episode runs without errors
5. Full-episode rollout completes properly
"""

import sys
import os
import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.environments.uav_swarm_env_gpu import UAVSwarmEnvGPU
from configs.env_configs import get_config_for_swarm_size


def test_gpu_env_creation():
    """Test that GPU env can be created."""
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    config = get_config_for_swarm_size(10, seed=42)
    env = UAVSwarmEnvGPU(config, device=device)
    
    assert env.num_uavs == 10
    assert env.num_gd == 50  # 5 per UAV
    assert env.episode_length == config['episode_length']
    print(f"✓ GPU env created on {device}: {env.num_uavs} UAVs, {env.num_gd} GDs")


def test_gpu_env_reset():
    """Test reset returns correct observation format."""
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    config = get_config_for_swarm_size(10, seed=42)
    env = UAVSwarmEnvGPU(config, device=device)
    
    obs, info = env.reset(seed=42)
    
    assert isinstance(obs, dict), f"obs should be dict, got {type(obs)}"
    assert len(obs) == 10, f"Should have 10 agents, got {len(obs)}"
    
    for i in range(10):
        assert i in obs, f"Agent {i} missing from obs"
        assert isinstance(obs[i], torch.Tensor), f"obs[{i}] should be Tensor"
        expected_dim = 8 + 20 + 4 * 9  # self(8) + tasks(20) + neighbors(4*9)
        assert obs[i].shape == (expected_dim,), f"obs[{i}] shape {obs[i].shape} != ({expected_dim},)"
        assert not obs[i].isnan().any(), f"obs[{i}] contains NaN"
    
    print(f"✓ Reset: obs_dim={obs[0].shape[0]}, device={obs[0].device}")


def test_gpu_env_step():
    """Test step with random actions."""
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    config = get_config_for_swarm_size(10, seed=42)
    env = UAVSwarmEnvGPU(config, device=device)
    env.reset(seed=42)
    
    # Random actions
    actions = {i: torch.randn(5).clamp(-1, 1) for i in range(10)}
    
    obs, rewards, done, truncated, info = env.step(actions)
    
    # Check obs
    assert isinstance(obs, dict) and len(obs) == 10
    for i in range(10):
        assert not obs[i].isnan().any(), f"obs[{i}] contains NaN after step"
    
    # Check rewards
    assert isinstance(rewards, dict) and len(rewards) == 10
    for i in range(10):
        assert isinstance(rewards[i], float), f"reward[{i}] should be float"
        assert not np.isnan(rewards[i]), f"reward[{i}] is NaN"
    
    # Check done/truncated
    assert isinstance(done, bool)
    assert isinstance(truncated, bool)
    
    # Check info
    assert 'metrics' in info
    assert 'alive_uavs' in info
    
    print(f"✓ Step: rewards={[rewards[i] for i in range(3)]}, alive={info['alive_uavs']}")


def test_gpu_env_multi_step():
    """Test running multiple steps."""
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    config = get_config_for_swarm_size(10, seed=42)
    env = UAVSwarmEnvGPU(config, device=device)
    env.reset(seed=42)
    
    total_reward = 0.0
    steps = 0
    
    for step in range(500):
        actions = {i: torch.randn(5).clamp(-1, 1) for i in range(10)}
        obs, rewards, done, truncated, info = env.step(actions)
        total_reward += sum(rewards.values())
        steps += 1
        
        if done or truncated:
            break
    
    metrics = info['metrics']
    print(f"✓ Multi-step ({steps} steps):")
    print(f"  Total reward: {total_reward:.2f}")
    print(f"  Alive UAVs: {info['alive_uavs']}")
    print(f"  Tasks completed: {metrics['tasks_completed']}")
    print(f"  Task success rate: {metrics['task_success_rate']:.1f}%")
    print(f"  Total energy: {metrics['total_energy_j']:.2f} J")


def test_gpu_env_numpy_actions():
    """Test that the env accepts numpy arrays as actions."""
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    config = get_config_for_swarm_size(10, seed=42)
    env = UAVSwarmEnvGPU(config, device=device)
    env.reset(seed=42)
    
    # Numpy actions (this is what trainers currently provide)
    actions = {i: np.random.randn(5).clip(-1, 1).astype(np.float32) for i in range(10)}
    
    obs, rewards, done, truncated, info = env.step(actions)
    assert isinstance(obs, dict) and len(obs) == 10
    print(f"✓ Numpy actions accepted successfully")


def test_gpu_env_performance():
    """Benchmark GPU env performance."""
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    
    for num_drones in [10, 20, 50]:
        config = get_config_for_swarm_size(num_drones, seed=42)
        env = UAVSwarmEnvGPU(config, device=device)
        env.reset(seed=42)
        
        # Warmup
        for _ in range(10):
            actions = {i: torch.randn(5).clamp(-1, 1) for i in range(num_drones)}
            env.step(actions)
        
        if device == 'cuda':
            torch.cuda.synchronize()
        
        import time
        start = time.time()
        num_steps = 200
        for _ in range(num_steps):
            actions = {i: torch.randn(5).clamp(-1, 1) for i in range(num_drones)}
            env.step(actions)
        
        if device == 'cuda':
            torch.cuda.synchronize()
        
        elapsed = time.time() - start
        fps = num_steps / elapsed
        
        print(f"  {num_drones:3d} drones: {fps:.0f} steps/sec ({elapsed*1000/num_steps:.1f} ms/step)")
    
    print(f"✓ Performance benchmark complete")


def test_observation_dim_consistency():
    """Test obs dim matches calculate_obs_dim formula."""
    from configs.marl_configs import calculate_obs_dim
    
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    
    for num_drones in [4, 10, 20]:
        expected_dim = calculate_obs_dim(num_drones)
        config = get_config_for_swarm_size(num_drones, seed=42)
        env = UAVSwarmEnvGPU(config, device=device)
        obs, _ = env.reset(seed=42)
        actual_dim = obs[0].shape[0]
        assert actual_dim == expected_dim, \
            f"{num_drones} drones: obs_dim {actual_dim} != expected {expected_dim}"
    
    print(f"✓ Obs dim matches calculate_obs_dim for all swarm sizes")


if __name__ == '__main__':
    print("=" * 60)
    print("GPU Environment Tests")
    print("=" * 60)
    
    tests = [
        test_gpu_env_creation,
        test_gpu_env_reset,
        test_gpu_env_step,
        test_gpu_env_multi_step,
        test_gpu_env_numpy_actions,
        test_observation_dim_consistency,
        test_gpu_env_performance,
    ]
    
    passed = 0
    failed = 0
    for test_fn in tests:
        print(f"\n--- {test_fn.__name__} ---")
        try:
            test_fn()
            passed += 1
        except Exception as e:
            print(f"✗ FAILED: {e}")
            import traceback
            traceback.print_exc()
            failed += 1
    
    print(f"\n{'=' * 60}")
    print(f"Results: {passed} passed, {failed} failed")
    print(f"{'=' * 60}")
