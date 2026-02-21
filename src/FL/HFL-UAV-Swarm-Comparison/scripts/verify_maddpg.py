#!/usr/bin/env python3
"""
Quick verification script for Abdalla's MADDPG implementation.
Run this to verify everything is working correctly.
"""

import sys
sys.path.append('/home/skyvision/HFL-UAV-Swarm-Comparison')

import torch
import numpy as np

print("\n" + "="*80)
print("🔍 ABDALLA's MADDPG VERIFICATION SCRIPT")
print("="*80 + "\n")

# Test 1: Import check
print("Test 1: Checking imports...")
try:
    from src.algorithms.maddpg.maddpg_agent import MADDPGAgent
    from src.algorithms.maddpg.maddpg_network import Actor, Critic
    from src.algorithms.maddpg.replay_buffer import ReplayBuffer
    from src.algorithms.base.base_agent import BaseAgent
    from src.federated.fedavg import FedAvg
    print("✅ All imports successful\n")
except ImportError as e:
    print(f"❌ Import failed: {e}\n")
    sys.exit(1)

# Test 2: Agent creation with new API
print("Test 2: Creating MADDPG agent with BaseAgent API...")
try:
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
    
    print(f"✅ Agent created successfully")
    print(f"   - Agent ID: {agent.agent_id}")
    print(f"   - Obs dim: {agent.obs_dim}")
    print(f"   - Action dim: {agent.action_dim}")
    print(f"   - Num agents: {agent.num_agents}")
    print(f"   - Device: {agent.device}\n")
except Exception as e:
    print(f"❌ Agent creation failed: {e}\n")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Test 3: Action selection
print("Test 3: Testing action selection...")
try:
    obs = np.random.randn(40)
    
    # Deterministic
    action_det = agent.select_action(obs, deterministic=True)
    print(f"✅ Deterministic action: shape={action_det.shape}, range=[{action_det.min():.2f}, {action_det.max():.2f}]")
    
    # Stochastic
    action_stoch = agent.select_action(obs, deterministic=False)
    print(f"✅ Stochastic action: shape={action_stoch.shape}, range=[{action_stoch.min():.2f}, {action_stoch.max():.2f}]")
    
    # Verify they're different (due to noise)
    if np.allclose(action_det, action_stoch):
        print("⚠️  Warning: Deterministic and stochastic actions are the same (noise might be too small)")
    else:
        print("✅ Actions differ (exploration noise working)\n")
except Exception as e:
    print(f"❌ Action selection failed: {e}\n")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Test 4: Batch update
print("Test 4: Testing batch update...")
try:
    # Important: Use same num_agents in config and batch!
    config_update = {'num_agents': 3, 'device': 'cpu', 'hidden_dim': 64}
    agents = [MADDPGAgent(i, 40, 5, config_update) for i in range(3)]
    
    batch = {
        'obs': torch.randn(32, 3, 40),
        'actions': torch.randn(32, 3, 5),
        'rewards': torch.randn(32, 3),
        'next_obs': torch.randn(32, 3, 40),
        'dones': torch.zeros(32, 3),
        'agents': agents
    }
    
    metrics = agents[0].update(batch)
    
    print(f"✅ Update successful")
    print(f"   - Critic loss: {metrics['critic_loss']:.4f}")
    print(f"   - Actor loss: {metrics['actor_loss']:.4f}")
    print(f"   - Q-value: {metrics['q_value']:.4f}\n")
except Exception as e:
    print(f"❌ Update failed: {e}\n")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Test 5: Weight sharing (FL compatibility)
print("Test 5: Testing federated learning weight sharing...")
try:
    agent1 = MADDPGAgent(0, 40, 5, config)
    agent2 = MADDPGAgent(1, 40, 5, config)
    
    # Get weights
    weights = agent1.get_weights()
    print(f"✅ get_weights() successful")
    print(f"   - Keys: {list(weights.keys())}")
    
    # Set weights
    agent2.set_weights(weights)
    print(f"✅ set_weights() successful")
    
    # Verify
    weights2 = agent2.get_weights()
    match = all(
        torch.allclose(weights['actor'][key], weights2['actor'][key])
        for key in weights['actor'].keys()
    )
    
    if match:
        print(f"✅ Weights match after transfer\n")
    else:
        print(f"❌ Weights don't match after transfer\n")
except Exception as e:
    print(f"❌ Weight sharing failed: {e}\n")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Test 6: FedAvg integration
print("Test 6: Testing FedAvg integration...")
try:
    fedavg = FedAvg(config={})
    agents = [MADDPGAgent(i, 40, 5, config) for i in range(3)]
    
    # Get client weights
    client_weights = [agent.get_weights() for agent in agents]
    print(f"✅ Collected weights from {len(agents)} agents")
    
    # Aggregate (new API)
    global_weights = fedavg.aggregate(client_weights)
    print(f"✅ FedAvg aggregation successful")
    
    # Broadcast
    for agent in agents:
        agent.set_weights(global_weights)
    print(f"✅ Broadcasted global weights to all agents\n")
except Exception as e:
    print(f"❌ FedAvg integration failed: {e}\n")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Final summary
print("="*80)
print("🎉 ALL VERIFICATION TESTS PASSED!")
print("="*80)
print("\n✅ Your MADDPG implementation is ready!")
print("\nNext steps:")
print("1. Run full unit tests: python tests/test_maddpg.py")
print("2. Run FL integration tests: python tests/test_maddpg_fedavg.py")
print("3. Create experiment scripts (see docs/ABDALLA_TASKS_COMPREHENSIVE.md)")
print("4. Start running experiments\n")
print("Good luck! 🚀\n")
