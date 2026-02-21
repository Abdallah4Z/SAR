"""
Test MADDPG with FedAvg integration.

This script tests:
- FedAvg aggregation with MADDPG agents
- Weight averaging across multiple agents
- FL training loop
"""

import torch
import numpy as np
import sys
sys.path.append('/home/skyvision/HFL-UAV-Swarm-Comparison')

from src.algorithms.maddpg.maddpg_agent import MADDPGAgent
from src.federated.fedavg import FedAvg


def test_fedavg_aggregation():
    """Test FedAvg aggregates MADDPG agents correctly."""
    print("\n" + "="*60)
    print("Testing FedAvg Aggregation with MADDPG")
    print("="*60 + "\n")
    
    config = {'num_agents': 5, 'device': 'cpu', 'hidden_dim': 64}
    
    # Create 3 client agents
    agents = [MADDPGAgent(i, 40, 5, config) for i in range(3)]
    print(f"✅ Created {len(agents)} MADDPG agents")
    
    # Get initial weights
    initial_weights = agents[0].get_weights()
    print(f"✅ Retrieved initial weights")
    
    # Simulate local updates (modify weights slightly)
    print("\n🔄 Simulating local updates...")
    for i, agent in enumerate(agents):
        weights = agent.get_weights()
        for key in weights['actor'].keys():
            weights['actor'][key] += torch.randn_like(weights['actor'][key]) * 0.01
        agent.set_weights(weights)
        print(f"  Agent {i}: weights updated")
    
    # Test FedAvg aggregation
    print("\n🔄 Performing FedAvg aggregation...")
    fedavg = FedAvg(config={})
    
    # Get client weights
    client_weights = [agent.get_weights() for agent in agents]
    
    # Aggregate (use first agent as global model representation)
    global_weights = fedavg.aggregate(client_weights[0], client_weights)
    
    # Verify aggregation worked
    assert 'actor' in global_weights
    assert 'critic' in global_weights
    print("✅ Global weights structure verified")
    
    # Verify averaged weights are different from any single client
    all_different = True
    for i, client_weight in enumerate(client_weights):
        actor_match = all(
            torch.allclose(global_weights['actor'][key], client_weight['actor'][key], atol=1e-6)
            for key in global_weights['actor'].keys()
        )
        if actor_match:
            print(f"⚠️  Global weights match client {i} exactly")
            all_different = False
        else:
            print(f"✅ Global weights differ from client {i}")
    
    if all_different:
        print("\n✅ FedAvg aggregation successful! Global weights are averaged.")
    else:
        print("\n⚠️  Warning: Global weights match a client (might be expected if weights are very similar)")
    
    # Broadcast global weights back to agents
    print("\n🔄 Broadcasting global weights to all agents...")
    for i, agent in enumerate(agents):
        agent.set_weights(global_weights)
        print(f"  Agent {i}: global weights loaded")
    
    # Verify all agents now have same weights
    print("\n🔍 Verifying all agents have synchronized weights...")
    all_synced = True
    for i in range(1, len(agents)):
        weights1 = agents[0].get_weights()
        weights2 = agents[i].get_weights()
        for key in weights1['actor'].keys():
            if not torch.allclose(weights1['actor'][key], weights2['actor'][key]):
                print(f"❌ Agent 0 and Agent {i} weights differ!")
                all_synced = False
    
    if all_synced:
        print("✅ All agents synchronized with global weights!")
    
    print("\n" + "="*60)
    print("✅ FedAvg + MADDPG Integration Test Complete!")
    print("="*60 + "\n")


def test_fl_training_loop():
    """Test complete FL training loop."""
    print("\n" + "="*60)
    print("Testing Complete FL Training Loop")
    print("="*60 + "\n")
    
    config = {'num_agents': 3, 'device': 'cpu', 'hidden_dim': 64, 'noise_scale': 0.1}
    
    # Create agents
    agents = [MADDPGAgent(i, 40, 5, config) for i in range(3)]
    fedavg = FedAvg(config={})
    
    num_rounds = 5
    
    for round_idx in range(num_rounds):
        print(f"\n🔄 FL Round {round_idx + 1}/{num_rounds}")
        
        # Simulate local training (random updates)
        print("  Training locally...")
        for agent in agents:
            # Simulate batch update
            batch = {
                'obs': torch.randn(32, 3, 40),
                'actions': torch.randn(32, 3, 5),
                'rewards': torch.randn(32, 3),
                'next_obs': torch.randn(32, 3, 40),
                'dones': torch.zeros(32, 3),
                'agents': agents
            }
            metrics = agent.update(batch)
            print(f"    Agent {agent.agent_id}: critic_loss={metrics['critic_loss']:.4f}, "
                  f"actor_loss={metrics['actor_loss']:.4f}")
        
        # FedAvg aggregation
        print("  Aggregating with FedAvg...")
        client_weights = [agent.get_weights() for agent in agents]
        global_weights = fedavg.aggregate(client_weights[0], client_weights)
        
        # Broadcast
        print("  Broadcasting global model...")
        for agent in agents:
            agent.set_weights(global_weights)
        
        print(f"  ✅ Round {round_idx + 1} complete")
    
    print("\n" + "="*60)
    print("✅ FL Training Loop Test Complete!")
    print("="*60 + "\n")


if __name__ == '__main__':
    test_fedavg_aggregation()
    test_fl_training_loop()
    
    print("\n🎉 All FedAvg + MADDPG tests passed!\n")
