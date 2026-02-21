#!/usr/bin/env python3
"""
QMIX Training Script with Federated Learning

Train QMIX agents with FedProx aggregation.

Usage:
    python scripts/train/train_qmix.py --num_drones 4 --fl fedprox --num_rounds 10
    python scripts/train/train_qmix.py --num_drones 10 --fl fedprox --num_rounds 50 --seed 42
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import argparse
import json
import numpy as np
import torch
import random
import os
from datetime import datetime

# Silence warnings
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import warnings
warnings.filterwarnings('ignore')

from src.environments.uav_swarm_env import UAVSwarmEnv
from src.algorithms.qmix.qmix_agent import QMIXAgent
from src.algorithms.qmix.qmix_trainer import QMIXTrainer
from src.federated.fedprox import FedProxAggregator, AdaptiveFedProxAggregator
from src.federated.hierarchical_aggregator import HierarchicalAggregator
from src.federated.fedavg import FedAvg
from src.federated.fedadam import FedAdam
from configs.env_configs import get_config_for_swarm_size
from configs.marl_configs import get_marl_config_for_swarm_size, calculate_obs_dim
from configs.fl_configs import get_fl_config


class NumpyEncoder(json.JSONEncoder):
    """JSON encoder that handles numpy types."""
    def default(self, obj):
        if isinstance(obj, np.floating):
            return float(obj)
        if isinstance(obj, np.integer):
            return int(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return super().default(obj)


def main():
    parser = argparse.ArgumentParser(description='QMIX Training with Federated Learning')
    parser.add_argument('--num_drones', type=int, default=4, help='Number of UAVs')
    parser.add_argument('--num_rounds', type=int, default=20, help='Number of FL rounds')
    parser.add_argument('--fl', type=str, default='fedprox',
                       choices=['fedavg', 'fedprox', 'adaptive_fedprox', 'fedadam', 'hierarchical'],
                       help='FL algorithm')
    parser.add_argument('--num_ground_devices', type=int, default=None, help='Override ground devices')
    parser.add_argument('--seed', type=int, default=42, help='Random seed')
    parser.add_argument('--device', type=str, default='cuda', help='Device (cuda or cpu)')
    parser.add_argument('--cpu_env', action='store_true', help='Use CPU environment instead of GPU (for debugging)')
    parser.add_argument('--debug', action='store_true', help='Debug mode')
    parser.add_argument('--resume', action='store_true', help='Resume training from checkpoint')
    parser.add_argument('--checkpoint_name', type=str, default=None, help='Checkpoint name to resume from')
    parser.add_argument('--save_interval', type=int, default=50, help='Save checkpoint every N rounds')
    
    args = parser.parse_args()
    
    # ══════════════════════════════════════════════════════════════
    # Setup
    # ══════════════════════════════════════════════════════════════
    print("=" * 70)
    print("QMIX Training with Federated Learning")
    print("=" * 70)
    print(f"UAV Swarm Size: {args.num_drones}")
    print(f"FL Algorithm: {args.fl}")
    print(f"Training Rounds: {args.num_rounds}")
    print(f"Random Seed: {args.seed}")
    print("=" * 70)
    
    # Set random seeds
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    random.seed(args.seed)
    print(f"✓ Random seeds set to {args.seed}")
    
    # Device setup
    device = args.device if torch.cuda.is_available() else 'cpu'
    print(f"✓ Using device: {device}")
    if device == 'cuda':
        gpu = torch.cuda.get_device_name(0)
        print(f"  GPU: {gpu}")
    
    # ══════════════════════════════════════════════════════════════
    # Environment
    # ══════════════════════════════════════════════════════════════
    print("\n[1/5] Creating environment...")
    env_config = get_config_for_swarm_size(args.num_drones, seed=args.seed)
    if args.num_ground_devices:
        env_config['num_ground_devices'] = args.num_ground_devices
    
    if args.cpu_env:
        env = UAVSwarmEnv(env_config)
        print("✓ CPU environment created (debugging mode)")
    else:
        from src.environments.uav_swarm_env_gpu import UAVSwarmEnvGPU
        env = UAVSwarmEnvGPU(env_config, device=device)
        print("✓ GPU-tensorized environment created")
    obs_dict, _ = env.reset()
    actual_obs_dim = obs_dict[0].shape[0]
    
    print(f"✓ Environment created:")
    print(f"  UAVs: {env_config['num_uavs']}")
    print(f"  Ground devices: {env_config['num_ground_devices']}")
    print(f"  Area size: {env_config['area_size']}")
    print(f"  Episode length: {env_config['episode_length']} steps")
    print(f"  Actual obs_dim from environment: {actual_obs_dim}")
    
    # ══════════════════════════════════════════════════════════════
    # Configurations
    # ══════════════════════════════════════════════════════════════
    print("\n[2/5] Loading configurations...")
    
    # FL Config - Member 2 manages these configurations
    fl_config = get_fl_config(args.fl)
    fl_config['num_rounds'] = args.num_rounds  # Override with CLI arg
    
    if args.fl == 'fedprox':
        fl_aggregator = FedProxAggregator(
            num_agents=args.num_drones,
            num_rounds=args.num_rounds,
            local_epochs=fl_config.get('local_epochs', 5),
            mu=fl_config.get('mu', 0.01),
            adaptive_mu=False
        )
    elif args.fl == 'adaptive_fedprox':
        fl_aggregator = AdaptiveFedProxAggregator(
            num_agents=args.num_drones,
            num_rounds=args.num_rounds,
            local_epochs=fl_config.get('local_epochs', 5),
            initial_mu=fl_config.get('mu', 0.01),
            learning_rate=fl_config.get('mu_lr', 0.01)
        )
    elif args.fl == 'hierarchical':
        hfl_config = {
            'cluster_update_interval': fl_config.get('cluster_update_interval', 10),
            'min_cluster_size': 2,
            'communication_range': 50.0,
            'reclustering_threshold': 50.0,
            'clustering_strategy': 'kmeans',
        }
        fl_aggregator = HierarchicalAggregator(
            num_agents=args.num_drones,
            num_clusters=fl_config.get('num_clusters', 'auto'),
            intra_cluster_rounds=fl_config.get('intra_cluster_rounds', 3),
            config=hfl_config
        )
    elif args.fl == 'fedavg':
        fl_aggregator = FedAvg({'num_agents': args.num_drones})
    elif args.fl == 'fedadam':
        fl_aggregator = FedAdam(config=fl_config)
    else:
        raise ValueError(f"Unknown FL algorithm: {args.fl}")
    
    print(f"✓ FL Config: {args.fl}")
    if hasattr(fl_aggregator, 'get_aggregation_info'):
        fl_info = fl_aggregator.get_aggregation_info()
        for key, value in fl_info.items():
            if key != 'algorithm':
                print(f"    {key}: {value}")
    else:
        print(f"    algorithm: {args.fl}")
        print(f"    num_agents: {args.num_drones}")
    
    # MARL Config
    marl_config = get_marl_config_for_swarm_size('qmix', args.num_drones)
    
    # Verify obs_dim matches actual environment
    config_obs_dim = marl_config['obs_dim']
    if config_obs_dim != actual_obs_dim:
        print(f"  ⚠ Config obs_dim={config_obs_dim} != actual={actual_obs_dim}, updating...")
        marl_config['obs_dim'] = actual_obs_dim
    
    print(f"✓ MARL Config: QMIX")
    print(f"    obs_dim: {marl_config['obs_dim']}")
    print(f"    action_dim: {marl_config['action_dim']}")
    print(f"    hidden_dim: {marl_config['hidden_dim']}")
    print(f"    num_actions: {marl_config.get('num_actions', 81)}")
    
    # ══════════════════════════════════════════════════════════════
    # Create agents
    # ══════════════════════════════════════════════════════════════
    print("\n[3/5] Creating QMIX agents...")
    
    agents = {}
    for agent_id in range(args.num_drones):
        agent = QMIXAgent(
            agent_id=agent_id,
            obs_dim=marl_config['obs_dim'],
            action_dim=5,  # Continuous action space (will be discretized)
            num_agents=args.num_drones,
            num_actions=marl_config.get('num_actions', 81),
            hidden_dim=marl_config.get('hidden_dim', 256),
            lr=marl_config.get('lr', 5e-4),
            gamma=marl_config.get('gamma', 0.99),
            tau=marl_config.get('tau', 0.005),
            buffer_size=marl_config.get('buffer_size', 100000),
            batch_size=marl_config.get('batch_size', 32),
            device=device
        )
        agents[agent_id] = agent
    
    # Count parameters
    num_params = sum(p.numel() for agent in agents.values()
                    for p in agent.network.parameters())
    
    print(f"✓ Created {len(agents)} QMIX agents")
    print(f"  Obs dim: {marl_config['obs_dim']}")
    print(f"  Discrete actions: {marl_config.get('num_actions', 81)}")
    print(f"  Hidden dim: {marl_config.get('hidden_dim', 256)}")
    print(f"  Total parameters: {num_params:,}")
    
    # ══════════════════════════════════════════════════════════════
    # Create trainer
    # ══════════════════════════════════════════════════════════════
    print("\n[4/5] Creating trainer...")
    
    trainer = QMIXTrainer(
        agents=agents,
        env=env,
        fl_aggregator=fl_aggregator,
        rollout_length='episode',
        update_interval=1,
        save_interval=args.save_interval
    )
    
    print(f"✓ Trainer created")
    print(f"  Rollout length: full episode")
    print(f"  Q-learning updates per FL round: 5")
    
    # ══════════════════════════════════════════════════════════════
    # Training
    # ══════════════════════════════════════════════════════════════
    print("\n[5/5] Starting training...")
    
    # Load checkpoint if resuming
    resume = False
    if args.resume:
        if args.checkpoint_name:
            checkpoint_name = args.checkpoint_name
        else:
            # Auto-detect latest checkpoint
            checkpoint_name = f"{args.num_drones}drones_{args.fl}"
        
        print(f"\nAttempting to resume from checkpoint: {checkpoint_name}")
        if trainer.load_checkpoint(checkpoint_name):
            resume = True
        else:
            print(f"  Starting from scratch (checkpoint not found)\n")
    
    print()
    print("=" * 60)
    print("QMIX Training with FedProx Aggregation")
    print("=" * 60)
    
    try:
        history = trainer.train(num_fl_rounds=args.num_rounds, resume=resume)
    except KeyboardInterrupt:
        print("\n\nTraining interrupted by user!")
        # Save checkpoint on interrupt
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        interrupt_checkpoint = f"{args.num_drones}drones_{args.fl}_interrupted_{timestamp}"
        print(f"Saving checkpoint: {interrupt_checkpoint}")
        trainer.save_checkpoint(interrupt_checkpoint)
    
    # ══════════════════════════════════════════════════════════════
    # Evaluation
    # ══════════════════════════════════════════════════════════════
    print("\n" + "=" * 60)
    print("Evaluation")
    print("=" * 60)
    
    eval_results = trainer.evaluate(num_episodes=5)
    print(f"Avg Reward: {eval_results['avg_reward']:,.0f} ± {eval_results['std_reward']:,.0f}")
    
    # ══════════════════════════════════════════════════════════════
    # Save results
    # ══════════════════════════════════════════════════════════════
    print("\n" + "=" * 70)
    print("Saving results...")
    print("=" * 70)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    results_dir = f"results/qmix/{args.num_drones}drones_{args.fl}_seed{args.seed}_{timestamp}"
    Path(results_dir).mkdir(parents=True, exist_ok=True)
    
    # Save training history (list-of-dicts format)
    with open(f"{results_dir}/training_history.json", 'w') as f:
        json.dump(history, f, indent=2, cls=NumpyEncoder)

    # Save experiment config
    experiment_config = {
        'num_drones': args.num_drones,
        'num_rounds': args.num_rounds,
        'fl_algorithm': args.fl,
        'marl_algorithm': 'qmix',
        'seed': args.seed,
        'timestamp': timestamp,
        'obs_dim': marl_config['obs_dim'],
        'num_actions': marl_config.get('num_actions', 81),
        'total_parameters': num_params,
        'eval_avg_reward': float(eval_results['avg_reward']),
        'eval_std_reward': float(eval_results['std_reward']),
    }

    with open(f"{results_dir}/experiment_config.json", 'w') as f:
        json.dump(experiment_config, f, indent=2)

    # Save final agents
    for agent_id, agent in agents.items():
        agent.save_checkpoint(f"{results_dir}/final_agent_{agent_id}.pt")

    print(f"✓ Results saved to: {results_dir}")
    print(f"  - training_history.json")
    print(f"  - experiment_config.json")
    print(f"  - final_agent_*.pt")

    # Summary
    print("\n" + "=" * 70)
    print("Training Summary")
    print("=" * 70)
    if history and history['fl_round']:
        print(f"Final Loss: {history['avg_loss'][-1]:.4f}")
        print(f"Final Epsilon: {history['avg_epsilon'][-1]:.4f}")
    print(f"Evaluation Reward: {eval_results['avg_reward']:,.0f}")
    print()


if __name__ == '__main__':
    main()
