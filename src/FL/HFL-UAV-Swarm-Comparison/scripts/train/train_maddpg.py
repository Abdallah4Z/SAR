"""
Train MADDPG with any FL algorithm.

Usage:
    python scripts/train/train_maddpg.py --num_drones 10 --fl fedavg --seed 42
"""

import argparse
import sys
import os
import json
import numpy as np
import torch
import random
from datetime import datetime
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

# Import environment
from src.environments.uav_swarm_env import UAVSwarmEnv
from configs.env_configs import get_config_for_swarm_size

# Import MADDPG components
from src.algorithms.maddpg.maddpg_agent import MADDPGAgent
from src.algorithms.maddpg.maddpg_trainer import MADDPGTrainer

# Import FL aggregators
from src.federated.fedavg import FedAvg
from src.federated.fedprox import FedProxAggregator
from src.federated.fedadam import FedAdam
from src.federated.hierarchical_aggregator import HierarchicalAggregator

# Import configurations
from configs.fl_configs import get_fl_config
from configs.marl_configs import get_marl_config_for_swarm_size

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

def parse_args():
    parser = argparse.ArgumentParser(description='Train MADDPG with Federated Learning')
    parser.add_argument('--num_drones', type=int, default=10, help='Number of UAVs')
    parser.add_argument('--fl_algorithm', '--fl', type=str, default='fedavg',
                       choices=['fedavg', 'fedprox', 'fedadam', 'hierarchical'],
                       help='FL algorithm')
    parser.add_argument('--num_rounds', type=int, default=100, help='Number of FL rounds')
    parser.add_argument('--episodes_per_round', type=int, default=5, help='Episodes per round')
    parser.add_argument('--seed', type=int, default=42, help='Random seed')
    parser.add_argument('--device', type=str, default='cuda', help='Device (cuda or cpu)')
    parser.add_argument('--save_dir', type=str, default='results/maddpg', help='Save directory')
    parser.add_argument('--cpu_env', action='store_true', help='Use CPU environment instead of GPU (for debugging)')
    parser.add_argument('--resume', action='store_true', help='Resume training from checkpoint')
    parser.add_argument('--checkpoint_name', type=str, default=None, help='Checkpoint name to resume from')
    parser.add_argument('--save_interval', type=int, default=25, help='Save checkpoint every N rounds')
    return parser.parse_args()

def main():
    args = parse_args()
    
    print("=" * 70)
    print("MADDPG Training with Federated Learning")
    print("=" * 70)
    
    # Set seeds
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    random.seed(args.seed)
    
    device = args.device if torch.cuda.is_available() else 'cpu'
    
    # 1. Environment
    print("\n[1/5] Creating environment...")
    env_config = get_config_for_swarm_size(args.num_drones, seed=args.seed)
    
    if args.cpu_env:
        env = UAVSwarmEnv(env_config)
        print("✓ CPU environment created (debugging mode)")
    else:
        from src.environments.uav_swarm_env_gpu import UAVSwarmEnvGPU
        env = UAVSwarmEnvGPU(env_config, device=device)
        print("✓ GPU-tensorized environment created")
    
    res = env.reset()
    obs_dict = res[0]
    actual_obs_dim = obs_dict[0].shape[0]
    
    # 2. FL Aggregator
    print("\n[2/5] Creating FL aggregator...")
    fl_config = get_fl_config(args.fl_algorithm)
    
    if args.fl_algorithm == 'fedavg':
        aggregator = FedAvg({'num_agents': args.num_drones})
    elif args.fl_algorithm == 'fedprox':
        aggregator = FedProxAggregator(num_agents=args.num_drones, num_rounds=args.num_rounds)
    elif args.fl_algorithm == 'fedadam':
        aggregator = FedAdam(config=fl_config)
    elif args.fl_algorithm == 'hierarchical':
        aggregator = HierarchicalAggregator(num_agents=args.num_drones)
    
    # 3. MARL Config & Agents
    print("\n[3/5] Creating MADDPG agents...")
    marl_config = get_marl_config_for_swarm_size('maddpg', args.num_drones)
    marl_config['obs_dim'] = actual_obs_dim
    marl_config['device'] = device
    
    agent_config = {
        'num_agents': args.num_drones,
        'device': device,
        'obs_dim': actual_obs_dim,
        'action_dim': 5,
        'hidden_dim': marl_config.get('hidden_dim', 256),
        'actor_lr': marl_config.get('actor_lr', 1e-3),
        'critic_lr': marl_config.get('critic_lr', 1e-3),
        'gamma': marl_config.get('gamma', 0.95),
        'tau': marl_config.get('tau', 0.005),
    }
    
    agents = [MADDPGAgent(i, actual_obs_dim, 5, agent_config) for i in range(args.num_drones)]
    
    # 4. Trainer
    print("\n[4/5] Creating trainer...")
    trainer = MADDPGTrainer(env, agents, aggregator, marl_config)
    trainer.save_interval = args.save_interval  # Set checkpoint save interval
    
    # 5. Training
    print("\n[5/5] Starting training...")
    
    # Load checkpoint if resuming
    resume = False
    if args.resume:
        if args.checkpoint_name:
            checkpoint_name = args.checkpoint_name
        else:
            # Auto-detect latest checkpoint
            checkpoint_name = f"{args.num_drones}drones_{args.fl_algorithm}"
        
        print(f"\nAttempting to resume from checkpoint: {checkpoint_name}")
        if trainer.load_checkpoint(checkpoint_name):
            resume = True
        else:
            print(f"  Starting from scratch (checkpoint not found)\n")
    
    try:
        history = trainer.train(
            num_fl_rounds=args.num_rounds,
            episodes_per_round=args.episodes_per_round,
            resume=resume
        )
    except KeyboardInterrupt:
        print("\n\nTraining interrupted by user!")
        # Save checkpoint on interrupt
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        interrupt_checkpoint = f"{args.num_drones}drones_{args.fl_algorithm}_interrupted_{timestamp}"
        print(f"Saving checkpoint: {interrupt_checkpoint}")
        trainer.save_checkpoint(interrupt_checkpoint)
    
    # Save results
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.join(args.save_dir, f"{args.num_drones}drones_{args.fl_algorithm}_{timestamp}")
    os.makedirs(output_dir, exist_ok=True)
    
    with open(os.path.join(output_dir, "history.json"), 'w') as f:
        json.dump(trainer.training_history, f, indent=2, cls=NumpyEncoder)
    
    trainer.save_agents(os.path.join(output_dir, "models"))
    print(f"\n✓ Training complete. Results saved to {output_dir}")

if __name__ == '__main__':
    main()
