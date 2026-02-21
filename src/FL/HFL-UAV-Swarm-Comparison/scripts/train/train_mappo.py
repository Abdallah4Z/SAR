"""
Train MAPPO with any FL algorithm.

Usage:
    python scripts/train/train_mappo.py --num_drones 10 --fl hierarchical --seed 42
    python scripts/train/train_mappo.py --num_drones 20 --fl fedavg --num_rounds 150
    python scripts/train/train_mappo.py --num_drones 100 --fl hierarchical --save_dir results/hfl_100
"""

import argparse
import sys
import os
import json
import numpy as np
import torch
from datetime import datetime

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

# Import environment
from src.environments.uav_swarm_env import UAVSwarmEnv
from configs.env_configs import get_config_for_swarm_size

# Import MAPPO components
from src.algorithms.mappo.mappo_agent import MAPPOAgent
from src.algorithms.mappo.mappo_trainer import MAPPOTrainer

# Import FL aggregators
from src.federated.base_aggregator import FedAvgAggregator
from src.federated.hierarchical_aggregator import HierarchicalAggregator
from src.federated.fedprox import FedProxAggregator
from src.federated.fedadam import FedAdam

# Import configurations
from configs.fl_configs import get_fl_config_for_swarm_size
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
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Train MAPPO with Federated Learning',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    # Environment
    parser.add_argument(
        '--num_drones', type=int, default=4,
        help='Number of UAVs in the swarm (4, 10, 20, 50, 100)'
    )
    parser.add_argument(
        '--num_ground_devices', type=int, default=None,
        help='Number of ground devices (default: 5 * num_drones)'
    )
    
    # FL Algorithm
    parser.add_argument(
        '--fl_algorithm', '--fl', type=str, default='hierarchical',
        choices=['hierarchical', 'fedavg', 'fedprox', 'fedadam'],
        help='Federated learning algorithm'
    )
    
    # Training
    parser.add_argument(
        '--num_rounds', type=int, default=100,
        help='Number of FL rounds'
    )
    parser.add_argument(
        '--seed', type=int, default=42,
        help='Random seed for reproducibility'
    )
    
    # Output
    parser.add_argument(
        '--save_dir', type=str, default='results/mappo',
        help='Directory to save results'
    )
    parser.add_argument(
        '--experiment_name', type=str, default=None,
        help='Experiment name (default: auto-generated)'
    )
    
    # Hyperparameters (optional overrides)
    parser.add_argument(
        '--num_clusters', type=int, default=None,
        help='Number of clusters for HFL (default: auto)'
    )
    parser.add_argument(
        '--hidden_dim', type=int, default=None,
        help='Hidden dimension for networks'
    )
    parser.add_argument(
        '--lr_actor', type=float, default=None,
        help='Actor learning rate'
    )
    parser.add_argument(
        '--lr_critic', type=float, default=None,
        help='Critic learning rate'
    )
    
    # Evaluation
    parser.add_argument(
        '--eval_interval', type=int, default=5,
        help='Evaluate every N rounds'
    )
    parser.add_argument(
        '--save_interval', type=int, default=10,
        help='Save checkpoint every N rounds'
    )
    
    # Device
    parser.add_argument(
        '--device', type=str, default='auto',
        choices=['auto', 'cpu', 'cuda'],
        help='Device to use for training'
    )
    
    # GPU Environment (GPU is default, use --cpu_env for CPU debugging)
    parser.add_argument(
        '--cpu_env', action='store_true',
        help='Use CPU environment instead of GPU-tensorized (for debugging)'
    )
    
    # Parallel Environments (for better GPU utilization)
    parser.add_argument(
        '--num_parallel_envs', type=int, default=1,
        help='Number of parallel environments for GPU utilization (default: 1, recommended: 8-16)'
    )
    
    # Checkpointing
    parser.add_argument(
        '--resume', action='store_true',
        help='Resume training from checkpoint'
    )
    parser.add_argument(
        '--checkpoint_name', type=str, default=None,
        help='Checkpoint name to resume from (default: latest checkpoint for this config)'
    )
    
    return parser.parse_args()


def set_random_seeds(seed: int):
    """Set random seeds for reproducibility."""
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed(seed)
        torch.cuda.manual_seed_all(seed)
    # Make cudnn deterministic (slower but reproducible)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False


def create_aggregator(fl_algorithm: str, num_agents: int, fl_config: dict):
    """
    Create FL aggregator based on algorithm name.
    
    Args:
        fl_algorithm: Name of FL algorithm
        num_agents: Number of agents
        fl_config: FL configuration
    
    Returns:
        BaseAggregator: Instantiated aggregator
    """
    if fl_algorithm == 'hierarchical':
        return HierarchicalAggregator(
            num_agents=num_agents,
            num_clusters=fl_config.get('num_clusters', 'auto'),
            intra_cluster_rounds=fl_config.get('intra_cluster_rounds', 3),
            config=fl_config
        )
    elif fl_algorithm == 'fedavg':
        return FedAvgAggregator(config=fl_config)
    elif fl_algorithm == 'fedprox':
        return FedProxAggregator(
            num_agents=num_agents,
            num_rounds=fl_config.get('num_rounds', 100),
            local_epochs=fl_config.get('local_epochs', 5),
            mu=fl_config.get('mu', 0.01),
            adaptive_mu=False,
        )
    elif fl_algorithm == 'fedadam':
        return FedAdam(config=fl_config)
    else:
        raise ValueError(f"Unknown FL algorithm: {fl_algorithm}")


def main():
    """Main training function."""
    args = parse_args()
    
    # Print header
    print("\n" + "="*70)
    print("MAPPO Training with Federated Learning")
    print("="*70)
    print(f"UAV Swarm Size: {args.num_drones}")
    print(f"FL Algorithm: {args.fl_algorithm}")
    print(f"Training Rounds: {args.num_rounds}")
    print(f"Random Seed: {args.seed}")
    print("="*70 + "\n")
    
    # Set random seeds
    set_random_seeds(args.seed)
    print(f"✓ Random seeds set to {args.seed}")
    
    # Determine device
    if args.device == 'auto':
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
    else:
        device = args.device
    print(f"✓ Using device: {device}")
    if device == 'cuda':
        print(f"  GPU: {torch.cuda.get_device_name(0)}")
    
    # ================================================================
    # Step 1: Create environment
    # ================================================================
    print("\n[1/5] Creating environment...")
    env_config = get_config_for_swarm_size(args.num_drones, seed=args.seed)
    if args.num_ground_devices is not None:
        env_config['num_ground_devices'] = args.num_ground_devices
    
    if args.cpu_env:
        env = UAVSwarmEnv(env_config)
        print(f"✓ CPU environment created (debugging mode)")
    else:
        from src.environments.uav_swarm_env_gpu import UAVSwarmEnvGPU
        
        # Use parallel environments for better GPU utilization if requested
        if args.num_parallel_envs > 1:
            from src.environments.parallel_env_gpu import ParallelEnvGPU
            env = ParallelEnvGPU(env_config, num_parallel=args.num_parallel_envs, device=device)
        else:
            env = UAVSwarmEnvGPU(env_config, device=device)
            print(f"✓ GPU-tensorized environment created")
    
    print(f"✓ Environment created:")
    print(f"  UAVs: {env_config['num_uavs']}")
    print(f"  Ground devices: {env_config['num_ground_devices']}")
    print(f"  Area size: {env_config['area_size']}")
    print(f"  Episode length: {env_config['episode_length']} steps")
    
    # Check actual observation dimensions from environment
    obs_check, _ = env.reset()
    
    # Handle both regular and parallel environments
    if len(obs_check.shape) == 3:  # Parallel: (num_parallel, num_agents, obs_dim)
        first_obs = obs_check[0, 0]  # Take first env, first agent
    else:  # Regular: (num_agents, obs_dim)
        first_obs = obs_check[0]
    
    if hasattr(first_obs, 'shape'):
        actual_obs_dim = first_obs.shape[0]
    else:
        actual_obs_dim = len(first_obs)
    print(f"  Actual obs_dim from environment: {actual_obs_dim}")
    
    # ================================================================
    # Step 2: Load configurations
    # ================================================================
    print("\n[2/5] Loading configurations...")
    fl_config = get_fl_config_for_swarm_size(args.fl_algorithm, args.num_drones)
    marl_config = get_marl_config_for_swarm_size('mappo', args.num_drones)
    
    # Update obs_dim to match actual environment observations
    marl_config['obs_dim'] = actual_obs_dim
    
    # Override with command line arguments
    if args.num_clusters is not None and args.fl_algorithm == 'hierarchical':
        fl_config['num_clusters'] = args.num_clusters
    if args.hidden_dim is not None:
        marl_config['hidden_dim'] = args.hidden_dim
    if args.lr_actor is not None:
        marl_config['lr_actor'] = args.lr_actor
    if args.lr_critic is not None:
        marl_config['lr_critic'] = args.lr_critic
    
    # Add evaluation parameters
    marl_config['eval_interval'] = args.eval_interval
    marl_config['save_interval'] = args.save_interval
    
    print(f"✓ FL Config: {args.fl_algorithm}")
    for key, value in list(fl_config.items())[:5]:
        print(f"    {key}: {value}")
    print(f"✓ MARL Config: MAPPO")
    for key, value in list(marl_config.items())[:5]:
        print(f"    {key}: {value}")
    
    # ================================================================
    # Step 3: Create agents
    # ================================================================
    print("\n[3/5] Creating MAPPO agents...")
    agents = []
    for agent_id in range(args.num_drones):
        agent = MAPPOAgent(
            agent_id=agent_id,
            obs_dim=marl_config['obs_dim'],
            action_dim=marl_config['action_dim'],
            num_agents=args.num_drones,
            config=marl_config
        )
        agents.append(agent)
    
    print(f"✓ Created {len(agents)} MAPPO agents")
    print(f"  Obs dim: {marl_config['obs_dim']}")
    print(f"  Action dim: {marl_config['action_dim']}")
    print(f"  Hidden dim: {marl_config['hidden_dim']}")
    total_params = sum(p.numel() for agent in agents for p in agent.actor.parameters())
    print(f"  Total parameters: {total_params:,}")
    
    # ================================================================
    # Step 4: Create FL aggregator
    # ================================================================
    print("\n[4/5] Creating FL aggregator...")
    aggregator = create_aggregator(args.fl_algorithm, args.num_drones, fl_config)
    print(f"✓ Created {type(aggregator).__name__}")
    
    if args.fl_algorithm == 'hierarchical':
        cluster_info = aggregator.get_cluster_info()
        print(f"  Initial clusters: {cluster_info['num_clusters']}")
        print(f"  Cluster sizes: {cluster_info['cluster_sizes']}")
        comm_saving = 1.0 - (cluster_info['num_clusters'] / args.num_drones)
        print(f"  Est. communication saving: {comm_saving:.1%}")
    
    # ================================================================
    # Step 5: Create trainer and start training
    # ================================================================
    print("\n[5/5] Creating trainer and starting training...")
    trainer = MAPPOTrainer(
        env=env,
        agents=agents,
        fl_aggregator=aggregator,
        config=marl_config
    )
    
    print(f"✓ Trainer created")
    print(f"  Rollout length: {marl_config['rollout_length']}")
    print(f"  PPO epochs: {marl_config['ppo_epochs']}")
    print(f"  Mini-batches: {marl_config['num_mini_batches']}")
    
    # Load checkpoint if resuming
    resume = False
    if args.resume:
        if args.checkpoint_name:
            checkpoint_name = args.checkpoint_name
        else:
            # Auto-detect latest checkpoint for this configuration
            checkpoint_name = f"{args.num_drones}drones_{args.fl_algorithm}"
        
        print(f"\nAttempting to resume from checkpoint: {checkpoint_name}")
        if trainer.load_checkpoint(checkpoint_name):
            resume = True
        else:
            print(f"  Starting from scratch (checkpoint not found)\n")
    
    # Start training
    print("\n" + "="*70)
    print("Starting training...")
    print("="*70)
    
    try:
        results = trainer.train(num_fl_rounds=args.num_rounds, resume=resume)
    except KeyboardInterrupt:
        print("\n\nTraining interrupted by user!")
        # Save checkpoint on interrupt
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        interrupt_checkpoint = f"{args.num_drones}drones_{args.fl_algorithm}_interrupted_{timestamp}"
        print(f"Saving checkpoint: {interrupt_checkpoint}")
        trainer.save_checkpoint(interrupt_checkpoint)
        results = {
            'history': trainer.training_history,
            'interrupted': True
        }
    
    # ================================================================
    # Step 6: Save results
    # ================================================================
    print("\n" + "="*70)
    print("Saving results...")
    print("="*70)
    
    # Generate experiment name
    if args.experiment_name is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        experiment_name = f"{args.num_drones}drones_{args.fl_algorithm}_seed{args.seed}_{timestamp}"
    else:
        experiment_name = args.experiment_name
    
    # Create save directory
    save_dir = os.path.join(args.save_dir, experiment_name)
    os.makedirs(save_dir, exist_ok=True)
    
    # Save training results
    trainer.save_results(save_dir)
    
    # Save experiment configuration
    experiment_config = {
        'num_drones': args.num_drones,
        'fl_algorithm': args.fl_algorithm,
        'num_rounds': args.num_rounds,
        'seed': args.seed,
        'env_config': env_config,
        'fl_config': fl_config,
        'marl_config': marl_config,
        'device': device,
        'timestamp': datetime.now().isoformat(),
    }
    
    config_path = os.path.join(save_dir, 'experiment_config.json')
    with open(config_path, 'w') as f:
        json.dump(experiment_config, f, indent=2, cls=NumpyEncoder)
    
    print(f"✓ Results saved to: {save_dir}")
    print(f"  - training_history.json")
    print(f"  - experiment_config.json")
    print(f"  - final_agent_*.pt")
    
    # Print final summary
    if results.get('final_metrics'):
        print("\n" + "="*70)
        print("Training Summary")
        print("="*70)
        final = results['final_metrics']
        
        if 'ppo_actor_loss' in final:
            print(f"Final Actor Loss: {final['ppo_actor_loss']:.4f}")
            print(f"Final Critic Loss: {final['ppo_critic_loss']:.4f}")
            print(f"Final Entropy: {final['ppo_entropy']:.4f}")
        
        if 'eval' in final:
            eval_metrics = final['eval']
            print(f"\nFinal Evaluation:")
            print(f"  Avg Reward: {eval_metrics['avg_episode_reward']:.2f} ± {eval_metrics['std_episode_reward']:.2f}")
            if 'avg_success_rate' in eval_metrics:
                print(f"  Success Rate: {eval_metrics['avg_success_rate']:.2%}")
            if 'avg_avg_latency_ms' in eval_metrics:
                print(f"  Avg Latency: {eval_metrics['avg_avg_latency_ms']:.2f}ms")
        
        if args.fl_algorithm == 'hierarchical':
            savings = aggregator.get_communication_savings()
            if savings:
                avg_saving = np.mean([s['saving_ratio'] for s in savings])
                print(f"\nCommunication Savings: {avg_saving:.1%}")
    
    print("\n" + "="*70)
    print("Training completed successfully!")
    print("="*70 + "\n")


if __name__ == '__main__':
    main()
