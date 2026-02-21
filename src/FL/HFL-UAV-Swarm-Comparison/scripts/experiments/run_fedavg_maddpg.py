"""
Run FedAvg + MADDPG experiments across different swarm sizes.
Tests the combination of standard FedAvg with MADDPG algorithm.
"""

import os
import sys
import argparse
import numpy as np
import torch
from pathlib import Path

# Add project root to path
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from src.federated import FederatedMADDPGTrainer, FedAvg
from configs.marl_configs import MADDPGConfig
from configs.env_configs import get_config_for_swarm_size
from src.environments import make_maddpg_env

def parse_args():
    parser = argparse.ArgumentParser(description='FedAvg + MADDPG Experiments')
    parser.add_argument('--swarm_sizes', nargs='+', type=int, 
                        default=[5, 10, 20, 50, 100],
                        help='List of swarm sizes to test')
    parser.add_argument('--num_clients', type=int, default=5,
                        help='Number of federated clients')
    parser.add_argument('--num_rounds', type=int, default=100,
                        help='Number of federated rounds')
    parser.add_argument('--local_episodes', type=int, default=10,
                        help='Local episodes per round')
    parser.add_argument('--max_steps', type=int, default=200,
                        help='Max steps per episode')
    parser.add_argument('--warmup_steps', type=int, default=1000,
                        help='Warmup steps before training')
    parser.add_argument('--client_fraction', type=float, default=1.0,
                        help='Fraction of clients to sample per round')
    parser.add_argument('--save_dir', type=str, 
                        default='./results/fedavg_maddpg',
                        help='Directory to save results')
    parser.add_argument('--seed', type=int, default=42,
                        help='Random seed')
    parser.add_argument('--no_cuda', action='store_true',
                        help='Disable CUDA')
    return parser.parse_args()

def run_experiment(swarm_size, args):
    """Run a single experiment for a given swarm size."""
    
    print(f"\n{'#'*80}")
    print(f"# Experiment: FedAvg + MADDPG with {swarm_size} drones")
    print(f"{'#'*80}\n")
    
    # Set random seeds
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    
    # Initialize config
    config = MADDPGConfig()
    config.use_gpu = not args.no_cuda and torch.cuda.is_available()
    
    # Create one environment per client with MADDPG adapter
    env_list = []
    for i in range(args.num_clients):
        env_config = get_config_for_swarm_size(swarm_size, seed=args.seed + i)
        env = make_maddpg_env(env_config)
        env_list.append(env)
    
    print(f"Initialized {args.num_clients} environments with {swarm_size} drones each")
    print(f"  Observation dim: {env_list[0].observation_space[0].shape[0]}")
    print(f"  Action dim: {env_list[0].action_space[0].shape[0]}")
    
    # Initialize federated trainer with FedAvg
    aggregator = FedAvg(config)
    fed_trainer = FederatedMADDPGTrainer(env_list, config, aggregator)
    
    # Run federated training
    metrics = fed_trainer.train_federated(
        num_rounds=args.num_rounds,
        local_episodes=args.local_episodes,
        max_steps=args.max_steps,
        warmup_steps=args.warmup_steps,
        client_sample_fraction=args.client_fraction
    )
    
    # Save results
    save_path = os.path.join(args.save_dir, f'{swarm_size}_drones')
    os.makedirs(save_path, exist_ok=True)
    
    # Save global model
    model_path = os.path.join(save_path, 'final_model')
    fed_trainer.save_global_model(model_path)
    
    # Save metrics
    metrics_path = os.path.join(save_path, 'metrics.npz')
    np.savez(
        metrics_path,
        client_rewards=metrics['client_rewards'],
        aggregation_rounds=metrics['aggregation_rounds'],
        swarm_size=swarm_size,
        num_clients=args.num_clients,
        num_rounds=args.num_rounds
    )
    print(f"\nResults saved to {save_path}")
    
    # Print summary statistics
    print(f"\n{'='*70}")
    print(f"Experiment Summary: {swarm_size} drones")
    print(f"{'='*70}")
    
    all_rewards = [r for round_rewards in metrics['client_rewards'] for r in round_rewards]
    print(f"Mean reward across all rounds: {np.mean(all_rewards):.2f} ± {np.std(all_rewards):.2f}")
    
    # Last 10 rounds
    last_10_rewards = [r for round_rewards in metrics['client_rewards'][-10:] for r in round_rewards]
    print(f"Mean reward (last 10 rounds): {np.mean(last_10_rewards):.2f} ± {np.std(last_10_rewards):.2f}")
    
    print(f"{'='*70}\n")
    
    return metrics

def main():
    args = parse_args()
    
    print(f"\n{'*'*80}")
    print(f"* FedAvg + MADDPG Scaling Experiments")
    print(f"{'*'*80}")
    print(f"Swarm sizes to test: {args.swarm_sizes}")
    print(f"Number of clients: {args.num_clients}")
    print(f"Federated rounds: {args.num_rounds}")
    print(f"Local episodes per round: {args.local_episodes}")
    print(f"Client sampling fraction: {args.client_fraction}")
    print(f"Save directory: {args.save_dir}")
    print(f"{'*'*80}\n")
    
    # Run experiments for each swarm size
    all_results = {}
    
    for swarm_size in args.swarm_sizes:
        try:
            metrics = run_experiment(swarm_size, args)
            if metrics is not None:
                all_results[swarm_size] = metrics
        except Exception as e:
            print(f"\nERROR in experiment with {swarm_size} drones: {e}")
            import traceback
            traceback.print_exc()
            continue
    
    # Save combined results
    if all_results:
        combined_path = os.path.join(args.save_dir, 'combined_results.npz')
        np.savez(
            combined_path,
            swarm_sizes=args.swarm_sizes,
            results=all_results,
            num_clients=args.num_clients,
            num_rounds=args.num_rounds,
            local_episodes=args.local_episodes
        )
        print(f"\nCombined results saved to {combined_path}")
    
    print(f"\n{'*'*80}")
    print(f"* All Experiments Complete!")
    print(f"{'*'*80}\n")
    
    # Print comparison table
    if all_results:
        print(f"\n{'='*80}")
        print(f"Performance Comparison Across Swarm Sizes")
        print(f"{'='*80}")
        print(f"{'Swarm Size':<15} {'Mean Reward':<20} {'Last 10 Rounds':<20}")
        print(f"{'-'*80}")
        
        for swarm_size in args.swarm_sizes:
            if swarm_size in all_results:
                metrics = all_results[swarm_size]
                all_rewards = [r for round_rewards in metrics['client_rewards'] for r in round_rewards]
                last_10 = [r for round_rewards in metrics['client_rewards'][-10:] for r in round_rewards]
                
                print(f"{swarm_size:<15} {np.mean(all_rewards):>8.2f} ± {np.std(all_rewards):<8.2f} "
                      f"{np.mean(last_10):>8.2f} ± {np.std(last_10):<8.2f}")
        
        print(f"{'='*80}\n")

if __name__ == '__main__':
    main()
