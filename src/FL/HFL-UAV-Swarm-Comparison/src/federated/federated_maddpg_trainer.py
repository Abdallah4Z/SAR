"""
Federated MADDPG Trainer
Implements FedAvg aggregation for MADDPG agents across multiple clients.
"""

import torch
import numpy as np
import copy
from typing import List, Dict, Any
from src.algorithms.maddpg import MADDPGAgent, MADDPGTrainer
from src.federated import FedAvg

class FederatedMADDPGTrainer:
    """
    Federated Learning coordinator for MADDPG.
    Each client has a local swarm environment and trains MADDPG agents.
    The server aggregates agent weights using FedAvg.
    """
    
    def __init__(self, env_list, config, aggregator=None):
        """
        Args:
            env_list: List of environments (one per client)
            config: MADDPG configuration
            aggregator: Federated aggregation strategy (default: FedAvg)
        """
        self.config = config
        self.num_clients = len(env_list)
        self.device = torch.device("cuda" if config.use_gpu and torch.cuda.is_available() else "cpu")
        
        # Initialize aggregator
        self.aggregator = aggregator if aggregator is not None else FedAvg(config)
        
        # Create local trainers for each client
        self.client_trainers = [
            MADDPGTrainer(env, config) 
            for env in env_list
        ]
        
        # Global agents (template for initialization)
        self.num_agents = env_list[0].num_agents
        self.obs_dim = env_list[0].observation_space[0].shape[0]
        self.act_dim = env_list[0].action_space[0].shape[0]
        
        self.global_agents = [
            MADDPGAgent(config, self.obs_dim, self.act_dim, self.num_agents, i, self.device)
            for i in range(self.num_agents)
        ]
        
        print(f"Initialized Federated MADDPG with {self.num_clients} clients")
        print(f"Each client has {self.num_agents} agents")
    
    def train_federated(self, 
                       num_rounds, 
                       local_episodes, 
                       max_steps,
                       warmup_steps=1000,
                       client_sample_fraction=1.0):
        """
        Main federated training loop.
        
        Args:
            num_rounds: Number of federated learning rounds
            local_episodes: Episodes per client per round
            max_steps: Max steps per episode
            warmup_steps: Warmup steps before training
            client_sample_fraction: Fraction of clients to sample per round
        """
        
        round_metrics = {
            'global_rewards': [],
            'client_rewards': [],
            'aggregation_rounds': []
        }
        
        print(f"\n{'='*70}")
        print(f"Starting Federated MADDPG Training")
        print(f"{'='*70}")
        print(f"Num rounds: {num_rounds}")
        print(f"Local episodes per round: {local_episodes}")
        print(f"Num clients: {self.num_clients}")
        print(f"Client sampling fraction: {client_sample_fraction}")
        print(f"{'='*70}\n")
        
        for round_idx in range(num_rounds):
            print(f"\n--- Round {round_idx + 1}/{num_rounds} ---")
            
            # Sample clients for this round
            num_selected = max(1, int(self.num_clients * client_sample_fraction))
            selected_clients = np.random.choice(self.num_clients, num_selected, replace=False)
            print(f"Selected clients: {selected_clients.tolist()}")
            
            # Broadcast global model to selected clients
            for client_idx in selected_clients:
                self._send_global_to_client(client_idx)
            
            # Local training on each selected client
            client_rewards = []
            for client_idx in selected_clients:
                print(f"  Training client {client_idx}...")
                rewards = self._train_client(
                    client_idx, 
                    local_episodes, 
                    max_steps,
                    warmup_steps
                )
                avg_reward = np.mean(rewards)
                client_rewards.append(avg_reward)
                print(f"    Avg reward: {avg_reward:.2f}")
            
            # Aggregate models from selected clients
            print(f"  Aggregating models from {num_selected} clients...")
            self._aggregate_clients(selected_clients)
            
            # Evaluation (optional - evaluate global model)
            # global_reward = self._evaluate_global_model()
            # round_metrics['global_rewards'].append(global_reward)
            
            round_metrics['client_rewards'].append(client_rewards)
            round_metrics['aggregation_rounds'].append(round_idx)
            
            print(f"  Round {round_idx + 1} complete. Avg client reward: {np.mean(client_rewards):.2f}")
        
        print(f"\n{'='*70}")
        print(f"Federated Training Complete!")
        print(f"{'='*70}\n")
        
        return round_metrics
    
    def _send_global_to_client(self, client_idx):
        """Send global model weights to a specific client."""
        for agent_idx in range(self.num_agents):
            global_weights = self.global_agents[agent_idx].get_weights()
            self.client_trainers[client_idx].agents[agent_idx].set_weights(global_weights)
    
    @staticmethod
    def _to_numpy(x):
        """Convert observation to numpy, handling GPU tensors from UAVSwarmEnvGPU."""
        if isinstance(x, torch.Tensor):
            return x.detach().cpu().numpy()
        return np.asarray(x, dtype=np.float32)

    def _train_client(self, client_idx, num_episodes, max_steps, warmup_steps):
        """Train a single client locally."""
        trainer = self.client_trainers[client_idx]
        episode_rewards = []
        
        for episode in range(num_episodes):
            obs_raw = trainer.env.reset()
            # Convert observations to numpy for compatibility
            if isinstance(obs_raw, tuple):
                obs_raw = obs_raw[0]  # Handle (obs, info) return
            obs = [self._to_numpy(obs_raw[i]) for i in range(self.num_agents)] if isinstance(obs_raw, dict) else [self._to_numpy(o) for o in obs_raw]
            episode_reward = 0
            
            for step in range(max_steps):
                # Select actions
                actions = []
                for i, agent in enumerate(trainer.agents):
                    action = agent.select_action(obs[i], noise_scale=trainer.noise_scale)
                    actions.append(action)
                
                result = trainer.env.step(actions)
                if len(result) == 5:
                    next_obs_raw, rewards, dones, _, _ = result
                else:
                    next_obs_raw, rewards, dones, _ = result
                
                # Convert next observations
                if isinstance(next_obs_raw, dict):
                    next_obs = [self._to_numpy(next_obs_raw[i]) for i in range(self.num_agents)]
                else:
                    next_obs = [self._to_numpy(o) for o in next_obs_raw]
                
                # Handle rewards dict
                if isinstance(rewards, dict):
                    rewards_list = [rewards[i] for i in range(self.num_agents)]
                else:
                    rewards_list = list(rewards)
                
                # Handle dones
                if isinstance(dones, (bool, int, float)):
                    dones_list = [float(dones)] * self.num_agents
                elif isinstance(dones, dict):
                    dones_list = [float(dones.get(i, False)) for i in range(self.num_agents)]
                else:
                    dones_list = [float(d) for d in dones]
                
                # Store experience
                trainer.buffer.push(
                    np.stack(obs),
                    np.stack(actions),
                    np.array(rewards_list),
                    np.stack(next_obs),
                    np.array(dones_list)
                )
                
                obs = next_obs
                episode_reward += sum(rewards_list)
                
                if all(d > 0.5 for d in dones_list):
                    break
                
                # Update if buffer is ready
                if len(trainer.buffer) > warmup_steps and len(trainer.buffer) > self.config.batch_size:
                    sample = trainer.buffer.sample(self.config.batch_size)
                    for agent in trainer.agents:
                        agent.update(sample, trainer.agents)
            
            episode_rewards.append(episode_reward)
        
        return episode_rewards
    
    def _aggregate_clients(self, selected_clients):
        """Aggregate models from selected clients into global model."""
        for agent_idx in range(self.num_agents):
            # Collect weights from selected clients for this agent
            client_agents = [
                self.client_trainers[client_idx].agents[agent_idx]
                for client_idx in selected_clients
            ]
            
            # Perform FedAvg aggregation
            # Pass global agent and list of client agents
            self.aggregator.aggregate(
                self.global_agents[agent_idx],
                client_agents
            )
    
    def save_global_model(self, save_dir):
        """Save global agents."""
        import os
        os.makedirs(save_dir, exist_ok=True)
        for i, agent in enumerate(self.global_agents):
            agent.save(os.path.join(save_dir, f'global_agent_{i}.pth'))
        print(f"Global model saved to {save_dir}")
    
    def load_global_model(self, save_dir):
        """Load global agents."""
        import os
        for i, agent in enumerate(self.global_agents):
            agent.load(os.path.join(save_dir, f'global_agent_{i}.pth'))
        print(f"Global model loaded from {save_dir}")
    
    def get_global_agents(self):
        """Return global agents."""
        return self.global_agents
    
    def get_client_trainers(self):
        """Return list of client trainers."""
        return self.client_trainers
