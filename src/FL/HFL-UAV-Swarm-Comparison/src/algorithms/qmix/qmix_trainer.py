"""
QMIX Trainer with Federated Learning

Orchestrates QMIX training with FL aggregation.
Handles discrete action conversion and replay buffer management.
"""

import numpy as np
import torch
import json
from typing import Dict, Tuple
from pathlib import Path
from datetime import datetime

from src.algorithms.qmix.qmix_agent import QMIXAgent
from src.algorithms.qmix.action_discretizer import ActionDiscretizer
from src.federated.base_aggregator import BaseAggregator


class QMIXTrainer:
    """
    QMIX trainer with FL integration.
    
    Manages:
    - Trajectory collection (discrete actions)
    - Experience replay
    - QMIX Q-learning updates
    - FL aggregation
    """
    
    def __init__(self,
                 agents: Dict[int, QMIXAgent],
                 env,
                 fl_aggregator: BaseAggregator,
                 rollout_length = 'episode',
                 update_interval: int = 1,
                 save_interval: int = 50):
        """
        Initialize QMIX trainer.
        
        Args:
            agents: Dict of agents
            env: Environment
            fl_aggregator: FL aggregation strategy
            rollout_length: 'episode' for full episode, or int for fixed steps
            update_interval: Q-learning update frequency
            save_interval: Save checkpoint every N rounds (default: 50)
        """
        self.agents = agents
        self.env = env
        self.fl_aggregator = fl_aggregator
        self.num_agents = len(agents)
        self.rollout_length = rollout_length
        self.update_interval = update_interval
        self.save_interval = save_interval
        
        # Action discretizer
        self.discretizer = ActionDiscretizer(self.num_agents)
        
        # Training metrics
        self.episode_count = 0
        self.total_steps = 0
        self.current_round = 0
        self.training_history = []
        
        # Store global weights for FedProx proximal term
        self.global_weights = None
        
        # Get proximal_mu from aggregator if it's FedProx
        self.proximal_mu = getattr(fl_aggregator, 'mu', 0.0)
    
    def train(self, num_fl_rounds: int = 100, resume: bool = False) -> Dict:
        """
        Run QMIX training with FL.
        
        Args:
            num_fl_rounds: Number of federated learning rounds
            resume: If True, resume from current_round
        
        Returns:
            Dict with training history
        """
        start_round = self.current_round + 1 if resume else 1
        
        history = {
            'fl_round': [],
            'avg_loss': [],
            'avg_epsilon': [],
            'communication_savings': [],
        }
        
        if resume:
            print(f"\nResuming QMIX training from round {start_round}/{num_fl_rounds}")
        
        for fl_round in range(start_round, num_fl_rounds + 1):
            self.current_round = fl_round
            # Collect trajectories and store in replay buffers
            self._collect_and_buffer_trajectories(self.rollout_length)
            
            # QMIX Q-learning updates (multiple passes over replay buffers)
            avg_loss, avg_epsilon = self._qmix_update()
            
            # Federated learning aggregation
            fl_metrics = self._fl_round()
            
            # Log metrics
            history['fl_round'].append(fl_round)
            history['avg_loss'].append(avg_loss)
            history['avg_epsilon'].append(avg_epsilon)
            history['communication_savings'].append(fl_metrics['communication_savings'])
            
            # Print progress
            print(f"FL Round {fl_round}/{num_fl_rounds}")
            print(f"  Loss: {avg_loss:.4f} | Epsilon: {avg_epsilon:.4f}")
            print(f"  Comm Savings: {fl_metrics['communication_savings']:.1f}%")
            
            # Periodic checkpoint saving
            if fl_round % self.save_interval == 0:
                checkpoint_name = f"round_{fl_round}"
                self.save_checkpoint(checkpoint_name)
                print(f"  ✓ Checkpoint saved: {checkpoint_name}")
        
        return history
    
    @staticmethod
    def _to_numpy(x):
        """Convert observation to numpy, handling GPU tensors from UAVSwarmEnvGPU."""
        if isinstance(x, torch.Tensor):
            return x.detach().cpu().numpy()
        return np.asarray(x, dtype=np.float32)

    def _collect_and_buffer_trajectories(self, rollout_length):
        """
        Collect trajectories and store in agent replay buffers (Vectorized).
        """
        full_episode = (rollout_length == 'episode')
        max_steps = 1000 if full_episode else int(rollout_length)
        N = self.num_agents
        dev = self.agents[0].device

        # Reset environment
        res = self.env.reset()
        obs_tensor = res[0] # (N, obs_dim)
        if isinstance(obs_tensor, dict):
            obs_tensor = torch.stack([obs_tensor[i] for i in range(N)])
        
        for step in range(max_steps):
            # 1. Global observation
            global_obs = obs_tensor.reshape(1, -1) # (1, N*obs_dim)
            
            # 2. Batched Action Selection (Zero-CPU)
            # Use agent 0 to select actions for the entire swarm batch
            action_indices_tensor = self.agents[0].select_action(obs_tensor)
            action_indices = action_indices_tensor if N > 1 else [action_indices_tensor]
            
            # 3. Convert discrete to continuous (Vectorized)
            continuous_actions = self.discretizer.discrete_to_continuous_tensor(action_indices_tensor)
            
            # 4. Step environment
            next_obs, rewards, done, truncated, info = self.env.step(continuous_actions)
            next_global_obs = next_obs.reshape(1, -1)
            
            # 5. Store in replay buffers
            # Transitions are stored individually because replay buffers are per-agent
            # (Though in QMIX we could use a single joint buffer)
            for i in range(N):
                self.agents[i].store_transition(
                    obs=obs_tensor[i],
                    action=action_indices[i],
                    reward=rewards[i],
                    next_obs=next_obs[i],
                    done=float(done or truncated),
                    global_obs=global_obs.squeeze(0),
                    next_global_obs=next_global_obs.squeeze(0),
                )
            
            obs_tensor = next_obs
            self.total_steps += 1
            if done or truncated:
                break
    
    def _qmix_update(self) -> Tuple[float, float]:
        """
        Run QMIX Q-learning updates WITH proximal term for FedProx.
        
        Returns:
            Tuple of (avg_loss, avg_epsilon)
        """
        losses = []
        epsilons = []
        proximal_losses = []
        
        # Each agent performs multiple Q-learning updates
        num_updates = 5  # Updates per agent per FL round
        for _ in range(num_updates):
            for agent_id, agent in self.agents.items():
                # Pass global weights and proximal_mu for FedProx
                metrics = agent.update(
                    batch={}, 
                    is_fl_round=True,
                    global_weights=self.global_weights,
                    proximal_mu=self.proximal_mu
                )
                if metrics['loss'] > 0:
                    losses.append(metrics['loss'])
                    epsilons.append(metrics['epsilon'])
                    if 'proximal_loss' in metrics:
                        proximal_losses.append(metrics['proximal_loss'])
        
        avg_loss = np.mean(losses) if losses else 0.0
        avg_epsilon = np.mean(epsilons) if epsilons else 0.0
        avg_proximal_loss = np.mean(proximal_losses) if proximal_losses else 0.0
        
        return avg_loss, avg_epsilon
    
    def _fl_round(self) -> Dict:
        """
        Perform federated learning aggregation.
        
        For FedProx: Store global weights BEFORE local training (not after).
        
        Returns:
            Dict with FL metrics
        """
        # Collect weights from all agents (as list for aggregator interface)
        client_weights = [self.agents[agent_id].get_model_weights() 
                         for agent_id in range(self.num_agents)]
        
        # Optional: Get agent positions for HFL clustering
        agent_positions = None
        if hasattr(self.fl_aggregator, 'update_clusters'):
            try:
                agent_positions = {
                    i: np.array([self.env.uavs[i].position[0], 
                                self.env.uavs[i].position[1],
                                self.env.uavs[i].position[2]])
                    for i in range(self.num_agents)
                }
            except:
                agent_positions = None
        
        # Aggregate weights
        if agent_positions is not None:
            global_weights = self.fl_aggregator.aggregate(
                client_weights,
                client_positions=agent_positions
            )
        else:
            global_weights = self.fl_aggregator.aggregate(client_weights)
        
        # ═══════════════════════════════════════════════════════════════
        # CRITICAL FIX: Store global weights for FedProx proximal term
        # ═══════════════════════════════════════════════════════════════
        # For FedProx, we need to store the global weights so that the next
        # round of local training can add the proximal term to the loss
        if isinstance(global_weights, list) and len(global_weights) > 0:
            self.global_weights = global_weights[0]  # Use first agent's weights as global
        elif isinstance(global_weights, dict):
            self.global_weights = global_weights
        
        # Distribute aggregated weights back to agents
        distributed_weights = self.fl_aggregator.distribute(global_weights)
        
        # Handle both dict return types:
        # - Per-agent dict {agent_id: weights}
        # - Global dict (same weights for all) sent to each agent
        if isinstance(distributed_weights, dict):
            if len(distributed_weights) == self.num_agents and 0 in distributed_weights:
                # Per-agent weights
                for agent_id in range(self.num_agents):
                    self.agents[agent_id].set_model_weights(distributed_weights[agent_id])
            else:
                # Global weights (same for all agents)
                for agent_id in range(self.num_agents):
                    self.agents[agent_id].set_model_weights(distributed_weights)
        elif isinstance(distributed_weights, list):
            for agent_id in range(self.num_agents):
                self.agents[agent_id].set_model_weights(distributed_weights[agent_id])
        
        # Get communication metrics
        if hasattr(self.fl_aggregator, 'get_communication_metrics'):
            comm_metrics = self.fl_aggregator.get_communication_metrics()
        else:
            comm_metrics = {'communication_savings': 0.0}
        
        return comm_metrics
    
    def evaluate(self, num_episodes: int = 10) -> Dict:
        """
        Evaluate QMIX agents.
        
        Args:
            num_episodes: Number of evaluation episodes
        
        Returns:
            Dict with evaluation metrics
        """
        episode_rewards = []
        
        for episode in range(num_episodes):
            obs_dict, _ = self.env.reset()
            episode_return = 0.0
            
            # Run episode (greedy, no exploration)
            for step in range(10000):  # Max 10k steps
                obs_np = {i: self._to_numpy(obs_dict[i]) for i in range(self.num_agents)}
                global_obs = np.concatenate([obs_np[i] for i in range(self.num_agents)])
                
                # Greedy actions
                discrete_actions = {}
                for agent_id, agent in self.agents.items():
                    discrete_actions[agent_id] = agent.select_action_greedy(obs_np[agent_id])
                
                # Convert to continuous
                continuous_actions = self.discretizer.discrete_to_continuous(discrete_actions)
                
                # Step environment
                next_obs_dict, rewards_dict, done, truncated, info = self.env.step(continuous_actions)
                
                # Sum rewards (handle GPU-tensorized env that returns tensors, not dicts)
                if isinstance(rewards_dict, dict):
                    total_reward = sum(r.item() if torch.is_tensor(r) else r for r in rewards_dict.values())
                elif torch.is_tensor(rewards_dict):
                    # GPU env returns tensor of rewards for all agents
                    total_reward = rewards_dict.sum().item()
                else:
                    # Assume it's an array or list
                    total_reward = float(sum(rewards_dict))
                episode_return += total_reward
                
                if done or truncated:
                    break
                
                obs_dict = next_obs_dict
            
            episode_rewards.append(episode_return)
        
        return {
            'avg_reward': np.mean(episode_rewards),
            'std_reward': np.std(episode_rewards),
            'episode_rewards': episode_rewards,
        }
    
    def save_checkpoint(self, checkpoint_name: str):
        """
        Save training checkpoint including trainer state, agents, and FL aggregator.
        
        Args:
            checkpoint_name: Name for this checkpoint
        """
        checkpoint_dir = Path('checkpoints') / 'qmix' / checkpoint_name
        checkpoint_dir.mkdir(parents=True, exist_ok=True)
        
        # Save each agent
        for agent_id, agent in self.agents.items():
            agent.save_checkpoint(str(checkpoint_dir / f"agent_{agent_id}.pt"))
        
        # Save FL aggregator state
        fl_state = {}
        if hasattr(self.fl_aggregator, 'm'):  # FedAdam
            fl_state['m'] = self.fl_aggregator.m
            fl_state['v'] = self.fl_aggregator.v
            fl_state['round_number'] = self.fl_aggregator.round_number
        elif hasattr(self.fl_aggregator, 'global_weights'):  # FedProx
            fl_state['global_weights'] = self.fl_aggregator.global_weights
            fl_state['round_number'] = self.fl_aggregator.round_number
        else:
            fl_state['round_number'] = self.fl_aggregator.round_number
        
        # Save trainer state
        trainer_state = {
            'current_round': self.current_round,
            'total_steps': self.total_steps,
            'episode_count': self.episode_count,
            'training_history': self.training_history,
            'fl_state': fl_state,
        }
        torch.save(trainer_state, checkpoint_dir / 'trainer_state.pt')
        print(f"  Checkpoint saved: {checkpoint_dir}")
    
    def load_checkpoint(self, checkpoint_name: str) -> bool:
        """
        Load training checkpoint to resume training.
        
        Args:
            checkpoint_name: Name of checkpoint to load
        
        Returns:
            bool: True if successful, False otherwise
        """
        checkpoint_dir = Path('checkpoints') / 'qmix' / checkpoint_name
        trainer_state_path = checkpoint_dir / 'trainer_state.pt'
        
        if not trainer_state_path.exists():
            print(f"  Warning: Checkpoint not found: {checkpoint_dir}")
            return False
        
        # Load trainer state
        trainer_state = torch.load(trainer_state_path)
        self.current_round = trainer_state['current_round']
        self.total_steps = trainer_state['total_steps']
        self.episode_count = trainer_state['episode_count']
        self.training_history = trainer_state.get('training_history', [])
        
        # Load each agent
        for agent_id, agent in self.agents.items():
            agent_path = checkpoint_dir / f"agent_{agent_id}.pt"
            if agent_path.exists():
                agent.load_checkpoint(str(agent_path))
            else:
                print(f"  Warning: Agent checkpoint not found: {agent_path}")
                return False
        
        # Load FL aggregator state
        if 'fl_state' in trainer_state:
            fl_state = trainer_state['fl_state']
            if 'm' in fl_state and hasattr(self.fl_aggregator, 'm'):
                self.fl_aggregator.m = fl_state['m']
                self.fl_aggregator.v = fl_state['v']
            if 'global_weights' in fl_state and hasattr(self.fl_aggregator, 'global_weights'):
                self.fl_aggregator.global_weights = fl_state['global_weights']
            if 'round_number' in fl_state:
                self.fl_aggregator.round_number = fl_state['round_number']
        
        print(f"  Checkpoint loaded: {checkpoint_dir}")
        print(f"  Resuming from round {self.current_round}, total steps: {self.total_steps}")
        return True
