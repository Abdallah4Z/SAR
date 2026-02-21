import numpy as np
import torch
import logging
import os
from datetime import datetime
from src.algorithms.maddpg.maddpg_agent import MADDPGAgent
from src.algorithms.maddpg.replay_buffer import ReplayBuffer

class MADDPGTrainer:
    def __init__(self, env, agents, fl_aggregator, config):
        self.env = env
        self.agents = agents
        self.fl_aggregator = fl_aggregator
        self.config = config
        
        # Handle both dict and object configs for compatibility
        def get_config_value(key, default=None):
            if isinstance(config, dict):
                return config.get(key, default)
            else:
                return getattr(config, key, default)
        
        self.device = torch.device(get_config_value('device', 'cuda') 
                                 if torch.cuda.is_available() else 'cpu')
        
        # Setup logger
        self.logger = self._setup_logger(config)
        
        self.num_agents = len(agents)
        self.obs_dim = get_config_value('obs_dim', 64)
        self.act_dim = get_config_value('action_dim', 5)
        
        self.batch_size = get_config_value('batch_size', 1024)
        self.noise_scale = get_config_value('noise_std', 0.1)
        self.noise_decay = 0.9999
        self.min_noise = 0.01
        
        self.logger.info("="*60)
        self.logger.info("Initializing MADDPG Trainer (Federated)")
        self.logger.info("="*60)
        self.logger.info(f"Device: {self.device}")
        self.logger.info(f"Number of agents: {self.num_agents}")
        
        # Experience buffer (Zero-copy GPU)
        self.buffer = ReplayBuffer(
            capacity=get_config_value('buffer_size', 100000),
            num_agents=self.num_agents,
            obs_dim=self.obs_dim,
            act_dim=self.act_dim,
            device=self.device
        )
        
        self.total_steps = 0
        self.training_history = []
        self.current_round = 0
        self.save_interval = 25  # Default: save checkpoint every 25 rounds

    def _fl_round(self):
        """Execute one federated learning round."""
        client_weights = [agent.get_weights() for agent in self.agents]
        
        # Aggregate
        global_weights = self.fl_aggregator.aggregate(client_weights)
        
        # Distribute
        distributed_weights = self.fl_aggregator.distribute(global_weights)
        
        if (isinstance(distributed_weights, dict) 
            and len(distributed_weights) == self.num_agents
            and 0 in distributed_weights):
            for i, agent in enumerate(self.agents):
                agent.set_weights(distributed_weights[i])
        else:
            for agent in self.agents:
                agent.set_weights(distributed_weights)
        
        # Collect FL metrics
        self.fl_aggregator.increment_round()
        return {
            'fl_round': self.fl_aggregator.get_round_number(),
        }
    
    def _setup_logger(self, config):
        """Setup detailed logger with file and console handlers."""
        # Create logs directory if it doesn't exist
        log_dir = getattr(config, 'log_dir', 'logs/maddpg')
        os.makedirs(log_dir, exist_ok=True)
        
        # Create logger
        logger = logging.getLogger('MADDPGTrainer')
        logger.setLevel(logging.DEBUG)
        
        # Remove existing handlers to avoid duplicates
        logger.handlers.clear()
        
        # Create timestamp for log file
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_file = os.path.join(log_dir, f'maddpg_training_{timestamp}.log')
        
        # File handler - detailed logs
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(logging.DEBUG)
        file_formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        file_handler.setFormatter(file_formatter)
        
        # Console handler - less detailed
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_formatter = logging.Formatter('%(levelname)s - %(message)s')
        console_handler.setFormatter(console_formatter)
        
        # Add handlers
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)
        
        logger.info(f"Log file created at: {log_file}")
        
        return logger

    def train(self, num_fl_rounds, episodes_per_round=5, max_steps=1000, warmup_steps=1000, resume=False):
        start_round = self.current_round if resume else 0
        
        self.logger.info("\n" + "="*60)
        if resume:
            self.logger.info(f"Resuming Federated MADDPG Training from Round {start_round + 1}/{num_fl_rounds}")
        else:
            self.logger.info("Starting Federated MADDPG Training")
        self.logger.info("="*60)
        
        episode_rewards = []
        current_noise = self.noise_scale
        best_reward = float('-inf')
        N = self.num_agents
        dev = self.device
        
        for fl_round in range(start_round, num_fl_rounds):
            self.current_round = fl_round
            self.logger.info(f"\n--- FL Round {fl_round+1}/{num_fl_rounds} Started ---")
            round_rewards = []
            
            for episode in range(episodes_per_round):
                res = self.env.reset()
                obs_tensor = res[0]
                if isinstance(obs_tensor, dict):
                    obs_tensor = torch.stack([obs_tensor[i] for i in range(N)])
                
                episode_reward = 0
                
                for step in range(max_steps):
                    # 1. Batched Action Selection (Zero-CPU)
                    with torch.no_grad():
                        # All agents in the swarm use the same model weights during the round
                        actions_stacked = self.agents[0].actor(obs_tensor)
                    
                    # Add noise
                    noise = torch.randn_like(actions_stacked, device=dev) * current_noise
                    actions_stacked = torch.clamp(actions_stacked + noise, -1.0, 1.0)
                    
                    # 2. Step environment
                    next_obs, rewards, done, truncated, info = self.env.step(actions_stacked)
                    
                    # 3. Store experience
                    self.buffer.push(
                        obs_tensor,
                        actions_stacked,
                        rewards,
                        next_obs,
                        torch.tensor([float(done or truncated)] * N, device=dev)
                    )
                    
                    obs_tensor = next_obs
                    episode_reward += rewards.sum().item()
                    self.total_steps += 1
                    
                    if done or truncated:
                        break
                        
                    # 4. Update
                    if len(self.buffer) > warmup_steps and len(self.buffer) > self.batch_size:
                        batch = self.buffer.sample(self.batch_size)
                        batch['agents'] = self.agents
                        for agent in self.agents:
                            agent.update(batch)
                    
                    current_noise = max(self.min_noise, current_noise * self.noise_decay)
                
                round_rewards.append(episode_reward)
                episode_rewards.append(episode_reward)
                
                if (episode + 1) % 5 == 0:
                    self.logger.info(f"  Episode {episode+1}/{episodes_per_round} completed. Reward: {episode_reward:.2f}")
            
            # End of local episodes: Run FL aggregation
            fl_metrics = self._fl_round()
            
            avg_round_reward = np.mean(round_rewards)
            self.logger.info(f"Round {fl_round+1} Completed. Avg Reward: {avg_round_reward:.2f}")
            
            if avg_round_reward > best_reward:
                best_reward = avg_round_reward
                self.logger.info(f"  *** New Best Round Reward: {best_reward:.2f} ***")
            
            # Periodic checkpoint saving
            if (fl_round + 1) % self.save_interval == 0:
                checkpoint_name = f"round_{fl_round + 1}"
                self.save_checkpoint(checkpoint_name)
                self.logger.info(f"  ✓ Checkpoint saved: {checkpoint_name}")
            
            # Store in history
            self.training_history.append({
                'round': fl_round + 1,
                'avg_reward': avg_round_reward,
                'total_steps': self.total_steps,
                **fl_metrics
            })
        
        return episode_rewards

    def save_agents(self, save_dir):
        """Save all agents to a directory."""
        import os
        self.logger.info(f"Saving agents to directory: {save_dir}")
        os.makedirs(save_dir, exist_ok=True)
        for i, agent in enumerate(self.agents):
            save_path = os.path.join(save_dir, f'agent_{i}.pth')
            agent.save(save_path)
            self.logger.debug(f"  Agent {i} saved to {save_path}")
        self.logger.info(f"Successfully saved all {len(self.agents)} agents")
    
    def save_checkpoint(self, checkpoint_name: str):
        """Save training checkpoint including trainer state, agents, and FL aggregator."""
        checkpoint_dir = os.path.join('checkpoints', 'maddpg', checkpoint_name)
        os.makedirs(checkpoint_dir, exist_ok=True)
        
        # Save each agent
        for i, agent in enumerate(self.agents):
            agent_path = os.path.join(checkpoint_dir, f'agent_{i}.pth')
            agent.save(agent_path)
        
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
            'training_history': self.training_history,
            'noise_scale': self.noise_scale,
            'fl_state': fl_state,
        }
        torch.save(trainer_state, os.path.join(checkpoint_dir, 'trainer_state.pt'))
        
        self.logger.info(f"Checkpoint saved: {checkpoint_dir}")
    
    def load_checkpoint(self, checkpoint_name: str):
        """Load training checkpoint to resume training."""
        checkpoint_dir = os.path.join('checkpoints', 'maddpg', checkpoint_name)
        trainer_state_path = os.path.join(checkpoint_dir, 'trainer_state.pt')
        
        if not os.path.exists(trainer_state_path):
            self.logger.warning(f"Checkpoint not found: {checkpoint_dir}")
            return False
        
        # Load trainer state
        trainer_state = torch.load(trainer_state_path)
        self.current_round = trainer_state['current_round']
        self.total_steps = trainer_state['total_steps']
        self.training_history = trainer_state['training_history']
        self.noise_scale = trainer_state.get('noise_scale', self.noise_scale)
        
        # Load each agent
        for i, agent in enumerate(self.agents):
            agent_path = os.path.join(checkpoint_dir, f'agent_{i}.pth')
            if os.path.exists(agent_path):
                agent.load(agent_path)
            else:
                self.logger.warning(f"Agent checkpoint not found: {agent_path}")
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
        
        self.logger.info(f"Checkpoint loaded: {checkpoint_dir}")
        self.logger.info(f"Resuming from round {self.current_round + 1}, total steps: {self.total_steps}")
        return True

    def load_agents(self, save_dir):
        """Load all agents from a directory."""
        import os
        self.logger.info(f"Loading agents from directory: {save_dir}")
        if not os.path.exists(save_dir):
            self.logger.error(f"Directory {save_dir} does not exist!")
            raise FileNotFoundError(f"Directory {save_dir} does not exist")
        
        for i, agent in enumerate(self.agents):
            load_path = os.path.join(save_dir, f'agent_{i}.pth')
            if not os.path.exists(load_path):
                self.logger.error(f"Agent file {load_path} not found!")
                raise FileNotFoundError(f"Agent file {load_path} not found")
            agent.load(load_path)
            self.logger.debug(f"  Agent {i} loaded from {load_path}")
        self.logger.info(f"Successfully loaded all {len(self.agents)} agents")

    def get_agents(self):
        return self.agents
