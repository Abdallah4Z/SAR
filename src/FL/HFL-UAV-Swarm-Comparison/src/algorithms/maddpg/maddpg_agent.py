import torch
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
import copy
from src.algorithms.base.base_agent import BaseAgent
from src.algorithms.maddpg.maddpg_network import Actor, Critic
from src.algorithms.maddpg.target_network import TargetNetwork

class MADDPGAgent(BaseAgent):
    """
    Multi-Agent Deep Deterministic Policy Gradient (MADDPG) agent.
    
    MADDPG uses:
    - Decentralized actors: Each agent has its own policy
    - Centralized critic: Critic sees observations and actions of all agents
    - Off-policy learning: Uses experience replay
    - Target networks: For stable learning
    
    Observation Space: 40 floats
        - Position (3): x, y, z
        - Velocity (3): vx, vy, vz
        - Battery (1): percentage
        - CPU (1): frequency
        - Tasks (20): 5 tasks × 4 features each
        - Neighbors (12): 3 neighbors × 4 features each
    
    Action Space: 5 floats in [-1, 1]
        - target_x, target_y, target_z: where to fly
        - task_decision: process locally or offload
        - cpu_speed: CPU frequency setting
    """
    
    def __init__(self, agent_id: int, obs_dim: int, action_dim: int, config: dict):
        """
        Initialize MADDPG agent with BaseAgent API.
        
        Args:
            agent_id: Unique identifier for this agent (UAV ID)
            obs_dim: Observation dimension (should be 40 for UAV environment)
            action_dim: Action dimension (should be 5 for UAV environment)
            config: Configuration dictionary containing:
                - num_agents: Total number of agents in swarm
                - device: 'cpu' or 'cuda'
                - hidden_dim: Hidden layer size (default 256)
                - actor_lr: Actor learning rate (default 0.001)
                - critic_lr: Critic learning rate (default 0.001)
                - tau: Soft update parameter (default 0.005)
                - gamma: Discount factor (default 0.99)
                - noise_scale: Exploration noise (default 0.1)
        """
        super().__init__(agent_id, obs_dim, action_dim, config)
        
        # Extract config parameters with defaults
        self.num_agents = config.get('num_agents', 1)
        self.device = torch.device(config.get('device', 'cuda' if torch.cuda.is_available() else 'cpu'))
        self.hidden_dim = config.get('hidden_dim', 256)
        self.actor_lr = config.get('actor_lr', 0.001)
        self.critic_lr = config.get('critic_lr', 0.001)
        self.tau = config.get('tau', 0.005)
        self.gamma = config.get('gamma', 0.99)
        self.noise_scale = config.get('noise_scale', 0.1)
        
        # Initialize networks
        self.actor = Actor(obs_dim, action_dim, self.hidden_dim).to(self.device)
        self.critic = Critic(obs_dim, action_dim, self.num_agents, self.hidden_dim).to(self.device)
        
        # Target networks
        self.target_actor = copy.deepcopy(self.actor)
        self.target_critic = copy.deepcopy(self.critic)
        
        # Target network managers
        self.actor_target_manager = TargetNetwork(self.target_actor, self.actor, self.tau)
        self.critic_target_manager = TargetNetwork(self.target_critic, self.critic, self.tau)
        
        # Optimizers
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=self.actor_lr)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=self.critic_lr)

    def select_action(self, obs: np.ndarray, deterministic: bool = False) -> np.ndarray:
        """
        Select action for UAV control based on current observation.
        
        This method implements the MADDPG actor policy that outputs continuous
        actions for UAV movement, task offloading, and CPU frequency control.
        
        Args:
            obs: Observation array of shape (40,) containing:
                [0-2]   Position (x, y, z)
                [3-5]   Velocity (vx, vy, vz)
                [6]     Battery percentage
                [7]     CPU frequency (normalized)
                [8-27]  5 tasks in queue (cpu, data, deadline, priority) × 5
                [28-39] 3 neighbors (x, y, z, battery%) × 3
            deterministic: If True, return mean action without exploration noise.
                          If False, add Gaussian noise for exploration (training).
        
        Returns:
            action: Array of shape (5,) in range [-1, 1]:
                [0] target_x: X coordinate to fly to (mapped to area)
                [1] target_y: Y coordinate to fly to (mapped to area)
                [2] target_z: Altitude to fly to (mapped to height)
                [3] task_decision: Process locally or offload to neighbor
                [4] cpu_speed: CPU frequency setting (higher = faster but more battery)
        """
        obs_tensor = torch.FloatTensor(obs).to(self.device).unsqueeze(0)  # Add batch dim
        
        with torch.no_grad():
            action = self.actor(obs_tensor).cpu().numpy()[0]
        
        # Add exploration noise during training
        if not deterministic:
            noise = np.random.normal(0, self.noise_scale, size=self.action_dim)
            action = np.clip(action + noise, -1, 1)
        
        return action

    def store_transition(
        self, 
        obs: np.ndarray,
        action: np.ndarray,
        reward: float,
        next_obs: np.ndarray,
        done: bool,
        **kwargs
    ):
        """
        Store transition in replay buffer.
        
        Note: MADDPG trainer handles buffer storage for all agents together.
        This method is provided for BaseAgent API compatibility but typically
        not used directly. Use MADDPGTrainer's buffer instead.
        
        Args:
            obs: Observation (obs_dim,)
            action: Action (action_dim,)
            reward: Reward
            next_obs: Next observation (obs_dim,)
            done: Done flag
        """
        # MADDPG uses multi-agent replay buffer in trainer
        # This method is for API compatibility only
        pass

    def get_model_weights(self) -> dict:
        """
        Get ONLY actor weights for federated learning.
        
        For MADDPG, only the actor (policy) is shared via FL.
        The critic remains local (centralized training, decentralized execution).
        
        Returns:
            dict: Actor network state dict
        """
        return self.actor.state_dict()

    def set_model_weights(self, weights: dict):
        """
        Set ONLY actor weights from federated aggregation.
        
        Updates the actor network and performs hard update of target actor
        to ensure consistency after FL aggregation.
        
        Args:
            weights: Actor network state dict
        """
        self.actor.load_state_dict(weights)
        # Hard update target actor to match new weights
        self.actor_target_manager.hard_update()

    def update(self, batch: dict) -> dict:
        """
        Update MADDPG actor and critic networks from experience batch.
        
        This implements the MADDPG learning algorithm:
        1. Update centralized critic using Bellman equation
        2. Update decentralized actor using policy gradient
        3. Soft update target networks
        
        Args:
            batch: Dictionary containing:
                - 'obs': Observations (batch_size, num_agents, obs_dim)
                - 'actions': Actions (batch_size, num_agents, action_dim)
                - 'rewards': Rewards (batch_size, num_agents) or (batch_size, num_agents, 1)
                - 'next_obs': Next observations (batch_size, num_agents, obs_dim)
                - 'dones': Done flags (batch_size, num_agents) or (batch_size, num_agents, 1)
                - 'agents': List of all MADDPGAgent objects (for centralized critic)
        
        Returns:
            metrics: Dictionary containing:
                - 'critic_loss': TD error loss
                - 'actor_loss': Policy gradient loss
                - 'q_value': Average Q-value
        """
        # Unpack batch dictionary
        obs = batch['obs']  # (batch_size, num_agents, obs_dim)
        actions = batch['actions']  # (batch_size, num_agents, action_dim)
        rewards = batch['rewards']  # (batch_size, num_agents) or (batch_size, num_agents, 1)
        next_obs = batch['next_obs']  # (batch_size, num_agents, obs_dim)
        dones = batch['dones']  # (batch_size, num_agents) or (batch_size, num_agents, 1)
        agents = batch['agents']  # List of all MADDPGAgent objects
        
        # Ensure rewards and dones have correct shape
        if rewards.dim() == 3:
            rewards = rewards.squeeze(-1)  # (batch_size, num_agents)
        if dones.dim() == 3:
            dones = dones.squeeze(-1)  # (batch_size, num_agents)
        
        # 1. Update Critic (Centralized)
        
        # Collect target actions for next state from all agents
        with torch.no_grad():
            next_actions = []
            for i, agent in enumerate(agents):
                # Target actions from target policies
                next_act = agent.target_actor(next_obs[:, i, :])
                next_actions.append(next_act)
            next_actions = torch.cat(next_actions, dim=1)
            
            # Target critic value (centralized)
            # Flatten next_obs for critic
            next_obs_full = next_obs.view(next_obs.size(0), -1) 
            target_q = self.target_critic(next_obs_full, next_actions).squeeze(-1)
            
            # Bellman equation
            # Reward for this specific agent
            reward = rewards[:, self.agent_id]
            done = dones[:, self.agent_id]
            target_value = reward + (1 - done) * self.gamma * target_q
        
        # Current critic value (centralized)
        obs_full = obs.view(obs.size(0), -1)
        actions_full = actions.view(actions.size(0), -1)
        current_q = self.critic(obs_full, actions_full).squeeze(-1)
        
        # Critic loss (TD error)
        critic_loss = F.mse_loss(current_q, target_value)
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        # Gradient clipping for stability
        torch.nn.utils.clip_grad_norm_(self.critic.parameters(), max_norm=0.5)
        self.critic_optimizer.step()
        
        # 2. Update Actor (Decentralized)
        
        # Re-calculate actions with current policy for this agent only
        # Other agents' actions are detached (no gradient flow)
        curr_pol_actions = []
        for i, agent in enumerate(agents):
            if i == self.agent_id:
                # This agent's action - keep gradient
                curr_pol_actions.append(self.actor(obs[:, i, :]))
            else:
                # Other agents' actions - detach
                with torch.no_grad():
                    curr_pol_actions.append(agent.actor(obs[:, i, :]))
        
        curr_pol_actions_cat = torch.cat(curr_pol_actions, dim=1)
        
        # Actor loss: Maximize Q-value with current actor and others fixed
        actor_loss = -self.critic(obs_full, curr_pol_actions_cat).mean()
        
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        # Gradient clipping for stability
        torch.nn.utils.clip_grad_norm_(self.actor.parameters(), max_norm=0.5)
        self.actor_optimizer.step()
        
        # 3. Soft update target networks
        self.actor_target_manager.soft_update()
        self.critic_target_manager.soft_update()
        
        return {
            'critic_loss': critic_loss.item(),
            'actor_loss': actor_loss.item(),
            'q_value': current_q.mean().item()
        }

    def save(self, path):
        torch.save({
            'actor': self.actor.state_dict(),
            'critic': self.critic.state_dict(),
            'target_actor': self.target_actor.state_dict(),
            'target_critic': self.target_critic.state_dict(),
            'actor_optimizer': self.actor_optimizer.state_dict(),
            'critic_optimizer': self.critic_optimizer.state_dict(),
        }, path)

    def load(self, path):
        checkpoint = torch.load(path)
        self.actor.load_state_dict(checkpoint['actor'])
        self.critic.load_state_dict(checkpoint['critic'])
        self.target_actor.load_state_dict(checkpoint['target_actor'])
        self.target_critic.load_state_dict(checkpoint['target_critic'])
        self.actor_optimizer.load_state_dict(checkpoint['actor_optimizer'])
        self.critic_optimizer.load_state_dict(checkpoint['critic_optimizer'])

    def get_weights(self):
        """Return the weights of the actor and critic networks."""
        return {
            'actor': self.actor.state_dict(),
            'critic': self.critic.state_dict()
        }

    def set_weights(self, weights):
        """Set the weights of the actor and critic networks."""
        self.actor.load_state_dict(weights['actor'])
        self.critic.load_state_dict(weights['critic'])
        
        # Hard update target networks after FL aggregation to keep them synchronized
        # This ensures target networks reflect the global model state
        self.actor_target_manager.hard_update()
        self.critic_target_manager.hard_update()

