import torch
import torch.nn as nn
import torch.nn.functional as F

class Actor(nn.Module):
    def __init__(self, obs_dim, act_dim, hidden_dim=64):
        super(Actor, self).__init__()
        self.fc1 = nn.Linear(obs_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, act_dim)

    def forward(self, obs):
        x = F.relu(self.fc1(obs))
        x = F.relu(self.fc2(x))
        return torch.tanh(self.fc3(x)) # Actions in range [-1, 1]

class Critic(nn.Module):
    def __init__(self, obs_dim, act_dim, num_agents, hidden_dim=64):
        super(Critic, self).__init__()
        # Centralized critic takes input from all agents
        self.total_obs_dim = obs_dim * num_agents
        self.total_act_dim = act_dim * num_agents
        
        self.fc1 = nn.Linear(self.total_obs_dim + self.total_act_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, 1)

    def forward(self, obs_full, act_full):
        """
        obs_full: (batch_size, num_agents * obs_dim) or (batch_size, num_agents, obs_dim)
        act_full: (batch_size, num_agents * act_dim) or (batch_size, num_agents, act_dim)
        """
        # Flatten inputs if they are not already flattened
        if obs_full.dim() > 2:
            obs_full = obs_full.view(obs_full.size(0), -1)
        if act_full.dim() > 2:
            act_full = act_full.view(act_full.size(0), -1)
            
        x = torch.cat([obs_full, act_full], dim=1)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        return self.fc3(x)
