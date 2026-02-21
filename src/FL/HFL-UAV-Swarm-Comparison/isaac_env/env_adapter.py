"""
Adapter: Isaac Env → Gymnasium Dict Interface.

Wraps the IsaacEnv / StandaloneEnv to provide the same
Dict[int, np.ndarray] interface as UAVSwarmEnv, so existing
MAPPO/MADDPG training scripts work without modification.

Usage:
    from isaac_env.env_adapter import IsaacToGymAdapter
    from isaac_env.uav_task_offloading import StandaloneUAVTaskOffloadingEnv

    base_env = StandaloneUAVTaskOffloadingEnv(num_envs=1, num_uavs=4)
    env = IsaacToGymAdapter(base_env)

    obs, info = env.reset()
    # obs = {0: np.array(...), 1: np.array(...), ...}

    actions = {i: np.random.uniform(-1, 1, size=(5,)) for i in range(4)}
    obs, rewards, terminated, truncated, info = env.step(actions)
"""

import numpy as np
import torch
from typing import Dict, Tuple, Optional


class IsaacToGymAdapter:
    """
    Wraps the GPU-tensorized Isaac environment to provide the same
    Dict[int, np.ndarray] interface as the original UAVSwarmEnv.

    This adapter uses env_idx=0 of the batched environment,
    converting between PyTorch GPU tensors and NumPy arrays.

    This allows existing training scripts (train_mappo.py, etc.)
    to use the Isaac environment with zero code changes, simply
    by swapping which environment class is instantiated.
    """

    def __init__(self, isaac_env, env_idx: int = 0):
        """
        Args:
            isaac_env: StandaloneUAVTaskOffloadingEnv or UAVTaskOffloadingEnv instance
            env_idx: Which batched environment to expose (default: 0)
        """
        self.env = isaac_env
        self.env_idx = env_idx
        self.num_uavs = isaac_env.num_uavs
        self.obs_dim = isaac_env.obs_dim
        self.action_dim = isaac_env.action_dim

        # Mimic gymnasium spaces for compatibility
        self.observation_space = _DictSpace(self.num_uavs, self.obs_dim)
        self.action_space = _DictSpace(self.num_uavs, self.action_dim)

        # Internal state
        self._step_count = 0
        self._episode_length = getattr(isaac_env, 'episode_length', 1000)

    def reset(
        self, seed: Optional[int] = None, options: Optional[dict] = None
    ) -> Tuple[Dict[int, np.ndarray], Dict]:
        """
        Reset environment.

        Returns:
            obs: Dict[int, np.ndarray] — per-UAV observations
            info: Dict
        """
        if seed is not None:
            torch.manual_seed(seed)

        env_ids = torch.tensor([self.env_idx], device=self.env.device)
        result = self.env.reset(env_ids)

        obs = self._tensordict_to_obs(result)
        self._step_count = 0

        return obs, {}

    def step(
        self, actions: Dict[int, np.ndarray]
    ) -> Tuple[Dict[int, np.ndarray], Dict[int, float], bool, bool, Dict]:
        """
        Step environment.

        Args:
            actions: Dict mapping UAV id → np.ndarray of shape (5,)

        Returns:
            obs, rewards, terminated, truncated, info
        """
        # Convert Dict[int, np.ndarray] → tensor [1, num_uavs, 5]
        action_tensor = torch.zeros(
            self.env.num_envs, self.num_uavs, self.action_dim,
            device=self.env.device,
        )

        for uav_id, action in actions.items():
            action_tensor[self.env_idx, uav_id] = torch.tensor(
                action, dtype=torch.float32, device=self.env.device
            )

        tensordict = {"agents": {"action": action_tensor}}
        result = self.env.step(tensordict)

        # Extract obs
        obs = self._result_to_obs(result)

        # Extract rewards
        rewards_tensor = result["agents"]["reward"][self.env_idx]  # [U, 1]
        rewards = {
            i: rewards_tensor[i, 0].item() for i in range(self.num_uavs)
        }

        # Extract done
        done = result["done"][self.env_idx, 0].item()

        self._step_count += 1
        terminated = done
        truncated = self._step_count >= self._episode_length

        # Info
        info = {}
        if "stats" in result:
            stats = result["stats"]
            info = {
                "total_completed": stats["total_completed"][self.env_idx].item(),
                "total_failed": stats["total_failed"][self.env_idx].item(),
                "total_generated": stats["total_generated"][self.env_idx].item(),
                "mean_battery": stats["mean_battery"][self.env_idx].item(),
            }

        return obs, rewards, terminated, truncated, info

    def _tensordict_to_obs(self, result: dict) -> Dict[int, np.ndarray]:
        """Convert 'agents.observation' tensor to Dict[int, np.ndarray]."""
        obs_tensor = result["agents"]["observation"][self.env_idx]  # [U, obs_dim]
        return {
            i: obs_tensor[i].cpu().numpy() for i in range(self.num_uavs)
        }

    def _result_to_obs(self, result: dict) -> Dict[int, np.ndarray]:
        """Extract obs from step result."""
        obs_tensor = result["agents"]["observation"][self.env_idx]
        return {
            i: obs_tensor[i].cpu().numpy() for i in range(self.num_uavs)
        }


class _DictSpace:
    """Minimal space mock for compatibility with existing code."""

    def __init__(self, num_agents: int, dim: int):
        self.num_agents = num_agents
        self.dim = dim
        self.shape = (dim,)

    def __getitem__(self, key):
        return _BoxSpace(self.dim)

    def __contains__(self, key):
        return 0 <= key < self.num_agents

    def __iter__(self):
        return iter(range(self.num_agents))


class _BoxSpace:
    """Minimal Box space mock."""

    def __init__(self, dim: int):
        self.shape = (dim,)
        self.low = np.full(dim, -1.0, dtype=np.float32)
        self.high = np.full(dim, 1.0, dtype=np.float32)

    def sample(self):
        return np.random.uniform(self.low, self.high).astype(np.float32)
