"""
Evaluator — Run evaluation episodes for any MARL + FL combination.

Provides a unified interface to evaluate any combination of MARL agents
and FL aggregator over multiple episodes, collecting all relevant metrics
(reward, latency, success rate, energy, fairness).

Follows the same patterns as MAPPOTrainer.evaluate() from Belal's code,
but is algorithm-agnostic so it can evaluate MAPPO, MADDPG, and QMIX
using the same code path.

Usage:
    from src.evaluation.evaluator import Evaluator
    from src.evaluation.metrics_tracker import MetricsTracker

    evaluator = Evaluator(env, agents, config={'num_eval_episodes': 20})
    results = evaluator.evaluate()
"""

import numpy as np
from typing import Dict, List, Optional, Any
import time

from src.evaluation.metrics_tracker import MetricsTracker


class Evaluator:
    """
    Algorithm-agnostic evaluator for MARL agents in the UAV swarm environment.

    Runs deterministic evaluation episodes (agents in eval mode, no exploration)
    and aggregates metrics across episodes to produce a comprehensive report.

    Designed to be called from:
        - MAPPOTrainer.evaluate() (Belal)
        - MADDPGTrainer.evaluate() (Abdalla)
        - QMIXTrainer.evaluate() (Jo)
        - CombinationTester for full FL-MARL sweeps (Ammar)

    Args:
        env:     UAV swarm Gymnasium environment
        agents:  List of agents (any BaseAgent subclass)
        config:  Evaluation configuration dict
    """

    def __init__(self, env, agents: List, config: Optional[Dict] = None):
        self.env = env
        self.agents = agents
        self.num_agents = len(agents)

        # Defaults
        default_config = {
            'num_eval_episodes': 10,
            'max_steps_per_episode': None,  # None = use env limit
            'verbose': False,
        }
        if config:
            default_config.update(config)
        self.config = default_config

    def evaluate(
        self,
        num_episodes: Optional[int] = None,
        deterministic: bool = True
    ) -> Dict:
        """
        Run evaluation episodes and return aggregated metrics.

        Args:
            num_episodes:  Override config num_eval_episodes
            deterministic: If True, agents use greedy/mean actions

        Returns:
            dict: Aggregated metrics:
                avg_episode_reward, std_episode_reward,
                avg_latency_ms, avg_success_rate, avg_energy_j,
                avg_fairness_cv, avg_episode_length, ...
        """
        n_ep = num_episodes or self.config['num_eval_episodes']

        # Set agents to eval mode (disables exploration / dropout)
        for agent in self.agents:
            agent.set_eval_mode()

        episode_rewards = []
        episode_lengths = []
        all_metrics = []

        for ep in range(n_ep):
            ep_reward, ep_len, ep_metrics = self._run_episode(deterministic)
            episode_rewards.append(ep_reward)
            episode_lengths.append(ep_len)
            all_metrics.append(ep_metrics)

            if self.config['verbose']:
                print(f"  Episode {ep+1}/{n_ep}: reward={ep_reward:.3f}, "
                      f"steps={ep_len}")

        # Restore training mode
        for agent in self.agents:
            agent.set_train_mode()

        return self._aggregate_results(episode_rewards, episode_lengths, all_metrics)

    def _run_episode(self, deterministic: bool) -> tuple:
        """
        Run a single evaluation episode.

        Returns:
            tuple: (total_reward, episode_length, metrics_dict)
        """
        obs_dict, info = self.env.reset()
        done = False
        truncated = False
        ep_reward = {i: 0.0 for i in range(self.num_agents)}
        ep_len = 0

        # MetricsTracker for this episode
        tracker = MetricsTracker(num_uavs=self.num_agents)
        current_time = 0.0

        while not (done or truncated):
            # Build action dict from all agents (deterministic)
            actions_dict = {}
            for agent_id, agent in enumerate(self.agents):
                action = agent.select_action(obs_dict[agent_id], deterministic=deterministic)
                actions_dict[agent_id] = action

            obs_dict, rewards_dict, done, truncated, info = self.env.step(actions_dict)

            for agent_id in range(self.num_agents):
                ep_reward[agent_id] += rewards_dict[agent_id]

            ep_len += 1
            current_time += self.env.dt if hasattr(self.env, 'dt') else 0.01

            # Track per-step energy if env provides it
            if 'energy' in info:
                tracker.update_energy(info.get('energy', 0.0))

            # Track task events if env provides them
            if 'task_events' in info:
                for event in info['task_events']:
                    if event['type'] == 'submit':
                        tracker.on_task_submit(event['task_id'], current_time)
                    elif event['type'] == 'complete':
                        tracker.on_task_complete(
                            event['task_id'], current_time,
                            event.get('uav_id', 0),
                            event.get('success', True)
                        )

            # Respect max_steps override
            if (self.config['max_steps_per_episode'] is not None
                    and ep_len >= self.config['max_steps_per_episode']):
                break

        avg_reward = float(np.mean(list(ep_reward.values())))
        env_metrics = tracker.get_metrics(current_time)

        # Merge with info[\'metrics\'] if env provides summary
        if 'metrics' in info:
            env_metrics.update(info['metrics'])

        return avg_reward, ep_len, env_metrics

    def _aggregate_results(
        self,
        episode_rewards: List[float],
        episode_lengths: List[int],
        all_metrics: List[Dict]
    ) -> Dict:
        """
        Aggregate episode-level results into a single metrics dict.

        Args:
            episode_rewards: Per-episode total rewards
            episode_lengths: Per-episode step counts
            all_metrics:     Per-episode MetricsTracker results

        Returns:
            dict: Aggregated metrics
        """
        results = {
            'avg_episode_reward': float(np.mean(episode_rewards)),
            'std_episode_reward': float(np.std(episode_rewards)),
            'min_episode_reward': float(np.min(episode_rewards)),
            'max_episode_reward': float(np.max(episode_rewards)),
            'avg_episode_length': float(np.mean(episode_lengths)),
            'num_episodes': len(episode_rewards),
        }

        # Aggregate environment metrics
        if all_metrics:
            for key in all_metrics[0].keys():
                values = [m[key] for m in all_metrics if key in m]
                if values and isinstance(values[0], (int, float)):
                    results[f'avg_{key}'] = float(np.mean(values))
                    results[f'std_{key}'] = float(np.std(values))

        return results

    def evaluate_with_seeds(
        self,
        seeds: List[int],
        num_episodes_per_seed: int = 5
    ) -> Dict:
        """
        Evaluate over multiple random seeds for robustness.

        Resets environment seed before each seed block to ensure
        reproducible evaluation across experiments.

        Args:
            seeds:                 List of random seeds to evaluate on
            num_episodes_per_seed: Episodes per seed

        Returns:
            dict: Aggregated results across all seeds
        """
        all_rewards = []
        all_metrics = []

        for seed in seeds:
            # Seed the environment if it supports it
            if hasattr(self.env, 'seed'):
                self.env.seed(seed)

            results = self.evaluate(num_episodes=num_episodes_per_seed)
            all_rewards.append(results['avg_episode_reward'])
            all_metrics.append(results)

        return {
            'per_seed_rewards': all_rewards,
            'mean_reward': float(np.mean(all_rewards)),
            'std_reward': float(np.std(all_rewards)),
            'seeds': seeds,
            'num_episodes_per_seed': num_episodes_per_seed,
        }


if __name__ == '__main__':
    """Smoke test for Evaluator — uses a mock environment and agents."""

    print("Testing Evaluator...\n")

    # Mock environment
    class MockEnv:
        def __init__(self, num_agents=4):
            self.num_agents = num_agents
            self.dt = 0.01
            self._step = 0

        def reset(self):
            self._step = 0
            obs = {i: np.random.randn(40).astype(np.float32)
                   for i in range(self.num_agents)}
            return obs, {}

        def step(self, actions):
            self._step += 1
            obs = {i: np.random.randn(40).astype(np.float32)
                   for i in range(self.num_agents)}
            rewards = {i: float(np.random.rand()) for i in range(self.num_agents)}
            done = self._step >= 50
            return obs, rewards, done, False, {}

    # Mock agent (inherits BaseAgent-compatible interface)
    class MockAgent:
        def __init__(self, agent_id):
            self.agent_id = agent_id

        def select_action(self, obs, deterministic=False):
            return np.random.randn(5).astype(np.float32)

        def set_eval_mode(self):
            pass

        def set_train_mode(self):
            pass

    num_agents = 4
    env = MockEnv(num_agents)
    agents = [MockAgent(i) for i in range(num_agents)]

    print("Test 1: Basic evaluation")
    evaluator = Evaluator(env, agents, config={'num_eval_episodes': 5})
    results = evaluator.evaluate()
    assert 'avg_episode_reward' in results
    assert 'std_episode_reward' in results
    assert results['num_episodes'] == 5
    print(f"  avg_reward={results['avg_episode_reward']:.3f}, "
          f"std={results['std_episode_reward']:.3f}")
    print("  \u2713 Basic evaluation works")

    print("\nTest 2: Multi-seed evaluation")
    multi = evaluator.evaluate_with_seeds(seeds=[0, 1, 2], num_episodes_per_seed=3)
    assert len(multi['per_seed_rewards']) == 3
    print(f"  mean_reward={multi['mean_reward']:.3f}, std={multi['std_reward']:.3f}")
    print("  \u2713 Multi-seed evaluation works")

    print("\n\u2705 Evaluator tests passed!")
