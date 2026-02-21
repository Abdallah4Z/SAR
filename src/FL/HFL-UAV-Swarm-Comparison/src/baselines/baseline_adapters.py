"""
BaselineAdapters — Run and store baseline evaluation results.

Provides adapters that:
    1. Run a baseline in the UAV environment using standard Gymnasium API
    2. Collect episode rewards compatible with LiteratureComparison
    3. Store results to disk for later analysis

Usage:
    from src.baselines.baseline_adapters import BaselineAdapter

    adapter = BaselineAdapter(env, 'greedy_offload', num_agents=10)
    results = adapter.run(num_episodes=50)
    adapter.save_results(results, 'results/literature_comparison/')
"""

import os
import json
import numpy as np
from typing import Dict, List, Optional

from src.baselines.literature_baselines import get_baseline_agents


class BaselineAdapter:
    """
    Adapter to run a named baseline in the UAV swarm environment and
    collect results compatible with LiteratureComparison.

    Args:
        env:           UAV swarm Gymnasium environment
        baseline_name: Key from BASELINE_CONFIGS ('greedy_offload', etc.)
        num_agents:    Number of UAVs
        config:        Optional configuration overrides
    """

    def __init__(
        self,
        env,
        baseline_name: str,
        num_agents: int,
        config: Optional[Dict] = None
    ):
        self.env = env
        self.baseline_name = baseline_name
        self.num_agents = num_agents

        default_config = {
            'num_eval_episodes': 50,
            'max_steps': None,
            'verbose': False,
        }
        if config:
            default_config.update(config)
        self.config = default_config

        # Build agent list
        self.agents = get_baseline_agents(baseline_name, num_agents)

    def run(self, num_episodes: Optional[int] = None) -> Dict:
        """
        Run baseline evaluation for the specified number of episodes.

        Args:
            num_episodes: Override config num_eval_episodes

        Returns:
            dict: {
                'rewards': [float, ...],      # per-episode rewards
                'episode_lengths': [int, ...],
                'mean_reward': float,
                'std_reward': float,
                'baseline_name': str,
                'num_agents': int,
            }
        """
        n_ep = num_episodes or self.config['num_eval_episodes']

        episode_rewards = []
        episode_lengths = []

        for ep in range(n_ep):
            obs_dict, info = self.env.reset()
            done = False
            truncated = False
            ep_reward = {i: 0.0 for i in range(self.num_agents)}
            ep_len = 0

            while not (done or truncated):
                actions_dict = {}
                for agent in self.agents:
                    actions_dict[agent.agent_id] = agent.select_action(
                        obs_dict[agent.agent_id], deterministic=True
                    )

                obs_dict, rewards_dict, done, truncated, info = self.env.step(actions_dict)
                for i in range(self.num_agents):
                    ep_reward[i] += rewards_dict[i]
                ep_len += 1

                if (self.config['max_steps'] is not None
                        and ep_len >= self.config['max_steps']):
                    break

            avg_reward = float(np.mean(list(ep_reward.values())))
            episode_rewards.append(avg_reward)
            episode_lengths.append(ep_len)

            if self.config['verbose']:
                print(f"  Ep {ep+1}/{n_ep}: reward={avg_reward:.3f}, steps={ep_len}")

        return {
            'baseline_name': self.baseline_name,
            'num_agents': self.num_agents,
            'rewards': episode_rewards,
            'episode_lengths': episode_lengths,
            'mean_reward': float(np.mean(episode_rewards)),
            'std_reward': float(np.std(episode_rewards)),
            'min_reward': float(np.min(episode_rewards)),
            'max_reward': float(np.max(episode_rewards)),
            'num_episodes': n_ep,
        }

    def save_results(self, results: Dict, save_dir: str):
        """
        Save baseline results to disk as JSON.

        Args:
            results:  Output from run()
            save_dir: Directory to save into
        """
        os.makedirs(save_dir, exist_ok=True)
        fname = os.path.join(
            save_dir,
            f"{self.baseline_name}_{self.num_agents}agents.json"
        )
        with open(fname, 'w') as f:
            json.dump(results, f, indent=2)
        print(f"  Baseline results saved: {fname}")

    @staticmethod
    def load_results(path: str) -> Dict:
        """
        Load baseline results from disk.

        Args:
            path: Path to JSON file

        Returns:
            dict: Loaded results
        """
        with open(path) as f:
            return json.load(f)


class AllBaselinesRunner:
    """
    Convenience class to run all rule-based baselines in one call.

    Args:
        env:        UAV swarm environment
        num_agents: Number of UAVs
        config:     Configuration dict
    """

    RULE_BASED_BASELINES = ['greedy_offload', 'round_robin', 'local_only']

    def __init__(self, env, num_agents: int, config: Optional[Dict] = None):
        self.env = env
        self.num_agents = num_agents
        self.config = config or {'num_eval_episodes': 50}

    def run_all(self, save_dir: Optional[str] = None) -> Dict:
        """
        Run all rule-based baselines and optionally save results.

        Args:
            save_dir: If provided, save each baseline result to disk

        Returns:
            dict: {baseline_name: results_dict}
        """
        all_results = {}
        for baseline_name in self.RULE_BASED_BASELINES:
            print(f"  Running baseline: {baseline_name}")
            adapter = BaselineAdapter(
                self.env, baseline_name, self.num_agents, self.config
            )
            results = adapter.run()
            all_results[baseline_name] = results
            print(f"    mean_reward={results['mean_reward']:.3f}, "
                  f"std={results['std_reward']:.3f}")

            if save_dir:
                adapter.save_results(results, save_dir)

        return all_results


if __name__ == '__main__':
    """Smoke test for baseline adapters — uses mock environment."""

    print("Testing BaselineAdapter...\n")

    class MockEnv:
        """Mock environment for testing."""
        def __init__(self, n=4):
            self.n = n
            self.dt = 0.01
            self._steps = 0

        def reset(self):
            self._steps = 0
            return {i: np.random.randn(40).astype(np.float32)
                    for i in range(self.n)}, {}

        def step(self, actions):
            self._steps += 1
            obs = {i: np.random.randn(40).astype(np.float32) for i in range(self.n)}
            rewards = {i: float(np.random.rand()) for i in range(self.n)}
            done = self._steps >= 20
            return obs, rewards, done, False, {}

    import tempfile

    num_agents = 4
    env = MockEnv(num_agents)

    print("Test 1: GreedyOffload adapter")
    adapter = BaselineAdapter(env, 'greedy_offload', num_agents,
                               config={'num_eval_episodes': 5})
    results = adapter.run()
    assert len(results['rewards']) == 5
    assert 'mean_reward' in results
    print(f"  mean_reward={results['mean_reward']:.3f} \u2713")

    print("\nTest 2: RoundRobin adapter")
    adapter2 = BaselineAdapter(env, 'round_robin', num_agents,
                                config={'num_eval_episodes': 5})
    r2 = adapter2.run()
    assert 'mean_reward' in r2
    print(f"  mean_reward={r2['mean_reward']:.3f} \u2713")

    print("\nTest 3: Save and load results")
    with tempfile.TemporaryDirectory() as tmpdir:
        adapter.save_results(results, tmpdir)
        fname = os.path.join(tmpdir,
                             f"greedy_offload_{num_agents}agents.json")
        assert os.path.exists(fname)
        loaded = BaselineAdapter.load_results(fname)
        assert loaded['baseline_name'] == 'greedy_offload'
        print(f"  Saved and loaded successfully \u2713")

    print("\nTest 4: AllBaselinesRunner")
    runner = AllBaselinesRunner(env, num_agents,
                                 config={'num_eval_episodes': 3})
    all_res = runner.run_all()
    assert 'greedy_offload' in all_res
    assert 'round_robin' in all_res
    assert 'local_only' in all_res
    print("  All baselines ran \u2713")

    print("\n\u2705 BaselineAdapter tests passed!")
