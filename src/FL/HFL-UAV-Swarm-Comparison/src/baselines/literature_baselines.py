"""
LiteratureBaselines — Rule-based and simple-learning baseline agents.

Implements the non-FL and non-MARL baselines from BASELINE_CONFIGS for fair
comparison against our FL-MARL combinations in the UAV swarm environment.

Each baseline is a lightweight policy that can be run in the standard
Gymnasium evaluation loop (same interface as MARL agents).

Usage:
    from src.baselines.literature_baselines import (
        GreedyOffloadBaseline, RoundRobinBaseline, LocalOnlyBaseline
    )

    baseline = GreedyOffloadBaseline(num_agents=10)
    evaluator = Evaluator(env, baseline.as_agent_list())
    results = evaluator.evaluate()
"""

import numpy as np
from typing import List, Optional, Dict


# ======================================================================
# Base class for all baselines
# ======================================================================

class BaselineAgent:
    """
    Lightweight baseline agent compatible with the evaluation API.

    Implements the minimum interface required by Evaluator:
        select_action(obs, deterministic) -> action
        set_eval_mode()
        set_train_mode()

    Args:
        agent_id:   UAV ID
        num_agents: Total number of UAVs (for coordination)
    """

    def __init__(self, agent_id: int, num_agents: int):
        self.agent_id = agent_id
        self.num_agents = num_agents
        self.training = False

    def select_action(self, obs: np.ndarray, deterministic: bool = True) -> np.ndarray:
        """Return a default 'do nothing' action."""
        return np.zeros(5, dtype=np.float32)

    def set_eval_mode(self):
        self.training = False

    def set_train_mode(self):
        self.training = True

    def get_model_weights(self) -> dict:
        """No weights for rule-based baselines."""
        return {}

    def set_model_weights(self, weights: dict):
        """No-op for rule-based baselines."""
        pass


# ======================================================================
# Greedy Offloading
# ======================================================================

class GreedyOffloadAgent(BaselineAgent):
    """
    Greedy rule: offload tasks to the UAV with the most battery.

    Observation layout (from marl_configs.py):
        [0:3]   own position (x, y, z)
        [3:6]   own velocity
        [6]     own battery %
        [7]     own cpu_freq
        [8:28]  local tasks (5 tasks x 4 dims)
        [28:]   neighbour states (x, y, z, battery) per neighbour

    Action:
        [0:3]   target position (stay in place: all zeros)
        [3]     offload decision: >0.33 = share with neighbour
        [4]     cpu_frequency: 0 (use default)
    """

    def select_action(self, obs: np.ndarray, deterministic: bool = True) -> np.ndarray:
        action = np.zeros(5, dtype=np.float32)
        own_battery = float(obs[6])

        # Extract neighbour batteries from obs[28:]
        n_neighbours = self.num_agents - 1
        neighbour_batteries = []
        for i in range(n_neighbours):
            base = 28 + i * 4
            if base + 3 < len(obs):
                neighbour_batteries.append(float(obs[base + 3]))  # battery is 4th dim

        # If a neighbour has higher battery than us, offload to them
        if neighbour_batteries and max(neighbour_batteries) > own_battery:
            action[3] = 0.8   # share with neighbour (> 0.33 threshold)
        else:
            action[3] = -0.8  # process locally (< -0.33 threshold)

        return action


class GreedyOffloadBaseline:
    """Factory for creating a greedy offloading agent list."""

    def __init__(self, num_agents: int):
        self.num_agents = num_agents

    def as_agent_list(self) -> List[GreedyOffloadAgent]:
        return [GreedyOffloadAgent(i, self.num_agents)
                for i in range(self.num_agents)]


# ======================================================================
# Round-Robin Offloading
# ======================================================================

class RoundRobinAgent(BaselineAgent):
    """
    Cyclic assignment: UAV i offloads every n_agents-th task to itself,
    the rest it forwards to neighbours.

    Implemented as: offload if (step_count % num_agents) == agent_id
    Since agents don't share a step counter, we use episode step count
    tracked via a shared counter object.
    """

    def __init__(self, agent_id: int, num_agents: int, step_counter: dict):
        super().__init__(agent_id, num_agents)
        self._counter = step_counter  # Shared mutable dict {'step': 0}

    def select_action(self, obs: np.ndarray, deterministic: bool = True) -> np.ndarray:
        action = np.zeros(5, dtype=np.float32)
        step = self._counter.get('step', 0)

        if step % self.num_agents == self.agent_id:
            action[3] = -0.8   # Process locally
        else:
            action[3] = 0.8    # Share with neighbour

        # Increment shared counter (only agent 0 increments to avoid double-count)
        if self.agent_id == self.num_agents - 1:
            self._counter['step'] = step + 1

        return action


class RoundRobinBaseline:
    """Factory for creating a round-robin agent list."""

    def __init__(self, num_agents: int):
        self.num_agents = num_agents

    def as_agent_list(self) -> List[RoundRobinAgent]:
        counter = {'step': 0}
        return [RoundRobinAgent(i, self.num_agents, counter)
                for i in range(self.num_agents)]


# ======================================================================
# Local-Only Processing
# ======================================================================

class LocalOnlyAgent(BaselineAgent):
    """
    Never offloads: all tasks are processed locally.
    Represents the no-cooperation baseline.
    """

    def select_action(self, obs: np.ndarray, deterministic: bool = True) -> np.ndarray:
        action = np.zeros(5, dtype=np.float32)
        action[3] = -0.8   # Always process locally
        action[4] = 0.8    # Max CPU frequency
        return action


class LocalOnlyBaseline:
    """Factory for local-only agents."""

    def __init__(self, num_agents: int):
        self.num_agents = num_agents

    def as_agent_list(self) -> List[LocalOnlyAgent]:
        return [LocalOnlyAgent(i, self.num_agents)
                for i in range(self.num_agents)]


# ======================================================================
# Random Policy (useful sanity check)
# ======================================================================

class RandomAgent(BaselineAgent):
    """Uniformly random actions in [-1, 1] for all 5 dimensions."""

    def __init__(self, agent_id: int, num_agents: int, seed: int = 0):
        super().__init__(agent_id, num_agents)
        self._rng = np.random.default_rng(seed + agent_id)

    def select_action(self, obs: np.ndarray, deterministic: bool = True) -> np.ndarray:
        return self._rng.uniform(-1, 1, size=5).astype(np.float32)


# ======================================================================
# Convenience factory
# ======================================================================

def get_baseline_agents(
    baseline_name: str,
    num_agents: int
) -> List[BaselineAgent]:
    """
    Factory: get a list of baseline agents for a given baseline name.

    Args:
        baseline_name: Must match a BASELINE_CONFIGS key for a rule-based type
        num_agents:    Number of UAVs

    Returns:
        list: BaselineAgent instances (one per UAV)

    Raises:
        ValueError: If baseline_name is not a rule-based baseline
    """
    mapping = {
        'greedy_offload': lambda: GreedyOffloadBaseline(num_agents).as_agent_list(),
        'round_robin':    lambda: RoundRobinBaseline(num_agents).as_agent_list(),
        'local_only':     lambda: LocalOnlyBaseline(num_agents).as_agent_list(),
    }
    if baseline_name not in mapping:
        raise ValueError(
            f"'{baseline_name}' is not a rule-based baseline. "
            f"Available: {list(mapping.keys())}"
        )
    return mapping[baseline_name]()


if __name__ == '__main__':
    """Smoke test for literature baselines."""

    print("Testing literature baselines...\n")

    num_agents = 4
    obs = np.random.randn(40).astype(np.float32)
    obs[6] = 0.5    # own battery = 50%
    obs[31] = 0.8   # neighbour 0 battery = 80%

    # Test GreedyOffloadBaseline
    print("Test 1: GreedyOffloadBaseline")
    agents = GreedyOffloadBaseline(num_agents).as_agent_list()
    assert len(agents) == num_agents
    action = agents[0].select_action(obs)
    assert action.shape == (5,)
    print(f"  Agent 0 action[3] (offload): {action[3]:.2f}")
    assert action[3] > 0.3, "Should offload when neighbour has higher battery"
    print("  \u2713 Greedy offloading works")

    # Test RoundRobinBaseline
    print("\nTest 2: RoundRobinBaseline")
    agents = RoundRobinBaseline(num_agents).as_agent_list()
    for step in range(num_agents):
        actions = [a.select_action(obs) for a in agents]
        local_agents = [i for i, a in enumerate(actions) if a[3] < 0]
        print(f"  Step {step}: agents processing locally = {local_agents}")
    print("  \u2713 Round-robin works")

    # Test LocalOnlyBaseline
    print("\nTest 3: LocalOnlyBaseline")
    agents = LocalOnlyBaseline(num_agents).as_agent_list()
    for agent in agents:
        action = agent.select_action(obs)
        assert action[3] < -0.3, "Should always process locally"
    print("  \u2713 Local-only works")

    # Test factory
    print("\nTest 4: get_baseline_agents factory")
    for name in ['greedy_offload', 'round_robin', 'local_only']:
        agents = get_baseline_agents(name, num_agents)
        assert len(agents) == num_agents
        print(f"  {name}: {len(agents)} agents \u2713")

    print("\n\u2705 All literature baseline tests passed!")
