"""
Literature baseline configurations — owned by Ammar.

Defines the reference baselines used to benchmark our FL-MARL combinations
against state-of-the-art approaches reported in the literature.

Usage:
    from configs.baseline_configs import get_baseline_config, get_all_baseline_names
    cfg = get_baseline_config('greedy_offload')
"""

BASELINE_CONFIGS = {

    'greedy_offload': {
        'name': 'Greedy Offloading',
        'description': 'Greedy rule-based: always offload to highest-battery UAV.',
        'type': 'rule_based',
        'paper': 'N/A — handcrafted baseline',
        'num_eval_episodes': 100,
        'deterministic': True,
    },

    'round_robin': {
        'name': 'Round-Robin Offloading',
        'description': 'Cyclic task assignment across all UAVs regardless of state.',
        'type': 'rule_based',
        'paper': 'N/A — handcrafted baseline',
        'num_eval_episodes': 100,
        'deterministic': True,
    },

    'local_only': {
        'name': 'Local-Only Processing',
        'description': 'No offloading; each UAV processes all tasks locally.',
        'type': 'rule_based',
        'paper': 'N/A — handcrafted baseline',
        'num_eval_episodes': 100,
        'deterministic': True,
    },

    'independent_ppo': {
        'name': 'Independent PPO (no FL)',
        'description': 'Each UAV trains PPO independently without model sharing.',
        'type': 'marl_no_fl',
        'paper': 'Schulman et al., 2017 — PPO',
        'hidden_dim': 256,
        'lr': 3e-4,
        'gamma': 0.99,
        'gae_lambda': 0.95,
        'clip_param': 0.2,
        'ppo_epochs': 10,
        'batch_size': 64,
        'entropy_coef': 0.01,
        'rollout_length': 'episode',
        'num_eval_episodes': 50,
    },

    'fedavg_ippo': {
        'name': 'FedAvg + Independent PPO',
        'description': 'Standard FedAvg aggregating independent PPO policies.',
        'type': 'fl_baseline',
        'paper': 'McMahan et al., 2017 — Communication-Efficient Learning',
        'hidden_dim': 256,
        'lr': 3e-4,
        'gamma': 0.99,
        'clip_param': 0.2,
        'ppo_epochs': 10,
        'batch_size': 64,
        'num_fl_rounds': 100,
        'local_epochs': 5,
        'client_fraction': 1.0,
        'num_eval_episodes': 50,
    },

    'maddpg_no_fl': {
        'name': 'MADDPG (no FL)',
        'description': 'Vanilla MADDPG without federated learning.',
        'type': 'marl_no_fl',
        'paper': 'Lowe et al., 2017 — MADDPG',
        'hidden_dim': 256,
        'actor_lr': 1e-3,
        'critic_lr': 1e-3,
        'gamma': 0.95,
        'tau': 0.005,
        'buffer_size': 100000,
        'batch_size': 1024,
        'warmup_steps': 1000,
        'noise_std': 0.1,
        'num_eval_episodes': 50,
    },

    'd2d_fl': {
        'name': 'D2D Federated Learning',
        'description': 'Device-to-device FL over proximity graph without hierarchy.',
        'type': 'fl_baseline',
        'paper': 'Simulated from D2D-FL literature',
        'communication_range': 50.0,
        'num_fl_rounds': 100,
        'local_epochs': 5,
        'aggregation': 'proximity_average',
        'min_neighbors': 1,
        'num_eval_episodes': 50,
    },

    'ctde_upper_bound': {
        'name': 'CTDE Upper Bound',
        'description': 'Fully centralised training; decentralised execution. Oracle upper bound.',
        'type': 'upper_bound',
        'paper': 'N/A — oracle baseline',
        'hidden_dim': 256,
        'lr': 3e-4,
        'gamma': 0.99,
        'batch_size': 256,
        'num_eval_episodes': 50,
    },
}


def get_baseline_config(baseline_name: str) -> dict:
    """
    Get configuration for a specific baseline.

    Args:
        baseline_name: Name of the baseline method.

    Returns:
        dict: Configuration dictionary (copy, safe to modify)

    Raises:
        AssertionError: If baseline name is unknown

    Example:
        >>> cfg = get_baseline_config('greedy_offload')
        >>> print(cfg['type'])
        rule_based
    """
    assert baseline_name in BASELINE_CONFIGS, (
        f"Unknown baseline: '{baseline_name}'. "
        f"Choose from {list(BASELINE_CONFIGS.keys())}"
    )
    return BASELINE_CONFIGS[baseline_name].copy()


def get_all_baseline_names() -> list:
    """
    Get list of all available baseline names.

    Returns:
        list: Baseline names

    Example:
        >>> names = get_all_baseline_names()
        >>> print(names[0])
        greedy_offload
    """
    return list(BASELINE_CONFIGS.keys())


def get_baselines_by_type(baseline_type: str) -> dict:
    """
    Get all baselines of a given type.

    Args:
        baseline_type: One of 'rule_based', 'marl_no_fl', 'fl_baseline', 'upper_bound'

    Returns:
        dict: {name: config} for matching baselines
    """
    return {
        name: cfg.copy()
        for name, cfg in BASELINE_CONFIGS.items()
        if cfg.get('type') == baseline_type
    }


def get_rule_based_baselines() -> dict:
    """Return all rule-based (non-learning) baselines."""
    return get_baselines_by_type('rule_based')


def get_fl_baselines() -> dict:
    """Return baselines that use federated learning."""
    return get_baselines_by_type('fl_baseline')


def get_marl_baselines() -> dict:
    """Return MARL baselines without FL."""
    return get_baselines_by_type('marl_no_fl')


if __name__ == '__main__':
    """Test baseline configurations."""

    print("Testing Baseline Configurations...\n")

    names = get_all_baseline_names()
    assert len(names) >= 5
    print(f"\u2713 {len(names)} baselines registered: {names}")

    for name in names:
        cfg = get_baseline_config(name)
        assert 'name' in cfg and 'type' in cfg
        print(f"  {name} ({cfg['type']}): {len(cfg)} keys")

    rule = get_rule_based_baselines()
    fl = get_fl_baselines()
    assert len(rule) >= 3
    print(f"\u2713 rule_based: {list(rule.keys())}")
    print(f"\u2713 fl_baseline: {list(fl.keys())}")

    cfg1 = get_baseline_config('greedy_offload')
    cfg2 = get_baseline_config('greedy_offload')
    cfg1['num_eval_episodes'] = 9999
    assert cfg2['num_eval_episodes'] != 9999
    print("\u2713 Config copies are independent")

    try:
        get_baseline_config('nonexistent')
        assert False
    except AssertionError as e:
        print(f"\u2713 Error handling works: {str(e)[:60]}...")

    print("\n\u2705 All baseline config tests passed!")
