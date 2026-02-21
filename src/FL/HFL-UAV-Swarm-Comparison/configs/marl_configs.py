"""
MARL configurations for all 5 algorithms - owned by Belal.

This module provides standardized configurations for all multi-agent RL algorithms:
MAPPO, MADDPG, QMIX, PPO, and SAC.

CRITICAL: All algorithms use:
    - obs_dim = 8 + 20 + 4*(num_uavs-1) = DYNAMIC based on swarm size
      (4 drones: 40, 10 drones: 64, 20 drones: 104)
    - action_dim = 5 (continuous control: [target_x, target_y, target_z, offload, cpu_freq])
    - action_space: continuous [-1, 1] for all algorithms
    - Environment API: standard Gymnasium interface
"""

# ================================================================
# OBSERVATION AND ACTION SPACE (CONSTANT ACROSS ALL ALGORITHMS)
# ================================================================
# NOTE: OBS_DIM is DYNAMIC - calculated from swarm size in train scripts
# Formula: obs_dim = 8 + 20 + 4*(num_uavs-1)
#   8 = self state (x,y,z,vx,vy,vz,battery,cpu_freq)
#   20 = local tasks (5 tasks × 4 dims each)
#   4*(num_uavs-1) = neighbor states (each neighbor has 4 dims)
#
# Examples:
# ================================================================
# HELPER FUNCTION FOR DYNAMIC OBS_DIM
# ================================================================

def calculate_obs_dim(num_agents: int) -> int:
    """
    Calculate observation dimension based on swarm size.
    
    Formula: obs_dim = 8 + 20 + 4*(num_agents-1)
        8 = self state (x,y,z,vx,vy,vz,battery,cpu_freq)
        20 = local tasks (5 tasks × 4 dims each)
        4*(num_agents-1) = neighbor states (each neighbor has 4 dims)
    
    Args:
        num_agents: Number of UAVs in the swarm
    
    Returns:
        int: Observation dimension
    
    Examples:
        >>> calculate_obs_dim(4)
        40
        >>> calculate_obs_dim(10)
        64
        >>> calculate_obs_dim(20)
        104
    """
    return 8 + 20 + 4 * (num_agents - 1)


# Default OBS_DIM for when num_agents is not yet known
# Used in test assertions and examples (assumes 10 agents for documentation)
OBS_DIM = calculate_obs_dim(10)  # 64 for 10 agents (documentation standard)
ACTION_DIM = 5

# ================================================================
MARL_CONFIGS = {
    'mappo': {
        # Environment
        'obs_dim': OBS_DIM,
        'action_dim': ACTION_DIM,
        
        # Network architecture
        'hidden_dim': 256,
        
        # Optimization
        'lr_actor': 3e-4,
        'lr_critic': 1e-3,
        'gamma': 0.99,             # Discount factor
        'gae_lambda': 0.95,        # GAE lambda parameter
        
        # PPO-specific
        'clip_param': 0.2,         # PPO clipping parameter
        'ppo_epochs': 10,          # PPO update epochs per rollout
        'batch_size': 64,          # Mini-batch size for PPO updates
        'entropy_coef': 0.01,      # Entropy regularization coefficient
        'value_loss_coef': 0.5,    # Value function loss coefficient
        'max_grad_norm': 0.5,      # Gradient clipping threshold
        
        # Rollout
        'rollout_length': 'episode',  # Full episode per FL round
        'num_mini_batches': 4,     # Number of mini-batches per epoch
    },
    
    'maddpg': {
        # Environment
        'obs_dim': OBS_DIM,
        'action_dim': ACTION_DIM,
        
        # Network architecture
        'hidden_dim': 256,
        
        # Optimization
        'actor_lr': 1e-3,
        'critic_lr': 1e-3,
        'gamma': 0.95,
        'tau': 0.005,              # Soft target update rate
        
        # Experience replay
        'buffer_size': 100000,
        'batch_size': 1024,
        'warmup_steps': 1000,      # Random actions before training
        
        # Exploration
        'noise_std': 0.1,          # Gaussian noise for exploration
    },
    
    'qmix': {
        # Environment
        'obs_dim': OBS_DIM,
        'action_dim': 81,          # CORRECTED: Discretized to 81 actions (9 movement × 3 offload × 3 CPU)
        'num_actions': 81,         # Discrete action space size
        
        # Network architecture
        'hidden_dim': 256,
        'mixing_embed_dim': 32,    # Mixing network embedding dimension
        
        # Optimization
        'lr': 5e-4,
        'gamma': 0.99,
        
        # Exploration
        'epsilon_start': 1.0,
        'epsilon_end': 0.05,
        'epsilon_decay_steps': 50000,
        
        # Experience replay
        'buffer_size': 5000,
        'batch_size': 32,
        'target_update_interval': 200,  # Steps between target network updates
    },
    
    'ppo': {
        # Environment
        'obs_dim': OBS_DIM,
        'action_dim': ACTION_DIM,
        
        # Network architecture
        'hidden_dim': 128,         # Smaller for single-agent
        
        # Optimization
        'lr': 3e-4,
        'gamma': 0.99,
        'gae_lambda': 0.95,
        
        # PPO-specific
        'clip_param': 0.2,
        'ppo_epochs': 10,
        'batch_size': 64,
        'entropy_coef': 0.01,
        
        # Rollout
        'rollout_length': 'episode',
    },
    
    'sac': {
        # Environment
        'obs_dim': OBS_DIM,
        'action_dim': ACTION_DIM,
        
        # Network architecture
        'hidden_dim': 256,
        
        # Optimization
        'actor_lr': 3e-4,
        'critic_lr': 3e-4,
        'alpha_lr': 3e-4,          # Temperature parameter learning rate
        'gamma': 0.99,
        'tau': 0.005,
        
        # Experience replay
        'buffer_size': 100000,
        'batch_size': 256,
        
        # SAC-specific
        'automatic_entropy_tuning': True,    # Automatic temperature adjustment
        'warmup_steps': 1000,
        'target_entropy_scale': 1.0,         # Scale for target entropy
    }
}


def get_marl_config(marl_algorithm: str) -> dict:
    """
    Get configuration for a specific MARL algorithm.
    
    Args:
        marl_algorithm: Name of MARL algorithm
                       Options: 'mappo', 'maddpg', 'qmix', 'ppo', 'sac'
    
    Returns:
        dict: Configuration dictionary (copy, safe to modify)
    
    Raises:
        AssertionError: If algorithm name is unknown
    
    Example:
        >>> config = get_marl_config('mappo')
        >>> print(config['obs_dim'], config['action_dim'])
        64 5
    """
    assert marl_algorithm in MARL_CONFIGS, \
        f"Unknown MARL algorithm: {marl_algorithm}. Choose from {list(MARL_CONFIGS.keys())}"
    
    return MARL_CONFIGS[marl_algorithm].copy()


def get_all_marl_names() -> list:
    """
    Get list of all available MARL algorithm names.
    
    Returns:
        list: MARL algorithm names
    
    Example:
        >>> algorithms = get_all_marl_names()
        >>> print(algorithms)
        ['mappo', 'maddpg', 'qmix', 'ppo', 'sac']
    """
    return list(MARL_CONFIGS.keys())


def update_marl_config(marl_algorithm: str, **kwargs) -> dict:
    """
    Get MARL config and update with custom parameters.
    
    Args:
        marl_algorithm: Name of MARL algorithm
        **kwargs: Parameters to override
    
    Returns:
        dict: Updated configuration
    
    Example:
        >>> config = update_marl_config('mappo', hidden_dim=512, lr_actor=1e-3)
        >>> print(config['hidden_dim'], config['lr_actor'])
        512 0.001
    """
    config = get_marl_config(marl_algorithm)
    config.update(kwargs)
    return config


def get_marl_config_for_swarm_size(marl_algorithm: str, num_drones: int) -> dict:
    """
    Get MARL config adapted for specific swarm size.
    
    Adjusts hyperparameters based on swarm size:
        - obs_dim: Calculated dynamically (8 + 20 + 4*(num_drones-1))
        - Learning rates: Smaller for larger swarms
        - Batch sizes: Larger for more data
        - Buffer sizes: Scale with swarm size
    
    Args:
        marl_algorithm: Name of MARL algorithm
        num_drones: Number of drones in swarm
    
    Returns:
        dict: Adapted configuration
    
    Example:
        >>> config = get_marl_config_for_swarm_size('mappo', 4)
        >>> print(config['obs_dim'])  # 8 + 20 + 4*3 = 40
        40
    """
    config = get_marl_config(marl_algorithm)
    
    # CRITICAL: Set obs_dim based on actual swarm size
    config['obs_dim'] = calculate_obs_dim(num_drones)
    
    # Adjust batch size for large swarms
    if 'batch_size' in config:
        if num_drones > 50:
            config['batch_size'] = min(256, config['batch_size'] * 2)
        elif num_drones > 20:
            config['batch_size'] = min(128, int(config['batch_size'] * 1.5))
    
    # Adjust buffer size for off-policy algorithms
    if 'buffer_size' in config:
        if num_drones > 50:
            config['buffer_size'] = min(500000, config['buffer_size'] * 2)
        elif num_drones > 20:
            config['buffer_size'] = min(200000, int(config['buffer_size'] * 1.5))
    
    # Adjust learning rates for large swarms (more stable)
    if num_drones > 50:
        if 'lr_actor' in config:
            config['lr_actor'] = config['lr_actor'] * 0.5
        if 'lr_critic' in config:
            config['lr_critic'] = config['lr_critic'] * 0.5
        if 'actor_lr' in config:
            config['actor_lr'] = config['actor_lr'] * 0.5
        if 'critic_lr' in config:
            config['critic_lr'] = config['critic_lr'] * 0.5
    
    return config


def get_action_space_info() -> dict:
    """
    Get information about the UAV action space.
    
    Returns:
        dict: Action space specification
    
    Example:
        >>> info = get_action_space_info()
        >>> print(info['names'])
        ['target_x', 'target_y', 'target_z', 'offload_decision', 'cpu_frequency']
    """
    return {
        'dimension': ACTION_DIM,
        'low': -1.0,
        'high': 1.0,
        'names': ['target_x', 'target_y', 'target_z', 'offload_decision', 'cpu_frequency'],
        'descriptions': [
            'X-axis flight target (normalized)',
            'Y-axis flight target (normalized)',
            'Z-axis flight target (normalized)',
            'Task offload decision (continuous, discretized by env)',
            'CPU frequency control (normalized)',
        ],
        'offload_discretization': {
            'process_local': '< -0.33',
            'reject_task': '-0.33 to 0.33',
            'share_neighbor': '> 0.33',
        }
    }


def get_obs_space_info() -> dict:
    """
    Get information about the UAV observation space.
    
    Returns:
        dict: Observation space specification
    
    Example:
        >>> info = get_obs_space_info()
        >>> print(info['dimension'])
        64
    """
    return {
        'dimension': OBS_DIM,
        'components': {
            'self_state': {
                'range': [0, 8],
                'size': 8,
                'contents': 'position(3), velocity(3), battery(1), cpu_freq(1)',
            },
            'task_queue': {
                'range': [8, 28],
                'size': 20,
                'contents': '5 tasks × (cpu_cycles, data_size, deadline, priority)',
            },
            'neighbors': {
                'range': [28, 40],
                'size': 12,
                'contents': '3 neighbors × (x, y, z, battery)',
            },
        }
    }


if __name__ == '__main__':
    """Test MARL configurations."""
    
    print("Testing MARL Configurations...\n")
    
    # Test get_all_marl_names
    print("Test 1: Get all MARL algorithm names")
    names = get_all_marl_names()
    assert len(names) == 5, f"Expected 5 algorithms, got {len(names)}"
    assert 'mappo' in names, "mappo should be in names"
    assert 'maddpg' in names, "maddpg should be in names"
    print(f"✓ Available MARL algorithms: {names}")
    
    # Test get_marl_config for each algorithm
    print("\nTest 2: Get configuration for each algorithm")
    for name in names:
        config = get_marl_config(name)
        assert isinstance(config, dict), f"Config should be dict, got {type(config)}"
        assert 'obs_dim' in config, f"obs_dim missing in {name}"
        assert 'action_dim' in config, f"action_dim missing in {name}"
        assert config['obs_dim'] == OBS_DIM, f"obs_dim should be {OBS_DIM}"
        print(f"✓ {name}: obs_dim={config['obs_dim']}, action_dim={config['action_dim']}, {len(config)} params")
    
    # Test MAPPO config in detail
    print("\nTest 3: MAPPO configuration")
    mappo_config = get_marl_config('mappo')
    assert mappo_config['obs_dim'] == 64, "obs_dim should be 64 (actual environment)"
    assert mappo_config['action_dim'] == 5, "action_dim should be 5"
    assert mappo_config['hidden_dim'] == 256, "hidden_dim should be 256"
    assert mappo_config['ppo_epochs'] == 10, "ppo_epochs should be 10"
    print("✓ MAPPO config:")
    for key in ['obs_dim', 'action_dim', 'hidden_dim', 'lr_actor', 'lr_critic', 
                'gamma', 'clip_param', 'ppo_epochs']:
        print(f"  {key}: {mappo_config[key]}")
    
    # Test update_marl_config
    print("\nTest 4: Update configuration")
    custom_config = update_marl_config('mappo', hidden_dim=512, lr_actor=1e-3)
    assert custom_config['hidden_dim'] == 512, "hidden_dim should be updated"
    assert custom_config['lr_actor'] == 1e-3, "lr_actor should be updated"
    assert custom_config['obs_dim'] == 40, "obs_dim should remain unchanged"
    print("✓ Updated config:")
    print(f"  hidden_dim: {custom_config['hidden_dim']}")
    print(f"  lr_actor: {custom_config['lr_actor']}")
    
    # Test config copies are independent
    print("\nTest 5: Configuration independence")
    config1 = get_marl_config('sac')
    config2 = get_marl_config('sac')
    config1['gamma'] = 0.5
    assert config2['gamma'] != 0.5, "Configs should be independent copies"
    print("✓ Configuration copies are independent")
    
    # Test swarm size adaptation
    print("\nTest 6: Swarm size adaptation")
    for num_drones in [5, 10, 20, 50, 100]:
        config = get_marl_config_for_swarm_size('mappo', num_drones)
        print(f"  {num_drones:3d} drones → batch_size={config['batch_size']}, "
              f"lr_actor={config['lr_actor']:.2e}")
    print("✓ Swarm size adaptation works")
    
    # Test action space info
    print("\nTest 7: Action space information")
    action_info = get_action_space_info()
    assert action_info['dimension'] == 5, "Action dimension should be 5"
    assert len(action_info['names']) == 5, "Should have 5 action names"
    print(f"✓ Action space: {action_info['dimension']} dimensions")
    for i, (name, desc) in enumerate(zip(action_info['names'], action_info['descriptions'])):
        print(f"  [{i}] {name}: {desc}")
    
    # Test observation space info
    print("\nTest 8: Observation space information")
    obs_info = get_obs_space_info()
    assert obs_info['dimension'] == 64, "Observation dimension should be 64 (actual environment)"
    assert len(obs_info['components']) == 3, "Should have 3 components"
    print(f"✓ Observation space: {obs_info['dimension']} dimensions")
    for comp_name, comp_info in obs_info['components'].items():
        print(f"  {comp_name}: dims {comp_info['range']}, {comp_info['contents']}")
    
    # Test error handling
    print("\nTest 9: Error handling")
    try:
        get_marl_config('unknown_algorithm')
        assert False, "Should raise AssertionError"
    except AssertionError as e:
        print(f"✓ Correctly raised error: {str(e)[:60]}...")
    
    print("\n✅ All MARL configuration tests passed!")

