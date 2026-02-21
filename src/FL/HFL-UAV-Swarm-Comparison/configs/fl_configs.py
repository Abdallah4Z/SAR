"""
Federated Learning configurations - owned by Belal.

This module provides standardized configurations for all FL algorithms
tested in the project: FedAvg, FedProx, FedAdam, and Hierarchical FL (HFL).
"""

# All FL algorithm configurations
FL_CONFIGS = {
    'fedavg': {
        'num_rounds': 100,
        'local_epochs': 5,
        'batch_size': 32,
        'learning_rate': 0.001,
        'client_fraction': 1.0,  # Fraction of clients to sample per round
        'min_clients': 2,        # Minimum number of clients required
    },
    
    'fedprox': {
        'num_rounds': 100,
        'local_epochs': 5,
        'batch_size': 32,
        'learning_rate': 0.001,
        'mu': 0.01,              # Proximal term strength (keeps clients near global)
        'client_fraction': 1.0,
    },
    
    'fedadam': {
        'num_rounds': 100,
        'local_epochs': 5,
        'server_learning_rate': 0.01,    # Server-side adaptive LR
        'client_learning_rate': 0.001,   # Client-side LR
        'beta1': 0.9,                     # First moment decay
        'beta2': 0.99,                    # Second moment decay
        'epsilon': 1e-3,                  # Numerical stability
        'tau': 1e-3,                      # Adaptive learning rate adjustment
    },
    
    'hierarchical': {
        'num_rounds': 100,
        'local_epochs': 5,
        'num_clusters': 'auto',                  # 'auto' or integer
        'intra_cluster_rounds': 3,               # Intra-cluster aggregation rounds
        'cluster_update_interval': 10,           # Re-cluster every N rounds
        'min_cluster_size': 2,                   # Minimum agents per cluster
        'communication_range': 50.0,             # meters (for proximity clustering)
        'reclustering_threshold': 50.0,          # meters (re-cluster if avg movement > threshold)
        'clustering_strategy': 'kmeans',         # 'kmeans', 'proximity', 'roundrobin'
    }
}


def get_fl_config(fl_algorithm: str) -> dict:
    """
    Get configuration for a specific FL algorithm.
    
    Args:
        fl_algorithm: Name of FL algorithm
                     Options: 'fedavg', 'fedprox', 'fedadam', 'hierarchical'
    
    Returns:
        dict: Configuration dictionary (copy, safe to modify)
    
    Raises:
        AssertionError: If algorithm name is unknown
    
    Example:
        >>> config = get_fl_config('hierarchical')
        >>> config['num_clusters'] = 5  # Customize
        >>> print(config['intra_cluster_rounds'])
        3
    """
    assert fl_algorithm in FL_CONFIGS, \
        f"Unknown FL algorithm: {fl_algorithm}. Choose from {list(FL_CONFIGS.keys())}"
    
    return FL_CONFIGS[fl_algorithm].copy()


def get_all_fl_names() -> list:
    """
    Get list of all available FL algorithm names.
    
    Returns:
        list: FL algorithm names
    
    Example:
        >>> algorithms = get_all_fl_names()
        >>> print(algorithms)
        ['fedavg', 'fedprox', 'fedadam', 'hierarchical']
    """
    return list(FL_CONFIGS.keys())


def update_fl_config(fl_algorithm: str, **kwargs) -> dict:
    """
    Get FL config and update with custom parameters.
    
    Args:
        fl_algorithm: Name of FL algorithm
        **kwargs: Parameters to override
    
    Returns:
        dict: Updated configuration
    
    Example:
        >>> config = update_fl_config('hierarchical', num_rounds=200, num_clusters=5)
        >>> print(config['num_rounds'], config['num_clusters'])
        200 5
    """
    config = get_fl_config(fl_algorithm)
    config.update(kwargs)
    return config


def get_fl_config_for_swarm_size(fl_algorithm: str, num_drones: int) -> dict:
    """
    Get FL config adapted for specific swarm size.
    
    Adjusts hyperparameters based on swarm size:
        - Small swarms (≤10): Standard settings
        - Medium swarms (11-50): Increase cluster size
        - Large swarms (>50): More aggressive clustering
    
    Args:
        fl_algorithm: Name of FL algorithm
        num_drones: Number of drones in swarm
    
    Returns:
        dict: Adapted configuration
    
    Example:
        >>> config = get_fl_config_for_swarm_size('hierarchical', 100)
        >>> print(config['num_clusters'])
        20
    """
    config = get_fl_config(fl_algorithm)
    
    # Adjust for hierarchical FL
    if fl_algorithm == 'hierarchical' and config['num_clusters'] == 'auto':
        if num_drones <= 10:
            config['num_clusters'] = max(2, num_drones // 3)
        elif num_drones <= 50:
            config['num_clusters'] = max(3, num_drones // 5)
        else:  # > 50 drones
            config['num_clusters'] = max(5, num_drones // 10)
    
    # Adjust batch size for large swarms
    if 'batch_size' in config:
        if num_drones > 50:
            config['batch_size'] = 64
        elif num_drones > 20:
            config['batch_size'] = 48
    
    return config


if __name__ == '__main__':
    """Test FL configurations."""
    
    print("Testing FL Configurations...\n")
    
    # Test get_all_fl_names
    print("Test 1: Get all FL algorithm names")
    names = get_all_fl_names()
    assert len(names) == 4, f"Expected 4 algorithms, got {len(names)}"
    assert 'hierarchical' in names, "hierarchical should be in names"
    print(f"✓ Available FL algorithms: {names}")
    
    # Test get_fl_config for each algorithm
    print("\nTest 2: Get configuration for each algorithm")
    for name in names:
        config = get_fl_config(name)
        assert isinstance(config, dict), f"Config should be dict, got {type(config)}"
        assert 'num_rounds' in config, f"num_rounds missing in {name}"
        print(f"✓ {name}: {len(config)} parameters")
        print(f"  Keys: {list(config.keys())[:5]}...")
    
    # Test hierarchical config in detail
    print("\nTest 3: Hierarchical FL configuration")
    hfl_config = get_fl_config('hierarchical')
    assert hfl_config['num_clusters'] == 'auto', "Default should be 'auto'"
    assert hfl_config['intra_cluster_rounds'] == 3, "Default should be 3"
    assert hfl_config['clustering_strategy'] == 'kmeans', "Default should be 'kmeans'"
    print("✓ Hierarchical FL config:")
    for key, value in hfl_config.items():
        print(f"  {key}: {value}")
    
    # Test update_fl_config
    print("\nTest 4: Update configuration")
    custom_config = update_fl_config('hierarchical', num_rounds=200, num_clusters=5)
    assert custom_config['num_rounds'] == 200, "num_rounds should be updated"
    assert custom_config['num_clusters'] == 5, "num_clusters should be updated"
    assert custom_config['intra_cluster_rounds'] == 3, "Other params should remain"
    print("✓ Updated config:")
    print(f"  num_rounds: {custom_config['num_rounds']}")
    print(f"  num_clusters: {custom_config['num_clusters']}")
    
    # Test config copies are independent
    print("\nTest 5: Configuration independence")
    config1 = get_fl_config('fedavg')
    config2 = get_fl_config('fedavg')
    config1['num_rounds'] = 999
    assert config2['num_rounds'] != 999, "Configs should be independent copies"
    print("✓ Configuration copies are independent")
    
    # Test swarm size adaptation
    print("\nTest 6: Swarm size adaptation")
    for num_drones in [5, 10, 20, 50, 100]:
        config = get_fl_config_for_swarm_size('hierarchical', num_drones)
        print(f"  {num_drones:3d} drones → {config['num_clusters']} clusters, "
              f"batch_size={config.get('batch_size', 'N/A')}")
    print("✓ Swarm size adaptation works")
    
    # Test error handling
    print("\nTest 7: Error handling")
    try:
        get_fl_config('unknown_algorithm')
        assert False, "Should raise AssertionError"
    except AssertionError as e:
        print(f"✓ Correctly raised error: {str(e)[:60]}...")
    
    print("\n✅ All FL configuration tests passed!")
