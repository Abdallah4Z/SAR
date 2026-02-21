"""
Base aggregator interface for federated learning algorithms.

This module defines the abstract base class that all FL algorithms
(FedAvg, FedProx, FedAdam, HFL) must inherit from, ensuring a consistent
API for the training pipeline.
"""

from abc import ABC, abstractmethod
from typing import List, Dict, Optional, Any
from collections.abc import Mapping
import numpy as np
import torch
import copy


class BaseAggregator(ABC):
    """
    Abstract base class for federated learning aggregation strategies.
    
    All FL algorithms must inherit from this class and implement:
        - aggregate(): Combine client models into global model
        - distribute(): Prepare global model for distribution to clients
    
    This design enables modular FL algorithm comparison:
        - Trainer accepts any BaseAggregator subclass
        - Easy to swap FL algorithms without changing training code
        - Consistent interface for evaluation and comparison
    
    Attributes:
        config (dict): Algorithm-specific configuration
        round_number (int): Current FL round
        history (list): Training history for logging
    """
    
    def __init__(self, config: Optional[Dict] = None):
        """
        Initialize federated aggregator.
        
        Args:
            config: Algorithm-specific configuration dictionary
        """
        self.config = config if config is not None else {}
        self.round_number = 0
        self.history = []
    
    @abstractmethod
    def aggregate(
        self,
        client_weights: List[Dict],
        client_sizes: Optional[List[int]] = None,
        **kwargs
    ) -> Dict:
        """
        Aggregate client model weights into a global model.
        
        This is the core FL operation where different algorithms diverge:
            - FedAvg: Weighted average based on data sizes
            - FedProx: Add proximal term to keep clients close to global
            - FedAdam: Use adaptive optimization (momentum + second moments)
            - HFL: Two-level aggregation (intra-cluster + inter-cluster)
        
        Args:
            client_weights: List of state_dicts from each client's actor network
            client_sizes: Optional list of local dataset sizes for weighted averaging
            **kwargs: Algorithm-specific additional arguments
        
        Returns:
            dict: Aggregated global model state_dict
        
        Example:
            >>> aggregator = FedAvgAggregator()
            >>> client_weights = [agent.get_model_weights() for agent in agents]
            >>> global_weights = aggregator.aggregate(client_weights)
        """
        pass
    
    @abstractmethod
    def distribute(self, global_weights: Dict) -> Dict:
        """
        Prepare global model for distribution to clients.
        
        Most algorithms simply return the global weights as-is,
        but some may apply post-processing or personalization.
        
        Args:
            global_weights: Aggregated global model state_dict
        
        Returns:
            dict: Weights to send to clients (usually same as input)
        """
        pass
    
    def select_clients(
        self,
        available: List[int],
        fraction: float = 1.0
    ) -> List[int]:
        """
        Select subset of clients to participate in this FL round.
        
        Client selection strategies:
            - Random sampling (default)
            - Importance sampling based on data distribution
            - Active selection based on model staleness
        
        Args:
            available: List of available client IDs
            fraction: Fraction of clients to select (0.0 to 1.0)
        
        Returns:
            list: Selected client IDs
        """
        num_selected = max(1, int(len(available) * fraction))
        selected = np.random.choice(available, num_selected, replace=False)
        return selected.tolist()
    
    def increment_round(self):
        """Increment the FL round counter."""
        self.round_number += 1
    
    def get_round_number(self) -> int:
        """
        Get current FL round number.
        
        Returns:
            int: Current round (0-indexed)
        """
        return self.round_number
    
    def reset(self):
        """Reset aggregator state (for new experiment)."""
        self.round_number = 0
        self.history = []
    
    def get_config(self) -> dict:
        """
        Get aggregator configuration.
        
        Returns:
            dict: Configuration dictionary
        """
        return self.config.copy()
    
    def log_round_stats(self, metrics: dict):
        """
        Log statistics for the current round.
        
        Args:
            metrics: Dictionary of metrics to log
        """
        entry = {
            'round': self.round_number,
            **metrics
        }
        self.history.append(entry)
    
    def get_history(self) -> List[Dict]:
        """
        Get complete training history.
        
        Returns:
            list: History of metrics from each round
        """
        return self.history.copy()
    
    def _weighted_average(
        self,
        weights_list: List[Dict],
        weights_factors: Optional[List[float]] = None
    ) -> Dict:
        """
        Compute weighted average of model weights.
        
        This is a utility method used by most FL algorithms.
        
        Args:
            weights_list: List of model state_dicts
            weights_factors: Optional list of weighting factors (normalized internally)
                            If None, use uniform weighting
        
        Returns:
            dict: Averaged model state_dict
        """
        if not weights_list:
            raise ValueError("weights_list cannot be empty")
        
        # Default to uniform weighting
        if weights_factors is None:
            weights_factors = [1.0] * len(weights_list)
        
        # Normalize weights to sum to 1
        total = sum(weights_factors)
        weights_factors = [w / total for w in weights_factors]
        
        # Initialize result with zeros (handle nested dicts recursively)
        avg_weights = {}
        for key in weights_list[0].keys():
            if isinstance(weights_list[0][key], (dict, Mapping)) and not isinstance(weights_list[0][key], torch.Tensor):
                # Nested dict (e.g., MADDPG with 'actor' and 'critic')
                # Recursively average nested structures
                nested_weights = [client[key] for client in weights_list]
                avg_weights[key] = self._weighted_average(nested_weights, weights_factors)
            else:
                # Tensor (e.g., MAPPO/QMIX flat state_dict)
                avg_weights[key] = torch.zeros_like(weights_list[0][key], dtype=torch.float32)
        
        # Weighted sum (skip nested dicts as they're already handled)
        for client_weights, factor in zip(weights_list, weights_factors):
            for key in avg_weights.keys():
                if not isinstance(avg_weights[key], (dict, Mapping)):  # Only process tensors
                    avg_weights[key] += factor * client_weights[key].float()
        
        return avg_weights
    
    def _compute_model_difference(
        self,
        weights1: Dict,
        weights2: Dict
    ) -> float:
        """
        Compute L2 norm of difference between two models.
        
        Useful for monitoring convergence and model drift.
        Handles nested structures (e.g., MADDPG with 'actor'/'critic').
        
        Args:
            weights1: First model state_dict (can be nested)
            weights2: Second model state_dict (can be nested)
        
        Returns:
            float: L2 norm of difference
        """
        diff = 0.0
        for key in weights1.keys():
            if key in weights2:
                if isinstance(weights1[key], dict):
                    # Nested dict - recursively compute difference
                    diff += self._compute_model_difference(weights1[key], weights2[key]) ** 2
                else:
                    # Tensor - compute squared difference
                    diff += torch.sum((weights1[key] - weights2[key]) ** 2).item()
        return np.sqrt(diff)


# Simple FedAvg implementation for reference
class FedAvgAggregator(BaseAggregator):
    """
    Federated Averaging (FedAvg) aggregator.
    
    The most basic FL algorithm: weighted average of client models
    based on local dataset sizes.
    
    Reference:
        Communication-Efficient Learning of Deep Networks from
        Decentralized Data - McMahan et al. 2017
    """
    
    def __init__(self, config: Optional[Dict] = None):
        super().__init__(config)
    
    def aggregate(
        self,
        client_weights: List[Dict],
        client_sizes: Optional[List[int]] = None,
        **kwargs
    ) -> Dict:
        """
        Simple weighted average aggregation.
        
        Args:
            client_weights: List of client model state_dicts
            client_sizes: Local dataset sizes for weighting
        
        Returns:
            dict: Averaged global model
        """
        if client_sizes is None:
            # Uniform weighting if sizes not provided
            client_sizes = [1] * len(client_weights)
        
        return self._weighted_average(client_weights, client_sizes)
    
    def distribute(self, global_weights: Dict) -> Dict:
        """Return global weights as-is."""
        return global_weights


if __name__ == '__main__':
    """Smoke test for base aggregator."""
    
    print("Testing BaseAggregator and FedAvgAggregator...\n")
    
    # Test FedAvgAggregator
    print("Testing FedAvgAggregator...")
    aggregator = FedAvgAggregator(config={'lr': 0.01})
    
    # Create mock client weights
    num_clients = 4
    client_weights = []
    for i in range(num_clients):
        weights = {
            'fc1.weight': torch.randn(10, 5),
            'fc1.bias': torch.randn(10),
            'fc2.weight': torch.randn(5, 10),
            'fc2.bias': torch.randn(5),
        }
        client_weights.append(weights)
    
    print(f"✓ Created {num_clients} mock client models")
    print(f"  Model structure: {list(client_weights[0].keys())}")
    
    # Test aggregation with uniform weights
    print("\nTesting uniform aggregation...")
    global_weights = aggregator.aggregate(client_weights)
    assert set(global_weights.keys()) == set(client_weights[0].keys()), \
        "Global weights should have same keys as client weights"
    print(f"✓ Aggregated {num_clients} clients (uniform)")
    print(f"  Global model keys: {list(global_weights.keys())}")
    
    # Test aggregation with weighted average
    print("\nTesting weighted aggregation...")
    client_sizes = [100, 200, 150, 50]  # Different data sizes
    global_weights_weighted = aggregator.aggregate(client_weights, client_sizes)
    print(f"✓ Aggregated with weights: {client_sizes}")
    
    # Verify weighted average is different from uniform
    diff = aggregator._compute_model_difference(global_weights, global_weights_weighted)
    print(f"  Difference from uniform: {diff:.4f}")
    assert diff > 0, "Weighted aggregation should differ from uniform"
    
    # Test distribution
    print("\nTesting distribution...")
    distributed = aggregator.distribute(global_weights)
    assert distributed is global_weights, "FedAvg should return weights as-is"
    print("✓ Distribution works (returns global model)")
    
    # Test round management
    print("\nTesting round management...")
    assert aggregator.get_round_number() == 0, "Should start at round 0"
    aggregator.increment_round()
    assert aggregator.get_round_number() == 1, "Should increment to round 1"
    aggregator.increment_round()
    assert aggregator.get_round_number() == 2, "Should increment to round 2"
    print(f"✓ Round management: {aggregator.get_round_number()} rounds")
    
    # Test logging
    print("\nTesting logging...")
    aggregator.log_round_stats({'loss': 0.5, 'accuracy': 0.8})
    aggregator.log_round_stats({'loss': 0.4, 'accuracy': 0.85})
    history = aggregator.get_history()
    assert len(history) == 2, f"Expected 2 history entries, got {len(history)}"
    print(f"✓ Logged {len(history)} rounds")
    print(f"  History: {history}")
    
    # Test client selection
    print("\nTesting client selection...")
    available_clients = list(range(10))
    selected = aggregator.select_clients(available_clients, fraction=0.5)
    assert len(selected) == 5, f"Expected 5 clients, got {len(selected)}"
    print(f"✓ Selected {len(selected)}/10 clients: {selected}")
    
    # Test reset
    print("\nTesting reset...")
    aggregator.reset()
    assert aggregator.get_round_number() == 0, "Round should reset to 0"
    assert len(aggregator.get_history()) == 0, "History should be empty"
    print("✓ Reset successful")
    
    # Test model difference computation
    print("\nTesting model difference computation...")
    weights_a = {
        'param1': torch.tensor([1.0, 2.0, 3.0]),
        'param2': torch.tensor([4.0, 5.0]),
    }
    weights_b = {
        'param1': torch.tensor([1.1, 2.1, 3.1]),
        'param2': torch.tensor([4.1, 5.1]),
    }
    diff = aggregator._compute_model_difference(weights_a, weights_b)
    expected_diff = np.sqrt(5 * (0.1**2))  # 5 parameters, each differs by 0.1
    assert abs(diff - expected_diff) < 1e-5, f"Expected {expected_diff}, got {diff}"
    print(f"✓ Model difference: {diff:.6f}")
    
    print("\n✅ All base aggregator tests passed!")

