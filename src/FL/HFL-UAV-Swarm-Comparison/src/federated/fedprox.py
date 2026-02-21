"""
FedProx Federated Learning Aggregator

Implements FedProx algorithm (Federated Proximal):
- Extends FedAvg with a proximal term
- Handles system heterogeneity (different computational capabilities)
- Prevents devices from straying too far from the global model

FedProx Equation:
    minimize: loss(w) + mu/2 * ||w - w_global||^2
    
Where:
    - loss(w): Local training loss
    - mu: Proximal coefficient (controls heterogeneity penalty)
    - w: Local model weights
    - w_global: Global model from previous round
"""

import numpy as np
import torch
from typing import Dict, List
from copy import deepcopy

from src.federated.base_aggregator import BaseAggregator


class FedProxAggregator(BaseAggregator):
    """
    FedProx aggregation strategy for heterogeneous federated learning.
    
    Features:
    - Proximal term for stability under system heterogeneity
    - Weighted averaging based on local epochs
    - Communication efficiency metrics
    """
    
    def __init__(self,
                 num_agents: int,
                 num_rounds: int = 100,
                 local_epochs: int = 5,
                 mu: float = 0.01,
                 adaptive_mu: bool = False):
        """
        Initialize FedProx aggregator.
        
        Args:
            num_agents: Number of agents
            num_rounds: Total number of FL rounds
            local_epochs: Local training epochs per agent
            mu: Proximal coefficient (0.001 to 0.1 typical)
            adaptive_mu: Whether to adapt mu based on communication round
        """
        config = {
            'num_agents': num_agents,
            'num_rounds': num_rounds,
            'local_epochs': local_epochs,
            'mu': mu,
            'adaptive_mu': adaptive_mu,
        }
        super().__init__(config)
        
        self.num_agents = num_agents
        self.num_rounds = num_rounds
        self.local_epochs = local_epochs
        self.mu = mu
        self.adaptive_mu = adaptive_mu
        
        # Track proximal coefficients per round
        self.mu_history = []
        
        # Global model weights (for proximal term)
        self.global_weights = None
        self.initialization_count = 0
        
        # Communication metrics tracking
        self.aggregation_rounds = 0
    
    def aggregate(self, client_weights: Dict[int, Dict]) -> Dict[int, Dict]:
        """
        Aggregate client weights using FedProx algorithm.
        
        CRITICAL: FedProx aggregation is the same as FedAvg.
        The proximal term is applied DURING local training, not during aggregation.
        
        The agents add (μ/2)||w - w_global||² to their training loss.
        This aggregator just averages the weights like FedAvg.
        
        Args:
            client_weights: Dict mapping client_id -> model_weights
        
        Returns:
            Dict mapping client_id -> aggregated_weights (same for all clients)
        """
        # Initialize global weights on first call
        if self.global_weights is None:
            self.global_weights = deepcopy(client_weights[0])
            self.initialization_count += 1
        
        # Get adaptive mu for this round (for reporting only)
        mu = self._get_adaptive_mu()
        self.mu_history.append(mu)
        
        # ═══════════════════════════════════════════════════════════════
        # CRITICAL FIX: FedProx uses FedAvg aggregation
        # ═══════════════════════════════════════════════════════════════
        # The proximal term is in the LOCAL training loss, not here.
        # We just average like FedAvg and return the same weights to all clients.
        
        aggregated = self._weighted_average(client_weights)
        
        # Update global weights (for next round's proximal term)
        self.global_weights = aggregated
        
        self.aggregation_rounds += 1
        self._update_communication_metrics()
        
        # Return single aggregated model (distribute() will wrap per-client)
        return aggregated
    
    def _weighted_average(self, client_weights: Dict[int, Dict]) -> Dict:
        """
        Compute weighted average of client weights.
        
        Args:
            client_weights: Dict of client weights
        
        Returns:
            Dict of averaged weights
        """
        # Simple equal-weight average (could use other weights based on data/computation)
        num_clients = len(client_weights)
        
        # Detect if weights are flat (tensor keys) or nested (multiple subkeys)
        first_weights = client_weights[0]
        is_flat = any(isinstance(v, torch.Tensor) for v in first_weights.values())
        
        if is_flat:
            # Flat dict: average directly
            averaged = {}
            for param_name in first_weights.keys():
                stacked = torch.stack([
                    client_weights[client_id][param_name]
                    for client_id in range(num_clients)
                ])
                averaged[param_name] = stacked.mean(dim=0)
        else:
            # Nested dict structure (e.g., from MAPPO with 'actor', 'critic' keys)
            averaged = {}
            for weight_key in first_weights.keys():
                averaged[weight_key] = {}
                for param_name in first_weights[weight_key].keys():
                    if isinstance(first_weights[weight_key][param_name], torch.Tensor):
                        stacked = torch.stack([
                            client_weights[client_id][weight_key][param_name]
                            for client_id in range(num_clients)
                        ])
                        averaged[weight_key][param_name] = stacked.mean(dim=0)
                    else:
                        averaged[weight_key][param_name] = first_weights[weight_key][param_name]
        
        return averaged
    
    def _get_adaptive_mu(self) -> float:
        """
        Get adaptive mu coefficient.
        
        Options:
        - Fixed: Always use initial mu
        - Decay: Decrease mu over rounds (start strict, become lenient)
        - Increase: Increase mu over rounds (start lenient, become strict)
        
        Returns:
            float: Mu coefficient for current round
        """
        if not self.adaptive_mu:
            return self.mu
        
        # Decay mu over time (start strict, become lenient)
        progress = self.aggregation_rounds / max(1, self.num_rounds)
        mu = self.mu * (1 - 0.5 * progress)  # Linearly decay by 50%
        
        return max(0.0001, mu)
    
    def distribute(self, aggregated_weights: Dict) -> Dict[int, Dict]:
        """
        Distribute aggregated weights to clients.
        
        Args:
            aggregated_weights: Aggregated weights from aggregate()
        
        Returns:
            Dict mapping client_id -> weights_for_that_client
        """
        # FedProx sends same global model to all clients
        # (they use it as w_global in their proximal term next round)
        return {client_id: aggregated_weights for client_id in range(self.num_agents)}
    
    def _update_communication_metrics(self):
        """Update communication metrics tracking."""
        # FedProx uses standard FedAvg communication
        self.last_comm_metrics = self.get_communication_metrics()
    
    def get_communication_metrics(self) -> Dict:
        """
        Get communication metrics for this round.
        
        FedProx has same communication cost as FedAvg:
        - Upload: Each client sends local weights
        - Download: Server sends global weights
        
        Returns:
            Dict with metrics
        """
        # FedProx doesn't reduce communication compared to FedAvg
        # But it improves convergence under heterogeneity
        bytes_per_weight = 4  # 32-bit float
        
        # Rough estimation: 100 weights per layer, 5 layers typical
        num_weights = 100 * 5
        bytes_per_round = self.num_agents * num_weights * bytes_per_weight
        
        return {
            'bytes_uploaded': bytes_per_round,
            'bytes_downloaded': bytes_per_round,
            'communication_savings': 0.0,  # No communication savings vs FedAvg
            'mu_coefficient': self.mu_history[-1] if self.mu_history else self.mu,
        }
    
    def get_aggregation_info(self) -> Dict:
        """
        Get information about aggregation strategy.
        
        Returns:
            Dict with strategy details
        """
        return {
            'algorithm': 'FedProx',
            'num_agents': self.num_agents,
            'num_rounds': self.num_rounds,
            'local_epochs': self.local_epochs,
            'base_mu': self.mu,
            'adaptive_mu': self.adaptive_mu,
            'aggregation_rounds': self.aggregation_rounds,
            'avg_mu': np.mean(self.mu_history) if self.mu_history else self.mu,
        }


class AdaptiveFedProxAggregator(FedProxAggregator):
    """
    Adaptive FedProx with learned proximal coefficient.
    
    Adjusts mu based on:
    - Client variance (higher variance -> higher mu)
    - Communication round (decay mu over time)
    - System heterogeneity metrics
    """
    
    def __init__(self,
                 num_agents: int,
                 num_rounds: int = 100,
                 local_epochs: int = 5,
                 initial_mu: float = 0.01,
                 learning_rate: float = 0.01):
        """Initialize adaptive FedProx."""
        super().__init__(num_agents, num_rounds, local_epochs, initial_mu, adaptive_mu=True)
        
        self.learning_rate = learning_rate
        self.client_variance_history = []
        self.mu_history = []
    
    def aggregate(self, client_weights: Dict[int, Dict]) -> Dict[int, Dict]:
        """
        Aggregate with adaptive mu based on client variance.
        
        Args:
            client_weights: Dict of client weights
        
        Returns:
            Dict of aggregated weights
        """
        # Compute variance of client weights
        variance = self._compute_weight_variance(client_weights)
        self.client_variance_history.append(variance)
        
        # Adapt mu based on variance
        if len(self.client_variance_history) > 1:
            variance_increase = variance / (self.client_variance_history[-2] + 1e-8)
            self.mu = self.mu * (1 + self.learning_rate * variance_increase)
            self.mu = np.clip(self.mu, 0.0001, 0.5)  # Clamp mu
        
        # Call parent aggregate
        return super().aggregate(client_weights)
    
    def _compute_weight_variance(self, client_weights: Dict[int, Dict]) -> float:
        """
        Compute variance across client weights.
        
        Args:
            client_weights: Dict of client weights
        
        Returns:
            float: Average variance of all weight tensors
        """
        variances = []
        
        for client_id in client_weights:
            for weight_key in client_weights[client_id]:
                for param_name, param in client_weights[client_id][weight_key].items():
                    if isinstance(param, torch.Tensor):
                        var = param.var().item()
                        variances.append(var)
        
        return np.mean(variances) if variances else 0.0
