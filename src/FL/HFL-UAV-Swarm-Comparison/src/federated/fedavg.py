import torch
import copy
from typing import List, Dict, Optional
from src.federated.base_aggregator import BaseAggregator

class FedAvg(BaseAggregator):
    """
    Standard Federated Averaging implementation.
    
    FedAvg performs simple averaging of client model weights,
    optionally weighted by local dataset sizes.
    """
    def __init__(self, config):
        super().__init__(config)
        
    def aggregate(
        self,
        client_weights: List[Dict],
        client_sizes: Optional[List[int]] = None,
        **kwargs
    ) -> Dict:
        """
        Average weights from client models.
        
        Supports aggregation of nested state_dicts (for actor-critic architectures).
        Can perform weighted averaging if client_sizes provided.
        
        Args:
            client_weights: List of client model state_dicts
            client_sizes: Optional list of dataset sizes for weighted averaging
            **kwargs: Additional arguments (legacy compatibility)
            
        Returns:
            Dict: Averaged state_dict
        """
        if not client_weights:
            return {}
        
        # Legacy support: if called with (global_model, client_models) signature
        # Extract the second argument as client_weights list
        if 'global_model' in kwargs or len(client_weights) == 1:
            # Old API compatibility
            if 'global_model' in kwargs and 'client_models' in kwargs:
                client_weights = kwargs['client_models']
        
        # Helper to get state
        def get_state(obj):
            if hasattr(obj, 'state_dict'):
                return obj.state_dict()
            elif hasattr(obj, 'get_weights'):  # Custom method for agents
                return obj.get_weights()
            return obj
        
        client_states = [get_state(m) for m in client_weights]
        if not client_states:
            return {}
        
        # Determine weights for averaging
        if client_sizes is not None:
            total_size = sum(client_sizes)
            weights = [size / total_size for size in client_sizes]
        else:
            weights = [1.0 / len(client_states)] * len(client_states)
            
        # Recursive averaging function with weighted average
        def recursive_average_weighted(states, weights):
            if not states:
                return None
            
            first = states[0]
            
            if isinstance(first, torch.Tensor):
                # Weighted average of tensors
                avg = torch.zeros_like(first).float()
                for state, weight in zip(states, weights):
                    avg += state.float() * weight
                return avg.type(first.dtype)
                
            elif isinstance(first, dict):
                # Recurse into dictionary
                avg_dict = {}
                for key in first.keys():
                    avg_dict[key] = recursive_average_weighted(
                        [s[key] for s in states],
                        weights
                    )
                return avg_dict
            else:
                # Fallback for non-tensor/non-dict types
                return first
        
        averaged_state = recursive_average_weighted(client_states, weights)
        
        # Increment round number
        self.round_number += 1
        
        return averaged_state
    def distribute(self, global_weights: Dict) -> Dict:
        """
        Distribute global weights to clients.
        
        For FedAvg, we simply return the global weights as-is.
        No post-processing needed.
        
        Args:
            global_weights: Aggregated global model state_dict
        
        Returns:
            dict: Same weights (no modification for FedAvg)
        """
        return global_weights