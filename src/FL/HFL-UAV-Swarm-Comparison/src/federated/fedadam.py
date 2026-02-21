"""
FedAdam — Adaptive Federated Optimization.

Implements server-side Adam optimizer for federated learning, combining
the stability of FedAvg with adaptive learning rates (momentum + second
moments) to improve convergence across heterogeneous clients.

Reference:
    "Adaptive Federated Optimization" — Reddi et al., ICLR 2021
    https://arxiv.org/abs/2003.00295

Key Idea:
    FedAvg computes pseudo-gradients (global - weighted_avg_client) and
    applies vanilla SGD on the server side. FedAdam replaces the server
    SGD step with an Adam update, using momentum (m) and second moments (v)
    to adapt the effective learning rate per parameter.

    Server update (one FL round):
        pseudo_grad = global_weights - aggregated_client_weights
        m  = beta1 * m  + (1 - beta1) * pseudo_grad          # 1st moment
        v  = beta2 * v  + (1 - beta2) * pseudo_grad^2        # 2nd moment
        m_hat  = m  / (1 - beta1^t)                          # bias-corrected
        v_hat  = v  / (1 - beta2^t)                          # bias-corrected
        global_weights = global_weights - lr * m_hat / (sqrt(v_hat) + eps)

Integration:
    Plugs directly into MAPPOTrainer / MADDPGTrainer via BaseAggregator interface.
    Compatible with all 3 MARL algorithms (MAPPO, MADDPG, QMIX).
"""

import copy
import torch
from typing import Dict, List, Optional

from src.federated.base_aggregator import BaseAggregator


class FedAdam(BaseAggregator):
    """
    FedAdam: Adaptive federated optimizer with server-side Adam.

    Extends FedAvg by maintaining server-side first and second moment
    estimates (bias-corrected Adam) so each parameter gets its own
    effective learning rate, accelerating convergence especially when
    gradient magnitudes differ widely across layers.

    Why FedAdam for UAV swarms?
        - Heterogeneous UAVs: different battery states and task loads
          produce heterogeneous gradients -> Adam per-parameter LR helps
        - Faster convergence: critical when communication rounds are costly
        - Robust to non-IID data: momentum smooths noisy pseudo-gradients

    Args:
        config: Configuration dict. Keys:
            server_learning_rate  (float, default 0.01)  -- eta_s in Adam
            client_learning_rate  (float, default 0.001) -- stored for reference
            beta1                 (float, default 0.9)   -- 1st moment decay
            beta2                 (float, default 0.99)  -- 2nd moment decay
            epsilon               (float, default 1e-3)  -- numerical stability
            tau                   (float, default 1e-3)  -- initial v floor
    """

    def __init__(self, config: Optional[Dict] = None):
        super().__init__(config)

        # Apply defaults then user overrides
        defaults = {
            'server_learning_rate': 0.01,
            'client_learning_rate': 0.001,
            'beta1': 0.9,
            'beta2': 0.99,
            'epsilon': 1e-3,
            'tau': 1e-3,
        }
        defaults.update(self.config)
        self.config = defaults

        self.server_lr = self.config['server_learning_rate']
        self.beta1 = self.config['beta1']
        self.beta2 = self.config['beta2']
        self.epsilon = self.config['epsilon']
        self.tau = self.config['tau']

        # Server-side Adam state (initialised on first aggregate call)
        self.m: Optional[Dict] = None   # 1st moment (momentum)
        self.v: Optional[Dict] = None   # 2nd moment (adaptive scale)
        self.global_weights: Optional[Dict] = None  # Current global model

        # Separate Adam step counter — decoupled from round_number so that
        # trainers that call increment_round() after aggregate() (e.g. MAPPO)
        # do not corrupt the bias-correction denominator.
        self._adam_t: int = 0

    # ------------------------------------------------------------------
    # Core FL interface
    # ------------------------------------------------------------------

    def aggregate(
        self,
        client_weights: List[Dict],
        client_sizes: Optional[List[int]] = None,
        **kwargs
    ) -> Dict:
        """
        Aggregate client models with server-side Adam update.

        Steps:
            1. Compute weighted average of client models (same as FedAvg)
            2. Compute pseudo-gradient = global - client_average
            3. Update server Adam state (m, v)
            4. Apply bias-corrected Adam step to global model

        Args:
            client_weights: List of actor state_dicts from each agent
            client_sizes:   Optional per-client dataset sizes for weighted avg.
                            If None, uniform weighting is used.
            **kwargs:       Ignored (legacy compatibility)

        Returns:
            dict: Updated global model state_dict
        """
        if not client_weights:
            return self.global_weights if self.global_weights else {}

        # Step 1: Weighted average of client models
        if client_sizes is None:
            client_sizes = [1] * len(client_weights)

        client_avg = self._weighted_average(client_weights, client_sizes)

        # Step 2: Initialise global model (first call only)
        if self.global_weights is None:
            self.global_weights = copy.deepcopy(client_avg)
            self._init_adam_state(client_avg)

        # Step 3: Pseudo-gradient  delta = global - client_average
        pseudo_grad = self._compute_pseudo_gradient(
            self.global_weights, client_avg
        )

        # Step 4: Adam update
        # Use dedicated _adam_t counter so trainers that call increment_round()
        # after aggregate() (e.g. MAPPOTrainer) don't corrupt bias correction.
        self._adam_t += 1
        t = self._adam_t
        bc1 = 1.0 - self.beta1 ** t   # bias-correction denominator for m
        bc2 = 1.0 - self.beta2 ** t   # bias-correction denominator for v

        updated = {}
        for key in self.global_weights.keys():
            if isinstance(self.global_weights[key], dict):
                # Nested dict - recursively update
                updated[key] = {}
                for sub_key in self.global_weights[key].keys():
                    g = pseudo_grad[key][sub_key]
                    
                    # Update moments
                    self.m[key][sub_key] = (
                        self.beta1 * self.m[key][sub_key] + (1 - self.beta1) * g
                    )
                    self.v[key][sub_key] = (
                        self.beta2 * self.v[key][sub_key] + (1 - self.beta2) * (g ** 2)
                    )
                    
                    # Bias-corrected moments
                    m_hat = self.m[key][sub_key] / bc1
                    v_hat = self.v[key][sub_key] / bc2
                    
                    # Adam step
                    updated[key][sub_key] = (
                        self.global_weights[key][sub_key] - self.server_lr * (m_hat / (torch.sqrt(v_hat) + self.epsilon))
                    )
                continue
            
            # Flat tensor case (original logic)
            if key not in pseudo_grad:
                updated[key] = self.global_weights[key]
                continue

            g = pseudo_grad[key].float()

            # Update moments
            self.m[key] = self.beta1 * self.m[key] + (1.0 - self.beta1) * g
            self.v[key] = self.beta2 * self.v[key] + (1.0 - self.beta2) * g * g

            # Bias-corrected estimates
            m_hat = self.m[key] / bc1
            v_hat = self.v[key] / bc2

            # Adam step: move global model toward client average adaptively
            # (pseudo-grad points from client_avg toward global, so subtracting
            #  it moves the global model toward the client average — FedAvg
            #  direction — but with adaptive per-parameter learning rates)
            step = self.server_lr * m_hat / (torch.sqrt(v_hat) + self.epsilon)
            updated[key] = (self.global_weights[key].float() - step).to(
                self.global_weights[key].dtype
            )

        self.global_weights = updated
        return copy.deepcopy(self.global_weights)

    def distribute(self, global_weights: Dict) -> Dict:
        """
        Return the global model for distribution to all clients.

        FedAdam distributes the same global model to every client (no
        personalisation), identical to FedAvg distribution.

        Args:
            global_weights: Aggregated global model state_dict

        Returns:
            dict: Weights to send to clients (unchanged)
        """
        return global_weights

    # ------------------------------------------------------------------
    # Utility helpers
    # ------------------------------------------------------------------

    def _init_adam_state(self, reference_weights: Dict):
        """
        Initialise Adam moment accumulators (m, v).

        m is zero-initialised.
        v is initialised to tau^2 (a small positive value) rather than 0
        to avoid division-by-zero in the first update and to dampen
        overly large initial steps (following the original paper).

        Handles nested structures (e.g., MADDPG with 'actor'/'critic').

        Args:
            reference_weights: Any state_dict to extract key/shape info from (can be nested)
        """
        self.m = {}
        self.v = {}
        for key, value in reference_weights.items():
            if isinstance(value, dict):
                # Nested dict - recursively initialize
                self.m[key] = {}
                self.v[key] = {}
                for sub_key, tensor in value.items():
                    self.m[key][sub_key] = torch.zeros_like(tensor, dtype=torch.float32)
                    self.v[key][sub_key] = torch.full_like(
                        tensor, self.tau ** 2, dtype=torch.float32
                    )
            else:
                # Tensor - direct initialization
                self.m[key] = torch.zeros_like(value, dtype=torch.float32)
                self.v[key] = torch.full_like(
                    value, self.tau ** 2, dtype=torch.float32
                )

    def _compute_pseudo_gradient(
        self,
        global_weights: Dict,
        client_avg: Dict
    ) -> Dict:
        """
        Compute pseudo-gradient as (global - client_average).

        This pseudo-gradient approximates the direction in which the server
        model differs from what the clients collectively learned. Applying
        Adam with this achieves adaptive server-side learning.

        Handles nested structures (e.g., MADDPG with 'actor'/'critic').

        Args:
            global_weights: Current global model state_dict (can be nested)
            client_avg:     Weighted average of client state_dicts (can be nested)

        Returns:
            dict: Per-parameter pseudo-gradient tensors (possibly nested)
        """
        pseudo_grad = {}
        for key in global_weights.keys():
            if key in client_avg:
                if isinstance(global_weights[key], dict):
                    # Nested dict - recursively compute gradient
                    pseudo_grad[key] = self._compute_pseudo_gradient(
                        global_weights[key], client_avg[key]
                    )
                else:
                    # Tensor - direct subtraction
                    pseudo_grad[key] = (
                        global_weights[key].float() - client_avg[key].float()
                    )
        return pseudo_grad

    def reset(self):
        """
        Reset aggregator state (Adam moments + round counter).

        Call this between independent experiments to avoid state leakage.
        """
        super().reset()
        self.m = None
        self.v = None
        self.global_weights = None

    def get_adam_state(self) -> Dict:
        """
        Return a copy of the current Adam state for inspection / logging.

        Returns:
            dict: {'m': ..., 'v': ..., 'round': int}
        """
        return {
            'round': self.round_number,
            'm': copy.deepcopy(self.m) if self.m else None,
            'v': copy.deepcopy(self.v) if self.v else None,
        }

    def get_effective_lr(self) -> Optional[Dict[str, float]]:
        """
        Compute the current effective learning rate per parameter group.

        Useful for diagnostics: large effective LR -> fast learning,
        small effective LR -> parameter has converged or is noisy.

        Returns:
            dict mapping param key to mean effective LR, or None if not yet initialised
        """
        if self.m is None or self.v is None:
            return None

        t = max(1, self.round_number)
        bc2 = 1.0 - self.beta2 ** t

        effective_lrs = {}
        for key in self.v.keys():
            v_hat = self.v[key] / bc2
            eff_lr = self.server_lr / (torch.sqrt(v_hat) + self.epsilon)
            effective_lrs[key] = eff_lr.mean().item()
        return effective_lrs


# ======================================================================
# Smoke test
# ======================================================================

if __name__ == '__main__':
    """Smoke test for FedAdam aggregator."""

    print("Testing FedAdam aggregator...\n")

    # Setup
    config = {
        'server_learning_rate': 0.01,
        'beta1': 0.9,
        'beta2': 0.99,
        'epsilon': 1e-3,
        'tau': 1e-3,
    }
    aggregator = FedAdam(config=config)
    print(f"✓ Created FedAdam aggregator")
    print(f"  server_lr={aggregator.server_lr}, beta1={aggregator.beta1}, "
          f"beta2={aggregator.beta2}, epsilon={aggregator.epsilon}")

    # Create mock client models
    num_clients = 4
    model_keys = ['fc1.weight', 'fc1.bias', 'fc2.weight', 'fc2.bias']
    shapes = [(64, 40), (64,), (5, 64), (5,)]

    def make_client_weights(seed: int) -> Dict:
        torch.manual_seed(seed)
        return {k: torch.randn(s) for k, s in zip(model_keys, shapes)}

    client_weights = [make_client_weights(i) for i in range(num_clients)]
    print(f"✓ Created {num_clients} mock client models")
    print(f"  Model structure: {model_keys}")

    # Test 1: First aggregation
    print("\nTest 1: First aggregation (Adam state initialisation)")
    global_w = aggregator.aggregate(client_weights)
    assert set(global_w.keys()) == set(model_keys), "Key mismatch"
    assert aggregator.m is not None, "Momentum (m) should be initialised"
    assert aggregator.v is not None, "Second moment (v) should be initialised"
    assert aggregator.round_number == 1, f"Round should be 1, got {aggregator.round_number}"
    print(f"✓ First aggregation completed, round={aggregator.round_number}")

    # Test 2: Multiple rounds — model must change
    print("\nTest 2: Multiple rounds with model drift tracking")
    prev_weights = {k: v.clone() for k, v in global_w.items()}

    for r in range(5):
        client_weights_r = [make_client_weights(r * 10 + i) for i in range(num_clients)]
        global_w = aggregator.aggregate(client_weights_r)

    any_changed = any(
        not torch.allclose(global_w[k], prev_weights[k])
        for k in model_keys
    )
    assert any_changed, "Global model should change across rounds"
    print(f"✓ Model evolves across {aggregator.round_number} rounds")

    # Test 3: Weighted vs uniform aggregation
    print("\nTest 3: Weighted vs uniform aggregation")
    agg_uniform = FedAdam(config=config)
    agg_weighted = FedAdam(config=config)

    clients = [make_client_weights(42 + i) for i in range(4)]
    sizes = [100, 200, 150, 50]

    gw_u = agg_uniform.aggregate(clients)
    gw_w = agg_weighted.aggregate(clients, client_sizes=sizes)

    diff = sum(
        (gw_u[k].float() - gw_w[k].float()).abs().mean().item()
        for k in model_keys
    )
    assert diff > 0, "Weighted and uniform results should differ"
    print(f"✓ Weighted aggregation differs from uniform (mean L1={diff:.6f})")

    # Test 4: Adaptive LR diagnostics
    print("\nTest 4: Effective learning rate diagnostics")
    eff_lrs = aggregator.get_effective_lr()
    assert eff_lrs is not None, "Should return effective LRs after training"
    assert set(eff_lrs.keys()) == set(model_keys), "Should have LR for each param"
    for k, lr in eff_lrs.items():
        assert lr > 0, f"Effective LR for {k} should be positive"
        print(f"  {k}: eff_lr approx {lr:.6f}")
    print("✓ Effective LR diagnostics work")

    # Test 5: Adam state inspection
    print("\nTest 5: Adam state access")
    state = aggregator.get_adam_state()
    assert 'round' in state and 'm' in state and 'v' in state
    assert state['round'] == aggregator.round_number
    print(f"✓ Adam state accessible (round={state['round']})")

    # Test 6: Distribute
    print("\nTest 6: Distribution")
    distributed = aggregator.distribute(global_w)
    assert distributed is global_w, "distribute() should return weights as-is"
    print("✓ Distribution returns global model unchanged")

    # Test 7: Reset
    print("\nTest 7: Reset")
    aggregator.reset()
    assert aggregator.round_number == 0, "Round should reset to 0"
    assert aggregator.m is None, "Adam moments should be cleared on reset"
    assert aggregator.v is None, "Adam moments should be cleared on reset"
    assert aggregator.global_weights is None, "Global weights should clear on reset"
    print("✓ Reset clears Adam state and round counter")

    # Test 8: Scalability across swarm sizes
    print("\nTest 8: Scalability across swarm sizes")
    for n_agents in [5, 10, 20, 50, 100]:
        obs_dim = 8 + 20 + 4 * (n_agents - 1)
        agg = FedAdam(config=config)
        clients_n = [
            {'actor.fc1.weight': torch.randn(256, obs_dim),
             'actor.fc1.bias':   torch.randn(256)}
            for _ in range(n_agents)
        ]
        gw_n = agg.aggregate(clients_n)
        assert gw_n is not None
        print(f"  {n_agents:3d} agents, obs_dim={obs_dim:3d} -> OK")

    print("\n✅ All FedAdam tests passed!")
