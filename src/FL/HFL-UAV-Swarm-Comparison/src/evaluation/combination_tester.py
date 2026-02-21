"""
CombinationTester — Test all FL-MARL algorithm combinations.

Runs the full 4 FL x 3 MARL = 12 combination matrix (or subset) across
all swarm sizes (5, 10, 20, 50, 100 drones) and multiple random seeds,
collecting aggregated metrics for each combination.

Results are stored as a nested dict and can be exported to CSV/JSON for
analysis by the Comparator and SelectionFramework.

Usage:
    from src.evaluation.combination_tester import CombinationTester

    tester = CombinationTester(config={'seeds': [0,1,2], 'num_eval_episodes': 10})
    results = tester.run_combination('mappo', 'fedadam', num_drones=10)
"""

import os
import json
import time
import numpy as np
from typing import Dict, List, Optional, Any
from itertools import product

from src.evaluation.metrics_tracker import MetricsTracker


# Canonical combination names matching team ownership
FL_ALGORITHMS = ['fedavg', 'fedprox', 'fedadam', 'hierarchical']
MARL_ALGORITHMS = ['mappo', 'maddpg', 'qmix']
SWARM_SIZES = [5, 10, 20, 50, 100]


class CombinationTester:
    """
    Orchestrates testing of all FL-MARL combinations.

    Coordinates results collection from the three MARL owners:
        - MAPPO (Belal): src/algorithms/mappo/
        - MADDPG (Abdalla): src/algorithms/maddpg/
        - QMIX (Jo): src/algorithms/qmix/

    Combined with all four FL algorithms owned by:
        - FedAvg (Abdalla), FedProx (Jo), FedAdam (Ammar), HFL (Belal)

    This class stores, loads, and summarises results from disk so that
    experiments can be run incrementally across team members.

    Args:
        config: Testing configuration dict
        results_dir: Directory to store raw results
    """

    def __init__(
        self,
        config: Optional[Dict] = None,
        results_dir: str = 'results/fl_marl_combinations'
    ):
        default_config = {
            'seeds': [0, 1, 2, 3, 4],
            'num_eval_episodes': 10,
            'swarm_sizes': SWARM_SIZES,
            'fl_algorithms': FL_ALGORITHMS,
            'marl_algorithms': MARL_ALGORITHMS,
            'verbose': True,
        }
        if config:
            default_config.update(config)
        self.config = default_config
        self.results_dir = results_dir
        os.makedirs(results_dir, exist_ok=True)

        # In-memory results store: {(fl, marl, n_drones, seed): metrics_dict}
        self.results: Dict = {}

    # ------------------------------------------------------------------
    # Result storage and retrieval
    # ------------------------------------------------------------------

    def store_result(
        self,
        fl_name: str,
        marl_name: str,
        num_drones: int,
        seed: int,
        metrics: Dict
    ):
        """
        Store evaluation results for one combination / seed.

        Args:
            fl_name:    FL algorithm name ('fedavg', 'fedprox', 'fedadam', 'hierarchical')
            marl_name:  MARL algorithm name ('mappo', 'maddpg', 'qmix')
            num_drones: Swarm size
            seed:       Random seed used
            metrics:    Dict from Evaluator.evaluate()
        """
        key = (fl_name, marl_name, num_drones, seed)
        self.results[key] = {
            'fl': fl_name,
            'marl': marl_name,
            'num_drones': num_drones,
            'seed': seed,
            **metrics
        }

        # Persist to disk for cross-session access
        combo_dir = os.path.join(
            self.results_dir, f'{fl_name}_{marl_name}'
        )
        os.makedirs(combo_dir, exist_ok=True)
        fname = os.path.join(combo_dir, f'{num_drones}drones_seed{seed}.json')
        with open(fname, 'w') as f:
            # Convert numpy types to native Python for JSON
            clean = {k: float(v) if hasattr(v, 'item') else v
                     for k, v in metrics.items()}
            json.dump({'fl': fl_name, 'marl': marl_name,
                       'num_drones': num_drones, 'seed': seed, **clean}, f, indent=2)

    def load_result(
        self,
        fl_name: str,
        marl_name: str,
        num_drones: int,
        seed: int
    ) -> Optional[Dict]:
        """
        Load stored result from disk if it exists.

        Returns:
            dict or None: Stored metrics, or None if not found
        """
        combo_dir = os.path.join(self.results_dir, f'{fl_name}_{marl_name}')
        fname = os.path.join(combo_dir, f'{num_drones}drones_seed{seed}.json')
        if os.path.exists(fname):
            with open(fname) as f:
                return json.load(f)
        return None

    def load_all_results(self):
        """
        Load all persisted results from disk into self.results.

        Call this at the start of analysis to aggregate results from
        all team members who saved their experiment results.
        """
        for fl_name in FL_ALGORITHMS:
            for marl_name in MARL_ALGORITHMS:
                combo_dir = os.path.join(self.results_dir, f'{fl_name}_{marl_name}')
                if not os.path.isdir(combo_dir):
                    continue
                for fname in os.listdir(combo_dir):
                    if not fname.endswith('.json'):
                        continue
                    fpath = os.path.join(combo_dir, fname)
                    with open(fpath) as f:
                        data = json.load(f)
                    key = (data['fl'], data['marl'], data['num_drones'], data['seed'])
                    self.results[key] = data

    # ------------------------------------------------------------------
    # Aggregation helpers
    # ------------------------------------------------------------------

    def get_combination_results(
        self,
        fl_name: str,
        marl_name: str,
        num_drones: Optional[int] = None
    ) -> List[Dict]:
        """
        Get all stored results for a given FL-MARL combination.

        Args:
            fl_name:    FL algorithm name
            marl_name:  MARL algorithm name
            num_drones: Filter by swarm size (None = all sizes)

        Returns:
            list: Matching result dicts
        """
        results = []
        for key, val in self.results.items():
            if key[0] == fl_name and key[1] == marl_name:
                if num_drones is None or key[2] == num_drones:
                    results.append(val)
        return results

    def summarize_combination(
        self,
        fl_name: str,
        marl_name: str,
        metric: str = 'avg_episode_reward'
    ) -> Dict:
        """
        Compute mean +/- std of a metric across all seeds and swarm sizes.

        Args:
            fl_name:   FL algorithm
            marl_name: MARL algorithm
            metric:    Metric key to summarize

        Returns:
            dict: {swarm_size: {'mean': float, 'std': float, 'n_seeds': int}}
        """
        summary = {}
        for n in SWARM_SIZES:
            results = self.get_combination_results(fl_name, marl_name, n)
            values = [r[metric] for r in results if metric in r]
            if values:
                summary[n] = {
                    'mean': float(np.mean(values)),
                    'std': float(np.std(values)),
                    'n_seeds': len(values),
                    'values': values,
                }
        return summary

    def build_performance_matrix(
        self,
        metric: str = 'avg_episode_reward',
        num_drones: int = 10
    ) -> Dict:
        """
        Build a FL x MARL performance matrix for a given metric / swarm size.

        Returns a nested dict compatible with seaborn heatmap plotting.

        Args:
            metric:     Metric to use for the matrix cells
            num_drones: Swarm size to evaluate at

        Returns:
            dict: {fl_name: {marl_name: mean_value}}
        """
        matrix = {}
        for fl_name in FL_ALGORITHMS:
            matrix[fl_name] = {}
            for marl_name in MARL_ALGORITHMS:
                results = self.get_combination_results(fl_name, marl_name, num_drones)
                values = [r[metric] for r in results if metric in r]
                matrix[fl_name][marl_name] = float(np.mean(values)) if values else None
        return matrix

    def get_all_combinations(self) -> List[tuple]:
        """
        Return all (fl_name, marl_name) pairs.

        Returns:
            list: [(fl_name, marl_name), ...]
        """
        return list(product(FL_ALGORITHMS, MARL_ALGORITHMS))

    def get_missing_combinations(
        self,
        swarm_sizes: Optional[List[int]] = None,
        seeds: Optional[List[int]] = None
    ) -> List[Dict]:
        """
        Identify which (fl, marl, n_drones, seed) experiments are missing.

        Useful for tracking which experiments still need to be run.

        Returns:
            list: Dicts describing missing experiments
        """
        swarm_sizes = swarm_sizes or self.config['swarm_sizes']
        seeds = seeds or self.config['seeds']
        missing = []
        for fl_name, marl_name in self.get_all_combinations():
            for n in swarm_sizes:
                for seed in seeds:
                    key = (fl_name, marl_name, n, seed)
                    if key not in self.results:
                        # Also check disk
                        if self.load_result(fl_name, marl_name, n, seed) is None:
                            missing.append({
                                'fl': fl_name,
                                'marl': marl_name,
                                'num_drones': n,
                                'seed': seed,
                            })
        return missing

    def print_progress_table(self):
        """Print a summary table of experiment completion status."""
        total = (len(FL_ALGORITHMS) * len(MARL_ALGORITHMS)
                 * len(self.config['swarm_sizes'])
                 * len(self.config['seeds']))
        completed = len(self.results)
        pct = 100.0 * completed / total if total > 0 else 0

        print(f"\n{'='*60}")
        print(f"FL-MARL Combination Testing Progress")
        print(f"{'='*60}")
        print(f"  Completed: {completed} / {total} ({pct:.1f}%)")
        print(f"  FL algorithms: {FL_ALGORITHMS}")
        print(f"  MARL algorithms: {MARL_ALGORITHMS}")
        print(f"  Swarm sizes: {self.config['swarm_sizes']}")
        print(f"  Seeds: {self.config['seeds']}")

        # Per-combination count
        print(f"\n  {'FL':15s} {'MARL':10s} {'Done':>5s}/{total // (len(FL_ALGORITHMS)*len(MARL_ALGORITHMS)):>5s}")
        print(f"  {'-'*35}")
        for fl, marl in self.get_all_combinations():
            n_done = len(self.get_combination_results(fl, marl))
            n_total = len(self.config['swarm_sizes']) * len(self.config['seeds'])
            bar = '#' * n_done + '.' * (n_total - n_done)
            print(f"  {fl:15s} {marl:10s} [{bar}] {n_done}/{n_total}")
        print(f"{'='*60}\n")


if __name__ == '__main__':
    """Smoke test for CombinationTester."""
    import tempfile

    print("Testing CombinationTester...\n")

    with tempfile.TemporaryDirectory() as tmpdir:
        tester = CombinationTester(
            config={
                'seeds': [0, 1, 2],
                'swarm_sizes': [5, 10],
                'fl_algorithms': FL_ALGORITHMS,
                'marl_algorithms': MARL_ALGORITHMS,
            },
            results_dir=tmpdir
        )

        # Test 1: Store and retrieve results
        print("Test 1: Store and load results")
        fake_metrics = {
            'avg_episode_reward': 1.5,
            'std_episode_reward': 0.2,
            'avg_average_latency_ms': 45.0,
            'avg_task_success_rate': 0.85,
        }
        tester.store_result('fedadam', 'mappo', 10, 0, fake_metrics)
        tester.store_result('fedadam', 'mappo', 10, 1, {**fake_metrics, 'avg_episode_reward': 1.6})
        tester.store_result('fedavg', 'maddpg', 5, 0, fake_metrics)

        results = tester.get_combination_results('fedadam', 'mappo', 10)
        assert len(results) == 2
        print(f"  \u2713 Stored 3 results, retrieved {len(results)} for fedadam+mappo+10drones")

        # Test 2: Load from disk
        loaded = tester.load_result('fedadam', 'mappo', 10, 0)
        assert loaded is not None
        assert loaded['avg_episode_reward'] == 1.5
        print("  \u2713 Load from disk works")

        # Test 3: Summarize combination
        print("\nTest 2: Summarize combination")
        summary = tester.summarize_combination('fedadam', 'mappo')
        assert 10 in summary
        assert 'mean' in summary[10] and 'std' in summary[10]
        print(f"  10 drones: mean={summary[10]['mean']:.3f}, std={summary[10]['std']:.3f}")
        print("  \u2713 Summarize works")

        # Test 3: Performance matrix
        print("\nTest 3: Performance matrix")
        matrix = tester.build_performance_matrix(num_drones=10)
        assert 'fedadam' in matrix
        assert 'mappo' in matrix['fedadam']
        print(f"  fedadam+mappo: {matrix['fedadam']['mappo']}")
        print("  \u2713 Performance matrix works")

        # Test 4: Missing combinations
        print("\nTest 4: Missing combinations")
        missing = tester.get_missing_combinations()
        assert len(missing) > 0  # Most not filled
        print(f"  {len(missing)} combinations still pending")
        print("  \u2713 Missing detection works")

        # Test 5: Progress table
        print("\nTest 5: Progress table")
        tester.print_progress_table()

    print("\u2705 CombinationTester tests passed!")
