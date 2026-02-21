"""
Comparator — Rank and compare FL-MARL algorithm combinations.

Takes stored results from CombinationTester and produces:
    - Ranked combination tables (best to worst per metric)
    - Statistical significance tests between top combinations
    - Per-metric performance profiles across swarm sizes

Usage:
    from src.evaluation.comparator import Comparator
    from src.evaluation.combination_tester import CombinationTester

    tester = CombinationTester()
    tester.load_all_results()
    comparator = Comparator(tester)
    rankings = comparator.rank_combinations(metric='avg_episode_reward')
"""

import numpy as np
from typing import Dict, List, Optional, Tuple

from src.evaluation.combination_tester import (
    CombinationTester, FL_ALGORITHMS, MARL_ALGORITHMS, SWARM_SIZES
)
from src.evaluation.statistical_tests import compare_distributions


class Comparator:
    """
    Ranks and compares FL-MARL combinations using stored results.

    Produces comprehensive comparison tables, statistical tests, and
    performance profiles useful for identifying the best combination
    and understanding trade-offs.

    Args:
        tester: CombinationTester with loaded results
    """

    def __init__(self, tester: CombinationTester):
        self.tester = tester

    def rank_combinations(
        self,
        metric: str = 'avg_episode_reward',
        num_drones: Optional[int] = None,
        ascending: bool = False
    ) -> List[Dict]:
        """
        Rank all FL-MARL combinations by a given metric.

        Args:
            metric:     Metric to rank by
            num_drones: If None, average across all swarm sizes
            ascending:  If True, lower is better (e.g., latency)

        Returns:
            list: Ranked combinations, each dict contains:
                {'rank', 'fl', 'marl', 'mean', 'std', 'n_results'}
        """
        rankings = []
        for fl_name in FL_ALGORITHMS:
            for marl_name in MARL_ALGORITHMS:
                if num_drones is not None:
                    results = self.tester.get_combination_results(
                        fl_name, marl_name, num_drones
                    )
                    values = [r[metric] for r in results if metric in r]
                else:
                    # Average across all swarm sizes
                    values = []
                    for n in SWARM_SIZES:
                        results = self.tester.get_combination_results(fl_name, marl_name, n)
                        values.extend([r[metric] for r in results if metric in r])

                if values:
                    rankings.append({
                        'fl': fl_name,
                        'marl': marl_name,
                        'mean': float(np.mean(values)),
                        'std': float(np.std(values)),
                        'n_results': len(values),
                    })

        # Sort: highest mean first (or lowest if ascending=True)
        rankings.sort(key=lambda x: x['mean'], reverse=not ascending)

        # Add rank number
        for i, entry in enumerate(rankings):
            entry['rank'] = i + 1

        return rankings

    def get_top_combinations(
        self,
        n: int = 5,
        metric: str = 'avg_episode_reward',
        num_drones: Optional[int] = None
    ) -> List[Dict]:
        """
        Get the top-N FL-MARL combinations.

        Args:
            n:          Number of top combinations to return
            metric:     Ranking metric
            num_drones: Filter by swarm size (None = all)

        Returns:
            list: Top-N ranked combinations
        """
        rankings = self.rank_combinations(metric, num_drones)
        return rankings[:n]

    def pairwise_comparison(
        self,
        fl_a: str, marl_a: str,
        fl_b: str, marl_b: str,
        metric: str = 'avg_episode_reward',
        num_drones: Optional[int] = None
    ) -> Dict:
        """
        Statistical comparison between two FL-MARL combinations.

        Args:
            fl_a, marl_a:   First combination
            fl_b, marl_b:   Second combination
            metric:         Metric to compare
            num_drones:     Swarm size filter

        Returns:
            dict: Full statistical comparison from compare_distributions()
        """
        def get_values(fl, marl):
            if num_drones is not None:
                results = self.tester.get_combination_results(fl, marl, num_drones)
            else:
                results = []
                for n in SWARM_SIZES:
                    results.extend(self.tester.get_combination_results(fl, marl, n))
            return [r[metric] for r in results if metric in r]

        values_a = get_values(fl_a, marl_a)
        values_b = get_values(fl_b, marl_b)

        label_a = f"{fl_a}+{marl_a}"
        label_b = f"{fl_b}+{marl_b}"

        if not values_a or not values_b:
            return {
                'error': 'Insufficient data for comparison',
                'n_a': len(values_a),
                'n_b': len(values_b)
            }

        return compare_distributions(
            values_a, values_b,
            label_a=label_a, label_b=label_b
        )

    def compare_fl_algorithms(
        self,
        marl_name: str,
        metric: str = 'avg_episode_reward',
        num_drones: Optional[int] = None
    ) -> Dict:
        """
        Compare all 4 FL algorithms for a fixed MARL algorithm.

        Useful for understanding which FL algorithm works best with a
        specific MARL approach.

        Args:
            marl_name:  MARL algorithm to fix
            metric:     Metric to compare
            num_drones: Swarm size filter

        Returns:
            dict: {fl_name: {'mean', 'std', 'rank'}}
        """
        fl_stats = {}
        for fl_name in FL_ALGORITHMS:
            if num_drones is not None:
                results = self.tester.get_combination_results(fl_name, marl_name, num_drones)
            else:
                results = []
                for n in SWARM_SIZES:
                    results.extend(self.tester.get_combination_results(fl_name, marl_name, n))
            values = [r[metric] for r in results if metric in r]
            fl_stats[fl_name] = {
                'mean': float(np.mean(values)) if values else None,
                'std': float(np.std(values)) if values else None,
                'n': len(values),
            }

        # Rank FL algorithms
        ranked = sorted(
            [(k, v) for k, v in fl_stats.items() if v['mean'] is not None],
            key=lambda x: x[1]['mean'], reverse=True
        )
        for rank_i, (fl_name, _) in enumerate(ranked):
            fl_stats[fl_name]['rank'] = rank_i + 1

        return fl_stats

    def compare_marl_algorithms(
        self,
        fl_name: str,
        metric: str = 'avg_episode_reward',
        num_drones: Optional[int] = None
    ) -> Dict:
        """
        Compare all 3 MARL algorithms for a fixed FL algorithm.

        Args:
            fl_name:    FL algorithm to fix
            metric:     Metric to compare
            num_drones: Swarm size filter

        Returns:
            dict: {marl_name: {'mean', 'std', 'rank'}}
        """
        marl_stats = {}
        for marl_name in MARL_ALGORITHMS:
            if num_drones is not None:
                results = self.tester.get_combination_results(fl_name, marl_name, num_drones)
            else:
                results = []
                for n in SWARM_SIZES:
                    results.extend(self.tester.get_combination_results(fl_name, marl_name, n))
            values = [r[metric] for r in results if metric in r]
            marl_stats[marl_name] = {
                'mean': float(np.mean(values)) if values else None,
                'std': float(np.std(values)) if values else None,
                'n': len(values),
            }

        ranked = sorted(
            [(k, v) for k, v in marl_stats.items() if v['mean'] is not None],
            key=lambda x: x[1]['mean'], reverse=True
        )
        for rank_i, (marl_name, _) in enumerate(ranked):
            marl_stats[marl_name]['rank'] = rank_i + 1

        return marl_stats

    def scaling_analysis(
        self,
        fl_name: str,
        marl_name: str,
        metric: str = 'avg_episode_reward'
    ) -> Dict:
        """
        Analyse how a combination performs as swarm size increases.

        Args:
            fl_name, marl_name: Combination to analyse
            metric:             Metric to track

        Returns:
            dict: {num_drones: {'mean', 'std', 'n'}}
        """
        scaling = {}
        for n in SWARM_SIZES:
            results = self.tester.get_combination_results(fl_name, marl_name, n)
            values = [r[metric] for r in results if metric in r]
            scaling[n] = {
                'mean': float(np.mean(values)) if values else None,
                'std': float(np.std(values)) if values else None,
                'n': len(values),
            }
        return scaling

    def print_ranking_table(
        self,
        metric: str = 'avg_episode_reward',
        num_drones: Optional[int] = None
    ):
        """Print a formatted ranking table to stdout."""
        scope = f"(all sizes)" if num_drones is None else f"({num_drones} drones)"
        print(f"\nFL-MARL Rankings by '{metric}' {scope}")
        print(f"{'Rank':>4}  {'FL':15s}  {'MARL':10s}  {'Mean':>10s}  {'Std':>8s}  {'N':>5s}")
        print(f"{'='*56}")
        rankings = self.rank_combinations(metric, num_drones)
        for r in rankings:
            mean_str = f"{r['mean']:.4f}" if r['mean'] is not None else "N/A"
            std_str = f"{r['std']:.4f}" if r['std'] is not None else "N/A"
            print(f"  {r['rank']:2d}   {r['fl']:15s}  {r['marl']:10s}  "
                  f"{mean_str:>10s}  {std_str:>8s}  {r['n_results']:>5d}")
        print(f"{'='*56}\n")


if __name__ == '__main__':
    """Smoke test for Comparator."""
    import tempfile
    import sys
    sys.path.insert(0, '/home/skyvision/HFL-UAV-Swarm-Comparison')

    print("Testing Comparator...\n")

    with tempfile.TemporaryDirectory() as tmpdir:
        tester = CombinationTester(
            config={'seeds': [0, 1, 2], 'swarm_sizes': [5, 10]},
            results_dir=tmpdir
        )

        # Populate with fake results
        base_rewards = {
            ('fedadam', 'mappo'):  1.8,
            ('fedavg', 'mappo'):   1.5,
            ('fedprox', 'maddpg'): 1.3,
            ('hierarchical', 'qmix'): 1.1,
        }
        for (fl, marl), base in base_rewards.items():
            for n in [5, 10]:
                for seed in [0, 1, 2]:
                    tester.store_result(fl, marl, n, seed, {
                        'avg_episode_reward': base + np.random.randn() * 0.05,
                        'avg_average_latency_ms': 50.0 - base * 5,
                        'avg_task_success_rate': base / 2.0,
                    })

        comp = Comparator(tester)

        print("Test 1: Ranking combinations")
        rankings = comp.rank_combinations('avg_episode_reward')
        assert len(rankings) > 0
        assert rankings[0]['rank'] == 1
        print(f"  Top combo: {rankings[0]['fl']}+{rankings[0]['marl']} "
              f"(mean={rankings[0]['mean']:.3f})")
        print("  \u2713 Ranking works")

        print("\nTest 2: Top-N")
        top3 = comp.get_top_combinations(n=3)
        assert len(top3) <= 3
        print(f"  Top 3 combos: {[(r['fl'], r['marl']) for r in top3]}")
        print("  \u2713 Top-N works")

        print("\nTest 3: Pairwise comparison")
        pcomp = comp.pairwise_comparison('fedadam', 'mappo', 'fedavg', 'mappo')
        if 'error' not in pcomp:
            print(f"  Winner: {pcomp['winner']}, d={pcomp['cohens_d']:.2f}")
        print("  \u2713 Pairwise comparison works")

        print("\nTest 4: FL algorithm comparison")
        fl_comp = comp.compare_fl_algorithms('mappo')
        assert 'fedadam' in fl_comp
        print(f"  FL rankings for MAPPO: {[(k, v['rank']) for k, v in fl_comp.items() if 'rank' in v]}")
        print("  \u2713 FL comparison works")

        print("\nTest 5: Scaling analysis")
        scaling = comp.scaling_analysis('fedadam', 'mappo')
        assert 5 in scaling and 10 in scaling
        print(f"  5 drones: {scaling[5]['mean']:.3f}, 10 drones: {scaling[10]['mean']:.3f}")
        print("  \u2713 Scaling analysis works")

        comp.print_ranking_table()

    print("\u2705 Comparator tests passed!")
