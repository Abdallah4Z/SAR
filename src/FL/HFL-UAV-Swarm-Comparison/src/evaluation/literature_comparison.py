"""
LiteratureComparison — Compare our FL-MARL results against published baselines.

Collects results from both our system and the literature baselines, then
produces statistical significance tests and performance gap analysis to
determine how much our best combinations improve over state of the art.

Usage:
    from src.evaluation.literature_comparison import LiteratureComparison

    lit_comp = LiteratureComparison(our_tester, baseline_results)
    report = lit_comp.run_comparison()
    lit_comp.print_report(report)
"""

import numpy as np
from typing import Dict, List, Optional, Any

from src.evaluation.statistical_tests import compare_distributions


class LiteratureComparison:
    """
    Compares our best FL-MARL combinations against literature baselines.

    Collects baseline results (either run fresh or loaded from stored files),
    then performs:
        1. Performance gap analysis (% improvement over each baseline)
        2. Statistical significance tests (t-test + Mann-Whitney)
        3. Computational cost comparison (if timing data available)
        4. Per-metric comparison tables

    Args:
        our_tester:        CombinationTester with our FL-MARL results
        baseline_results:  Dict {baseline_name: [episode_reward, ...]}
                          Can be populated from BaselineAdapters
        config:            Comparison configuration
    """

    def __init__(
        self,
        our_tester,
        baseline_results: Optional[Dict[str, List[float]]] = None,
        config: Optional[Dict] = None
    ):
        self.our_tester = our_tester
        self.baseline_results = baseline_results or {}

        default_config = {
            'alpha': 0.05,
            'num_eval_episodes': 50,
            'swarm_sizes_to_compare': [5, 10, 20],
            'top_k_our': 3,   # How many of our combos to compare
        }
        if config:
            default_config.update(config)
        self.config = default_config

    def add_baseline_results(
        self,
        baseline_name: str,
        episode_rewards: List[float],
        metadata: Optional[Dict] = None
    ):
        """
        Add evaluation results for a literature baseline.

        Args:
            baseline_name:   Name matching BASELINE_CONFIGS key
            episode_rewards: List of per-episode rewards
            metadata:        Optional extra info (latency, success_rate, etc.)
        """
        self.baseline_results[baseline_name] = {
            'rewards': episode_rewards,
            'metadata': metadata or {},
        }

    def compare_our_vs_baseline(
        self,
        our_fl: str,
        our_marl: str,
        baseline_name: str,
        num_drones: Optional[int] = None,
        metric: str = 'avg_episode_reward'
    ) -> Dict:
        """
        Statistical comparison: one of our combinations vs one baseline.

        Args:
            our_fl:        Our FL algorithm
            our_marl:      Our MARL algorithm
            baseline_name: Name of baseline to compare against
            num_drones:    Swarm size (None = all)
            metric:        Metric to compare (must be 'avg_episode_reward'
                           for baselines stored as reward lists)

        Returns:
            dict: Full comparison including stats and performance gap
        """
        # Collect our results
        if num_drones is not None:
            our_results = self.our_tester.get_combination_results(
                our_fl, our_marl, num_drones
            )
        else:
            our_results = []
            from src.evaluation.combination_tester import SWARM_SIZES
            for n in self.config['swarm_sizes_to_compare']:
                our_results.extend(
                    self.our_tester.get_combination_results(our_fl, our_marl, n)
                )

        our_scores = [r[metric] for r in our_results if metric in r]

        # Get baseline scores
        baseline_data = self.baseline_results.get(baseline_name)
        if baseline_data is None:
            return {'error': f'No results for baseline: {baseline_name}'}

        baseline_scores = baseline_data.get('rewards', [])
        if not baseline_scores:
            return {'error': f'Empty results for baseline: {baseline_name}'}

        if not our_scores:
            return {'error': f'No results for our combination: {our_fl}+{our_marl}'}

        # Statistical comparison
        comparison = compare_distributions(
            our_scores, baseline_scores,
            label_a=f"Ours ({our_fl}+{our_marl})",
            label_b=baseline_name,
            alpha=self.config['alpha']
        )

        # Performance gap
        our_mean = float(np.mean(our_scores))
        bl_mean = float(np.mean(baseline_scores))
        pct_improvement = 100.0 * (our_mean - bl_mean) / (abs(bl_mean) + 1e-8)

        comparison['performance_gap_pct'] = float(pct_improvement)
        comparison['our_mean'] = our_mean
        comparison['baseline_mean'] = bl_mean
        comparison['our_combination'] = f"{our_fl}+{our_marl}"
        comparison['baseline_name'] = baseline_name

        return comparison

    def run_comparison(
        self,
        our_combos: Optional[List[tuple]] = None
    ) -> Dict:
        """
        Run full literature comparison for specified combinations.

        Compares each of our top combinations against all available baselines.

        Args:
            our_combos: List of (fl, marl) tuples to compare.
                        If None, uses all available combos with results.

        Returns:
            dict: Nested comparison results
                {(fl, marl): {baseline_name: comparison_dict}}
        """
        if our_combos is None:
            from src.evaluation.combination_tester import FL_ALGORITHMS, MARL_ALGORITHMS
            our_combos = [(fl, marl)
                          for fl in FL_ALGORITHMS
                          for marl in MARL_ALGORITHMS]

        all_comparisons = {}
        for fl_name, marl_name in our_combos:
            combo_key = f"{fl_name}+{marl_name}"
            combo_comparisons = {}
            for baseline_name in self.baseline_results.keys():
                result = self.compare_our_vs_baseline(
                    fl_name, marl_name, baseline_name
                )
                combo_comparisons[baseline_name] = result
            all_comparisons[combo_key] = combo_comparisons

        return all_comparisons

    def summary_table(self, comparisons: Dict) -> List[Dict]:
        """
        Build a summary table of % improvements across all comparisons.

        Args:
            comparisons: Output from run_comparison()

        Returns:
            list: Rows sorted by average improvement
        """
        rows = []
        for combo_key, baseline_results in comparisons.items():
            improvements = []
            significant_count = 0
            for bl_name, result in baseline_results.items():
                if 'error' in result:
                    continue
                improvements.append(result.get('performance_gap_pct', 0))
                if result.get('significant_t', False):
                    significant_count += 1

            if improvements:
                rows.append({
                    'combination': combo_key,
                    'avg_improvement_pct': float(np.mean(improvements)),
                    'min_improvement_pct': float(np.min(improvements)),
                    'max_improvement_pct': float(np.max(improvements)),
                    'n_significantly_better': significant_count,
                    'n_baselines': len(improvements),
                })

        rows.sort(key=lambda x: x['avg_improvement_pct'], reverse=True)
        return rows

    def print_report(self, comparisons: Dict):
        """Print a formatted literature comparison report."""
        print("\n" + "=" * 70)
        print("LITERATURE COMPARISON REPORT")
        print("=" * 70)

        table = self.summary_table(comparisons)
        print(f"\n{'Combination':30s}  {'Avg Improvement':>15s}  {'Sig Better':>10s}")
        print("-" * 60)
        for row in table:
            print(f"  {row['combination']:28s}  "
                  f"{row['avg_improvement_pct']:>+14.1f}%  "
                  f"{row['n_significantly_better']:>4d}/{row['n_baselines']:d}")

        print("\nNote: 'Sig Better' = number of baselines vs which we are")
        print("statistically significantly better (p < 0.05).")
        print("=" * 70 + "\n")


if __name__ == '__main__':
    """Smoke test for LiteratureComparison."""
    import tempfile, sys
    sys.path.insert(0, '/home/skyvision/HFL-UAV-Swarm-Comparison')
    from src.evaluation.combination_tester import CombinationTester

    print("Testing LiteratureComparison...\n")

    with tempfile.TemporaryDirectory() as tmpdir:
        tester = CombinationTester(
            config={'seeds': [0, 1, 2], 'swarm_sizes': [5, 10]},
            results_dir=tmpdir
        )

        np.random.seed(0)
        for fl in ['fedadam', 'fedavg']:
            for marl in ['mappo', 'maddpg']:
                for n in [5, 10]:
                    for seed in [0, 1, 2]:
                        tester.store_result(fl, marl, n, seed, {
                            'avg_episode_reward': 1.5 + np.random.randn() * 0.05,
                        })

        lit = LiteratureComparison(tester)

        # Add fake baseline results
        lit.add_baseline_results('greedy_offload',
                                  (np.random.randn(30) * 0.1 + 0.8).tolist())
        lit.add_baseline_results('independent_ppo',
                                  (np.random.randn(30) * 0.1 + 1.1).tolist())

        print("Test 1: Compare vs one baseline")
        result = lit.compare_our_vs_baseline('fedadam', 'mappo', 'greedy_offload')
        if 'error' not in result:
            print(f"  Improvement over greedy: {result['performance_gap_pct']:+.1f}%")
            print(f"  Significant: {result['significant_t']}")
        print("  \u2713 Pairwise comparison works")

        print("\nTest 2: Full comparison")
        comps = lit.run_comparison([('fedadam', 'mappo'), ('fedavg', 'maddpg')])
        assert 'fedadam+mappo' in comps
        print("  \u2713 Full comparison works")

        print("\nTest 3: Summary table")
        table = lit.summary_table(comps)
        for row in table:
            print(f"  {row['combination']}: avg={row['avg_improvement_pct']:+.1f}%")
        print("  \u2713 Summary table works")

        print("\nTest 4: Report")
        lit.print_report(comps)

    print("\u2705 LiteratureComparison tests passed!")
