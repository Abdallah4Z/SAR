"""
SelectionFramework — Identify top-performing FL-MARL combinations.

Analyses the full combination matrix to select the 3–5 best-performing
combinations using multi-criteria scoring, considering:
    - Average reward across seeds and swarm sizes
    - Task success rate and latency
    - Fairness (load distribution across UAVs)
    - Scalability (performance degradation from 5→100 drones)
    - Computational efficiency (convergence speed)

Usage:
    from src.evaluation.selection_framework import SelectionFramework

    sf = SelectionFramework(tester, config={'top_k': 5})
    top_combos = sf.select_top_combinations()
    sf.print_selection_report(top_combos)
"""

import numpy as np
from typing import Dict, List, Optional, Tuple

from src.evaluation.combination_tester import (
    CombinationTester, FL_ALGORITHMS, MARL_ALGORITHMS, SWARM_SIZES
)


class SelectionFramework:
    """
    Multi-criteria selection of best FL-MARL combinations.

    Scores combinations on multiple metrics and uses weighted aggregation
    to find combinations that excel across all evaluation criteria and
    all swarm sizes.

    Args:
        tester: CombinationTester with loaded results
        config: Selection configuration
    """

    # Default metric weights (higher = more important)
    DEFAULT_WEIGHTS = {
        'avg_episode_reward':      0.35,   # Primary performance metric
        'avg_task_success_rate':   0.25,   # Task completion quality
        'avg_average_latency_ms':  0.20,   # Latency (lower is better)
        'avg_fairness_cv':         0.10,   # Fairness (lower CV is better)
        'scalability_score':       0.10,   # Performance robustness across sizes
    }

    # Metrics where lower is better
    LOWER_IS_BETTER = {'avg_average_latency_ms', 'avg_fairness_cv'}

    def __init__(
        self,
        tester: CombinationTester,
        config: Optional[Dict] = None
    ):
        self.tester = tester
        default_config = {
            'top_k': 5,
            'metric_weights': self.DEFAULT_WEIGHTS.copy(),
            'swarm_sizes': SWARM_SIZES,
            'min_results_required': 3,   # Minimum seeds to include a combo
        }
        if config:
            default_config.update(config)
        self.config = default_config
        self.weights = self.config['metric_weights']

    def score_combination(
        self,
        fl_name: str,
        marl_name: str
    ) -> Optional[Dict]:
        """
        Compute a multi-criteria composite score for one combination.

        Collects results for all swarm sizes, normalizes each metric,
        applies weights, and returns a composite score in [0, 1].

        Args:
            fl_name:   FL algorithm name
            marl_name: MARL algorithm name

        Returns:
            dict with raw metrics and composite score, or None if insufficient data
        """
        # Collect raw metric values across all sizes
        raw = {}
        for n in self.config['swarm_sizes']:
            results = self.tester.get_combination_results(fl_name, marl_name, n)
            for r in results:
                for metric in self.weights.keys():
                    if metric == 'scalability_score':
                        continue
                    if metric in r:
                        raw.setdefault(metric, []).append(r[metric])

        if not raw:
            return None
        min_n = min(len(v) for v in raw.values())
        if min_n < self.config['min_results_required']:
            return None

        # Compute scalability score: mean reward at largest vs smallest size
        scale_small = self.tester.get_combination_results(
            fl_name, marl_name, self.config['swarm_sizes'][0]
        )
        scale_large = self.tester.get_combination_results(
            fl_name, marl_name, self.config['swarm_sizes'][-1]
        )
        reward_small = [r['avg_episode_reward'] for r in scale_small
                        if 'avg_episode_reward' in r]
        reward_large = [r['avg_episode_reward'] for r in scale_large
                        if 'avg_episode_reward' in r]

        if reward_small and reward_large:
            # Higher ratio = better scalability (closer to 1.0)
            scalability = np.mean(reward_large) / (np.mean(reward_small) + 1e-8)
        else:
            scalability = 0.5  # unknown

        return {
            'fl': fl_name,
            'marl': marl_name,
            'raw_metrics': {k: {'mean': float(np.mean(v)), 'std': float(np.std(v))}
                            for k, v in raw.items()},
            'scalability_score_raw': float(scalability),
            'n_results': min_n,
        }

    def normalize_scores(self, scored_combos: List[Dict]) -> List[Dict]:
        """
        Normalize each metric across all combinations to [0, 1].

        For metrics where lower is better, invert so that 1 = best.

        Args:
            scored_combos: List of outputs from score_combination()

        Returns:
            Same list with 'normalized' field added to each entry
        """
        # Collect all metric values across combos
        all_metric_values = {}
        for combo in scored_combos:
            for metric_key, stats in combo['raw_metrics'].items():
                all_metric_values.setdefault(metric_key, []).append(stats['mean'])
            if 'scalability_score_raw' in combo:
                all_metric_values.setdefault('scalability_score', []).append(
                    combo['scalability_score_raw']
                )

        # Compute min/max for normalization
        metric_min = {k: min(v) for k, v in all_metric_values.items()}
        metric_max = {k: max(v) for k, v in all_metric_values.items()}

        # Normalize each combination
        for combo in scored_combos:
            normalized = {}
            for metric_key in self.weights.keys():
                if metric_key == 'scalability_score':
                    raw_val = combo.get('scalability_score_raw', 0.5)
                elif metric_key in combo['raw_metrics']:
                    raw_val = combo['raw_metrics'][metric_key]['mean']
                else:
                    normalized[metric_key] = 0.0
                    continue

                mn = metric_min.get(metric_key, raw_val)
                mx = metric_max.get(metric_key, raw_val)
                if mx == mn:
                    norm_val = 0.5
                else:
                    norm_val = (raw_val - mn) / (mx - mn)

                # Invert if lower is better
                if metric_key in self.LOWER_IS_BETTER:
                    norm_val = 1.0 - norm_val

                normalized[metric_key] = float(np.clip(norm_val, 0, 1))

            combo['normalized'] = normalized

            # Compute composite score
            composite = sum(
                self.weights.get(k, 0) * normalized.get(k, 0)
                for k in self.weights
            )
            combo['composite_score'] = float(composite)

        return scored_combos

    def select_top_combinations(self) -> List[Dict]:
        """
        Select the top-K FL-MARL combinations by composite score.

        Returns:
            list: Top-K combinations sorted by composite_score (highest first),
                  each with raw metrics, normalized scores, and composite score
        """
        scored = []
        for fl_name in FL_ALGORITHMS:
            for marl_name in MARL_ALGORITHMS:
                result = self.score_combination(fl_name, marl_name)
                if result is not None:
                    scored.append(result)

        if not scored:
            return []

        scored = self.normalize_scores(scored)
        scored.sort(key=lambda x: x['composite_score'], reverse=True)

        # Add rank
        for i, combo in enumerate(scored):
            combo['rank'] = i + 1

        return scored[:self.config['top_k']]

    def get_best_for_swarm_size(
        self,
        num_drones: int,
        metric: str = 'avg_episode_reward'
    ) -> Optional[Dict]:
        """
        Find the best FL-MARL combination for a specific swarm size.

        Args:
            num_drones: Target swarm size
            metric:     Metric to optimise

        Returns:
            dict: Best combination info, or None if no data
        """
        best = None
        best_val = -np.inf

        for fl_name in FL_ALGORITHMS:
            for marl_name in MARL_ALGORITHMS:
                results = self.tester.get_combination_results(fl_name, marl_name, num_drones)
                values = [r[metric] for r in results if metric in r]
                if values:
                    mean_val = np.mean(values)
                    if mean_val > best_val:
                        best_val = mean_val
                        best = {
                            'fl': fl_name,
                            'marl': marl_name,
                            'mean_value': float(mean_val),
                            'metric': metric,
                            'num_drones': num_drones,
                        }
        return best

    def generate_recommendations(self, top_combos: List[Dict]) -> str:
        """
        Generate a text recommendations report for the top combinations.

        Args:
            top_combos: Output from select_top_combinations()

        Returns:
            str: Formatted recommendations text
        """
        if not top_combos:
            return "No results available yet. Run experiments first."

        lines = [
            "=" * 60,
            "FL-MARL COMBINATION RECOMMENDATIONS",
            "=" * 60,
            "",
        ]

        for combo in top_combos:
            fl = combo['fl'].upper()
            marl = combo['marl'].upper()
            score = combo.get('composite_score', 0)
            lines.append(f"#{combo['rank']}: {fl} + {marl} (composite score: {score:.3f})")

            # Key strengths
            norm = combo.get('normalized', {})
            strengths = sorted(norm.items(), key=lambda x: x[1], reverse=True)
            lines.append("  Strengths:")
            for metric, val in strengths[:3]:
                lines.append(f"    - {metric.replace('avg_', '')}: {val:.2f}/1.00")

            # Scalability
            sc = combo.get('scalability_score_raw', None)
            if sc is not None:
                label = "good" if sc > 0.8 else "moderate" if sc > 0.5 else "poor"
                lines.append(f"  Scalability: {label} ({sc:.2f})")
            lines.append("")

        lines.extend([
            "=" * 60,
            "USAGE GUIDELINES:",
            "  - Small swarms (5-10): prioritise communication efficiency -> HFL",
            "  - Large swarms (50-100): adaptive LR helps -> FedAdam",
            "  - Heterogeneous tasks: proximal term helps -> FedProx",
            "  - Homogeneous tasks: simplest choice -> FedAvg",
            "=" * 60,
        ])

        return "\n".join(lines)

    def print_selection_report(self, top_combos: List[Dict]):
        """Print the selection report to stdout."""
        print(self.generate_recommendations(top_combos))


if __name__ == '__main__':
    """Smoke test for SelectionFramework."""
    import tempfile
    import sys
    sys.path.insert(0, '/home/skyvision/HFL-UAV-Swarm-Comparison')

    print("Testing SelectionFramework...\n")

    with tempfile.TemporaryDirectory() as tmpdir:
        tester = CombinationTester(
            config={'seeds': [0, 1, 2], 'swarm_sizes': [5, 10, 20]},
            results_dir=tmpdir
        )

        # Populate fake results
        np.random.seed(42)
        combos = [(fl, marl)
                  for fl in FL_ALGORITHMS
                  for marl in MARL_ALGORITHMS]
        base_rewards = {c: 1.0 + np.random.rand() * 0.8 for c in combos}

        for (fl, marl), base in base_rewards.items():
            for n in [5, 10, 20]:
                for seed in [0, 1, 2]:
                    tester.store_result(fl, marl, n, seed, {
                        'avg_episode_reward': base + np.random.randn() * 0.05,
                        'avg_task_success_rate': min(1.0, base / 2.0 + 0.1),
                        'avg_average_latency_ms': max(10, 100 - base * 30),
                        'avg_fairness_cv': max(0, 0.5 - base * 0.1),
                    })

        sf = SelectionFramework(tester, config={'top_k': 3})

        print("Test 1: Score individual combination")
        score = sf.score_combination('fedadam', 'mappo')
        assert score is not None
        print(f"  fedadam+mappo score result: n_results={score['n_results']}")
        print("  \u2713 Scoring works")

        print("\nTest 2: Select top combinations")
        top = sf.select_top_combinations()
        assert len(top) <= 3
        print(f"  Top combinations (up to 3):")
        for c in top:
            print(f"    #{c['rank']}: {c['fl']}+{c['marl']} "
                  f"composite={c['composite_score']:.3f}")
        print("  \u2713 Selection works")

        print("\nTest 3: Best for specific swarm size")
        best = sf.get_best_for_swarm_size(10)
        if best:
            print(f"  Best for 10 drones: {best['fl']}+{best['marl']} "
                  f"(reward={best['mean_value']:.3f})")
        print("  \u2713 Best-per-size works")

        print("\nTest 4: Recommendations report")
        report = sf.generate_recommendations(top)
        assert 'RECOMMENDATIONS' in report
        print("  \u2713 Recommendations report generated")
        sf.print_selection_report(top)

    print("\n\u2705 SelectionFramework tests passed!")
