"""
plot_comparison.py — Algorithm comparison bar/box plots.

Generates side-by-side comparison plots for FL-MARL combinations across
multiple metrics (reward, latency, success rate, fairness).

Usage:
    from src.visualization.plot_comparison import ComparisonPlotter
    plotter = ComparisonPlotter()
    fig = plotter.plot_bar_comparison(rankings, metric='avg_episode_reward')
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from typing import Dict, List, Optional, Tuple

from src.evaluation.combination_tester import (
    CombinationTester, FL_ALGORITHMS, MARL_ALGORITHMS, SWARM_SIZES
)

FL_COLORS = {
    'fedavg': '#1f77b4', 'fedprox': '#ff7f0e',
    'fedadam': '#2ca02c', 'hierarchical': '#d62728',
}


class ComparisonPlotter:
    """
    Generates comparison bar charts and box plots for FL-MARL results.

    Args:
        figsize: Default figure size
        dpi:     Save DPI
    """

    def __init__(self, figsize=(12, 6), dpi=150):
        self.figsize = figsize
        self.dpi = dpi

    def plot_bar_comparison(
        self,
        rankings: List[Dict],
        metric_label: str = 'Average Episode Reward',
        title: str = '',
        top_k: int = 12
    ) -> plt.Figure:
        """
        Horizontal bar chart of combination rankings.

        Args:
            rankings:     Output from Comparator.rank_combinations()
            metric_label: Y-axis label
            title:        Plot title
            top_k:        Show top-K entries only

        Returns:
            plt.Figure
        """
        data = rankings[:top_k]
        if not data:
            fig, ax = plt.subplots(figsize=self.figsize)
            ax.text(0.5, 0.5, 'No data', transform=ax.transAxes, ha='center')
            return fig

        labels = [f"{r['fl'].upper()}+{r['marl'].upper()}" for r in data]
        means = [r['mean'] for r in data]
        stds = [r['std'] for r in data]
        colors = [FL_COLORS.get(r['fl'], '#7f7f7f') for r in data]

        fig, ax = plt.subplots(figsize=self.figsize)
        y_pos = range(len(labels))
        bars = ax.barh(y_pos, means, xerr=stds, align='center',
                       color=colors, alpha=0.85, edgecolor='white',
                       ecolor='gray', capsize=4, height=0.65)

        ax.set_yticks(list(y_pos))
        ax.set_yticklabels(labels, fontsize=10)
        ax.set_xlabel(metric_label, fontsize=12)
        ax.set_title(title or f'FL-MARL Comparison: {metric_label}', fontsize=13)
        ax.invert_yaxis()
        ax.grid(True, axis='x', alpha=0.4)

        # Legend patches for FL algorithms
        patches = [mpatches.Patch(color=FL_COLORS[fl], label=fl.upper())
                   for fl in FL_ALGORITHMS if fl in FL_COLORS]
        ax.legend(handles=patches, fontsize=9, loc='lower right', title='FL Algorithm')

        fig.tight_layout()
        return fig

    def plot_multi_metric_bars(
        self,
        tester: CombinationTester,
        combinations: List[Tuple[str, str]],
        metrics: Optional[List[str]] = None,
        num_drones: int = 10
    ) -> plt.Figure:
        """
        Multi-panel bar chart showing several metrics for each combination.

        Args:
            tester:       CombinationTester
            combinations: (fl, marl) list
            metrics:      Metric keys to show
            num_drones:   Swarm size filter

        Returns:
            plt.Figure
        """
        if metrics is None:
            metrics = [
                'avg_episode_reward',
                'avg_task_success_rate',
                'avg_average_latency_ms',
                'avg_fairness_cv',
            ]

        n_metrics = len(metrics)
        fig, axes = plt.subplots(1, n_metrics, figsize=(n_metrics * 4, 5))
        if n_metrics == 1:
            axes = [axes]

        labels = [f"{fl.upper()}\n+{marl.upper()}" for fl, marl in combinations]
        colors = [FL_COLORS.get(fl, 'gray') for fl, _ in combinations]

        for ax, metric in zip(axes, metrics):
            means = []
            stds = []
            for fl, marl in combinations:
                results = tester.get_combination_results(fl, marl, num_drones)
                vals = [r[metric] for r in results if metric in r]
                means.append(np.mean(vals) if vals else 0)
                stds.append(np.std(vals) if vals else 0)

            x = range(len(labels))
            ax.bar(x, means, yerr=stds, color=colors, alpha=0.8,
                   edgecolor='white', capsize=4)
            ax.set_xticks(list(x))
            ax.set_xticklabels(labels, fontsize=7, rotation=30, ha='right')
            ax.set_title(metric.replace('avg_', '').replace('_', ' ').title(),
                         fontsize=10)
            ax.grid(True, axis='y', alpha=0.3)

        fig.suptitle(f'Multi-Metric Comparison ({num_drones} drones)', fontsize=13)
        fig.tight_layout()
        return fig

    def plot_box_comparison(
        self,
        tester: CombinationTester,
        fl_name: str,
        metric: str = 'avg_episode_reward',
        num_drones: int = 10
    ) -> plt.Figure:
        """
        Box plots comparing MARL algorithms for a fixed FL method.

        Args:
            tester:     CombinationTester
            fl_name:    FL algorithm to fix
            metric:     Metric to compare
            num_drones: Swarm size

        Returns:
            plt.Figure
        """
        fig, ax = plt.subplots(figsize=(8, 5))

        data_per_marl = []
        labels = []
        for marl in MARL_ALGORITHMS:
            results = tester.get_combination_results(fl_name, marl, num_drones)
            vals = [r[metric] for r in results if metric in r]
            if vals:
                data_per_marl.append(vals)
                labels.append(marl.upper())

        if data_per_marl:
            bp = ax.boxplot(data_per_marl, patch_artist=True, notch=False,
                            labels=labels, widths=0.5)
            colors_list = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
            for patch, color in zip(bp['boxes'], colors_list):
                patch.set_facecolor(color)
                patch.set_alpha(0.7)

        ax.set_xlabel('MARL Algorithm', fontsize=12)
        ax.set_ylabel(metric.replace('avg_', '').replace('_', ' ').title(), fontsize=12)
        ax.set_title(f'{fl_name.upper()} — MARL Algorithm Comparison ({num_drones} drones)',
                     fontsize=13)
        ax.grid(True, axis='y', alpha=0.4)
        fig.tight_layout()
        return fig

    def save_figure(self, fig, path):
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        fig.savefig(path, dpi=self.dpi, bbox_inches='tight')
        print(f"  Figure saved: {path}")


if __name__ == '__main__':
    """Smoke test for ComparisonPlotter."""
    import tempfile, sys
    sys.path.insert(0, '/home/skyvision/HFL-UAV-Swarm-Comparison')
    from src.evaluation.combination_tester import CombinationTester
    from src.evaluation.comparator import Comparator

    print("Testing ComparisonPlotter...\n")

    with tempfile.TemporaryDirectory() as tmpdir:
        tester = CombinationTester(
            config={'seeds': [0, 1, 2], 'swarm_sizes': [5, 10]},
            results_dir=tmpdir
        )
        np.random.seed(42)
        for fl in FL_ALGORITHMS:
            for marl in MARL_ALGORITHMS:
                for n in [5, 10]:
                    for seed in [0, 1, 2]:
                        tester.store_result(fl, marl, n, seed, {
                            'avg_episode_reward': np.random.rand() * 0.5 + 1.0,
                            'avg_task_success_rate': np.random.rand() * 0.3 + 0.6,
                            'avg_average_latency_ms': np.random.rand() * 20 + 30,
                            'avg_fairness_cv': np.random.rand() * 0.2,
                        })

        comparator = Comparator(tester)
        plotter = ComparisonPlotter()

        print("Test 1: Bar comparison")
        rankings = comparator.rank_combinations('avg_episode_reward')
        fig = plotter.plot_bar_comparison(rankings)
        assert fig is not None
        print("  \u2713 Bar comparison generated")

        print("Test 2: Multi-metric bars")
        combos = [('fedadam', 'mappo'), ('fedavg', 'maddpg'), ('hierarchical', 'qmix')]
        fig2 = plotter.plot_multi_metric_bars(tester, combos)
        assert fig2 is not None
        print("  \u2713 Multi-metric bars generated")

        print("Test 3: Box comparison")
        fig3 = plotter.plot_box_comparison(tester, 'fedadam')
        assert fig3 is not None
        print("  \u2713 Box comparison generated")

        plt.close('all')

    print("\n\u2705 ComparisonPlotter tests passed!")
