"""
plot_scaling.py — Scaling analysis visualizations (5 to 100 drones).

Generates plots showing how FL-MARL combination performance changes as
the swarm size increases from 5 to 100 drones.

Usage:
    from src.visualization.plot_scaling import ScalingPlotter
    plotter = ScalingPlotter()
    fig = plotter.plot_scaling_curves(tester, metric='avg_episode_reward')
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, List, Optional, Tuple

from src.evaluation.combination_tester import (
    CombinationTester, FL_ALGORITHMS, MARL_ALGORITHMS, SWARM_SIZES
)

FL_COLORS = {
    'fedavg': '#1f77b4', 'fedprox': '#ff7f0e',
    'fedadam': '#2ca02c', 'hierarchical': '#d62728',
}
MARL_MARKERS = {'mappo': 'o', 'maddpg': 's', 'qmix': '^'}


class ScalingPlotter:
    """
    Generates scaling analysis plots.

    Args:
        figsize: Default figure size
        dpi:     DPI for saved figures
    """

    def __init__(self, figsize=(12, 7), dpi=150):
        self.figsize = figsize
        self.dpi = dpi

    def plot_scaling_curves(
        self,
        tester: CombinationTester,
        metric: str = 'avg_episode_reward',
        combinations: Optional[List[Tuple[str, str]]] = None,
        swarm_sizes: Optional[List[int]] = None
    ) -> plt.Figure:
        """
        Plot performance vs swarm size for multiple combinations.

        Args:
            tester:       CombinationTester with loaded results
            metric:       Metric to plot on y-axis
            combinations: List of (fl, marl) tuples. None = all.
            swarm_sizes:  X-axis values. None = SWARM_SIZES.

        Returns:
            plt.Figure
        """
        swarm_sizes = swarm_sizes or SWARM_SIZES
        combinations = combinations or [(fl, marl)
                                         for fl in FL_ALGORITHMS
                                         for marl in MARL_ALGORITHMS]

        fig, ax = plt.subplots(figsize=self.figsize)

        for fl_name, marl_name in combinations:
            means, stds = [], []
            for n in swarm_sizes:
                results = tester.get_combination_results(fl_name, marl_name, n)
                vals = [r[metric] for r in results if metric in r]
                means.append(np.mean(vals) if vals else np.nan)
                stds.append(np.std(vals) if vals else 0)

            means = np.array(means, dtype=float)
            stds = np.array(stds, dtype=float)
            valid = ~np.isnan(means)

            if valid.any():
                color = FL_COLORS.get(fl_name, 'gray')
                marker = MARL_MARKERS.get(marl_name, 'x')
                label = f"{fl_name.upper()}+{marl_name.upper()}"
                ax.plot(np.array(swarm_sizes)[valid], means[valid],
                        marker=marker, color=color, linewidth=1.8,
                        markersize=7, label=label)
                ax.fill_between(
                    np.array(swarm_sizes)[valid],
                    (means - stds)[valid], (means + stds)[valid],
                    color=color, alpha=0.1
                )

        ax.set_xscale('log')
        ax.set_xticks(swarm_sizes)
        ax.set_xticklabels([str(n) for n in swarm_sizes])
        ax.set_xlabel('Swarm Size (num drones)', fontsize=12)
        ax.set_ylabel(metric.replace('avg_', '').replace('_', ' ').title(), fontsize=12)
        ax.set_title('Scaling Analysis: FL-MARL Performance vs Swarm Size', fontsize=14)
        ax.legend(fontsize=8, ncol=2, loc='best')
        ax.grid(True, alpha=0.4, which='both')
        fig.tight_layout()
        return fig

    def plot_scalability_heatmap(
        self,
        tester: CombinationTester,
        metric: str = 'avg_episode_reward'
    ) -> plt.Figure:
        """
        Heatmap of scalability ratio (performance at 100 / performance at 5 drones).

        Args:
            tester: CombinationTester
            metric: Metric to compute ratio for

        Returns:
            plt.Figure
        """
        try:
            import seaborn as sns
        except ImportError:
            print("seaborn not available; using matplotlib fallback")
            sns = None

        rows = FL_ALGORITHMS
        cols = MARL_ALGORITHMS

        data = np.full((len(rows), len(cols)), np.nan)
        for i, fl in enumerate(rows):
            for j, marl in enumerate(cols):
                r_small = tester.get_combination_results(fl, marl, SWARM_SIZES[0])
                r_large = tester.get_combination_results(fl, marl, SWARM_SIZES[-1])
                v_small = [r[metric] for r in r_small if metric in r]
                v_large = [r[metric] for r in r_large if metric in r]
                if v_small and v_large:
                    ratio = np.mean(v_large) / (np.mean(v_small) + 1e-8)
                    data[i, j] = ratio

        fig, ax = plt.subplots(figsize=(8, 5))
        if sns is not None:
            import pandas as pd
            df = pd.DataFrame(data, index=rows, columns=cols)
            sns.heatmap(df, annot=True, fmt='.2f', cmap='RdYlGn',
                        center=1.0, ax=ax, linewidths=0.5)
        else:
            im = ax.imshow(data, cmap='RdYlGn', aspect='auto', vmin=0.5, vmax=1.5)
            ax.set_xticks(range(len(cols)))
            ax.set_xticklabels(cols)
            ax.set_yticks(range(len(rows)))
            ax.set_yticklabels(rows)
            for i in range(len(rows)):
                for j in range(len(cols)):
                    if not np.isnan(data[i, j]):
                        ax.text(j, i, f'{data[i, j]:.2f}',
                                ha='center', va='center', fontsize=10)
            plt.colorbar(im, ax=ax)

        ax.set_title('Scalability Ratio (100 drones / 5 drones performance)', fontsize=13)
        ax.set_xlabel('MARL Algorithm', fontsize=11)
        ax.set_ylabel('FL Algorithm', fontsize=11)
        fig.tight_layout()
        return fig

    def save_figure(self, fig, path):
        import os
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        fig.savefig(path, dpi=self.dpi, bbox_inches='tight')
        print(f"  Figure saved: {path}")


if __name__ == '__main__':
    """Smoke test for ScalingPlotter."""
    import tempfile, sys
    sys.path.insert(0, '/home/skyvision/HFL-UAV-Swarm-Comparison')
    from src.evaluation.combination_tester import CombinationTester

    print("Testing ScalingPlotter...\n")

    with tempfile.TemporaryDirectory() as tmpdir:
        tester = CombinationTester(
            config={'seeds': [0, 1], 'swarm_sizes': [5, 10, 20]},
            results_dir=tmpdir
        )
        np.random.seed(0)
        for fl in FL_ALGORITHMS:
            for marl in MARL_ALGORITHMS:
                base = np.random.rand() * 0.5 + 1.0
                for n in [5, 10, 20]:
                    for seed in [0, 1]:
                        # Reward decreases slightly with swarm size (scaling challenge)
                        tester.store_result(fl, marl, n, seed, {
                            'avg_episode_reward': base - n * 0.005 + np.random.randn() * 0.03,
                        })

        plotter = ScalingPlotter()

        print("Test 1: Scaling curves")
        fig = plotter.plot_scaling_curves(tester, swarm_sizes=[5, 10, 20])
        assert fig is not None
        print("  \u2713 Scaling curves generated")

        print("Test 2: Scalability heatmap")
        fig2 = plotter.plot_scalability_heatmap(tester)
        assert fig2 is not None
        print("  \u2713 Scalability heatmap generated")

        plt.close('all')

    print("\n\u2705 ScalingPlotter tests passed!")
