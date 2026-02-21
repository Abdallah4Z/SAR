"""
plot_fl_marl_matrix.py — FL-MARL combination heatmaps.

Generates heatmaps visualizing the 4 FL x 3 MARL performance matrix
for different metrics and swarm sizes. The primary visualization for
identifying the best FL-MARL combinations.

Usage:
    from src.visualization.plot_fl_marl_matrix import MatrixPlotter
    plotter = MatrixPlotter()
    fig = plotter.plot_combination_heatmap(tester, metric='avg_episode_reward')
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, List, Optional, Tuple

from src.evaluation.combination_tester import (
    CombinationTester, FL_ALGORITHMS, MARL_ALGORITHMS, SWARM_SIZES
)


class MatrixPlotter:
    """
    Generates FL x MARL combination heatmaps.

    Args:
        figsize: Default figure size
        dpi:     Save DPI
    """

    def __init__(self, figsize=(8, 5), dpi=150):
        self.figsize = figsize
        self.dpi = dpi

    def plot_combination_heatmap(
        self,
        tester: CombinationTester,
        metric: str = 'avg_episode_reward',
        num_drones: int = 10,
        title: str = '',
        annotate: bool = True
    ) -> plt.Figure:
        """
        Heatmap of FL x MARL combination performance.

        Args:
            tester:     CombinationTester with results
            metric:     Metric to display in cells
            num_drones: Swarm size to use
            title:      Plot title
            annotate:   If True, show numeric values in cells

        Returns:
            plt.Figure
        """
        matrix = tester.build_performance_matrix(metric, num_drones)

        data = np.full((len(FL_ALGORITHMS), len(MARL_ALGORITHMS)), np.nan)
        for i, fl in enumerate(FL_ALGORITHMS):
            for j, marl in enumerate(MARL_ALGORITHMS):
                val = matrix.get(fl, {}).get(marl, None)
                if val is not None:
                    data[i, j] = val

        fig, ax = plt.subplots(figsize=self.figsize)

        # Try seaborn, fall back to matplotlib
        try:
            import seaborn as sns
            import pandas as pd
            df = pd.DataFrame(data, index=FL_ALGORITHMS, columns=MARL_ALGORITHMS)
            sns.heatmap(df, annot=annotate, fmt='.3f', cmap='YlOrRd',
                        ax=ax, linewidths=0.5, linecolor='white',
                        cbar_kws={'label': metric.replace('avg_', '').replace('_', ' ').title()})
        except ImportError:
            im = ax.imshow(data, cmap='YlOrRd', aspect='auto')
            ax.set_xticks(range(len(MARL_ALGORITHMS)))
            ax.set_xticklabels(MARL_ALGORITHMS)
            ax.set_yticks(range(len(FL_ALGORITHMS)))
            ax.set_yticklabels(FL_ALGORITHMS)
            if annotate:
                for i in range(len(FL_ALGORITHMS)):
                    for j in range(len(MARL_ALGORITHMS)):
                        if not np.isnan(data[i, j]):
                            ax.text(j, i, f'{data[i, j]:.3f}',
                                    ha='center', va='center', fontsize=9)
            plt.colorbar(im, ax=ax,
                         label=metric.replace('avg_', '').replace('_', ' ').title())

        ax.set_xlabel('MARL Algorithm', fontsize=12)
        ax.set_ylabel('FL Algorithm', fontsize=12)
        default_title = (f'FL-MARL Combination: '
                         f'{metric.replace("avg_","").replace("_"," ").title()} '
                         f'({num_drones} drones)')
        ax.set_title(title or default_title, fontsize=13)
        fig.tight_layout()
        return fig

    def plot_multi_size_grid(
        self,
        tester: CombinationTester,
        metric: str = 'avg_episode_reward',
        swarm_sizes: Optional[List[int]] = None
    ) -> plt.Figure:
        """
        Grid of heatmaps, one per swarm size.

        Args:
            tester:      CombinationTester
            metric:      Metric to plot
            swarm_sizes: Swarm sizes to include

        Returns:
            plt.Figure with one subplot per swarm size
        """
        sizes = swarm_sizes or SWARM_SIZES
        n_sizes = len(sizes)
        fig, axes = plt.subplots(1, n_sizes, figsize=(n_sizes * 4, 4))
        if n_sizes == 1:
            axes = [axes]

        for ax, n in zip(axes, sizes):
            matrix = tester.build_performance_matrix(metric, n)
            data = np.full((len(FL_ALGORITHMS), len(MARL_ALGORITHMS)), np.nan)
            for i, fl in enumerate(FL_ALGORITHMS):
                for j, marl in enumerate(MARL_ALGORITHMS):
                    val = matrix.get(fl, {}).get(marl, None)
                    if val is not None:
                        data[i, j] = val

            im = ax.imshow(data, cmap='YlOrRd', aspect='auto')
            ax.set_xticks(range(len(MARL_ALGORITHMS)))
            ax.set_xticklabels([m[:4].upper() for m in MARL_ALGORITHMS], fontsize=8)
            ax.set_yticks(range(len(FL_ALGORITHMS)))
            ax.set_yticklabels([f[:6].upper() for f in FL_ALGORITHMS], fontsize=8)
            ax.set_title(f'{n} drones', fontsize=10)
            plt.colorbar(im, ax=ax, fraction=0.046)

        fig.suptitle(f'FL-MARL Performance: {metric} across Swarm Sizes', fontsize=12)
        fig.tight_layout()
        return fig

    def save_figure(self, fig, path):
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        fig.savefig(path, dpi=self.dpi, bbox_inches='tight')
        print(f"  Figure saved: {path}")


if __name__ == '__main__':
    """Smoke test."""
    import tempfile, sys
    sys.path.insert(0, '/home/skyvision/HFL-UAV-Swarm-Comparison')
    from src.evaluation.combination_tester import CombinationTester

    print("Testing MatrixPlotter...\n")

    with tempfile.TemporaryDirectory() as tmpdir:
        tester = CombinationTester(
            config={'seeds': [0, 1], 'swarm_sizes': [5, 10, 20]},
            results_dir=tmpdir
        )
        np.random.seed(0)
        for fl in FL_ALGORITHMS:
            for marl in MARL_ALGORITHMS:
                for n in [5, 10, 20]:
                    for seed in [0, 1]:
                        tester.store_result(fl, marl, n, seed, {
                            'avg_episode_reward': np.random.rand() + 0.5,
                        })

        plotter = MatrixPlotter()
        print("Test 1: Combination heatmap")
        fig = plotter.plot_combination_heatmap(tester, num_drones=10)
        assert fig is not None
        print("  \u2713 Heatmap generated")

        print("Test 2: Multi-size grid")
        fig2 = plotter.plot_multi_size_grid(tester, swarm_sizes=[5, 10, 20])
        assert fig2 is not None
        print("  \u2713 Multi-size grid generated")

        plt.close('all')

    print("\n\u2705 MatrixPlotter tests passed!")
