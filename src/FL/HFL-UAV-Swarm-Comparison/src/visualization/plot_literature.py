"""
plot_literature.py — Literature comparison visualization.

Generates plots comparing our FL-MARL combinations against literature
baselines, including bar charts, scatter plots, and improvement tables.

Usage:
    from src.visualization.plot_literature import LiteraturePlotter
    plotter = LiteraturePlotter()
    fig = plotter.plot_improvement_bars(comparisons)
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from typing import Dict, List, Optional

FL_COLORS = {
    'fedavg': '#1f77b4', 'fedprox': '#ff7f0e',
    'fedadam': '#2ca02c', 'hierarchical': '#d62728',
}
BASELINE_COLORS = {
    'greedy_offload': '#9467bd',
    'round_robin': '#8c564b',
    'local_only': '#e377c2',
    'independent_ppo': '#7f7f7f',
    'fedavg_ippo': '#bcbd22',
    'd2d_fl': '#17becf',
    'ctde_upper_bound': '#aec7e8',
    'maddpg_no_fl': '#ffbb78',
}


class LiteraturePlotter:
    """Generates literature comparison plots."""

    def __init__(self, figsize=(12, 6), dpi=150):
        self.figsize = figsize
        self.dpi = dpi

    def plot_improvement_bars(
        self,
        comparisons: Dict,
        title: str = '',
        min_improvement_pct: float = -50
    ) -> plt.Figure:
        """
        Bar chart of % improvement over each baseline.

        Args:
            comparisons:          Output from LiteratureComparison.run_comparison()
            title:                Plot title
            min_improvement_pct:  Filter out very negative bars

        Returns:
            plt.Figure
        """
        fig, ax = plt.subplots(figsize=self.figsize)

        all_combos = list(comparisons.keys())
        all_baselines = set()
        for bl_dict in comparisons.values():
            for bl_name in bl_dict.keys():
                all_baselines.add(bl_name)
        all_baselines = sorted(all_baselines)

        n_combos = len(all_combos)
        n_bl = len(all_baselines)
        if n_combos == 0 or n_bl == 0:
            ax.text(0.5, 0.5, 'No comparison data', transform=ax.transAxes, ha='center')
            return fig

        x = np.arange(n_bl)
        width = 0.8 / max(n_combos, 1)

        for ci, combo in enumerate(all_combos):
            improvements = []
            for bl_name in all_baselines:
                result = comparisons[combo].get(bl_name, {})
                pct = result.get('performance_gap_pct', 0) if 'error' not in result else 0
                improvements.append(pct)

            offset = (ci - n_combos / 2 + 0.5) * width
            ax.bar(x + offset, improvements, width=width * 0.9,
                   label=combo, alpha=0.85)

        ax.axhline(y=0, color='black', linestyle='-', linewidth=0.8)
        ax.set_xticks(x)
        ax.set_xticklabels([b.replace('_', '\n') for b in all_baselines],
                            fontsize=8, rotation=20, ha='right')
        ax.set_xlabel('Literature Baseline', fontsize=11)
        ax.set_ylabel('% Improvement over Baseline', fontsize=11)
        ax.set_title(title or 'Our FL-MARL vs Literature Baselines', fontsize=13)
        ax.legend(fontsize=8, loc='upper right', ncol=2)
        ax.grid(True, axis='y', alpha=0.3)
        fig.tight_layout()
        return fig

    def plot_reward_vs_baseline(
        self,
        our_results: Dict[str, List[float]],
        baseline_results: Dict[str, List[float]],
        title: str = ''
    ) -> plt.Figure:
        """
        Box plot comparing our top combinations against all baselines.

        Args:
            our_results:      {combo_name: [reward, ...]}
            baseline_results: {baseline_name: [reward, ...]}
            title:            Plot title

        Returns:
            plt.Figure
        """
        fig, ax = plt.subplots(figsize=self.figsize)

        all_data = {}
        # Baselines first (grey tones), then our combos (colored)
        for name, rewards in baseline_results.items():
            all_data[name] = rewards
        for name, rewards in our_results.items():
            all_data[name] = rewards

        positions = list(range(len(all_data)))
        labels = list(all_data.keys())
        data = [all_data[k] for k in labels]

        n_bl = len(baseline_results)
        colors = (['#aaaaaa'] * n_bl + ['#2ca02c'] * len(our_results))

        bp = ax.boxplot(data, positions=positions, patch_artist=True,
                        widths=0.5, notch=False)
        for patch, color in zip(bp['boxes'], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.7)

        ax.set_xticks(positions)
        ax.set_xticklabels([l.replace('_', '\n').replace('+', '\n+')
                            for l in labels], fontsize=8, rotation=30, ha='right')
        ax.set_ylabel('Episode Reward', fontsize=12)
        ax.set_title(title or 'Our Methods vs Literature Baselines', fontsize=13)
        ax.grid(True, axis='y', alpha=0.3)

        our_patch = mpatches.Patch(color='#2ca02c', alpha=0.7, label='Our combinations')
        bl_patch = mpatches.Patch(color='#aaaaaa', alpha=0.7, label='Literature baselines')
        ax.legend(handles=[our_patch, bl_patch], fontsize=10)

        fig.tight_layout()
        return fig

    def save_figure(self, fig, path):
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        fig.savefig(path, dpi=self.dpi, bbox_inches='tight')
        print(f"  Figure saved: {path}")


if __name__ == '__main__':
    """Smoke test."""
    print("Testing LiteraturePlotter...\n")

    np.random.seed(0)
    comparisons = {
        'fedadam+mappo': {
            'greedy_offload': {'performance_gap_pct': 35.0, 'significant_t': True},
            'independent_ppo': {'performance_gap_pct': 12.0, 'significant_t': True},
        },
        'hierarchical+mappo': {
            'greedy_offload': {'performance_gap_pct': 28.0, 'significant_t': True},
            'independent_ppo': {'performance_gap_pct': 5.0, 'significant_t': False},
        },
    }

    plotter = LiteraturePlotter()

    print("Test 1: Improvement bars")
    fig = plotter.plot_improvement_bars(comparisons)
    assert fig is not None
    print("  \u2713 Improvement bars generated")

    print("Test 2: Reward vs baseline")
    our = {'fedadam+mappo': (np.random.randn(30) * 0.1 + 1.5).tolist()}
    bl = {
        'greedy_offload':  (np.random.randn(30) * 0.1 + 0.9).tolist(),
        'independent_ppo': (np.random.randn(30) * 0.1 + 1.2).tolist(),
    }
    fig2 = plotter.plot_reward_vs_baseline(our, bl)
    assert fig2 is not None
    print("  \u2713 Reward vs baseline generated")

    plt.close('all')
    print("\n\u2705 LiteraturePlotter tests passed!")
