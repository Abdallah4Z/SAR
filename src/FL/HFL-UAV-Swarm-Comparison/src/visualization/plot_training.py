"""
plot_training.py — Training curve visualizations.

Plots reward curves, loss curves, and convergence diagnostics from
training history (saved as JSON by each MARL trainer).

Usage:
    from src.visualization.plot_training import TrainingPlotter

    plotter = TrainingPlotter(style='seaborn-v0_8-paper')
    fig = plotter.plot_reward_curve(history, title='MAPPO + FedAdam')
    plotter.save_figure(fig, 'results/plots/reward_curve.png')
"""

import os
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from typing import Dict, List, Optional, Tuple


# Consistent color palette for FL algorithms (used across all plots)
FL_COLORS = {
    'fedavg':       '#1f77b4',   # blue
    'fedprox':      '#ff7f0e',   # orange
    'fedadam':      '#2ca02c',   # green  (Ammar)
    'hierarchical': '#d62728',   # red
}

MARL_STYLES = {
    'mappo':  '-',    # solid
    'maddpg': '--',   # dashed
    'qmix':   ':',    # dotted
}


class TrainingPlotter:
    """
    Generates training curve plots for FL-MARL combinations.

    Args:
        style: Matplotlib style ('seaborn-v0_8-paper', 'ggplot', etc.)
        figsize: Default figure size
        dpi: Figure DPI for saved files
    """

    def __init__(
        self,
        style: str = 'seaborn-v0_8-whitegrid',
        figsize: Tuple[int, int] = (10, 6),
        dpi: int = 150
    ):
        self.style = style
        self.figsize = figsize
        self.dpi = dpi

    def _apply_style(self):
        """Apply plot style, falling back gracefully."""
        try:
            plt.style.use(self.style)
        except Exception:
            plt.style.use('ggplot')

    def plot_reward_curve(
        self,
        history: List[Dict],
        title: str = '',
        smooth_window: int = 5,
        show_std: bool = True,
        ax: Optional[plt.Axes] = None
    ) -> plt.Figure:
        """
        Plot episode reward over FL training rounds.

        Args:
            history:       List of round metrics dicts (from trainer.training_history)
            title:         Plot title
            smooth_window: Moving average window for smoothing
            show_std:      If True, shade +/- std around the curve
            ax:            Existing axes to plot into (creates new if None)

        Returns:
            plt.Figure: Figure object
        """
        self._apply_style()
        fig, ax = (plt.subplots(figsize=self.figsize) if ax is None
                   else (ax.figure, ax))

        # Extract reward values
        rounds = [m.get('round', i+1) for i, m in enumerate(history)]
        rewards = [m.get('eval', {}).get('avg_episode_reward',
                    m.get('avg_episode_reward', np.nan))
                   for m in history]
        rewards = [r for r in rewards if not np.isnan(r)]

        if not rewards:
            ax.text(0.5, 0.5, 'No reward data', transform=ax.transAxes, ha='center')
            return fig

        rewards = np.array(rewards, dtype=float)
        r_smooth = _smooth(rewards, smooth_window)

        ax.plot(range(len(r_smooth)), r_smooth, linewidth=2.0, label='Reward')
        if show_std and len(rewards) > smooth_window:
            r_std = _rolling_std(rewards, smooth_window)
            ax.fill_between(range(len(r_smooth)),
                            r_smooth - r_std, r_smooth + r_std,
                            alpha=0.2)

        ax.set_xlabel('FL Round', fontsize=12)
        ax.set_ylabel('Average Episode Reward', fontsize=12)
        ax.set_title(title or 'Training Reward Curve', fontsize=14)
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.4)
        fig.tight_layout()
        return fig

    def plot_loss_curves(
        self,
        history: List[Dict],
        title: str = '',
        ax: Optional[plt.Axes] = None
    ) -> plt.Figure:
        """
        Plot actor and critic loss curves.

        Args:
            history: Training history list
            title:   Plot title
            ax:      Existing axes

        Returns:
            plt.Figure
        """
        self._apply_style()
        fig, ax = (plt.subplots(figsize=self.figsize) if ax is None
                   else (ax.figure, ax))

        actor_losses = [m.get('ppo_actor_loss', np.nan) for m in history]
        critic_losses = [m.get('ppo_critic_loss', np.nan) for m in history]
        rounds = list(range(len(history)))

        actor_clean = [(r, v) for r, v in zip(rounds, actor_losses) if not np.isnan(v)]
        critic_clean = [(r, v) for r, v in zip(rounds, critic_losses) if not np.isnan(v)]

        if actor_clean:
            r, v = zip(*actor_clean)
            ax.plot(r, _smooth(list(v), 5), label='Actor Loss',
                    color='#1f77b4', linewidth=1.8)
        if critic_clean:
            r, v = zip(*critic_clean)
            ax.plot(r, _smooth(list(v), 5), label='Critic Loss',
                    color='#ff7f0e', linewidth=1.8)

        ax.set_xlabel('FL Round', fontsize=12)
        ax.set_ylabel('Loss', fontsize=12)
        ax.set_title(title or 'Training Loss Curves', fontsize=14)
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.4)
        fig.tight_layout()
        return fig

    def plot_multi_fl_reward(
        self,
        histories: Dict[str, List[Dict]],
        marl_name: str = '',
        smooth_window: int = 5
    ) -> plt.Figure:
        """
        Overlay reward curves for 4 FL algorithms on the same MARL.

        Args:
            histories:     {fl_name: training_history_list}
            marl_name:     MARL algorithm name (for title)
            smooth_window: Smoothing window

        Returns:
            plt.Figure
        """
        self._apply_style()
        fig, ax = plt.subplots(figsize=self.figsize)

        for fl_name, history in histories.items():
            rewards = [m.get('eval', {}).get('avg_episode_reward',
                        m.get('avg_episode_reward', np.nan))
                       for m in history]
            rewards = [r for r in rewards if not np.isnan(r)]
            if not rewards:
                continue
            rewards = np.array(rewards)
            r_smooth = _smooth(rewards, smooth_window)
            color = FL_COLORS.get(fl_name, 'gray')
            ax.plot(range(len(r_smooth)), r_smooth,
                    label=fl_name.upper(), color=color, linewidth=2.0)

        ax.set_xlabel('FL Round', fontsize=12)
        ax.set_ylabel('Average Episode Reward', fontsize=12)
        title = f"{marl_name.upper()} — FL Algorithm Comparison" if marl_name else "FL Comparison"
        ax.set_title(title, fontsize=14)
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.4)
        fig.tight_layout()
        return fig

    def save_figure(self, fig: plt.Figure, path: str):
        """Save figure to disk, creating directories if needed."""
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        fig.savefig(path, dpi=self.dpi, bbox_inches='tight')
        print(f"  Figure saved: {path}")

    def close_all(self):
        """Close all open matplotlib figures."""
        plt.close('all')


# ======================================================================
# Utility functions
# ======================================================================

def _smooth(values: np.ndarray, window: int) -> np.ndarray:
    """Apply a simple moving average for smoothing."""
    if len(values) < window:
        return values
    kernel = np.ones(window) / window
    return np.convolve(values, kernel, mode='valid')


def _rolling_std(values: np.ndarray, window: int) -> np.ndarray:
    """Compute rolling standard deviation."""
    if len(values) < window:
        return np.zeros(len(values))
    result = []
    for i in range(window - 1, len(values)):
        result.append(np.std(values[i - window + 1: i + 1]))
    return np.array(result)


if __name__ == '__main__':
    """Smoke test for training plotter."""
    print("Testing TrainingPlotter...\n")

    # Mock training history
    np.random.seed(42)
    n_rounds = 50
    history = []
    reward = 0.0
    for i in range(n_rounds):
        reward += np.random.randn() * 0.05 + 0.03
        history.append({
            'round': i + 1,
            'ppo_actor_loss': max(0, 1.0 - i * 0.015 + np.random.randn() * 0.05),
            'ppo_critic_loss': max(0, 0.8 - i * 0.01 + np.random.randn() * 0.04),
            'eval': {'avg_episode_reward': reward},
        })

    plotter = TrainingPlotter()

    print("Test 1: Reward curve")
    fig = plotter.plot_reward_curve(history, title='MAPPO + FedAdam (10 UAVs)')
    assert fig is not None
    print("  \u2713 Reward curve generated")

    print("Test 2: Loss curves")
    fig2 = plotter.plot_loss_curves(history, title='Training Losses')
    assert fig2 is not None
    print("  \u2713 Loss curves generated")

    print("Test 3: Multi-FL comparison")
    histories = {
        'fedavg':       history,
        'fedprox':      [{**h, 'eval': {'avg_episode_reward': h['eval']['avg_episode_reward'] - 0.1}}
                         for h in history],
        'fedadam':      [{**h, 'eval': {'avg_episode_reward': h['eval']['avg_episode_reward'] + 0.05}}
                         for h in history],
        'hierarchical': [{**h, 'eval': {'avg_episode_reward': h['eval']['avg_episode_reward'] - 0.05}}
                         for h in history],
    }
    fig3 = plotter.plot_multi_fl_reward(histories, marl_name='mappo')
    assert fig3 is not None
    print("  \u2713 Multi-FL comparison plot generated")

    plotter.close_all()
    print("\n\u2705 TrainingPlotter tests passed!")
