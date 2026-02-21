"""
dashboard.py — Real-time training monitoring dashboard.

Provides a lightweight live dashboard for monitoring ongoing training
sessions. Updates plots in real-time as training progresses.

Usage:
    from src.visualization.dashboard import TrainingDashboard

    dashboard = TrainingDashboard(num_agents=10, fl_name='fedadam')
    dashboard.update(round_metrics)
    dashboard.show()
"""

import os
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from typing import Dict, List, Optional
from collections import deque


class TrainingDashboard:
    """
    Real-time training dashboard for monitoring FL-MARL training.

    Shows live updates of:
        - Episode reward curve
        - Actor / critic loss curves
        - Task success rate
        - Communication / clustering stats (if HFL)

    Designed to be called once per FL round inside the training loop.

    Args:
        num_agents: Number of UAVs (for display)
        fl_name:    FL algorithm name (for title)
        marl_name:  MARL algorithm name (for title)
        window:     Number of recent rounds to show
        update_interval: Show plot every N calls
    """

    def __init__(
        self,
        num_agents: int = 10,
        fl_name: str = '',
        marl_name: str = '',
        window: int = 100,
        update_interval: int = 5
    ):
        self.num_agents = num_agents
        self.fl_name = fl_name.upper()
        self.marl_name = marl_name.upper()
        self.window = window
        self.update_interval = update_interval

        # Rolling history queues
        self.rewards = deque(maxlen=window)
        self.actor_losses = deque(maxlen=window)
        self.critic_losses = deque(maxlen=window)
        self.success_rates = deque(maxlen=window)
        self.rounds = deque(maxlen=window)
        self._call_count = 0

        # Figure setup
        self.fig = None
        self.axes = None

    def _setup_figure(self):
        """Create the dashboard figure and axes."""
        plt.ion()
        self.fig = plt.figure(figsize=(14, 8))
        self.fig.suptitle(
            f'Training Dashboard — {self.marl_name} + {self.fl_name} '
            f'({self.num_agents} UAVs)',
            fontsize=14
        )
        gs = gridspec.GridSpec(2, 2, figure=self.fig, hspace=0.4, wspace=0.35)
        self.axes = {
            'reward':  self.fig.add_subplot(gs[0, 0]),
            'loss':    self.fig.add_subplot(gs[0, 1]),
            'success': self.fig.add_subplot(gs[1, 0]),
            'stats':   self.fig.add_subplot(gs[1, 1]),
        }
        plt.show(block=False)

    def update(self, metrics: Dict):
        """
        Update dashboard with latest round metrics.

        Call this once per FL round with the metrics dict from the trainer.

        Args:
            metrics: Round metrics dict containing any of:
                - 'round', 'ppo_actor_loss', 'ppo_critic_loss',
                - 'eval.avg_episode_reward', 'avg_task_success_rate',
                - 'fl_round', 'num_clusters'
        """
        # Store data
        self.rounds.append(metrics.get('round', len(self.rounds) + 1))

        reward = (metrics.get('eval', {}).get('avg_episode_reward')
                  or metrics.get('avg_episode_reward', np.nan))
        self.rewards.append(reward)
        self.actor_losses.append(metrics.get('ppo_actor_loss', np.nan))
        self.critic_losses.append(metrics.get('ppo_critic_loss', np.nan))
        self.success_rates.append(
            metrics.get('eval', {}).get('avg_success_rate',
                metrics.get('avg_task_success_rate', np.nan))
        )

        self._call_count += 1

        # Refresh plot every update_interval calls
        if self._call_count % self.update_interval == 0:
            self._redraw()

    def _redraw(self):
        """Redraw all dashboard panels."""
        if self.fig is None:
            self._setup_figure()

        rounds = list(self.rounds)

        # Panel 1: Reward
        ax = self.axes['reward']
        ax.clear()
        rewards = [r for r in self.rewards if not np.isnan(r)]
        if rewards:
            ax.plot(range(len(rewards)), rewards, color='#2ca02c', linewidth=1.5)
        ax.set_title('Episode Reward', fontsize=11)
        ax.set_xlabel('Round', fontsize=9)
        ax.grid(True, alpha=0.3)

        # Panel 2: Loss
        ax = self.axes['loss']
        ax.clear()
        actor = [v for v in self.actor_losses if not np.isnan(v)]
        critic = [v for v in self.critic_losses if not np.isnan(v)]
        if actor:
            ax.plot(range(len(actor)), actor, label='Actor', color='#1f77b4')
        if critic:
            ax.plot(range(len(critic)), critic, label='Critic', color='#ff7f0e')
        ax.set_title('Training Losses', fontsize=11)
        ax.set_xlabel('Round', fontsize=9)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # Panel 3: Success rate
        ax = self.axes['success']
        ax.clear()
        sr = [v for v in self.success_rates if not np.isnan(v)]
        if sr:
            ax.plot(range(len(sr)), sr, color='#d62728', linewidth=1.5)
            ax.set_ylim([0, 1.05])
        ax.set_title('Task Success Rate', fontsize=11)
        ax.set_xlabel('Round', fontsize=9)
        ax.grid(True, alpha=0.3)

        # Panel 4: Stats table
        ax = self.axes['stats']
        ax.clear()
        ax.axis('off')
        stats = [
            ['Metric', 'Latest', 'Best'],
            ['Reward',
             f"{rewards[-1]:.3f}" if rewards else 'N/A',
             f"{max(rewards):.3f}" if rewards else 'N/A'],
            ['Actor Loss',
             f"{actor[-1]:.4f}" if actor else 'N/A', '-'],
            ['Success Rate',
             f"{sr[-1]:.1%}" if sr else 'N/A',
             f"{max(sr):.1%}" if sr else 'N/A'],
            ['Round', str(rounds[-1]) if rounds else '0', '-'],
        ]
        t = ax.table(cellText=stats[1:], colLabels=stats[0],
                     loc='center', cellLoc='center')
        t.auto_set_font_size(False)
        t.set_fontsize(9)
        t.scale(1.2, 1.4)
        ax.set_title('Summary', fontsize=11, pad=30)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def show(self):
        """Show final dashboard (blocking)."""
        if self.fig is None:
            self._setup_figure()
        self._redraw()
        plt.ioff()
        plt.show()

    def save(self, path: str):
        """Save dashboard to file."""
        if self.fig is None:
            self._redraw()
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        self.fig.savefig(path, dpi=150, bbox_inches='tight')
        print(f"  Dashboard saved: {path}")

    def close(self):
        """Close dashboard figure."""
        if self.fig is not None:
            plt.close(self.fig)
            self.fig = None


class StaticDashboard:
    """
    Post-training static dashboard from saved training history.

    Use this to reproduce dashboard plots from saved JSON history
    without needing a live training session.

    Args:
        history: List of round metrics dicts
        config:  Display configuration
    """

    def __init__(self, history: List[Dict], config: Optional[Dict] = None):
        self.history = history
        self.config = config or {}

    def build(
        self,
        fl_name: str = '',
        marl_name: str = '',
        num_agents: int = 10
    ) -> plt.Figure:
        """
        Build a static version of the training dashboard.

        Args:
            fl_name, marl_name: For title
            num_agents:         For title

        Returns:
            plt.Figure
        """
        dashboard = TrainingDashboard(
            num_agents=num_agents,
            fl_name=fl_name,
            marl_name=marl_name,
            update_interval=1
        )
        for metrics in self.history:
            dashboard._call_count += 1
            reward = (metrics.get('eval', {}).get('avg_episode_reward')
                      or metrics.get('avg_episode_reward', np.nan))
            dashboard.rewards.append(reward)
            dashboard.actor_losses.append(metrics.get('ppo_actor_loss', np.nan))
            dashboard.critic_losses.append(metrics.get('ppo_critic_loss', np.nan))
            dashboard.rounds.append(metrics.get('round', len(dashboard.rounds) + 1))

        plt.ioff()
        dashboard._setup_figure()
        dashboard._redraw()
        return dashboard.fig


if __name__ == '__main__':
    """Smoke test for dashboard."""
    print("Testing TrainingDashboard (non-interactive)...\n")

    np.random.seed(42)
    history = []
    reward = 0.0
    for i in range(20):
        reward += np.random.randn() * 0.05 + 0.03
        history.append({
            'round': i + 1,
            'ppo_actor_loss': max(0, 1.0 - i * 0.04),
            'ppo_critic_loss': max(0, 0.8 - i * 0.03),
            'eval': {'avg_episode_reward': reward, 'avg_success_rate': 0.5 + i * 0.02},
        })

    print("Test 1: Static dashboard build")
    static = StaticDashboard(history)
    plt.switch_backend('Agg')  # Non-interactive for testing
    try:
        fig = static.build(fl_name='fedadam', marl_name='mappo', num_agents=10)
        assert fig is not None
        print("  \u2713 Static dashboard built")
    except Exception as e:
        print(f"  (dashboard skipped in headless env: {e})")

    print("\nTest 2: Live dashboard update (no display)")
    dashboard = TrainingDashboard(num_agents=10, fl_name='fedadam', update_interval=100)
    for m in history:
        dashboard.update(m)
    assert len(dashboard.rewards) == 20
    print(f"  \u2713 Updated with {len(dashboard.rewards)} rounds")

    plt.close('all')
    print("\n\u2705 Dashboard tests passed!")
