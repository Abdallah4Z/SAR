"""
Metrics Tracker — Track performance metrics for the UAV swarm environment.

Tracks:
- Task latencies (submission → completion)
- Task success/failure rates
- Energy consumption
- Fairness (coefficient of variation across UAVs)
"""

import numpy as np
from typing import Dict


class MetricsTracker:
    """Track performance metrics."""

    def __init__(self, num_uavs: int):
        self.num_uavs = num_uavs
        self.reset()

    def reset(self):
        """Reset all metrics."""
        self.task_submit_times = {}
        self.task_complete_times = {}
        self.task_success = {}
        self.total_energy = 0.0
        self.tasks_per_uav = np.zeros(self.num_uavs)
        
        # CRITICAL FIX: Track energy components separately
        self.computation_energy = 0.0
        self.communication_energy = 0.0
        self.flight_energy = 0.0

    def on_task_submit(self, task_id: int, time: float):
        """Record task submission."""
        self.task_submit_times[task_id] = time

    def on_task_complete(self, task_id: int, time: float, uav_id: int, success: bool):
        """Record task completion."""
        self.task_complete_times[task_id] = time
        self.task_success[task_id] = success
        if success:
            self.tasks_per_uav[uav_id] += 1

    def update_energy(self, energy: float):
        """Add energy consumption."""
        self.total_energy += energy
    
    def update_energy_components(self, computation: float = 0.0, 
                                 communication: float = 0.0, 
                                 flight: float = 0.0):
        """
        Track energy consumption by component.
        
        CRITICAL FIX: Separately track computation, communication, and flight energy.
        
        Args:
            computation: Computation energy (J)
            communication: Communication energy (J)
            flight: Flight/movement energy (J)
        """
        self.computation_energy += computation
        self.communication_energy += communication
        self.flight_energy += flight

    def get_metrics(self, current_time: float) -> Dict:
        """Calculate all metrics."""
        # Latency
        latencies = []
        for tid in self.task_complete_times:
            if tid in self.task_submit_times:
                latency = (self.task_complete_times[tid] -
                          self.task_submit_times[tid]) * 1000  # Convert to ms
                latencies.append(latency)
        avg_latency = np.mean(latencies) if latencies else 0

        # Success rate
        total_completed = len(self.task_complete_times)
        successful = sum(self.task_success.values())
        success_rate = (successful / total_completed * 100) if total_completed > 0 else 0

        # Fairness (coefficient of variation)
        mean_tasks = np.mean(self.tasks_per_uav)
        std_tasks = np.std(self.tasks_per_uav)
        fairness_cv = std_tasks / mean_tasks if mean_tasks > 0 else 0

        return {
            'average_latency_ms': avg_latency,
            'total_energy_j': self.total_energy,
            'computation_energy_j': self.computation_energy,
            'communication_energy_j': self.communication_energy,
            'flight_energy_j': self.flight_energy,
            'task_success_rate': success_rate,
            'tasks_completed': successful,
            'tasks_failed': total_completed - successful,
            'fairness_cv': fairness_cv
        }
