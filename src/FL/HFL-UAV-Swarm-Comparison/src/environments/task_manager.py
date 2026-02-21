"""
Task Manager — Task generation with Poisson process and 3 QoS classes.

V4 CRITICAL FIXES:
- Task parameters now FEASIBLE (latency-critical: 20M cycles + 50KB = 23ms < 50ms)
- Uses instance RNG for reproducibility
- Task IDs are instance-specific
"""

import numpy as np
from typing import List, Dict


class Task:
    """
    Represents a single computation task.
    """

    def __init__(self,
                 task_id: int,
                 source_gd_id: int,
                 qos_class: str,
                 cpu_megacycles: float,
                 data_size_mb: float,
                 deadline_seconds: float,
                 arrival_time: float,
                 priority: int):
        """
        Create a new task.

        Args:
            task_id: Unique identifier
            source_gd_id: Ground device that generated this task
            qos_class: 'latency_critical', 'compute_intensive', or 'best_effort'
            cpu_megacycles: CPU cycles in millions (e.g., 500 = 500M cycles)
            data_size_mb: Data size in megabytes
            deadline_seconds: Deadline in seconds from arrival
            arrival_time: Simulation time when task arrived
            priority: Integer priority (higher = more important)
        """
        self.id = task_id
        self.source_gd_id = source_gd_id
        self.qos_class = qos_class
        self.cpu_megacycles = cpu_megacycles   # millions of cycles
        self.data_size_mb = data_size_mb
        self.deadline_seconds = deadline_seconds
        self.arrival_time = arrival_time
        self.deadline_time = arrival_time + deadline_seconds
        self.priority = priority

        # Status tracking
        self.assigned_uav_id = None
        self.start_time = None
        self.completion_time = None
        self.status = 'pending'  # 'pending', 'processing', 'completed', 'failed'


class TaskGenerator:
    """
    Generate tasks using Poisson process with 3 QoS classes.

    V4 FIXES:
    - Uses instance RNG for reproducibility
    - Task IDs are instance-specific
    - Deadlines compatible with 10ms timestep
    - Task parameters FEASIBLE
    """

    def __init__(self,
                 num_ground_devices: int = 20,
                 lambda_rate: float = 0.5,
                 rng: np.random.RandomState = None):
        """
        Initialize task generator.

        Args:
            num_ground_devices: Number of ground devices
            lambda_rate: Average tasks/second/device (Poisson parameter)
            rng: Random number generator for reproducibility
        """
        self.num_gd = num_ground_devices
        self.lambda_rate = lambda_rate
        self.current_time = 0.0

        # Instance RNG, not global np.random
        self.rng = rng if rng is not None else np.random.RandomState()

        # Task ID counter is instance variable
        self._task_id_counter = 0

        # QoS class definitions — V4 FEASIBLE parameters
        # CPU cycles in MILLIONS, deadlines in SECONDS
        self.qos_classes = {
            'latency_critical': {
                'probability': 0.3,
                'cpu_megacycles': 20,       # 20M cycles → 10ms @ 2GHz
                'data_size_mb': 0.05,       # 50 KB → ~13ms upload @ 30Mbps
                'deadline_seconds': 0.05,   # 50ms = 0.05s (5 timesteps at 10ms)
                'priority': 3
            },
            'compute_intensive': {
                'probability': 0.5,
                'cpu_megacycles': 800,      # 800M cycles → 400ms @ 2GHz
                'data_size_mb': 2,          # 2 MB → ~50ms upload @ 30Mbps
                'deadline_seconds': 0.5,    # 500ms = 0.5s (50 timesteps)
                'priority': 2
            },
            'best_effort': {
                'probability': 0.2,
                'cpu_megacycles': 1000,     # 1000M cycles → 500ms @ 2GHz
                'data_size_mb': 5,          # 5 MB
                'deadline_seconds': 1.0,    # 1000ms = 1.0s (100 timesteps)
                'priority': 1
            }
        }

    def generate_tasks(self, dt: float) -> List[Task]:
        """
        Generate tasks for this timestep using Poisson process.

        Args:
            dt: Timestep duration in seconds

        Returns:
            List of newly generated Task objects
        """
        tasks = []

        for gd_id in range(self.num_gd):
            # Use instance RNG
            num_arrivals = self.rng.poisson(self.lambda_rate * dt)

            for _ in range(num_arrivals):
                # Select QoS class randomly
                qos_name = self.rng.choice(
                    list(self.qos_classes.keys()),
                    p=[0.3, 0.5, 0.2]
                )

                qos = self.qos_classes[qos_name]

                # Use instance task counter
                task = Task(
                    task_id=self._task_id_counter,
                    source_gd_id=gd_id,
                    qos_class=qos_name,
                    cpu_megacycles=qos['cpu_megacycles'],
                    data_size_mb=qos['data_size_mb'],
                    deadline_seconds=qos['deadline_seconds'],
                    arrival_time=self.current_time,
                    priority=qos['priority']
                )

                self._task_id_counter += 1
                tasks.append(task)

        self.current_time += dt
        return tasks
