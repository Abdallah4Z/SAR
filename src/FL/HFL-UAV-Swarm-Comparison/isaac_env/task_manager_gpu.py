"""
GPU-Tensorized Task Manager for Isaac Sim Environment.

Mirrors the logic of src/environments/task_manager.py but operates
entirely on PyTorch GPU tensors for batched environments.

Task buffer layout per env per task slot:
    [cpu_megacycles, data_size_mb, deadline_remaining, priority,
     source_gd_id, status, assigned_uav_id]

Status codes: 0=empty, 1=pending, 2=assigned, 3=completed, 4=failed
"""

import torch
from typing import Tuple


class TaskManagerGPU:
    """
    GPU-parallelized task generation and management.

    Generates tasks via Poisson process across all environments
    simultaneously using torch.poisson.
    """

    # Status codes
    EMPTY = 0
    PENDING = 1
    ASSIGNED = 2
    COMPLETED = 3
    FAILED = 4

    # Task tensor column indices
    COL_CPU = 0
    COL_DATA = 1
    COL_DEADLINE_REM = 2
    COL_PRIORITY = 3
    COL_SOURCE_GD = 4
    COL_STATUS = 5
    COL_ASSIGNED_UAV = 6
    NUM_COLS = 7

    def __init__(
        self,
        num_envs: int,
        num_ground_devices: int = 20,
        max_pending_tasks: int = 100,
        lambda_rate: float = 0.5,
        device: str = "cuda",
    ):
        """
        Args:
            num_envs: Number of parallel environments.
            num_ground_devices: Ground devices per environment.
            max_pending_tasks: Max tasks tracked per environment.
            lambda_rate: Average tasks/sec/device (Poisson λ).
            device: 'cuda' or 'cpu'.
        """
        self.num_envs = num_envs
        self.num_gd = num_ground_devices
        self.max_tasks = max_pending_tasks
        self.lambda_rate = lambda_rate
        self.device = torch.device(device)

        # QoS class parameters [cpu_megacycles, data_size_mb, deadline_seconds, priority]
        # Matches src/environments/task_manager.py exactly
        self.qos_params = torch.tensor([
            [20.0,   0.05, 0.05, 3.0],   # latency_critical  (30%)
            [800.0,  2.0,  0.5,  2.0],   # compute_intensive  (50%)
            [1000.0, 5.0,  1.0,  1.0],   # best_effort        (20%)
        ], device=self.device)

        self.qos_probs = torch.tensor([0.3, 0.5, 0.2], device=self.device)

        # Task buffer: [num_envs, max_tasks, NUM_COLS]
        self.tasks = torch.zeros(
            num_envs, max_pending_tasks, self.NUM_COLS,
            device=self.device
        )

        # Per-env counters
        self.task_counts = torch.zeros(num_envs, dtype=torch.long, device=self.device)
        self.total_generated = torch.zeros(num_envs, dtype=torch.long, device=self.device)
        self.total_completed = torch.zeros(num_envs, dtype=torch.long, device=self.device)
        self.total_failed = torch.zeros(num_envs, dtype=torch.long, device=self.device)

    def reset(self, env_ids: torch.Tensor = None):
        """Reset task buffers for specified environments."""
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)

        self.tasks[env_ids] = 0
        self.task_counts[env_ids] = 0
        self.total_generated[env_ids] = 0
        self.total_completed[env_ids] = 0
        self.total_failed[env_ids] = 0

    def generate_tasks(self, dt: float) -> torch.Tensor:
        """
        Generate new tasks via Poisson process for all envs.

        Args:
            dt: Timestep in seconds.

        Returns:
            Number of new tasks per env: [num_envs]
        """
        # Expected arrivals per device per timestep
        rate = self.lambda_rate * dt

        # Sample arrivals: [num_envs, num_gd]
        arrivals = torch.poisson(
            torch.full((self.num_envs, self.num_gd), rate, device=self.device)
        ).long()

        # Total new tasks per env
        new_per_env = arrivals.sum(dim=1)  # [num_envs]

        # For each env, insert new tasks into buffer
        for env_idx in range(self.num_envs):
            n_new = new_per_env[env_idx].item()
            if n_new == 0:
                continue

            # Find empty slots
            empty_mask = self.tasks[env_idx, :, self.COL_STATUS] == self.EMPTY
            empty_indices = empty_mask.nonzero(as_tuple=True)[0]

            n_insert = min(n_new, len(empty_indices))
            if n_insert == 0:
                continue

            slots = empty_indices[:n_insert]

            # Sample QoS classes
            qos_indices = torch.multinomial(
                self.qos_probs.expand(n_insert, -1), 1
            ).squeeze(-1)

            # Fill task data
            params = self.qos_params[qos_indices]  # [n_insert, 4]
            self.tasks[env_idx, slots, self.COL_CPU] = params[:, 0]
            self.tasks[env_idx, slots, self.COL_DATA] = params[:, 1]
            self.tasks[env_idx, slots, self.COL_DEADLINE_REM] = params[:, 2]
            self.tasks[env_idx, slots, self.COL_PRIORITY] = params[:, 3]
            self.tasks[env_idx, slots, self.COL_STATUS] = self.PENDING

            # Assign source ground device IDs (round-robin from arrivals)
            # Spread across devices that generated tasks
            device_ids = arrivals[env_idx].nonzero(as_tuple=True)[0]
            if len(device_ids) > 0:
                gd_assignments = device_ids[
                    torch.arange(n_insert, device=self.device) % len(device_ids)
                ]
                self.tasks[env_idx, slots, self.COL_SOURCE_GD] = gd_assignments.float()

            self.task_counts[env_idx] += n_insert
            self.total_generated[env_idx] += n_insert

        return new_per_env

    def tick_deadlines(self, dt: float):
        """
        Decrement deadline_remaining for all active tasks.
        Tasks that expire are marked FAILED.

        Args:
            dt: Timestep in seconds.
        """
        # Mask for active tasks (pending or assigned)
        active = (
            (self.tasks[:, :, self.COL_STATUS] == self.PENDING) |
            (self.tasks[:, :, self.COL_STATUS] == self.ASSIGNED)
        )

        # Decrement deadlines
        self.tasks[:, :, self.COL_DEADLINE_REM] -= dt * active.float()

        # Find expired tasks
        expired = active & (self.tasks[:, :, self.COL_DEADLINE_REM] <= 0)

        # Mark expired as failed
        self.tasks[:, :, self.COL_STATUS] = torch.where(
            expired,
            torch.full_like(self.tasks[:, :, self.COL_STATUS], self.FAILED),
            self.tasks[:, :, self.COL_STATUS],
        )

        # Count failures
        self.total_failed += expired.sum(dim=1)

    def get_uav_tasks(
        self, uav_id: int, max_tasks_per_uav: int = 5
    ) -> torch.Tensor:
        """
        Get tasks assigned to a specific UAV across all envs.

        Args:
            uav_id: UAV index.
            max_tasks_per_uav: Max tasks to return per env.

        Returns:
            Task features: [num_envs, max_tasks_per_uav, 4]
            (cpu_norm, data_norm, deadline_rem, priority_norm)
        """
        MAX_CPU = 1000.0
        MAX_DATA = 5.0
        MAX_PRIORITY = 3.0

        result = torch.zeros(
            self.num_envs, max_tasks_per_uav, 4, device=self.device
        )

        assigned_mask = (
            (self.tasks[:, :, self.COL_STATUS] == self.ASSIGNED) &
            (self.tasks[:, :, self.COL_ASSIGNED_UAV] == uav_id)
        )

        for env_idx in range(self.num_envs):
            indices = assigned_mask[env_idx].nonzero(as_tuple=True)[0]
            n = min(len(indices), max_tasks_per_uav)
            if n == 0:
                continue

            slots = indices[:n]
            result[env_idx, :n, 0] = self.tasks[env_idx, slots, self.COL_CPU] / MAX_CPU
            result[env_idx, :n, 1] = self.tasks[env_idx, slots, self.COL_DATA] / MAX_DATA
            result[env_idx, :n, 2] = self.tasks[env_idx, slots, self.COL_DEADLINE_REM]
            result[env_idx, :n, 3] = self.tasks[env_idx, slots, self.COL_PRIORITY] / MAX_PRIORITY

        return result

    def assign_to_nearest_uav(
        self,
        uav_positions: torch.Tensor,
        gd_positions: torch.Tensor,
        max_queue_size: int = 5,
    ):
        """
        Auto-assign pending tasks to nearest UAV that has capacity.

        Args:
            uav_positions: [num_envs, num_uavs, 3]
            gd_positions: [num_envs, num_gd, 3]
            max_queue_size: Max tasks per UAV.
        """
        num_uavs = uav_positions.shape[1]

        pending_mask = self.tasks[:, :, self.COL_STATUS] == self.PENDING

        for env_idx in range(self.num_envs):
            pending_indices = pending_mask[env_idx].nonzero(as_tuple=True)[0]
            if len(pending_indices) == 0:
                continue

            # Count current assignments per UAV
            assigned_mask_env = (
                self.tasks[env_idx, :, self.COL_STATUS] == self.ASSIGNED
            )
            uav_load = torch.zeros(num_uavs, device=self.device)
            for u in range(num_uavs):
                uav_load[u] = (
                    assigned_mask_env &
                    (self.tasks[env_idx, :, self.COL_ASSIGNED_UAV] == u)
                ).sum()

            for task_idx in pending_indices:
                gd_id = int(self.tasks[env_idx, task_idx, self.COL_SOURCE_GD].item())
                gd_pos = gd_positions[env_idx, gd_id]

                # Distance to each UAV
                dists = torch.norm(
                    uav_positions[env_idx] - gd_pos.unsqueeze(0), dim=1
                )

                # Mask out UAVs at capacity
                available = uav_load < max_queue_size
                if not available.any():
                    break

                dists[~available] = float("inf")
                best_uav = dists.argmin().item()

                self.tasks[env_idx, task_idx, self.COL_STATUS] = self.ASSIGNED
                self.tasks[env_idx, task_idx, self.COL_ASSIGNED_UAV] = best_uav
                uav_load[best_uav] += 1

    def complete_task(self, env_idx: int, task_slot: int, success: bool):
        """Mark a specific task as completed or failed."""
        if success:
            self.tasks[env_idx, task_slot, self.COL_STATUS] = self.COMPLETED
            self.total_completed[env_idx] += 1
        else:
            self.tasks[env_idx, task_slot, self.COL_STATUS] = self.FAILED
            self.total_failed[env_idx] += 1

    def clear_finished(self):
        """Clear completed/failed tasks from buffers (free slots)."""
        finished = (
            (self.tasks[:, :, self.COL_STATUS] == self.COMPLETED) |
            (self.tasks[:, :, self.COL_STATUS] == self.FAILED)
        )
        self.tasks[finished] = 0
