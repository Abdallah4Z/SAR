"""
Multi-UAV Task Offloading Environment for Federated Learning.

EXTENDED ACTION SPACE (MARL-compatible):
  action[0:3] = target_position [x, y, z]  — where to fly
  action[3]   = offload_decision            — what to do with tasks
  action[4]   = cpu_frequency               — how fast to compute

All actions are CONTINUOUS (Box space) for compatibility with
MADDPG, MAPPO, PPO, SAC. Discrete decisions are discretized
internally by the environment.

V4 CRITICAL FIXES:
- Task parameters now FEASIBLE (latency-critical can succeed)
- Rewards are PER-STEP (not cumulative)
- Battery depletion handled (tasks don't get stuck)

V3 FIXES:
- Correct timestep (0.01s = 10ms, not 100ms)
- Communication upload time included in latency
- Observation construction fixed (array, not list)
- Task priority actually used
- Collision detection implemented
- Instance RNG for reproducibility
"""

import gymnasium as gym
from gymnasium import spaces
import numpy as np
from typing import Dict, Tuple, List

from src.environments.drone_dynamics import UAV
from src.environments.task_manager import TaskGenerator, Task
from src.communication.channel_model import CommunicationChannel
from src.communication.energy_model import EnergyModel
from src.evaluation.metrics_tracker import MetricsTracker


class UAVSwarmEnv(gym.Env):
    """
    Multi-UAV task offloading environment.

    Extended Action Space (per UAV):
        [target_x, target_y, target_z, offload_decision, cpu_frequency]

    The MARL agent controls:
        1. MOVEMENT:    Where each UAV flies (target position)
        2. OFFLOADING:  What to do with the top task in queue
        3. COMPUTATION: How fast to run the CPU (energy-speed tradeoff)
    """

    metadata = {}

    # ──────────────────────────────────────────────
    #  Constants for action discretization
    # ──────────────────────────────────────────────
    OFFLOAD_PROCESS = 0   # Process task locally
    OFFLOAD_REJECT = 1    # Reject task (return to pending)
    OFFLOAD_SHARE = 2     # Share with nearest neighbor

    def __init__(self, config: Dict):
        super().__init__()

        # Validate config
        assert config['num_uavs'] > 0, "Must have at least one UAV"
        assert config['num_ground_devices'] > 0, "Must have at least one ground device"
        assert len(config['area_size']) == 3, "area_size must be [width, height, altitude]"
        assert config['episode_length'] > 0, "Episode length must be positive"
        assert config.get('dt', 0.01) > 0, "Timestep must be positive"

        # Configuration
        self.num_uavs = config['num_uavs']
        self.num_gd = config['num_ground_devices']
        self.area_size = tuple(config['area_size'])  # (width, height, altitude)
        self.episode_length = config['episode_length']
        self.dt = config.get('dt', 0.01)  # 10ms timestep

        # CPU frequency range (GHz)
        self.MIN_CPU_GHZ = 1.0
        self.MAX_CPU_GHZ = 3.0

        # Normalization constants
        self.MAX_CPU_MEGACYCLES = 1000
        self.MAX_DATA_SIZE_MB = 5
        self.MAX_PRIORITY = 3

        # Instance RNG (set in reset)
        self.rng = None

        # Components
        self.uavs: List[UAV] = []
        self.ground_devices: List[Dict] = []
        self.task_generator = None
        self.channel = CommunicationChannel('urban')
        self.energy_model = EnergyModel()
        self.metrics = MetricsTracker(self.num_uavs)

        # State
        self.current_step = 0
        self.current_time = 0.0
        self.pending_tasks: List[Task] = []

        # Define spaces
        self._create_spaces()

    def _create_spaces(self):
        """
        Define observation and action spaces.

        Action: [target_x, target_y, target_z, offload_decision, cpu_frequency]
                All continuous for MADDPG/MAPPO/PPO compatibility.

        Observation: [self_state(8) | local_tasks(20) | neighbors(4*(n-1))]
        """
        # ── Observation Space ──
        # self_state: [x, y, z, vx, vy, vz, battery%, cpu_freq_norm] = 8 dims
        # local_tasks: [cpu, data, deadline, priority] × 5 = 20 dims
        # neighbors: [x, y, z, battery%] × (n-1) = 4*(n-1) dims
        obs_dim = 8 + 20 + 4 * (self.num_uavs - 1)

        self.observation_space = spaces.Dict({
            i: spaces.Box(
                low=-np.inf, high=np.inf,
                shape=(obs_dim,), dtype=np.float32
            )
            for i in range(self.num_uavs)
        })

        # ── Action Space ──
        # 5 continuous dims: [target_x, target_y, target_z, offload, cpu_freq]
        # All normalized to [-1, 1] — environment rescales internally
        self.action_space = spaces.Dict({
            i: spaces.Box(
                low=-1.0, high=1.0,
                shape=(5,),
                dtype=np.float32
            )
            for i in range(self.num_uavs)
        })

    # ──────────────────────────────────────────────
    #  Action parsing
    # ──────────────────────────────────────────────

    def _parse_action(self, raw_action: np.ndarray) -> Dict:
        """
        Parse the raw continuous action into meaningful variables.

        Raw action shape: (5,) with values in [-1, 1]

        Returns dict with:
            target_position: [x, y, z] in meters
            offload_decision: int in {0, 1, 2}
            cpu_frequency_hz: float in [1e9, 3e9]
        """
        # 1. MOVEMENT: rescale [-1,1] → [0, area_size]
        target_x = (raw_action[0] + 1) / 2 * self.area_size[0]  # [0, width]
        target_y = (raw_action[1] + 1) / 2 * self.area_size[1]  # [0, height]
        target_z = (raw_action[2] + 1) / 2 * (self.area_size[2] - 10) + 10  # [10, altitude]
        target_position = np.array([target_x, target_y, target_z])

        # 2. OFFLOADING: discretize [-1,1] → {0, 1, 2}
        offload_raw = raw_action[3]
        if offload_raw < -0.33:
            offload_decision = self.OFFLOAD_PROCESS  # 0: process locally
        elif offload_raw < 0.33:
            offload_decision = self.OFFLOAD_REJECT   # 1: reject
        else:
            offload_decision = self.OFFLOAD_SHARE    # 2: share with neighbor

        # 3. CPU FREQUENCY: rescale [-1,1] → [1.0, 3.0] GHz → Hz
        cpu_freq_ghz = (raw_action[4] + 1) / 2 * (self.MAX_CPU_GHZ - self.MIN_CPU_GHZ) + self.MIN_CPU_GHZ
        cpu_frequency_hz = cpu_freq_ghz * 1e9

        return {
            'target_position': target_position,
            'offload_decision': offload_decision,
            'cpu_frequency_hz': cpu_frequency_hz,
            'cpu_frequency_ghz': cpu_freq_ghz,
        }

    # ──────────────────────────────────────────────
    #  Core Gymnasium API
    # ──────────────────────────────────────────────

    def reset(self, seed=None, options=None) -> Tuple[Dict, Dict]:
        """Reset environment to initial state."""
        super().reset(seed=seed)

        # Instance RNG
        if seed is not None:
            self.rng = np.random.RandomState(seed)
        else:
            self.rng = np.random.RandomState()

        # Reset reward tracking (V4: per-step rewards)
        self._prev_completed = {i: 0 for i in range(self.num_uavs)}
        self._prev_failed = {i: 0 for i in range(self.num_uavs)}

        # Reset UAVs at random positions
        self.uavs = []
        for i in range(self.num_uavs):
            pos = np.array([
                self.rng.uniform(0, self.area_size[0]),
                self.rng.uniform(0, self.area_size[1]),
                self.rng.uniform(20, self.area_size[2])
            ])
            uav = UAV(i, pos, rng=self.rng)
            self.uavs.append(uav)

        # Reset ground devices (stationary, on ground)
        self.ground_devices = []
        for gd_id in range(self.num_gd):
            gd = {
                'id': gd_id,
                'position': np.array([
                    self.rng.uniform(0, self.area_size[0]),
                    self.rng.uniform(0, self.area_size[1]),
                    0.0
                ])
            }
            self.ground_devices.append(gd)

        # Reset task generator
        self.task_generator = TaskGenerator(
            self.num_gd, lambda_rate=0.5, rng=self.rng
        )
        self.pending_tasks = []

        # Reset metrics & time
        self.metrics.reset()
        self.current_step = 0
        self.current_time = 0.0

        obs = self._get_observations()
        return obs, {}

    def step(self, actions: Dict[int, np.ndarray]) -> Tuple:
        """
        Execute one timestep.

        Args:
            actions: Dict mapping UAV id → np.ndarray of shape (5,)
                     [target_x, target_y, target_z, offload_decision, cpu_freq]
                     All values in [-1, 1], rescaled internally.
        """
        # Validate
        assert len(actions) == self.num_uavs, \
            f"Expected {self.num_uavs} actions, got {len(actions)}"

        # Parse all actions
        parsed_actions = {}
        for i, raw_action in actions.items():
            assert raw_action.shape == (5,), \
                f"Action for UAV {i} must be shape (5,), got {raw_action.shape}"
            parsed_actions[i] = self._parse_action(raw_action)

        # ── 1. MOVEMENT (MARL-controlled) ──
        # CRITICAL FIX: Track flight energy from UAV movement
        flight_energy_consumed = 0.0
        for i, uav in enumerate(self.uavs):
            target_pos = parsed_actions[i]['target_position']
            movement_info = uav.move_toward(target_pos, self.dt, self.area_size)
            flight_energy_consumed += movement_info['energy_consumed']

        # ── 2. CPU FREQUENCY (MARL-controlled) ──
        for i, uav in enumerate(self.uavs):
            uav.cpu_frequency = parsed_actions[i]['cpu_frequency_hz']

        # ── 3. COLLISION DETECTION ──
        collision_penalties = {}
        for uav in self.uavs:
            if uav.check_collision(self.uavs, min_separation=10.0):
                collision_penalties[uav.id] = -50
            else:
                collision_penalties[uav.id] = 0

        # ── 4. GENERATE NEW TASKS ──
        new_tasks = self.task_generator.generate_tasks(self.dt)
        for task in new_tasks:
            self.metrics.on_task_submit(task.id, self.current_time)
        self.pending_tasks.extend(new_tasks)

        # ── 5. AUTO-ASSIGN pending tasks to nearest UAV ──
        # Tasks still get assigned automatically (nearest UAV),
        # but the MARL agent decides what to DO with them via offload_decision
        self._assign_tasks()

        # ── 6. OFFLOADING DECISIONS (MARL-controlled) ──
        self._apply_offload_decisions(parsed_actions)

        # ── 7. PROCESS remaining tasks in queues ──
        self._process_queues()
        
        # ── 7.5 TRACK FLIGHT ENERGY ──
        # CRITICAL FIX: Track flight energy separately
        self.metrics.update_energy_components(flight=flight_energy_consumed)

        # ── 8. REWARDS ──
        rewards = self._calculate_rewards(collision_penalties)

        # ── 9. TIME UPDATE ──
        self.current_step += 1
        self.current_time += self.dt

        # ── 10. TERMINATION ──
        terminated = all(not uav.is_alive for uav in self.uavs)
        truncated = self.current_step >= self.episode_length

        # ── 11. OBSERVATIONS ──
        obs = self._get_observations()

        info = {
            'metrics': self.metrics.get_metrics(self.current_time),
            'alive_uavs': sum(uav.is_alive for uav in self.uavs),
            'pending_tasks': len(self.pending_tasks),
            'collisions': sum(1 for p in collision_penalties.values() if p < 0),
            'flight_energy': flight_energy_consumed  # CRITICAL FIX: Track flight energy
        }

        return obs, rewards, terminated, truncated, info

    # ──────────────────────────────────────────────
    #  Task Assignment (automatic nearest-UAV)
    # ──────────────────────────────────────────────

    def _assign_tasks(self):
        """Assign pending tasks to nearest available UAV."""
        # Remove expired tasks
        expired = [t for t in self.pending_tasks if t.deadline_time <= self.current_time]
        for task in expired:
            task.status = 'failed'
            task.completion_time = self.current_time
            self.metrics.on_task_complete(task.id, self.current_time, -1, False)
            self.pending_tasks.remove(task)

        # Assign remaining to nearest UAV
        for task in self.pending_tasks[:]:
            best_uav = None
            best_distance = float('inf')
            gd_pos = self.ground_devices[task.source_gd_id]['position']

            for uav in self.uavs:
                if not uav.can_accept_task():
                    continue
                distance = np.linalg.norm(uav.position - gd_pos)
                if distance < best_distance:
                    best_distance = distance
                    best_uav = uav

            if best_uav is not None:
                task.assigned_uav_id = best_uav.id
                task.status = 'assigned'
                best_uav.task_queue.append(task)
                self.pending_tasks.remove(task)

    # ──────────────────────────────────────────────
    #  Offloading Decisions (MARL-controlled)
    # ──────────────────────────────────────────────

    def _apply_offload_decisions(self, parsed_actions: Dict):
        """
        Apply MARL offloading decisions to each UAV's task queue.

        For each UAV, the agent chose one of:
            0 = PROCESS:  Keep the task, will be processed in _process_queues
            1 = REJECT:   Remove task from queue, return to pending
            2 = SHARE:    Transfer task to nearest neighbor UAV
        """
        for i, uav in enumerate(self.uavs):
            if not uav.is_alive or len(uav.task_queue) == 0:
                continue

            decision = parsed_actions[i]['offload_decision']
            task = uav.task_queue[0]  # Decision applies to top task

            if decision == self.OFFLOAD_PROCESS:
                # Keep task — will be processed in _process_queues()
                pass

            elif decision == self.OFFLOAD_REJECT:
                # Reject task — return to pending pool
                uav.task_queue.pop(0)
                task.status = 'pending'
                task.assigned_uav_id = None
                self.pending_tasks.append(task)

            elif decision == self.OFFLOAD_SHARE:
                # Share with nearest neighbor
                neighbor = uav.find_nearest_neighbor(self.uavs)
                if neighbor is not None:
                    uav.task_queue.pop(0)
                    task.assigned_uav_id = neighbor.id
                    task.status = 'assigned'
                    neighbor.task_queue.append(task)
                # If no neighbor available, keep task (fallback to process)

    # ──────────────────────────────────────────────
    #  Task Processing
    # ──────────────────────────────────────────────

    def _process_queues(self):
        """
        Process tasks in each UAV's queue using the
        MARL-controlled cpu_frequency.
        """
        for uav in self.uavs:
            if not uav.is_alive or len(uav.task_queue) == 0:
                continue

            # Sort by priority (highest first)
            uav.task_queue.sort(key=lambda t: t.priority, reverse=True)

            # Process top task
            task = uav.task_queue[0]

            # Computation time (uses MARL-controlled cpu_frequency)
            cpu_cycles = task.cpu_megacycles * 1e6
            computation_time = cpu_cycles / uav.cpu_frequency

            # Communication upload time
            gd_pos = self.ground_devices[task.source_gd_id]['position']
            snr = self.channel.calculate_snr(uav.position, gd_pos)
            data_rate = self.channel.calculate_data_rate(snr)

            if data_rate > 1e-6:
                upload_time = (task.data_size_mb * 8) / data_rate
            else:
                upload_time = 1e6

            total_processing_time = upload_time + computation_time

            # Energy (uses MARL-controlled cpu_frequency)
            E_comp = self.energy_model.calculate_computation_energy(
                cpu_cycles, uav.cpu_frequency
            )
            E_comm_rx = self.energy_model.calculate_communication_energy(
                task.data_size_mb, data_rate, is_transmit=False
            )
            E_comm_tx = self.energy_model.calculate_communication_energy(
                task.data_size_mb, data_rate, is_transmit=True
            )
            total_energy = E_comp + E_comm_rx + E_comm_tx

            if uav.battery >= total_energy:
                uav.battery -= total_energy
                self.metrics.update_energy(total_energy)
                
                # CRITICAL FIX: Track component energies separately
                if hasattr(self.metrics, 'update_energy_components'):
                    self.metrics.update_energy_components(
                        computation=E_comp,
                        communication=E_comm_rx + E_comm_tx,
                        flight=0.0  # Flight energy tracked separately in step()
                    )

                completion_time = self.current_time + total_processing_time
                success = completion_time <= task.deadline_time

                task.completion_time = completion_time
                task.status = 'completed' if success else 'failed'

                self.metrics.on_task_complete(
                    task.id, completion_time, uav.id, success
                )

                if success:
                    uav.tasks_completed += 1
                else:
                    uav.tasks_failed += 1

                uav.task_queue.pop(0)
            else:
                # Insufficient battery — task fails
                task.status = 'failed'
                task.completion_time = self.current_time
                self.metrics.on_task_complete(
                    task.id, self.current_time, uav.id, False
                )
                uav.tasks_failed += 1
                uav.task_queue.pop(0)

    # ──────────────────────────────────────────────
    #  Rewards (per-step, V4)
    # ──────────────────────────────────────────────

    def _calculate_rewards(self, collision_penalties: Dict[int, float]) -> Dict[int, float]:
        """Calculate per-step reward for each UAV."""
        rewards = {}

        # Fairness across UAVs
        tasks_per_uav = np.array([uav.tasks_completed for uav in self.uavs])
        mean_tasks = np.mean(tasks_per_uav)
        fairness_cv = np.std(tasks_per_uav) / mean_tasks if mean_tasks > 0 else 0

        for i, uav in enumerate(self.uavs):
            if not uav.is_alive:
                rewards[i] = -100
                continue

            # Per-step DELTA (not cumulative)
            completed_this_step = uav.tasks_completed - self._prev_completed[i]
            failed_this_step = uav.tasks_failed - self._prev_failed[i]
            self._prev_completed[i] = uav.tasks_completed
            self._prev_failed[i] = uav.tasks_failed

            r_success = completed_this_step * 10.0
            r_failure = failed_this_step * (-5.0)
            r_battery = (uav.battery / uav.battery_capacity) * 0.1
            r_fairness = -fairness_cv * 0.5
            r_collision = collision_penalties[i]

            rewards[i] = r_success + r_failure + r_battery + r_fairness + r_collision

        return rewards

    # ──────────────────────────────────────────────
    #  Observations
    # ──────────────────────────────────────────────

    def _get_observations(self) -> Dict[int, np.ndarray]:
        """
        Get observation for each UAV.

        Per-UAV observation vector:
            self_state (8):  [x, y, z, vx, vy, vz, battery%, cpu_freq_norm]
            local_tasks (20): 5 tasks × [cpu, data, deadline_remaining, priority]
            neighbors (4*(n-1)): (n-1) UAVs × [x, y, z, battery%]
        """
        observations = {}

        for i, uav in enumerate(self.uavs):
            # Self state (8 dims) — now includes cpu_frequency
            battery_pct = uav.battery / uav.battery_capacity
            cpu_freq_norm = (uav.cpu_frequency / 1e9 - self.MIN_CPU_GHZ) / \
                            (self.MAX_CPU_GHZ - self.MIN_CPU_GHZ)
            self_state = np.array([
                uav.position[0], uav.position[1], uav.position[2],
                uav.velocity[0], uav.velocity[1], uav.velocity[2],
                battery_pct,
                cpu_freq_norm
            ], dtype=np.float32)

            # Local tasks (20 dims)
            task_features = []
            for j in range(5):
                if j < len(uav.task_queue):
                    task = uav.task_queue[j]
                    deadline_rem = max(0.0, task.deadline_time - self.current_time)
                    features = [
                        task.cpu_megacycles / self.MAX_CPU_MEGACYCLES,
                        task.data_size_mb / self.MAX_DATA_SIZE_MB,
                        deadline_rem,
                        task.priority / self.MAX_PRIORITY
                    ]
                else:
                    features = [0, 0, 0, 0]
                task_features.extend(features)
            task_array = np.array(task_features, dtype=np.float32)

            # Neighbor states
            neighbor_parts = []
            for other in self.uavs:
                if other.id == uav.id:
                    continue
                other_batt = other.battery / other.battery_capacity
                neighbor_parts.append(np.array([
                    other.position[0], other.position[1], other.position[2],
                    other_batt
                ], dtype=np.float32))

            if neighbor_parts:
                neighbor_array = np.concatenate(neighbor_parts)
            else:
                neighbor_array = np.array([], dtype=np.float32)

            observations[i] = np.concatenate([
                self_state, task_array, neighbor_array
            ])

        return observations
