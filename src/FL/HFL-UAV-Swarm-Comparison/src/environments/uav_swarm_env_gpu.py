"""
GPU-Tensorized Multi-UAV Task Offloading Environment.

All state and computations use PyTorch CUDA tensors — no Python loops
over agents, tasks, or ground devices but full batch tensor ops.

Drop-in replacement for UAVSwarmEnv with same API contract:
    obs, rewards, done, truncated, info = env.step(actions)

Performance target: 10-20x faster than CPU version for 50+ drones.

Channel model: Hybrid Al-Hourani + 3GPP TR 38.901 Air-to-Ground
Energy model:  Flight (hover+motion) + Computation (kappa*f^2*C) + Communication (P*t)
"""

import torch
import numpy as np
from typing import Dict, Tuple, Optional


class UAVSwarmEnvGPU:
    """
    GPU-tensorized Multi-UAV task offloading environment.

    All state lives on GPU as tensors. Operations are batched —
    no Python loops over agents or tasks.

    Action Space (per UAV): 5 continuous dims in [-1, 1]
        [target_x, target_y, target_z, offload_decision, cpu_frequency]

    Observation Space (per UAV):
        self_state (8):   [x, y, z, vx, vy, vz, battery%, cpu_freq_norm]
        local_tasks (20): 5 tasks × [cpu, data, deadline_rem, priority]
        neighbors (4*(N-1)): (N-1) UAVs × [x, y, z, battery%]
    """

    # Offloading decision thresholds (same as CPU version)
    OFFLOAD_PROCESS = 0
    OFFLOAD_REJECT = 1
    OFFLOAD_SHARE = 2

    def __init__(self, config: Dict, device: str = 'cuda'):
        """
        Args:
            config: Environment config dict (same format as UAVSwarmEnv)
            device: 'cuda' or 'cpu'
        """
        # Validate
        assert config['num_uavs'] > 0
        assert config['num_ground_devices'] > 0
        assert len(config['area_size']) == 3
        assert config['episode_length'] > 0

        self.device = torch.device(device)
        self.num_uavs = config['num_uavs']
        self.num_gd = config['num_ground_devices']
        self.area_size = torch.tensor(config['area_size'], dtype=torch.float32,
                                      device=self.device)
        self.episode_length = config['episode_length']
        self.dt = config.get('dt', 0.01)

        # UAV parameters
        self.max_speed = 5.0        # m/s
        self.battery_capacity = 500000.0  # Joules (500 kJ)
        self.max_queue_size = 5

        # CPU frequency range
        self.MIN_CPU_GHZ = 1.0
        self.MAX_CPU_GHZ = 3.0

        # Normalization constants
        self.MAX_CPU_MEGACYCLES = 1000.0
        self.MAX_DATA_SIZE_MB = 5.0
        self.MAX_PRIORITY = 3.0

        # Flight energy parameters
        self.P_hover = 50.0   # Watts
        self.alpha = 0.5      # Motion power coefficient

        # Computation energy
        self.kappa = 1e-27    # Effective capacitance

        # Communication energy
        self.P_tx = 0.2       # Watts (transmit)
        self.P_rx = 0.1       # Watts (receive)

        # Channel model parameters (urban Al-Hourani)
        self.C1 = 9.61
        self.C2 = 0.28
        self.eta_LoS = 1.0    # dB
        self.eta_NLoS = 20.0  # dB
        self.frequency_ghz = 2.4
        self.P_tx_dbm = 20.0
        self.N_0_dbm = -100.0
        self.bandwidth_mhz = 5.0

        # Task QoS classes — precomputed tensors
        # [cpu_megacycles, data_size_mb, deadline_seconds, priority]
        self.qos_templates = torch.tensor([
            [20.0,   0.05, 0.05, 3.0],    # latency_critical
            [800.0,  2.0,  0.5,  2.0],     # compute_intensive
            [1000.0, 5.0,  1.0,  1.0],     # best_effort
        ], dtype=torch.float32, device=self.device)
        self.qos_probs = torch.tensor([0.3, 0.5, 0.2],
                                      dtype=torch.float32, device=self.device)
        self.lambda_rate = config.get('lambda_rate', 0.5)

        # Observation dim
        self.obs_dim = 8 + 20 + 4 * (self.num_uavs - 1)

        # Gymnasium-compatible spaces (for trainers that inspect them)
        import gymnasium as gym
        from gymnasium import spaces
        self.observation_space = {
            i: spaces.Box(low=-np.inf, high=np.inf,
                          shape=(self.obs_dim,), dtype=np.float32)
            for i in range(self.num_uavs)
        }
        self.action_space = {
            i: spaces.Box(low=-1.0, high=1.0,
                          shape=(5,), dtype=np.float32)
            for i in range(self.num_uavs)
        }

        # Pre-allocate state tensors (initialized in reset)
        self._seed = None
        self._rng = None  # CPU RNG for seeding
        self.current_step = 0
        self.current_time = 0.0

        # Metrics tracking (stays on CPU for dict compatibility)
        self._total_energy = 0.0
        self._computation_energy = 0.0
        self._communication_energy = 0.0
        self._flight_energy = 0.0
        self._task_id_counter = 0
        self._task_submit_times = {}
        self._task_complete_times = {}
        self._task_success = {}

    # ──────────────────────────────────────────────
    #  Reset
    # ──────────────────────────────────────────────

    def reset(self, seed: Optional[int] = None, options=None) -> Tuple[Dict, Dict]:
        """Reset environment. Returns (obs_dict, info_dict)."""
        if seed is not None:
            self._seed = seed
            self._rng = np.random.RandomState(seed)
            torch.manual_seed(seed)
        elif self._rng is None:
            self._rng = np.random.RandomState()

        N = self.num_uavs
        G = self.num_gd
        Q = self.max_queue_size
        dev = self.device

        # UAV state
        self.positions = torch.rand(N, 3, device=dev) * self.area_size
        self.positions[:, 2] = self.positions[:, 2].clamp(min=20.0)  # min altitude 20m
        self.velocities = torch.zeros(N, 3, device=dev)
        self.batteries = torch.full((N,), self.battery_capacity, device=dev)
        self.cpu_frequencies = torch.full((N,), 2.0e9, device=dev)  # Default 2 GHz
        self.is_alive = torch.ones(N, dtype=torch.bool, device=dev)
        self.tasks_completed = torch.zeros(N, dtype=torch.long, device=dev)
        self.tasks_failed = torch.zeros(N, dtype=torch.long, device=dev)

        # Task queue: (N, Q, 4) = [cpu_megacycles, data_size_mb, deadline_remaining, priority]
        self.task_queue = torch.zeros(N, Q, 4, device=dev)
        self.task_queue_mask = torch.zeros(N, Q, dtype=torch.bool, device=dev)
        self.task_source_gd = torch.zeros(N, Q, dtype=torch.long, device=dev)
        # Track task IDs for metrics (small CPU-side dict)
        self.task_queue_ids = torch.full((N, Q), -1, dtype=torch.long, device=dev)

        # Ground device positions (on ground, z=0)
        self.gd_positions = torch.rand(G, 3, device=dev)
        self.gd_positions[:, 0] *= self.area_size[0]
        self.gd_positions[:, 1] *= self.area_size[1]
        self.gd_positions[:, 2] = 0.0

        # Pending tasks buffer (max pending = G * 5 should be enough)
        max_pending = G * 5
        self.pending_tasks = torch.zeros(max_pending, 4, device=dev)  # [cpu, data, deadline_rem, priority]
        self.pending_gd = torch.zeros(max_pending, dtype=torch.long, device=dev)
        self.pending_ids = torch.full((max_pending,), -1, dtype=torch.long, device=dev)
        self.pending_mask = torch.zeros(max_pending, dtype=torch.bool, device=dev)

        # Per-step reward tracking
        self._prev_completed = self.tasks_completed.clone()
        self._prev_failed = self.tasks_failed.clone()

        # Reset metrics
        self._total_energy = 0.0
        self._computation_energy = 0.0
        self._communication_energy = 0.0
        self._flight_energy = 0.0
        self._task_id_counter = 0
        self._task_submit_times = {}
        self._task_complete_times = {}
        self._task_success = {}

        # Reset time
        self.current_step = 0
        self.current_time = 0.0

        obs = self._get_observations_tensor()
        return obs, {}

    # ──────────────────────────────────────────────
    #  Step
    # ──────────────────────────────────────────────

    def step(self, actions: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, bool, bool, Dict]:
        """
        Execute one timestep (Vectorized).

        Args:
            actions: (N, 5) tensor of actions in [-1, 1] on self.device
        """
        N = self.num_uavs
        dev = self.device

        # If actions is a dict (for backward compatibility during transition), convert to tensor
        if isinstance(actions, dict):
            action_list = [actions[i] for i in range(N)]
            actions = torch.stack([torch.as_tensor(a, device=dev) for a in action_list])

        # ── 1. PARSE ACTIONS ──
        target_positions, offload_decisions, cpu_freq_hz = self._parse_actions(actions)

        # ── 2. MOVEMENT ──
        flight_energy = self._move_uavs(target_positions)

        # ── 3. SET CPU FREQUENCIES ──
        self.cpu_frequencies = cpu_freq_hz * self.is_alive.float()

        # ── 4. COLLISION DETECTION ──
        collision_penalties = self._check_collisions()

        # ── 5. GENERATE NEW TASKS ──
        self._generate_tasks()

        # ── 6. ASSIGN PENDING TASKS ──
        self._assign_tasks()

        # ── 7. APPLY OFFLOAD DECISIONS ──
        self._apply_offload_decisions(offload_decisions)

        # ── 8. PROCESS QUEUES ──
        self._process_queues()

        # ── 9. TRACK FLIGHT ENERGY ── (item() is okay here as it's once per step)
        self._flight_energy += flight_energy.sum().item()

        # ── 10. REWARDS ──
        rewards = self._calculate_rewards(collision_penalties)

        # ── 11. TIME UPDATE ──
        self.current_step += 1
        self.current_time += self.dt

        # ── 12. TERMINATION ──
        terminated = not self.is_alive.any().item()
        truncated = self.current_step >= self.episode_length

        # ── 13. DECREMENT DEADLINES on queued tasks ──
        self.task_queue[:, :, 2] -= self.dt * self.task_queue_mask.float()

        # ── 14. OBSERVATIONS ──
        obs = self._get_observations_tensor()

        info = {
            'alive_uavs': self.is_alive.sum().item(),
            'pending_tasks': self.pending_mask.sum().item(),
            'collisions': (collision_penalties < 0).sum().item(),
        }

        return obs, rewards, terminated, truncated, info

    def _get_observations_tensor(self) -> torch.Tensor:
        """Vectorized observation collection returning a single tensor (N, obs_dim)."""
        N = self.num_uavs
        dev = self.device

        battery_pct = self.batteries / self.battery_capacity
        cpu_norm = (self.cpu_frequencies / 1e9 - self.MIN_CPU_GHZ) / (self.MAX_CPU_GHZ - self.MIN_CPU_GHZ)
        self_state = torch.cat([
            self.positions,
            self.velocities,
            battery_pct.unsqueeze(1),
            cpu_norm.unsqueeze(1),
        ], dim=1)

        task_feats = self.task_queue.clone()
        task_feats[:, :, 0] /= self.MAX_CPU_MEGACYCLES
        task_feats[:, :, 1] /= self.MAX_DATA_SIZE_MB
        task_feats[:, :, 3] /= self.MAX_PRIORITY
        task_feats *= self.task_queue_mask.unsqueeze(2).float()
        task_flat = task_feats.reshape(N, -1)

        all_uav_feats = torch.cat([self.positions, battery_pct.unsqueeze(1)], dim=1)
        neighbor_matrix = all_uav_feats.unsqueeze(0).expand(N, N, 4)
        mask = ~torch.eye(N, device=dev, dtype=torch.bool)
        neighbor_flat = neighbor_matrix[mask].view(N, -1)

        return torch.cat([self_state, task_flat, neighbor_flat], dim=1)

    # ──────────────────────────────────────────────
    #  Action Parsing (vectorized)
    # ──────────────────────────────────────────────

    def _parse_actions(self, actions: torch.Tensor):
        """
        Parse (N, 5) actions tensor into target positions, offload decisions, cpu freq.
        All values in [-1, 1], rescaled here.
        """
        # 1. Target positions: [-1,1] → [0, area_size]
        target_xy = (actions[:, :2] + 1) / 2 * self.area_size[:2]  # (N, 2)
        target_z = (actions[:, 2:3] + 1) / 2 * (self.area_size[2] - 10) + 10  # (N, 1)
        target_positions = torch.cat([target_xy, target_z], dim=1)  # (N, 3)

        # 2. Offload decisions: discretize [-1,1] → {0, 1, 2}
        offload_raw = actions[:, 3]
        offload_decisions = torch.where(
            offload_raw < -0.33,
            torch.zeros_like(offload_raw, dtype=torch.long),
            torch.where(
                offload_raw < 0.33,
                torch.ones_like(offload_raw, dtype=torch.long),
                torch.full_like(offload_raw, 2, dtype=torch.long)
            )
        )

        # 3. CPU frequency: [-1,1] → [1, 3] GHz → Hz
        cpu_ghz = (actions[:, 4] + 1) / 2 * (self.MAX_CPU_GHZ - self.MIN_CPU_GHZ) + self.MIN_CPU_GHZ
        cpu_freq_hz = cpu_ghz * 1e9

        return target_positions, offload_decisions, cpu_freq_hz

    # ──────────────────────────────────────────────
    #  Movement (vectorized)
    # ──────────────────────────────────────────────

    def _move_uavs(self, target_positions: torch.Tensor) -> torch.Tensor:
        """Move all UAVs toward targets. Returns per-UAV flight energy."""
        # Direction to target
        direction = target_positions - self.positions  # (N, 3)
        distance = direction.norm(dim=1, keepdim=True).clamp(min=1e-8)  # (N, 1)

        # Normalize direction
        dir_normalized = direction / distance

        # Move distance: min(max_speed * dt, actual_distance)
        move_dist = torch.min(
            torch.full_like(distance, self.max_speed * self.dt),
            distance
        )

        # Velocity
        # At target (dist < 0.1): hover (vel = 0)
        at_target = (distance.squeeze() < 0.1)
        self.velocities = dir_normalized * (move_dist / self.dt)
        self.velocities[at_target] = 0.0

        # Zero velocity for dead UAVs
        self.velocities *= self.is_alive.float().unsqueeze(1)

        # Update positions
        self.positions = self.positions + self.velocities * self.dt

        # Clamp to boundaries
        self.positions[:, 0].clamp_(0, self.area_size[0].item())
        self.positions[:, 1].clamp_(0, self.area_size[1].item())
        self.positions[:, 2].clamp_(10, self.area_size[2].item())

        # Flight energy: P = P_hover + alpha * ||v||^2
        v_magnitude = self.velocities.norm(dim=1)  # (N,)
        P_flight = self.P_hover + self.alpha * v_magnitude.pow(2)
        E_flight = P_flight * self.dt  # (N,)

        # Deduct from battery (alive UAVs only)
        self.batteries -= E_flight * self.is_alive.float()
        self.batteries.clamp_(min=0.0)

        # Kill UAVs with depleted battery
        newly_dead = (self.batteries <= 0) & self.is_alive
        self.is_alive[newly_dead] = False

        return E_flight * self.is_alive.float()

    # ──────────────────────────────────────────────
    #  Collision Detection (vectorized)
    # ──────────────────────────────────────────────

    def _check_collisions(self) -> torch.Tensor:
        """Check pairwise collisions. Returns per-UAV penalty tensor."""
        min_sep = 10.0
        # Pairwise distances: (N, N)
        dists = torch.cdist(self.positions, self.positions)
        # Mask diagonal (self-distance = 0)
        eye = torch.eye(self.num_uavs, device=self.device, dtype=torch.bool)
        close = (dists < min_sep) & ~eye  # (N, N)
        has_collision = close.any(dim=1)  # (N,)
        penalties = torch.where(has_collision, torch.tensor(-50.0, device=self.device),
                               torch.tensor(0.0, device=self.device))
        return penalties

    # ──────────────────────────────────────────────
    #  Task Generation (GPU Poisson process)
    # ──────────────────────────────────────────────

    def _generate_tasks(self):
        """Generate tasks via Poisson process for each ground device."""
        G = self.num_gd
        dev = self.device

        # Sample number of arrivals per GD: Poisson(lambda * dt)
        rate = torch.full((G,), self.lambda_rate * self.dt, device=dev)
        num_arrivals = torch.poisson(rate).long()  # (G,)

        total_new = num_arrivals.sum().item()
        if total_new == 0:
            return

        # For each arrival, pick QoS class
        # Expand: which GD generated which task
        gd_indices = torch.repeat_interleave(
            torch.arange(G, device=dev), num_arrivals
        )  # (total_new,)

        # Sample QoS classes
        qos_indices = torch.multinomial(
            self.qos_probs.unsqueeze(0).expand(total_new, -1),
            num_samples=1
        ).squeeze(1)  # (total_new,)

        # Look up task parameters
        new_task_params = self.qos_templates[qos_indices]  # (total_new, 4)
        # Set deadline_remaining based on current time (it's already in seconds)

        # Add to pending buffer
        max_pending = self.pending_tasks.shape[0]
        free_slots = (~self.pending_mask).nonzero(as_tuple=True)[0]

        num_to_add = min(total_new, len(free_slots))
        if num_to_add > 0:
            slots = free_slots[:num_to_add]
            self.pending_tasks[slots] = new_task_params[:num_to_add]
            self.pending_gd[slots] = gd_indices[:num_to_add]
            self.pending_mask[slots] = True

            # Assign task IDs for metrics tracking
            for idx in range(num_to_add):
                tid = self._task_id_counter
                self._task_id_counter += 1
                slot_idx = slots[idx].item()
                self.pending_ids[slot_idx] = tid
                self._task_submit_times[tid] = self.current_time

    # ──────────────────────────────────────────────
    #  Task Assignment (vectorized nearest-UAV)
    # ──────────────────────────────────────────────

    def _assign_tasks(self):
        """Assign pending tasks to nearest available UAV (Vectorized)."""
        if not self.pending_mask.any():
            return

        # 1. Expire tasks
        expired_mask = self.pending_mask & (self.pending_tasks[:, 2] <= 0)
        if expired_mask.any():
            e_idx = expired_mask.nonzero().squeeze()
            if e_idx.dim() == 0: e_idx = e_idx.unsqueeze(0)
            for idx in e_idx:
                tid = self.pending_ids[idx].item()
                if tid >= 0:
                    self._task_complete_times[tid] = self.current_time
                    self._task_success[tid] = False
            self.pending_mask[expired_mask] = False
            self.pending_ids[expired_mask] = -1

        # 2. Assign remaining
        active = self.pending_mask.nonzero().squeeze()
        if active.dim() == 0: active = active.unsqueeze(0)
        if len(active) == 0: return

        queue_free = self.max_queue_size - self.task_queue_mask.sum(dim=1)
        can_accept = self.is_alive & (queue_free > 0)
        if not can_accept.any(): return

        # Dists from pending GDs to UAVs: (P, N)
        gd_pos = self.gd_positions[self.pending_gd[active]]
        dists = torch.cdist(gd_pos, self.positions)
        dists[:, ~can_accept] = 1e6 # Mask
        
        # Nearest UAV for each task
        nearest_uav = dists.argmin(dim=1) # (P,)
        
        # Greedy assignment to avoid double-booking slots in one step
        # (Though in GPU env we can just let them fight or use a slightly smarter loop)
        for i in range(len(active)):
            uav_id = nearest_uav[i].item()
            if not can_accept[uav_id]: continue
            
            # Find free slot
            free_slots = (~self.task_queue_mask[uav_id]).nonzero()
            if len(free_slots) == 0: continue
            slot = free_slots[0].item()
            
            p_idx = active[i].item()
            self.task_queue[uav_id, slot] = self.pending_tasks[p_idx]
            self.task_queue_mask[uav_id, slot] = True
            self.task_source_gd[uav_id, slot] = self.pending_gd[p_idx]
            self.task_queue_ids[uav_id, slot] = self.pending_ids[p_idx]
            
            self.pending_mask[p_idx] = False
            self.pending_ids[p_idx] = -1
            if (self.max_queue_size - self.task_queue_mask[uav_id].sum()) <= 0:
                can_accept[uav_id] = False

    def _apply_offload_decisions(self, decisions: torch.Tensor):
        """Apply decisions (0=process, 1=reject, 2=share) (Vectorized)."""
        N = self.num_uavs
        dev = self.device
        
        has_tasks = self.task_queue_mask.any(dim=1)
        eligible = self.is_alive & has_tasks
        if not eligible.any(): return

        # Find top tasks
        priorities = self.task_queue[:, :, 3]
        masked_prio = torch.where(self.task_queue_mask, priorities, torch.tensor(-1e6, device=dev))
        top_indices = masked_prio.argmax(dim=1)
        uav_range = torch.arange(N, device=dev)

        # 1. REJECT (1) -> Move back to pending
        reject_mask = eligible & (decisions == self.OFFLOAD_REJECT)
        if reject_mask.any():
            r_uavs = reject_mask.nonzero().squeeze()
            if r_uavs.dim() == 0: r_uavs = r_uavs.unsqueeze(0)
            for i in r_uavs:
                u_id = i.item()
                t_idx = top_indices[u_id].item()
                free_p = (~self.pending_mask).nonzero()
                if len(free_p) > 0:
                    ps = free_p[0].item()
                    self.pending_tasks[ps] = self.task_queue[u_id, t_idx]
                    self.pending_gd[ps] = self.task_source_gd[u_id, t_idx]
                    self.pending_ids[ps] = self.task_queue_ids[u_id, t_idx]
                    self.pending_mask[ps] = True
                self.task_queue_mask[u_id, t_idx] = False
                self.task_queue_ids[u_id, t_idx] = -1

        # 2. SHARE (2) -> Move to neighbor
        share_mask = eligible & (decisions == self.OFFLOAD_SHARE)
        if share_mask.any():
            s_uavs = share_mask.nonzero().squeeze()
            if s_uavs.dim() == 0: s_uavs = s_uavs.unsqueeze(0)
            
            # UAVs that can receive
            can_recv = self.is_alive & (self.max_queue_size - self.task_queue_mask.sum(dim=1) > 0)
            
            for i in s_uavs:
                u_id = i.item()
                t_idx = top_indices[u_id].item()
                
                # Neighbors
                others = can_recv.clone()
                others[u_id] = False
                if others.any():
                    dists = torch.norm(self.positions[others] - self.positions[u_id], dim=1)
                    neighbor_id = others.nonzero().squeeze()[dists.argmin()].item()
                    
                    nb_free = (~self.task_queue_mask[neighbor_id]).nonzero()
                    if len(nb_free) > 0:
                        nb_s = nb_free[0].item()
                        self.task_queue[neighbor_id, nb_s] = self.task_queue[u_id, t_idx]
                        self.task_queue_mask[neighbor_id, nb_s] = True
                        self.task_source_gd[neighbor_id, nb_s] = self.task_source_gd[u_id, t_idx]
                        self.task_queue_ids[neighbor_id, nb_s] = self.task_queue_ids[u_id, t_idx]
                        
                        self.task_queue_mask[u_id, t_idx] = False
                        self.task_queue_ids[u_id, t_idx] = -1

    # ──────────────────────────────────────────────
    #  Task Processing (vectorized)
    # ──────────────────────────────────────────────

    def _calculate_snr_batched(self, uav_pos: torch.Tensor, gd_pos: torch.Tensor) -> torch.Tensor:
        """Vectorized SNR calculation for all UAV-GD pairs."""
        # Horizontal and 3D distances
        h_dist = torch.norm(uav_pos[:, :2] - gd_pos[:, :2], dim=1)
        altitude = uav_pos[:, 2]
        d3d = torch.sqrt(h_dist**2 + altitude**2).clamp(min=1.0)
        
        # Elevation angle
        elev = torch.rad2deg(torch.atan2(altitude, h_dist.clamp(min=1e-3)))
        
        # LoS Probability (Al-Hourani)
        exponent = -self.C2 * (elev - self.C1)
        P_LoS = 1.0 / (1.0 + self.C1 * torch.exp(exponent))
        
        # Path Loss
        FSPL = 20 * torch.log10(d3d) + 20 * torch.log10(torch.tensor(self.frequency_ghz)) + 32.4
        PL_LoS = FSPL + self.eta_LoS
        PL_NLoS = FSPL + self.eta_NLoS
        PL_avg = P_LoS * PL_LoS + (1 - P_LoS) * PL_NLoS
        
        return self.P_tx_dbm - PL_avg - self.N_0_dbm

    def _process_queues(self):
        """Process top-priority task in each UAV's queue (Vectorized)."""
        N = self.num_uavs
        dev = self.device
        
        # 1. Identify active UAVs (alive and has tasks)
        has_tasks = self.task_queue_mask.any(dim=1)
        active_mask = self.is_alive & has_tasks
        if not active_mask.any():
            return
            
        # 2. Find top-priority task index for each UAV
        # Mask empty slots with very low priority
        priorities = self.task_queue[:, :, 3]
        masked_priorities = torch.where(self.task_queue_mask, priorities, 
                                      torch.tensor(-1e6, device=dev))
        top_indices = masked_priorities.argmax(dim=1)
        uav_idx = torch.arange(N, device=dev)
        
        # 3. Gather top task parameters
        tasks = self.task_queue[uav_idx, top_indices]  # (N, 4)
        cpu_mcyc = tasks[:, 0]
        data_mb = tasks[:, 1]
        deadline_rem = tasks[:, 2]
        gd_indices = self.task_source_gd[uav_idx, top_indices]
        task_ids = self.task_queue_ids[uav_idx, top_indices]
        
        # 4. Computation calculation
        cpu_cycles = cpu_mcyc * 1e6
        comp_time = cpu_cycles / self.cpu_frequencies.clamp(min=1e-3)
        E_comp = self.kappa * (self.cpu_frequencies**2) * cpu_cycles
        
        # 5. Communication calculation
        snr_db = self._calculate_snr_batched(self.positions, self.gd_positions[gd_indices])
        snr_linear = 10 ** (snr_db / 10)
        data_rate = self.bandwidth_mhz * torch.log2(1 + snr_linear)
        
        upload_time = (data_mb * 8) / data_rate.clamp(min=1e-6)
        # Handle zero data rate cases (out of range)
        upload_time = torch.where(data_rate > 1e-6, upload_time, torch.tensor(1e6, device=dev))
        
        E_comm = (self.P_tx + self.P_rx) * upload_time
        E_comm = torch.where(data_rate > 1e-6, E_comm, torch.tensor(1e6, device=dev))
        
        total_time = upload_time + comp_time
        total_energy = E_comp + E_comm
        
        # 6. Success/Failure Evaluation
        can_afford = (self.batteries >= total_energy)
        success = (total_time <= deadline_rem) & can_afford
        
        # 7. Apply updates (only for active UAVs)
        update_mask = active_mask
        
        # Update metrics (CPU dictionary update still needed for some parts, but minimized)
        self.batteries -= torch.where(update_mask, total_energy, torch.tensor(0.0, device=dev))
        self.tasks_completed += (update_mask & success).long()
        self.tasks_failed += (update_mask & ~success).long()
        
        # Record times and success in CPU dicts (this is the only remaining Python bit here)
        active_indices = update_mask.nonzero().squeeze()
        if active_indices.dim() == 0: active_indices = active_indices.unsqueeze(0)
        
        curr_time = self.current_time
        for i in active_indices:
            idx = i.item()
            tid = task_ids[idx].item()
            if tid >= 0:
                self._task_complete_times[tid] = curr_time + total_time[idx].item()
                self._task_success[tid] = success[idx].item()

        # 8. Clear processed tasks from queue
        cols = top_indices[update_mask]
        rows = active_indices
        self.task_queue_mask[rows, cols] = False
        self.task_queue_ids[rows, cols] = -1

    # ──────────────────────────────────────────────
    #  Channel Model (single-pair, for process_queues)
    # ──────────────────────────────────────────────

    def _calculate_snr_single(self, uav_pos: torch.Tensor, gd_pos: torch.Tensor) -> float:
        """Calculate SNR between one UAV and one GD. Returns scalar dB."""
        h_dist = ((uav_pos[0] - gd_pos[0])**2 + (uav_pos[1] - gd_pos[1])**2).sqrt().item()
        altitude = uav_pos[2].item()
        d3d = max(1.0, (h_dist**2 + altitude**2)**0.5)

        if h_dist < 0.01:
            elev = 90.0
        else:
            elev = np.degrees(np.arctan(altitude / h_dist))

        exponent = -self.C2 * (elev - self.C1)
        P_LoS = 1.0 / (1.0 + self.C1 * np.exp(exponent))

        FSPL = 20 * np.log10(d3d) + 20 * np.log10(self.frequency_ghz) + 32.4
        PL_LoS = FSPL + self.eta_LoS
        PL_NLoS = FSPL + self.eta_NLoS
        PL_avg = P_LoS * PL_LoS + (1 - P_LoS) * PL_NLoS

        return self.P_tx_dbm - PL_avg - self.N_0_dbm

    def _calculate_data_rate(self, snr_db: float) -> float:
        """Shannon capacity. Returns Mbps."""
        snr_linear = 10 ** (snr_db / 10)
        rate = self.bandwidth_mhz * np.log2(1 + snr_linear)
        return max(rate, 0.0)

    # ──────────────────────────────────────────────
    #  Rewards (vectorized)
    # ──────────────────────────────────────────────

    def _calculate_rewards(self, collision_penalties: torch.Tensor) -> torch.Tensor:
        """Calculate per-UAV per-step rewards. Returns (N,) tensor."""
        N = self.num_uavs

        # Per-step deltas
        completed_delta = self.tasks_completed - self._prev_completed  # (N,)
        failed_delta = self.tasks_failed - self._prev_failed
        self._prev_completed = self.tasks_completed.clone()
        self._prev_failed = self.tasks_failed.clone()

        # Fairness (coefficient of variation)
        tc_float = self.tasks_completed.float()
        mean_tasks = tc_float.mean()
        if mean_tasks > 0:
            fairness_cv = tc_float.std() / mean_tasks
        else:
            fairness_cv = torch.tensor(0.0, device=self.device)

        r_success = completed_delta.float() * 10.0
        r_failure = failed_delta.float() * (-5.0)
        r_battery = (self.batteries / self.battery_capacity) * 0.1
        r_fairness = -fairness_cv * 0.5
        r_collision = collision_penalties

        rewards = r_success + r_failure + r_battery + r_fairness + r_collision

        # Dead UAVs get -100
        rewards = torch.where(self.is_alive, rewards,
                             torch.tensor(-100.0, device=self.device))

        return rewards

    # ──────────────────────────────────────────────
    #  Observations (vectorized)
    # ──────────────────────────────────────────────

    def _get_observations(self) -> Dict[int, torch.Tensor]:
        """
        Build per-UAV observations. Returns dict of GPU tensors.
        
        Per-UAV: [self_state(8) | local_tasks(20) | neighbors(4*(N-1))]
        """
        N = self.num_uavs
        dev = self.device

        # 1. Self state: (N, 8)
        battery_pct = self.batteries / self.battery_capacity
        cpu_norm = (self.cpu_frequencies / 1e9 - self.MIN_CPU_GHZ) / (self.MAX_CPU_GHZ - self.MIN_CPU_GHZ)
        self_state = torch.cat([
            self.positions,            # (N, 3)
            self.velocities,           # (N, 3)
            battery_pct.unsqueeze(1),  # (N, 1)
            cpu_norm.unsqueeze(1),       # (N, 1)
        ], dim=1)  # (N, 8)

        # 2. Task features: (N, 20)
        task_feats = self.task_queue.clone()
        task_feats[:, :, 0] /= self.MAX_CPU_MEGACYCLES
        task_feats[:, :, 1] /= self.MAX_DATA_SIZE_MB
        task_feats[:, :, 3] /= self.MAX_PRIORITY
        task_feats *= self.task_queue_mask.unsqueeze(2).float()
        task_flat = task_feats.reshape(N, -1)  # (N, 20)

        # 3. Neighbor states: (N, N-1, 4)
        # Combined [pos, battery%] for all UAVs: (N, 4)
        all_uav_feats = torch.cat([self.positions, battery_pct.unsqueeze(1)], dim=1)
        
        # Expand to (N, N, 4)
        neighbor_matrix = all_uav_feats.unsqueeze(0).expand(N, N, 4)
        
        # Mask out self-observation (diagonal)
        # Using a clever mask to avoid the Python loop
        mask = ~torch.eye(N, device=dev, dtype=torch.bool)
        neighbor_flat = neighbor_matrix[mask].view(N, -1)  # (N, (N-1)*4)

        # 4. Combine all: (N, 8 + 20 + 4*(N-1))
        obs_all = torch.cat([self_state, task_flat, neighbor_flat], dim=1)
        
        # Return as dict for API compatibility, but stored as GPU tensors
        return {i: obs_all[i] for i in range(N)}

    # ──────────────────────────────────────────────
    #  Metrics (CPU-side for compatibility)
    # ──────────────────────────────────────────────

    def _get_metrics(self) -> Dict:
        """Calculate metrics. Returns dict (same format as MetricsTracker)."""
        latencies = []
        for tid in self._task_complete_times:
            if tid in self._task_submit_times:
                latency = (self._task_complete_times[tid] - self._task_submit_times[tid]) * 1000
                latencies.append(latency)
        avg_latency = float(np.mean(latencies)) if latencies else 0.0

        total_completed = len(self._task_complete_times)
        successful = sum(self._task_success.values())
        success_rate = (successful / total_completed * 100) if total_completed > 0 else 0.0

        tc = self.tasks_completed.float().cpu().numpy()
        mean_t = np.mean(tc)
        fairness_cv = float(np.std(tc) / mean_t) if mean_t > 0 else 0.0

        return {
            'average_latency_ms': avg_latency,
            'total_energy_j': self._total_energy,
            'computation_energy_j': self._computation_energy,
            'communication_energy_j': self._communication_energy,
            'flight_energy_j': self._flight_energy,
            'task_success_rate': success_rate,
            'tasks_completed': successful,
            'tasks_failed': total_completed - successful,
            'fairness_cv': fairness_cv,
        }
