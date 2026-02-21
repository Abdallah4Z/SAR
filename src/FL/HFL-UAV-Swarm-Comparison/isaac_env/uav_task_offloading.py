import torch
import torch.distributions as D
from torch.func import vmap

from omni_drones.envs.isaac_env import AgentSpec, IsaacEnv, List, Optional
from omni_drones.robots.drone import MultirotorBase
from omni_drones.utils.kit import prim_utils
from omni_drones.utils.scene import design_scene
from omni_drones.utils.torch import cpos, off_diag, others, make_cells, euler_to_quaternion

from tensordict.tensordict import TensorDict, TensorDictBase
from torchrl.data import CompositeSpec, UnboundedContinuousTensorSpec, DiscreteTensorSpec, BoundedTensorSpec

from .task_manager_gpu import TaskManagerGPU
from .channel_model_gpu import ChannelModelGPU
from .energy_model_gpu import EnergyModelGPU

class UAVTaskOffloadingEnv(IsaacEnv):
    """
    UAV Task Offloading Environment on OmniDrones.
    
    This environment simulates a swarm of UAVs serving as mobile edge computing nodes.
    Each UAV receives computation tasks from ground devices (simulated via Poisson process)
    and must decide whether to process them locally, reject them, or share them with neighbors.
    
    Physics and drone control are handled by OmniDrones (MultirotorBase).
    """
    def __init__(self, cfg, headless):
        # Env parameters
        self.num_uavs = cfg.task.num_uavs
        self.area_size = torch.tensor(cfg.task.area_size, device=cfg.sim.device)
        self.dt = cfg.sim.dt
        
        # Modules
        self.task_manager = TaskManagerGPU(
            num_envs=cfg.env.num_envs,
            num_uavs=self.num_uavs,
            lambda_rate=cfg.task.lambda_rate,
            device=cfg.sim.device
        )
        self.channel_model = ChannelModelGPU(device=cfg.sim.device)
        self.energy_model = EnergyModelGPU(device=cfg.sim.device)
        
        super().__init__(cfg, headless)
        
        # Initialize drone views
        self.drone.initialize()
        self.init_poses = self.drone.get_world_poses(clone=True)
        
        # Buffers for task logic
        self.battery = torch.full((self.num_envs, self.num_uavs), 100000.0, device=self.device) # 100k Joules
        self.battery_capacity = 100000.0
        self.cpu_frequency = torch.full((self.num_envs, self.num_uavs), 2.0e9, device=self.device) # 2 GHz
        
        # Metrics trackers
        self.alpha = 0.8
        self.stats = self.observation_spec["stats"].zero()

    def _design_scene(self) -> Optional[List[str]]:
        # Spawning ground plane
        design_scene()
        
        # Spawning drones
        drone_model_cfg = self.cfg.task.drone_model
        self.drone, self.controller = MultirotorBase.make(
            drone_model_cfg.name, drone_model_cfg.controller
        )
        
        # Initial positions: spread drones in the area at height 30m-50m
        init_positions = torch.zeros((self.num_uavs, 3), device=self.device)
        init_positions[:, 0] = torch.linspace(10, self.area_size[0]-10, self.num_uavs)
        init_positions[:, 1] = self.area_size[1] / 2.0
        init_positions[:, 2] = 40.0
        
        # Spawn them in the template env
        self.drone.spawn(translations=init_positions)
        
        # Ground devices are handled logically (as Poisson event generators)
        # We don't necessarily need to spawn USD actors for 100s of ground devices
        # unless visual feedback is required.
        
        return ["/World/defaultGroundPlane"]

    def _set_specs(self):
        # Observation Spec
        # Self: pos(3), vel(3), battery(1), cpu(1) = 8
        # Tasks: 5 tasks * 4 features = 20
        # Neighbors: (n-1) * 4 features (pos_rel, battery) = 4*(n-1)
        
        drone_state_dim = 8 # Custom self state
        task_dim = 20
        neighbor_dim = 4 * (self.num_uavs - 1)
        obs_dim = drone_state_dim + task_dim + neighbor_dim
        
        observation_spec = CompositeSpec({
            "observation": UnboundedContinuousTensorSpec((self.num_uavs, obs_dim)),
        }).to(self.device)
        
        # Action Spec
        # Drone controller action (thrusts or velocity) + offload(1) + cpu(1)
        # Note: MultirotorBase.action_spec is for rotors. 
        # If we use action_transform=velocity, the input is [vx, vy, vz, yaw_rate]
        # For simplicity, we assume the agent outputs [vx, vy, vz, offload, cpu]
        
        # We define a custom action spec that includes our logic actions
        # The OmniDrones train.py might need adjustment if we change the structure,
        # but IsaacEnv expects self.action_spec to match the agent output.
        
        # We use a combined spec
        self.action_spec = CompositeSpec({
            "agents": {
                "action": BoundedTensorSpec(
                    low=-1.0, 
                    high=1.0, 
                    shape=(self.num_uavs, 5), # [vx, vy, vz, offload, cpu]
                    device=self.device
                )
            }
        }).expand(self.num_envs).to(self.device)
        
        self.observation_spec = CompositeSpec({
            "agents": {
                "observation": observation_spec.expand(self.num_envs),
            },
            "stats": CompositeSpec({
                "return": UnboundedContinuousTensorSpec(self.num_uavs),
                "episode_len": UnboundedContinuousTensorSpec(1),
                "tasks_completed": UnboundedContinuousTensorSpec(self.num_uavs),
                "energy_consumed": UnboundedContinuousTensorSpec(self.num_uavs),
            }).expand(self.num_envs).to(self.device)
        }).to(self.device)
        
        self.reward_spec = CompositeSpec({
            "agents": {
                "reward": UnboundedContinuousTensorSpec((self.num_uavs, 1))
            }
        }).expand(self.num_envs).to(self.device)
        
        # Register agent spec
        self.agent_spec["drone"] = AgentSpec(
            "drone",
            self.num_uavs,
            observation_key=("agents", "observation"),
            action_key=("agents", "action"),
            reward_key=("agents", "reward")
        )

    def _reset_idx(self, env_ids: torch.Tensor):
        self.drone._reset_idx(env_ids)
        
        # Random initial positions inside the area
        num_resets = len(env_ids)
        pos = torch.zeros((num_resets, self.num_uavs, 3), device=self.device)
        pos[..., 0] = torch.rand((num_resets, self.num_uavs), device=self.device) * self.area_size[0]
        pos[..., 1] = torch.rand((num_resets, self.num_uavs), device=self.device) * self.area_size[1]
        pos[..., 2] = torch.rand((num_resets, self.num_uavs), device=self.device) * 30.0 + 20.0 # 20-50m
        
        # Standard orientation (no rotation)
        rot = torch.zeros((num_resets, self.num_uavs, 4), device=self.device)
        rot[..., 0] = 1.0 # Scalar part if [w, x, y, z] or similar
        
        self.drone.set_world_poses(pos + self.envs_positions[env_ids].unsqueeze(1), rot, env_ids)
        self.drone.set_velocities(torch.zeros(num_resets, self.num_uavs, 6, device=self.device), env_ids)
        
        # Reset internal modules
        self.task_manager.reset_idx(env_ids)
        self.battery[env_ids] = self.battery_capacity
        self.cpu_frequency[env_ids] = 2.0e9
        
        self.stats[env_ids] = 0.0

    def _pre_sim_step(self, tensordict: TensorDictBase):
        # Extract actions
        actions = tensordict[("agents", "action")] # (num_envs, num_drones, 5)
        
        # Movement actions: [0:3] -> vx, vy, vz
        # We pass these to the controller. Note: OmniDrones might expect different format
        # if using AttitudeController vs PositionController.
        # Here we assume a simple velocity control provided by OmniDrones wrapper
        # but since we are inside _pre_sim_step, we apply to drone directly or via controller.
        
        # If we use MultirotorBase.apply_action, it expects rotor thrusts.
        # But we want velocity control. OmniDrones' train.py usually handles this 
        # via 'action_transform' in TransformedEnv.
        # If so, tensordict[("agents", "action")] ALREADY contains rotor thrusts 
        # after being transformed by the Attitude/Position controller.
        
        # Wait, if we define the spec as 5 dims, the train.py transform might fail 
        # if it expects exact rotor count.
        
        # Strategy: We apply drone actions (first 3 or N rotors) and then our custom logic.
        # For this implementation, let's assume raw actions if no transform is specified.
        # But better: Use OmniDrones' built-in velocity controller if possible.
        
        # For now, let's just use the first 4 dims for drone if N_rotors=4.
        drone_actions = actions[..., :4] # Assuming 4 rotors for Firefly
        self.drone.apply_action(drone_actions)
        
        # Task offloading actions: index 3 or 4
        offload_actions = actions[..., 3]
        cpu_actions = actions[..., 4]
        
        # Map [-1, 1] to discrete decisions [0, 1, 2]
        # Reject: < -0.33, Process: [-0.33, 0.33], Share: > 0.33
        offload_decisions = torch.bucketize(offload_actions, torch.tensor([-0.33, 0.33], device=self.device))
        
        # Map [-1, 1] to CPU freq [1.0, 3.0] GHz
        self.cpu_frequency = (cpu_actions + 1.0) * (2.0e9 - 1.0e9) / 2.0 + 1.0e9
        
        # 1. Generate new tasks
        self.task_manager.generate_tasks(self.dt)
        
        # 2. Assign tasks from pending pool to drones (those with empty local queues)
        # This mirrors the logic of assignment in the original env.
        self.task_manager.assign_tasks()
        
        # 3. Apply offload decisions to the TOP task of each drone's local queue
        self.task_manager.apply_offload_decisions(offload_decisions)
        
        # 4. Process queues (Computation, Communication, Energy)
        # Use simple distance-based path loss for energy calculation
        drone_pos = self.drone.pos
        info = self.task_manager.process_queues(
            dt=self.dt,
            cpu_frequencies=self.cpu_frequency,
            drone_positions=drone_pos,
            channel_model=self.channel_model,
            energy_model=self.energy_model
        )
        
        # Update batteries
        self.battery -= info["energy_consumed"]
        self.battery.clamp_(min=0.0)
        
        # Save info for reward calculation
        self.last_step_info = info

    def _compute_state_and_obs(self) -> TensorDictBase:
        # Get physics state
        drone_state = self.drone.get_state() # (num_envs, num_uavs, ...)
        pos = self.drone.pos
        vel = self.drone.vel
        
        # Self state: pos(3), vel(3), battery_pct(1), cpu_norm(1)
        battery_pct = (self.battery / self.battery_capacity).unsqueeze(-1)
        cpu_norm = ((self.cpu_frequency / 1e9 - 1.0) / 2.0).unsqueeze(-1)
        
        # We need to map global pos to relative if needed, but current spec says absolute area pos
        self_state = torch.cat([pos, vel, battery_pct, cpu_norm], dim=-1)
        
        # Local tasks: 5 tasks * 4 features = 20
        task_obs = self.task_manager.get_task_observations() # (num_envs, num_uavs, 20)
        
        # Neighbor states: relative position + battery
        # (num_envs, num_uavs, num_uavs, 3) relative distances
        rel_pos = vmap(cpos)(pos, pos)
        rel_pos_others = vmap(off_diag)(rel_pos) # excludes self
        
        battery_others = vmap(others)(battery_pct) # excludes self
        
        neighbor_state = torch.cat([rel_pos_others, battery_others], dim=-1).flatten(2) # (num_envs, num_uavs, 4*(n-1))
        
        obs = torch.cat([self_state, task_obs, neighbor_state], dim=-1)
        
        return TensorDict({
            "agents": {
                "observation": obs,
            },
            "stats": self.stats.clone()
        }, self.batch_size)

    def _compute_reward_and_done(self) -> TensorDictBase:
        # Task rewards
        completed = self.last_step_info["completed_tasks"]
        failed = self.last_step_info["failed_tasks"]
        energy = self.last_step_info["energy_consumed"]
        
        # Basic rewards
        r_success = completed.float() * 10.0
        r_failure = failed.float() * (-5.0)
        r_battery = (self.battery / self.battery_capacity) * 0.1
        
        # Fairness (CV of tasks completed)
        # We can track per-drone cumulative stats
        self.stats["tasks_completed"].add_(completed.float())
        self.stats["energy_consumed"].add_(energy)
        
        total_completed = self.stats["tasks_completed"]
        mean_comp = total_completed.mean(dim=1, keepdim=True)
        std_comp = total_completed.std(dim=1, keepdim=True)
        fairness_cv = std_comp / (mean_comp + 1e-6)
        r_fairness = -fairness_cv * 0.5
        
        # Collisions
        # drone_pdist = num_envs, num_uavs, num_uavs-1
        rel_pos = vmap(cpos)(self.drone.pos, self.drone.pos)
        dist = torch.norm(rel_pos, dim=-1)
        dist = vmap(off_diag)(dist)
        
        collision_mask = (dist < 1.0).any(dim=-1) # 1 meter safe zone
        r_collision = collision_mask.float() * (-100.0)
        
        reward = r_success + r_failure + r_battery + r_fairness + r_collision.unsqueeze(-1)
        
        # Termination
        # 1. Any drone crashes (z < 0.2)
        # 2. Battery empty
        # 3. Time out
        crashed = (self.drone.pos[..., 2] < 0.2).any(dim=1, keepdim=True)
        battery_dead = (self.battery <= 0).any(dim=1, keepdim=True)
        
        terminated = crashed | battery_dead
        truncated = (self.progress_buf >= self.max_episode_length).unsqueeze(-1)
        
        self.stats["return"].add_(reward)
        self.stats["episode_len"][:] = self.progress_buf.float().unsqueeze(-1)
        
        return TensorDict({
            "agents": {
                "reward": reward.unsqueeze(-1),
            },
            "done": terminated | truncated,
            "terminated": terminated,
            "truncated": truncated,
        }, self.batch_size)
