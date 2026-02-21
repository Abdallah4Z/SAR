"""
GPU-Tensorized Energy Model for UAVs.

Mirrors the logic of src/communication/energy_model.py but operates
entirely on PyTorch GPU tensors for batched environments.

Three components:
1. Flight energy:       E = (P_hover + α * ||v||²) * dt
2. Computation energy:  E = κ * f² * C
3. Communication energy: E = P * (data_MB * 8 / rate_Mbps)
"""

import torch


class EnergyModelGPU:
    """
    GPU-parallelized energy consumption model.

    References:
    - Flight: Typical quadrotor specs
    - Computation: IEEE JSAC 2019
    - Communication: Typical UAV radio specs
    """

    def __init__(self, device: str = "cuda"):
        self.device = torch.device(device)

        # Flight parameters
        self.P_hover = 50.0   # Watts (hovering power)
        self.alpha = 0.5      # Motion power coefficient W/(m/s)²

        # Computation parameters
        # E = κ * f² * C  (κ absorbs V² from DVFS model)
        self.kappa = 1e-27    # Effective capacitance [J/Hz²/cycle]

        # Communication parameters
        self.P_tx = 0.2       # Watts (transmission power)
        self.P_rx = 0.1       # Watts (reception power)

    def flight_energy(
        self, velocity: torch.Tensor, dt: float
    ) -> torch.Tensor:
        """
        Calculate flight energy for one timestep.

        E = (P_hover + α * ||v||²) * dt

        Args:
            velocity: [..., 3] velocity vectors in m/s
            dt: Timestep in seconds

        Returns:
            Energy in Joules, shape [...]
        """
        v_mag_sq = (velocity ** 2).sum(dim=-1)  # ||v||²
        P_total = self.P_hover + self.alpha * v_mag_sq
        return P_total * dt

    def computation_energy(
        self,
        cpu_cycles: torch.Tensor,
        cpu_frequency: torch.Tensor,
    ) -> torch.Tensor:
        """
        Calculate computation energy.

        E = κ * f² * C

        Args:
            cpu_cycles: Total CPU cycles (not megacycles — already converted)
            cpu_frequency: CPU frequency in Hz

        Returns:
            Energy in Joules, same shape as inputs
        """
        return self.kappa * (cpu_frequency ** 2) * cpu_cycles

    def communication_energy(
        self,
        data_size_mb: torch.Tensor,
        data_rate_mbps: torch.Tensor,
        is_transmit: bool = True,
    ) -> torch.Tensor:
        """
        Calculate communication energy.

        E = P * t, where t = (data_MB * 8) / rate_Mbps

        Args:
            data_size_mb: Data size in megabytes
            data_rate_mbps: Channel data rate in Mbps
            is_transmit: True for TX, False for RX

        Returns:
            Energy in Joules
        """
        power = self.P_tx if is_transmit else self.P_rx

        # Handle zero/tiny rates (return large penalty energy)
        safe_rate = data_rate_mbps.clamp(min=1e-6)
        time_seconds = (data_size_mb * 8.0) / safe_rate

        # Cap at 1MJ for effectively infinite energy at zero rate
        time_seconds = torch.where(
            data_rate_mbps < 1e-6,
            torch.full_like(time_seconds, 1e6 / power),
            time_seconds,
        )

        return power * time_seconds
