"""
Energy Consumption Model for UAVs.

Three components:
1. Flight energy (hovering + motion)
2. Computation energy (CPU cycles)
3. Communication energy (transmission/reception)

References:
- Flight: Typical quadrotor drone specifications
- Computation: "Energy Efficient Resource Allocation in UAV-Enabled
  Mobile Edge Computing Networks" (IEEE JSAC 2019)
- Communication: Typical UAV radio specifications
"""

import numpy as np


class EnergyModel:
    """
    Energy consumption model for UAVs.

    Three components:
    1. Flight energy (hovering + motion)
    2. Computation energy (CPU cycles)
    3. Communication energy (transmission/reception)
    """

    def __init__(self):
        # Flight parameters
        # Source: Typical quadrotor drone specifications
        self.P_hover = 50   # Watts (hovering power)
        self.alpha = 0.5    # Motion power coefficient (W/(m/s)²)

        # Computation parameters
        # Formula: E = κ * f² * C
        # where κ is the effective switched capacitance
        # This is a simplified model where we absorb the supply
        # voltage squared into κ. The full formula is E = C_eff * V² * f * N
        # but we use the approximation E ≈ κ * f² * N common in literature.
        # Source: IEEE JSAC 2019
        self.kappa = 1e-27  # Effective capacitance [J/Hz²/cycle]

        # Communication parameters
        # Source: Typical UAV radio specifications
        self.P_tx = 0.2  # Watts (transmission power)
        self.P_rx = 0.1  # Watts (reception power)

    def calculate_flight_energy(self, velocity: np.ndarray, dt: float) -> float:
        """
        Calculate flight energy for one timestep.

        Formula: E = (P_hover + α * ||v||²) * dt

        Args:
            velocity: 3D velocity vector [vx, vy, vz] in m/s
            dt: Timestep duration in seconds

        Returns:
            Energy in Joules
        """
        v_magnitude = np.linalg.norm(velocity)
        P_total = self.P_hover + self.alpha * (v_magnitude ** 2)
        return P_total * dt

    def calculate_computation_energy(self,
                                     total_cpu_cycles: float,
                                     cpu_frequency: float = 2.0e9) -> float:
        """
        Calculate energy to execute CPU cycles.

        Formula: E = κ * f² * C
        where:
        - κ = 1e-27 (effective switched capacitance parameter)
        - f = CPU frequency (Hz)
        - C = total CPU cycles (dimensionless)

        Args:
            total_cpu_cycles: Number of CPU cycles to execute
            cpu_frequency: CPU frequency in Hz (default 2 GHz)

        Returns:
            Energy in Joules
        """
        E_comp = self.kappa * (cpu_frequency ** 2) * total_cpu_cycles
        return E_comp

    def calculate_communication_energy(self,
                                        data_size_mb: float,
                                        data_rate_mbps: float,
                                        is_transmit: bool = True) -> float:
        """
        Calculate energy to transmit or receive data.

        Formula: E = P * t
        where t = (data_size_MB * 8 bits/byte) / (data_rate_Mbps)

        Args:
            data_size_mb: Data size in megabytes (MB)
            data_rate_mbps: Channel data rate in megabits per second (Mbps)
            is_transmit: True for TX, False for RX

        Returns:
            Energy in Joules
        """
        # Edge case: if data rate is essentially zero, transmission is impossible
        if data_rate_mbps < 1e-6:
            return 1e6  # 1 MJ (effectively infinite)

        # Time = (Data in megabytes * 8 bits/byte) / (Rate in Mbps)
        # Result: time in seconds
        time_seconds = (data_size_mb * 8) / data_rate_mbps

        # Energy = Power × Time
        power = self.P_tx if is_transmit else self.P_rx
        return power * time_seconds
