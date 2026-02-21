"""
GPU-Tensorized Communication Channel Model.

Mirrors the logic of src/communication/channel_model.py but operates
entirely on PyTorch GPU tensors for batched environments.

Model: Hybrid Al-Hourani (LoS probability) + 3GPP TR 38.901 (path loss)
"""

import torch
import math


class ChannelModelGPU:
    """
    GPU-parallelized Air-to-Ground channel model.

    References:
    - LoS probability: Al-Hourani et al. IEEE WCL 2014
    - Path loss: 3GPP TR 38.901 V14.0.0 Section 7.4.1
    """

    def __init__(self, environment: str = "urban", device: str = "cuda"):
        """
        Args:
            environment: 'suburban', 'urban', or 'dense_urban'
            device: 'cuda' or 'cpu'
        """
        self.device = torch.device(device)

        # Al-Hourani model parameters
        params = {
            "suburban": {"C1": 4.88, "C2": 0.43, "eta_LoS": 0.1, "eta_NLoS": 21.0},
            "urban": {"C1": 9.61, "C2": 0.28, "eta_LoS": 1.0, "eta_NLoS": 20.0},
            "dense_urban": {"C1": 12.08, "C2": 0.11, "eta_LoS": 1.6, "eta_NLoS": 23.0},
        }

        if environment not in params:
            raise ValueError(f"Environment must be one of {list(params.keys())}")

        p = params[environment]
        self.C1 = p["C1"]
        self.C2 = p["C2"]
        self.eta_LoS = p["eta_LoS"]
        self.eta_NLoS = p["eta_NLoS"]

        # Radio parameters (same as original)
        self.frequency_ghz = 2.4
        self.P_tx_dbm = 20.0
        self.N_0_dbm = -100.0
        self.bandwidth_mhz = 5.0

        # Precompute log10(freq)
        self._log10_freq = math.log10(self.frequency_ghz)

    def calculate_snr(
        self, uav_pos: torch.Tensor, gd_pos: torch.Tensor
    ) -> torch.Tensor:
        """
        Calculate SNR between UAV(s) and ground device(s).

        Args:
            uav_pos: [..., 3] UAV positions in meters
            gd_pos:  [..., 3] ground device positions in meters

        Returns:
            SNR in dB, same batch shape as input (without last dim)
        """
        # Distances
        diff = uav_pos - gd_pos
        horizontal_dist = torch.norm(diff[..., :2], dim=-1)  # [...]
        altitude = uav_pos[..., 2].clamp(min=0.1)  # [...]

        distance_3d = torch.norm(diff, dim=-1).clamp(min=1.0)  # [...]

        # Elevation angle in degrees
        elevation_angle = torch.atan2(altitude, horizontal_dist.clamp(min=0.01))
        elevation_angle = elevation_angle * (180.0 / math.pi)

        # P_LoS — Al-Hourani model
        exponent = -self.C2 * (elevation_angle - self.C1)
        P_LoS = 1.0 / (1.0 + self.C1 * torch.exp(exponent))

        # Path loss — 3GPP TR 38.901
        # PL = 20*log10(d_3D) + 20*log10(fc) + 32.4 + η
        FSPL = 20.0 * torch.log10(distance_3d) + 20.0 * self._log10_freq + 32.4

        PL_LoS = FSPL + self.eta_LoS
        PL_NLoS = FSPL + self.eta_NLoS

        # Weighted average path loss
        PL_avg = P_LoS * PL_LoS + (1.0 - P_LoS) * PL_NLoS

        # SNR = P_tx - PL - N_0
        SNR_dB = self.P_tx_dbm - PL_avg - self.N_0_dbm

        return SNR_dB

    def calculate_data_rate(self, snr_db: torch.Tensor) -> torch.Tensor:
        """
        Calculate data rate using Shannon capacity.

        Args:
            snr_db: SNR in dB, any shape

        Returns:
            Data rate in Mbps, same shape
        """
        snr_linear = 10.0 ** (snr_db / 10.0)

        # Shannon: C = B * log2(1 + SNR)
        rate_mbps = self.bandwidth_mhz * torch.log2(1.0 + snr_linear)

        # Floor tiny rates to zero
        rate_mbps = torch.where(
            rate_mbps < 1e-6,
            torch.zeros_like(rate_mbps),
            rate_mbps,
        )

        return rate_mbps
