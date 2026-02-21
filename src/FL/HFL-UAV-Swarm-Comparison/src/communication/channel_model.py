"""
Communication Channel Model — Hybrid Al-Hourani + 3GPP TR 38.901 Air-to-Ground.

References:
- LoS probability: Al-Hourani et al. "Optimal LAP Altitude for Maximum Coverage"
  IEEE Wireless Communications Letters, Vol. 3, No. 6, 2014
- Path loss: 3GPP TR 38.901 V14.0.0 (2017-03) Section 7.4.1

V4 FIXES:
- Path loss constant: +32.4 (CORRECT, not -28.0)
- LoS formula: Al-Hourani model (NOT 3GPP)
- Edge case handling for numerical stability
"""

import numpy as np


class CommunicationChannel:
    """
    Hybrid Air-to-Ground channel model.

    References:
    - LoS probability: Al-Hourani et al. "Optimal LAP Altitude for Maximum Coverage"
      IEEE Wireless Communications Letters, Vol. 3, No. 6, 2014
    - Path loss: 3GPP TR 38.901 V14.0.0 (2017-03) Section 7.4.1

    V3 FIXES:
    - Path loss constant: +32.4 (CORRECT, not -28.0)
    - LoS formula: Al-Hourani model (NOT 3GPP)
    - Edge case handling for numerical stability
    """

    def __init__(self, environment: str = 'urban'):
        """
        Initialize channel model.

        Args:
            environment: 'suburban', 'urban', or 'dense_urban'
        """
        # Al-Hourani model parameters
        self.params = {
            'suburban': {
                'C1': 4.88,
                'C2': 0.43,
                'eta_LoS': 0.1,   # dB
                'eta_NLoS': 21.0  # dB
            },
            'urban': {
                'C1': 9.61,
                'C2': 0.28,
                'eta_LoS': 1.0,   # dB
                'eta_NLoS': 20.0  # dB
            },
            'dense_urban': {
                'C1': 12.08,
                'C2': 0.11,
                'eta_LoS': 1.6,   # dB
                'eta_NLoS': 23.0  # dB
            }
        }

        if environment not in self.params:
            raise ValueError(f"Environment must be one of {list(self.params.keys())}")

        self.env_params = self.params[environment]

        # Radio parameters
        self.frequency_ghz = 2.4   # 2.4 GHz (NOT Hz!)
        self.P_tx_dbm = 20         # 20 dBm = 0.1 W
        self.N_0_dbm = -100        # Noise power in dBm
        self.bandwidth_mhz = 5     # 5 MHz

    def calculate_snr(self, uav_pos: np.ndarray, gd_pos: np.ndarray) -> float:
        """
        Calculate SNR between UAV and ground device.

        Args:
            uav_pos: UAV position [x, y, z] in meters
            gd_pos: Ground device position [x, y, 0] in meters

        Returns:
            SNR in dB
        """
        # 1. Calculate distances
        horizontal_dist = np.sqrt((uav_pos[0] - gd_pos[0])**2 +
                                  (uav_pos[1] - gd_pos[1])**2)
        altitude = uav_pos[2]

        # Minimum distance to prevent division by zero
        distance_3d = max(1.0, np.sqrt(horizontal_dist**2 + altitude**2))  # meters

        # 2. Calculate elevation angle (in degrees)
        if horizontal_dist < 0.01:  # Essentially vertical
            elevation_angle = 90.0
        else:
            elevation_angle = np.arctan(altitude / horizontal_dist) * 180 / np.pi

        # 3. Calculate P_LoS using Al-Hourani model
        # Al-Hourani et al. (IEEE WCL 2014): P_LoS(θ) = 1 / (1 + a * exp(-b * (θ - a)))
        # where a = C1, b = C2, θ = elevation angle in degrees
        # NOTE: This is NOT from 3GPP TR 38.901, it's from Al-Hourani's research
        C1 = self.env_params['C1']
        C2 = self.env_params['C2']

        # The formula uses C1 in BOTH places (it's correct, not a typo)
        exponent = -C2 * (elevation_angle - C1)
        P_LoS = 1.0 / (1.0 + C1 * np.exp(exponent))

        # 4. Calculate path loss (3GPP TR 38.901 Section 7.4.1)
        # CRITICAL: PL = 20*log10(d_3D) + 20*log10(fc) + 32.4 + η
        # where fc is in GHz, d_3D is in meters
        # The constant is +32.4 (NOT -28.0)!
        # Reference: 3GPP TR 38.901 V14.0.0, Equation 7.4-1
        FSPL = 20 * np.log10(distance_3d) + 20 * np.log10(self.frequency_ghz) + 32.4

        PL_LoS = FSPL + self.env_params['eta_LoS']
        PL_NLoS = FSPL + self.env_params['eta_NLoS']

        # Weighted average path loss
        PL_avg = P_LoS * PL_LoS + (1 - P_LoS) * PL_NLoS

        # 5. Calculate SNR
        # SNR = P_tx - PL - N_0
        # Note: N_0_dbm is already negative (-100 dBm)
        SNR_dB = self.P_tx_dbm - PL_avg - self.N_0_dbm

        return SNR_dB

    def calculate_data_rate(self, snr_db: float) -> float:
        """
        Calculate data rate using Shannon capacity.

        Args:
            snr_db: SNR in dB

        Returns:
            Data rate in Mbps
        """
        # Convert SNR from dB to linear
        snr_linear = 10 ** (snr_db / 10)

        # Shannon capacity: C = B * log2(1 + SNR)
        # Since B is in MHz, result is in Mbps
        rate_mbps = self.bandwidth_mhz * np.log2(1 + snr_linear)

        # Handle extremely low data rates
        if rate_mbps < 1e-6:
            return 0.0

        return rate_mbps
