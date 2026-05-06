"""
Virtual Sensor Bus for M130 — generates multi-rate noisy measurements from true state.

Generates:
  - IMU (accel + gyro)  @ imu_rate_hz  (default 200 Hz)
  - GPS (pos + vel)     @ gps_rate_hz  (default 5 Hz)
  - Baro (altitude)     @ baro_rate_hz (default 25 Hz)

Output: 13-element measurement vector aligned with MHE h(x):
  [gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z,
   baro_alt, gps_x, gps_y, gps_z, gps_vn, gps_ve, gps_vd]
"""

import logging
from dataclasses import dataclass, field

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class SensorPacket:
    t: float
    y_meas: np.ndarray          # [13] full measurement vector for MHE
    imu_fresh: bool = True
    gps_fresh: bool = False
    baro_fresh: bool = False


class SensorBus:
    """Generate virtual sensor measurements from true simulation state."""

    def __init__(self, estimation_cfg: dict, rng_seed: int = None):
        sensors = estimation_cfg.get("sensors", {})

        # Rates
        self._imu_rate = float(sensors.get("imu_rate_hz", 200.0))
        self._gps_rate = float(sensors.get("gps_rate_hz", 5.0))
        self._baro_rate = float(sensors.get("baro_rate_hz", 25.0))

        # GPS dropout window [t_start, t_end] — GPS reports stale during this window
        dropout = sensors.get("gps_dropout_window", None)
        self._gps_dropout = tuple(dropout) if dropout else None

        # Noise stds
        self._accel_std = float(sensors.get("accel_noise_std", 0.08))
        self._gyro_std = float(sensors.get("gyro_noise_std", 0.003))
        self._gps_pos_std = float(sensors.get("gps_pos_noise_std", 2.5))
        self._gps_vel_std = float(sensors.get("gps_vel_noise_std", 0.6))
        self._baro_std = float(sensors.get("baro_noise_std", 1.2))

        # Biases (fixed per run)
        self._accel_bias_std = float(sensors.get("accel_bias_std", 0.1))
        self._gyro_bias_std = float(sensors.get("gyro_bias_std", 0.005))

        # Delays
        self._gps_delay = float(sensors.get("gps_delay_s", 0.08))
        self._baro_delay = float(sensors.get("baro_delay_s", 0.03))

        # RNG
        self._rng = np.random.default_rng(rng_seed)

        # Generate fixed biases for this run
        self._accel_bias = self._rng.normal(0, self._accel_bias_std, 3)
        self._gyro_bias = self._rng.normal(0, self._gyro_bias_std, 3)

        # Rate tracking
        self._last_imu_t = -1.0
        self._last_gps_t = -1.0
        self._last_baro_t = -1.0

        # Hold-last-sample for GPS and baro
        self._last_gps_pos = np.zeros(3)
        self._last_gps_vel = np.zeros(3)
        self._last_baro_alt = 0.0

        logger.info(
            f"SensorBus: IMU@{self._imu_rate}Hz, GPS@{self._gps_rate}Hz, Baro@{self._baro_rate}Hz, "
            f"accel_bias={self._accel_bias}, gyro_bias={self._gyro_bias}"
        )

    def set_biases(self, accel_bias: np.ndarray, gyro_bias: np.ndarray):
        """Override biases (e.g. from error_injection config)."""
        self._accel_bias = np.asarray(accel_bias, dtype=float).ravel()
        self._gyro_bias = np.asarray(gyro_bias, dtype=float).ravel()

    @property
    def true_gyro_bias(self):
        return self._gyro_bias.copy()

    @property
    def true_accel_bias(self):
        return self._accel_bias.copy()

    def update(self, snapshot: dict, t: float) -> SensorPacket:
        """
        Generate sensor measurements from a simulation snapshot.

        Args:
            snapshot: dict from _compute_dynamics() with keys:
                angular_velocity, g_force_vector, position, velocity,
                vel_ned or vel_ned_launch, altitude_km or position[2], etc.
            t: current simulation time (s)

        Returns:
            SensorPacket with y_meas[13] and freshness flags.
        """
        # === Angular velocity (body frame) ===
        omega = np.asarray(snapshot["angular_velocity"], dtype=float)

        # Gyro: omega + bias + noise
        gyro_meas = omega + self._gyro_bias + self._rng.normal(0, self._gyro_std, 3)

        # === Specific force (body axes) from G-force vector ===
        # g_force_vector = specific_force / g0 (in body frame)
        g_vec = np.asarray(snapshot.get("g_force_vector", [0, 0, -1]), dtype=float)
        accel_body = g_vec * 9.80665  # convert from G to m/s²
        accel_meas = accel_body + self._accel_bias + self._rng.normal(0, self._accel_std, 3)

        # === Position and velocity in NED/ground frame ===
        pos = np.asarray(snapshot["position"], dtype=float)
        vel = np.asarray(snapshot["velocity"], dtype=float)

        # Altitude
        if "altitude_km" in snapshot:
            alt_true = snapshot["altitude_km"] * 1000.0
        else:
            alt_true = -pos[2]  # NED: z is down

        # Ground position
        if "altitude_km" in snapshot:
            # Long-range mode: use vel_ned_launch for consistent NED
            vel_ned = np.asarray(snapshot.get("vel_ned_launch", vel), dtype=float)
            x_ground = pos[0]  # NED north
            y_ground = pos[1]  # NED east
        else:
            vel_ned = vel.copy()
            x_ground = pos[0]
            y_ground = pos[1]

        # === Rate gates ===
        imu_fresh = True  # IMU always available at sim rate

        gps_fresh = False
        gps_dt = 1.0 / self._gps_rate
        gps_dropout_active = (self._gps_dropout is not None
                              and self._gps_dropout[0] <= t <= self._gps_dropout[1])
        if not gps_dropout_active and t - self._last_gps_t >= gps_dt - 1e-6:
            gps_fresh = True
            self._last_gps_t = t
            self._last_gps_pos = np.array([
                x_ground + self._rng.normal(0, self._gps_pos_std),
                y_ground + self._rng.normal(0, self._gps_pos_std),
                alt_true + self._rng.normal(0, self._gps_pos_std * 1.5),
            ])
            self._last_gps_vel = np.array([
                vel_ned[0] + self._rng.normal(0, self._gps_vel_std),
                vel_ned[1] + self._rng.normal(0, self._gps_vel_std),
                vel_ned[2] + self._rng.normal(0, self._gps_vel_std),
            ])

        baro_fresh = False
        baro_dt = 1.0 / self._baro_rate
        if t - self._last_baro_t >= baro_dt - 1e-6:
            baro_fresh = True
            self._last_baro_t = t
            self._last_baro_alt = alt_true + self._rng.normal(0, self._baro_std)

        # === Build 13-element measurement vector ===
        y_meas = np.array([
            gyro_meas[0], gyro_meas[1], gyro_meas[2],
            accel_meas[0], accel_meas[1], accel_meas[2],
            self._last_baro_alt,
            self._last_gps_pos[0], self._last_gps_pos[1], self._last_gps_pos[2],
            self._last_gps_vel[0], self._last_gps_vel[1], self._last_gps_vel[2],
        ])

        return SensorPacket(
            t=t,
            y_meas=y_meas,
            imu_fresh=imu_fresh,
            gps_fresh=gps_fresh,
            baro_fresh=baro_fresh,
        )
