#!/usr/bin/env python3
"""
Error injection for Monte Carlo simulations (launch angles, thrust, mass, sensors, aero coefficients).
"""

import numpy as np
import warnings


class ErrorInjectionModel:
    """Error injection for Monte Carlo sims using instance-local RNG."""

    def __init__(self, config):
        """Initialize with config dict (seed, sig_* standard deviations)."""

        self.sig_elevation = config.get('sig_elevation', np.deg2rad(1.0/60.0))  # rad
        self.sig_elevation_rate = config.get('sig_elevation_rate', np.deg2rad(0.1))  # rad/s
        self.sig_azimuth = config.get('sig_azimuth', np.deg2rad(1.0/60.0))  # rad
        self.sig_azimuth_rate = config.get('sig_azimuth_rate', np.deg2rad(0.1))  # rad/s

        self.sig_burn_time = config.get('sig_burn_time', 0.1)  # s
        self.sig_thrust = config.get('sig_thrust', 0.05)  # fraction (5%)
        self.sig_thrust_misalignment_pitch = config.get('sig_thrust_misalignment_pitch', np.deg2rad(1.0/60.0))  # rad
        self.sig_thrust_misalignment_yaw = config.get('sig_thrust_misalignment_yaw', np.deg2rad(1.0/60.0))  # rad

        self.sig_mass = config.get('sig_mass', 5.0)  # kg

        self.sig_cg = config.get('sig_cg', 0.05)  # m

        self.sig_accel_bias_x = config.get('sig_accel_bias_x', 0.1)  # m/s²
        self.sig_accel_bias_y = config.get('sig_accel_bias_y', 0.1)  # m/s²
        self.sig_accel_bias_z = config.get('sig_accel_bias_z', 0.1)  # m/s²

        self.sig_gyro_bias_x = config.get('sig_gyro_bias_x', np.deg2rad(0.5))  # rad/s
        self.sig_gyro_bias_y = config.get('sig_gyro_bias_y', np.deg2rad(0.5))  # rad/s
        self.sig_gyro_bias_z = config.get('sig_gyro_bias_z', np.deg2rad(0.5))  # rad/s

        self.sig_CA = config.get('sig_CA', 0.05)  # fraction
        self.sig_CN = config.get('sig_CN', 0.05)  # fraction
        self.sig_CM = config.get('sig_CM', 0.05)  # fraction

        self.seed = config.get('seed', None)
        self._rng = np.random.default_rng(self.seed)

        self._errors_generated = False
        self.errors = {}

    def _generate_positive_scale(self, rng, mean, sigma, name, max_tries=100):
        """Generate a positive scale factor using lognormal distribution."""
        if sigma <= 0:
            return mean

        log_sigma = np.sqrt(np.log(1 + (sigma / mean) ** 2))
        log_mean = np.log(mean) - 0.5 * log_sigma ** 2

        value = rng.lognormal(log_mean, log_sigma)
        return value

    def _generate_lower_bounded_normal(self, rng, mean, sigma, min_val, name, max_tries=100):
        """Generate normal value with lower bound via rejection sampling."""
        if sigma <= 0:
            return mean

        for _ in range(max_tries):
            value = rng.normal(mean, sigma)
            if value >= min_val:
                return value

        warnings.warn(
            f"Rejection sampling for '{name}' reached max tries ({max_tries}). "
            f"Clamping to minimum value {min_val}.",
            RuntimeWarning
        )
        return min_val

    def generate_errors(self, seed=None):
        """Generate a complete set of errors for one Monte Carlo run."""

        if seed is not None:
            rng = np.random.default_rng(seed)
        else:
            rng = self._rng

        errors = {
            'elevation_error': rng.normal(0, self.sig_elevation),
            'elevation_rate_error': rng.normal(0, self.sig_elevation_rate),
            'azimuth_error': rng.normal(0, self.sig_azimuth),
            'azimuth_rate_error': rng.normal(0, self.sig_azimuth_rate),

            'burn_time_error': self._generate_lower_bounded_normal(
                rng, 0, self.sig_burn_time, -0.9 * self.sig_burn_time * 3, 'burn_time_error'
            ),
            'thrust_scale_error': self._generate_positive_scale(
                rng, 1.0, self.sig_thrust, 'thrust_scale_error'
            ),
            'thrust_misalignment_pitch': rng.normal(0, self.sig_thrust_misalignment_pitch),
            'thrust_misalignment_yaw': rng.normal(0, self.sig_thrust_misalignment_yaw),

            'mass_error': rng.normal(0, self.sig_mass),

            'cg_error': rng.normal(0, self.sig_cg),

            'accel_bias': np.array([
                rng.normal(0, self.sig_accel_bias_x),
                rng.normal(0, self.sig_accel_bias_y),
                rng.normal(0, self.sig_accel_bias_z)
            ]),
            'gyro_bias': np.array([
                rng.normal(0, self.sig_gyro_bias_x),
                rng.normal(0, self.sig_gyro_bias_y),
                rng.normal(0, self.sig_gyro_bias_z)
            ]),

            'CA_scale_error': self._generate_positive_scale(
                rng, 1.0, self.sig_CA, 'CA_scale_error'
            ),
            'CN_scale_error': self._generate_positive_scale(
                rng, 1.0, self.sig_CN, 'CN_scale_error'
            ),
            'CM_scale_error': self._generate_positive_scale(
                rng, 1.0, self.sig_CM, 'CM_scale_error'
            ),
        }

        self.errors = errors
        self._errors_generated = True
        return errors

    def _get_errors(self, errors):
        """Return errors dict, or raise RuntimeError if not generated yet."""
        if errors is not None:
            return errors

        if not self._errors_generated:
            raise RuntimeError(
                "No errors available. Call generate_errors() first or pass an "
                "explicit errors dict to this method."
            )

        return self.errors

    def apply_to_initial_conditions(self, initial_conditions, errors=None):
        """Apply errors to initial conditions. Returns new dict."""

        errors = self._get_errors(errors)

        ic = dict(initial_conditions)

        if 'attitude' in ic:
            attitude = ic['attitude']
            if hasattr(attitude, 'copy'):
                attitude = attitude.copy()
            else:
                attitude = list(attitude)

            if len(attitude) == 3:
                roll, pitch, yaw = attitude
                pitch += errors['elevation_error']
                yaw += errors['azimuth_error']
                ic['attitude'] = [roll, pitch, yaw]

        if 'angular_velocity' in ic:
            angular_velocity = np.array(ic['angular_velocity'], dtype=float, copy=True)
            angular_velocity[1] += errors['elevation_rate_error']
            angular_velocity[2] += errors['azimuth_rate_error']
            ic['angular_velocity'] = angular_velocity

        return ic

    def apply_to_propulsion(self, propulsion_config, errors=None):
        """Apply errors to propulsion config. Returns new dict."""

        errors = self._get_errors(errors)

        config = dict(propulsion_config)

        if 'burn_time' in config:
            new_burn_time = config['burn_time'] + errors['burn_time_error']
            if new_burn_time <= 0:
                warnings.warn(
                    f"burn_time would become non-positive ({new_burn_time:.4f}s). "
                    f"Clamping to 0.01s.",
                    RuntimeWarning
                )
                new_burn_time = 0.01
            config['burn_time'] = new_burn_time

        if 'thrust_scale' in config:
            config['thrust_scale'] *= errors['thrust_scale_error']
        else:
            config['thrust_scale'] = errors['thrust_scale_error']

        config['thrust_misalignment'] = [
            errors['thrust_misalignment_pitch'],
            errors['thrust_misalignment_yaw']
        ]

        return config

    def apply_to_mass_properties(self, mass_config, errors=None):
        """Apply errors to mass properties. Returns new dict."""

        errors = self._get_errors(errors)

        config = dict(mass_config)

        if 'mass_dry' in config:
            new_mass = config['mass_dry'] + errors['mass_error']
            if new_mass <= 0:
                warnings.warn(
                    f"mass_dry would become non-positive ({new_mass:.4f}kg). "
                    f"Clamping to 0.1kg.",
                    RuntimeWarning
                )
                new_mass = 0.1
            config['mass_dry'] = new_mass

        if 'mass_propellant' in config:
            new_mass = config['mass_propellant'] + errors['mass_error'] * 0.5
            if new_mass < 0:
                warnings.warn(
                    f"mass_propellant would become negative ({new_mass:.4f}kg). "
                    f"Clamping to 0.",
                    RuntimeWarning
                )
                new_mass = 0.0
            config['mass_propellant'] = new_mass

        if 'xcg_0' in config:
            config['xcg_0'] += errors['cg_error']
        if 'xcg_end' in config:
            config['xcg_end'] += errors['cg_error']

        return config

    def apply_to_aerodynamics(self, aero_config, errors=None):
        """Apply errors to aerodynamic coefficients. Returns new dict."""

        errors = self._get_errors(errors)

        config = dict(aero_config)

        if 'CA0' in config:
            config['CA0'] *= errors['CA_scale_error']
        if 'CNa' in config:
            config['CNa'] *= errors['CN_scale_error']
        if 'CMa' in config:
            config['CMa'] *= errors['CM_scale_error']

        return config

    def get_sensor_biases(self, errors=None):
        """Get sensor bias errors (accel_bias, gyro_bias)."""

        errors = self._get_errors(errors)

        return {
            'accel_bias': errors['accel_bias'].copy(),
            'gyro_bias': errors['gyro_bias'].copy()
        }

    def apply_thrust_misalignment(self, thrust_vector, errors=None):
        """Apply thrust misalignment via pitch/yaw rotation matrices."""

        errors = self._get_errors(errors)

        thrust_vector = np.asarray(thrust_vector, dtype=float)

        pitch_error = errors['thrust_misalignment_pitch']
        yaw_error = errors['thrust_misalignment_yaw']

        # Rotation matrix about Y-axis (pitch)
        cp = np.cos(pitch_error)
        sp = np.sin(pitch_error)
        R_pitch = np.array([
            [cp,  0, sp],
            [0,   1,  0],
            [-sp, 0, cp]
        ])

        # Rotation matrix about Z-axis (yaw)
        cy = np.cos(yaw_error)
        sy = np.sin(yaw_error)
        R_yaw = np.array([
            [cy, -sy, 0],
            [sy,  cy, 0],
            [0,   0,  1]
        ])

        R_combined = R_yaw @ R_pitch
        thrust_misaligned = R_combined @ thrust_vector

        return thrust_misaligned
