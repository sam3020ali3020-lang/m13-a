#!/usr/bin/env python3
"""
Table-based atmosphere model using linear interpolation from atmosphere_table.csv.
Provides smooth atmospheric properties up to 200km, avoiding ISA discontinuities above 86km.

Required CSV columns: altitude_m, pressure_Pa, temperature_K, density_kg_m3, speed_of_sound_m_s
Optional: wind_north_m_s, wind_east_m_s, wind_down_m_s
"""

import numpy as np
from typing import Dict, Tuple, Optional, Any
import pandas as pd
from scipy.interpolate import interp1d
import logging

logger = logging.getLogger(__name__)


class AtmosphereModel:
    """Table-based atmosphere model with linear interpolation from precomputed tables."""

    SUTHERLAND_C1 = 1.458e-6  # kg/(m·s·K^0.5)
    SUTHERLAND_S = 110.4      # K
    G0_STANDARD = 9.80665     # Standard gravity (m/s²)
    R_EARTH = 6371000.0       # Mean Earth radius (m)
    R_UNIVERSAL = 8314.462618 # Universal gas constant J/(kmol·K)
    M_SEA_LEVEL = 28.9644     # Sea level molecular weight (kg/kmol)

    _warned_below_min = False
    _warned_above_max = False

    def __init__(self, atmosphere_table: pd.DataFrame, config: Optional[dict] = None):
        """Initialize table-based atmosphere model.

        Args:
            atmosphere_table: DataFrame from atmosphere_table.csv
            config: Optional dict with 'wind' (enabled, speed, direction, direction_unit)
                    and 'latitude_rad' for gravity calculation.
        """
        if atmosphere_table is None or len(atmosphere_table) == 0:
            raise ValueError(
                "atmosphere_table is required. Each rocket model must include "
                "an atmosphere_table.csv file."
            )

        self.table = atmosphere_table.copy()
        self.config = config or {}

        self._validate_table()
        self._create_interpolators()

        self.latitude_rad = self.config.get('latitude_rad', 0.0)
        self._compute_gravity_params()

        wind_config = self.config.get('wind', {})
        if not wind_config:
            wind_config = {
                'enabled': self.config.get('wind_enabled', False),
                'speed': self.config.get('wind_speed', 0.0),
                'direction': self.config.get('wind_direction', 0.0),
                'direction_unit': self.config.get('wind_direction_unit', 'degrees')
            }
        self.wind_override_speed = wind_config.get('speed', 0.0)
        wind_enabled = wind_config.get('enabled', False)

        # Mode selection:
        # - override_enabled=True  -> constant wind from speed/direction
        # - table_enabled=True     -> altitude-varying wind from atmosphere table
        # Backward-compatible default: if enabled and speed is effectively zero,
        # use table wind when available; otherwise use constant override.
        self.wind_override_enabled = wind_config.get(
            'override_enabled',
            bool(wind_enabled and abs(self.wind_override_speed) > 1e-6)
        )
        self.wind_table_enabled = wind_config.get(
            'table_enabled',
            bool(wind_enabled and not self.wind_override_enabled)
        )

        direction_value = wind_config.get('direction', 0.0)
        direction_unit = wind_config.get('direction_unit', 'degrees').lower()

        if direction_unit not in ('degrees', 'radians', 'deg', 'rad'):
            logger.warning(f"Unknown wind direction unit '{direction_unit}', assuming degrees.")
            direction_unit = 'degrees'

        if direction_unit in ('degrees', 'deg'):
            self.wind_override_direction = np.radians(direction_value)
            if abs(direction_value) > 360:
                logger.warning(
                    f"Wind direction {direction_value} degrees seems unusually large."
                )
        else:
            self.wind_override_direction = direction_value
            if abs(direction_value) > 2 * np.pi:
                logger.warning(
                    f"Wind direction {direction_value} radians is > 2*pi. "
                    f"Did you mean to use degrees? Set direction_unit: 'degrees' in config."
                )

        logger.info(f"AtmosphereModel initialized with altitude range: "
                    f"{self.min_altitude:.0f} - {self.max_altitude:.0f} m")

    def _validate_table(self):
        """Validate that the atmosphere table has required columns and valid data."""
        required_cols = ['altitude_m', 'pressure_Pa', 'temperature_K',
                         'density_kg_m3', 'speed_of_sound_m_s']

        missing = [col for col in required_cols if col not in self.table.columns]
        if missing:
            raise ValueError(f"Atmosphere table missing required columns: {missing}")

        nan_counts = self.table[required_cols].isna().sum()
        cols_with_nan = nan_counts[nan_counts > 0]
        if len(cols_with_nan) > 0:
            nan_info = ", ".join([f"{col}: {count}" for col, count in cols_with_nan.items()])
            raise ValueError(
                f"Atmosphere table contains NaN values in required columns: {nan_info}."
            )

        self.table = self.table.sort_values('altitude_m', kind='mergesort').reset_index(drop=True)

        duplicate_mask = self.table['altitude_m'].duplicated(keep=False)
        if duplicate_mask.any():
            duplicate_altitudes = self.table.loc[duplicate_mask, 'altitude_m'].unique()

            for alt in duplicate_altitudes:
                rows = self.table[self.table['altitude_m'] == alt]
                first_row = rows.iloc[0][required_cols].values

                for idx in range(1, len(rows)):
                    other_row = rows.iloc[idx][required_cols].values
                    if not np.allclose(first_row, other_row, rtol=1e-6, atol=1e-9):
                        differing_cols = []
                        for i, col in enumerate(required_cols):
                            if not np.isclose(first_row[i], other_row[i], rtol=1e-6, atol=1e-9):
                                differing_cols.append(f"{col}: {first_row[i]} vs {other_row[i]}")
                        raise ValueError(
                            f"Atmosphere table has duplicate altitude {alt} m with different values. "
                            f"Differences: {', '.join(differing_cols)}."
                        )

            logger.warning(
                f"Atmosphere table contains {duplicate_mask.sum() - len(duplicate_altitudes)} duplicate row(s) "
                f"with identical values. Removing duplicates."
            )
            self.table = self.table.drop_duplicates(subset='altitude_m', keep='first').reset_index(drop=True)

        self.min_altitude = self.table['altitude_m'].min()
        self.max_altitude = self.table['altitude_m'].max()

        self._validate_physical_ranges()

        self.has_wind_data = all(col in self.table.columns for col in
                                 ['wind_north_m_s', 'wind_east_m_s', 'wind_down_m_s'])

    def _validate_physical_ranges(self):
        """Validate atmospheric properties are within physical ranges. Raises ValueError if not."""
        validation_rules = {
            'temperature_K': {
                'min': 1.0,
                'description': 'Temperature must be positive (> 1 K)',
            },
            'pressure_Pa': {
                'min': 0.0,
                'description': 'Pressure must be non-negative (>= 0 Pa)',
            },
            'density_kg_m3': {
                'min': 0.0,
                'description': 'Density must be non-negative (>= 0 kg/m³)',
            },
            'speed_of_sound_m_s': {
                'min': 1.0,
                'description': 'Speed of sound must be positive (> 1 m/s)',
            }
        }

        errors = []
        for col, rules in validation_rules.items():
            min_val = rules['min']
            invalid_mask = self.table[col] < min_val
            if invalid_mask.any():
                invalid_rows = self.table[invalid_mask]
                invalid_altitudes = invalid_rows['altitude_m'].tolist()
                invalid_values = invalid_rows[col].tolist()

                error_details = [f"altitude={alt:.1f}m: {col}={val}"
                                 for alt, val in zip(invalid_altitudes[:5], invalid_values[:5])]
                if len(invalid_altitudes) > 5:
                    error_details.append(f"... and {len(invalid_altitudes) - 5} more")

                errors.append(
                    f"{rules['description']}. Found {len(invalid_altitudes)} invalid value(s): "
                    f"{', '.join(error_details)}."
                )

        if errors:
            raise ValueError(
                "Atmosphere table contains physically invalid values:\n" +
                "\n".join(f"  - {e}" for e in errors)
            )

    def _create_interpolators(self):
        """Create linear interpolation functions for each atmospheric property."""
        altitudes = self.table['altitude_m'].values
        pressure = self.table['pressure_Pa'].values
        temperature = self.table['temperature_K'].values
        density = self.table['density_kg_m3'].values
        speed_of_sound = self.table['speed_of_sound_m_s'].values

        self._interp_pressure = interp1d(
            altitudes, pressure,
            kind='linear', bounds_error=False,
            fill_value=(pressure[0], pressure[-1])
        )
        self._interp_temperature = interp1d(
            altitudes, temperature,
            kind='linear', bounds_error=False,
            fill_value=(temperature[0], temperature[-1])
        )
        self._interp_density = interp1d(
            altitudes, density,
            kind='linear', bounds_error=False,
            fill_value=(density[0], density[-1])
        )
        self._interp_speed_of_sound = interp1d(
            altitudes, speed_of_sound,
            kind='linear', bounds_error=False,
            fill_value=(speed_of_sound[0], speed_of_sound[-1])
        )

        if self.has_wind_data:
            wind_north = self.table['wind_north_m_s'].values
            wind_east = self.table['wind_east_m_s'].values
            wind_down = self.table['wind_down_m_s'].values

            self._interp_wind_north = interp1d(
                altitudes, wind_north,
                kind='linear', bounds_error=False,
                fill_value=(wind_north[0], wind_north[-1])
            )
            self._interp_wind_east = interp1d(
                altitudes, wind_east,
                kind='linear', bounds_error=False,
                fill_value=(wind_east[0], wind_east[-1])
            )
            self._interp_wind_down = interp1d(
                altitudes, wind_down,
                kind='linear', bounds_error=False,
                fill_value=(wind_down[0], wind_down[-1])
            )

    def _compute_gravity_params(self):
        """Compute latitude-dependent gravity parameters."""
        lat = self.latitude_rad
        sin_lat_sq = np.sin(lat) ** 2
        self.g0_lat = 9.780327 * (1.0 + (0.005279 + 0.000023 * sin_lat_sq) * sin_lat_sq)

    def compute_dynamic_viscosity(self, temperature_K: float) -> float:
        """Compute dynamic viscosity using Sutherland's law."""
        if temperature_K <= 0:
            return 0.0
        return self.SUTHERLAND_C1 * (temperature_K ** 1.5) / (temperature_K + self.SUTHERLAND_S)

    def get_properties(self, altitude: float) -> Dict[str, Any]:
        """Get interpolated atmosphere properties at given altitude (m)."""
        h = float(altitude)

        if h < self.min_altitude:
            if not AtmosphereModel._warned_below_min:
                logger.warning(
                    f"Altitude {h:.1f} m is below table minimum ({self.min_altitude:.1f} m). "
                    f"Clamping to minimum altitude. (This warning will only appear once)"
                )
                AtmosphereModel._warned_below_min = True
            h = self.min_altitude
        elif h > self.max_altitude:
            if not AtmosphereModel._warned_above_max:
                logger.warning(
                    f"Altitude {h:.1f} m exceeds table maximum ({self.max_altitude:.1f} m). "
                    f"Clamping to maximum altitude. (This warning will only appear once)"
                )
                AtmosphereModel._warned_above_max = True
            h = self.max_altitude

        T = float(self._interp_temperature(h))
        P = float(self._interp_pressure(h))
        rho = float(self._interp_density(h))
        a = float(self._interp_speed_of_sound(h))

        g = self.g0_lat * (self.R_EARTH / (self.R_EARTH + h)) ** 2

        mu = self.compute_dynamic_viscosity(T)

        if self.wind_override_enabled:
            wind_ned = np.array([
                self.wind_override_speed * np.cos(self.wind_override_direction),
                self.wind_override_speed * np.sin(self.wind_override_direction),
                0.0
            ])
        elif self.has_wind_data and self.wind_table_enabled:
            wind_ned = np.array([
                float(self._interp_wind_north(h)),
                float(self._interp_wind_east(h)),
                float(self._interp_wind_down(h))
            ])
        else:
            wind_ned = np.zeros(3)

        h_geopotential = (self.g0_lat / self.G0_STANDARD) * (self.R_EARTH * h) / (self.R_EARTH + h)

        if P > 1e-6 and rho > 1e-12:
            M = (rho * self.R_UNIVERSAL * T) / P
            M = np.clip(M, 2.0, 40.0)
        else:
            M = self.M_SEA_LEVEL

        return {
            'temperature': T,
            'pressure': P,
            'density': rho,
            'speed_of_sound': a,
            'wind_ned': wind_ned,
            'dynamic_viscosity': mu,
            'gravity': g,
            'molecular_weight': M,
            'geopotential_altitude': h_geopotential,
        }

