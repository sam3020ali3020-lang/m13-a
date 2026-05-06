#!/usr/bin/env python3
"""
Aerodynamics model for 6-DOF rocket simulation.

3D/2D interpolation for aero coefficients (CA, CN, Cm),
damping derivatives, fin effects, and launcher dynamics.
"""

import numpy as np
from scipy.interpolate import RegularGridInterpolator
import pandas as pd
from pathlib import Path
from typing import Dict, Tuple, Optional, List, TYPE_CHECKING
from dataclasses import dataclass
import logging
import warnings

from dynamics.quaternion_utils import (
    DegenerateQuaternionError,
    QUAT_NORM_THRESHOLD,
)

if TYPE_CHECKING:
    from ..rocket_data_loader import AeroTables

logger = logging.getLogger(__name__)

# Configure warnings to show each unique message only once
warnings.filterwarnings('once', category=UserWarning, module=__name__)


class AeroDataError(Exception):
    """Exception raised when aerodynamic data cannot be loaded."""
    pass


class AeroInterpolationWarning(UserWarning):
    """Warning raised when aerodynamic interpolation fails or data is missing."""
    pass


# Configure AeroInterpolationWarning to show each unique message only once
warnings.filterwarnings('once', category=AeroInterpolationWarning)


class BoundedGridInterpolator:
    """RegularGridInterpolator wrapper that clamps inputs to grid bounds."""
    
    def __init__(self, points: tuple, values: np.ndarray, method: str = 'linear',
                 dim_names: Optional[List[str]] = None, warn_on_clamp: bool = True):
        self._points = points
        self._bounds = [(p[0], p[-1]) for p in points]
        self._dim_names = dim_names if dim_names else [f'dim{i}' for i in range(len(points))]
        self._warn_on_clamp = warn_on_clamp
        self._interp = RegularGridInterpolator(
            points,
            values,
            method=method,
            bounds_error=False,
            fill_value=None
        )
        self._warned_dims = set()
    
    def _warn_clamp(self, dim_name: str, direction: str, val: float,
                    lo: float, hi: float, context: str = None):
        """Emit a clamping warning once per (dim, direction)."""
        warn_key = (dim_name, direction)
        if warn_key in self._warned_dims:
            return
        self._warned_dims.add(warn_key)
        bound_val = lo if direction == 'below' else hi
        ctx_str = f" for {context}" if context else ""
        warnings.warn(
            f"Aerodynamic table clamping{ctx_str}: {dim_name}={val:.4f} is {direction} "
            f"table bounds [{lo:.4f}, {hi:.4f}]. Using boundary value {bound_val:.4f}. "
            f"Results may be inaccurate outside the validated flight envelope.",
            AeroInterpolationWarning
        )

    def __call__(self, xi, context: str = None) -> np.ndarray:
        """Evaluate interpolator at given points, clamping to grid bounds."""
        xi_array = np.asarray(xi)
        single_point = xi_array.ndim == 1

        if self._warn_on_clamp:
            for i, (lo, hi) in enumerate(self._bounds):
                dim_name = self._dim_names[i]
                vals = xi_array[i] if single_point else xi_array[..., i]
                vals_arr = np.atleast_1d(vals)
                below = vals_arr < lo
                above = vals_arr > hi
                if np.any(below):
                    self._warn_clamp(dim_name, 'below', float(np.min(vals_arr[below])), lo, hi, context)
                if np.any(above):
                    self._warn_clamp(dim_name, 'above', float(np.max(vals_arr[above])), lo, hi, context)

        xi_clamped = np.empty_like(xi_array, dtype=float)
        for i, (lo, hi) in enumerate(self._bounds):
            if single_point:
                xi_clamped[i] = np.clip(xi_array[i], lo, hi)
            else:
                xi_clamped[..., i] = np.clip(xi_array[..., i], lo, hi)

        result = self._interp(xi_clamped)

        if single_point and result.ndim > 0 and result.size == 1:
            result = result.squeeze()

        return result


@dataclass
class FinMoments:
    """Per-fin hinge and bending moments."""
    hinge_moments: np.ndarray  # [4] N-m
    bending_moments: np.ndarray  # [4] N-m
    max_hinge: float  # Maximum hinge moment across all fins
    max_bending: float  # Maximum bending moment across all fins


@dataclass
class AeroState:
    """Complete aerodynamic state output."""
    forces: np.ndarray  # Body frame forces [Fx, Fy, Fz] (N)
    moments: np.ndarray  # Body frame moments [Mx, My, Mz] (N-m)
    alpha: float  # Angle of attack (rad)
    beta: float  # Sideslip angle (rad)
    alpha_total: float  # Total angle of attack (rad)
    mach: float  # Mach number (clamped to table bounds for interpolation)
    mach_raw: float  # True Mach number (unclamped, for downstream logic)
    reynolds: float  # Reynolds number
    CA: float  # Axial force coefficient
    CN: float  # Normal force coefficient (total)
    CM: float  # Pitching moment coefficient (total)
    CY: float  # Side force coefficient (total)
    Cn: float  # Yawing moment coefficient (total)
    Cl: float  # Rolling moment coefficient (total)
    fin_drag: np.ndarray  # Per-fin drag coefficients [4]
    fin_moments: Optional[FinMoments]  # Per-fin structural moments
    # Control coefficients (from fin deflections)
    CN_control: float = 0.0  # Normal force control coefficient
    CM_control: float = 0.0  # Pitching moment control coefficient
    CY_control: float = 0.0  # Side force control coefficient
    Cn_control: float = 0.0  # Yawing moment control coefficient
    # Raw aerodynamic moments (before any modifications)
    M_roll: float = 0.0  # Roll moment (N-m)
    M_pitch: float = 0.0  # Pitch moment (N-m)
    M_yaw: float = 0.0  # Yaw moment (N-m)


def _build_index_map(values: np.ndarray) -> Dict[float, int]:
    """Build a dictionary mapping values to their indices."""
    return {v: i for i, v in enumerate(values)}


def _get_index(index_map: Dict[float, int], value: float) -> Optional[int]:
    """Get index for a value using the index map."""
    return index_map.get(value)


def _build_grid_axes(df: pd.DataFrame, axis_columns: List[str]) -> Tuple[List[np.ndarray], List[Dict[float, int]]]:
    """Build sorted axis values and index maps from a DataFrame."""
    axis_values = []
    index_maps = []
    for col in axis_columns:
        vals = np.sort(df[col].unique())
        axis_values.append(vals)
        index_maps.append(_build_index_map(vals))
    return axis_values, index_maps


def _fill_grid_from_df(df: pd.DataFrame, axis_columns: List[str], value_column: str,
                       axis_values: List[np.ndarray], index_maps: List[Dict[float, int]]) -> np.ndarray:
    """Fill a grid from DataFrame rows using index maps."""
    grid_shape = tuple(len(vals) for vals in axis_values)
    grid = np.zeros(grid_shape)

    for _, row in df.iterrows():
        indices = []
        valid = True
        for col, idx_map in zip(axis_columns, index_maps):
            idx = _get_index(idx_map, row[col])
            if idx is None:
                valid = False
                break
            indices.append(idx)
        if valid:
            grid[tuple(indices)] = row[value_column]

    return grid


def _create_interpolator(axis_values: List[np.ndarray], grid: np.ndarray,
                         dim_names: Optional[List[str]] = None) -> BoundedGridInterpolator:
    """Create a BoundedGridInterpolator with standard options."""
    return BoundedGridInterpolator(
        tuple(axis_values),
        grid,
        method='linear',
        dim_names=dim_names
    )


class AdvancedAerodynamicsModel:
    """Advanced aerodynamic model with 3D tables, damping, fin effects, and launcher dynamics."""

    def __init__(self, config: dict, aero_tables: Optional['AeroTables'] = None):
        """Initialize advanced aerodynamics model."""

        # reference_length = body diameter, NOT total rocket length!
        self.S_ref = config.get('reference_area',
                     config.get('ref_area_m2',
                     config.get('S_ref', None)))

        # reference_length = diameter (characteristic length for aerodynamics)
        self.L_ref = config.get('reference_length',
                     config.get('ref_diameter_m',
                     config.get('L_ref', None)))

        # Actual rocket length (for documentation and other calculations)
        self.rocket_length = config.get('ref_length_m',
                             config.get('rocket_length', None))

        if self.S_ref is None:
            raise AeroDataError(
                "reference_area is required but not found in configuration. "
                "Supported keys: 'reference_area', 'ref_area_m2', 'S_ref'. "
                "Please specify one of these in aerodynamics config or rocket_properties.yaml."
            )
        if self.L_ref is None:
            raise AeroDataError(
                "reference_length (diameter) is required but not found in configuration. "
                "Supported keys: 'reference_length', 'ref_diameter_m', 'L_ref'. "
                "Note: This is the body DIAMETER, not the rocket length! "
                "Please specify one of these in aerodynamics config or rocket_properties.yaml."
            )

        # D_body = diameter for damping D/(2V) and torque scaling. Defaults to L_ref.
        self.D_body = config.get('body_diameter', config.get('D_body', self.L_ref))

        self.use_tables = config.get('use_aero_tables', True)  # Default to True
        self.tables_path = Path(config.get('aero_tables_path', 'data/rocket_models'))

        self._ca_interp = None  # Axial force coefficient interpolator
        self._cn_cm_interp = None  # Normal and pitching moment coefficients interpolator
        self._damping_interp = None  # Damping derivatives interpolator
        self._clnr_interp = None  # Yaw moment damping derivative (CLNR)
        self._cyr_interp = None   # Side force damping derivative (CYR)
        self._fin_drag_interp = None  # Fin drag interpolator
        self._hinge_bending_interp = None  # Hinge and bending moments interpolator
        self._fin_effectiveness_interp = None  # Fin effectiveness interpolator
        self._roll_aero_interp = None  # Roll aero coeffs interpolator (Mach, alpha, def_roll) -> Cll

        # Motor on/off CA table interpolators (new Badr3 format)
        self._ca_motor_on_interp = None  # CA interpolator for motor on
        self._ca_motor_off_interp = None  # CA interpolator for motor off
        self._ca_motor_on_bounds = None  # CA bounds for motor on
        self._ca_motor_off_bounds = None  # CA bounds for motor off
        self._ca_motor_on_has_reynolds = False  # CA table has Reynolds for motor on
        self._ca_motor_off_has_reynolds = False  # CA table has Reynolds for motor off
        self._use_ca_motor_pair = False  # Use separate motor on/off CA tables

        # Fin effectiveness source tracking
        self._fin_effectiveness_source = None

        # Dynamic bounds from loaded tables (None = use defaults, set during table loading)
        self._mach_min = None
        self._mach_max = None
        self._alpha_min = None   # alpha bounds (deg)
        self._alpha_max = None
        self._delta_min = None   # delta/fin deflection bounds (deg)
        self._delta_max = None
        self._reynolds_min = None  # Reynolds bounds
        self._reynolds_max = None

        # Load tables from AeroTables object if provided
        if aero_tables is not None:
            self._load_from_aero_tables(aero_tables)
        elif self.use_tables:
            self._load_aero_tables()
        else:
            raise AeroDataError(
                "Aerodynamic tables are required. Set use_aero_tables=True and provide "
                "either aero_tables_path or aero_tables object."
            )

        # Validate that we have minimum required tables
        self._validate_tables()

        # These are only used as fallback if tables don't have the data
        self.CMq_default = config.get('CMq', -5.0)
        self.Cnq_default = config.get('Cnq', -5.0)
        self.Clp_default = config.get('Clp', -0.5)

        # CA multiplier when motor is on (default 1.05 = 5% drag increase from plume effects)
        self.CA_motor_multiplier = config.get('CA_motor_multiplier', 1.05)

        # General CA multiplier from rocket_properties.yaml. Applied to all CA values.
        self.CA_multiplier = config.get('CA_multiplier', 1.0)

        # CN multiplier — for domain randomization in RL training
        self.CN_multiplier = config.get('CN_multiplier', 1.0)

        # CM multiplier — for domain randomization in RL training
        self.CM_multiplier = config.get('CM_multiplier', 1.0)

        self.compute_hinge_moments = config.get('compute_hinge_moments', True)
        self.compute_fin_drag = config.get('compute_fin_drag', True)
        
        # Moment transfer ref point (m from nose). None = tables at CG.
        self.moment_reference_point = config.get('moment_reference_point', None)
        if self.moment_reference_point is not None:
            logger.info(f"Moment reference point set to {self.moment_reference_point:.4f} m from nose")
        
        # negate_Cnq: True (default) = force -abs(Cnq) for stability; False = trust table sign.
        self.negate_Cnq = config.get('negate_Cnq', True)
        if self.negate_Cnq:
            logger.debug("Yaw damping sign convention: forcing -abs(Cnq) for stability")
        else:
            logger.info("Yaw damping sign convention: using Cnq directly from tables (trust data)")

        # damping_coefficients_units: "per_rad" (default) or "per_deg" (applies r2d=57.3 factor)
        self.damping_coefficients_units = config.get('damping_coefficients_units', 'per_rad')
        allowed_damping_units = ('per_rad', 'per_deg')
        if self.damping_coefficients_units not in allowed_damping_units:
            raise AeroDataError(
                f"damping_coefficients_units must be one of {allowed_damping_units}, "
                f"got '{self.damping_coefficients_units}'"
            )
        if self.damping_coefficients_units == 'per_deg':
            logger.info("Damping coefficients units: per_deg (ddv will include r2d factor)")
        else:
            logger.debug("Damping coefficients units: per_rad (standard)")

        # fin_channel_mapping: permutation of [1,2,3,4] → internal δ1-δ4. Default = identity.
        fin_mapping = config.get('fin_channel_mapping', [1, 2, 3, 4])
        if fin_mapping is None:
            fin_mapping = [1, 2, 3, 4]
        
        # Validate mapping
        if len(fin_mapping) != 4:
            raise AeroDataError(
                f"fin_channel_mapping must have exactly 4 elements, got {len(fin_mapping)}"
            )
        if sorted(fin_mapping) != [1, 2, 3, 4]:
            raise AeroDataError(
                f"fin_channel_mapping must be a permutation of [1, 2, 3, 4], got {fin_mapping}"
            )
        
        # Convert to 0-indexed for internal use
        self._fin_channel_mapping = [f - 1 for f in fin_mapping]

        # Check if mapping is identity (no remapping needed)
        self._needs_fin_remapping = (self._fin_channel_mapping != [0, 1, 2, 3])
        
        if self._needs_fin_remapping:
            logger.info(f"Fin channel mapping enabled: physical fins {fin_mapping} → internal [1,2,3,4]")
        else:
            logger.debug("Fin channel mapping: identity (no remapping)")

        # config_type: 'X' (fins at 45°, default) or '+' (fins aligned with body axes)
        self.config_type = config.get('config_type', 'X')
        allowed_config_types = ('X', '+')
        if self.config_type not in allowed_config_types:
            raise AeroDataError(
                f"config_type must be one of {allowed_config_types}, "
                f"got '{self.config_type}'"
            )
        logger.info(f"Aerodynamics using {self.config_type} fin configuration")

        # Fin location: 'tail' (default) or 'canard' (negates roll moment due to opposite moment arm)
        self.fin_location = config.get('fin_location', 'tail')
        allowed_fin_locations = ('canard', 'tail')
        if self.fin_location not in allowed_fin_locations:
            raise AeroDataError(
                f"fin_location must be one of {allowed_fin_locations}, "
                f"got '{self.fin_location}'"
            )
        if self.fin_location == 'canard':
            logger.info("Aerodynamics using canard fin configuration - roll moment will be negated")
        else:
            logger.info(f"Aerodynamics using {self.fin_location} fin configuration")

        self._load_launcher_dynamics(config)

        self.max_hinge_moment = np.zeros(4)
        self.max_bending_moment = np.zeros(4)

    def _validate_tables(self):
        """Validate that minimum required tables are loaded."""
        if not self._cn_cm_interp and not hasattr(self, '_aero_coeffs_df'):
            raise AeroDataError(
                "No aerodynamic coefficient tables loaded. "
                "At least aero_coeffs.csv or cn_cm_coefficients.csv is required."
            )

    def _update_mach_bounds(self, mach_min: float, mach_max: float):
        """Update Mach bounds from loaded table data (takes intersection)."""
        if self._mach_min is None:
            self._mach_min = mach_min
        else:
            self._mach_min = max(self._mach_min, mach_min)
            
        if self._mach_max is None:
            self._mach_max = mach_max
        else:
            self._mach_max = min(self._mach_max, mach_max)

    def _update_alpha_bounds(self, alpha_min: float, alpha_max: float):
        """Update alpha bounds from loaded table data (takes intersection)."""
        if self._alpha_min is None:
            self._alpha_min = alpha_min
        else:
            self._alpha_min = max(self._alpha_min, alpha_min)
            
        if self._alpha_max is None:
            self._alpha_max = alpha_max
        else:
            self._alpha_max = min(self._alpha_max, alpha_max)

    def _update_delta_bounds(self, delta_min: float, delta_max: float):
        """Update delta bounds symmetrically (e.g., [0,20] → [-20,+20])."""
        # Make bounds symmetric: use the maximum absolute value for both bounds
        max_abs = max(abs(delta_min), abs(delta_max))
        symmetric_min = -max_abs
        symmetric_max = max_abs
        
        if self._delta_min is None:
            self._delta_min = symmetric_min
        else:
            # Take intersection but maintain symmetry
            self._delta_min = max(self._delta_min, symmetric_min)
            
        if self._delta_max is None:
            self._delta_max = symmetric_max
        else:
            self._delta_max = min(self._delta_max, symmetric_max)

    def _update_reynolds_bounds(self, reynolds_min: float, reynolds_max: float):
        """Update Reynolds bounds from loaded table data (takes intersection)."""
        if self._reynolds_min is None:
            self._reynolds_min = reynolds_min
        else:
            self._reynolds_min = max(self._reynolds_min, reynolds_min)
            
        if self._reynolds_max is None:
            self._reynolds_max = reynolds_max
        else:
            self._reynolds_max = min(self._reynolds_max, reynolds_max)

    def _validate_alpha_symmetry(self, df: pd.DataFrame, table_name: str,
                                  odd_columns: List[str], even_columns: List[str],
                                  tolerance: float = 0.01):
        """Validate coefficient symmetry (odd: f(-α)=-f(α), even: f(-α)=f(α))."""
        if 'alpha_deg' not in df.columns:
            return
            
        alpha_vals = df['alpha_deg'].unique()
        
        has_negative = any(a < 0 for a in alpha_vals)
        has_positive = any(a > 0 for a in alpha_vals)
        
        if not (has_negative and has_positive):
            return
        
        mach_col = 'Mach' if 'Mach' in df.columns else None
        if mach_col is None:
            return
            
        mach_vals = df[mach_col].unique()
        
        violations = []
        
        for mach in mach_vals:
            mach_data = df[df[mach_col] == mach]
            
            for alpha in alpha_vals:
                if alpha <= 0:
                    continue
                    
                pos_row = mach_data[mach_data['alpha_deg'] == alpha]
                neg_row = mach_data[mach_data['alpha_deg'] == -alpha]
                
                if pos_row.empty or neg_row.empty:
                    continue
                
                for col in odd_columns:
                    if col not in df.columns:
                        continue
                    pos_val = pos_row[col].values[0]
                    neg_val = neg_row[col].values[0]
                    
                    if abs(pos_val) < 1e-10 and abs(neg_val) < 1e-10:
                        continue
                        
                    expected_neg = -pos_val
                    if abs(pos_val) > 1e-10:
                        rel_error = abs(neg_val - expected_neg) / abs(pos_val)
                        if rel_error > tolerance:
                            violations.append(
                                f"{col}: Mach={mach}, alpha=±{alpha}°, "
                                f"f(+α)={pos_val:.4f}, f(-α)={neg_val:.4f}, "
                                f"expected f(-α)={expected_neg:.4f} (antisymmetric)"
                            )
                
                for col in even_columns:
                    if col not in df.columns:
                        continue
                    pos_val = pos_row[col].values[0]
                    neg_val = neg_row[col].values[0]
                    
                    if abs(pos_val) < 1e-10 and abs(neg_val) < 1e-10:
                        continue
                        
                    if abs(pos_val) > 1e-10:
                        rel_error = abs(neg_val - pos_val) / abs(pos_val)
                        if rel_error > tolerance:
                            violations.append(
                                f"{col}: Mach={mach}, alpha=±{alpha}°, "
                                f"f(+α)={pos_val:.4f}, f(-α)={neg_val:.4f}, "
                                f"expected f(-α)={pos_val:.4f} (symmetric)"
                            )
        
        if violations:
            warning_msg = (
                f"ASYMMETRIC AERO TABLE WARNING: Symmetry violations detected in {table_name}. "
                f"The code uses abs(alpha) for lookup and applies sign(alpha) to restore "
                f"directionality, which assumes antisymmetric Cn/Cm and symmetric Cd. "
                f"If your tables are intentionally asymmetric, the negative alpha data will be "
                f"SILENTLY IGNORED and results may be incorrect. "
                f"Violations (showing first 5):\n" + "\n".join(violations[:5])
            )
            if len(violations) > 5:
                warning_msg += f"\n... and {len(violations) - 5} more violations"
            warnings.warn(warning_msg, AeroInterpolationWarning)
            logger.warning(warning_msg)

    def _load_from_aero_tables(self, aero_tables: 'AeroTables'):
        """Load interpolators from pre-loaded AeroTables object (rocket_ws format)."""
        logger.info("Loading aerodynamic tables from AeroTables object (rocket_ws format)")

        # Load aero_coeffs (Cd, Cn, Cm vs Mach, alpha) - rocket_ws format
        if aero_tables.aero_coeffs is not None:
            self._load_aero_coeffs_rocket_ws(aero_tables.aero_coeffs)

        # Load fin_deflection_coeffs (CN_delta, CM_delta) — 3D or 2D format
        if hasattr(aero_tables, 'fin_deflection_coeffs') and aero_tables.fin_deflection_coeffs is not None:
            self._load_fin_deflection_coeffs_from_df(aero_tables.fin_deflection_coeffs)

        # Load damping_coeffs (Cmq, Cnq, Clp)
        if aero_tables.damping_coeffs is not None:
            self._load_damping_from_df(aero_tables.damping_coeffs)

        # Load fin_deflection_drag (Cd_delta)
        if aero_tables.fin_deflection_drag is not None:
            self._load_fin_drag_from_df(aero_tables.fin_deflection_drag)

        # Load CA tables: 1) motor_on+off pair, 2) legacy ca_3d, 3) single table
        has_motor_on = hasattr(aero_tables, 'ca_3d_coeffs_motor_on') and aero_tables.ca_3d_coeffs_motor_on is not None
        has_motor_off = hasattr(aero_tables, 'ca_3d_coeffs_motor_off') and aero_tables.ca_3d_coeffs_motor_off is not None

        if has_motor_on and has_motor_off:
            # Load motor_on/motor_off pair - highest priority
            self._load_ca_3d_motor_pair(aero_tables.ca_3d_coeffs_motor_on, aero_tables.ca_3d_coeffs_motor_off)
            self._use_ca_motor_pair = True
        elif aero_tables.ca_3d_coeffs is not None:
            # Legacy single CA table
            self._load_ca_3d_from_df(aero_tables.ca_3d_coeffs)
            self._use_ca_motor_pair = False
        elif has_motor_on:
            # Only motor_on available - use for both states
            self._load_ca_3d_from_df(aero_tables.ca_3d_coeffs_motor_on)
            self._use_ca_motor_pair = False
            logger.warning(
                "Only ca_3d_coeffs_motor_on available. Using it for both motor states. "
                "CA_motor_multiplier will be applied for motor_on state."
            )
        elif has_motor_off:
            # Only motor_off available - use for both states
            self._load_ca_3d_from_df(aero_tables.ca_3d_coeffs_motor_off)
            self._use_ca_motor_pair = False
            logger.warning(
                "Only ca_3d_coeffs_motor_off available. Using it for both motor states. "
                "CA_motor_multiplier will be applied for motor_on state."
            )
        else:
            self._use_ca_motor_pair = False

        # Load roll_aero_coeffs (Cll vs Mach, alpha, def_roll)
        if aero_tables.roll_aero_coeffs is not None:
            self._load_roll_aero_coeffs_from_df(aero_tables.roll_aero_coeffs)

        # Load fin_loads (hinge and bending moment coefficients)
        if aero_tables.fin_loads is not None:
            self._load_fin_loads_from_df(aero_tables.fin_loads)

        # Store reference to original tables for debugging
        self._aero_tables = aero_tables

    def _load_aero_coeffs_rocket_ws(self, df: pd.DataFrame):
        """Load aero_coeffs.csv (Cd, Cn, Cm vs Mach, alpha)."""
        logger.debug("Loading aero_coeffs (rocket_ws format)")
        
        self._validate_alpha_symmetry(
            df, 'aero_coeffs.csv',
            odd_columns=['Cn', 'Cm'],
            even_columns=['Cd']
        )

        mach_vals = np.sort(df['Mach'].unique())
        alpha_vals = np.sort(df['alpha_deg'].unique())

        # Build index maps for robust lookup (avoids float precision issues with searchsorted)
        mach_idx = _build_index_map(mach_vals)
        alpha_idx = _build_index_map(alpha_vals)

        cd_grid = np.zeros((len(mach_vals), len(alpha_vals)))
        cn_grid = np.zeros((len(mach_vals), len(alpha_vals)))
        cm_grid = np.zeros((len(mach_vals), len(alpha_vals)))

        for _, row in df.iterrows():
            i = _get_index(mach_idx, row['Mach'])
            j = _get_index(alpha_idx, row['alpha_deg'])
            if i is not None and j is not None:
                cd_grid[i, j] = row['Cd']
                cn_grid[i, j] = row['Cn']
                cm_grid[i, j] = row['Cm']

        # Create 2D interpolators for CN, CM
        self._cn_interp = BoundedGridInterpolator(
            (mach_vals, alpha_vals),
            cn_grid,
            method='linear',
            dim_names=['Mach', 'alpha_deg']
        )
        self._cm_interp = BoundedGridInterpolator(
            (mach_vals, alpha_vals),
            cm_grid,
            method='linear',
            dim_names=['Mach', 'alpha_deg']
        )
        self._cn_cm_interp = True
        self._aero_coeffs_df = df
        
        # Store Mach and alpha bounds from this table (primary source for bounds)
        self._update_mach_bounds(mach_vals.min(), mach_vals.max())
        self._update_alpha_bounds(alpha_vals.min(), alpha_vals.max())

        # Fin drag per fin: Cd_per_fin = (Cd(Mach,δ) - Cd(Mach,0)) / 4, using alpha_deg as delta_deg
        zero_idx = np.argmin(np.abs(alpha_vals))

        # Compute Cd_delta per fin: (Cd - Cd_0) / 4
        cd_delta_per_fin_grid = np.zeros_like(cd_grid)
        for i in range(len(mach_vals)):
            cd_at_zero = cd_grid[i, zero_idx]
            for j in range(len(alpha_vals)):
                cd_delta_per_fin_grid[i, j] = (cd_grid[i, j] - cd_at_zero) / 4.0

        # Fin drag interpolator using |alpha| as delta (drag is symmetric)
        delta_vals = np.abs(alpha_vals)
        unique_delta_vals = np.sort(np.unique(delta_vals))

        # Rebuild grid for positive values only
        cd_delta_fin_grid = np.zeros((len(mach_vals), len(unique_delta_vals)))
        for i in range(len(mach_vals)):
            for j, delta in enumerate(unique_delta_vals):
                # Find corresponding alpha index (positive or negative)
                pos_idx = np.where(np.isclose(alpha_vals, delta))[0]
                neg_idx = np.where(np.isclose(alpha_vals, -delta))[0]
                if len(pos_idx) > 0:
                    cd_delta_fin_grid[i, j] = cd_delta_per_fin_grid[i, pos_idx[0]]
                elif len(neg_idx) > 0:
                    cd_delta_fin_grid[i, j] = cd_delta_per_fin_grid[i, neg_idx[0]]
        
        self._cd_delta_interp = BoundedGridInterpolator(
            (mach_vals, unique_delta_vals),
            cd_delta_fin_grid,
            method='linear',
            dim_names=['Mach', 'delta_deg']
        )
        self._fin_drag_interp = True
        self._update_delta_bounds(unique_delta_vals.min(), unique_delta_vals.max())
        
        logger.info(f"Loaded aero_coeffs: {len(mach_vals)} Mach x {len(alpha_vals)} alpha points, "
                    f"Mach range [{mach_vals.min():.2f}, {mach_vals.max():.2f}], "
                    f"alpha range [{alpha_vals.min():.1f}, {alpha_vals.max():.1f}] deg")
        logger.info(f"Created fin drag interpolator from Cd values: "
                    f"delta range [0, {unique_delta_vals.max():.1f}] deg, "
                    f"Cd_per_fin formula: (Cd(Mach, delta) - Cd(Mach, 0)) / 4")

    def _load_damping_from_df(self, df: pd.DataFrame):
        """Load damping coefficients (CMq, CNq, Clp, Clnr, Cyr) from DataFrame."""
        logger.debug("Loading damping_coeffs")
        
        # Check which columns are available
        has_clnr = 'Clnr' in df.columns
        has_cyr = 'Cyr' in df.columns
        
        self._validate_alpha_symmetry(
            df, 'damping_coeffs.csv',
            odd_columns=[],
            even_columns=['Cmq', 'Cnq', 'Clp']
        )

        mach_vals = np.sort(df['Mach'].unique())
        alpha_vals = np.sort(df['alpha_deg'].unique())

        # Build index maps for robust lookup (avoids float precision issues with searchsorted)
        mach_idx = _build_index_map(mach_vals)
        alpha_idx = _build_index_map(alpha_vals)

        cmq_grid = np.zeros((len(mach_vals), len(alpha_vals)))
        cnq_grid = np.zeros((len(mach_vals), len(alpha_vals)))
        clp_grid = np.zeros((len(mach_vals), len(alpha_vals)))
        clnr_grid = np.zeros((len(mach_vals), len(alpha_vals)))
        cyr_grid = np.zeros((len(mach_vals), len(alpha_vals)))

        for _, row in df.iterrows():
            i = _get_index(mach_idx, row['Mach'])
            j = _get_index(alpha_idx, row['alpha_deg'])
            if i is not None and j is not None:
                cmq_grid[i, j] = row['Cmq']
                cnq_grid[i, j] = row['Cnq']
                clp_grid[i, j] = row['Clp']
                if has_clnr:
                    clnr_grid[i, j] = row['Clnr']
                if has_cyr:
                    cyr_grid[i, j] = row['Cyr']

        # Using BoundedGridInterpolator to prevent extrapolation beyond table bounds
        self._cmq_interp = BoundedGridInterpolator(
            (mach_vals, alpha_vals),
            cmq_grid,
            method='linear',
            dim_names=['Mach', 'alpha_deg']
        )
        self._cnq_interp = BoundedGridInterpolator(
            (mach_vals, alpha_vals),
            cnq_grid,
            method='linear',
            dim_names=['Mach', 'alpha_deg']
        )
        self._clp_interp = BoundedGridInterpolator(
            (mach_vals, alpha_vals),
            clp_grid,
            method='linear',
            dim_names=['Mach', 'alpha_deg']
        )
        
        # Load Clnr and Cyr if available (for force damping and correct yaw moment damping)
        if has_clnr:
            self._clnr_interp = BoundedGridInterpolator(
                (mach_vals, alpha_vals),
                clnr_grid,
                method='linear',
                dim_names=['Mach', 'alpha_deg']
            )
            logger.info("Loaded Clnr (yaw moment damping derivative)")
        else:
            self._clnr_interp = None
            logger.warning("Clnr column not found in damping_coeffs.csv - using Cnq for yaw moment damping (legacy behavior)")
            
        if has_cyr:
            self._cyr_interp = BoundedGridInterpolator(
                (mach_vals, alpha_vals),
                cyr_grid,
                method='linear',
                dim_names=['Mach', 'alpha_deg']
            )
            logger.info("Loaded Cyr (side force damping derivative)")
        else:
            self._cyr_interp = None
            logger.info("Cyr column not found in damping_coeffs.csv - side force damping disabled")
            
        self._damping_interp = True
        
        # Update Mach and alpha bounds from this table
        self._update_mach_bounds(mach_vals.min(), mach_vals.max())
        self._update_alpha_bounds(alpha_vals.min(), alpha_vals.max())

        logger.info(f"Loaded damping_coeffs: {len(mach_vals)} Mach x {len(alpha_vals)} alpha points, "
                    f"Mach range [{mach_vals.min():.2f}, {mach_vals.max():.2f}], "
                    f"force damping: CNq={'yes' if has_clnr else 'no'}, Cyr={'yes' if has_cyr else 'no'}")

    def _load_fin_drag_from_df(self, df: pd.DataFrame):
        """Load fin deflection drag from DataFrame."""
        logger.debug("Loading fin_deflection_drag")

        mach_vals = np.sort(df['Mach'].unique())
        delta_vals = np.sort(df['delta_deg'].unique())

        # Build index maps for robust lookup (avoids float precision issues with searchsorted)
        mach_idx = _build_index_map(mach_vals)
        delta_idx = _build_index_map(delta_vals)

        cd_delta_grid = np.zeros((len(mach_vals), len(delta_vals)))

        for _, row in df.iterrows():
            i = _get_index(mach_idx, row['Mach'])
            j = _get_index(delta_idx, row['delta_deg'])
            if i is not None and j is not None:
                cd_delta_grid[i, j] = row['Cd_delta']

        # Using BoundedGridInterpolator to prevent extrapolation beyond table bounds
        self._cd_delta_interp = BoundedGridInterpolator(
            (mach_vals, delta_vals),
            cd_delta_grid,
            method='linear',
            dim_names=['Mach', 'delta_deg']
        )
        self._fin_drag_interp = True
        
        # Update Mach and delta bounds from this table
        self._update_mach_bounds(mach_vals.min(), mach_vals.max())
        self._update_delta_bounds(delta_vals.min(), delta_vals.max())
        
        logger.info(f"Loaded fin_deflection_drag: {len(mach_vals)} Mach x {len(delta_vals)} delta points, "
                    f"Mach range [{mach_vals.min():.2f}, {mach_vals.max():.2f}], "
                    f"delta range [{delta_vals.min():.1f}, {delta_vals.max():.1f}] deg")

    def _load_ca_3d_from_df(self, df: pd.DataFrame):
        """Load 3D axial force coefficient table from DataFrame."""
        logger.debug("Loading ca_3d_coeffs")

        # Check if Reynolds column exists
        if 'Reynolds' not in df.columns:
            logger.warning("ca_3d_coeffs has no Reynolds column, skipping (use ca_3d_coeffs_motor_on/off instead)")
            return
        else:
            # 3D table (Mach x alpha x Reynolds)
            mach_vals = np.sort(df['Mach'].unique())
            alpha_vals = np.sort(df['alpha_deg'].unique())
            reynolds_vals = np.sort(df['Reynolds'].unique())

            # Build index maps for robust lookup (avoids float precision issues with searchsorted)
            mach_idx = _build_index_map(mach_vals)
            alpha_idx = _build_index_map(alpha_vals)
            reynolds_idx = _build_index_map(reynolds_vals)

            ca_grid = np.zeros((len(mach_vals), len(alpha_vals), len(reynolds_vals)))

            for _, row in df.iterrows():
                i = _get_index(mach_idx, row['Mach'])
                j = _get_index(alpha_idx, row['alpha_deg'])
                k = _get_index(reynolds_idx, row['Reynolds'])
                if i is not None and j is not None and k is not None:
                    ca_grid[i, j, k] = row['CA']

            # Using BoundedGridInterpolator to prevent extrapolation beyond table bounds
            self._ca_interp = BoundedGridInterpolator(
                (mach_vals, alpha_vals, reynolds_vals),
                ca_grid,
                method='linear',
                dim_names=['Mach', 'alpha_deg', 'Reynolds']
            )
            self._ca_bounds = {
                'mach': (mach_vals[0], mach_vals[-1]),
                'alpha': (alpha_vals[0], alpha_vals[-1]),
                'reynolds': (reynolds_vals[0], reynolds_vals[-1])
            }
            
            # Update Mach, alpha, and Reynolds bounds from this table
            self._update_mach_bounds(mach_vals.min(), mach_vals.max())
            self._update_alpha_bounds(alpha_vals.min(), alpha_vals.max())
            self._update_reynolds_bounds(reynolds_vals.min(), reynolds_vals.max())
            
            logger.info(f"Loaded ca_3d_coeffs (3D): {len(mach_vals)} Mach x {len(alpha_vals)} alpha x {len(reynolds_vals)} Re points, "
                        f"Mach range [{mach_vals.min():.2f}, {mach_vals.max():.2f}], "
                        f"Re range [{reynolds_vals.min():.2e}, {reynolds_vals.max():.2e}]")

    def _load_ca_3d_motor_pair(self, df_motor_on: pd.DataFrame, df_motor_off: pd.DataFrame):
        """Load separate CA tables for motor_on and motor_off states."""
        logger.debug("Loading ca_3d_coeffs motor_on/motor_off pair")
        
        def build_ca_interpolator(df: pd.DataFrame, name: str):
            """Build interpolator for a single CA table."""
            has_reynolds = 'Reynolds' in df.columns
            
            if not has_reynolds:
                mach_vals = np.sort(df['Mach'].unique())
                alpha_vals = np.sort(df['alpha_deg'].unique())
                
                mach_idx = _build_index_map(mach_vals)
                alpha_idx = _build_index_map(alpha_vals)
                
                ca_grid = np.zeros((len(mach_vals), len(alpha_vals)))
                
                for _, row in df.iterrows():
                    i = _get_index(mach_idx, row['Mach'])
                    j = _get_index(alpha_idx, row['alpha_deg'])
                    if i is not None and j is not None:
                        ca_grid[i, j] = row['CA']
                
                interp = BoundedGridInterpolator(
                    (mach_vals, alpha_vals),
                    ca_grid,
                    method='linear',
                    dim_names=['Mach', 'alpha_deg']
                )
                bounds = {
                    'mach': (mach_vals[0], mach_vals[-1]),
                    'alpha': (alpha_vals[0], alpha_vals[-1]),
                }
                logger.info(f"Loaded {name} (2D): {len(mach_vals)} Mach x {len(alpha_vals)} alpha points")
                return interp, bounds, False
            else:
                mach_vals = np.sort(df['Mach'].unique())
                alpha_vals = np.sort(df['alpha_deg'].unique())
                reynolds_vals = np.sort(df['Reynolds'].unique())
                
                mach_idx = _build_index_map(mach_vals)
                alpha_idx = _build_index_map(alpha_vals)
                reynolds_idx = _build_index_map(reynolds_vals)
                
                ca_grid = np.zeros((len(mach_vals), len(alpha_vals), len(reynolds_vals)))
                
                for _, row in df.iterrows():
                    i = _get_index(mach_idx, row['Mach'])
                    j = _get_index(alpha_idx, row['alpha_deg'])
                    k = _get_index(reynolds_idx, row['Reynolds'])
                    if i is not None and j is not None and k is not None:
                        ca_grid[i, j, k] = row['CA']
                
                interp = BoundedGridInterpolator(
                    (mach_vals, alpha_vals, reynolds_vals),
                    ca_grid,
                    method='linear',
                    dim_names=['Mach', 'alpha_deg', 'Reynolds']
                )
                bounds = {
                    'mach': (mach_vals[0], mach_vals[-1]),
                    'alpha': (alpha_vals[0], alpha_vals[-1]),
                    'reynolds': (reynolds_vals[0], reynolds_vals[-1])
                }
                logger.info(f"Loaded {name} (3D): {len(mach_vals)} Mach x {len(alpha_vals)} alpha x {len(reynolds_vals)} Re points")
                return interp, bounds, True
        
        self._ca_motor_on_interp, self._ca_motor_on_bounds, self._ca_motor_on_has_reynolds = \
            build_ca_interpolator(df_motor_on, "ca_3d_coeffs_motor_on")
        self._ca_motor_off_interp, self._ca_motor_off_bounds, self._ca_motor_off_has_reynolds = \
            build_ca_interpolator(df_motor_off, "ca_3d_coeffs_motor_off")
        
        self._update_mach_bounds(
            max(self._ca_motor_on_bounds['mach'][0], self._ca_motor_off_bounds['mach'][0]),
            min(self._ca_motor_on_bounds['mach'][1], self._ca_motor_off_bounds['mach'][1])
        )
        self._update_alpha_bounds(
            max(self._ca_motor_on_bounds['alpha'][0], self._ca_motor_off_bounds['alpha'][0]),
            min(self._ca_motor_on_bounds['alpha'][1], self._ca_motor_off_bounds['alpha'][1])
        )
        
        if self._ca_motor_on_has_reynolds and self._ca_motor_off_has_reynolds:
            self._update_reynolds_bounds(
                max(self._ca_motor_on_bounds['reynolds'][0], self._ca_motor_off_bounds['reynolds'][0]),
                min(self._ca_motor_on_bounds['reynolds'][1], self._ca_motor_off_bounds['reynolds'][1])
            )
        
        logger.info(
            "CA motor_on/motor_off pair loaded. CA_motor_multiplier will NOT be applied."
        )

    def _load_fin_deflection_coeffs_from_df(self, df: pd.DataFrame):
        """Load fin deflection coefficients (3D, 2D Mach×delta, or 2D Mach×alpha)."""
        delta_col = None
        for col in ['delta_deg', 'def_deg', 'delta']:
            if col in df.columns:
                delta_col = col
                break

        has_alpha = 'alpha_deg' in df.columns

        if delta_col is not None:
            if has_alpha:
                self._load_fin_deflection_coeffs_3d(df, delta_col)
            else:
                self._load_fin_deflection_coeffs_mach_delta(df, delta_col)
        elif has_alpha:
            raise AeroDataError(
                "fin_deflection_coeffs.csv has Mach×alpha format (per-radian derivatives) "
                "but no delta column. This format is not supported because it produces "
                "incorrect control forces. Please provide data in one of these formats:\n"
                "  - 3D: Mach, alpha_deg, delta_deg, CN_delta, CM_delta\n"
                "  - 2D: Mach, delta_deg, CN_delta, CM_delta"
            )
        else:
            raise AeroDataError(
                "fin_deflection_coeffs.csv must have either 'delta_deg' or 'alpha_deg' column."
            )

    def _load_fin_deflection_coeffs_3d(self, df: pd.DataFrame, delta_col: str):
        """Load 3D fin deflection coefficients (Mach × alpha × delta)."""
        cn_col = self._find_column(df, ['CN_delta', 'Cn_delta', 'CNd'])
        cm_col = self._find_column(df, ['CM_delta', 'Cm_delta', 'CMd'])
        
        if cn_col is None or cm_col is None:
            raise AeroDataError(
                "fin_deflection_coeffs.csv 3D format requires CN_delta and CM_delta columns."
            )
        
        mach_vals = np.sort(df['Mach'].unique())
        alpha_vals = np.sort(df['alpha_deg'].unique())
        delta_vals = np.sort(df[delta_col].unique())
        
        mach_idx = _build_index_map(mach_vals)
        alpha_idx = _build_index_map(alpha_vals)
        delta_idx = _build_index_map(delta_vals)
        
        cn_grid = np.zeros((len(mach_vals), len(alpha_vals), len(delta_vals)))
        cm_grid = np.zeros((len(mach_vals), len(alpha_vals), len(delta_vals)))
        
        for _, row in df.iterrows():
            i = _get_index(mach_idx, row['Mach'])
            j = _get_index(alpha_idx, row['alpha_deg'])
            k = _get_index(delta_idx, row[delta_col])
            if i is not None and j is not None and k is not None:
                cn_grid[i, j, k] = row[cn_col]
                cm_grid[i, j, k] = row[cm_col]
        
        self._cn_delta_3d_interp = BoundedGridInterpolator(
            (mach_vals, alpha_vals, delta_vals),
            cn_grid,
            method='linear',
            dim_names=['Mach', 'alpha_deg', 'delta_deg']
        )
        self._cm_delta_3d_interp = BoundedGridInterpolator(
            (mach_vals, alpha_vals, delta_vals),
            cm_grid,
            method='linear',
            dim_names=['Mach', 'alpha_deg', 'delta_deg']
        )
        
        self._fin_deflection_coeffs_3d = True
        self._fin_effectiveness_interp = True
        self._fin_effectiveness_source = 'fin_deflection_coeffs_3d'
        
        self._update_mach_bounds(mach_vals.min(), mach_vals.max())
        self._update_alpha_bounds(alpha_vals.min(), alpha_vals.max())
        self._update_delta_bounds(delta_vals.min(), delta_vals.max())
        
        logger.info(
            f"Loaded fin_deflection_coeffs (3D): {len(mach_vals)} Mach x {len(alpha_vals)} alpha x "
            f"{len(delta_vals)} delta points, Mach [{mach_vals.min():.2f}, {mach_vals.max():.2f}], "
            f"alpha [{alpha_vals.min():.1f}, {alpha_vals.max():.1f}] deg, "
            f"delta [{delta_vals.min():.1f}, {delta_vals.max():.1f}] deg"
        )

    def _load_fin_deflection_coeffs_mach_delta(self, df: pd.DataFrame, delta_col: str):
        """Load 2D fin deflection coefficients (Mach × delta)."""
        cn_col = self._find_column(df, ['CN_delta', 'Cn_delta', 'CNd'])
        cm_col = self._find_column(df, ['CM_delta', 'Cm_delta', 'CMd'])
        
        if cn_col is None or cm_col is None:
            raise AeroDataError(
                "fin_deflection_coeffs.csv 2D (Mach x delta) format requires CN_delta and CM_delta columns."
            )
        
        mach_vals = np.sort(df['Mach'].unique())
        delta_vals = np.sort(df[delta_col].unique())
        
        mach_idx = _build_index_map(mach_vals)
        delta_idx = _build_index_map(delta_vals)
        
        cn_grid = np.zeros((len(mach_vals), len(delta_vals)))
        cm_grid = np.zeros((len(mach_vals), len(delta_vals)))
        
        for _, row in df.iterrows():
            i = _get_index(mach_idx, row['Mach'])
            j = _get_index(delta_idx, row[delta_col])
            if i is not None and j is not None:
                cn_grid[i, j] = row[cn_col]
                cm_grid[i, j] = row[cm_col]
        
        self._cn_delta_mach_delta_interp = BoundedGridInterpolator(
            (mach_vals, delta_vals),
            cn_grid,
            method='linear',
            dim_names=['Mach', 'delta_deg']
        )
        self._cm_delta_mach_delta_interp = BoundedGridInterpolator(
            (mach_vals, delta_vals),
            cm_grid,
            method='linear',
            dim_names=['Mach', 'delta_deg']
        )
        
        self._fin_deflection_coeffs_mach_delta = True
        self._fin_effectiveness_interp = True
        self._fin_effectiveness_source = 'fin_deflection_coeffs_mach_delta'
        
        self._update_mach_bounds(mach_vals.min(), mach_vals.max())
        self._update_delta_bounds(delta_vals.min(), delta_vals.max())
        
        logger.info(
            f"Loaded fin_deflection_coeffs (2D Mach x delta): {len(mach_vals)} Mach x "
            f"{len(delta_vals)} delta points, Mach [{mach_vals.min():.2f}, {mach_vals.max():.2f}], "
            f"delta [{delta_vals.min():.1f}, {delta_vals.max():.1f}] deg"
        )

    def _find_column(self, df: pd.DataFrame, candidates: list, required: bool = True) -> Optional[str]:
        """Find a column in DataFrame from a list of candidate names."""
        for col in candidates:
            if col in df.columns:
                return col
        if required:
            raise AeroDataError(
                f"Required column not found. Tried: {candidates}."
            )
        return None

    def _load_roll_aero_coeffs_from_df(self, df: pd.DataFrame):
        """Load roll aerodynamic coefficients table (Cll vs Mach, alpha, def_roll)."""
        logger.debug("Loading roll_aero_coeffs table")
        
        # Check for required columns - support both 'def_roll' and 'delta_roll'
        def_col = None
        for col in ['def_roll', 'delta_roll', 'def_roll_deg', 'delta_roll_deg']:
            if col in df.columns:
                def_col = col
                break
        
        if def_col is None:
            logger.warning(
                f"roll_aero_coeffs table missing deflection column (def_roll/delta_roll). "
                f"Available columns: {list(df.columns)}. Roll aero table will not be used."
            )
            return
        
        # Check for Cll column
        cll_col = None
        for col in ['Cll', 'CLL', 'Cl_roll', 'CL_roll']:
            if col in df.columns:
                cll_col = col
                break
        
        if cll_col is None:
            logger.warning(
                f"roll_aero_coeffs table missing Cll column. "
                f"Available columns: {list(df.columns)}. Roll aero table will not be used."
            )
            return
        
        if 'Mach' not in df.columns or 'alpha_deg' not in df.columns:
            logger.warning(
                f"roll_aero_coeffs table missing Mach or alpha_deg columns. "
                f"Available columns: {list(df.columns)}. Roll aero table will not be used."
            )
            return
        
        mach_vals = np.sort(df['Mach'].unique())
        alpha_vals = np.sort(df['alpha_deg'].unique())
        def_vals = np.sort(df[def_col].unique())
        
        # Build index maps for robust lookup
        mach_idx = _build_index_map(mach_vals)
        alpha_idx = _build_index_map(alpha_vals)
        def_idx = _build_index_map(def_vals)
        
        # Build 3D Cll grid (Mach x alpha x def_roll)
        cll_grid = np.zeros((len(mach_vals), len(alpha_vals), len(def_vals)))
        
        for _, row in df.iterrows():
            i = _get_index(mach_idx, row['Mach'])
            j = _get_index(alpha_idx, row['alpha_deg'])
            k = _get_index(def_idx, row[def_col])
            if i is not None and j is not None and k is not None:
                cll_grid[i, j, k] = row[cll_col]
        
        # Create 3D interpolator
        # Using BoundedGridInterpolator to prevent extrapolation beyond table bounds
        self._roll_aero_interp = BoundedGridInterpolator(
            (mach_vals, alpha_vals, def_vals),
            cll_grid,
            method='linear',
            dim_names=['Mach', 'alpha_deg', 'def_roll']
        )
        
        # Store bounds for reference
        self._roll_aero_mach_bounds = (mach_vals.min(), mach_vals.max())
        self._roll_aero_alpha_bounds = (alpha_vals.min(), alpha_vals.max())
        self._roll_aero_def_bounds = (def_vals.min(), def_vals.max())
        
        logger.info(
            f"Loaded roll_aero_coeffs: {len(mach_vals)} Mach x {len(alpha_vals)} alpha x {len(def_vals)} def_roll points, "
            f"Mach range [{mach_vals.min():.2f}, {mach_vals.max():.2f}], "
            f"alpha range [{alpha_vals.min():.1f}, {alpha_vals.max():.1f}] deg, "
            f"def_roll range [{def_vals.min():.1f}, {def_vals.max():.1f}] deg, "
            f"Cll range [{cll_grid.min():.4f}, {cll_grid.max():.4f}]"
        )

    def _load_fin_loads_from_df(self, df: pd.DataFrame):
        """Load fin structural loads table (hinge and bending moment coefficients)."""
        logger.debug("Loading fin_loads table")
        
        # Check for required columns
        required_cols = ['Mach', 'Ch_delta', 'Cb_delta', 'Ch_alpha', 'Cb_alpha']

        # Support both 'delta_deg' and 'def_deg' column names
        delta_col = None
        for col in ['delta_deg', 'def_deg', 'deflection_deg']:
            if col in df.columns:
                delta_col = col
                break
        
        if delta_col is None:
            logger.warning(
                f"fin_loads table missing deflection column (delta_deg/def_deg). "
                f"Available columns: {list(df.columns)}. Fin loads table will not be used."
            )
            return
        
        missing_cols = [col for col in required_cols if col not in df.columns]
        if missing_cols:
            logger.warning(
                f"fin_loads table missing columns: {missing_cols}. "
                f"Available columns: {list(df.columns)}. Fin loads table will not be used."
            )
            return
        
        # Use the helper functions to build grid axes and interpolators
        axis_cols = ['Mach', delta_col]
        axis_values, index_maps = _build_grid_axes(df, axis_cols)
        
        ch_delta_grid = _fill_grid_from_df(df, axis_cols, 'Ch_delta', axis_values, index_maps)
        cb_delta_grid = _fill_grid_from_df(df, axis_cols, 'Cb_delta', axis_values, index_maps)
        ch_alpha_grid = _fill_grid_from_df(df, axis_cols, 'Ch_alpha', axis_values, index_maps)
        cb_alpha_grid = _fill_grid_from_df(df, axis_cols, 'Cb_alpha', axis_values, index_maps)
        
        self._ch_delta_interp = _create_interpolator(axis_values, ch_delta_grid, dim_names=['Mach', 'delta_deg'])
        self._cb_delta_interp = _create_interpolator(axis_values, cb_delta_grid, dim_names=['Mach', 'delta_deg'])
        self._ch_alpha_interp = _create_interpolator(axis_values, ch_alpha_grid, dim_names=['Mach', 'delta_deg'])
        self._cb_alpha_interp = _create_interpolator(axis_values, cb_alpha_grid, dim_names=['Mach', 'delta_deg'])
        self._hinge_bending_interp = True
        
        mach_vals = axis_values[0]
        delta_vals = axis_values[1]
        
        logger.info(
            f"Loaded fin_loads: {len(mach_vals)} Mach x {len(delta_vals)} delta points,"
            f"Mach range [{mach_vals.min():.2f}, {mach_vals.max():.2f}], "
            f"delta range [{delta_vals.min():.1f}, {delta_vals.max():.1f}] deg, "
            f"Ch_delta range [{ch_delta_grid.min():.6f}, {ch_delta_grid.max():.6f}], "
            f"Cb_delta range [{cb_delta_grid.min():.6f}, {cb_delta_grid.max():.6f}]"
        )

    def _load_aero_tables(self):
        """Load all aerodynamic coefficient tables from CSV files."""

        try:
            ca_file = self.tables_path / 'ca_coefficients.csv'
            if ca_file.exists():
                self._load_ca_table(ca_file)
            else:
                logger.warning(f"{ca_file} not found, using simple CA model")
                self._ca_interp = None
        except Exception as e:
            logger.error(f"Error loading CA table: {e}")
            self._ca_interp = None

        try:
            cn_cm_file = self.tables_path / 'cn_cm_coefficients.csv'
            if cn_cm_file.exists():
                self._load_cn_cm_table(cn_cm_file)
            else:
                logger.warning(f"{cn_cm_file} not found, using simple CN/CM model")
                self._cn_cm_interp = None
        except Exception as e:
            logger.error(f"Error loading CN/CM table: {e}")
            self._cn_cm_interp = None

        try:
            damping_file = self.tables_path / 'damping_derivatives.csv'
            if damping_file.exists():
                self._load_damping_table(damping_file)
            else:
                logger.warning(f"{damping_file} not found, using constant damping")
                self._damping_interp = None
        except Exception as e:
            logger.error(f"Error loading damping table: {e}")
            self._damping_interp = None

        try:
            fin_drag_file = self.tables_path / 'fin_deflection_drag.csv'
            if fin_drag_file.exists():
                self._load_fin_drag_table(fin_drag_file)
            else:
                logger.warning(f"{fin_drag_file} not found, ignoring fin drag")
                self._fin_drag_interp = None
        except Exception as e:
            logger.error(f"Error loading fin drag table: {e}")
            self._fin_drag_interp = None

        try:
            hinge_file = self.tables_path / 'hinge_bending_coefficients.csv'
            if hinge_file.exists():
                self._load_hinge_bending_table(hinge_file)
            else:
                logger.warning(f"{hinge_file} not found, ignoring hinge moments")
                self._hinge_bending_interp = None
        except Exception as e:
            logger.error(f"Error loading hinge/bending table: {e}")
            self._hinge_bending_interp = None

        self._fin_effectiveness_interp = None

    def _load_ca_table(self, filepath: Path):
        """Load axial force coefficient table (Mach x alpha x Reynolds)."""
        df = pd.read_csv(filepath, comment='#')
        axis_cols = ['Mach', 'alpha_deg', 'Reynolds']
        axis_values, index_maps = _build_grid_axes(df, axis_cols)
        ca_grid = _fill_grid_from_df(df, axis_cols, 'CA', axis_values, index_maps)
        self._ca_interp = _create_interpolator(axis_values, ca_grid, dim_names=['Mach', 'alpha_deg', 'Reynolds'])
        self._ca_bounds = {
            'mach': (axis_values[0][0], axis_values[0][-1]),
            'alpha': (axis_values[1][0], axis_values[1][-1]),
            'reynolds': (axis_values[2][0], axis_values[2][-1])
        }

    def _load_cn_cm_table(self, filepath: Path):
        """Load normal force and pitching moment coefficient table (Mach x alpha)."""
        df = pd.read_csv(filepath, comment='#')
        axis_cols = ['Mach', 'alpha_deg']
        axis_values, index_maps = _build_grid_axes(df, axis_cols)
        cn_grid = _fill_grid_from_df(df, axis_cols, 'CN', axis_values, index_maps)
        cm_grid = _fill_grid_from_df(df, axis_cols, 'CM', axis_values, index_maps)
        self._cn_interp = _create_interpolator(axis_values, cn_grid, dim_names=['Mach', 'alpha_deg'])
        self._cm_interp = _create_interpolator(axis_values, cm_grid, dim_names=['Mach', 'alpha_deg'])
        self._cn_cm_interp = True

    def _load_damping_table(self, filepath: Path):
        """Load damping derivatives table (Mach x alpha)."""
        df = pd.read_csv(filepath, comment='#')
        axis_cols = ['Mach', 'alpha_deg']
        axis_values, index_maps = _build_grid_axes(df, axis_cols)
        cmq_grid = _fill_grid_from_df(df, axis_cols, 'Cmq', axis_values, index_maps)
        cnq_grid = _fill_grid_from_df(df, axis_cols, 'Cnq', axis_values, index_maps)
        clp_grid = _fill_grid_from_df(df, axis_cols, 'Clp', axis_values, index_maps)
        self._cmq_interp = _create_interpolator(axis_values, cmq_grid, dim_names=['Mach', 'alpha_deg'])
        self._cnq_interp = _create_interpolator(axis_values, cnq_grid, dim_names=['Mach', 'alpha_deg'])
        self._clp_interp = _create_interpolator(axis_values, clp_grid, dim_names=['Mach', 'alpha_deg'])
        self._damping_interp = True

    def _load_fin_drag_table(self, filepath: Path):
        """Load fin deflection drag table (Mach x delta)."""
        df = pd.read_csv(filepath, comment='#')
        axis_cols = ['Mach', 'delta_deg']
        axis_values, index_maps = _build_grid_axes(df, axis_cols)
        cd_delta_grid = _fill_grid_from_df(df, axis_cols, 'Cd_delta', axis_values, index_maps)
        self._cd_delta_interp = _create_interpolator(axis_values, cd_delta_grid, dim_names=['Mach', 'delta_deg'])
        self._fin_drag_interp = True

    def _load_hinge_bending_table(self, filepath: Path):
        """Load hinge and bending moment coefficient table (Mach x delta)."""
        df = pd.read_csv(filepath, comment='#')
        axis_cols = ['Mach', 'delta_deg']
        axis_values, index_maps = _build_grid_axes(df, axis_cols)
        ch_delta_grid = _fill_grid_from_df(df, axis_cols, 'Ch_delta', axis_values, index_maps)
        cb_delta_grid = _fill_grid_from_df(df, axis_cols, 'Cb_delta', axis_values, index_maps)
        ch_alpha_grid = _fill_grid_from_df(df, axis_cols, 'Ch_alpha', axis_values, index_maps)
        cb_alpha_grid = _fill_grid_from_df(df, axis_cols, 'Cb_alpha', axis_values, index_maps)
        self._ch_delta_interp = _create_interpolator(axis_values, ch_delta_grid, dim_names=['Mach', 'delta_deg'])
        self._cb_delta_interp = _create_interpolator(axis_values, cb_delta_grid, dim_names=['Mach', 'delta_deg'])
        self._ch_alpha_interp = _create_interpolator(axis_values, ch_alpha_grid, dim_names=['Mach', 'delta_deg'])
        self._cb_alpha_interp = _create_interpolator(axis_values, cb_alpha_grid, dim_names=['Mach', 'delta_deg'])
        self._hinge_bending_interp = True

    def _load_launcher_dynamics(self, config: dict):
        """Load launcher dynamics parameters."""
        launcher_config = config.get('launcher', {})

        self.rail_length = launcher_config.get('rail_length', 9.5)
        self.rail_effective_length = launcher_config.get('rail_effective_length', 4.85)
        self.rail_friction = launcher_config.get('rail_friction_coefficient', 0.1)
        self.launch_elevation = np.radians(launcher_config.get('launch_elevation', 45.0))

        self.on_rail = False
        self.rail_distance = 0.0

    def get_forces_moments(self, velocity_ned: np.ndarray, angular_velocity_body: np.ndarray,
                          quaternion: np.ndarray, q_dyn: float, rho: float,
                          control_input: np.ndarray, motor_on: bool = False,
                          sound_speed: float = 340.0,
                          dynamic_viscosity: float = None,
                          cg_position: float = None) -> AeroState:
        """Calculate aerodynamic forces and moments with full feature support."""

        # R maps body->NED: v_ned = R @ v_body, so v_body = R.T @ v_ned.
        R = self._quaternion_to_rotation_matrix(quaternion)

        # Validate rotation matrix: det(R) should be 1.0 for proper rotation
        det_R = np.linalg.det(R)
        if abs(det_R - 1.0) > 1e-4:
            logger.warning(
                f"Rotation matrix determinant {det_R:.6f} != 1.0, "
                f"quaternion may be invalid (q={quaternion}). Using identity rotation."
            )
            R = np.eye(3)
        
        velocity_body = R.T @ velocity_ned  # NED→body transformation

        V = np.linalg.norm(velocity_body)
        V_ned_norm = np.linalg.norm(velocity_ned)
        
        # Validate velocity magnitude consistency after frame transform
        # Rotation should preserve vector magnitude (orthogonal transformation)
        if V_ned_norm > 1.0 and abs(V - V_ned_norm) > 1e-3:
            logger.error(
                f"CRITICAL: Velocity magnitude changed during NED→Body transform! "
                f"|V_NED|={V_ned_norm:.4f} m/s, |V_body|={V:.4f} m/s, "
                f"diff={abs(V - V_ned_norm):.6f} m/s. "
                f"This indicates invalid rotation matrix or quaternion."
            )

        # Low-speed guard: below this airspeed, velocity direction is noisy, so alpha/beta are forced to zero.
        MIN_AIRSPEED_FOR_AERO = 5.0  # m/s - minimum airspeed for meaningful aero angles

        if V < MIN_AIRSPEED_FOR_AERO:
            # At low speeds, return minimal aerodynamic state with zero angles
            # This prevents spurious moments from destabilizing the rocket during launch
            return AeroState(
                forces=np.zeros(3),
                moments=np.zeros(3),
                alpha=0.0,
                beta=0.0,
                alpha_total=0.0,
                mach=0.0,
                mach_raw=V / sound_speed if sound_speed > 0 else 0.0,
                reynolds=0.0,
                CA=0.3,
                CN=0.0,
                CM=0.0,
                CY=0.0,
                Cn=0.0,
                Cl=0.0,
                fin_drag=np.zeros(4),
                fin_moments=None,
                CN_control=0.0,
                CM_control=0.0,
                CY_control=0.0,
                Cn_control=0.0,
                M_roll=0.0,
                M_pitch=0.0,
                M_yaw=0.0
            )

        # α = atan2(w, u): positive α = nose up (standard aerospace convention)
        alpha = np.arctan2(velocity_body[2], velocity_body[0])
        
        # β = atan2(v, √(u²+w²)): geometrically exact sideslip (accurate at high α)
        longitudinal_speed = np.sqrt(velocity_body[0]**2 + velocity_body[2]**2)
        beta = np.arctan2(velocity_body[1], longitudinal_speed) if longitudinal_speed > 1e-6 else 0.0
        
        alpha_total = np.arctan2(
            np.sqrt(velocity_body[1]**2 + velocity_body[2]**2),
            velocity_body[0]
        )

        mach_raw = V / sound_speed
        # Clamp Mach to table bounds (dynamically set from loaded data intersection)
        mach_min = self._mach_min if self._mach_min is not None else 0.3
        mach_max = self._mach_max if self._mach_max is not None else 11.0
        mach = np.clip(mach_raw, mach_min, mach_max)

        # Sub-table fade: when mach_raw < mach_min, coefficient tables are extrapolated.
        # Apply smooth quadratic fade to forces/moments to avoid unrealistic aero at launch.
        # Factor = (mach_raw/mach_min)² matches dynamic-pressure scaling rationale.
        if mach_raw < mach_min and mach_min > 0:
            _sub_table_fade = (mach_raw / mach_min) ** 2
        else:
            _sub_table_fade = 1.0

        # Clamp alpha to table bounds (deg, defaults = typical rocket range)
        alpha_min_deg = self._alpha_min if self._alpha_min is not None else 0.0
        alpha_max_deg = self._alpha_max if self._alpha_max is not None else 20.0
        
        # Clamp delta (fin deflection) to table bounds (deg, defaults = typical limits)
        delta_min_deg = self._delta_min if self._delta_min is not None else -20.0
        delta_max_deg = self._delta_max if self._delta_max is not None else 20.0

        if dynamic_viscosity is not None and dynamic_viscosity > 0:
            mu = dynamic_viscosity
        else:
            # Derive T from speed of sound: T = a²/(γR), more accurate than fixed 288.15 K
            gamma = 1.4
            R_air = 287.05  # J/(kg·K)
            T = sound_speed**2 / (gamma * R_air)
            # Sutherland's law: μ = C₁·T^1.5/(T+S), C₁=1.458e-6, S=110.4 K
            mu = 1.458e-6 * T**1.5 / (T + 110.4)
        reynolds_raw = rho * V * self.L_ref / mu if mu > 0 else 1e6
        # Clamp Reynolds to table bounds (defaults = typical rocket Re range)
        reynolds_min = self._reynolds_min if self._reynolds_min is not None else 1e5
        reynolds_max = self._reynolds_max if self._reynolds_max is not None else 6e7
        reynolds = np.clip(reynolds_raw, reynolds_min, reynolds_max)

        # Angle conversions: signed for sign restoration, fabs for table lookup
        
        alpha_deg_signed = np.degrees(alpha)

        # Alpha in degrees (with fabs) for damping coefficients CMq, CNq
        alpha_deg_abs = np.degrees(np.fabs(alpha))
        alpha_deg_for_damping = np.clip(alpha_deg_abs, alpha_min_deg, alpha_max_deg)

        beta_deg_signed = np.degrees(beta)

        # Beta in degrees (with fabs) for Cyr, Clnr
        beta_deg_abs = np.degrees(np.fabs(beta))
        beta_deg_for_damping = np.clip(beta_deg_abs, alpha_min_deg, alpha_max_deg)

        # Alpha total in degrees (with fabs) for Clp and Cll_from_table
        alpha_total_deg = np.degrees(alpha_total)
        alpha_total_deg_abs = np.degrees(np.fabs(alpha_total))
        alpha_total_deg_for_damping = np.clip(alpha_total_deg_abs, alpha_min_deg, alpha_max_deg)

        CA = self._get_axial_force_coeff(mach, alpha_total_deg, reynolds, motor_on)

        # CN_base, CM_base: lookup with |α|, then restore sign
        CN_base, CM_base = self._get_normal_moment_coeffs(mach, alpha_deg_for_damping)
        alpha_sign = np.where(alpha_deg_signed >= 0, 1.0, -1.0)
        CN_base = CN_base * alpha_sign * -1
        CM_base = CM_base * alpha_sign

        # CY_base, Cn_base: lookup with |β|, restore sign (axial symmetry, same table as α)
        CY_base, Cn_base = self._get_side_yaw_coeffs(mach, beta_deg_for_damping)
        beta_sign = np.where(beta_deg_signed >= 0, 1.0, -1.0)
        CY_base = -CY_base * beta_sign
        Cn_base = -Cn_base * beta_sign

        # Damping derivatives (CMq, CNq use |α|; Cyr, Clnr use |β|; Clp uses |α_total|)
        CMq, CNq, Clp, Clnr, Cyr = self._get_damping_derivatives(
            mach, alpha_deg_for_damping, beta_deg_for_damping, alpha_total_deg_for_damping
        )

        # ddv = D/(2V) [per_rad] or D/(2V)×r2d [per_deg] — non-dimensional angular rate factor
        r2d = np.degrees(1.0)  # radians to degrees conversion factor ≈ 57.2958
        if self.damping_coefficients_units == 'per_deg':
            ddv = self.D_body / (2.0 * V) * r2d
        else:
            ddv = self.D_body / (2.0 * V)
        p, q, r = angular_velocity_body

        # Damping: force (CNq·q, Cyr·r) and moment (CMq·q, Clp·p, Clnr·r) × ddv

        # Force damping
        CN_damp = CNq * q * ddv  # Normal force damping due to pitch rate
        CY_damp = Cyr * r * ddv  # Side force damping due to yaw rate

        # Moment damping
        CM_damp = CMq * q * ddv  # Pitch moment damping (CMq is negative = stabilizing)
        Cl_damp = Clp * p * ddv  # Roll moment damping (Clp is negative = stabilizing)

        # Yaw moment damping - uses CLNR (correct) instead of CNQ (incorrect)
        if self.negate_Cnq:
            # Force stabilizing sign regardless of table data
            Cn_damp = -np.abs(Clnr) * r * ddv
        else:
            # Trust the sign convention in the data files
            Cn_damp = Clnr * r * ddv

        # Fin mixing: δ1-δ4 → δ_pitch, δ_yaw, δ_roll (X: all 4 fins; +: paired fins)
        
        # Remap physical fin channels to internal ordering (if configured)
        if self._needs_fin_remapping:
            # Remap: internal_delta[i] = physical_delta[mapping[i]]
            control_input = np.array([
                control_input[self._fin_channel_mapping[0]],
                control_input[self._fin_channel_mapping[1]],
                control_input[self._fin_channel_mapping[2]],
                control_input[self._fin_channel_mapping[3]]
            ])
        
        delta1, delta2, delta3, delta4 = control_input
        
        # For fin drag calculation, clamp magnitude only (preserve sign for mixing)
        delta_max_rad = np.radians(delta_max_deg)
        delta_deg = np.degrees(np.array([
            min(np.abs(delta1), delta_max_rad),
            min(np.abs(delta2), delta_max_rad),
            min(np.abs(delta3), delta_max_rad),
            min(np.abs(delta4), delta_max_rad)
        ]))

        # Use RAW (unclamped) values for mixing to preserve control signal sign
        if self.config_type == 'X':
            # X configuration: all 4 fins contribute to pitch/yaw, common-mode roll
            delta_pitch = 0.25 * (-delta1 - delta2 + delta3 + delta4)
            delta_yaw = 0.25 * (-delta1 + delta2 + delta3 - delta4)
            delta_roll = 0.25 * (delta1 + delta2 + delta3 + delta4)
        else:  # config_type == '+'
            # + configuration: pitch uses top/bottom, yaw uses right/left, differential roll
            delta_pitch = 0.5 * (-delta1 + delta3)
            delta_yaw = 0.5 * (delta2 - delta4)
            delta_roll = 0.25 * (delta1 - delta2 + delta3 - delta4)

        # Get fin effectiveness coefficients (derivatives)
        # CN_delta and CM_delta are derivatives: dCN/d(delta), dCM/d(delta)
        # Query at a reference delta to get the derivative values
        delta_pitch_deg = np.degrees(np.abs(delta_pitch))
        delta_yaw_deg = np.degrees(np.abs(delta_yaw))
        delta_roll_deg = np.degrees(np.abs(delta_roll))

        # Pitch effectiveness derivatives (CN_delta, CM_delta) using |alpha| lookup
        CN_delta, CM_delta = self._get_fin_effectiveness(
            mach, alpha_deg_for_damping, delta_pitch_deg
        )

        # Restore control direction: CN_delta/CM_delta use |delta| lookup, so apply sign(delta).
        pitch_sign = np.sign(delta_pitch)
        CN_control = CN_delta * pitch_sign
        CM_control = CM_delta * pitch_sign

        # For yaw, query with yaw deflection magnitude
        CN_delta_yaw, CM_delta_yaw = self._get_fin_effectiveness(
            mach, alpha_deg_for_damping, delta_yaw_deg
        )
        yaw_sign = np.sign(delta_yaw)
        CY_control = CN_delta_yaw * yaw_sign
        Cn_control = CM_delta_yaw * yaw_sign

        # Roll: use roll_aero_coeffs table (if available) with Mach, |alpha_total|, delta_roll_deg.
        Cll_from_table = self.get_roll_moment_coeff(mach, alpha_total_deg_for_damping, delta_roll_deg)
        if Cll_from_table is not None:
            # Table gives Cll for |delta| only; apply sign(delta_roll) for odd-symmetric extension.
            Cl_control = np.sign(delta_roll) * Cll_from_table
        else:
            # No roll coefficient data available, use zero
            Cl_control = 0.0

        # Canard correction: fins ahead of CG reverse pitch/yaw control moment signs vs tail fins.
        if self.fin_location == 'canard':
            CM_control = -CM_control  # Negate pitch control moment
            Cn_control = -Cn_control  # Negate yaw control moment
            # Roll is not negated here; its sign is handled by roll geometry/table modeling.

        fin_drag = np.zeros(4)
        CA_fin_total = 0.0
        if self.compute_fin_drag and self._fin_drag_interp:
            for i, d_deg in enumerate(delta_deg):
                cd_delta = self._get_fin_drag_coeff(mach, d_deg)
                fin_drag[i] = cd_delta
                CA_fin_total += cd_delta

        CA_total = CA + CA_fin_total
        CN_total = CN_base + CN_control + CN_damp  # Normal force with damping
        CY_total = CY_base + CY_control + CY_damp  # Side force with damping
        Cl_total = Cl_damp + Cl_control

        forcc = q_dyn * self.S_ref
        torqc = forcc * self.D_body

        # Reference-point correction: transfer table moments to CG using M_CG = M_ref + (x_CG - x_ref) x F.

        # Total moment coefficients = static (from table) + damping + control
        CM_total = CM_base + CM_damp + CM_control
        Cn_total = Cn_base + Cn_damp + Cn_control

        # Apply moment transfer correction if configured
        if self.moment_reference_point is not None and cg_position is not None:
            # Moment arm ref->CG; x_ref and x_cg are from nose, positive toward tail.
            delta_x = cg_position - self.moment_reference_point

            # Non-dimensionalize by reference length
            delta_x_nd = delta_x / self.L_ref

            # Cross-product transfer gives: ΔCm = -CN_total * delta_x_nd, ΔCn = +CY_total * delta_x_nd.
            # Keep moment_reference_point and xcg_0 consistent to avoid double-counting CG correction.
            CM_transfer = -CN_total * delta_x_nd
            CM_total = CM_total + CM_transfer

            Cn_transfer = CY_total * delta_x_nd
            Cn_total = Cn_total + Cn_transfer

        # Warn if flying at extreme angles (outside table validity)
        # Only warn when velocity is high enough for alpha to be meaningful (> 50 m/s)
        # At low speeds, alpha is mathematically undefined and the warning is misleading
        MIN_VELOCITY_FOR_ALPHA_WARNING = 50.0  # m/s
        HIGH_ALPHA_THRESHOLD = np.radians(30)  # 30 degrees
        
        if np.abs(alpha_total) > HIGH_ALPHA_THRESHOLD and V > MIN_VELOCITY_FOR_ALPHA_WARNING:
            if not getattr(self, '_high_alpha_warned', False):
                warnings.warn(
                    f"High angle of attack detected (alpha_total={np.degrees(alpha_total):.1f}°). "
                    f"Aerodynamic coefficients may be outside table validity range.",
                    AeroInterpolationWarning,
                    stacklevel=2
                )
                self._high_alpha_warned = True
        
        # Compute velocity unit vector in body frame
        velocity_unit = velocity_body / V

        if np.abs(alpha_total) <= HIGH_ALPHA_THRESHOLD:
            # Traditional body-frame decomposition for low AoA
            u_sign = np.sign(velocity_body[0]) if np.abs(velocity_body[0]) > 0.1 else 1.0
            F_axial = -forcc * CA_total * u_sign
            F_normal = forcc * CN_total
            F_side = forcc * CY_total
            forces_body = np.array([F_axial, F_side, F_normal])
        else:
            # High AoA: apply drag along velocity with cross-flow-enhanced effective drag.
            sin_alpha = np.sin(np.abs(alpha_total))
            cos_alpha = np.cos(np.abs(alpha_total))

            # Effective drag coefficient: blend between axial (CA) and crossflow
            # At 90° AoA, use a higher drag coefficient (typical bluff body ~1.0-1.2)
            CD_crossflow = 1.2  # Typical bluff body drag coefficient
            CD_effective = CA_total * cos_alpha**2 + CD_crossflow * sin_alpha**2

            # Total drag force magnitude
            F_drag_magnitude = forcc * CD_effective

            # Apply drag force opposing velocity direction
            F_drag_body = -F_drag_magnitude * velocity_unit

            # Add lift/side forces (perpendicular to velocity)
            F_perp_normal = forcc * CN_total * cos_alpha
            F_perp_side = forcc * CY_total * cos_alpha
            F_non_drag_body = np.array([0.0, F_perp_side, F_perp_normal])
            F_along_v = np.dot(F_non_drag_body, velocity_unit)
            F_non_drag_body = F_non_drag_body - F_along_v * velocity_unit

            # Combine drag and perpendicular forces
            forces_body = F_drag_body + F_non_drag_body
        
        # Energy guard: enforce F_aero · v_air <= 0 so aerodynamics cannot add kinetic energy.
        F_along_velocity = np.dot(forces_body, velocity_unit)
        if F_along_velocity > 0:
            # Remove the accelerating component
            forces_body = forces_body - F_along_velocity * velocity_unit

        M_roll = torqc * Cl_total
        M_pitch = torqc * CM_total
        M_yaw = torqc * Cn_total

        # Apply sub-table fade for sub-mach_min regime
        if _sub_table_fade < 1.0:
            forces_body = forces_body * _sub_table_fade
            M_roll *= _sub_table_fade
            M_pitch *= _sub_table_fade
            M_yaw *= _sub_table_fade

        moments_body = np.array([M_roll, M_pitch, M_yaw])

        fin_moments = None
        if self.compute_hinge_moments and self._hinge_bending_interp:
            fin_moments = self._compute_fin_moments(
                mach, control_input, alpha, beta, q_dyn
            )

        return AeroState(
            forces=forces_body,
            moments=moments_body,
            alpha=alpha,
            beta=beta,
            alpha_total=alpha_total,
            mach=mach,
            mach_raw=mach_raw,
            reynolds=reynolds,
            CA=CA_total,
            CN=CN_total,
            CM=CM_total,
            CY=CY_total,
            Cn=Cn_total,
            Cl=Cl_total,
            fin_drag=fin_drag,
            fin_moments=fin_moments,
            CN_control=CN_control,
            CM_control=CM_control,
            CY_control=CY_control,
            Cn_control=Cn_control,
            M_roll=M_roll,
            M_pitch=M_pitch,
            M_yaw=M_yaw
        )

    def _get_axial_force_coeff(self, mach: float, alpha_deg: float,
                               reynolds: float, motor_on: bool) -> float:
        """Get axial force coefficient from available tables."""
        CA = None

        # Priority 1: Use motor_on/motor_off pair if available
        if getattr(self, '_use_ca_motor_pair', False):
            interp = self._ca_motor_on_interp if motor_on else self._ca_motor_off_interp
            has_reynolds = self._ca_motor_on_has_reynolds if motor_on else self._ca_motor_off_has_reynolds
            
            try:
                if has_reynolds:
                    CA = float(interp((mach, alpha_deg, reynolds)))
                else:
                    CA = float(interp((mach, alpha_deg)))
            except (ValueError, IndexError) as e:
                state_name = "motor_on" if motor_on else "motor_off"
                warnings.warn(
                    f"CA {state_name} interpolation failed (Mach={mach:.2f}, alpha={alpha_deg:.2f}): {e}",
                    AeroInterpolationWarning,
                    stacklevel=2
                )
        
        # Priority 2: Try legacy 3D CA table (Mach x alpha x Reynolds)
        if CA is None and self._ca_interp is not None:
            try:
                CA = float(self._ca_interp((mach, alpha_deg, reynolds)))
                if motor_on:
                    CA *= self.CA_motor_multiplier
            except (ValueError, IndexError) as e:
                warnings.warn(
                    f"CA 3D interpolation failed (Mach={mach:.2f}, alpha={alpha_deg:.2f}, Re={reynolds:.0f}): {e}",
                    AeroInterpolationWarning,
                    stacklevel=2
                )

        if CA is None:
            raise AeroDataError(
                f"No axial force coefficient table available for Mach={mach}, alpha={alpha_deg}"
            )

        # Always apply CA_multiplier after getting CA from tables
        CA *= self.CA_multiplier
        
        return CA

    def _get_normal_moment_coeffs(self, mach: float, alpha_deg: float) -> Tuple[float, float]:
        """Get normal force and pitching moment coefficients."""
        if self._cn_cm_interp and hasattr(self, '_cn_interp'):
            try:
                CN = float(self._cn_interp((mach, alpha_deg))) * self.CN_multiplier
                CM = float(self._cm_interp((mach, alpha_deg))) * self.CM_multiplier
                return CN, CM
            except (ValueError, IndexError) as e:
                warnings.warn(
                    f"CN/CM interpolation failed (Mach={mach:.2f}, alpha={alpha_deg:.2f}): {e}",
                    AeroInterpolationWarning,
                    stacklevel=2
                )

        raise AeroDataError(
            f"No normal force/moment coefficient table available for Mach={mach}, alpha={alpha_deg}"
        )

    def _get_side_yaw_coeffs(self, mach: float, beta_deg: float) -> Tuple[float, float]:
        """Get side force and yawing moment coefficients (uses CN/CM via axial symmetry)."""
        return self._get_normal_moment_coeffs(mach, beta_deg)

    def _get_damping_derivatives(self, mach: float, alpha_deg: float,
                                   beta_deg: float, alpha_total_deg: float) -> Tuple[float, float, float, float, float]:
        """Get damping derivatives (CMq, CNq, Clp, Clnr, Cyr) from table or defaults."""
        CMq = self.CMq_default
        CNq = self.Cnq_default  # Note: This is a FORCE derivative
        Clp = self.Clp_default
        Clnr = self.Cnq_default  # Default: use Cnq for yaw moment (legacy behavior)
        Cyr = 0.0  # Default: no side force damping
        
        if self._damping_interp:
            try:
                # CMq: from CMq table with fabs(alpha)
                CMq = float(self._cmq_interp((mach, alpha_deg)))

                # CNq: from CNq table with fabs(alpha)
                CNq = float(self._cnq_interp((mach, alpha_deg)))

                # Clp: from Clp table with fabs(alpha_total)
                Clp = float(self._clp_interp((mach, alpha_total_deg)))

                # Clnr: use dedicated table if available, otherwise approximate from CMq(β)
                if self._clnr_interp is not None:
                    Clnr = float(self._clnr_interp((mach, beta_deg)))
                else:
                    Clnr = float(self._cmq_interp((mach, beta_deg)))

                # Cyr: use dedicated table if available, otherwise approximate from CNq(β)
                if self._cyr_interp is not None:
                    Cyr = float(self._cyr_interp((mach, beta_deg)))
                else:
                    Cyr = float(self._cnq_interp((mach, beta_deg)))
                    
            except (ValueError, IndexError) as e:
                warnings.warn(
                    f"Damping interpolation failed (Mach={mach:.2f}, alpha={alpha_deg:.2f}, "
                    f"beta={beta_deg:.2f}, alpha_total={alpha_total_deg:.2f}): {e}, using defaults",
                    AeroInterpolationWarning,
                    stacklevel=2
                )

        return CMq, CNq, Clp, Clnr, Cyr

    def _get_fin_effectiveness(self, mach: float, alpha_deg: float,
                               delta_deg: float) -> Tuple[float, float]:
        """Get fin effectiveness coefficients (CN_delta, CM_delta) from available tables."""
        # Priority 1: Try 3D fin_deflection_coeffs table (Mach x alpha x delta)
        if getattr(self, '_fin_deflection_coeffs_3d', False):
            try:
                CN_delta = float(self._cn_delta_3d_interp((mach, alpha_deg, delta_deg)))
                CM_delta = float(self._cm_delta_3d_interp((mach, alpha_deg, delta_deg)))
                return CN_delta, CM_delta
            except (ValueError, IndexError) as e:
                warnings.warn(
                    f"fin_deflection_coeffs 3D interpolation failed (Mach={mach:.2f}, alpha={alpha_deg:.2f}, delta={delta_deg:.2f}): {e}",
                    AeroInterpolationWarning,
                    stacklevel=2
                )

        # Priority 2: Try 2D fin_deflection_coeffs table (Mach x delta)
        if getattr(self, '_fin_deflection_coeffs_mach_delta', False):
            try:
                CN_delta = float(self._cn_delta_mach_delta_interp((mach, delta_deg)))
                CM_delta = float(self._cm_delta_mach_delta_interp((mach, delta_deg)))
                return CN_delta, CM_delta
            except (ValueError, IndexError) as e:
                warnings.warn(
                    f"fin_deflection_coeffs (Mach x delta) interpolation failed (Mach={mach:.2f}, delta={delta_deg:.2f}): {e}",
                    AeroInterpolationWarning,
                    stacklevel=2
                )

        raise AeroDataError(
            "No fin deflection coefficient table loaded. "
            "Please provide fin_deflection_coeffs.csv with either:\n"
            "  - 3D format: Mach, alpha_deg, delta_deg, CN_delta, CM_delta\n"
            "  - 2D format: Mach, delta_deg, CN_delta, CM_delta"
        )

    def _get_fin_drag_coeff(self, mach: float, delta_deg: float) -> float:
        """Get fin deflection drag coefficient."""
        if self._fin_drag_interp and hasattr(self, '_cd_delta_interp'):
            try:
                return float(self._cd_delta_interp((mach, delta_deg)))
            except (ValueError, IndexError) as e:
                warnings.warn(
                    f"Fin drag interpolation failed (Mach={mach:.2f}, delta={delta_deg:.2f}): {e}, using 0.0",
                    AeroInterpolationWarning,
                    stacklevel=2
                )
        return 0.0

    def has_fin_effectiveness_tables(self) -> bool:
        """
        Check if fin effectiveness tables are loaded from data files.

        This method checks if any fin effectiveness data source is available:
        - 3D fin_deflection_coeffs table (Mach x alpha x delta)
        - 2D fin_deflection_coeffs table (Mach x delta)

        Returns:
            True if any fin effectiveness table is loaded, False otherwise
        """
        if getattr(self, '_fin_deflection_coeffs_3d', False):
            return True
        if getattr(self, '_fin_deflection_coeffs_mach_delta', False):
            return True
        return False

    def has_roll_aero_table(self) -> bool:
        """Check if roll aero coefficients table is loaded."""
        return self._roll_aero_interp is not None

    def get_roll_moment_coeff(self, mach: float, alpha_deg: float,
                               def_roll_deg: float) -> Optional[float]:
        """Get roll moment coefficient (Cll) from 3D table interpolation."""
        if self._roll_aero_interp is None:
            return None

        try:
            # Get the minimum deflection value from the table bounds
            def_min = self._roll_aero_def_bounds[0]

            # Handle deflection below table minimum with linear scaling to ensure Cll(0) = 0
            if def_roll_deg < def_min:
                if def_roll_deg <= 0.0:
                    # Zero deflection = zero roll moment (physically correct)
                    return 0.0
                # Linear scaling: Cll(def) = (def / def_min) * Cll(def_min)
                # This ensures Cll(0) = 0 and smooth transition to table values
                cll_at_min = float(self._roll_aero_interp((mach, alpha_deg, def_min)))
                return (def_roll_deg / def_min) * cll_at_min
            
            return float(self._roll_aero_interp((mach, alpha_deg, def_roll_deg)))
        except (ValueError, IndexError) as e:
            warnings.warn(
                f"Roll aero interpolation failed (Mach={mach:.2f}, alpha={alpha_deg:.2f}, "
                f"def_roll={def_roll_deg:.2f}): {e}",
                AeroInterpolationWarning,
                stacklevel=2
            )
            return None

    def _compute_fin_moments(self, mach: float, control_input: np.ndarray,
                            alpha: float, beta: float, q_dyn: float) -> FinMoments:
        """Compute per-fin hinge and bending moments."""
        torq2 = q_dyn * self.S_ref

        hinge_moments = np.zeros(4)
        bending_moments = np.zeros(4)
        
        # Compute total angle of attack magnitude (combines alpha and beta effects)
        # This is used to scale the alpha-dependent hinge/bending coefficients
        alpha_total_deg = np.degrees(np.sqrt(alpha**2 + beta**2))

        for i, delta in enumerate(control_input):
            delta_deg = np.degrees(np.abs(delta))

            try:
                ch_df = float(self._ch_delta_interp((mach, delta_deg)))
                cb_df = float(self._cb_delta_interp((mach, delta_deg)))
                # Ch_alpha/Cb_alpha are derivatives (per degree), multiply by alpha_total
                ch_al_coeff = float(self._ch_alpha_interp((mach, delta_deg)))
                cb_al_coeff = float(self._cb_alpha_interp((mach, delta_deg)))
                ch_al = ch_al_coeff * alpha_total_deg
                cb_al = cb_al_coeff * alpha_total_deg
            except (ValueError, IndexError, AttributeError) as e:
                warnings.warn(
                    f"Hinge/bending interpolation failed (Mach={mach:.2f}, delta={delta_deg:.2f}): {e}, using linear approximation",
                    AeroInterpolationWarning,
                    stacklevel=2
                )
                ch_df = 0.012 * delta_deg
                cb_df = 0.018 * delta_deg
                ch_al = 0.01 * alpha_total_deg
                cb_al = 0.015 * alpha_total_deg

            hinge_moments[i] = torq2 * (ch_df + ch_al)
            bending_moments[i] = torq2 * (cb_df + cb_al)

            self.max_hinge_moment[i] = max(
                np.abs(hinge_moments[i]),
                self.max_hinge_moment[i]
            )
            self.max_bending_moment[i] = max(
                np.abs(bending_moments[i]),
                self.max_bending_moment[i]
            )

        return FinMoments(
            hinge_moments=hinge_moments,
            bending_moments=bending_moments,
            max_hinge=np.max(np.abs(hinge_moments)),
            max_bending=np.max(np.abs(bending_moments))
        )

    def get_launcher_forces(self, velocity_body: np.ndarray, mass: float,
                           launch_elevation: float, q_dyn: float,
                           mach: float, reynolds: float) -> Tuple[np.ndarray, np.ndarray]:
        """Calculate forces during launcher phase."""
        g = 9.81

        CA = self._get_axial_force_coeff(mach, 0.0, reynolds, True)
        F_drag = -q_dyn * self.S_ref * CA

        F_gravity_parallel = -mass * g * np.sin(launch_elevation)
        F_friction = -self.rail_friction * mass * g * np.cos(launch_elevation)

        F_axial = F_drag + F_gravity_parallel + F_friction

        forces = np.array([F_axial, 0.0, 0.0])
        moments = np.zeros(3)

        return forces, moments

    def compute_tip_off(self, launch_elevation: float,
                       current_pitch: float) -> float:
        """Calculate tip-off angle at rail exit."""
        return launch_elevation - current_pitch

    def get_max_fin_moments(self) -> Dict[str, np.ndarray]:
        """Get maximum recorded fin moments during simulation."""
        return {
            'max_hinge_moments': self.max_hinge_moment.copy(),
            'max_bending_moments': self.max_bending_moment.copy()
        }

    def reset_max_moments(self):
        """Reset maximum moment tracking."""
        self.max_hinge_moment = np.zeros(4)
        self.max_bending_moment = np.zeros(4)

    def _quaternion_to_rotation_matrix(self, q: np.ndarray) -> np.ndarray:
        """
        Convert quaternion to rotation matrix.

        Args:
            q: Quaternion [q0, q1, q2, q3] (scalar-first convention)

        Returns:
            3x3 rotation matrix

        Raises:
            DegenerateQuaternionError: If quaternion norm is below threshold or contains NaN/Inf
        """
        q = np.asarray(q)

        # Check for NaN or Inf values
        if not np.all(np.isfinite(q)):
            raise DegenerateQuaternionError(
                f"Quaternion contains NaN or Inf values: {q}. "
                f"This indicates numerical instability in the aerodynamics model."
            )

        # Check for degenerate quaternion (near-zero norm)
        q_norm = np.linalg.norm(q)
        if q_norm < QUAT_NORM_THRESHOLD:
            raise DegenerateQuaternionError(
                f"Quaternion norm {q_norm} is below threshold {QUAT_NORM_THRESHOLD}. "
                f"Cannot convert degenerate quaternion to rotation matrix."
            )
        
        # Normalize to ensure orthogonal rotation matrix (handles integrator drift)
        q0, q1, q2, q3 = q / q_norm

        R = np.array([
            [1 - 2*(q2**2 + q3**2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
            [2*(q1*q2 + q0*q3), 1 - 2*(q1**2 + q3**2), 2*(q2*q3 - q0*q1)],
            [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1**2 + q2**2)]
        ])

        return R
