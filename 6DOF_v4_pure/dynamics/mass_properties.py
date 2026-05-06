#!/usr/bin/env python3
"""
Time-Varying Mass Properties Module

Computes mass, CG, and inertia as propellant burns.
Supports single/multi-stage rockets with separation.
"""

import numpy as np
import warnings
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any
from scipy import interpolate

from .enums import SeparationTrigger

# NumPy compatibility: trapezoid was added in NumPy 2.0, trapz was removed
if hasattr(np, 'trapezoid'):
    _trapezoid = np.trapezoid
elif hasattr(np, 'trapz'):
    _trapezoid = np.trapz
else:
    from scipy.integrate import trapezoid as _trapezoid


class ThrustCurve:
    """
    Thrust curve for non-linear propellant consumption.

    depletion_time = time_points[-1] (all propellant consumed).
    burnout_time = thrust < 10% max (for phase/separation logic).
    Propellant fraction via mdot integration if available, else impulse-based.
    Use from_loader_curve() to convert from rocket_data_loader.ThrustCurve.
    """
    
    def __init__(self, time_points: List[float], thrust_values: List[float],
                 mass_flow_values: Optional[List[float]] = None,
                 propellant_mass: Optional[float] = None):
        """
        Args:
            time_points: Time points (s), monotonically increasing
            thrust_values: Thrust values (N) at each time point
            mass_flow_values: Optional mass flow rate (kg/s) for variable Isp tracking
            propellant_mass: Total propellant mass (kg) for accurate fraction calc
        """
        if len(time_points) != len(thrust_values):
            raise ValueError("time_points and thrust_values must have same length")
        
        if len(time_points) < 2:
            raise ValueError("Need at least 2 points to define a curve")
        
        # Check monotonically increasing
        if not all(time_points[i] < time_points[i+1] for i in range(len(time_points)-1)):
            raise ValueError("time_points must be monotonically increasing")
        
        self.time_points = np.array(time_points)
        self.thrust_values = np.array(thrust_values)
        self.propellant_mass = propellant_mass
        
        # Validate non-negative thrust values
        if np.any(self.thrust_values < 0):
            negative_indices = np.where(self.thrust_values < 0)[0]
            negative_times = self.time_points[negative_indices]
            negative_values = self.thrust_values[negative_indices]
            raise ValueError(
                f"thrust_values must be non-negative. Found {len(negative_indices)} negative value(s): "
                f"at t={negative_times[:3].tolist()}{'...' if len(negative_indices) > 3 else ''} "
                f"with values={negative_values[:3].tolist()}{'...' if len(negative_indices) > 3 else ''}"
            )
        
        # Store mass flow data if provided
        # Fail fast if mass_flow_values length doesn't match time_points
        if mass_flow_values is not None:
            if len(mass_flow_values) != len(time_points):
                raise ValueError(
                    f"mass_flow_values length ({len(mass_flow_values)}) must match "
                    f"time_points length ({len(time_points)}). "
                    f"Either provide matching arrays or omit mass_flow_values entirely."
                )
            self._has_mass_flow = True
        else:
            self._has_mass_flow = False
        
        if self._has_mass_flow:
            self.mass_flow_values = np.array(mass_flow_values)
            
            # Validate non-negative mass flow values
            if np.any(self.mass_flow_values < 0):
                negative_indices = np.where(self.mass_flow_values < 0)[0]
                negative_times = self.time_points[negative_indices]
                negative_values = self.mass_flow_values[negative_indices]
                raise ValueError(
                    f"mass_flow_values must be non-negative. Found {len(negative_indices)} negative value(s): "
                    f"at t={negative_times[:3].tolist()}{'...' if len(negative_indices) > 3 else ''} "
                    f"with values={negative_values[:3].tolist()}{'...' if len(negative_indices) > 3 else ''}"
                )
            # Compute total propellant from mass flow integration
            self._total_propellant_from_mdot = _trapezoid(self.mass_flow_values, self.time_points)
            self._compute_cumulative_mass_consumed()
        else:
            self.mass_flow_values = None
            self._total_propellant_from_mdot = None
        
        # Create interpolator (linear by default, can be changed to cubic)
        # fill_value=(0.0, 0.0): Before start and after end, thrust is zero
        # This ensures no unexpected thrust before motor ignition
        self.interpolator = interpolate.interp1d(
            self.time_points, 
            self.thrust_values,
            kind='linear',
            bounds_error=False,
            fill_value=(0.0, 0.0)  # Before start: 0, after end: 0
        )
        
        # Compute total impulse (integral of thrust over time)
        self.total_impulse = _trapezoid(self.thrust_values, self.time_points)
        
        # Validate total_impulse > 0 when using impulse-based method (no mass_flow data)
        # This is critical because the impulse method is used as fallback for propellant
        # fraction calculation when mass_flow data is not available.
        if not self._has_mass_flow and self.total_impulse <= 0:
            raise ValueError(
                f"total_impulse must be positive when mass_flow data is not provided. "
                f"Got total_impulse={self.total_impulse:.6e} N·s. "
                f"This indicates the thrust curve has no meaningful thrust data. "
                f"Either provide positive thrust values or include mass_flow data for "
                f"accurate propellant tracking."
            )
        
        # Compute cumulative impulse for mass fraction calculation (fallback method)
        self._compute_cumulative_impulse()
    
    @classmethod
    def from_loader_curve(cls, loader_curve, propellant_mass: Optional[float] = None) -> 'ThrustCurve':
        """
        Create from a rocket_data_loader.ThrustCurve object (.time, .thrust, optional .mass_flow).

        Args:
            loader_curve: Object with .time, .thrust, and optional .mass_flow attributes
            propellant_mass: Optional total propellant mass (kg) for accurate fraction calc
        """
        # Extract arrays from loader curve
        time_points = list(loader_curve.time)
        thrust_values = list(loader_curve.thrust)
        
        # Extract mass flow if available
        mass_flow_values = None
        if hasattr(loader_curve, 'mass_flow') and loader_curve.mass_flow is not None:
            mass_flow_values = list(loader_curve.mass_flow)
        
        return cls(
            time_points=time_points,
            thrust_values=thrust_values,
            mass_flow_values=mass_flow_values,
            propellant_mass=propellant_mass
        )
    
    def _compute_cumulative_mass_consumed(self):
        """
        Compute cumulative mass consumed at each time point from mass flow data.
        
        This provides accurate propellant tracking for variable Isp motors.
        """
        self.cumulative_mass_consumed = np.zeros_like(self.time_points)
        for i in range(1, len(self.time_points)):
            self.cumulative_mass_consumed[i] = self.cumulative_mass_consumed[i-1] + _trapezoid(
                self.mass_flow_values[i-1:i+1],
                self.time_points[i-1:i+1]
            )
        
        # Create interpolator for cumulative mass consumed
        self.cumulative_mass_interpolator = interpolate.interp1d(
            self.time_points,
            self.cumulative_mass_consumed,
            kind='linear',
            bounds_error=False,
            fill_value=(0.0, self._total_propellant_from_mdot)
        )
    
    def _compute_cumulative_impulse(self):
        """Compute cumulative impulse at each time point."""
        self.cumulative_impulse = np.zeros_like(self.time_points)
        for i in range(1, len(self.time_points)):
            self.cumulative_impulse[i] = self.cumulative_impulse[i-1] + _trapezoid(
                self.thrust_values[i-1:i+1],
                self.time_points[i-1:i+1]
            )
        
        # Create interpolator for cumulative impulse
        self.cumulative_interpolator = interpolate.interp1d(
            self.time_points,
            self.cumulative_impulse,
            kind='linear',
            bounds_error=False,
            fill_value=(0.0, self.total_impulse)
        )
    
    def get_thrust(self, t: float) -> float:
        """Get thrust at time t."""
        return float(self.interpolator(t))
    
    def get_mass_flow(self, t: float) -> float:
        """Get mass flow rate (kg/s) at time t. Returns 0.0 if no mass_flow data or outside burn window."""
        if not self._has_mass_flow:
            return 0.0
        
        # Return 0 before first time point (motor not ignited)
        # Return 0 at or after last time point (motor burned out)
        if t < self.time_points[0] or t >= self.time_points[-1]:
            return 0.0
        
        # Create interpolator on demand if not exists
        if not hasattr(self, '_mass_flow_interpolator'):
            self._mass_flow_interpolator = interpolate.interp1d(
                self.time_points,
                self.mass_flow_values,
                kind='linear',
                bounds_error=False,
                fill_value=(0.0, 0.0)
            )
        
        return float(self._mass_flow_interpolator(t))
    
    def has_mass_flow_data(self) -> bool:
        """
        Check if mass flow data is available.
        
        Returns:
            True if mass_flow data was provided and can be used for
            accurate propellant fraction calculation.
        """
        return self._has_mass_flow
    
    def get_propellant_fraction_consumed(self, t: float) -> float:
        """Get fraction of propellant consumed (0.0-1.0) at time t. Uses mdot integration if available, else impulse-based."""
        if t <= 0:
            return 0.0
        if t >= self.time_points[-1]:
            return 1.0
        
        # Method 1: Use mass flow integration (most accurate)
        if self._has_mass_flow and self._total_propellant_from_mdot > 0:
            cumulative_mass = float(self.cumulative_mass_interpolator(t))
            # Use propellant_mass if provided, otherwise use integrated total
            total_prop = self.propellant_mass if self.propellant_mass else self._total_propellant_from_mdot
            return min(1.0, cumulative_mass / total_prop)
        
        # Method 2: Fallback to impulse-based calculation (assumes constant Isp)
        cumulative = float(self.cumulative_interpolator(t))
        return cumulative / self.total_impulse if self.total_impulse > 0 else 0.0
    
    def get_propellant_consumption_rate(self, t: float) -> float:
        """Get instantaneous propellant consumption rate (1/s). Uses mdot if available, else impulse-based."""
        if t <= 0 or t >= self.time_points[-1]:
            return 0.0
        
        # Method 1: Use mass flow data (most accurate)
        if self._has_mass_flow:
            mdot = self.get_mass_flow(t)
            total_prop = self.propellant_mass if self.propellant_mass else self._total_propellant_from_mdot
            if total_prop > 0:
                return mdot / total_prop
        
        # Method 2: Fallback to impulse-based derivative (assumes constant Isp)
        if self.total_impulse <= 0:
            return 0.0
        
        thrust = self.get_thrust(t)
        return thrust / self.total_impulse
    
    def get_depletion_time(self) -> float:
        """Get propellant depletion time (s) - last time point in thrust curve."""
        return float(self.time_points[-1])
    
    def get_burnout_time(self) -> float:
        """Get burnout time (s) - when thrust drops below 10% of max. For phase/separation logic, not mass calc."""
        if len(self.thrust_values) == 0:
            return 0.0
        
        max_thrust = np.max(self.thrust_values)
        threshold = 0.1 * max_thrust
        
        # Find last index where thrust is above threshold
        above_threshold = self.thrust_values > threshold
        if np.any(above_threshold):
            last_idx = np.where(above_threshold)[0][-1]
            return float(self.time_points[last_idx])
        else:
            return float(self.time_points[-1])
    
    def get_burn_time(self) -> float:
        """Get burn time (s) for mass calculations. Returns depletion_time (not burnout_time)."""
        return self.get_depletion_time()


@dataclass
class StageProperties:
    """
    Properties for a single stage.

    All inertia tensors MUST be about each stage's CG in the SAME body-fixed frame
    (X=nose, Y=right, Z=down). Parallel axis theorem shifts them to combined CG.

    inertia_gamma controls interpolation: 1.0=linear, <1.0=concave (outside-in),
    >1.0=convex (inside-out). effective_fraction = fraction^gamma.
    """
    stage_id: int
    name: str
    
    mass_dry: float
    mass_propellant: float
    
    Ixx_full: float
    Iyy_full: float
    Izz_full: float
    
    Ixx_empty: float
    Iyy_empty: float
    Izz_empty: float
    
    xcg_full: float
    xcg_empty: float
    
    burn_time: float
    
    has_motor: bool = True
    has_fins: bool = True
    
    # Insulation mass - ablates proportionally with propellant
    mass_insulation: float = 0.0
    
    separation_trigger: SeparationTrigger = SeparationTrigger.BURNOUT
    separation_value: float = 0.0
    
    # Off-diagonal inertia elements (products of inertia)
    Ixy_full: float = 0.0
    Ixz_full: float = 0.0
    Iyz_full: float = 0.0
    
    Ixy_empty: float = 0.0
    Ixz_empty: float = 0.0
    Iyz_empty: float = 0.0
    
    # Thrust curve for non-linear burn
    thrust_curve: Optional[ThrustCurve] = None
    
    # Inertia interpolation gamma parameter (default 1.0 = linear)
    # gamma > 1.0: convex curve (for inside-out burning)
    # gamma < 1.0: concave curve (for outside-in burning)
    inertia_gamma: float = 1.0
    
    # Cholesky factors for SPD-aware interpolation (computed in __post_init__)
    # These ensure I(t) is SPD and smooth by construction, eliminating
    # discontinuous corrections that cause İ noise.
    _chol_full: np.ndarray = field(default=None, repr=False)
    _chol_empty: np.ndarray = field(default=None, repr=False)
    
    def __post_init__(self):
        """Post-initialization to update burn_time and compute Cholesky factors."""
        if self.thrust_curve is not None:
            self.burn_time = self.thrust_curve.get_burn_time()
        
        self._compute_cholesky_factors()
    
    def _compute_cholesky_factors(self) -> None:
        """
        Compute Cholesky factors for SPD-aware inertia interpolation.

        L(s) = (1-s)*L_full + s*L_empty, I(s) = L(s) @ L(s).T
        Guarantees SPD output and smooth İ (no discontinuous eigenvalue clamping).
        """
        I_full = np.array([
            [self.Ixx_full, self.Ixy_full, self.Ixz_full],
            [self.Ixy_full, self.Iyy_full, self.Iyz_full],
            [self.Ixz_full, self.Iyz_full, self.Izz_full]
        ])
        
        I_empty = np.array([
            [self.Ixx_empty, self.Ixy_empty, self.Ixz_empty],
            [self.Ixy_empty, self.Iyy_empty, self.Iyz_empty],
            [self.Ixz_empty, self.Iyz_empty, self.Izz_empty]
        ])
        
        I_full_sym = 0.5 * (I_full + I_full.T)
        I_empty_sym = 0.5 * (I_empty + I_empty.T)
        
        try:
            self._chol_full = np.linalg.cholesky(I_full_sym)
            self._chol_empty = np.linalg.cholesky(I_empty_sym)
        except np.linalg.LinAlgError:
            self._chol_full = None
            self._chol_empty = None
    
    def get_inertia_from_cholesky(self, fraction: float) -> Optional[np.ndarray]:
        """Get SPD inertia matrix via Cholesky interpolation. Returns None if factors unavailable."""
        if self._chol_full is None or self._chol_empty is None:
            return None
        
        L_interp = (1.0 - fraction) * self._chol_full + fraction * self._chol_empty
        
        I_interp = L_interp @ L_interp.T
        
        return I_interp
    
    def get_propellant_fraction_consumed(self, t: float) -> float:
        """
        Get fraction of propellant consumed at time t.
        
        Uses thrust curve if available, otherwise linear.
        
        Args:
            t: Time since stage start (s)
            
        Returns:
            Fraction consumed (0.0 to 1.0)
        """
        if t <= 0:
            return 0.0
        
        if self.thrust_curve is not None:
            return self.thrust_curve.get_propellant_fraction_consumed(t)
        else:
            # Linear interpolation
            if self.burn_time <= 0:
                return 0.0
            return min(1.0, t / self.burn_time)
    
    def get_propellant_consumption_rate(self, t: float) -> float:
        """
        Get instantaneous propellant consumption rate at time t.
        
        Uses thrust curve if available for analytical derivative,
        otherwise returns constant linear rate.
        
        Args:
            t: Time since stage start (s)
            
        Returns:
            Consumption rate (fraction/s)
        """
        if t <= 0:
            return 0.0
        
        if self.thrust_curve is not None:
            # Analytical derivative from thrust curve
            return self.thrust_curve.get_propellant_consumption_rate(t)
        else:
            # Linear burn: constant consumption rate
            if self.burn_time <= 0:
                return 0.0
            if t >= self.burn_time:
                return 0.0
            return 1.0 / self.burn_time
    
    def get_burnout_time(self) -> float:
        """
        Get burnout time (when thrust drops below 10% of max).
        
        This is used for separation triggers (SeparationTrigger.BURNOUT) and
        flight phase logic. For mass/CG/inertia calculations, use burn_time
        (which is the full propellant depletion time).
                
        Returns:
            Burnout time in seconds
        """
        if self.thrust_curve is not None:
            return self.thrust_curve.get_burnout_time()
        else:
            # Without thrust curve, burnout_time equals burn_time
            return self.burn_time
    
    def validate(self) -> None:
        """Validate stage properties. Raises ValueError on failure, warns on marginal issues."""
        # Check positive mass
        if self.mass_dry <= 0:
            raise ValueError(f"Stage '{self.name}': mass_dry must be positive, got {self.mass_dry}")
        if self.mass_propellant < 0:
            raise ValueError(f"Stage '{self.name}': mass_propellant cannot be negative, got {self.mass_propellant}")
        
        # Check motor/propellant consistency
        # Cannot have propellant without a motor - this is physically impossible
        if not self.has_motor and self.mass_propellant > 0:
            raise ValueError(
                f"Stage '{self.name}': has_motor=False but mass_propellant={self.mass_propellant} > 0. "
                f"Cannot have propellant without a motor. Either set has_motor=True or mass_propellant=0."
            )
        
        # Check positive burn time if motor exists
        if self.has_motor and self.mass_propellant > 0 and self.burn_time <= 0:
            raise ValueError(f"Stage '{self.name}': burn_time must be positive when motor has propellant, got {self.burn_time}")
        
        # Check positive inertia (diagonal elements)
        if self.Ixx_full <= 0 or self.Iyy_full <= 0 or self.Izz_full <= 0:
            raise ValueError(f"Stage '{self.name}': All full inertia diagonal elements must be positive")
        if self.Ixx_empty <= 0 or self.Iyy_empty <= 0 or self.Izz_empty <= 0:
            raise ValueError(f"Stage '{self.name}': All empty inertia diagonal elements must be positive")
        
        # Convert I_empty <= I_full check from error to warning
        # This constraint may not always hold when CG shifts significantly during burn,
        # as inertia about a new CG can increase in some cases.
        inertia_increase_threshold = 0.1  # 10% threshold for warning
        if self.Ixx_empty > self.Ixx_full * (1 + inertia_increase_threshold):
            warnings.warn(
                f"Stage '{self.name}': Ixx_empty ({self.Ixx_empty:.4f}) exceeds Ixx_full ({self.Ixx_full:.4f}) "
                f"by more than {inertia_increase_threshold*100:.0f}%. This is unusual but may be valid if CG shifts significantly. "
                f"Please verify your inertia data.",
                RuntimeWarning
            )
        if self.Iyy_empty > self.Iyy_full * (1 + inertia_increase_threshold):
            warnings.warn(
                f"Stage '{self.name}': Iyy_empty ({self.Iyy_empty:.4f}) exceeds Iyy_full ({self.Iyy_full:.4f}) "
                f"by more than {inertia_increase_threshold*100:.0f}%. This is unusual but may be valid if CG shifts significantly. "
                f"Please verify your inertia data.",
                RuntimeWarning
            )
        if self.Izz_empty > self.Izz_full * (1 + inertia_increase_threshold):
            warnings.warn(
                f"Stage '{self.name}': Izz_empty ({self.Izz_empty:.4f}) exceeds Izz_full ({self.Izz_full:.4f}) "
                f"by more than {inertia_increase_threshold*100:.0f}%. This is unusual but may be valid if CG shifts significantly. "
                f"Please verify your inertia data.",
                RuntimeWarning
            )
        
        # Check triangle inequality on principal moments (eigenvalues)
        # Triangle inequality is only valid for principal moments, not diagonal elements
        # when off-diagonal elements (products of inertia) are non-zero.
        self._check_inertia_triangle_inequality_principal('full')
        self._check_inertia_triangle_inequality_principal('empty')
        
        # Validate full and empty inertia tensors are symmetric positive-definite (SPD)
        # This ensures interpolated inertia will also be SPD (convex combination of SPD matrices)
        self._validate_inertia_spd('full')
        self._validate_inertia_spd('empty')
        
        # Check CG location is non-negative (from nose)
        if self.xcg_full < 0:
            raise ValueError(f"Stage '{self.name}': xcg_full must be non-negative, got {self.xcg_full}")
        if self.xcg_empty < 0:
            raise ValueError(f"Stage '{self.name}': xcg_empty must be non-negative, got {self.xcg_empty}")
        
        # Check inertia_gamma is positive (required for valid interpolation)
        # gamma <= 0 can cause fraction^gamma to blow up or exceed valid range [0,1]
        if self.inertia_gamma <= 0:
            raise ValueError(
                f"Stage '{self.name}': inertia_gamma must be positive, got {self.inertia_gamma}. "
                f"Use gamma=1.0 for linear interpolation, gamma>1.0 for convex curve (inside-out burning), "
                f"gamma<1.0 for concave curve (outside-in burning)."
            )
        
        # Check propellant mass consistency with thrust curve if both are provided
        if self.thrust_curve is not None and self.thrust_curve.propellant_mass is not None:
            if self.mass_propellant > 0:
                relative_diff = abs(self.mass_propellant - self.thrust_curve.propellant_mass) / self.mass_propellant
                if relative_diff > 0.01:  # Allow 1% tolerance for rounding
                    warnings.warn(
                        f"Stage '{self.name}': mass_propellant ({self.mass_propellant:.2f} kg) differs from "
                        f"ThrustCurve.propellant_mass ({self.thrust_curve.propellant_mass:.2f} kg) by {relative_diff*100:.1f}%. "
                        f"This may cause inconsistent mass evolution during burn. "
                        f"Consider aligning these values for accurate simulation.",
                        RuntimeWarning
                    )
    
    def _check_inertia_triangle_inequality_principal(self, state: str) -> None:
        """Check triangle inequality on principal moments. Raises ValueError if significant, warns if marginal."""
        if state == 'full':
            I = np.array([
                [self.Ixx_full, self.Ixy_full, self.Ixz_full],
                [self.Ixy_full, self.Iyy_full, self.Iyz_full],
                [self.Ixz_full, self.Iyz_full, self.Izz_full]
            ])
        else:
            I = np.array([
                [self.Ixx_empty, self.Ixy_empty, self.Ixz_empty],
                [self.Ixy_empty, self.Iyy_empty, self.Iyz_empty],
                [self.Ixz_empty, self.Iyz_empty, self.Izz_empty]
            ])
        
        I_sym = 0.5 * (I + I.T)
        principal_moments = np.linalg.eigvalsh(I_sym)
        principal_moments = np.sort(principal_moments)
        I1, I2, I3 = principal_moments[0], principal_moments[1], principal_moments[2]
        
        scale = np.max(principal_moments)
        rel_tol = 1e-6
        abs_tol = rel_tol * scale
        
        violations = []
        if I1 + I2 < I3 - abs_tol:
            violations.append(f"I1 + I2 ({I1 + I2:.6e}) < I3 ({I3:.6e})")
        if I1 + I3 < I2 - abs_tol:
            violations.append(f"I1 + I3 ({I1 + I3:.6e}) < I2 ({I2:.6e})")
        if I2 + I3 < I1 - abs_tol:
            violations.append(f"I2 + I3 ({I2 + I3:.6e}) < I1 ({I1:.6e})")
        
        if violations:
            significant_tol = 0.01 * scale
            significant_violations = []
            if I1 + I2 < I3 - significant_tol:
                significant_violations.append(f"I1 + I2 < I3")
            if I1 + I3 < I2 - significant_tol:
                significant_violations.append(f"I1 + I3 < I2")
            if I2 + I3 < I1 - significant_tol:
                significant_violations.append(f"I2 + I3 < I1")
            
            if significant_violations:
                raise ValueError(
                    f"Stage '{self.name}' ({state}): Inertia triangle inequality significantly violated "
                    f"on principal moments. Principal moments: I1={I1:.6e}, I2={I2:.6e}, I3={I3:.6e}. "
                    f"Violations: {', '.join(significant_violations)}. "
                    f"This indicates non-physical inertia data. Please check your input values."
                )
            else:
                warnings.warn(
                    f"Stage '{self.name}' ({state}): Inertia triangle inequality marginally violated "
                    f"on principal moments (within numerical tolerance). "
                    f"Principal moments: I1={I1:.6e}, I2={I2:.6e}, I3={I3:.6e}. "
                    f"Violations: {', '.join(violations)}. This may be numerical noise.",
                    RuntimeWarning
                )
    
    def _validate_inertia_spd(self, state: str) -> None:
        """Validate that the inertia tensor ('full' or 'empty') is SPD. Raises ValueError if not."""
        if state == 'full':
            I = np.array([
                [self.Ixx_full, self.Ixy_full, self.Ixz_full],
                [self.Ixy_full, self.Iyy_full, self.Iyz_full],
                [self.Ixz_full, self.Iyz_full, self.Izz_full]
            ])
        else:
            I = np.array([
                [self.Ixx_empty, self.Ixy_empty, self.Ixz_empty],
                [self.Ixy_empty, self.Iyy_empty, self.Iyz_empty],
                [self.Ixz_empty, self.Iyz_empty, self.Izz_empty]
            ])
        
        I_sym = 0.5 * (I + I.T)
        
        try:
            np.linalg.cholesky(I_sym)
        except np.linalg.LinAlgError:
            eigenvalues = np.linalg.eigvalsh(I_sym)
            min_eigenvalue = np.min(eigenvalues)
            raise ValueError(
                f"Stage '{self.name}' ({state}): Inertia tensor is not positive-definite. "
                f"Eigenvalues: {eigenvalues}. Min eigenvalue: {min_eigenvalue:.6e}. "
                f"Diagonal: [{I[0,0]:.4f}, {I[1,1]:.4f}, {I[2,2]:.4f}]. "
                f"Check that off-diagonal elements (products of inertia) are physically valid."
            )


class MultiStageMassPropertiesModel:
    """
    Multi-stage mass properties model handling mass, CG, and inertia over time.
    Supports separation events and non-linear propellant consumption.
    Body frame: +X nose, +Y right, +Z down.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize multi-stage mass properties model.
        
        Args:
            config: Configuration dictionary with 'stages' list
        """
        self.stages: List[StageProperties] = []
        self.current_stage_idx = 0
        self.stage_start_times: List[float] = [0.0]
        self.separation_times: List[float] = []
        self.total_separation_mass: float = 0.0
        
        # Performance optimization: cache for frequently called methods
        self._cache_enabled: bool = config.get('enable_cache', True)
        self._mass_cache: Dict[int, float] = {}  # Key is quantized time
        self._cg_cache: Dict[int, float] = {}  # Key is quantized time
        self._inertia_cache: Dict[int, np.ndarray] = {}  # Key is quantized time
        self._max_cache_size: int = config.get('max_cache_size', 128)
        # Time quantization resolution for cache keys (seconds)
        # Using integer keys avoids floating-point equality issues
        # Default 1e-9s supports adaptive integrators with very small time steps
        # Set to 0 or negative to disable caching entirely
        self._cache_time_resolution: float = config.get('cache_time_resolution', 1e-9)
        
        # Tolerance for SPD validation of interpolated inertia matrices
        # Eigenvalues more negative than -tol*scale are considered invalid data
        # Eigenvalues in [-tol*scale, 0) are considered numerical noise and clamped
        # Default 1e-10 is conservative: catches real errors while allowing numerical noise
        self._inertia_spd_tol: float = config.get('inertia_spd_tol', 1e-10)
        
        # Diagnostics for SPD correction frequency (helps verify Cholesky interpolation effectiveness)
        self._spd_correction_count: int = 0
        self._spd_correction_total_magnitude: float = 0.0
        
        self._parse_config(config)
        
        self._compute_initial_properties()
    
    def _cache_key(self, t: float) -> int:
        """
        Generate a quantized integer cache key from a time value to avoid float equality issues.
        
        Args:
            t: Time value (s)
            
        Returns:
            Integer cache key (or unique raw float bits if caching is disabled).
        """
        if self._cache_time_resolution <= 0:
            # Disable caching by using unique keys (raw float bits)
            import struct
            return struct.unpack('q', struct.pack('d', t))[0]
        return int(round(t / self._cache_time_resolution))
    
    @staticmethod
    def parallel_axis_theorem(I_cm: np.ndarray, mass: float, offset: np.ndarray) -> np.ndarray:
        """
        Apply parallel axis theorem to transform inertia tensor based on offset vector.
        
        Args:
            I_cm: 3x3 inertia tensor about center of mass (kg·m²)
            mass: Mass (kg)
            offset: 3D offset vector [dx, dy, dz] (m)
            
        Returns:
            3x3 inertia tensor about new reference point (kg·m²)
        """
        # For rockets, we typically only care about x-axis offset (along rocket axis)
        # offset = [dx, 0, 0] where dx is the CG location
        dx, dy, dz = offset[0], offset[1], offset[2]
        
        # Parallel axis theorem components
        d_squared = dx*dx + dy*dy + dz*dz
        
        # Steiner term: m * (d² * I - d ⊗ d)
        I_identity = np.eye(3)
        d_outer = np.outer(offset, offset)
        steiner = mass * (d_squared * I_identity - d_outer)
        
        return I_cm + steiner
    
    def _validate_positive_definite(self, I: np.ndarray, stage_id: int = None, 
                                     fraction: float = None) -> np.ndarray:
        """
        Validate and smoothly correct inertia matrix to ensure positive-definiteness.
        
        Uses a scale-aware tolerance to distinguish between numerical noise (which is smoothly clamped) 
        and true invalid data (which raises an error).
        
        Args:
            I: 3x3 inertia tensor to validate
            stage_id: Stage identifier for context in error messages (optional)
            fraction: Propellant fraction for context in error messages (optional)
            
        Returns:
            The corrected positive-definite matrix, or the original if valid.
            
        Raises:
            ValueError: If the matrix has significant negative eigenvalues.
        """
        I_sym = 0.5 * (I + I.T)
        
        try:
            np.linalg.cholesky(I_sym)
            return I_sym
        except np.linalg.LinAlgError:
            eigenvalues, eigenvectors = np.linalg.eigh(I_sym)
            min_eigenvalue = np.min(eigenvalues)
            
            # Scale-aware tolerance for distinguishing numerical noise from real errors
            scale = np.max(np.diag(I_sym))
            tolerance = self._inertia_spd_tol * scale
            
            # Build context string for error/warning messages
            context = ""
            if stage_id is not None:
                context += f" stage={stage_id}"
            if fraction is not None:
                context += f" fraction={fraction:.4f}"
            
            if min_eigenvalue < -tolerance:
                # Significant violation - raise error to avoid hiding bad data
                raise ValueError(
                    f"Interpolated inertia matrix is not positive-definite.{context} "
                    f"Min eigenvalue: {min_eigenvalue:.6e} (tolerance: {-tolerance:.6e}). "
                    f"Diagonal: [{I[0,0]:.4f}, {I[1,1]:.4f}, {I[2,2]:.4f}]. "
                    f"Off-diagonal: Ixy={I[0,1]:.6e}, Ixz={I[0,2]:.6e}, Iyz={I[1,2]:.6e}. "
                    f"This indicates invalid inertia configuration - check input data for "
                    f"incorrect off-diagonal elements, wrong units, or inconsistent full/empty values."
                )
            
            # Numerical noise - use smooth clamp to avoid discontinuity in İ
            epsilon = 1e-9 * scale
            delta = 0.1 * tolerance  # Smoothing width (عرض التنعيم)
            
            # Smooth max: smoothmax(λ, ε) = 0.5 * (λ + ε + sqrt((λ - ε)² + δ²))
            # This is C∞ smooth and approaches max(λ, ε) as δ → 0
            eigenvalues_clamped = 0.5 * (eigenvalues + epsilon + 
                                         np.sqrt((eigenvalues - epsilon)**2 + delta**2))
            
            I_corrected = eigenvectors @ np.diag(eigenvalues_clamped) @ eigenvectors.T
            I_corrected = 0.5 * (I_corrected + I_corrected.T)
            
            # Update diagnostics
            self._spd_correction_count += 1
            correction_magnitude = np.linalg.norm(I_corrected - I_sym)
            self._spd_correction_total_magnitude += correction_magnitude
            
            return I_corrected
    
    def _compute_combined_cg(self, t: float) -> float:
        """
        Compute combined center of gravity for all remaining stages.
        
        Args:
            t: Current time (s)
            
        Returns:
            Combined CG location from reference point (m)
        """
        total_mass = 0.0
        total_moment = 0.0
        
        for i, stage in enumerate(self.stages):
            if i < self.current_stage_idx:
                continue  # Skip separated stages
            
            if i == self.current_stage_idx:
                # Current stage - account for propellant consumption
                local_t = self.get_stage_local_time(t)
                stage_mass = stage.mass_dry
                stage_cg = stage.xcg_empty
                
                if stage.has_motor and local_t < stage.burn_time and stage.burn_time > 0:
                    fraction = stage.get_propellant_fraction_consumed(local_t)
                    propellant_remaining = stage.mass_propellant * (1.0 - fraction)
                    stage_mass += propellant_remaining
                    stage_cg = stage.xcg_full + fraction * (stage.xcg_empty - stage.xcg_full)
            else:
                # Future stages - full mass and CG
                stage_mass = stage.mass_dry + stage.mass_propellant
                stage_cg = stage.xcg_full
            
            total_mass += stage_mass
            total_moment += stage_mass * stage_cg
        
        if total_mass > 0:
            return total_moment / total_mass
        return 0.0
    
    def _parse_config(self, config: Dict[str, Any]):
        """Parse configuration and create stage properties."""
        stages_config = config.get('stages', [])
        
        if not stages_config:
            self._create_single_stage_from_legacy(config)
            return
        
        for i, stage_cfg in enumerate(stages_config):
            stage = StageProperties(
                stage_id=stage_cfg.get('id', i + 1),
                name=stage_cfg.get('name', f'Stage {i + 1}'),
                mass_dry=stage_cfg.get('mass_dry', 100.0),
                mass_propellant=stage_cfg.get('mass_propellant', 0.0),
                mass_insulation=stage_cfg.get('mass_insulation', 0.0),
                Ixx_full=stage_cfg.get('Ixx_full', stage_cfg.get('Ixx_0', 1.0)),
                Iyy_full=stage_cfg.get('Iyy_full', stage_cfg.get('Iyy_0', 10.0)),
                Izz_full=stage_cfg.get('Izz_full', stage_cfg.get('Izz_0', 10.0)),
                Ixx_empty=stage_cfg.get('Ixx_empty', stage_cfg.get('Ixx_end', 1.0)),
                Iyy_empty=stage_cfg.get('Iyy_empty', stage_cfg.get('Iyy_end', 10.0)),
                Izz_empty=stage_cfg.get('Izz_empty', stage_cfg.get('Izz_end', 10.0)),
                xcg_full=stage_cfg.get('xcg_full', stage_cfg.get('xcg_0', 1.0)),
                xcg_empty=stage_cfg.get('xcg_empty', stage_cfg.get('xcg_end', 1.0)),
                burn_time=stage_cfg.get('burn_time', 0.0),
                has_motor=stage_cfg.get('has_motor', True),
                has_fins=stage_cfg.get('has_fins', True),
                separation_trigger=SeparationTrigger.from_config(
                    stage_cfg.get('separation_trigger', SeparationTrigger.BURNOUT)
                ),
                separation_value=stage_cfg.get('separation_value', 0.0),
                Ixy_full=stage_cfg.get('Ixy_full', stage_cfg.get('Ixy', 0.0)),
                Ixz_full=stage_cfg.get('Ixz_full', stage_cfg.get('Ixz', 0.0)),
                Iyz_full=stage_cfg.get('Iyz_full', stage_cfg.get('Iyz', 0.0)),
                Ixy_empty=stage_cfg.get('Ixy_empty', stage_cfg.get('Ixy', 0.0)),
                Ixz_empty=stage_cfg.get('Ixz_empty', stage_cfg.get('Ixz', 0.0)),
                Iyz_empty=stage_cfg.get('Iyz_empty', stage_cfg.get('Iyz', 0.0)),
                inertia_gamma=stage_cfg.get('inertia_gamma', 1.0),
            )
            
            # Add thrust curve if provided
            if 'thrust_curve' in stage_cfg:
                curve_data = stage_cfg['thrust_curve']
                # Handle both ThrustCurve object and dict formats
                if isinstance(curve_data, ThrustCurve):
                    # Already a ThrustCurve object, use directly
                    stage.thrust_curve = curve_data
                    # Update burn_time from thrust curve since __post_init__ was bypassed
                    stage.burn_time = stage.thrust_curve.get_burn_time()
                elif isinstance(curve_data, dict) and 'time' in curve_data and 'thrust' in curve_data:
                    # Check for mass_flow data for accurate propellant tracking
                    mass_flow_values = curve_data.get('mass_flow', None)
                    propellant_mass = stage_cfg.get('mass_propellant', None)
                    
                    stage.thrust_curve = ThrustCurve(
                        curve_data['time'],
                        curve_data['thrust'],
                        mass_flow_values=mass_flow_values,
                        propellant_mass=propellant_mass
                    )
                    # Update burn_time from thrust curve since __post_init__ was bypassed
                    # (thrust_curve was assigned after StageProperties construction)
                    stage.burn_time = stage.thrust_curve.get_burn_time()
            
            self.stages.append(stage)
            
            # Validate stage properties
            try:
                stage.validate()
            except ValueError as e:
                raise ValueError(f"Invalid configuration for stage {i + 1}: {e}")
    
    def _create_single_stage_from_legacy(self, config: Dict[str, Any]):
        """Create single stage from legacy config format."""
        stage = StageProperties(
            stage_id=1,
            name='Stage 1',
            mass_dry=config.get('mass_dry', 375.0),
            mass_propellant=config.get('mass_propellant', 238.0),
            Ixx_full=config.get('Ixx_0', 7.77),
            Iyy_full=config.get('Iyy_0', 7025.0),
            Izz_full=config.get('Izz_0', 7025.0),
            Ixx_empty=config.get('Ixx_end', 5.318),
            Iyy_empty=config.get('Iyy_end', 3735.0),
            Izz_empty=config.get('Izz_end', 3735.0),
            xcg_full=config.get('xcg_0', 3.005),
            xcg_empty=config.get('xcg_end', 2.65),
            burn_time=config.get('burn_time', 6.34),
            has_motor=True,
            has_fins=True,
            Ixy_full=config.get('Ixy_full', config.get('Ixy', 0.0)),
            Ixz_full=config.get('Ixz_full', config.get('Ixz', 0.0)),
            Iyz_full=config.get('Iyz_full', config.get('Iyz', 0.0)),
            Ixy_empty=config.get('Ixy_empty', config.get('Ixy', 0.0)),
            Ixz_empty=config.get('Ixz_empty', config.get('Ixz', 0.0)),
            Iyz_empty=config.get('Iyz_empty', config.get('Iyz', 0.0)),
        )
        self.stages.append(stage)
        
        # Validate stage properties
        try:
            stage.validate()
        except ValueError as e:
            raise ValueError(f"Invalid legacy configuration: {e}")
    
    def _compute_initial_properties(self):
        """Compute initial combined properties."""
        self.total_mass_initial = sum(
            s.mass_dry + s.mass_propellant for s in self.stages
        )
    
    def get_current_stage(self) -> StageProperties:
        """Get current active stage."""
        return self.stages[self.current_stage_idx]
    
    def get_stage_local_time(self, t: float) -> float:
        """Get time relative to current stage start."""
        return t - self.stage_start_times[self.current_stage_idx]
    
    def check_separation(self, t: float, altitude: float = 0.0, 
                        velocity: float = 0.0) -> bool:
        """
        Check if separation should occur based on the configured trigger (BURNOUT, TIME, ALTITUDE, or VELOCITY).
        Returns False for MANUAL trigger, which must be executed explicitly via `execute_separation()`.
        
        Args:
            t: Current time (s)
            altitude: Current altitude (m)
            velocity: Current velocity magnitude (m/s)
            
        Returns:
            True if separation condition is met, False otherwise.
        """
        if self.current_stage_idx >= len(self.stages) - 1:
            return False
        
        stage = self.get_current_stage()
        local_t = self.get_stage_local_time(t)
        
        if stage.separation_trigger == SeparationTrigger.BURNOUT:
            # Use burnout_time (thrust < 10% of max) instead of burn_time (propellant depletion)
            burnout_time = stage.get_burnout_time()
            if stage.has_motor and local_t >= burnout_time:
                return True
        elif stage.separation_trigger == SeparationTrigger.TIME:
            # Use stage-local time for consistency with BURNOUT and flight_phase.py
            # separation_value represents "time since stage start", not global time
            if local_t >= stage.separation_value:
                return True
        elif stage.separation_trigger == SeparationTrigger.ALTITUDE:
            if altitude >= stage.separation_value:
                return True
        elif stage.separation_trigger == SeparationTrigger.VELOCITY:
            if velocity >= stage.separation_value:
                return True
        # Note: SeparationTrigger.MANUAL is intentionally not handled here.
        # Manual separation requires direct call to execute_separation().
        
        return False
    
    def execute_separation(self, t: float) -> bool:
        """
        Execute stage separation.
        
        Args:
            t: Current time (s)
            
        Returns:
            True if separation was executed
        """
        if self.current_stage_idx >= len(self.stages) - 1:
            return False
        
        separated_stage = self.stages[self.current_stage_idx]
        
        # Calculate remaining mass of the stage being dropped
        stage_mass = separated_stage.mass_dry
        
        # Check if there is remaining propellant and insulation
        local_t = self.get_stage_local_time(t)
        if separated_stage.has_motor and local_t < separated_stage.burn_time and separated_stage.burn_time > 0:
            fraction = separated_stage.get_propellant_fraction_consumed(local_t)
            propellant_remaining = separated_stage.mass_propellant * (1.0 - fraction)
            insulation_remaining = separated_stage.mass_insulation * (1.0 - fraction)
            stage_mass += propellant_remaining + insulation_remaining
        elif separated_stage.has_motor and local_t < 0:
            # Separation before burn start (unlikely but possible)
            stage_mass += separated_stage.mass_propellant + separated_stage.mass_insulation
            
        self.total_separation_mass += stage_mass
        
        self.separation_times.append(t)
        
        self.current_stage_idx += 1
        self.stage_start_times.append(t)
        
        # Clear cache after separation as properties change
        self._clear_cache()
        
        return True
    
    def get_mass(self, t: float) -> float:
        """
        Get current total mass at time t.
        
        Args:
            t: Time (s)
            
        Returns:
            Current mass (kg)
        """
        # Handle negative time
        if t < 0:
            t = 0.0
        
        # Check cache first (use quantized key to handle float equality issues)
        cache_key = self._cache_key(t)
        if self._cache_enabled and cache_key in self._mass_cache:
            return self._mass_cache[cache_key]
        
        total_mass = 0.0
        
        for i, stage in enumerate(self.stages):
            if i < self.current_stage_idx:
                continue
            
            if i == self.current_stage_idx:
                local_t = self.get_stage_local_time(t)
                if stage.has_motor and local_t < stage.burn_time and stage.burn_time > 0:
                    # Use thrust curve if available for more accurate mass calculation
                    fraction_consumed = stage.get_propellant_fraction_consumed(local_t)
                    propellant_remaining = stage.mass_propellant * (1.0 - fraction_consumed)
                    # Insulation ablates proportionally with propellant
                    insulation_remaining = stage.mass_insulation * (1.0 - fraction_consumed)
                    total_mass += stage.mass_dry + propellant_remaining + insulation_remaining
                else:
                    total_mass += stage.mass_dry
            else:
                total_mass += stage.mass_dry + stage.mass_propellant + stage.mass_insulation
        
        # Store in cache (with size limit)
        if self._cache_enabled:
            if len(self._mass_cache) >= self._max_cache_size:
                # Remove oldest entry (simple FIFO)
                self._mass_cache.pop(next(iter(self._mass_cache)))
            self._mass_cache[cache_key] = total_mass
        
        return total_mass
    
    def get_mass_flow_rate(self, t: float) -> float:
        """
        Get mass flow rate at time t.
        
        Args:
            t: Time (s)
            
        Returns:
            Mass flow rate (kg/s)
        """
        # Handle negative time
        if t < 0:
            return 0.0
        
        stage = self.get_current_stage()
        local_t = self.get_stage_local_time(t)
        
        if stage.has_motor and local_t < stage.burn_time and stage.burn_time > 0:
            # Use analytical derivative for accurate mass flow calculation
            # Include insulation consumption rate (ablates proportionally with propellant)
            consumption_rate = stage.get_propellant_consumption_rate(local_t)
            total_consumable_mass = stage.mass_propellant + stage.mass_insulation
            return total_consumable_mass * consumption_rate
        return 0.0
    
    def get_cg_location(self, t: float) -> float:
        """
        Get combined center of gravity location at time t.
        
        For multi-stage rockets, this computes the combined CG
        of all remaining stages.
        
        Args:
            t: Time (s)
            
        Returns:
            Combined CG location from reference point (m)
        """
        # Handle negative time
        if t < 0:
            t = 0.0
        
        # Check cache first (use quantized key to handle float equality issues)
        cache_key = self._cache_key(t)
        if self._cache_enabled and cache_key in self._cg_cache:
            return self._cg_cache[cache_key]
        
        cg = self._compute_combined_cg(t)
        
        # Store in cache (with size limit)
        if self._cache_enabled:
            if len(self._cg_cache) >= self._max_cache_size:
                self._cg_cache.pop(next(iter(self._cg_cache)))
            self._cg_cache[cache_key] = cg
        
        return cg
    
    def get_inertia_matrix(self, t: float, use_cache: bool = True) -> np.ndarray:
        """
        Get 3x3 inertia matrix at time t about combined CG via parallel axis theorem.
     
        Args:
            t: Time (s)
            use_cache: Set to False to bypass cache (useful for finite difference derivatives).
            
        Returns:
            3x3 inertia matrix (kg·m²)
        """
        # Handle negative time
        if t < 0:
            t = 0.0
        
        # Check cache first (use quantized key to handle float equality issues)
        # Allow bypassing cache when use_cache=False
        cache_key = self._cache_key(t)
        if use_cache and self._cache_enabled and cache_key in self._inertia_cache:
            return self._inertia_cache[cache_key].copy()  # Return copy to prevent modification
        
        # Compute combined center of gravity
        combined_cg = self._compute_combined_cg(t)
        
        # Initialize combined inertia tensor
        I_combined = np.zeros((3, 3))
        
        for i, stage in enumerate(self.stages):
            if i < self.current_stage_idx:
                continue  # Skip separated stages
            
            # Get stage properties
            if i == self.current_stage_idx:
                local_t = self.get_stage_local_time(t)
                stage_mass = stage.mass_dry
                stage_cg = stage.xcg_empty
                
                # Compute stage inertia about its own CG
                if stage.has_motor and local_t < stage.burn_time and stage.burn_time > 0:
                    fraction = stage.get_propellant_fraction_consumed(local_t)
                    propellant_remaining = stage.mass_propellant * (1.0 - fraction)
                    stage_mass += propellant_remaining
                    stage_cg = stage.xcg_full + fraction * (stage.xcg_empty - stage.xcg_full)
                    
                    # Apply nonlinear interpolation using inertia_gamma
                    # effective_fraction = fraction^gamma
                    # gamma = 1.0: linear interpolation (default)
                    # gamma > 1.0: convex curve (for inside-out burning, e.g., hollow cylinder)
                    # gamma < 1.0: concave curve (for outside-in burning)
                    gamma = stage.inertia_gamma
                    if gamma != 1.0 and fraction > 0:
                        effective_fraction = fraction ** gamma
                    else:
                        effective_fraction = fraction
                    
                    # Try Cholesky-factor interpolation first (guarantees SPD and smooth İ)
                    I_chol = stage.get_inertia_from_cholesky(effective_fraction)
                    if I_chol is not None:
                        Ixx = I_chol[0, 0]
                        Iyy = I_chol[1, 1]
                        Izz = I_chol[2, 2]
                        Ixy = I_chol[0, 1]
                        Ixz = I_chol[0, 2]
                        Iyz = I_chol[1, 2]
                    else:
                        # Fallback to linear interpolation if Cholesky not available
                        Ixx = stage.Ixx_full + effective_fraction * (stage.Ixx_empty - stage.Ixx_full)
                        Iyy = stage.Iyy_full + effective_fraction * (stage.Iyy_empty - stage.Iyy_full)
                        Izz = stage.Izz_full + effective_fraction * (stage.Izz_empty - stage.Izz_full)
                        Ixy = stage.Ixy_full + effective_fraction * (stage.Ixy_empty - stage.Ixy_full)
                        Ixz = stage.Ixz_full + effective_fraction * (stage.Ixz_empty - stage.Ixz_full)
                        Iyz = stage.Iyz_full + effective_fraction * (stage.Iyz_empty - stage.Iyz_full)
                else:
                    Ixx = stage.Ixx_empty
                    Iyy = stage.Iyy_empty
                    Izz = stage.Izz_empty
                    Ixy = stage.Ixy_empty
                    Ixz = stage.Ixz_empty
                    Iyz = stage.Iyz_empty
            else:
                # Future stages - full mass and inertia
                stage_mass = stage.mass_dry + stage.mass_propellant
                stage_cg = stage.xcg_full
                Ixx = stage.Ixx_full
                Iyy = stage.Iyy_full
                Izz = stage.Izz_full
                Ixy = stage.Ixy_full
                Ixz = stage.Ixz_full
                Iyz = stage.Iyz_full
            
            # Stage inertia tensor about its own CG
            I_stage_cm = np.array([
                [Ixx, Ixy, Ixz],
                [Ixy, Iyy, Iyz],
                [Ixz, Iyz, Izz]
            ])
            
            # Validate and correct positive-definiteness for interpolated inertia
            # (only when interpolation occurred, i.e., during burn)
            # Also check burn_time > 0 to ensure fraction was computed
            if i == self.current_stage_idx and stage.has_motor and local_t < stage.burn_time and stage.burn_time > 0:
                I_stage_cm = self._validate_positive_definite(I_stage_cm, stage.stage_id, fraction)
            
            # Offset from stage CG to combined CG
            # For axially symmetric rockets, offset is only along x-axis
            offset = np.array([stage_cg - combined_cg, 0.0, 0.0])
            
            # Apply parallel axis theorem to transform to combined CG
            I_stage_combined = self.parallel_axis_theorem(I_stage_cm, stage_mass, offset)
            
            # Add to combined inertia
            I_combined += I_stage_combined
        
        # Store in cache (with size limit)
        # Only store in cache if use_cache=True
        if use_cache and self._cache_enabled:
            if len(self._inertia_cache) >= self._max_cache_size:
                self._inertia_cache.pop(next(iter(self._inertia_cache)))
            self._inertia_cache[cache_key] = I_combined.copy()
        
        return I_combined
    
    def get_inertia_matrix_from_mass(self, current_mass: float) -> np.ndarray:
        """
        Get inertia matrix based on current mass rather than elapsed time.
        
        Args:
            current_mass: Current total mass (kg).
            
        Returns:
            3x3 inertia matrix about combined CG (kg·m²).
        """
        # Get current stage properties
        stage = self.get_current_stage()
        
        # Compute propellant fraction from mass
        # fraction = (initial_mass - current_mass) / propellant_mass
        # This gives us how much propellant has been consumed (0 to 1)
        
        # Calculate total initial mass for remaining stages
        total_initial_mass = 0.0
        for i, s in enumerate(self.stages):
            if i < self.current_stage_idx:
                continue
            total_initial_mass += s.mass_dry + s.mass_propellant
        
        # Calculate dry mass for remaining stages
        total_dry_mass = 0.0
        for i, s in enumerate(self.stages):
            if i < self.current_stage_idx:
                continue
            total_dry_mass += s.mass_dry
        
        # Total propellant for remaining stages
        total_propellant = total_initial_mass - total_dry_mass
        
        # Clamp current_mass to valid range
        current_mass = max(current_mass, total_dry_mass)
        current_mass = min(current_mass, total_initial_mass)
        
        # Propellant consumed
        propellant_consumed = total_initial_mass - current_mass
        
        # Fraction consumed (0 = full, 1 = empty)
        if total_propellant > 0:
            fraction = propellant_consumed / total_propellant
        else:
            fraction = 0.0
        
        # Clamp fraction to [0, 1]
        fraction = max(0.0, min(1.0, fraction))
        
        # Now compute inertia using this fraction
        
        # Compute combined center of gravity based on mass
        combined_cg = self._compute_combined_cg_from_fraction(fraction)
        
        # Initialize combined inertia tensor
        I_combined = np.zeros((3, 3))
        
        for i, s in enumerate(self.stages):
            if i < self.current_stage_idx:
                continue  # Skip separated stages
            
            # Get stage properties
            if i == self.current_stage_idx:
                stage_mass = s.mass_dry
                stage_cg = s.xcg_empty
                
                # Compute stage inertia about its own CG
                if s.has_motor and s.mass_propellant > 0:
                    propellant_remaining = s.mass_propellant * (1.0 - fraction)
                    stage_mass += propellant_remaining
                    stage_cg = s.xcg_full + fraction * (s.xcg_empty - s.xcg_full)
                    
                    # Apply nonlinear interpolation using inertia_gamma
                    gamma = s.inertia_gamma
                    if gamma != 1.0 and fraction > 0:
                        effective_fraction = fraction ** gamma
                    else:
                        effective_fraction = fraction
                    
                    # Try Cholesky-factor interpolation first
                    I_chol = s.get_inertia_from_cholesky(effective_fraction)
                    if I_chol is not None:
                        Ixx = I_chol[0, 0]
                        Iyy = I_chol[1, 1]
                        Izz = I_chol[2, 2]
                        Ixy = I_chol[0, 1]
                        Ixz = I_chol[0, 2]
                        Iyz = I_chol[1, 2]
                    else:
                        # Fallback to linear interpolation
                        Ixx = s.Ixx_full + effective_fraction * (s.Ixx_empty - s.Ixx_full)
                        Iyy = s.Iyy_full + effective_fraction * (s.Iyy_empty - s.Iyy_full)
                        Izz = s.Izz_full + effective_fraction * (s.Izz_empty - s.Izz_full)
                        Ixy = s.Ixy_full + effective_fraction * (s.Ixy_empty - s.Ixy_full)
                        Ixz = s.Ixz_full + effective_fraction * (s.Ixz_empty - s.Ixz_full)
                        Iyz = s.Iyz_full + effective_fraction * (s.Iyz_empty - s.Iyz_full)
                else:
                    Ixx = s.Ixx_empty
                    Iyy = s.Iyy_empty
                    Izz = s.Izz_empty
                    Ixy = s.Ixy_empty
                    Ixz = s.Ixz_empty
                    Iyz = s.Iyz_empty
            else:
                # Future stages - full mass and inertia
                stage_mass = s.mass_dry + s.mass_propellant
                stage_cg = s.xcg_full
                Ixx = s.Ixx_full
                Iyy = s.Iyy_full
                Izz = s.Izz_full
                Ixy = s.Ixy_full
                Ixz = s.Ixz_full
                Iyz = s.Iyz_full
            
            # Stage inertia tensor about its own CG
            I_stage_cm = np.array([
                [Ixx, Ixy, Ixz],
                [Ixy, Iyy, Iyz],
                [Ixz, Iyz, Izz]
            ])
            
            # Validate and correct positive-definiteness
            if i == self.current_stage_idx and s.has_motor and s.mass_propellant > 0:
                I_stage_cm = self._validate_positive_definite(I_stage_cm, s.stage_id, fraction)
            
            # Offset from stage CG to combined CG
            offset = np.array([stage_cg - combined_cg, 0.0, 0.0])
            
            # Apply parallel axis theorem
            I_stage_combined = self.parallel_axis_theorem(I_stage_cm, stage_mass, offset)
            
            # Add to combined inertia
            I_combined += I_stage_combined
        
        return I_combined
    
    def _compute_combined_cg_from_fraction(self, fraction: float) -> float:
        """
        Compute combined CG location based on propellant fraction consumed.
        
        Args:
            fraction: Propellant fraction consumed (0 = full, 1 = empty)
            
        Returns:
            Combined CG location from reference point (m)
        """
        total_mass = 0.0
        mass_moment = 0.0
        
        for i, stage in enumerate(self.stages):
            if i < self.current_stage_idx:
                continue
            
            if i == self.current_stage_idx:
                stage_mass = stage.mass_dry
                stage_cg = stage.xcg_empty
                
                if stage.has_motor and stage.mass_propellant > 0:
                    propellant_remaining = stage.mass_propellant * (1.0 - fraction)
                    stage_mass += propellant_remaining
                    stage_cg = stage.xcg_full + fraction * (stage.xcg_empty - stage.xcg_full)
            else:
                stage_mass = stage.mass_dry + stage.mass_propellant
                stage_cg = stage.xcg_full
            
            total_mass += stage_mass
            mass_moment += stage_mass * stage_cg
        
        if total_mass > 0:
            return mass_moment / total_mass
        return 0.0

    def get_inertia_diagonal(self, t: float) -> tuple:
        """
        Get diagonal inertia components at time t.
        
        Args:
            t: Time (s)
            
        Returns:
            tuple: (Ixx, Iyy, Izz) in kg·m²
        """
        if t < 0:
            t = 0.0
        
        # Use get_inertia_matrix to ensure consistency with combined properties
        # This properly accounts for all stages and parallel axis theorem
        matrix = self.get_inertia_matrix(t)
        
        return matrix[0, 0], matrix[1, 1], matrix[2, 2]
    
    def get_properties(self, t: float) -> dict:
        """
        Get all mass properties at time t.
        
        Args:
            t: Time (s)
            
        Returns:
            dict with mass properties and stage info
        """
        Ixx, Iyy, Izz = self.get_inertia_diagonal(t)
        stage = self.get_current_stage()
        
        return {
            'mass': self.get_mass(t),
            'cg': self.get_cg_location(t),
            'inertia': self.get_inertia_matrix(t),
            'mass_flow': self.get_mass_flow_rate(t),
            'Ixx': Ixx,
            'Iyy': Iyy,
            'Izz': Izz,
            'current_stage': self.current_stage_idx + 1,
            'stage_name': stage.name,
            'has_motor': stage.has_motor,
            'has_fins': stage.has_fins,
            'num_stages': len(self.stages),
            'separated_mass': self.total_separation_mass,
        }
    
    def is_multi_stage(self) -> bool:
        """Check if this is a multi-stage configuration."""
        return len(self.stages) > 1
    
    def get_num_stages(self) -> int:
        """Get total number of stages."""
        return len(self.stages)
    
    def get_remaining_stages(self) -> int:
        """Get number of remaining stages (including current)."""
        return len(self.stages) - self.current_stage_idx
    
    def _clear_cache(self) -> None:
        """Clear all cached values."""
        self._mass_cache.clear()
        self._cg_cache.clear()
        self._inertia_cache.clear()
    
    def enable_cache(self, enabled: bool = True) -> None:
        """
        Enable or disable caching.
        
        Args:
            enabled: True to enable caching, False to disable
        """
        self._cache_enabled = enabled
        if not enabled:
            self._clear_cache()
    
    def get_cache_stats(self) -> Dict[str, Any]:
        """
        Get cache and diagnostics statistics.
        
        Returns:
            Dictionary with cache sizes and SPD correction diagnostics
        """
        avg_correction = 0.0
        if self._spd_correction_count > 0:
            avg_correction = self._spd_correction_total_magnitude / self._spd_correction_count
        
        return {
            'mass_cache_size': len(self._mass_cache),
            'cg_cache_size': len(self._cg_cache),
            'inertia_cache_size': len(self._inertia_cache),
            'max_cache_size': self._max_cache_size,
            'cache_enabled': self._cache_enabled,
            'spd_correction_count': self._spd_correction_count,
            'spd_correction_total_magnitude': self._spd_correction_total_magnitude,
            'spd_correction_avg_magnitude': avg_correction
        }
    
    def set_stage_start_time(self, t: float):
        """
        Set the stage start time for the current stage.

        Args:
            t: Simulation time at stage start (s).
        """
        # Ensure the list has enough entries for the current stage
        while len(self.stage_start_times) <= self.current_stage_idx:
            self.stage_start_times.append(0.0)
        
        # Update the start time for the current stage
        self.stage_start_times[self.current_stage_idx] = t
        
        # Clear cache since time reference has changed
        self._clear_cache()
        
        import logging
        logger = logging.getLogger(__name__)
        logger.info(f"MassPropertiesModel: stage_start_time set to {t:.4f}s for stage {self.current_stage_idx + 1}")

    def reset(self):
        """Reset to initial state."""
        self.current_stage_idx = 0
        self.stage_start_times = [0.0]
        self.separation_times = []
        self.total_separation_mass = 0.0
        self._clear_cache()
        self._spd_correction_count = 0
        self._spd_correction_total_magnitude = 0.0


class MassPropertiesModel(MultiStageMassPropertiesModel):
    """
    Time-varying mass properties model for single-stage rockets.
    
    This is a simplified interface that inherits from MultiStageMassPropertiesModel
    with a single stage. It provides backward compatibility while eliminating code duplication.
    
    Tracks mass, CG location, and inertia tensor as propellant burns.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize single-stage mass properties model.
        
        Converts legacy config format to multi-stage format with one stage.
        
        Args:
            config: Dictionary with mass properties configuration
        """
        # Convert single-stage config to multi-stage format
        stage_config = {
            'stages': [{
                'id': 1,
                'name': 'Stage 1',
                'mass_dry': config.get('mass_dry', 375.0),
                'mass_propellant': config.get('mass_propellant', 238.0),
                'Ixx_full': config.get('Ixx_0', 7.77),
                'Iyy_full': config.get('Iyy_0', 7025.0),
                'Izz_full': config.get('Izz_0', 7025.0),
                'Ixx_empty': config.get('Ixx_end', 5.318),
                'Iyy_empty': config.get('Iyy_end', 3735.0),
                'Izz_empty': config.get('Izz_end', 3735.0),
                'xcg_full': config.get('xcg_0', 3.005),
                'xcg_empty': config.get('xcg_end', 2.65),
                'burn_time': config.get('burn_time', 6.34),
                'has_motor': True,
                'has_fins': True,
                'Ixy_full': config.get('Ixy_0', config.get('Ixy', 0.0)),
                'Ixz_full': config.get('Ixz_0', config.get('Ixz', 0.0)),
                'Iyz_full': config.get('Iyz_0', config.get('Iyz', 0.0)),
                'Ixy_empty': config.get('Ixy_end', config.get('Ixy', 0.0)),
                'Ixz_empty': config.get('Ixz_end', config.get('Ixz', 0.0)),
                'Iyz_empty': config.get('Iyz_end', config.get('Iyz', 0.0)),
            }],
            # Forward cache configuration to parent class
            'enable_cache': config.get('enable_cache', True),
            'max_cache_size': config.get('max_cache_size', 128),
            # Forward cache time resolution (default 1e-9s for adaptive integrators)
            # Set to 0 or negative to disable caching entirely
            'cache_time_resolution': config.get('cache_time_resolution', 1e-9),
            # Forward SPD tolerance for inertia validation
            'inertia_spd_tol': config.get('inertia_spd_tol', 1e-10)
        }
        
        # Add thrust curve if provided
        if 'thrust_curve' in config:
            stage_config['stages'][0]['thrust_curve'] = config['thrust_curve']
        
        # Initialize parent class with single-stage configuration
        super().__init__(stage_config)
        
        # Store legacy attribute names for backward compatibility
        stage = self.stages[0]
        self.M_body = stage.mass_dry
        self.M_fuel = stage.mass_propellant
        self.M_total = self.M_body + self.M_fuel
        self.Ixx_0 = stage.Ixx_full
        self.Iyy_0 = stage.Iyy_full
        self.Izz_0 = stage.Izz_full
        self.Ixx_end = stage.Ixx_empty
        self.Iyy_end = stage.Iyy_empty
        self.Izz_end = stage.Izz_empty
        self.xcg_0 = stage.xcg_full
        self.xcg_end = stage.xcg_empty
        self.t_burn = stage.burn_time
        self.Ixy_0 = stage.Ixy_full
        self.Ixz_0 = stage.Ixz_full
        self.Iyz_0 = stage.Iyz_full
        self.Ixy_end = stage.Ixy_empty
        self.Ixz_end = stage.Ixz_empty
        self.Iyz_end = stage.Iyz_empty
        self.thrust_curve = stage.thrust_curve
    
    # Inherits all mass-property APIs from MultiStageMassPropertiesModel.

