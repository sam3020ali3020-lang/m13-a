#!/usr/bin/env python3
"""
Rocket Data Loader - loads rocket properties, aero tables, thrust curves from rocket_models/.
Supports single-stage & multi-stage rockets.
"""

import yaml
import pandas as pd
import numpy as np
from pathlib import Path
from typing import Dict, Any, Optional, List, Tuple
from dataclasses import dataclass, field
import logging

from dynamics.enums import SeparationTrigger

logger = logging.getLogger(__name__)

# Default safety limits constants
# ثوابت حدود السلامة الافتراضية
DEFAULT_PITCH_MAX_RAD = 1.48    # rad (85 deg) - Maximum pitch angle
DEFAULT_ROLL_MAX_RAD = 0.785    # rad (45 deg) - Maximum roll angle


class RocketDataError(Exception):
    """Exception raised when rocket data cannot be loaded."""
    pass


def estimate_propellant_inertia_hollow_cylinder(
    mass: float,
    outer_radius: float,
    length: float,
    inner_radius_ratio: float = 0.5
) -> List[float]:
    """Estimate propellant inertia [Ixx, Iyy, Izz] (kg·m²) as a hollow cylinder.
    Ixx = m/2*(R²+r²), Iyy=Izz = m/12*(3*(R²+r²)+L²)."""
    # Validate input parameters
    if mass < 0:
        raise ValueError(f"Propellant mass must be non-negative, got {mass} kg")
    if outer_radius < 0:
        raise ValueError(f"Outer radius must be non-negative, got {outer_radius} m")
    if length < 0:
        raise ValueError(f"Length must be non-negative, got {length} m")
    if not (0 <= inner_radius_ratio < 1):
        raise ValueError(
            f"inner_radius_ratio must be in range [0, 1), got {inner_radius_ratio}"
        )
    
    # Physical edge case: zero mass → zero inertia
    if mass == 0:
        return [0.0, 0.0, 0.0]
    
    # Physical edge case: zero dimensions with non-zero mass is invalid
    if outer_radius == 0:
        raise ValueError(
            f"Invalid propellant geometry: outer_radius=0 with mass={mass} kg. "
            f"A propellant grain with non-zero mass must have non-zero outer radius. "
            f"هندسة الوقود غير صالحة: نصف القطر الخارجي=0 مع كتلة={mass} كجم"
        )
    if length == 0:
        raise ValueError(
            f"Invalid propellant geometry: length=0 with mass={mass} kg. "
            f"A propellant grain with non-zero mass must have non-zero length. "
            f"هندسة الوقود غير صالحة: الطول=0 مع كتلة={mass} كجم"
        )
    
    R = outer_radius
    r = inner_radius_ratio * outer_radius
    R2 = R * R
    r2 = r * r
    L2 = length * length
    
    # Roll inertia (about longitudinal axis)
    Ixx = 0.5 * mass * (R2 + r2)
    
    # Pitch/Yaw inertia (about lateral axes)
    Iyy = (1.0 / 12.0) * mass * (3.0 * (R2 + r2) + L2)
    Izz = Iyy  # Axially symmetric
    
    return [Ixx, Iyy, Izz]


def compute_full_inertia_with_parallel_axis(
    inertia_dry: List[float],
    mass_dry: float,
    cg_dry: List[float],
    inertia_propellant: List[float],
    mass_propellant: float,
    cg_propellant: List[float]
) -> List[float]:
    """Combine dry + propellant inertia using 3D parallel axis theorem (diagonal elements only).
    Returns [Ixx, Iyy, Izz] about combined CG (kg·m²)."""
    import numpy as np
    
    mass_total = mass_dry + mass_propellant
    
    if mass_total <= 0:
        return list(inertia_dry)
    
    # Convert to numpy arrays for vector operations
    cg_dry_vec = np.array(cg_dry)
    cg_prop_vec = np.array(cg_propellant)
    
    # Calculate combined CG (3D)
    cg_combined = (mass_dry * cg_dry_vec + mass_propellant * cg_prop_vec) / mass_total
    
    # Offset vectors from each component CG to combined CG (3D)
    d_dry = cg_dry_vec - cg_combined
    d_prop = cg_prop_vec - cg_combined
    
    # Extract components
    dx_dry, dy_dry, dz_dry = d_dry[0], d_dry[1], d_dry[2]
    dx_prop, dy_prop, dz_prop = d_prop[0], d_prop[1], d_prop[2]
    
    # 3D parallel axis theorem: Ixx += m*(dy²+dz²), Iyy += m*(dx²+dz²), Izz += m*(dx²+dy²)
    
    Ixx_combined = (inertia_dry[0] + mass_dry * (dy_dry**2 + dz_dry**2) +
                   inertia_propellant[0] + mass_propellant * (dy_prop**2 + dz_prop**2))
    
    Iyy_combined = (inertia_dry[1] + mass_dry * (dx_dry**2 + dz_dry**2) +
                   inertia_propellant[1] + mass_propellant * (dx_prop**2 + dz_prop**2))
    
    Izz_combined = (inertia_dry[2] + mass_dry * (dx_dry**2 + dy_dry**2) +
                   inertia_propellant[2] + mass_propellant * (dx_prop**2 + dy_prop**2))
    
    return [Ixx_combined, Iyy_combined, Izz_combined]


def estimate_full_inertia(
    props: 'RocketProperties',
    identifier: str = "Rocket"
) -> List[float]:
    """Estimate full inertia [Ixx, Iyy, Izz] (kg·m²) using parallel axis theorem
    and hollow cylinder propellant model. Uses explicit inertia_full_kgm2 if provided."""
    inertia_dry = props.inertia_dry_kgm2
    
    if props.inertia_full_kgm2 is not None:
        inertia_full = list(props.inertia_full_kgm2)
        logger.info(
            f"{identifier}: Using explicit inertia_full from YAML: "
            f"[{inertia_full[0]:.2f}, {inertia_full[1]:.2f}, {inertia_full[2]:.2f}] kg·m²"
        )
        return inertia_full
    
    if props.propellant_mass_kg > 0 and props.mass_dry_kg > 0:
        # Use full 3D CG vectors for accurate parallel axis theorem calculation
        cg_dry = props.cg_dry_body_m if props.cg_dry_body_m else [0.0, 0.0, 0.0]
        cg_prop = props.cg_propellant_body_m if props.cg_propellant_body_m else list(cg_dry)
        
        if props.propellant_outer_radius_m is not None and props.propellant_length_m is not None:
            propellant_outer_radius = props.propellant_outer_radius_m
            propellant_length = props.propellant_length_m
            inner_radius_ratio = props.propellant_inner_radius_ratio
            logger.info(
                f"{identifier}: Using explicit propellant geometry from YAML: "
                f"R={propellant_outer_radius:.4f}m, L={propellant_length:.4f}m, "
                f"inner_ratio={inner_radius_ratio:.2f}"
            )
        else:
            propellant_outer_radius = 0.4 * props.ref_diameter_m
            propellant_length = 0.6 * props.ref_length_m
            inner_radius_ratio = props.propellant_inner_radius_ratio
            logger.warning(
                f"{identifier}: Propellant geometry not specified in YAML. "
                f"Using estimated values: R={propellant_outer_radius:.4f}m (40% of diameter), "
                f"L={propellant_length:.4f}m (60% of length), inner_ratio={inner_radius_ratio:.2f}. "
                f"For better accuracy, specify 'propellant_outer_radius_m' and 'propellant_length_m' in YAML."
            )
        
        inertia_propellant = estimate_propellant_inertia_hollow_cylinder(
            mass=props.propellant_mass_kg,
            outer_radius=propellant_outer_radius,
            length=propellant_length,
            inner_radius_ratio=inner_radius_ratio
        )
        
        inertia_full = compute_full_inertia_with_parallel_axis(
            inertia_dry=inertia_dry,
            mass_dry=props.mass_dry_kg,
            cg_dry=cg_dry,
            inertia_propellant=inertia_propellant,
            mass_propellant=props.propellant_mass_kg,
            cg_propellant=cg_prop
        )
        
        logger.info(
            f"{identifier}: Estimated inertia_full using parallel axis theorem: "
            f"[{inertia_full[0]:.2f}, {inertia_full[1]:.2f}, {inertia_full[2]:.2f}] kg·m². "
            f"Propellant modeled as hollow cylinder (R={propellant_outer_radius:.3f}m, L={propellant_length:.3f}m). "
            f"For best accuracy, specify inertia_full_kgm2 in YAML."
        )
        return inertia_full
    
    elif props.propellant_mass_kg > 0 and props.mass_dry_kg <= 0:
        logger.warning(
            f"{identifier}: mass_dry_kg={props.mass_dry_kg} <= 0 with "
            f"propellant_mass_kg={props.propellant_mass_kg} > 0. "
            f"Using dry inertia as fallback."
        )
        return list(inertia_dry)
    
    else:
        return list(inertia_dry)


@dataclass
class RocketProperties:
    """Container for rocket physical properties.
    """
    mass_dry_kg: float
    propellant_mass_kg: float
    cg_dry_body_m: List[float]
    inertia_dry_kgm2: List[float]
    ref_diameter_m: float
    ref_length_m: float
    ref_area_m2: float
    
    # Propellant CG (defaults to cg_dry if None and propellant_mass > 0)
    cg_propellant_body_m: Optional[List[float]] = None
    
    nozzle_exit_area_m2: float = 0.0
    sea_level_pressure_Pa: float = 101325.0
    inertia_full_kgm2: Optional[List[float]] = None  # Optional: explicit full inertia [Ixx, Iyy, Izz]
    
    # Propellant geometry for inertia estimation (defaults estimated from rocket dims)
    propellant_outer_radius_m: Optional[float] = None
    propellant_length_m: Optional[float] = None
    propellant_inner_radius_ratio: float = 0.5  # inner/outer radius ratio (hollow grain)
    
    fin: Dict[str, Any] = field(default_factory=dict)
    actuator: Dict[str, Any] = field(default_factory=dict)
    safety: Dict[str, Any] = field(default_factory=dict)
    num_stages: int = 1
    servo_torque_max_Nm: float = 50.0
    
    # Launcher (launch rail) parameters - specific to each rocket model
    launcher: Dict[str, Any] = field(default_factory=dict)
    
    # Insulation mass - ablates proportionally with propellant
    insulation_mass_kg: float = 0.0
    
    # CA multiplier - multiplied with axial force coefficient
    CA_multiplier: float = 1.0
    
    # Burn time scale (scales thrust curve time, preserves total impulse; >1=longer/lower, <1=shorter/higher)
    burn_time_scale: float = 1.0
    
    # Thrust multiplier (scales thrust, keeps time & mass constant, effectively scales Isp)
    thrust_multiplier: float = 1.0
    
    # Fin config: "X" (45°, all fins pitch/yaw) or "+" (axis-aligned, pairs)
    config_type: str = "X"
    
    # Fin location: "canard" (front, +defl -> -pitch) or "tail" (rear, +defl -> +pitch)
    fin_location: str = "tail"
    
    # Moment reference point (m from nose) - used in CFD/wind tunnel for Cm/Cn.
    # When CG != ref point: M_CG = M_ref + (x_CG - x_ref) * F_normal
    # If None, no moment transfer correction (assumes tables are about CG).
    moment_reference_point: Optional[float] = None

    # Motor type - "solid" (default) or "none"
    motor_type: str = "solid"
    
    def get_cg_propellant(self) -> List[float]:
        """Get propellant CG, defaulting to dry CG if not specified."""
        if self.cg_propellant_body_m is not None:
            return self.cg_propellant_body_m
        return list(self.cg_dry_body_m)


@dataclass
class AeroTables:
    """Container for aerodynamic coefficient tables.

    CA priority: 1) ca_motor_on + ca_motor_off (if both), 2) ca single table (legacy).
    Fin priority: 1) fin_coeffs (per radian, legacy), 2) fin_deflection_coeffs (CNd/CMd, converted).
    """
    aero_coeffs: Optional[pd.DataFrame] = None  # Cd, Cn, Cm vs Mach, alpha
    fin_coeffs: Optional[pd.DataFrame] = None   # Cn_delta, Cm_delta, Cl_delta, Ch_delta
    damping_coeffs: Optional[pd.DataFrame] = None  # Cmq, Cnq, Clp
    fin_deflection_drag: Optional[pd.DataFrame] = None  # Cd_delta
    ca_3d_coeffs: Optional[pd.DataFrame] = None  # CA vs Mach, alpha, Reynolds (legacy)
    roll_aero_coeffs: Optional[pd.DataFrame] = None  # Roll coefficients
    fin_loads: Optional[pd.DataFrame] = None  # Fin structural loads
    
    # New tables for enhanced simulation
    # جداول جديدة للمحاكاة المحسنة
    ca_3d_coeffs_motor_on: Optional[pd.DataFrame] = None  # CA with fins/motor on
    ca_3d_coeffs_motor_off: Optional[pd.DataFrame] = None  # CA body only/motor off
    fin_deflection_coeffs: Optional[pd.DataFrame] = None  # CNd, CMd vs Mach, alpha


@dataclass
class ThrustCurve:
    """Container for thrust curve data (lightweight data transport).

    Two ThrustCurve implementations exist:
    1. This class: lightweight loader/interpolation container.
    2. dynamics/mass_properties.ThrustCurve: physics (impulse, propellant fraction).

    Convert via: PhysicsThrustCurve.from_loader_curve(loader_curve, propellant_mass=...)

    Attributes:
        time: Time points array (s) - strictly increasing
        thrust: Thrust values array (N)
        mass_flow: Optional mass flow rate array (kg/s)
    """
    time: np.ndarray
    thrust: np.ndarray
    mass_flow: Optional[np.ndarray] = None

    def __post_init__(self):
        """Validate thrust curve data integrity."""
        # Check that time and thrust arrays have the same length
        if len(self.time) != len(self.thrust):
            raise ValueError(
                f"ThrustCurve validation failed: time and thrust arrays must have the same length. "
                f"Got time: {len(self.time)}, thrust: {len(self.thrust)}"
            )
        
        # Check that time array has at least 2 points for interpolation
        if len(self.time) < 2:
            raise ValueError(
                f"ThrustCurve validation failed: time array must have at least 2 points for interpolation. "
                f"Got {len(self.time)} points"
            )
        
        # Check that time array is strictly increasing (no duplicate times)
        # This is required for proper interpolation behavior
        if not np.all(np.diff(self.time) > 0):
            raise ValueError(
                "ThrustCurve validation failed: time array must be strictly increasing (no duplicate time values allowed)"
            )
        
        # Check that mass_flow (if provided) has the same length as time
        if self.mass_flow is not None and len(self.mass_flow) != len(self.time):
            raise ValueError(
                f"ThrustCurve validation failed: mass_flow array (if provided) must have the same length as time. "
                f"Got time: {len(self.time)}, mass_flow: {len(self.mass_flow)}"
            )
        
        # Check for non-negative thrust values (physical constraint)
        if np.any(self.thrust < 0):
            raise ValueError(
                "ThrustCurve validation failed: thrust values must be non-negative (physical constraint)"
            )
        
        # Check for non-negative time values
        if np.any(self.time < 0):
            raise ValueError(
                "ThrustCurve validation failed: time values must be non-negative"
            )

    def get_thrust_at_time(self, t: float) -> float:
        """Interpolate thrust at given time. Returns 0.0 outside burn period."""
        return np.interp(t, self.time, self.thrust, left=0.0, right=0.0)

    def get_mass_flow_at_time(self, t: float) -> float:
        """Interpolate mass flow at given time. Returns 0.0 outside burn period."""
        if self.mass_flow is not None:
            return np.interp(t, self.time, self.mass_flow, left=0.0, right=0.0)
        return 0.0

    def calculate_burn_time(self, threshold_ratio: float = 0.1) -> float:
        """Calculate burn time using linear interpolation when thrust drops below threshold_ratio of max."""
        if len(self.thrust) == 0:
            return 0.0
        
        max_thrust = np.max(self.thrust)
        
        # Handle zero or negligible thrust curves
        if max_thrust <= 1e-9:
            return 0.0
        
        threshold = threshold_ratio * max_thrust
        
        above_threshold = self.thrust > threshold
        if not np.any(above_threshold):
            return 0.0
        
        last_idx = np.where(above_threshold)[0][-1]
        
        # If last point above threshold is the last data point, return that time
        if last_idx >= len(self.thrust) - 1:
            return float(self.time[last_idx])
        
        # Linear interpolation to find exact crossing point
        t1 = self.time[last_idx]
        t2 = self.time[last_idx + 1]
        f1 = self.thrust[last_idx]
        f2 = self.thrust[last_idx + 1]
        
        # Avoid division by zero
        if abs(f1 - f2) < 1e-12:
            return float(t1)
        
        # Linear interpolation: t = t1 + (threshold - f1) * (t2 - t1) / (f2 - f1)
        t_crossing = t1 + (threshold - f1) * (t2 - t1) / (f2 - f1)
        
        return float(t_crossing)

    def get_depletion_time(self) -> float:
        """Get depletion time (last time point), may be longer than burn_time if tail-off included."""
        if len(self.time) == 0:
            return 0.0
        return float(self.time[-1])


@dataclass
class RocketData:
    """Complete rocket data container."""
    name: str
    stage: Optional[str]
    properties: RocketProperties
    aero_tables: AeroTables
    thrust_curve: Optional[ThrustCurve]
    atmosphere_table: Optional[pd.DataFrame] = None
    data_dir: Optional[Path] = None  # Directory from which this data was loaded


@dataclass
class StageData:
    """Container for a single stage's data in a multi-stage rocket."""
    id: int
    name: str
    name_ar: str
    rocket_data: RocketData
    has_motor: bool
    has_fins: bool
    separation_trigger: SeparationTrigger
    separation_value: float


@dataclass
class MultiStageRocketData:
    """Complete multi-stage rocket data container."""
    name: str
    stages: List[StageData]
    num_stages: int
    
    def get_stage(self, stage_id: int) -> Optional[StageData]:
        """Get stage by ID (1-indexed)."""
        for stage in self.stages:
            if stage.id == stage_id:
                return stage
        return None
    
    def get_stage_by_name(self, name: str) -> Optional[StageData]:
        """Get stage by name (e.g., 'stage1', 'warhead')."""
        for stage in self.stages:
            if stage.rocket_data.stage == name:
                return stage
        return None


class RocketDataLoader:
    """Loader for rocket data from the rocket_models directory."""

    def __init__(self, base_dir: str = "data/rocket_models"):
        """Initialize with base_dir (absolute or relative to 6dof directory)."""
        self.base_dir = Path(base_dir)

        # If relative path, make it relative to 6dof directory
        if not self.base_dir.is_absolute():
            sixdof_dir = Path(__file__).parent
            self.base_dir = sixdof_dir / base_dir

        if not self.base_dir.exists():
            raise RocketDataError(f"Rocket data directory not found: {self.base_dir}")

    def list_available_rockets(self) -> List[str]:
        """List all available rocket models."""
        rockets = []
        for item in self.base_dir.iterdir():
            if item.is_dir() and item.name.startswith("rocket_"):
                rockets.append(item.name)
        return sorted(rockets)

    # Supported stage directory names
    # These are recognized as stage directories in multi-stage rockets
    KNOWN_STAGE_NAMES = {
        'booster', 'stage1', 'stage2', 'stage3', 'stage4',
        'sustainer', 'payload', 'warhead'
    }

    def _find_stage_dir(self, rocket_dir: Path, stage_name: str) -> Optional[Path]:
        """Find stage directory with case-insensitive matching. Returns None if not found."""
        stage_lower = stage_name.lower()
        for item in rocket_dir.iterdir():
            if item.is_dir() and item.name.lower() == stage_lower:
                return item
        return None

    def list_stages(self, rocket_name: str) -> List[str]:
        """List available stages for a multi-stage rocket. Returns [] for single-stage."""
        rocket_dir = self.base_dir / rocket_name
        if not rocket_dir.exists():
            raise RocketDataError(f"Rocket not found: {rocket_name}")

        stages = []
        for item in rocket_dir.iterdir():
            if not item.is_dir():
                continue
            
            # Case-insensitive comparison to handle Stage1, STAGE1, stage1, etc.
            if item.name.lower() in self.KNOWN_STAGE_NAMES:
                stages.append(item.name.lower())  # Normalize to lowercase
            # Also detect custom stages: directories with rocket_properties.yaml
            elif (item / 'rocket_properties.yaml').exists():
                stages.append(item.name.lower())  # Normalize to lowercase
                logger.debug(f"Detected custom stage directory: {item.name}")

        return sorted(stages)  # Empty list means single-stage rocket

    def load(self, rocket_name: str, stage: Optional[str] = None) -> RocketData:
        """Load complete rocket data. Use stage param for multi-stage rockets."""
        rocket_dir = self.base_dir / rocket_name

        if not rocket_dir.exists():
            available = self.list_available_rockets()
            raise RocketDataError(
                f"Rocket '{rocket_name}' not found in {self.base_dir}. "
                f"Available rockets: {available}"
            )

        # Determine data directory (root or stage subdirectory)
        if stage:
            # Use case-insensitive lookup for stage directory
            stage_dir = self._find_stage_dir(rocket_dir, stage)
            if stage_dir is None:
                available_stages = self.list_stages(rocket_name)
                raise RocketDataError(
                    f"Stage '{stage}' not found for rocket '{rocket_name}'. "
                    f"Available stages: {available_stages}"
                )
            data_dir = stage_dir
            stage = stage.lower()  # Normalize stage name for consistency
        else:
            # Check if this is a multi-stage rocket
            stages = self.list_stages(rocket_name)
            if stages:
                # Multi-stage rocket, default to stage1
                logger.info(
                    f"Rocket '{rocket_name}' has multiple stages: {stages}. "
                    f"Auto-selecting 'stage1'. Specify stage explicitly for other stages."
                )
                stage = "stage1"
                # Use case-insensitive lookup for stage directory
                stage_dir = self._find_stage_dir(rocket_dir, stage)
                if stage_dir is None:
                    raise RocketDataError(
                        f"Stage '{stage}' directory not found for rocket '{rocket_name}'"
                    )
                data_dir = stage_dir
            else:
                # Single-stage rocket - data is in root directory
                data_dir = rocket_dir

        # Load all data
        properties = self._load_properties(data_dir, rocket_name)
        aero_tables = self._load_aero_tables(data_dir)
        thrust_curve = self._load_thrust_curve(data_dir)
        atmosphere_table = self._load_atmosphere_table(data_dir)

        return RocketData(
            name=rocket_name,
            stage=stage,
            properties=properties,
            aero_tables=aero_tables,
            thrust_curve=thrust_curve,
            atmosphere_table=atmosphere_table,
            data_dir=data_dir,
        )

    def load_multi_stage(self, rocket_name: str) -> MultiStageRocketData:
        """Load all stages of a multi-stage rocket into MultiStageRocketData."""
        stages_list = self.list_stages(rocket_name)
        
        if not stages_list:
            raise RocketDataError(
                f"Rocket '{rocket_name}' is not a multi-stage rocket (no stage directories found). "
                f"Use load() method for single-stage rockets."
            )
        
        # Stage IDs (1-indexed): booster=1, stage1=2, ..., warhead=100 (always last)
        # Unknown stages get sequential IDs from max(known_ids)+1
        stage_order = {
            'booster': 1,
            'stage1': 2,
            'stage2': 3,
            'stage3': 4,
            'stage4': 5,
            'sustainer': 6,
            'payload': 7,
            'warhead': 100,  # Always last
        }
        stage_names = {
            'booster': ('Booster', 'الدافع'),
            'stage1': ('Stage 1', 'المرحلة الأولى'),
            'stage2': ('Stage 2', 'المرحلة الثانية'),
            'stage3': ('Stage 3', 'المرحلة الثالثة'),
            'stage4': ('Stage 4', 'المرحلة الرابعة'),
            'sustainer': ('Sustainer', 'المستدام'),
            'payload': ('Payload', 'الحمولة'),
            'warhead': ('Warhead', 'الرأس الحربي'),
        }
        
        stages = []
        used_ids = set()  # Track used IDs to avoid collisions
        next_unknown_id = 50  # Start unknown stages at ID 50 (between known stages and warhead)
        
        for stage_name in stages_list:
            rocket_data = self.load(rocket_name, stage=stage_name)
            props = rocket_data.properties
            
            # Determine stage properties with collision avoidance
            if stage_name in stage_order:
                stage_id = stage_order[stage_name]
            else:
                # Unknown stage: assign next available ID
                while next_unknown_id in used_ids or next_unknown_id in stage_order.values():
                    next_unknown_id += 1
                stage_id = next_unknown_id
                next_unknown_id += 1
                logger.warning(
                    f"Unknown stage name '{stage_name}' assigned ID {stage_id}. "
                    f"Consider adding it to the stage_order dictionary. "
                    f"اسم مرحلة غير معروف '{stage_name}' تم تعيين المعرف {stage_id}"
                )
            
            used_ids.add(stage_id)
            name_en, name_ar = stage_names.get(stage_name, (stage_name.replace('_', ' ').title(), stage_name))
            
            # Determine if stage has motor (propellant > 0) and fins
            has_motor = props.propellant_mass_kg > 0
            has_fins = bool(props.fin) and props.fin.get('S_fin_m2', 0) > 0
            
            # Default separation trigger: BURNOUT for stages with motor, MANUAL for warhead
            if stage_name == 'warhead':
                separation_trigger = SeparationTrigger.MANUAL  # Never triggers automatically
            else:
                separation_trigger = SeparationTrigger.BURNOUT  # Separate when motor burns out
            
            stage_data = StageData(
                id=stage_id,
                name=name_en,
                name_ar=name_ar,
                rocket_data=rocket_data,
                has_motor=has_motor,
                has_fins=has_fins,
                separation_trigger=separation_trigger,
                separation_value=0.0
            )
            stages.append(stage_data)
        
        # Sort stages by ID
        stages.sort(key=lambda s: s.id)
        
        return MultiStageRocketData(
            name=rocket_name,
            stages=stages,
            num_stages=len(stages)
        )

    def build_multi_stage_config(self, multi_stage_data: MultiStageRocketData) -> Dict[str, Any]:
        """Build a 6DOF simulation config dict for a multi-stage rocket."""
        # Build stages list
        stages_config = []
        propulsion_config = {}
        cumulative_burn_time = 0.0
        stage_burnout_times = []
        
        # Use first stage for reference dimensions
        first_stage = multi_stage_data.stages[0]
        ref_props = first_stage.rocket_data.properties
        
        for stage in multi_stage_data.stages:
            props = stage.rocket_data.properties
            tc = stage.rocket_data.thrust_curve
            
            # Calculate inertia at full and empty propellant using helper function
            inertia_dry = props.inertia_dry_kgm2
            mass_total = props.mass_dry_kg + props.propellant_mass_kg
            inertia_full = estimate_full_inertia(props, identifier=f"Stage '{stage.name}'")
            
            # Calculate CG positions
            cg_dry = props.cg_dry_body_m[0] if props.cg_dry_body_m else 0.0
            cg_prop = props.cg_propellant_body_m[0] if props.cg_propellant_body_m else cg_dry
            if props.propellant_mass_kg > 0:
                cg_full = (props.mass_dry_kg * cg_dry + props.propellant_mass_kg * cg_prop) / mass_total
            else:
                cg_full = cg_dry
            
            # Get burn time from thrust curve using threshold-based calculation
            # This is consistent with AdvancedPropulsionModel._calculate_burn_time()
            if tc:
                burn_time = tc.calculate_burn_time()
            else:
                burn_time = 0.0
            cumulative_burn_time += burn_time
            stage_burnout_times.append(cumulative_burn_time)
            
            stage_config = {
                'id': stage.id,
                'name': stage.name,
                'name_ar': stage.name_ar,
                'mass_dry': props.mass_dry_kg,
                'mass_propellant': props.propellant_mass_kg,
                'mass_insulation': props.insulation_mass_kg,
                'Ixx_full': inertia_full[0],
                'Iyy_full': inertia_full[1],
                'Izz_full': inertia_full[2],
                'Ixx_empty': inertia_dry[0],
                'Iyy_empty': inertia_dry[1],
                'Izz_empty': inertia_dry[2],
                'xcg_full': cg_full,
                'xcg_empty': cg_dry,
                'burn_time': burn_time,
                'has_motor': stage.has_motor,
                'has_fins': stage.has_fins,
                'separation_trigger': stage.separation_trigger.value,  # Use .value for YAML/JSON serialization
                'separation_value': stage.separation_value,
            }
            stages_config.append(stage_config)
            
            # Build per-stage propulsion config
            stage_key = f'stage_{stage.id}'
            if stage.has_motor and tc:
                thrust_curve_list = [[float(t), float(f)] for t, f in zip(tc.time, tc.thrust)]
                stage_propulsion = {
                    'motor_type': 'solid',
                    'burn_time': burn_time,
                    'propellant_mass': props.propellant_mass_kg,
                    'thrust_curve': thrust_curve_list,
                    'thrust_scale': props.thrust_multiplier,  # Use thrust_multiplier from rocket_properties.yaml
                    'scale_mass_flow_with_thrust': False,  # Keep mass constant when thrust_multiplier != 1.0 (effectively changes Isp)
                }
                # Add mass flow curve if available for accurate propellant tracking
                if tc.mass_flow is not None:
                    stage_propulsion['mass_flow_curve'] = [float(m) for m in tc.mass_flow]

                # Altitude correction: F_corrected = F_vacuum + (P_ref - P_ambient) * A_exit
                if props.nozzle_exit_area_m2 > 0:
                    stage_propulsion['nozzle_exit_area'] = props.nozzle_exit_area_m2
                    stage_propulsion['reference_pressure'] = props.sea_level_pressure_Pa
                    logger.info(
                        f"Stage {stage.id}: Altitude correction enabled "
                        f"(nozzle_exit_area={props.nozzle_exit_area_m2:.6f} m², "
                        f"ref_pressure={props.sea_level_pressure_Pa:.0f} Pa)"
                    )
                else:
                    # Warn if this is a high-altitude rocket without altitude correction
                    if props.propellant_mass_kg > 100:  # Likely high-altitude rocket
                        logger.warning(
                            f"Stage {stage.id}: nozzle_exit_area not specified. "
                            f"Altitude-dependent thrust correction will be disabled. "
                            f"For accurate high-altitude performance, specify nozzle_exit_area_m2 in YAML."
                        )
                    stage_propulsion['reference_pressure'] = props.sea_level_pressure_Pa

                propulsion_config[stage_key] = stage_propulsion
            else:
                propulsion_config[stage_key] = {
                    'motor_type': 'none',
                    'burn_time': 0.0,
                    'propellant_mass': 0.0,
                }
        
        # Build aerodynamics config from first stage (full stack)
        aero_config = {
            'reference_area': ref_props.ref_area_m2,
            'reference_length': ref_props.ref_diameter_m,  # Note: reference_length is diameter, not rocket length
            'use_aero_tables': True,
        }
        
        # Actuator defaults in radians; 'units': 'rad' prevents double conversion in rocket_6dof_sim.py
        # Values below are MEASURED from /direct (PC↔CAN↔servo) baseline tests
        # on XQPOWER servos (2026-05-01). YAML-provided values override these.
        actuator_config = {
            'tau_servo': 0.012,               # 12ms — measured /direct 2026-05-01 (was 0.025)
            'delta_max': np.deg2rad(15.0),   # 15 deg -> rad
            'delta_min': np.deg2rad(-15.0),  # -15 deg -> rad
            'rate_max': np.deg2rad(300.0),   # 300 deg/s -> rad/s
            'n_actuators': 4,
            'config_type': 'X',
            'delta_trim': [0.0, 0.0, 0.0, 0.0],
            'delta_scale': [1.0, 1.0, 1.0, 1.0],
            'backlash': np.deg2rad(2.5),     # 2.5° — measured /direct 2026-05-01 (was 0.0)
            'units': 'rad',  # Values are already in radians - prevents double conversion
        }
        
        for stage in multi_stage_data.stages:
            if stage.has_fins and stage.rocket_data.properties.fin:
                fin = stage.rocket_data.properties.fin
                # Convert degrees to radians for ActuatorModel (which expects radians)
                delta_max_deg = fin.get('delta_max_deg', 15.0)
                actuator_config['delta_max'] = np.deg2rad(delta_max_deg)
                actuator_config['delta_min'] = np.deg2rad(-delta_max_deg)
                actuator_config['rate_max'] = np.deg2rad(fin.get('delta_dot_max_deg_s', 300.0))
                break
        
        # Build safety config
        # Note: q_alpha_max is loaded from rocket_properties.yaml (single source of truth)
        # Attitude and rate limits are enforced by SafetyMonitor
        safety_config = {
                        'pitch_max_rad': DEFAULT_PITCH_MAX_RAD,
                        'roll_max_rad': DEFAULT_ROLL_MAX_RAD,
            'rate_max_rad_s': 3.14,   # rad/s (180 deg/s)
        }
        
        # Use safety from first stage if available
        if ref_props.safety:
            safety_config['q_alpha_max'] = ref_props.safety.get('qa_max_Pa_rad', 50000.0)
        
        # Build flight phase config
        flight_phase_config = {
            'burnout_time_s': stage_burnout_times[0] if stage_burnout_times else 0.0,
            'burnout_accel_g': 1.0,
            'burnout_hold_s': 0.2,
            'apogee_vz_threshold': 5.0,
            'apogee_hold_s': 0.5,
            'num_stages': multi_stage_data.num_stages,
            'stage_burnout_times': stage_burnout_times,
            'separation_delay_s': 0.5,
        }
        
        # Calculate total simulation duration
        total_burn_time = cumulative_burn_time
        simulation_duration = max(120.0, total_burn_time * 3)  # At least 3x burn time
        
        config = {
            'simulation': {
                'duration': simulation_duration,
                'dt': 0.01,
                'output_rate': 10.0,
                'integrator': 'rk4',
                'use_advanced_aerodynamics': True,
                'use_mass_properties_model': True,
                'use_advanced_propulsion': True,
                'use_actuator_dynamics': True,
                'use_error_injection': False,
                'multi_stage_enabled': True,
                'num_stages': multi_stage_data.num_stages,
            },
            'stages': stages_config,
            'propulsion': propulsion_config,
            'aerodynamics': aero_config,
            'actuator': actuator_config,
            'safety': safety_config,
            'flight_phase': flight_phase_config,
            'atmosphere': {
                'temperature_sea_level': 288.15,
                'pressure_sea_level': 101325.0,
                'density_sea_level': 1.225,
                'use_advanced_model': True,
            },
            'initial_conditions': {
                'position': [0.0, 0.0, 0.0],
                'velocity': [0.0, 0.0, 0.0],
                'attitude': [0.9239, 0.0, 0.3827, 0.0],  # 45 deg pitch
                'angular_velocity': [0.0, 0.0, 0.0],
            },
        }
        
        return config

    def _load_properties(self, data_dir: Path, rocket_name: str) -> RocketProperties:
        """Load rocket properties from YAML file with comprehensive validation."""
        props_file = data_dir / "rocket_properties.yaml"

        if not props_file.exists():
            raise RocketDataError(
                f"Rocket properties file not found: {props_file}. "
                f"Each rocket must have a rocket_properties.yaml file."
            )

        with open(props_file, 'r', encoding='utf-8') as f:
            props = yaml.safe_load(f)

        # Extract required properties
        try:
            # cg_propellant_body_m: optional, defaults to cg_dry if propellant_mass > 0
            propellant_mass = props['propellant_mass_kg']
            cg_propellant = props.get('cg_propellant_body_m')

            if propellant_mass > 0 and cg_propellant is None:
                cg_propellant = props['cg_dry_body_m']
                logger.warning(
                    f"cg_propellant_body_m not specified for rocket with propellant_mass_kg={propellant_mass}. "
                    f"Using cg_dry_body_m as default. For better accuracy, specify cg_propellant_body_m in YAML. "
                    f"cg_propellant_body_m غير محدد - استخدام cg_dry_body_m كقيمة افتراضية"
                )
            
            rocket_props = RocketProperties(
                mass_dry_kg=props['mass_dry_kg'],
                propellant_mass_kg=propellant_mass,
                cg_dry_body_m=props['cg_dry_body_m'],
                inertia_dry_kgm2=props['inertia_dry_kgm2'],
                ref_diameter_m=props['ref_diameter_m'],
                ref_length_m=props['ref_length_m'],
                ref_area_m2=props['ref_area_m2'],
                cg_propellant_body_m=cg_propellant,  # Optional, may be None for stages without propellant
                nozzle_exit_area_m2=props.get('nozzle_exit_area_m2', 0.0),
                sea_level_pressure_Pa=props.get('sea_level_pressure_Pa', 101325.0),
                inertia_full_kgm2=props.get('inertia_full_kgm2'),  # Optional explicit full inertia
                # Optional propellant geometry for accurate inertia estimation
                propellant_outer_radius_m=props.get('propellant_outer_radius_m'),
                propellant_length_m=props.get('propellant_length_m'),
                propellant_inner_radius_ratio=props.get('propellant_inner_radius_ratio', 0.5),
                fin=props.get('fin', {}),
                actuator=props.get('actuator', {}),
                safety=props.get('safety', {}),
                num_stages=props.get('num_stages', 1),
                servo_torque_max_Nm=props.get('servo_torque_max_Nm', 50.0),
                launcher=props.get('launcher', {}),
                insulation_mass_kg=props.get('insulation_mass_kg', 0.0),
                CA_multiplier=props.get('CA_multiplier', 1.0),
                burn_time_scale=props.get('burn_time_scale', 1.0),
                thrust_multiplier=props.get('thrust_multiplier', 1.0),
                config_type=props.get('config_type', 'X'),
                fin_location=props.get('fin_location', 'tail'),
                moment_reference_point=props.get('moment_reference_point'),
                motor_type=props.get('motor_type', 'solid'),
            )
        except KeyError as e:
            raise RocketDataError(
                f"Missing required property in {props_file}: {e}"
            )

        # Validate physical correctness
        errors = []
        
        # Validate mass properties
        # Dry mass must be positive (rocket structure must have mass)
        if rocket_props.mass_dry_kg <= 0:
            errors.append(
                f"mass_dry_kg must be positive (rocket structure must have mass), "
                f"got {rocket_props.mass_dry_kg}"
            )
        
        if rocket_props.propellant_mass_kg < 0:
            errors.append(f"propellant_mass_kg must be non-negative, got {rocket_props.propellant_mass_kg}")
        
        # Validate geometric properties (positive values)
        if rocket_props.ref_diameter_m <= 0:
            errors.append(f"ref_diameter_m must be positive, got {rocket_props.ref_diameter_m}")
        
        if rocket_props.ref_length_m <= 0:
            errors.append(f"ref_length_m must be positive, got {rocket_props.ref_length_m}")
        
        if rocket_props.ref_area_m2 <= 0:
            errors.append(f"ref_area_m2 must be positive, got {rocket_props.ref_area_m2}")
        
        # Validate ref_area vs geometric area (may differ intentionally for warheads)
        expected_area = np.pi * (rocket_props.ref_diameter_m / 2.0) ** 2
        area_ratio = rocket_props.ref_area_m2 / expected_area if expected_area > 0 else 0

        if not (0.8 <= area_ratio <= 1.2):
            if area_ratio > 5.0 or area_ratio < 0.2:
                logger.warning(
                    f"ref_area_m2 ({rocket_props.ref_area_m2:.6f}) differs significantly from "
                    f"geometric area ~{expected_area:.6f} m² (ratio: {area_ratio:.2f}). "
                    f"May be intentional for aero reference purposes."
                )
            else:
                logger.warning(
                    f"ref_area_m2 ({rocket_props.ref_area_m2:.6f}) inconsistent with ref_diameter_m "
                    f"({rocket_props.ref_diameter_m:.3f}). Expected ~{expected_area:.6f} m² "
                    f"(ratio: {area_ratio:.2f})."
                )
        
        # Validate inertia values (non-negative)
        if len(rocket_props.inertia_dry_kgm2) != 3:
            errors.append(f"inertia_dry_kgm2 must have 3 elements [Ixx, Iyy, Izz], got {len(rocket_props.inertia_dry_kgm2)}")
        else:
            for i, (name, inertia) in enumerate(zip(['Ixx', 'Iyy', 'Izz'], rocket_props.inertia_dry_kgm2)):
                if inertia < 0:
                    errors.append(f"inertia_dry_kgm2[{i}] ({name}) must be non-negative, got {inertia}")
        
        # Validate inertia_full consistency (must be >= inertia_dry)
        # Physics: with propellant added, full inertia should be >= dry inertia
        if rocket_props.inertia_full_kgm2 is not None:
            if len(rocket_props.inertia_full_kgm2) != 3:
                errors.append(
                    f"inertia_full_kgm2 must have 3 elements [Ixx, Iyy, Izz], "
                    f"got {len(rocket_props.inertia_full_kgm2)}"
                )
            elif len(rocket_props.inertia_dry_kgm2) == 3:
                for i, (name, I_full, I_dry) in enumerate(zip(
                    ['Ixx', 'Iyy', 'Izz'],
                    rocket_props.inertia_full_kgm2,
                    rocket_props.inertia_dry_kgm2
                )):
                    if I_full < 0:
                        errors.append(f"inertia_full_kgm2[{i}] ({name}) must be non-negative, got {I_full}")
                    elif I_full < I_dry:
                        # Full inertia should be >= dry inertia when propellant is added
                        # Only warn if propellant mass is significant (> 1% of dry mass)
                        if rocket_props.propellant_mass_kg > 0.01 * rocket_props.mass_dry_kg:
                            errors.append(
                                f"inertia_full_kgm2[{i}] ({name})={I_full:.4f} < inertia_dry_kgm2[{i}]={I_dry:.4f}. "
                                f"Full inertia (with propellant) should be >= dry inertia. "
                                f"Check your inertia values."
                            )
        
        # Validate CG positions (must be within rocket length)
        # Tolerance for y/z offset validation (0.1mm - accounts for floating point and minor asymmetries)
        CG_YZ_TOLERANCE_M = 1e-4
        
        if len(rocket_props.cg_dry_body_m) != 3:
            errors.append(f"cg_dry_body_m must have 3 elements [x, y, z], got {len(rocket_props.cg_dry_body_m)}")
        else:
            cg_x = rocket_props.cg_dry_body_m[0]
            cg_y = rocket_props.cg_dry_body_m[1]
            cg_z = rocket_props.cg_dry_body_m[2]
            if not (0 <= cg_x <= rocket_props.ref_length_m):
                errors.append(
                    f"cg_dry_body_m[0] ({cg_x:.3f}) must be within [0, ref_length_m={rocket_props.ref_length_m:.3f}]"
                )
            # Note: y/z offsets are now fully supported by the 3D parallel axis theorem implementation
        
        # Validate cg_propellant_body_m only if it's provided
        # Note: cg_propellant_body_m is optional when propellant_mass_kg = 0
        if rocket_props.cg_propellant_body_m is not None:
            if len(rocket_props.cg_propellant_body_m) != 3:
                errors.append(f"cg_propellant_body_m must have 3 elements [x, y, z], got {len(rocket_props.cg_propellant_body_m)}")
            else:
                cg_prop_x = rocket_props.cg_propellant_body_m[0]
                cg_prop_y = rocket_props.cg_propellant_body_m[1]
                cg_prop_z = rocket_props.cg_propellant_body_m[2]
                if not (0 <= cg_prop_x <= rocket_props.ref_length_m):
                    errors.append(
                        f"cg_propellant_body_m[0] ({cg_prop_x:.3f}) must be within [0, ref_length_m={rocket_props.ref_length_m:.3f}]"
                    )
                # Note: y/z offsets are now fully supported by the 3D parallel axis theorem implementation
        
        # Validate nozzle properties
        if rocket_props.nozzle_exit_area_m2 < 0:
            errors.append(f"nozzle_exit_area_m2 must be non-negative, got {rocket_props.nozzle_exit_area_m2}")
        
        if rocket_props.sea_level_pressure_Pa <= 0:
            errors.append(f"sea_level_pressure_Pa must be positive, got {rocket_props.sea_level_pressure_Pa}")
        
        # Validate fin properties if present
        if rocket_props.fin:
            fin = rocket_props.fin
            if 'S_fin_m2' in fin and fin['S_fin_m2'] < 0:
                errors.append(f"fin.S_fin_m2 must be non-negative, got {fin['S_fin_m2']}")
            
            if 'c_fin_m' in fin and fin['c_fin_m'] <= 0:
                errors.append(f"fin.c_fin_m must be positive, got {fin['c_fin_m']}")
            
            if 'x_fin_m' in fin:
                x_fin = fin['x_fin_m']
                if not (0 <= x_fin <= rocket_props.ref_length_m):
                    errors.append(
                        f"fin.x_fin_m ({x_fin:.3f}) must be within [0, ref_length_m={rocket_props.ref_length_m:.3f}]"
                    )
            
            if 'delta_max_deg' in fin:
                delta_max = fin['delta_max_deg']
                # Basic validation: must be positive and <= 90 degrees
                if not (0 < delta_max <= 90):
                    errors.append(f"fin.delta_max_deg must be in (0, 90], got {delta_max}")
                # Unit consistency check: warn if value seems to be in radians
                elif delta_max < 1.0:
                    logger.warning(
                        f"fin.delta_max_deg = {delta_max:.4f} is very small. "
                        f"Verify this is in degrees, not radians. "
                        f"(Value in radians would be ~{np.rad2deg(delta_max):.2f} degrees)"
                    )
                # Reasonableness check for typical rocket fins
                elif delta_max > 45:
                    logger.warning(
                        f"fin.delta_max_deg = {delta_max:.1f}° is unusually large for rocket fins. "
                        f"Typical values are 10-30 degrees. Verify this is correct."
                    )
            
            if 'delta_dot_max_deg_s' in fin:
                delta_dot = fin['delta_dot_max_deg_s']
                if delta_dot <= 0:
                    errors.append(f"fin.delta_dot_max_deg_s must be positive, got {delta_dot}")
                # Unit consistency check
                elif delta_dot < 1.0:
                    logger.warning(
                        f"fin.delta_dot_max_deg_s = {delta_dot:.4f} is very small. "
                        f"Verify this is in deg/s, not rad/s. "
                        f"(Value in rad/s would be ~{np.rad2deg(delta_dot):.2f} deg/s)"
                    )
                # Reasonableness check for servo capabilities
                elif delta_dot > 1000:
                    logger.warning(
                        f"fin.delta_dot_max_deg_s = {delta_dot:.1f} deg/s is extremely high. "
                        f"Typical servo rates are 100-500 deg/s. Verify this is correct."
                    )
        
        # Validate safety properties if present
        if rocket_props.safety:
            safety = rocket_props.safety
            if 'qa_max_Pa_rad' in safety and safety['qa_max_Pa_rad'] <= 0:
                errors.append(f"safety.qa_max_Pa_rad must be positive, got {safety['qa_max_Pa_rad']}")
        
        # Validate other properties
        if rocket_props.num_stages < 1:
            errors.append(f"num_stages must be at least 1, got {rocket_props.num_stages}")
        
        if rocket_props.servo_torque_max_Nm < 0:
            errors.append(f"servo_torque_max_Nm must be non-negative, got {rocket_props.servo_torque_max_Nm}")
        
        # Validate burn_time_scale (must be positive)
        if rocket_props.burn_time_scale <= 0:
            errors.append(f"burn_time_scale must be positive, got {rocket_props.burn_time_scale}")
        
        # Validate thrust_multiplier (must be positive)
        if rocket_props.thrust_multiplier <= 0:
            errors.append(f"thrust_multiplier must be positive, got {rocket_props.thrust_multiplier}")
        
        # Validate config_type (must be "X" or "+")
        allowed_config_types = ('X', '+')
        if rocket_props.config_type not in allowed_config_types:
            errors.append(f"config_type must be one of {allowed_config_types}, got '{rocket_props.config_type}'")
        
        # Validate fin_location (must be "canard" or "tail")
        allowed_fin_locations = ('canard', 'tail')
        if rocket_props.fin_location not in allowed_fin_locations:
            errors.append(f"fin_location must be one of {allowed_fin_locations}, got '{rocket_props.fin_location}'")
        
        # Raise error if any validation failed
        if errors:
            error_msg = f"Invalid rocket properties in {props_file}:\n  - " + "\n  - ".join(errors)
            raise RocketDataError(error_msg)
        
        return rocket_props

    def _load_aero_tables(self, data_dir: Path) -> AeroTables:
        """Load all aerodynamic coefficient tables.
        
        Loads CSV files for aerodynamic coefficients. If a file exists but cannot
        be read (corrupted/malformed), an error is raised rather than silently
        continuing with incomplete data.
        """
        # Define table files mapping
        # Maps attribute name -> filename
        table_files = {
            'aero_coeffs': 'aero_coeffs.csv',
            'fin_coeffs': 'fin_coeffs.csv',
            'damping_coeffs': 'damping_coeffs.csv',
            'fin_deflection_drag': 'fin_deflection_drag.csv',
            'ca_3d_coeffs': 'ca_3d_coeffs.csv',
            'roll_aero_coeffs': 'roll_aero_coeffs.csv',
            'fin_loads': 'fin_loads.csv',
            # New tables for enhanced simulation
            'ca_3d_coeffs_motor_on': 'ca_3d_coeffs_motor_on.csv',
            'ca_3d_coeffs_motor_off': 'ca_3d_coeffs_motor_off.csv',
            'fin_deflection_coeffs': 'fin_deflection_coeffs.csv',
        }

        # Load tables with explicit assignment (better for type-checkers)
        loaded_tables: Dict[str, pd.DataFrame] = {}
        load_errors: List[str] = []
        
        for attr, filename in table_files.items():
            filepath = data_dir / filename
            if filepath.exists():
                try:
                    df = pd.read_csv(filepath, comment='#')
                    loaded_tables[attr] = df
                    logger.info(f"Loaded {filename}: {len(df)} rows")
                except Exception as e:
                    # Collect errors for files that exist but failed to load
                    # This indicates corrupted/malformed data that should not be ignored
                    load_errors.append(f"{filename}: {e}")
        
        # Fail-fast: raise error if any existing files failed to load
        if load_errors:
            raise RocketDataError(
                f"Failed to load aerodynamic tables from {data_dir}. "
                f"The following files exist but could not be read:\n  - " +
                "\n  - ".join(load_errors)
            )
        
        
        # Required columns per table type: {table: {standard_name: [aliases]}}
        # Cd ≈ Ca at small alpha; Ca preferred if both exist (more explicit for axial force)
        required_columns = {
            'aero_coeffs': {
                'Mach': ['Mach', 'mach', 'MACH'],
                'alpha_deg': ['alpha_deg', 'Alpha', 'alpha', 'ALPHA'],
                # Ca/CA takes precedence over Cd/CD if both exist (Ca is more explicit for axial force)
                'Cd': ['Ca', 'CA', 'ca', 'Cd', 'CD', 'cd'],
                'Cn': ['Cn', 'CN', 'cn'],
                'Cm': ['Cm', 'CM', 'cm'],
            },
            'fin_coeffs': {
                'Mach': ['Mach', 'mach', 'MACH'],
                'alpha_deg': ['alpha_deg', 'Alpha', 'alpha', 'ALPHA'],
                'Cn_delta': ['Cn_delta', 'CN_delta', 'cn_delta'],
                'Cm_delta': ['Cm_delta', 'CM_delta', 'cm_delta'],
            },
            'damping_coeffs': {
                'Mach': ['Mach', 'mach', 'MACH'],
                'alpha_deg': ['alpha_deg', 'Alpha', 'alpha', 'ALPHA'],
                'Cmq': ['Cmq', 'CMQ', 'cmq'],
                'Cnq': ['Cnq', 'CNQ', 'cnq'],
                'Clp': ['Clp', 'CLP', 'clp'],
            },
            'fin_deflection_drag': {
                'Mach': ['Mach', 'mach', 'MACH'],
                'delta_deg': ['delta_deg', 'Delta', 'delta', 'DELTA', 'def_deg'],
                'Cd_delta': ['Cd_delta', 'CD_delta', 'cd_delta'],
            },
            'ca_3d_coeffs': {
                'Mach': ['Mach', 'mach', 'MACH'],
                'alpha_deg': ['alpha_deg', 'Alpha', 'alpha', 'ALPHA'],
                'CA': ['CA', 'Ca', 'ca'],
            },
            # New tables for motor_on/motor_off CA
            'ca_3d_coeffs_motor_on': {
                'Mach': ['Mach', 'mach', 'MACH'],
                'alpha_deg': ['alpha_deg', 'Alpha', 'alpha', 'ALPHA'],
                'CA': ['CA', 'Ca', 'ca'],
            },
            'ca_3d_coeffs_motor_off': {
                'Mach': ['Mach', 'mach', 'MACH'],
                'alpha_deg': ['alpha_deg', 'Alpha', 'alpha', 'ALPHA'],
                'CA': ['CA', 'Ca', 'ca'],
            },
            # Fin deflection coeffs: 3D (Mach,alpha,delta), 2D (Mach,delta), or 2D (Mach,alpha per-rad)
            'fin_deflection_coeffs': {
                'Mach': ['Mach', 'mach', 'MACH'],
                'CNd': ['CNd', 'Cnd', 'cnd', 'CN_d', 'Cn_d', 'CN_delta', 'Cn_delta'],
                'CMd': ['CMd', 'Cmd', 'cmd', 'CM_d', 'Cm_d', 'CM_delta', 'Cm_delta'],
            },
        }
        
        # Normalize column names to standard names
        # This ensures downstream code (e.g., aerodynamics.py) can use standard names
        for attr, df in loaded_tables.items():
            if attr in required_columns:
                rename_map = {}
                for standard_name, aliases in required_columns[attr].items():
                    # Find which alias is present in the dataframe
                    for alias in aliases:
                        if alias in df.columns and alias != standard_name:
                            rename_map[alias] = standard_name
                            break
                
                if rename_map:
                    loaded_tables[attr] = df.rename(columns=rename_map)
                    logger.info(
                        f"{table_files[attr]}: Normalized column names: {rename_map}. "
                        f"تم توحيد أسماء الأعمدة"
                    )
        
        # Optional columns that generate warnings if missing
        optional_columns = {
            'fin_coeffs': {
                'Cl_delta': ['Cl_delta', 'CL_delta', 'cl_delta'],
                'Ch_delta': ['Ch_delta', 'CH_delta', 'ch_delta'],
            },
            'ca_3d_coeffs': {
                'Reynolds': ['Reynolds', 'Re', 'reynolds'],
            },
            'ca_3d_coeffs_motor_on': {
                'Reynolds': ['Reynolds', 'Re', 'reynolds'],
            },
            'ca_3d_coeffs_motor_off': {
                'Reynolds': ['Reynolds', 'Re', 'reynolds'],
            },
        }
        
        # Validate loaded tables for minimum data requirements
        validation_errors = []
        for attr, df in loaded_tables.items():
            # Check minimum rows (need at least 2 for interpolation, warn if < 5)
            if len(df) < 2:
                validation_errors.append(
                    f"{table_files[attr]}: Must have at least 2 data rows for interpolation, got {len(df)}"
                )
            elif len(df) < 5:
                logger.warning(
                    f"{table_files[attr]}: Only {len(df)} data rows. "
                    f"Recommend at least 5 rows for accurate interpolation."
                )
            
            # Check for empty dataframes
            if df.empty:
                validation_errors.append(f"{table_files[attr]}: DataFrame is empty (no data)")
            
            # Check for NaN/inf values in numeric columns
            numeric_cols = df.select_dtypes(include=[np.number]).columns
            for col in numeric_cols:
                nan_count = df[col].isna().sum()
                inf_count = np.isinf(df[col].values).sum() if np.issubdtype(df[col].dtype, np.floating) else 0
                
                if nan_count > 0:
                    validation_errors.append(
                        f"{table_files[attr]}: Column '{col}' contains {nan_count} NaN values. "
                        f"NaN values will cause interpolation failures. "
                        f"العمود '{col}' يحتوي على {nan_count} قيم NaN"
                    )
                if inf_count > 0:
                    validation_errors.append(
                        f"{table_files[attr]}: Column '{col}' contains {inf_count} infinite values. "
                        f"Infinite values are not physically meaningful. "
                        f"العمود '{col}' يحتوي على {inf_count} قيم لانهائية"
                    )
            
            # Validate required columns for known table types
            if attr in required_columns:
                df_columns = set(df.columns)
                missing_required = []
                
                for col_name, aliases in required_columns[attr].items():
                    # Check if any alias is present in the dataframe
                    found = any(alias in df_columns for alias in aliases)
                    if not found:
                        missing_required.append(f"{col_name} (aliases: {aliases})")
                
                if missing_required:
                    validation_errors.append(
                        f"{table_files[attr]}: Missing required columns: {', '.join(missing_required)}. "
                        f"Available columns: {list(df.columns)}"
                    )
            
            # Warn about missing optional columns
            if attr in optional_columns:
                df_columns = set(df.columns)
                for col_name, aliases in optional_columns[attr].items():
                    found = any(alias in df_columns for alias in aliases)
                    if not found:
                        logger.warning(
                            f"{table_files[attr]}: Optional column '{col_name}' not found. "
                            f"This may affect some calculations. "
                            f"العمود الاختياري '{col_name}' غير موجود - قد يؤثر على بعض الحسابات"
                        )
            
            # Check for negative Mach values (physically impossible)
            mach_col = None
            for alias in ['Mach', 'mach', 'MACH']:
                if alias in df.columns:
                    mach_col = alias
                    break
            
            if mach_col is not None:
                mach_values = df[mach_col].values
                if np.any(mach_values < 0):
                    neg_count = np.sum(mach_values < 0)
                    validation_errors.append(
                        f"{table_files[attr]}: Mach column contains {neg_count} negative values. "
                        f"Mach number must be non-negative (physical constraint). "
                        f"عمود Mach يحتوي على {neg_count} قيم سالبة - رقم ماخ يجب أن يكون غير سالب"
                    )
            
            # Validate alpha_deg values are in degrees (not radians)
            alpha_col = None
            for alias in ['alpha_deg', 'Alpha', 'alpha', 'ALPHA']:
                if alias in df.columns:
                    alpha_col = alias
                    break
            
            if alpha_col is not None:
                alpha_values = df[alpha_col].values
                alpha_min = np.min(alpha_values)
                alpha_max = np.max(alpha_values)
                
                # Warn if alpha seems in radians (max |alpha| < 1.5 with multiple unique values)
                unique_alpha = np.unique(alpha_values)
                is_single_point = len(unique_alpha) == 1
                if abs(alpha_max) < 1.5 and abs(alpha_min) < 1.5 and len(alpha_values) > 1 and not is_single_point:
                    logger.warning(
                        f"{table_files[attr]}: alpha_deg values appear to be in radians "
                        f"(range: [{alpha_min:.4f}, {alpha_max:.4f}]). "
                        f"Expected values in degrees (typical range: -30 to +30)."
                    )

                # Large alpha values (>90°) are valid for warhead reentry scenarios
        
        # Raise error if validation failed
        if validation_errors:
            raise RocketDataError(
                f"Invalid aerodynamic table data in {data_dir}:\n  - " +
                "\n  - ".join(validation_errors)
            )

        # Physical sanity checks (warnings only — never stops simulation)
        self._validate_aero_physics(loaded_tables, table_files)

        # Create AeroTables with explicit attribute assignment
        tables = AeroTables(
            aero_coeffs=loaded_tables.get('aero_coeffs'),
            fin_coeffs=loaded_tables.get('fin_coeffs'),
            damping_coeffs=loaded_tables.get('damping_coeffs'),
            fin_deflection_drag=loaded_tables.get('fin_deflection_drag'),
            ca_3d_coeffs=loaded_tables.get('ca_3d_coeffs'),
            roll_aero_coeffs=loaded_tables.get('roll_aero_coeffs'),
            fin_loads=loaded_tables.get('fin_loads'),
            # New tables for enhanced simulation
            ca_3d_coeffs_motor_on=loaded_tables.get('ca_3d_coeffs_motor_on'),
            ca_3d_coeffs_motor_off=loaded_tables.get('ca_3d_coeffs_motor_off'),
            fin_deflection_coeffs=loaded_tables.get('fin_deflection_coeffs'),
        )
        
        # Log information about CA table availability and precedence
        has_motor_pair = (tables.ca_3d_coeffs_motor_on is not None and 
                         tables.ca_3d_coeffs_motor_off is not None)
        has_legacy_ca = tables.ca_3d_coeffs is not None
        
        if has_motor_pair:
            logger.info(
                "CA tables: Using motor_on/motor_off pair (CA_motor_multiplier will NOT be applied). "
                "جداول CA: استخدام زوج motor_on/motor_off (لن يتم تطبيق CA_motor_multiplier)"
            )
        elif has_legacy_ca:
            logger.info(
                "CA tables: Using legacy ca_3d_coeffs (CA_motor_multiplier WILL be applied during motor burn). "
                "جداول CA: استخدام ca_3d_coeffs القديم (سيتم تطبيق CA_motor_multiplier أثناء الدفع)"
            )
        elif tables.ca_3d_coeffs_motor_on is not None:
            logger.warning(
                "CA tables: Only motor_on table found (motor_off missing). "
                "Will use motor_on for both states with CA_motor_multiplier for motor_on. "
                "جداول CA: تم العثور على جدول motor_on فقط (motor_off مفقود)"
            )
        elif tables.ca_3d_coeffs_motor_off is not None:
            logger.warning(
                "CA tables: Only motor_off table found (motor_on missing). "
                "Will use motor_off for both states with CA_motor_multiplier for motor_on. "
                "جداول CA: تم العثور على جدول motor_off فقط (motor_on مفقود)"
            )
        
        # Log information about fin deflection coefficients
        if tables.fin_deflection_coeffs is not None and tables.fin_coeffs is None:
            logger.info(
                "Fin effectiveness: Using fin_deflection_coeffs (CNd, CMd). "
                "WARNING: Units assumed to be per-radian. If per-degree, results will be incorrect. "
                "فعالية الزعانف: استخدام fin_deflection_coeffs - تحذير: الوحدات مفترضة per-radian"
            )
        elif tables.fin_coeffs is not None:
            logger.info(
                "Fin effectiveness: Using fin_coeffs (Cn_delta, Cm_delta per-radian). "
                "فعالية الزعانف: استخدام fin_coeffs"
            )

        if not loaded_tables:
            raise RocketDataError(
                f"No aerodynamic tables found in {data_dir}. "
                f"At least one of the following files is required: {list(table_files.values())}"
            )

        # Validate that we have minimum required tables
        if tables.aero_coeffs is None and tables.fin_coeffs is None:
            raise RocketDataError(
                f"Missing required aerodynamic data in {data_dir}. "
                f"At least aero_coeffs.csv or fin_coeffs.csv is required."
            )

        return tables

    def _validate_aero_physics(self, loaded_tables: dict, table_files: dict) -> None:
        """
        Physical sanity checks on aerodynamic coefficient tables.

        Detects: non-monotonic Cd, spikes/dips, suspicious duplicate values,
        and cross-Mach copy-paste errors. Issues warnings only — never raises.

        فحوصات الصحة الفيزيائية لجداول المعاملات الايروديناميكية.
        تكتشف: عدم الرتابة، القفزات الشاذة، التكرار المشبوه، وأخطاء النسخ بين أرقام Mach.
        تصدر تحذيرات فقط — لا توقف المحاكاة.
        """
        warnings_list = []

        # Which tables/columns to check
        check_targets = {
            'aero_coeffs': {
                'ind': 'alpha_deg', 'mach': 'Mach',
                'monotone_cols': ['Cd'],
                'all_cols': ['Cd', 'Cn', 'Cm'],
                'cross_mach_col': 'Cd',
            },
            'fin_deflection_coeffs': {
                'ind': 'delta_deg', 'mach': 'Mach',
                'monotone_cols': [],
                'all_cols': ['CNd', 'CMd'],
                'cross_mach_col': None,
            },
            'roll_aero_coeffs': {
                'ind': 'alpha_deg', 'mach': 'Mach',
                'monotone_cols': [],
                'all_cols': ['Cll'],
                'cross_mach_col': None,
            },
        }

        MONO_THRESHOLD = -0.002
        SPIKE_THRESHOLD = 0.4
        DUP_VALUE_TOL = 1e-8
        DUP_ANGLE_GAP = 2.0
        CROSS_MACH_LOW_RATIO = 0.85
        CROSS_MACH_COPY_TOL = 0.02

        for table_key, cfg in check_targets.items():
            df = loaded_tables.get(table_key)
            if df is None:
                continue
            filename = table_files.get(table_key, table_key)
            ind_col = cfg['ind']
            mach_col = cfg['mach']

            if ind_col not in df.columns or mach_col not in df.columns:
                continue

            # Filter to columns that actually exist in this table
            coeff_cols = [c for c in cfg['all_cols'] if c in df.columns]
            mono_cols = [c for c in cfg['monotone_cols'] if c in df.columns]

            mach_values = sorted(df[mach_col].unique())

            for mach_val in mach_values:
                sl = df[df[mach_col] == mach_val].sort_values(ind_col)
                angles = sl[ind_col].values

                if len(angles) < 3:
                    continue

                # --- Check A: Monotonicity of Cd vs angle ---
                for col in mono_cols:
                    vals = sl[col].values
                    diffs = np.diff(vals)
                    for j, d in enumerate(diffs):
                        if d < MONO_THRESHOLD:
                            warnings_list.append(
                                f"[Monotonicity] {filename}: {col} decreases at Mach={mach_val}, "
                                f"{ind_col} {angles[j]:.0f}°→{angles[j+1]:.0f}° "
                                f"({vals[j]:.4f}→{vals[j+1]:.4f}, diff={d:+.4f})"
                            )

                # --- Check B: Spike detection (second difference) ---
                for col in coeff_cols:
                    vals = sl[col].values
                    val_range = vals.max() - vals.min()
                    if val_range < 1e-10:
                        continue
                    for j in range(1, len(vals) - 1):
                        d2 = abs(vals[j + 1] - 2.0 * vals[j] + vals[j - 1])
                        normalized = d2 / val_range
                        if normalized > SPIKE_THRESHOLD:
                            warnings_list.append(
                                f"[Spike] {filename}: {col} spike at Mach={mach_val}, "
                                f"{ind_col}={angles[j]:.0f}° (value={vals[j]:.4f}, "
                                f"neighbors={vals[j-1]:.4f},{vals[j+1]:.4f}, "
                                f"normalized_d2={normalized:.2f})"
                            )

                # --- Check C: Suspicious duplicate values ---
                for col in coeff_cols:
                    vals = sl[col].values
                    for j in range(len(vals) - 1):
                        if (abs(vals[j] - vals[j + 1]) < DUP_VALUE_TOL
                                and abs(angles[j + 1] - angles[j]) >= DUP_ANGLE_GAP):
                            warnings_list.append(
                                f"[Duplicate] {filename}: {col} identical at Mach={mach_val}, "
                                f"{ind_col}={angles[j]:.0f}° and {angles[j+1]:.0f}° "
                                f"(value={vals[j]:.4f})"
                            )

            # --- Check D: Cross-Mach consistency for Cd ---
            cross_col = cfg.get('cross_mach_col')
            if cross_col and cross_col in df.columns and len(mach_values) >= 2:
                for i in range(len(mach_values) - 1):
                    m_lo, m_hi = mach_values[i], mach_values[i + 1]
                    # Only subsonic
                    if m_hi >= 0.9:
                        continue
                    mach_gap = m_hi - m_lo
                    if mach_gap < 0.05:
                        continue

                    sl_lo = df[df[mach_col] == m_lo].set_index(ind_col)[cross_col]
                    sl_hi = df[df[mach_col] == m_hi].set_index(ind_col)[cross_col]
                    common_angles = sorted(set(sl_lo.index) & set(sl_hi.index))

                    for ang in common_angles:
                        cd_lo = sl_lo.loc[ang]
                        cd_hi = sl_hi.loc[ang]
                        if abs(cd_lo) < 1e-10:
                            continue
                        ratio = cd_hi / cd_lo

                        if abs(ratio - 1.0) < CROSS_MACH_COPY_TOL and mach_gap > 0.1:
                            warnings_list.append(
                                f"[Cross-Mach] {filename}: {cross_col} nearly identical at "
                                f"{ind_col}={ang:.0f}° (Mach={m_lo}: {cd_lo:.4f}, "
                                f"Mach={m_hi}: {cd_hi:.4f}, ratio={ratio:.3f}) — possible copy error"
                            )
                        elif ratio < CROSS_MACH_LOW_RATIO:
                            warnings_list.append(
                                f"[Cross-Mach] {filename}: {cross_col} lower at higher Mach, "
                                f"{ind_col}={ang:.0f}° (Mach={m_lo}: {cd_lo:.4f}, "
                                f"Mach={m_hi}: {cd_hi:.4f}, ratio={ratio:.3f})"
                            )

        if warnings_list:
            detail = "\n  - ".join(warnings_list[:15])
            msg = (
                "AERO DATA PHYSICS CHECK: Potential data errors detected in CFD tables.\n"
                "  These may indicate copy-paste errors in coefficient files.\n"
                "  فحص الصحة الفيزيائية: أخطاء محتملة في جداول CFD — قد تكون أخطاء نسخ.\n"
                f"  Review flagged entries ({len(warnings_list)} total):\n"
                f"  - {detail}"
            )
            if len(warnings_list) > 15:
                msg += f"\n  ... and {len(warnings_list) - 15} more warnings"
            logger.warning(msg)

    def _load_thrust_curve(self, data_dir: Path) -> Optional[ThrustCurve]:
        """Load thrust curve from CSV file. Validates before creating ThrustCurve for better error context."""
        thrust_file = data_dir / "thrust_curve.csv"

        if not thrust_file.exists():
            logger.info(f"Thrust curve not found: {thrust_file} (optional file)")
            return None

        try:
            df = pd.read_csv(thrust_file, comment='#')

            # Handle different column naming conventions
            time_col = None
            thrust_col = None
            mass_flow_col = None

            for col in df.columns:
                col_lower = col.lower().strip()
                if col_lower in ['time', 't', 'time_s', 'time(s)']:
                    time_col = col
                elif col_lower in ['thrust', 'thrust_n', 'thrust(n)', 'f']:
                    thrust_col = col
                elif col_lower in ['mass_flow', 'mdot', 'mass_flow_kg_s', 'mdot(kg/s)']:
                    mass_flow_col = col

            if time_col is None or thrust_col is None:
                # Try positional columns
                if len(df.columns) >= 2:
                    time_col = df.columns[0]
                    thrust_col = df.columns[1]
                    if len(df.columns) >= 3:
                        mass_flow_col = df.columns[2]
                else:
                    raise RocketDataError(
                        f"Invalid thrust curve format in {thrust_file}. "
                        f"Expected columns: time, thrust, [mass_flow]"
                    )

            time = df[time_col].values
            thrust = df[thrust_col].values
            mass_flow = df[mass_flow_col].values if mass_flow_col else None

            # Validate thrust curve data
            errors = []
            
            # Check minimum data points (need at least 2 points for interpolation)
            if len(time) < 2:
                errors.append(f"Thrust curve must have at least 2 data points, got {len(time)}")
            
            # Check time is monotonically increasing and starts at 0 or positive
            if len(time) >= 2:
                if time[0] < 0:
                    errors.append(f"Thrust curve time must start at 0 or positive, got {time[0]}")
                
                for i in range(1, len(time)):
                    if time[i] <= time[i-1]:
                        errors.append(
                            f"Thrust curve time must be strictly increasing, "
                            f"but time[{i}]={time[i]} <= time[{i-1}]={time[i-1]}"
                        )
                        break
            
            # Check thrust is non-negative
            if np.any(thrust < 0):
                neg_indices = np.where(thrust < 0)[0]
                errors.append(
                    f"Thrust values must be non-negative, but found {len(neg_indices)} negative values "
                    f"(e.g., thrust[{neg_indices[0]}]={thrust[neg_indices[0]]})"
                )
            
            # Check mass flow is non-negative if provided
            if mass_flow is not None and np.any(mass_flow < 0):
                neg_indices = np.where(mass_flow < 0)[0]
                errors.append(
                    f"Mass flow values must be non-negative, but found {len(neg_indices)} negative values "
                    f"(e.g., mass_flow[{neg_indices[0]}]={mass_flow[neg_indices[0]]})"
                )
            
            # Raise error if validation failed
            if errors:
                error_msg = f"Invalid thrust curve data in {thrust_file}:\n  - " + "\n  - ".join(errors)
                raise RocketDataError(error_msg)

            return ThrustCurve(time=time, thrust=thrust, mass_flow=mass_flow)

        except pd.errors.ParserError as e:
            raise RocketDataError(f"Failed to parse thrust curve CSV file {thrust_file}: {e}")
        except pd.errors.EmptyDataError:
            raise RocketDataError(f"Thrust curve file {thrust_file} is empty")
        except KeyError as e:
            raise RocketDataError(f"Missing required column in thrust curve file {thrust_file}: {e}")
        except Exception as e:
            raise RocketDataError(f"Failed to load thrust curve from {thrust_file}: {e}")

    def _load_atmosphere_table(self, data_dir: Path) -> Optional[pd.DataFrame]:
        """Load atmosphere table if available. Raises error if file exists but is malformed."""
        atm_file = data_dir / "atmosphere_table.csv"

        if not atm_file.exists():
            return None

        try:
            df = pd.read_csv(atm_file, comment='#')
            logger.info(f"Loaded atmosphere table from {atm_file}: {len(df)} rows")
            return df
        except (pd.errors.ParserError, pd.errors.EmptyDataError, UnicodeDecodeError) as e:
            # Fail-fast: file exists but is corrupted/malformed
            raise RocketDataError(
                f"Failed to load atmosphere table from {atm_file}: {e}. "
                f"The file exists but could not be parsed. "
                f"Fix the file or remove it to use default atmosphere model. "
                f"فشل تحميل جدول الغلاف الجوي - الملف موجود لكن لا يمكن تحليله"
            )
        except Exception as e:
            # Fail-fast: unexpected error
            raise RocketDataError(
                f"Unexpected error loading atmosphere table from {atm_file}: {e}. "
                f"Fix the file or remove it to use default atmosphere model."
            )

    def build_simulation_config(self, rocket_data: RocketData) -> Dict[str, Any]:
        """Build a 6DOF simulation config dict from loaded rocket data."""
        props = rocket_data.properties

        # Calculate derived properties
        mass_total = props.mass_dry_kg + props.propellant_mass_kg

        # Calculate initial CG (weighted average)
        # Guard against division by zero when mass_total <= 0
        cg_dry = np.array(props.cg_dry_body_m)
        cg_prop = np.array(props.get_cg_propellant())  # Use helper method for optional field
        if mass_total > 0:
            cg_initial = (props.mass_dry_kg * cg_dry + props.propellant_mass_kg * cg_prop) / mass_total
        else:
            logger.warning(
                f"Rocket '{rocket_data.name}': mass_total={mass_total} <= 0. "
                f"Using cg_dry as fallback for cg_initial."
            )
            cg_initial = cg_dry

        # Estimate initial inertia using helper function
        inertia_dry = np.array(props.inertia_dry_kgm2)
        inertia_initial = np.array(estimate_full_inertia(props, identifier=f"Rocket '{rocket_data.name}'"))

        # Build rocket config
        rocket_config = {
            'mass_dry': props.mass_dry_kg,
            'mass_propellant': props.propellant_mass_kg,
            'Ixx_0': inertia_initial[0],
            'Iyy_0': inertia_initial[1],
            'Izz_0': inertia_initial[2],
            'Ixx_end': inertia_dry[0],
            'Iyy_end': inertia_dry[1],
            'Izz_end': inertia_dry[2],
            'xcg_0': cg_initial[0],
            'xcg_end': cg_dry[0],
            'length': props.ref_length_m,
            'diameter': props.ref_diameter_m,
            'reference_area': props.ref_area_m2,
        }

        # Build propulsion config
        # Only set motor_type if thrust_curve is available to avoid PropulsionDataError
        propulsion_config = {
            'propellant_mass': props.propellant_mass_kg,
            'burn_time_scale': props.burn_time_scale,
        }

        if rocket_data.thrust_curve:
            tc = rocket_data.thrust_curve
            # Set motor_type only when thrust_curve is available
            propulsion_config['motor_type'] = 'solid'
            # Build thrust curve list
            thrust_curve_list = [[float(t), float(f)] for t, f in zip(tc.time, tc.thrust)]
            propulsion_config['thrust_curve'] = thrust_curve_list
            # Use threshold-based burn_time calculation (consistent with propulsion.py)
            propulsion_config['burn_time'] = tc.calculate_burn_time()
            
            # Add mass flow curve if available for accurate propellant tracking
            if tc.mass_flow is not None:
                mass_flow_list = [float(m) for m in tc.mass_flow]
                propulsion_config['mass_flow_curve'] = mass_flow_list
            
            # Altitude correction parameters (used by PropulsionModel)
            # NOTE: Actual thrust correction physics in dynamics/propulsion.py
            if props.nozzle_exit_area_m2 > 0:
                propulsion_config['nozzle_exit_area'] = props.nozzle_exit_area_m2
                propulsion_config['reference_pressure'] = props.sea_level_pressure_Pa
                logger.info(
                    f"Altitude correction enabled for '{rocket_data.name}' "
                    f"(nozzle_exit_area={props.nozzle_exit_area_m2:.6f} m²)"
                )
            else:
                if props.propellant_mass_kg > 100:
                    logger.warning(
                        f"Rocket '{rocket_data.name}': nozzle_exit_area not specified. "
                        f"For accurate high-altitude performance, specify nozzle_exit_area_m2 in YAML."
                    )
                propulsion_config['reference_pressure'] = props.sea_level_pressure_Pa

        # Build aerodynamics config
        # Note: Center of pressure (CP) is implicitly encoded in the Cm coefficient tables
        aero_config = {
            'reference_area': props.ref_area_m2,
            'reference_length': props.ref_diameter_m,  # Note: reference_length is diameter, not rocket length
            'use_aero_tables': True,
        }

        # Add fin properties if available
        if props.fin:
            fin = props.fin
            aero_config['fin_area'] = fin.get('S_fin_m2', 0.06)
            aero_config['fin_chord'] = fin.get('c_fin_m', 0.2)
            aero_config['fin_position_x'] = fin.get('x_fin_m', 4.0)

        # Build actuator config
        # IMPORTANT: 'units': 'rad' prevents double conversion in rocket_6dof_sim.py
        actuator_config = {
            'n_actuators': 4,
            'config_type': 'X',
            'units': 'rad',  # Values are already in radians - prevents double conversion
        }

        if props.fin:
            fin = props.fin
            # Convert degrees to radians for ActuatorModel (which expects radians)
            delta_max_deg = fin.get('delta_max_deg', 20.0)
            actuator_config['delta_max'] = np.deg2rad(delta_max_deg)
            actuator_config['delta_min'] = np.deg2rad(-delta_max_deg)
            actuator_config['rate_max'] = np.deg2rad(fin.get('delta_dot_max_deg_s', 300.0))
            # Default matches SOLVER_TAU_SERVO_S (0.025s) baked into the
            # acados MPC solver. YAML's actuator.tau_servo (if present)
            # overrides this via _apply_rocket_properties() downstream.
            actuator_config['tau_servo'] = 0.025

        # Build safety config
        # Note: q_alpha_max is loaded from rocket_properties.yaml (single source of truth)
        # Attitude and rate limits are enforced by SafetyMonitor
        safety_config = {
                        'pitch_max_rad': DEFAULT_PITCH_MAX_RAD,
                        'roll_max_rad': DEFAULT_ROLL_MAX_RAD,
            'rate_max_rad_s': 3.14,   # rad/s (180 deg/s)
        }

        if props.safety:
            safety_config['q_alpha_max'] = props.safety.get('qa_max_Pa_rad', 50000.0)

        # Complete config
        config = {
            'rocket': rocket_config,
            'propulsion': propulsion_config,
            'aerodynamics': aero_config,
            'actuator': actuator_config,
            'safety': safety_config,
            'simulation': {
                'duration': 30.0,
                'dt': 0.01,
                'output_rate': 10.0,
                'integrator': 'rk4',
                'use_advanced_aerodynamics': True,
                'use_mass_properties_model': True,
                'use_advanced_propulsion': True,
                'use_actuator_dynamics': True,
            },
            'atmosphere': {
                'temperature_sea_level': 288.15,
                'pressure_sea_level': 101325.0,
                'density_sea_level': 1.225,
                'use_advanced_model': True,
            },
            'initial_conditions': {
                'position': [0.0, 0.0, 0.0],
                'velocity': [0.0, 0.0, 0.0],
                'attitude': [0.9239, 0.0, 0.3827, 0.0],  # 45 deg pitch
                'angular_velocity': [0.0, 0.0, 0.0],
            },
        }

        return config


def load_rocket(rocket_name: str, stage: Optional[str] = None,
                base_dir: str = "data/rocket_models") -> Tuple[RocketData, Dict[str, Any]]:
    """Load rocket data and build simulation config. Returns (RocketData, config_dict)."""
    loader = RocketDataLoader(base_dir)
    rocket_data = loader.load(rocket_name, stage)
    config = loader.build_simulation_config(rocket_data)
    return rocket_data, config


if __name__ == "__main__":
    # Example usage
    import sys

    logging.basicConfig(level=logging.INFO)

    loader = RocketDataLoader()

    print("Available rockets:")
    for rocket in loader.list_available_rockets():
        stages = loader.list_stages(rocket)
        if not stages:
            print(f"  - {rocket} (single-stage)")
        else:
            print(f"  - {rocket} (stages: {stages})")

    # Load example rocket
    if len(sys.argv) > 1:
        rocket_name = sys.argv[1]
        stage = sys.argv[2] if len(sys.argv) > 2 else None
    else:
        rocket_name = "rocket_example"
        stage = None

    print(f"\nLoading {rocket_name}" + (f" stage {stage}" if stage else "") + "...")

    try:
        rocket_data, config = load_rocket(rocket_name, stage)
        print("Loaded successfully!")
        print(f"  Mass (dry): {rocket_data.properties.mass_dry_kg} kg")
        print(f"  Propellant: {rocket_data.properties.propellant_mass_kg} kg")
        print(f"  Diameter: {rocket_data.properties.ref_diameter_m} m")
        print(f"  Length: {rocket_data.properties.ref_length_m} m")

        if rocket_data.thrust_curve:
            print(f"  Burn time: {rocket_data.thrust_curve.time[-1]:.2f} s")
            print(f"  Max thrust: {max(rocket_data.thrust_curve.thrust):.0f} N")

        print("\nAero tables loaded:")
        tables = rocket_data.aero_tables
        for attr in ['aero_coeffs', 'fin_coeffs', 'damping_coeffs',
                     'fin_deflection_drag', 'ca_3d_coeffs', 'roll_aero_coeffs']:
            table = getattr(tables, attr)
            if table is not None:
                print(f"  - {attr}: {len(table)} rows")

    except RocketDataError as e:
        print(f"Error: {e}")
        sys.exit(1)
