#!/usr/bin/env python3
"""
6-DOF Rocket Simulation - Standard (NED, <50km) and Long-range (ECEF, 100-2000+km) modes.
Integrates propulsion, aerodynamics, atmosphere, and equations of motion.
"""

import sys
import os
import logging
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import yaml
from pathlib import Path
import warnings
import copy
from typing import Optional, Dict, Any

from dynamics.equations_of_motion import RocketDynamics
from dynamics.propulsion import AdvancedPropulsionModel as PropulsionModel
from dynamics.aerodynamics import AdvancedAerodynamicsModel as AerodynamicsModel
from dynamics.atmosphere import AtmosphereModel
from dynamics.flight_phase import FlightPhaseManager
from dynamics.error_injection import ErrorInjectionModel
from dynamics.mass_properties import MassPropertiesModel, MultiStageMassPropertiesModel
from dynamics.actuator import ActuatorModel, ActuatorWithBacklash, SecondOrderActuatorModel
from dynamics.launcher import LaunchRailModel
from dynamics.frame_manager import (
    FrameConfiguration,
    create_frame_manager_from_config,
    transform_ned_to_fur,
)
from dynamics.quaternion_utils import (
    quaternion_to_euler,
    euler_to_quaternion,
    degrees_to_quaternion,
    QUAT_NORM_THRESHOLD,
    DegenerateQuaternionError,
)
from dynamics.quaternion_state_manager import QuaternionStateManager
from rocket_data_loader import (
    RocketDataLoader, RocketData,
    MultiStageRocketData
)
from dynamics.fin_deflection_profile import FinDeflectionProfile, load_fin_deflection_profile
from mpc.m130_mpc_autopilot import MpcController


logger = logging.getLogger(__name__)


class Rocket6DOFSimulation:
    """
    Complete 6-DOF rocket simulation.

    State vector (14): [x,y,z, vx,vy,vz, q0,q1,q2,q3, wx,wy,wz, mass]
    - Position/velocity: NED (m, m/s) or ECEF in long_range_mode
    - Quaternion: body → local NED
    - Angular velocity: body frame (rad/s)

    Long-range mode adds: ECEF tracking, Coriolis/centrifugal, J2-J4 gravity, geodesic range.
    """

    def __init__(self, config_file=None, long_range_mode=False,
                 rocket_name: Optional[str] = None, rocket_stage: Optional[str] = None):
        """
        Initialize the 6-DOF simulation.

        Args:
            config_file: Path to configuration YAML file
            long_range_mode: Enable ECEF tracking for long-range (100-2000+ km)
            rocket_name: Rocket model name from data/rocket_models/ (overrides config)
            rocket_stage: Stage name for multi-stage rockets (e.g., 'stage1', 'warhead')
        """

        if config_file is None:
            config_file = Path(__file__).parent / 'config' / '6dof_config_advanced.yaml'

        with open(config_file, 'r', encoding='utf-8') as f:
            self.config = yaml.safe_load(f)

        # توحيد ارتفاع موقع الإطلاق - Unify launch altitude from config
        # يتم التحقق من وجود قيمتين مختلفتين وتحذير المستخدم
        self._unify_launch_altitude()

        self.long_range_mode = long_range_mode
        self.rocket_data: Optional[RocketData] = None
        self.multi_stage_data: Optional[MultiStageRocketData] = None
        self.use_launch_fixed_ned = True

        # Check if rocket_name is specified in config
        rocket_config = self.config.get('rocket_data', {})
        if rocket_name is None:
            rocket_name = rocket_config.get('name')
        if rocket_stage is None:
            rocket_stage = rocket_config.get('stage')

        # Check for multi-stage loading mode
        multi_stage_mode = rocket_config.get('multi_stage', False)

        # Load rocket data if rocket_name is specified
        if rocket_name is not None:
            base_dir = rocket_config.get('base_dir', 'data/rocket_models')
            base_path = Path(__file__).parent / base_dir
            loader = RocketDataLoader(base_path)

            if multi_stage_mode:
                # Load all stages of a multi-stage rocket
                logger.info(f"Loading multi-stage rocket data: {rocket_name}")
                self.multi_stage_data = loader.load_multi_stage(rocket_name)

                # Build multi-stage config and merge with existing config
                multi_stage_config = loader.build_multi_stage_config(self.multi_stage_data)

                # Merge multi-stage config (overwrites most sections, merges flight_phase)
                for key in ['stages', 'propulsion', 'aerodynamics', 'actuator', 'safety']:
                    if key in multi_stage_config:
                        self.config[key] = multi_stage_config[key]
                
                # flight_phase: merge (preserve user separation settings)
                if 'flight_phase' in multi_stage_config:
                    existing_fp = self.config.get('flight_phase', {})
                    multi_stage_fp = multi_stage_config['flight_phase']
                    # Start with multi-stage config, then overlay user settings
                    merged_fp = {**multi_stage_fp, **existing_fp}
                    # Ensure multi-stage specific settings are always present
                    merged_fp['num_stages'] = multi_stage_fp.get('num_stages', 1)
                    merged_fp['stage_burnout_times'] = multi_stage_fp.get('stage_burnout_times', [])
                    self.config['flight_phase'] = merged_fp

                # Update simulation settings for multi-stage
                sim_config = self.config.get('simulation', {})
                sim_config['multi_stage_enabled'] = True
                sim_config['num_stages'] = self.multi_stage_data.num_stages
                self.config['simulation'] = sim_config

                # Use first stage for initial rocket data
                first_stage = self.multi_stage_data.stages[0]
                self.rocket_data = first_stage.rocket_data

                logger.info(f"Loaded {self.multi_stage_data.num_stages} stages:")
                for stage in self.multi_stage_data.stages:
                    props = stage.rocket_data.properties
                    logger.info(f"  Stage {stage.id} ({stage.name}): "
                               f"{props.mass_dry_kg:.1f} kg dry, "
                               f"{props.propellant_mass_kg:.1f} kg propellant")

                # Update config with first stage properties before creating models
                rocket_props = self.rocket_data.properties
                if rocket_props is not None:
                    self._update_config_from_rocket_data(rocket_props)

                # Initialize models with first stage data (after config is updated)
                self.propulsion = PropulsionModel(
                    self.config.get('propulsion', {}),
                    thrust_curve=self.rocket_data.thrust_curve
                )
                self.aerodynamics = AerodynamicsModel(
                    self.config.get('aerodynamics', {}),
                    aero_tables=self.rocket_data.aero_tables
                )

                # Track current stage index (0-based) for model switching
                self._current_stage_index = 0
                # Last stage checked for separation (0-based for consistency with _current_stage_index)
                self._last_separation_check_stage_idx = 0
                self._last_separation_time = -1.0  # Time-based guard against double-firing
            else:
                # Single-stage loading (original behavior)
                logger.info(f"Loading rocket data: {rocket_name} (stage: {rocket_stage})")
                self.rocket_data = loader.load(rocket_name, stage=rocket_stage)

                # Update rocket config with properties from rocket data BEFORE creating models
                # This ensures reference_area and reference_length are available for AerodynamicsModel
                rocket_props = self.rocket_data.properties
                if rocket_props is not None:
                    self._update_config_from_rocket_data(rocket_props)

                # Initialize models with rocket data (after config is updated)
                self.propulsion = PropulsionModel(
                    self.config.get('propulsion', {}),
                    thrust_curve=self.rocket_data.thrust_curve
                )
                self.aerodynamics = AerodynamicsModel(
                    self.config.get('aerodynamics', {}),
                    aero_tables=self.rocket_data.aero_tables
                )
        else:
            # Legacy mode: load from config file only
            # This will raise an error if no data is available
            self.propulsion = PropulsionModel(self.config['propulsion'])
            self.aerodynamics = AerodynamicsModel(self.config['aerodynamics'])

        if rocket_name is not None:
            if self.rocket_data is None or self.rocket_data.atmosphere_table is None:
                raise ValueError(
                    "atmosphere_table.csv is required. Each rocket model must include "
                    "an atmosphere_table.csv file. "
                    "جدول الغلاف الجوي مطلوب. يجب أن يتضمن كل نموذج صاروخ ملف atmosphere_table.csv"
                )

        # تهيئة نموذج الغلاف الجوي
        if self.rocket_data is not None and self.rocket_data.atmosphere_table is not None:
            self.atmosphere = AtmosphereModel(
                self.rocket_data.atmosphere_table,
                self.config['atmosphere']
            )
        else:
            # الوضع القديم: استخدام نموذج الغلاف الجوي من التكوين فقط
            self.atmosphere = AtmosphereModel(
                None,
                self.config['atmosphere']
            )

        # إنشاء نموذج خصائص الكتلة للكتلة المتغيرة مع الزمن ومركز الثقل والعطالة
        self.mass_properties_model = self._create_mass_properties_model()

        # Merge simulation settings into rocket config for RocketDynamics
        # دمج إعدادات المحاكاة في تكوين الصاروخ لـ RocketDynamics
        rocket_config_for_dynamics = self.config['rocket'].copy()
        sim_config = self.config.get('simulation', {})
        # Pass use_inertia_derivative setting to RocketDynamics
        if 'use_inertia_derivative' in sim_config:
            rocket_config_for_dynamics['use_inertia_derivative'] = sim_config['use_inertia_derivative']
        # Pass conserve_angular_momentum setting to RocketDynamics
        if 'conserve_angular_momentum' in sim_config:
            rocket_config_for_dynamics['conserve_angular_momentum'] = sim_config['conserve_angular_momentum']
        self.dynamics = RocketDynamics(rocket_config_for_dynamics, mass_properties_model=self.mass_properties_model)
        self.dry_mass = getattr(self.dynamics, 'mass_dry', self.config['rocket'].get('mass_dry', 0.0))

        # Log rocket data info if loaded
        if self.rocket_data is not None:
            logger.info("Rocket data loaded successfully:")
            logger.info(f"  Name: {self.rocket_data.name}")
            if self.rocket_data.properties:
                props = self.rocket_data.properties
                mass_total = props.mass_dry_kg + props.propellant_mass_kg
                logger.info(f"  Mass: {mass_total:.2f} kg")
                logger.info(f"  Length: {props.ref_length_m:.3f} m")
            logger.info(f"  Aero tables: {self.rocket_data.aero_tables is not None}")
            logger.info(f"  Thrust curve: {self.rocket_data.thrust_curve is not None}")

        long_range_config = self.config.get('long_range', {})
        if long_range_config.get('enabled', False):
            self.long_range_mode = True

        # Default target altitude (used for ground impact detection)
        target_config = self.config.get('target', {})
        self.target_altitude_m = target_config.get('altitude', 0.0)
        self.target_range_m = target_config.get('range_m', None)

        if self.long_range_mode:
            gravity_model = long_range_config.get('gravity_model', 'j2_j3_j4')
            self.dynamics.gravity_model = gravity_model

            launch_config = self.config.get('launch', {})
            atm_config = self.config.get('atmosphere', {})

            if 'latitude' in launch_config:
                self.launch_lat_rad = np.radians(launch_config.get('latitude', 0.0))
                self.launch_lon_rad = np.radians(launch_config.get('longitude', 0.0))
                self.launch_alt_m = launch_config.get('altitude', 0.0)
            else:
                self.launch_lat_rad = atm_config.get('latitude_rad', 0.0)
                self.launch_lon_rad = atm_config.get('longitude_rad', 0.0)
                self.launch_alt_m = atm_config.get('launch_altitude_m', 0.0)

            self.launch_ecef = self.dynamics.earth_model.lla_to_ecef(
                self.launch_lat_rad, self.launch_lon_rad, self.launch_alt_m
            )

            self.include_coriolis = long_range_config.get('include_coriolis', True)
            self.include_centrifugal = long_range_config.get('include_centrifugal', True)

            # use_launch_fixed_ned=True: Earth rotation only (launch-fixed NED)
            # use_launch_fixed_ned=False: Earth rotation + transport rate (local NED)
            self.use_launch_fixed_ned = long_range_config.get('use_launch_fixed_ned', True)

            # Validate launch coordinates consistency across all sources
            self._validate_launch_coordinates_consistency(launch_config, atm_config)

            # Resolve target range if only target lat/lon are provided.
            if self.target_range_m is None and ('latitude' in target_config) and ('longitude' in target_config):
                tgt_lat = np.radians(target_config.get('latitude', np.degrees(self.launch_lat_rad)))
                tgt_lon = np.radians(target_config.get('longitude', np.degrees(self.launch_lon_rad)))
                self.target_range_m = self.dynamics.earth_model.geodesic_distance(
                    self.launch_lat_rad, self.launch_lon_rad, tgt_lat, tgt_lon
                )

            logger.info(f"Target altitude set to {self.target_altitude_m:.1f} m (ارتفاع الهدف)")

        # Shared frozen FrameConfiguration for all subsystems
        if self.long_range_mode:
            # Long-range mode: use computed ECEF and rotation matrix
            C_ecef_to_ned_launch = self.dynamics.earth_model.rot_ecef_to_ned(
                self.launch_lat_rad, self.launch_lon_rad
            )
            self.frame_configuration = FrameConfiguration(
                long_range_mode=self.long_range_mode,
                use_launch_fixed_ned=self.use_launch_fixed_ned,
                include_earth_rate=self.long_range_mode,  # Synchronized with long_range_mode
                include_coriolis=self.include_coriolis,
                include_centrifugal=self.include_centrifugal,
                launch_lat_rad=self.launch_lat_rad,
                launch_lon_rad=self.launch_lon_rad,
                launch_alt_m=self.launch_alt_m,
                launch_ecef=tuple(self.launch_ecef),
                C_ecef_to_ned_launch=tuple(C_ecef_to_ned_launch.flatten())
            )
            logger.info("Created SHARED FrameConfiguration for long-range mode:")
            logger.info(f"  long_range_mode={self.frame_configuration.long_range_mode}")
            logger.info(f"  use_launch_fixed_ned={self.frame_configuration.use_launch_fixed_ned}")
            logger.info(f"  include_earth_rate={self.frame_configuration.include_earth_rate}")
            logger.info(f"  launch_lat_rad={self.frame_configuration.launch_lat_rad:.6f}")
            logger.info(f"  launch_lon_rad={self.frame_configuration.launch_lon_rad:.6f}")
        else:
            # Short-range mode: use default values
            self.frame_configuration = FrameConfiguration(
                long_range_mode=False,
                use_launch_fixed_ned=True,
                include_earth_rate=False,
                include_coriolis=True,
                include_centrifugal=True,
                launch_lat_rad=0.0,
                launch_lon_rad=0.0,
                launch_alt_m=0.0,
                launch_ecef=None,
                C_ecef_to_ned_launch=None
            )
            logger.info("Created SHARED FrameConfiguration for short-range mode (default values)")

        # Build flight phase configuration
        fp_config = self.config.get('flight_phase', {})
        flight_config = {
            'burnout_time_s': self.config.get('propulsion', {}).get('burn_time', 5.0),
            'burnout_accel_g': 1.0,
            'burnout_hold_s': 0.2,
            'apogee_vz_threshold': 5.0,
            'apogee_hold_s': 0.5,
            # Auto-arm configuration: when True, system starts in ARMED state ready for launch
            'auto_arm': fp_config.get('auto_arm', True),  # Default True for simulation
            'long_range_mode': self.long_range_mode,
        }

        sim_config = self.config.get('simulation', {})
        if sim_config.get('multi_stage_enabled', False):
            flight_config['num_stages'] = sim_config.get('num_stages', 1)

            # Build stage burnout times from config
            if 'stage_burnout_times' in fp_config:
                flight_config['stage_burnout_times'] = fp_config['stage_burnout_times']
                logger.info(f"Using stage_burnout_times from flight_phase config: {fp_config['stage_burnout_times']}")
            else:
                # Second: try to build from stages config
                stages_config = self.config.get('stages', [])
                if stages_config:
                    stage_burnout_times = []
                    for stage in stages_config:
                        burn_time = stage.get('propulsion', {}).get('burn_time', 5.0)
                        stage_burnout_times.append(burn_time)
                    flight_config['stage_burnout_times'] = stage_burnout_times
                    logger.info(f"Built stage_burnout_times from stages config: {stage_burnout_times}")
                else:
                    # Third: use burnout times from multi-stage propulsion models
                    if hasattr(self, 'multi_stage_data') and self.multi_stage_data is not None:
                        stage_burnout_times = []
                        for stage in self.multi_stage_data.stages:
                            if stage.rocket_data.thrust_curve is not None:
                                burn_time = stage.rocket_data.thrust_curve.calculate_burn_time()
                                stage_burnout_times.append(burn_time)
                            else:
                                stage_burnout_times.append(5.0)  # Default fallback
                        flight_config['stage_burnout_times'] = stage_burnout_times
                        logger.info(f"Calculated stage_burnout_times from thrust curves: {stage_burnout_times}")

            # الحصول على تكوين الانفصال من تكوين مرحلة الطيران أو الافتراضيات
            flight_config['separation_trigger'] = fp_config.get('separation_trigger', 0)  # Default: BURNOUT
            flight_config['separation_delay_s'] = fp_config.get('separation_delay_s', 0.5)
            flight_config['separation_altitude_m'] = fp_config.get('separation_altitude_m', 50000.0)
            flight_config['separation_velocity_ms'] = fp_config.get('separation_velocity_ms', 1000.0)
            flight_config['separation_time_s'] = fp_config.get('separation_time_s', 10.0)

            # Configurable phases where separation is allowed
            # Default: None (uses FlightPhaseManager defaults: BURNOUT, COAST_ASCENT, etc.)
            # To allow separation during POWERED_ASCENT, add it to the list:
            # separation_allowed_phases: ["POWERED_ASCENT", "BURNOUT", "COAST_ASCENT"]
            if 'separation_allowed_phases' in fp_config:
                flight_config['separation_allowed_phases'] = fp_config['separation_allowed_phases']

            # This makes Rocket6DOFSimulation the single source of truth for separation
            # timing, using precise event detection (bisection).
            # FlightPhaseManager will NOT autonomously trigger separation in update().
            flight_config['external_separation_handling'] = True

            logger.info(f"Multi-stage flight phase config: {flight_config['num_stages']} stages")

        # ==========================================================================
        # Pass shared FrameConfiguration to FlightPhaseManager
        # ==========================================================================
        # This ensures FlightPhaseManager uses the same frame configuration as all
        # other subsystems, preventing parameter divergence.
        # ==========================================================================
        self.flight_phase_manager = FlightPhaseManager(
            flight_config, 
            frame_configuration=self.frame_configuration
        )
        
        # ==========================================================================
        # VALIDATION: Verify long_range_mode synchronization with FlightPhaseManager
        # With shared FrameConfiguration, this validation should always pass.
        # ==========================================================================
        if self.flight_phase_manager.long_range_mode != self.long_range_mode:
            logger.error(
                "CRITICAL: FlightPhaseManager long_range_mode mismatch despite shared FrameConfiguration! "
                f"FlightPhaseManager.long_range_mode={self.flight_phase_manager.long_range_mode} "
                f"but simulation.long_range_mode={self.long_range_mode}. "
                "This indicates a bug in the configuration flow. "
                "حرج: عدم تطابق long_range_mode في FlightPhaseManager رغم FrameConfiguration المشترك! "
                "هذا يشير إلى خطأ في تدفق التكوين."
            )
            self.flight_phase_manager.long_range_mode = self.long_range_mode
        logger.info(f"FlightPhaseManager using shared FrameConfiguration: long_range_mode={self.flight_phase_manager.long_range_mode}")

        # تسليح نظام الانفصال إذا كان متعدد المراحل مفعلاً
        if sim_config.get('multi_stage_enabled', False):
            self.flight_phase_manager.arm_separation()
            logger.info("Multi-stage separation system armed (نظام الانفصال متعدد المراحل مسلح)")

        # Error injection configuration (تكوين حقن الأخطاء)
        sim_config = self.config.get('simulation', {})
        self.use_error_injection = sim_config.get('use_error_injection', False)
        self.error_model = None
        self.errors = None

        if self.use_error_injection:
            error_config = self.config.get('error_injection', {})
            self.error_model = ErrorInjectionModel(error_config)
            self.errors = self.error_model.generate_errors()

            # تطبيق الأخطاء على تكوين الشروط الابتدائية قبل تهيئة الحالة
            self._apply_error_injection_to_config()

            logger.info("Error injection enabled (حقن الأخطاء مفعل)")
            logger.info(f"  Thrust scale error: {self.errors['thrust_scale_error']:.4f}")
            logger.info(f"  Elevation error: {np.degrees(self.errors['elevation_error']):.4f} deg")
            logger.info(f"  Azimuth error: {np.degrees(self.errors['azimuth_error']):.4f} deg")

        # Ballistic mode configuration (وضع المقذوف الباليستي)
        self.ballistic_mode = sim_config.get('ballistic_mode', False)
        self.ballistic_fin_deflection = np.array(
            sim_config.get('ballistic_fin_deflection', [0.0, 0.0, 0.0, 0.0])
        )
        
        # Validate ballistic fin deflection configuration
        if self.ballistic_mode and np.any(self.ballistic_fin_deflection != 0):
            self._validate_ballistic_fin_deflection()
        
        # ==========================================================================
        # Fin deflection from external file (انحراف الزعانف من ملف خارجي)
        # ==========================================================================
        # When enabled, fin deflections are read from an external file instead of
        # ==========================================================================
        self.use_fin_deflection_file = sim_config.get('use_fin_deflection_file', False)
        self.fin_deflection_profile: Optional[FinDeflectionProfile] = None
        
        if self.use_fin_deflection_file:
            base_path = Path(__file__).parent
            self.fin_deflection_profile = load_fin_deflection_profile(self.config, base_path)
            
            if self.fin_deflection_profile is not None:
                profile_info = self.fin_deflection_profile.get_info()
                logger.info(
                    f"Fin deflection profile loaded from file: {profile_info['file_path']}\n"
                    f"  تم تحميل ملف انحراف الزعانف: {profile_info['file_path']}\n"
                    f"  Points: {profile_info['num_points']}, "
                    f"Time range: [{profile_info['time_start']:.3f}, {profile_info['time_end']:.3f}] s, "
                    f"Frequency: {profile_info['frequency_hz']:.1f} Hz"
                )
            else:
                logger.warning(
                    "use_fin_deflection_file is True but failed to load profile. "
                    "Falling back to normal mode.\n"
                    "use_fin_deflection_file مفعّل لكن فشل تحميل الملف. "
                    "الرجوع إلى الوضع العادي."
                )
                self.use_fin_deflection_file = False

        # Output rate configuration (معدل تسجيل البيانات)
        # Controls how often data is recorded to history (Hz)
        # Default: None (record every time step)
        output_rate_hz = sim_config.get('output_rate', None)
        if output_rate_hz is not None and output_rate_hz > 0:
            self.output_interval = 1.0 / output_rate_hz  # Time between recorded samples (s)
            logger.info(f"Output rate: {output_rate_hz} Hz (interval: {self.output_interval:.4f} s)")
        else:
            self.output_interval = None  # Record every time step
            logger.info("Output rate: every time step (كل خطوة زمنية)")

        # Degenerate quaternion handling: "raise" (default) or "warn_and_recover"
        self.degenerate_quaternion_handling = sim_config.get(
            'degenerate_quaternion_handling', 'raise'
        )
        if self.degenerate_quaternion_handling not in ('raise', 'warn_and_recover'):
            logger.warning(
                f"Invalid degenerate_quaternion_handling value: '{self.degenerate_quaternion_handling}'. "
                f"Using 'raise' as default. Valid values: 'raise', 'warn_and_recover'"
            )
            self.degenerate_quaternion_handling = 'raise'
        
        # Unified Quaternion State Manager (see dynamics/quaternion_state_manager.py)
        self.max_quaternion_recoveries = sim_config.get('max_quaternion_recoveries', 3)
        self.quaternion_state_manager = QuaternionStateManager(
            max_recoveries=self.max_quaternion_recoveries,
            quat_norm_threshold=QUAT_NORM_THRESHOLD  # Use module-level constant directly
        )
        
        # DEPRECATED: Use quaternion_state_manager directly instead of legacy property aliases
        
        if self.degenerate_quaternion_handling == 'warn_and_recover':
            logger.warning(
                "degenerate_quaternion_handling='warn_and_recover' is enabled. "
                "This mode is intended for DEBUGGING/DIAGNOSTICS ONLY. "
                "When a degenerate quaternion is detected, the simulation will attempt "
                "to recover using the last valid quaternion, but results may be NON-PHYSICAL. "
                "For production use, set degenerate_quaternion_handling='raise' (default). "
                "Maximum recoveries allowed: %d. NaN quaternions will always raise an error. "
                "تحذير: وضع warn_and_recover مفعّل. هذا الوضع للتشخيص فقط. "
                "النتائج قد تكون غير فيزيائية. للاستخدام الإنتاجي، استخدم 'raise'.",
                self.max_quaternion_recoveries
            )

        # Attitude mode: "none" | "velocity_aligned" | "max_range" | "3dof"
        self.attitude_mode = sim_config.get('attitude_mode', 'none')

        # Liftoff threshold for ground impact detection (configurable for low-altitude tests)
        # Default 10.0m is suitable for most rockets, but can be lowered for hop tests
        self.liftoff_threshold = sim_config.get('liftoff_threshold', 10.0)

        self._motor_has_burned_out = False

        if self.attitude_mode == 'velocity_aligned':
            logger.info("Attitude mode: velocity_aligned - always align with velocity vector")
            logger.info("  وضع الاتجاه: محاذاة دائمة مع متجه السرعة (alpha ≈ 0)")
        elif self.attitude_mode == 'max_range':
            logger.info("Attitude mode: max_range - maximum range mode for theoretical maximum range")
            logger.info("  وضع الاتجاه: المدى الأقصى - لحساب أقصى مدى نظري")
            logger.info("  Based on reference: 'قبل انطفاء المحرك لا يؤخذ إلا عامل مقاومة الهواء'")
            logger.info("  During thrust: velocity-aligned (alpha=0) + only drag (CA), lift/moments zeroed")
            logger.info("  أثناء الدفع: محاذاة مع السرعة (alpha=0) + سحب فقط (CA)، تصفير الرفع والعزوم")
            logger.info("  After burnout: free 6DOF dynamics - alpha develops from gravity turn, full aero model")
            logger.info("  بعد الانطفاء: ديناميكا 6DOF حرة - alpha يتطور من الدوران الجاذبي، نموذج هوائي كامل")
        elif self.attitude_mode == '3dof':
            logger.info("Attitude mode: 3dof - point mass simulation (3 translational DOF only)")
            logger.info("  وضع الاتجاه: 3DOF - محاكاة نقطة الكتلة (3 درجات حرية انتقالية فقط)")
            logger.info("  Rocket treated as point mass with no rotational dynamics")
            logger.info("  الصاروخ يُعامل كنقطة كتلة بدون ديناميكيات دورانية")
            logger.info("  Always velocity-aligned (alpha = 0, beta = 0)")
            logger.info("  محاذاة دائمة مع السرعة (alpha = 0, beta = 0)")
            logger.info("  Only drag (CA) used - no lift or moments throughout flight")
            logger.info("  يُستخدم فقط السحب (CA) - بدون رفع أو عزوم طوال الرحلة")
        elif self.attitude_mode != 'none':
            logger.warning(f"Unknown attitude_mode '{self.attitude_mode}', using 'none'")

        self.state = self._initialize_state()
        self._initial_quaternion = None

        # Launch rail model: 1-DOF constraint until CG clears rail
        self.launcher = LaunchRailModel(
            self.config,
            frame_configuration=self.frame_configuration
        )
        self.use_launcher = self.launcher.enabled
        
        if self.use_launcher:
            # Initialize rail axis from initial quaternion
            initial_quaternion = self.state[6:10].copy()
            
            # For long_range_mode, also compute ECEF rail axis
            C_ned_to_ecef = None
            if self.long_range_mode:
                C_ecef_to_ned = self.dynamics.earth_model.ecef_to_ned_rotation(
                    self.launch_lat_rad, self.launch_lon_rad
                )
                C_ned_to_ecef = C_ecef_to_ned.T
            
            self.launcher.initialize_rail_axis(initial_quaternion, C_ned_to_ecef)
            
            # ==========================================================================
            # التحقق من تهيئة محور القضيب | Validate rail axis initialization
            # ==========================================================================
            if not self.launcher.is_initialized:
                raise RuntimeError(
                    "CRITICAL: Launcher rail axis initialization failed! "
                    "launcher.is_initialized returned False after calling initialize_rail_axis(). "
                    "This should never happen - check initial_quaternion validity. "
                    "حرج: فشلت تهيئة محور قضيب المنصة! "
                    "launcher.is_initialized أرجع False بعد استدعاء initialize_rail_axis(). "
                    "هذا لا يجب أن يحدث أبداً - تحقق من صلاحية initial_quaternion."
                )
            logger.info(
                f"Launcher rail axis initialization VERIFIED (تم التحقق من تهيئة محور القضيب): "
                f"is_initialized={self.launcher.is_initialized}"
            )
            
            # Store initial quaternion for launcher if not already stored
            if self._initial_quaternion is None:
                self._initial_quaternion = initial_quaternion
            
            logger.info("Launch rail model initialized (نموذج قضيب الإطلاق مُهيأ)")
            logger.info(f"  Rail axis (NED): {self.launcher._rail_axis_ned}")
        else:
            logger.info("Launch rail model disabled (نموذج قضيب الإطلاق معطّل)")

        self.history = {}

        sim_config_for_dt = self.config.get('simulation', {})
        self._current_dt = sim_config_for_dt.get('dt', 0.01)

        # Frame Consistency Manager (uses explicit long_range_mode and shared FrameConfiguration)
        self.frame_manager = create_frame_manager_from_config(
            self.config, 
            self.dynamics.earth_model, 
            long_range_mode=self.long_range_mode,
            include_earth_rate=self.long_range_mode,
            quaternion_state_manager=self.quaternion_state_manager,
            frame_configuration=self.frame_configuration
        )
        logger.info("Frame consistency manager initialized with shared FrameConfiguration (مدير اتساق الإطارات مُهيأ مع FrameConfiguration مشترك)")
        logger.info(f"  Frame mode: {self.frame_manager.frame_mode.value}")
        logger.info(f"  Long-range mode: {self.frame_manager.config.long_range_mode}")

        # ==========================================================================
        # Synchronize initial quaternion with all managers
        # ==========================================================================
        initial_quaternion = self.state[6:10].copy()
        self._update_last_valid_quaternion(initial_quaternion)
        logger.debug(f"Initial quaternion synchronized: {initial_quaternion}")
        
        # Actuator dynamics (servo lag, rate/deflection limits) - toggle via simulation.use_actuator_dynamics
        self.use_actuator_dynamics = sim_config.get('use_actuator_dynamics', False)

        if self.use_actuator_dynamics:
            actuator_config = self.config.get('actuator', {})

            # Convert units from degrees to radians
            # YAML always specifies values in degrees, so we always convert to radians
            # User can specify units: "rad" to skip conversion if values are already in radians
            actuator_config_converted = actuator_config.copy()

            # Check config units - default is "deg" (degrees)
            units = actuator_config.get('units', 'deg')
            convert_to_rad = (units.lower() in ['deg', 'degrees', 'degree'])

            if convert_to_rad:
                # delta_max: تحويل من درجات إلى راديان
                delta_max_deg = actuator_config.get('delta_max', 15.0)
                actuator_config_converted['delta_max'] = np.deg2rad(delta_max_deg)
                logger.debug(f"Converted delta_max from {delta_max_deg}° to {actuator_config_converted['delta_max']:.4f} rad")

                # delta_min: تحويل من درجات إلى راديان
                delta_min_deg = actuator_config.get('delta_min', -15.0)
                actuator_config_converted['delta_min'] = np.deg2rad(delta_min_deg)
                logger.debug(f"Converted delta_min from {delta_min_deg}° to {actuator_config_converted['delta_min']:.4f} rad")

                # rate_max: تحويل من درجات/ثانية إلى راديان/ثانية
                rate_max_deg = actuator_config.get('rate_max', 300.0)
                actuator_config_converted['rate_max'] = np.deg2rad(rate_max_deg)
                logger.debug(f"Converted rate_max from {rate_max_deg}°/s to {actuator_config_converted['rate_max']:.4f} rad/s")

                # backlash: تحويل من درجات إلى راديان
                backlash_deg = actuator_config.get('backlash', 0.0)
                actuator_config_converted['backlash'] = np.deg2rad(backlash_deg)
                logger.debug(f"Converted backlash from {backlash_deg}° to {actuator_config_converted['backlash']:.4f} rad")

                # delta_trim: Convert from degrees to radians
                delta_trim_config = actuator_config.get('delta_trim', None)
                if delta_trim_config is not None:
                    delta_trim_array = np.asarray(delta_trim_config)
                    actuator_config_converted['delta_trim'] = np.deg2rad(delta_trim_array).tolist()
                    logger.debug(f"Converted delta_trim from {delta_trim_config}° to {actuator_config_converted['delta_trim']} rad")

                # Set units to radians after conversion to prevent double conversion in ActuatorWithBacklash
                actuator_config_converted['units'] = 'rad'

                logger.info(f"Actuator config converted from degrees to radians (units: {units})")

            # التحقق من صحة المعاملات | Validate parameters
            if actuator_config_converted.get('delta_min', -np.deg2rad(15.0)) >= actuator_config_converted.get('delta_max', np.deg2rad(15.0)):
                raise ValueError(
                    f"actuator.delta_min ({actuator_config_converted.get('delta_min')}) must be < "
                    f"delta_max ({actuator_config_converted.get('delta_max')})"
                )

            if actuator_config_converted.get('rate_max', np.deg2rad(300.0)) <= 0:
                raise ValueError(
                    f"actuator.rate_max ({actuator_config_converted.get('rate_max')}) must be > 0"
                )

            # ==========================================================================
            # Select actuator model based on model value in config
            # ==========================================================================
            # model: 1 = نموذج من الدرجة الأولى (ActuatorModel/ActuatorWithBacklash)
            # model: 2 = نموذج من الدرجة الثانية مع تأخير زمني (SecondOrderActuatorModel)
            # ==========================================================================
            actuator_model_type = actuator_config.get('model', 1)
            
            if actuator_model_type == 2:
                # ==========================================================================
                # Second-order model with time delay
                # ==========================================================================
                self.actuator = SecondOrderActuatorModel(actuator_config_converted)
                logger.info("Second-order actuator dynamics enabled (ديناميكا المشغلات من الدرجة الثانية مفعّلة)")
                logger.info(f"  wn: {self.actuator.wn:.1f} rad/s (natural frequency)")
                logger.info(f"  zeta_wn: {self.actuator.zeta_wn:.1f} (damping * natural frequency)")
                logger.info(f"  delay_steps: {self.actuator.delay_steps}")
                logger.info(f"  delay_buffer_size: {self.actuator.delay_buffer_size}")
            else:
                # First-order model (default): use ActuatorWithBacklash if backlash != 0
                backlash_value = actuator_config_converted.get('backlash', 0.0)
                if np.any(np.asarray(backlash_value) != 0):
                    self.actuator = ActuatorWithBacklash(actuator_config_converted)
                    logger.info("First-order actuator dynamics enabled with backlash (ديناميكا المشغلات من الدرجة الأولى مفعّلة مع backlash)")
                    # تنسيق backlash للطباعة - التعامل مع القيم العددية والمصفوفات
                    # Format backlash for logging - handle both scalar and array
                    backlash_arr = np.asarray(self.actuator.backlash)
                    if backlash_arr.ndim == 0 or np.allclose(backlash_arr, backlash_arr.flat[0]):
                        # Scalar or uniform array - show single value
                        logger.info(f"  backlash: {np.rad2deg(backlash_arr.flat[0]):.2f}°")
                    else:
                        # Non-uniform array - show all values
                        backlash_deg_str = ", ".join([f"{np.rad2deg(v):.2f}" for v in backlash_arr])
                        logger.info(f"  backlash: [{backlash_deg_str}]°")
                else:
                    self.actuator = ActuatorModel(actuator_config_converted)
                    logger.info("First-order actuator dynamics enabled (ديناميكا المشغلات من الدرجة الأولى مفعّلة)")
                tau = self.actuator.tau_servo
                if np.isscalar(tau) or (hasattr(tau, 'ndim') and tau.ndim == 0):
                    logger.info(f"  tau_servo: {float(tau):.4f} s")
                else:
                    tau_strs = ', '.join(f'{t*1000:.1f}' for t in tau)
                    logger.info(f"  tau_servo: per-fin [{tau_strs}] ms (mean={np.mean(tau)*1000:.1f}ms)")
            
            # Common info for all models
            logger.info(f"  delta_max: {np.rad2deg(self.actuator.delta_max):.1f}°")
            logger.info(f"  rate_max: {np.rad2deg(self.actuator.rate_max):.1f}°/s")
            logger.info(f"  n_actuators: {self.actuator.n_actuators}")

        else:
            self.actuator = None
            logger.info("Actuator dynamics disabled (ديناميكا المشغلات معطّلة)")

    # Legacy aliases synced with QuaternionStateManager for backward compatibility
    
    @property
    def _quaternion_recovery_count(self) -> int:
        """
        Legacy alias for quaternion recovery count (DEPRECATED).
        
        This property automatically reads from QuaternionStateManager.
        For new code, use quaternion_state_manager.get_recovery_count() directly.        
        """
        return self.quaternion_state_manager.get_recovery_count()
    
    @_quaternion_recovery_count.setter
    def _quaternion_recovery_count(self, value: int) -> None:
        """
        Legacy alias setter for quaternion recovery count (DEPRECATED).
        
        This property automatically writes to QuaternionStateManager.
        For new code, use quaternion_state_manager.reset() to reset the count.
       
        Note: Only setting to 0 is supported (resets the manager).
        Setting to other values logs a warning.
        """
        if value == 0:
            self.quaternion_state_manager.reset()
        else:
            logger.warning(
                f"Setting _quaternion_recovery_count to non-zero value ({value}) is deprecated. "
                f"Use quaternion_state_manager.record_recovery() instead. "
                f"تعيين _quaternion_recovery_count إلى قيمة غير صفرية ({value}) مهمل. "
                f"استخدم quaternion_state_manager.record_recovery() بدلاً من ذلك."
            )
    
    def _unify_launch_altitude(self):
        """Unify launch.altitude and atmosphere.launch_altitude_m; prefers launch.altitude on conflict."""
        launch_config = self.config.get('launch', {})
        atm_config = self.config.get('atmosphere', {})

        launch_altitude = launch_config.get('altitude')
        atm_launch_altitude = atm_config.get('launch_altitude_m')

        if launch_altitude is not None and atm_launch_altitude is not None:
            if abs(launch_altitude - atm_launch_altitude) > 0.1:
                logger.warning(
                    f"تحذير: قيمتان مختلفتان لارتفاع موقع الإطلاق! "
                    f"WARNING: Conflicting launch altitude values detected! "
                    f"launch.altitude={launch_altitude} m, "
                    f"atmosphere.launch_altitude_m={atm_launch_altitude} m. "
                    f"سيتم استخدام launch.altitude={launch_altitude} m كمصدر أساسي. "
                    f"Using launch.altitude={launch_altitude} m as primary source."
                )
                atm_config['launch_altitude_m'] = launch_altitude
                self.config['atmosphere'] = atm_config
        elif launch_altitude is not None:
            atm_config['launch_altitude_m'] = launch_altitude
            self.config['atmosphere'] = atm_config
        elif atm_launch_altitude is not None:
            launch_config['altitude'] = atm_launch_altitude
            self.config['launch'] = launch_config

    def _validate_launch_coordinates_consistency(self, launch_config: dict, atm_config: dict):
        """Validate that launch coordinates are consistent across all config sources.

        Args:
            launch_config: Launch configuration dictionary
            atm_config: Atmosphere configuration dictionary
        """
        # Tolerance for coordinate comparison (approximately 11 meters at equator)
        COORD_TOLERANCE_RAD = 1e-4  # ~0.006 degrees
        
        # Get coordinates from different sources
        sources = {}
        
        # Source 1: launch config (degrees)
        if 'latitude' in launch_config:
            sources['launch_config'] = {
                'lat': np.radians(launch_config.get('latitude', 0.0)),
                'lon': np.radians(launch_config.get('longitude', 0.0))
            }
        
        # Compare all sources against the actual launch coordinates being used
        actual = {'lat': self.launch_lat_rad, 'lon': self.launch_lon_rad}
        
        mismatches = []
        for source_name, coords in sources.items():
            lat_diff = abs(coords['lat'] - actual['lat'])
            lon_diff = abs(coords['lon'] - actual['lon'])
            
            if lat_diff > COORD_TOLERANCE_RAD or lon_diff > COORD_TOLERANCE_RAD:
                mismatches.append({
                    'source': source_name,
                    'lat_diff_deg': np.degrees(lat_diff),
                    'lon_diff_deg': np.degrees(lon_diff),
                    'source_lat_deg': np.degrees(coords['lat']),
                    'source_lon_deg': np.degrees(coords['lon'])
                })
        
        if mismatches:
            logger.warning(
                "==========================================================================\n"
                "WARNING: Launch coordinate mismatch detected! (تحذير: عدم تطابق إحداثيات الإطلاق!)\n"
                "==========================================================================\n"
                "This can cause frame transformation errors that grow with distance.\n"
                "هذا قد يسبب أخطاء تحويل الإطارات التي تتزايد مع المسافة.\n"
                f"Actual launch coordinates used: lat={np.degrees(actual['lat']):.6f}°, "
                f"lon={np.degrees(actual['lon']):.6f}°\n"
                f"إحداثيات الإطلاق الفعلية المستخدمة: lat={np.degrees(actual['lat']):.6f}°, "
                f"lon={np.degrees(actual['lon']):.6f}°"
            )
            for m in mismatches:
                logger.warning(
                    f"  Mismatch with {m['source']}: "
                    f"lat={m['source_lat_deg']:.6f}°, lon={m['source_lon_deg']:.6f}° "
                    f"(diff: lat={m['lat_diff_deg']:.6f}°, lon={m['lon_diff_deg']:.6f}°)"
                )
            logger.warning(
                "Please ensure all coordinate sources are consistent.\n"
                "يرجى التأكد من اتساق جميع مصادر الإحداثيات.\n"
                "=========================================================================="
            )
        else:
            logger.info(
                f"Launch coordinates validated: lat={np.degrees(actual['lat']):.6f}°, "
                f"lon={np.degrees(actual['lon']):.6f}° (تم التحقق من إحداثيات الإطلاق)"
            )

    def _validate_ballistic_fin_deflection(self):
        """Validate ballistic fin deflection: compute virtual axis deflections and warn on issues."""
        delta1, delta2, delta3, delta4 = self.ballistic_fin_deflection
        config_type = self.config.get('config_type', 'X')
        fin_location = self.config.get('fin_location', 'tail')
        
        # Compute virtual axis deflections based on configuration type
        if config_type == 'X':
            delta_pitch = 0.25 * (-delta1 - delta2 + delta3 + delta4)
            delta_yaw = 0.25 * (-delta1 + delta2 + delta3 - delta4)
            delta_roll = 0.25 * (delta1 + delta2 + delta3 + delta4)
        else:  # config_type == '+'
            delta_pitch = 0.5 * (-delta1 + delta3)
            delta_yaw = 0.5 * (delta2 - delta4)
            delta_roll = 0.25 * (delta1 - delta2 + delta3 - delta4)
        
        # Convert to degrees for logging
        delta_pitch_deg = np.degrees(delta_pitch)
        delta_yaw_deg = np.degrees(delta_yaw)
        delta_roll_deg = np.degrees(delta_roll)
        
        # Log the analysis
        logger.info(
            "Ballistic fin deflection analysis (تحليل انحراف الزعانف الباليستي):"
        )
        logger.info(
            f"  Config: {config_type} configuration, {fin_location} fins"
        )
        logger.info(
            f"  Input [δ1, δ2, δ3, δ4]: [{delta1:.4f}, {delta2:.4f}, {delta3:.4f}, {delta4:.4f}] rad"
        )
        logger.info(
            f"  Input [δ1, δ2, δ3, δ4]: [{np.degrees(delta1):.2f}, {np.degrees(delta2):.2f}, "
            f"{np.degrees(delta3):.2f}, {np.degrees(delta4):.2f}] deg"
        )
        logger.info(
            "  Resulting virtual deflections (انحرافات المحاور الافتراضية الناتجة):"
        )
        logger.info(
            f"    delta_pitch: {delta_pitch:.4f} rad ({delta_pitch_deg:.2f} deg)"
        )
        logger.info(
            f"    delta_yaw:   {delta_yaw:.4f} rad ({delta_yaw_deg:.2f} deg)"
        )
        logger.info(
            f"    delta_roll:  {delta_roll:.4f} rad ({delta_roll_deg:.2f} deg)"
        )
        
        # Threshold for considering a value as "effectively zero"
        zero_threshold = 1e-6
        
        # Check for potential issues and provide warnings
        warnings_issued = []
        
        # Issue 1: Zero roll with non-zero pitch/yaw
        if abs(delta_roll) < zero_threshold and (abs(delta_pitch) > zero_threshold or abs(delta_yaw) > zero_threshold):
            warnings_issued.append(
                f"WARNING: Ballistic fin configuration produces ZERO roll moment but non-zero "
                f"pitch ({delta_pitch_deg:.2f}°) and/or yaw ({delta_yaw_deg:.2f}°). "
                f"For ballistic rockets, roll stabilization (spin) is typically desired. "
                f"If you intended roll stabilization, use equal deflections for all fins "
                f"(e.g., [0.09, 0.09, 0.09, 0.09] for X config). "
                f"تحذير: تكوين الزعانف الباليستي ينتج عزم دوران صفري ولكن ميل/انعراج غير صفري. "
                f"للصواريخ الباليستية، عادة ما يكون استقرار الدوران (الدوران) مطلوباً."
            )
        
        # Issue 2: Non-zero pitch/yaw might cause instability
        if abs(delta_pitch) > zero_threshold or abs(delta_yaw) > zero_threshold:
            effect_desc = []
            if abs(delta_pitch) > zero_threshold:
                pitch_dir = "nose-up" if delta_pitch > 0 else "nose-down"
                if fin_location == 'canard':
                    pitch_dir = "nose-down" if delta_pitch > 0 else "nose-up"  # Canard reverses effect
                effect_desc.append(f"pitch ({pitch_dir}, {abs(delta_pitch_deg):.2f}°)")
            if abs(delta_yaw) > zero_threshold:
                yaw_dir = "right" if delta_yaw > 0 else "left"
                if fin_location == 'canard':
                    yaw_dir = "left" if delta_yaw > 0 else "right"  # Canard reverses effect
                effect_desc.append(f"yaw ({yaw_dir}, {abs(delta_yaw_deg):.2f}°)")
            
            warnings_issued.append(
                f"INFO: Ballistic fin configuration produces constant {' and '.join(effect_desc)} input. "
                f"This will cause the rocket to continuously pitch/yaw during flight, which may "
                f"lead to instability at high speeds. If this is not intended, consider using "
                f"a symmetric configuration for roll-only stabilization. "
                f"معلومات: تكوين الزعانف الباليستي ينتج إدخال {' و '.join(effect_desc)} ثابت."
            )
        
        if config_type == 'X':
            logger.info(
                "  Fin numbering for X configuration (looking from rear toward nose):"
            )
            logger.info(
                "    Fin 1: Upper-Left  (+Y, +Z) | Fin 2: Upper-Right (-Y, +Z)"
            )
            logger.info(
                "    Fin 3: Lower-Right (-Y, -Z) | Fin 4: Lower-Left  (+Y, -Z)"
            )
            logger.info(
                "  For roll stabilization (spin), use: [+δ, +δ, +δ, +δ] (all same sign)"
            )
            logger.info(
                "  For pitch control, use: [-δ, -δ, +δ, +δ] (upper vs lower)"
            )
            logger.info(
                "  For yaw control, use: [-δ, +δ, +δ, -δ] (diagonal pairs)"
            )
        
        # Log warnings
        for warning in warnings_issued:
            logger.warning(warning)

    def _update_config_from_rocket_data(self, props):
        """
        Update configuration from rocket properties loaded from rocket_ws format.

        Args:
            props: RocketProperties dataclass from RocketDataLoader
        """
        # Update rocket config
        rocket_config = self.config.get('rocket', {})

        # Calculate mass_total from dry mass and propellant mass
        mass_total = props.mass_dry_kg + props.propellant_mass_kg
        rocket_config['mass_total'] = mass_total
        rocket_config['mass_dry'] = props.mass_dry_kg
        rocket_config['mass_propellant'] = props.propellant_mass_kg
        rocket_config['length'] = props.ref_length_m
        rocket_config['diameter'] = props.ref_diameter_m

        # CG position (use x component from cg_dry_body_m as cg_from_nose)
        if props.cg_dry_body_m:
            rocket_config['xcg_end'] = props.cg_dry_body_m[0]
            cg_dry = np.array(props.cg_dry_body_m)
            cg_prop = np.array(props.get_cg_propellant())  # Use helper method for optional field
            cg_initial = (props.mass_dry_kg * cg_dry + props.propellant_mass_kg * cg_prop) / mass_total
            rocket_config['xcg_0'] = cg_initial[0]

        # Inertia (inertia_dry_kgm2 is a list [Ixx, Iyy, Izz])
        if props.inertia_dry_kgm2 and len(props.inertia_dry_kgm2) >= 3:
            inertia = props.inertia_dry_kgm2
            rocket_config['Ixx_end'] = inertia[0]
            rocket_config['Iyy_end'] = inertia[1]
            rocket_config['Izz_end'] = inertia[2]
            # Estimate initial inertia (propellant adds to inertia)
            inertia_factor = 1.0 + 0.5 * (props.propellant_mass_kg / props.mass_dry_kg)
            rocket_config['Ixx_0'] = inertia[0]  # Roll inertia doesn't change much
            rocket_config['Iyy_0'] = inertia[1] * inertia_factor
            rocket_config['Izz_0'] = inertia[2] * inertia_factor

        self.config['rocket'] = rocket_config

        # Update aerodynamics config
        aero_config = self.config.get('aerodynamics', {})
        aero_config['reference_length'] = props.ref_diameter_m
        aero_config['reference_area'] = props.ref_area_m2
        
        # Pass damping_coefficients_units from simulation config to aerodynamics config
        sim_config = self.config.get('simulation', {})
        damping_units = sim_config.get('damping_coefficients_units', 'per_rad')
        aero_config['damping_coefficients_units'] = damping_units
        
        # Pass fin_channel_mapping from simulation config to aerodynamics config
        fin_mapping = sim_config.get('fin_channel_mapping', [1, 2, 3, 4])
        aero_config['fin_channel_mapping'] = fin_mapping
        
        # Pass config_type and fin_location from rocket properties to top-level config and aerodynamics config
        # CRITICAL: These determine the fin mixing matrix and sign convention for pitch/yaw moments
        config_type = props.config_type
        self.config['config_type'] = config_type
        aero_config['config_type'] = config_type
        logger.info(f"config_type = '{config_type}' from rocket properties")

        # For canard fins (front), positive deflection creates negative pitch moment (nose down)
        # For tail fins (rear), positive deflection creates positive pitch moment (nose up)
        # The aerodynamics model negates CM_control and Cn_control for canard fins
        fin_location = props.fin_location
        self.config['fin_location'] = fin_location
        aero_config['fin_location'] = fin_location
        logger.info(f"fin_location = '{fin_location}' from rocket properties")
        
        # Pass CA_multiplier from rocket properties to aerodynamics config
        # Default 1.0 = no change, > 1.0 = increase drag, < 1.0 = decrease drag
        aero_config['CA_multiplier'] = props.CA_multiplier
        
        # Pass moment_reference_point from rocket properties to aerodynamics config
        # This is critical for moment transfer correction when CG differs from aero table reference
        if hasattr(props, 'moment_reference_point') and props.moment_reference_point is not None:
            aero_config['moment_reference_point'] = props.moment_reference_point
            logger.info(f"moment_reference_point = {props.moment_reference_point:.4f} m from rocket properties")

        self.config['aerodynamics'] = aero_config

        # Update propulsion config
        prop_config = self.config.get('propulsion', {})
        prop_config['propellant_mass'] = props.propellant_mass_kg
        if props.nozzle_exit_area_m2 > 0:
            prop_config['nozzle_exit_area'] = props.nozzle_exit_area_m2
        # Pass sea_level_pressure_Pa as reference_pressure for altitude correction
        # Use setdefault to preserve any explicit user override in config
        prop_config.setdefault('reference_pressure', props.sea_level_pressure_Pa)
        
        # Pass burn_time_scale from rocket properties to propulsion config
        # This scales the thrust curve time while maintaining total_impulse
        if hasattr(props, 'burn_time_scale') and props.burn_time_scale != 1.0:
            prop_config['burn_time_scale'] = props.burn_time_scale
            logger.info(f"burn_time_scale = {props.burn_time_scale:.4f} from rocket properties")

        # Pass thrust_multiplier from rocket properties to propulsion config
        # This multiplies thrust while keeping time and mass constant (effectively multiplies Isp)
        if hasattr(props, 'thrust_multiplier') and props.thrust_multiplier != 1.0:
            prop_config['thrust_scale'] = props.thrust_multiplier
            # Keep mass constant when thrust is scaled (don't scale mass_flow proportionally)
            prop_config['scale_mass_flow_with_thrust'] = False
            logger.info(f"thrust_multiplier = {props.thrust_multiplier:.4f} from rocket properties (Isp effectively multiplied)")

        self.config['propulsion'] = prop_config

        # Update safety config from rocket properties (single source of truth)
        # Rocket-specific safety limits should come from rocket_properties.yaml
        if props.safety:
            safety_config = self.config.get('safety', {})

            # q_alpha_max is set by rocket_data_loader.build_simulation_config()
            # which reads it from rocket_properties.yaml - this is the expected path

            # Override with rocket-specific safety values from rocket_properties.yaml
            if 'qa_max_Pa_rad' in props.safety:
                safety_config['q_alpha_max'] = props.safety['qa_max_Pa_rad']

            self.config['safety'] = safety_config
            logger.info(f"Updated safety config from rocket properties: q_alpha_max={safety_config.get('q_alpha_max')}")

        # Update launcher config from rocket properties (single source of truth)
        # Launcher parameters are now stored in rocket_properties.yaml for each rocket model
        if props.launcher:
            self.config['launcher'] = props.launcher
            logger.info(f"Updated launcher config from rocket properties: enabled={props.launcher.get('enabled', False)}")

        # Update actuator config from rocket properties (if actuator section defined)
        # Priority: rocket_properties.yaml actuator > 6dof_config_advanced.yaml actuator
        if props.actuator:
            existing_actuator = self.config.get('actuator', {})
            # Merge: rocket model values override global config values
            for key, value in props.actuator.items():
                existing_actuator[key] = value
            self.config['actuator'] = existing_actuator
            logger.info(f"Updated actuator config from rocket model: {props.actuator.get('type', 'unknown')}")

        logger.info("Updated config from rocket properties")

    def _get_stage_propulsion_config(self, stage_id: int) -> dict:
        """Get propulsion config for a stage (1-based stage_id) from config['propulsion'][f'stage_{id}']."""
        propulsion_config = self.config.get('propulsion', {})

        # Check for per-stage propulsion config (multi-stage mode)
        stage_key = f"stage_{stage_id}"
        if stage_key in propulsion_config:
            stage_config = propulsion_config[stage_key]
            logger.debug(f"Using per-stage propulsion config for {stage_key}")
            return stage_config

        # Fallback: check stages list for inline propulsion config
        stages_config = self.config.get('stages', [])
        stage_index = stage_id - 1  # Convert to 0-based index
        if stage_index < len(stages_config):
            stage_data = stages_config[stage_index]
            if 'propulsion' in stage_data:
                logger.debug(f"Using inline propulsion config from stages[{stage_index}]")
                return stage_data['propulsion']

            # Check if stage has no motor (unpowered stage like warhead)
            if not stage_data.get('has_motor', True):
                logger.debug(f"Stage {stage_id} has no motor, returning unpowered config")
                return {
                    'motor_type': 'none',
                    'burn_time': 0.0,
                    'propellant_mass': stage_data.get('mass_propellant', 0.0),
                }

        # Final fallback: return global propulsion config (single-stage mode)
        logger.debug(f"Using global propulsion config for stage {stage_id}")
        return propulsion_config

    def _create_mass_properties_model(self) -> Optional[MassPropertiesModel]:
        """Create time-varying mass/CG/inertia model; returns MultiStageMassPropertiesModel for multi-stage."""
        # Check for multi-stage mode
        sim_config = self.config.get('simulation', {})
        stages_config = self.config.get('stages', [])
        if sim_config.get('multi_stage_enabled', False) and stages_config:
            # Multi-stage: create MultiStageMassPropertiesModel
            multi_stage_mass_config = {'stages': stages_config}
            # Add thrust curves from loaded rocket data
            if self.multi_stage_data is not None:
                for i, stage in enumerate(self.multi_stage_data.stages):
                    if i < len(stages_config) and stage.rocket_data.thrust_curve is not None:
                        from dynamics.mass_properties import ThrustCurve
                        tc = stage.rocket_data.thrust_curve
                        propellant_mass = stages_config[i].get('mass_propellant', 0.0)
                        thrust_curve = ThrustCurve.from_loader_curve(tc, propellant_mass=propellant_mass)
                        stages_config[i]['thrust_curve'] = thrust_curve
            try:
                mass_model = MultiStageMassPropertiesModel(config=multi_stage_mass_config)
                total_mass = mass_model.get_mass(0.0)
                logger.info(f"Multi-stage mass properties model created: {len(stages_config)} stages, total mass={total_mass:.1f} kg")
                for i, stage_cfg in enumerate(stages_config):
                    logger.info(f"  Stage {stage_cfg.get('id', i+1)} ({stage_cfg.get('name', '?')}): "
                               f"dry={stage_cfg.get('mass_dry', 0):.1f} kg, "
                               f"propellant={stage_cfg.get('mass_propellant', 0):.1f} kg")
                return mass_model
            except Exception as e:
                raise RuntimeError(f"Failed to create multi-stage mass properties model: {e}") from e

        rocket_config = self.config.get('rocket', {})
        propulsion_config = self.config.get('propulsion', {})

        # الحصول على وقت الاحتراق من نموذج الدفع (يستخدم عتبة 10% للاتساق)
        burn_time = self.propulsion.get_burn_time()

        # MassPropertiesModel expects xcg_0 (CG at full propellant) and xcg_end (CG when dry)
        mass_props_config = {
            'mass_dry': rocket_config.get('mass_dry', 0.0),
            'mass_propellant': rocket_config.get('mass_propellant',
                               propulsion_config.get('propellant_mass', 0.0)),
            'burn_time': burn_time,
            'has_motor': True,

            # CG positions: xcg_0 = CG at ignition (full), xcg_end = CG at burnout (dry)
            'xcg_0': rocket_config.get('xcg_0', 0.0),
            'xcg_end': rocket_config.get('xcg_end', 0.0),

            # Inertia at burnout (dry)
            'Ixx_end': rocket_config.get('Ixx_end', 1.0),
            'Iyy_end': rocket_config.get('Iyy_end', 1.0),
            'Izz_end': rocket_config.get('Izz_end', 1.0),

            # Inertia at ignition (full propellant)
            'Ixx_0': rocket_config.get('Ixx_0', rocket_config.get('Ixx_end', 1.0)),
            'Iyy_0': rocket_config.get('Iyy_0', rocket_config.get('Iyy_end', 1.0)),
            'Izz_0': rocket_config.get('Izz_0', rocket_config.get('Izz_end', 1.0)),
        }

        # Pass post_separation_inertia_scale from simulation config
        sim_config = self.config.get('simulation', {})
        mass_props_config['post_separation_inertia_scale'] = sim_config.get(
            'post_separation_inertia_scale', 1.0
        )

        # Use thrust curve if available for accurate propellant consumption calculation
        if self.rocket_data is not None and self.rocket_data.thrust_curve is not None:
            from dynamics.mass_properties import ThrustCurve
            tc = self.rocket_data.thrust_curve
            propellant_mass = mass_props_config['mass_propellant']
            
            # This ensures accurate propellant tracking for variable Isp motors
            if isinstance(tc, dict):
                # Dict format: extract data manually (legacy support)
                time_data = tc['time']
                thrust_data = tc['thrust']
                mass_flow_data = tc.get('mass_flow', None)
                thrust_curve = ThrustCurve(
                    time_points=time_data,
                    thrust_values=thrust_data,
                    mass_flow_values=mass_flow_data,
                    propellant_mass=propellant_mass
                )
            else:
                # Dataclass format: use from_loader_curve() to preserve all data
                thrust_curve = ThrustCurve.from_loader_curve(tc, propellant_mass=propellant_mass)
            
            # Pass thrust curve inside config (MassPropertiesModel expects it there)
            mass_props_config['thrust_curve'] = thrust_curve

        try:
            mass_model = MassPropertiesModel(config=mass_props_config)
            logger.info("Mass properties model created successfully")
            logger.info(f"  Burn time: {burn_time:.2f} s (from propulsion model)")
            logger.info(f"  Dry mass: {mass_props_config['mass_dry']:.2f} kg")
            logger.info(f"  Propellant mass: {mass_props_config['mass_propellant']:.2f} kg")
            return mass_model
        except Exception as e:
            # Failed to create mass properties model
            raise RuntimeError(
                f"Failed to create mass properties model: {e}\n"
                f"This usually means the thrust curve is missing or invalid, "
                f"or there is a mismatch between has_motor/mass_propellant settings.\n"
                f"Please ensure:\n"
                f"  1. thrust_curve.csv exists and has valid non-zero thrust values\n"
                f"  2. mass_propellant > 0 only if the stage has a motor\n"
                f"  3. burn_time is calculated correctly from the thrust curve"
            ) from e

    def _switch_to_stage(self, stage_index: int, t: float, state: np.ndarray = None):
        """Switch physics models to a new stage after separation.

        Args:
            stage_index: 0-based index of the new stage
            t: Separation time (s)
            state: Current state vector for fin effectiveness update; uses Mach=0, alpha=0 if None
        """
        if self.multi_stage_data is None:
            logger.warning("Cannot switch stage: no multi-stage data loaded")
            return False

        if stage_index >= len(self.multi_stage_data.stages):
            logger.warning(f"Cannot switch to stage {stage_index}: only {len(self.multi_stage_data.stages)} stages available")
            return False

        new_stage = self.multi_stage_data.stages[stage_index]
        new_rocket_data = new_stage.rocket_data

        # Get ignition delay for this stage
        ignition_delay = self.flight_phase_manager.get_ignition_delay(stage_index)
        ignition_time = t + ignition_delay

        logger.info(f"=== STAGE SEPARATION at t={t:.3f}s ===")
        logger.info(f"Switching from stage {self._current_stage_index + 1} to stage {stage_index + 1} ({new_stage.name})")
        if ignition_delay > 0:
            logger.info(f"  Ignition delay: {ignition_delay:.3f}s (ignition at t={ignition_time:.3f}s)")

        # تحديث مرجع بيانات الصاروخ
        self.rocket_data = new_rocket_data

        # Get stage propulsion config (stored as config['propulsion'][f"stage_{stage_id}"])
        stage_propulsion_config = self._get_stage_propulsion_config(new_stage.id)

        # Switch propulsion model with new stage's thrust curve (or no thrust for unpowered stages)
        self.propulsion = PropulsionModel(
            stage_propulsion_config,
            thrust_curve=new_rocket_data.thrust_curve
        )
        # Set thrust curve start to ignition_time (thrust=0 during coast if ignition_delay > 0)
        self.propulsion.set_stage_start_time(ignition_time)
        logger.info(f"  Propulsion model updated (burn_time: {self.propulsion.get_burn_time():.2f}s)")

        # Update rocket config before creating aero model (ensures reference_area/length are set)
        if new_rocket_data.properties is not None:
            self._update_config_from_rocket_data(new_rocket_data.properties)

        # Switch aerodynamics model with new stage's aero tables
        stages_config = self.config.get('stages', [])
        if stage_index < len(stages_config):
            stage_aero_config = stages_config[stage_index].get('aerodynamics', {})
        else:
            stage_aero_config = self.config.get('aerodynamics', {})

        # Merge with global aerodynamics config for settings not specified per-stage
        global_aero_config = self.config.get('aerodynamics', {})
        merged_aero_config = {**global_aero_config, **stage_aero_config}

        self.aerodynamics = AerodynamicsModel(
            merged_aero_config,
            aero_tables=new_rocket_data.aero_tables
        )
        logger.info("  Aerodynamics model updated")

        # Update dry mass from new stage properties (config was already updated above)
        if new_rocket_data.properties is not None:
            props = new_rocket_data.properties
            self.dry_mass = props.mass_dry_kg
            logger.info(f"  New dry mass: {self.dry_mass:.2f} kg")
            logger.info(f"  New propellant mass: {props.propellant_mass_kg:.2f} kg")

        # Angular momentum conservation: ω_new = I_new⁻¹ × I_old × ω_old
        
        # Get old inertia matrix BEFORE updating the mass properties model
        I_old = None
        omega_old = None
        if state is not None and self.dynamics is not None:
            omega_old = state[10:13].copy()
            try:
                I_old = self.dynamics.get_inertia_matrix(t)
                logger.info(f"  Old inertia (before separation): diag(I) = [{I_old[0,0]:.2f}, {I_old[1,1]:.2f}, {I_old[2,2]:.2f}] kg·m²")
                logger.info(f"  Old angular velocity: ω = [{omega_old[0]:.4f}, {omega_old[1]:.4f}, {omega_old[2]:.4f}] rad/s")
            except Exception as e:
                logger.warning(f"  Could not get old inertia matrix: {e}. Angular momentum will not be conserved.")
                I_old = None

        # تحديث نموذج خصائص الكتلة للمرحلة الجديدة
        self.mass_properties_model = self._create_mass_properties_model()
        if self.mass_properties_model is not None:
            # Sync start time to ignition_time for consistent propellant consumption tracking
            self.mass_properties_model.set_stage_start_time(ignition_time)
            # Update dynamics.mass_model (not mass_properties_model) - this is what get_mass() uses
            self.dynamics.mass_model = self.mass_properties_model
            logger.info(f"  Mass properties model updated (stage_start_time synchronized to {ignition_time:.4f}s)")

        # Check if angular momentum conservation is enabled (default: True)
        conserve_angular_momentum = self.config.get('simulation', {}).get('conserve_angular_momentum', True)
        
        if I_old is not None and omega_old is not None and state is not None:
            if conserve_angular_momentum:
                try:
                    # Get new inertia matrix AFTER updating the mass properties model
                    I_new = self.dynamics.get_inertia_matrix(t)
                    logger.info(f"  New inertia (after separation): diag(I) = [{I_new[0,0]:.2f}, {I_new[1,1]:.2f}, {I_new[2,2]:.2f}] kg·m²")
                    
                    # Compute new angular velocity to conserve angular momentum
                    omega_new = self._conserve_angular_momentum_at_separation(I_old, I_new, omega_old)
                    
                    # Update state with new angular velocity
                    state[10:13] = omega_new
                    
                    # Log the change
                    L_old = I_old @ omega_old
                    L_new = I_new @ omega_new
                    L_error = np.linalg.norm(L_new - L_old) / (np.linalg.norm(L_old) + 1e-10)
                    
                    logger.info(f"  New angular velocity: ω = [{omega_new[0]:.4f}, {omega_new[1]:.4f}, {omega_new[2]:.4f}] rad/s")
                    logger.info(f"  Angular momentum conserved (relative error: {L_error:.2e})")
                    logger.info(f"  Angular velocity change: Δω = [{omega_new[0]-omega_old[0]:.4f}, {omega_new[1]-omega_old[1]:.4f}, {omega_new[2]-omega_old[2]:.4f}] rad/s")
                except Exception as e:
                    logger.warning(f"  Could not apply angular momentum conservation: {e}. Keeping original angular velocity.")
            else:
                # Angular momentum conservation disabled - keep angular velocity constant
                logger.info("  Angular momentum conservation DISABLED (conserve_angular_momentum=false)")
                logger.info(f"  Keeping angular velocity constant: ω = [{omega_old[0]:.4f}, {omega_old[1]:.4f}, {omega_old[2]:.4f}] rad/s")

        # تحديث فهرس المرحلة الحالية
        self._current_stage_index = stage_index

        # Preserve actuator state on separation (avoid moment discontinuity → oscillation)
        if self.use_actuator_dynamics and self.actuator is not None:
            current_deflections = self.actuator.delta_current.copy()
            logger.info(f"  Actuator state preserved (fin deflections: [{current_deflections[0]:.2f}, {current_deflections[1]:.2f}, {current_deflections[2]:.2f}, {current_deflections[3]:.2f}] deg)")

        logger.info(f"=== Stage {stage_index + 1} active ===")
        return True

    def _apply_error_injection_to_config(self):
        """
        Apply error injection (attitude, mass, CG, sensors) to config copy.
        Original preserved in _original_config for multi-run resets.
        """
        if self.errors is None:
            return

        # Preserve original config for potential reset (multiple simulation runs)
        if not hasattr(self, '_original_config'):
            self._original_config = copy.deepcopy(self.config)

        # Work on a deep copy to avoid modifying the original config
        self.config = copy.deepcopy(self._original_config)

        ic = self.config.get('initial_conditions', {})

        # Apply attitude errors
        attitude = ic.get('attitude', [1, 0, 0, 0])

        if len(attitude) == 4:
            # تنسيق quaternion: تحويل إلى Euler، تطبيق الأخطاء، ثم التحويل مرة أخرى
            roll, pitch, yaw = quaternion_to_euler(attitude)
            pitch += self.errors['elevation_error']
            yaw += self.errors['azimuth_error']
            attitude = euler_to_quaternion(roll, pitch, yaw)
            ic['attitude'] = attitude
        elif len(attitude) == 3:
            # تنسيق Euler: تطبيق الأخطاء مباشرة
            roll, pitch, yaw = attitude
            pitch += self.errors['elevation_error']
            yaw += self.errors['azimuth_error']
            ic['attitude'] = [roll, pitch, yaw]

        # تطبيق أخطاء السرعة الزاوية
        angular_velocity = list(ic.get('angular_velocity', [0, 0, 0]))
        angular_velocity[1] += self.errors['elevation_rate_error']  # pitch rate
        angular_velocity[2] += self.errors['azimuth_rate_error']    # yaw rate
        ic['angular_velocity'] = angular_velocity

        self.config['initial_conditions'] = ic

        # Store effective burn time (nominal + error, clamped to valid range)
        # Use depletion_time (end of thrust curve) instead of burn_time (10% threshold)
        # to avoid cutting off the tail-off phase where thrust is still nonzero
        nominal_burn_time = self.propulsion.depletion_time
        burn_time_error = self.errors['burn_time_error']
        self.effective_burn_time = max(0.01, nominal_burn_time + burn_time_error)

        # تطبيق خطأ مقياس الدفع على نموذج الدفع
        current_scale = self.propulsion.thrust_scale
        self.propulsion.thrust_scale = current_scale * self.errors['thrust_scale_error']

        logger.info(f"  Effective burn time: {self.effective_burn_time:.3f}s (nominal: {nominal_burn_time:.3f}s)")
        logger.info(f"  Thrust scale: {self.propulsion.thrust_scale:.4f}")

        # تطبيق خطأ الكتلة على نموذج الديناميكيات
        mass_error = self.errors['mass_error']
        if not self.dynamics.use_mass_model:
            # Apply mass error to dry mass (affects both dry mass and total mass)
            original_mass_dry = self.dynamics.mass_dry
            new_mass_dry = max(0.1, original_mass_dry + mass_error)
            self.dynamics.mass_dry = new_mass_dry
            self.dynamics.mass_total = new_mass_dry + self.dynamics.mass_propellant
            # Update cached dry_mass used for clamping in _normalize_state
            self.dry_mass = new_mass_dry
            logger.info(f"  Mass error: {mass_error:.4f} kg (dry mass: {original_mass_dry:.2f} -> {new_mass_dry:.2f} kg)")
        else:
            # For mass model, apply error to the model's parameters
            if hasattr(self.dynamics.mass_model, 'M_body'):
                original_mass = self.dynamics.mass_model.M_body
                self.dynamics.mass_model.M_body = max(0.1, original_mass + mass_error)
                logger.info(f"  Mass error: {mass_error:.4f} kg (M_body: {original_mass:.2f} -> {self.dynamics.mass_model.M_body:.2f} kg)")

        # Store CG error for moment correction in _compute_dynamics
        self.cg_error = self.errors['cg_error']
        logger.info(f"  CG error: {self.cg_error:.4f} m")

        # Store sensor biases for measurement corruption
        self.sensor_biases = self.error_model.get_sensor_biases(self.errors)
        logger.info(f"  Accel bias: [{self.sensor_biases['accel_bias'][0]:.4f}, {self.sensor_biases['accel_bias'][1]:.4f}, {self.sensor_biases['accel_bias'][2]:.4f}] m/s²")
        logger.info(f"  Gyro bias: [{self.sensor_biases['gyro_bias'][0]:.4f}, {self.sensor_biases['gyro_bias'][1]:.4f}, {self.sensor_biases['gyro_bias'][2]:.4f}] rad/s")

    def _initialize_state(self):
        """
        Initialize state vector. | تهيئة متجه الحالة.
        long_range: NED→ECEF conversion. Attitude: quaternion, radians, or attitude_degrees.
        """

        x0_ned = self.config['initial_conditions'].get('position', [0, 0, 0])
        v0_ned = self.config['initial_conditions'].get('velocity', [0, 0, 0])

        attitude_degrees_config = self.config['initial_conditions'].get('attitude_degrees', None)
        attitude_config = self.config['initial_conditions'].get('attitude', [1, 0, 0, 0])

        if attitude_degrees_config is not None and len(attitude_degrees_config) == 3:
            roll_deg, pitch_deg, yaw_deg = attitude_degrees_config
            q0 = degrees_to_quaternion(roll_deg, pitch_deg, yaw_deg)
            logger.info(f"Initial attitude from degrees: roll={roll_deg}°, pitch={pitch_deg}°, yaw={yaw_deg}°")
        elif len(attitude_config) == 3:
            roll, pitch, yaw = attitude_config
            q0 = euler_to_quaternion(roll, pitch, yaw)
        elif len(attitude_config) == 4:
            q0 = attitude_config
        else:
            q0 = [1, 0, 0, 0]

        w0 = self.config['initial_conditions'].get('angular_velocity', [0, 0, 0])

        mass0 = self.dynamics.get_mass(0.0)

        if self.long_range_mode:
            C_ecef_to_ned = self.dynamics.earth_model.rot_ecef_to_ned(
                self.launch_lat_rad, self.launch_lon_rad
            )
            C_ned_to_ecef = C_ecef_to_ned.T

            pos_ned = np.array(x0_ned)
            pos_ecef_offset = C_ned_to_ecef @ pos_ned
            pos_ecef = self.launch_ecef + pos_ecef_offset

            vel_ned = np.array(v0_ned)
            vel_ecef = C_ned_to_ecef @ vel_ned

            state = np.array([
                pos_ecef[0], pos_ecef[1], pos_ecef[2],
                vel_ecef[0], vel_ecef[1], vel_ecef[2],
                q0[0], q0[1], q0[2], q0[3],
                w0[0], w0[1], w0[2],
                mass0
            ])
        else:
            state = np.array([
                x0_ned[0], x0_ned[1], x0_ned[2],
                v0_ned[0], v0_ned[1], v0_ned[2],
                q0[0], q0[1], q0[2], q0[3],
                w0[0], w0[1], w0[2],
                mass0
            ])

        return state

    def _handle_degenerate_quaternion(self, quaternion: np.ndarray, context: str) -> np.ndarray:
        """Handle degenerate quaternion (near-zero norm). NaN is always fatal.
        Behavior: "raise" raises DegenerateQuaternionError; "warn_and_recover" returns last valid quaternion.

        Args:
            quaternion: The degenerate quaternion array
            context: Where this occurred (for error messages)

        Raises:
            DegenerateQuaternionError: On "raise" mode, NaN input, or max recoveries exceeded
        """
        # Explicit NaN check: np.linalg.norm(NaN) returns NaN and NaN > threshold is False,
        # which would silently enter recovery path instead of raising.
        if not np.all(np.isfinite(quaternion)):
            error_msg = (
                f"NaN/Inf quaternion detected in {context}. "
                f"Quaternion: {quaternion}. "
                f"This indicates a CRITICAL numerical failure (NaN propagation). "
                f"NaN quaternions cannot be recovered - the simulation state is corrupted. "
                f"Check for: division by zero, sqrt of negative, overflow in forces/moments. "
                f"تم اكتشاف رباعي NaN/Inf في {context}. "
                f"الرباعي: {quaternion}. "
                f"هذا يشير إلى فشل عددي حرج (انتشار NaN). "
                f"لا يمكن استعادة رباعيات NaN - حالة المحاكاة فاسدة."
            )
            # NaN is ALWAYS fatal, regardless of degenerate_quaternion_handling setting
            raise DegenerateQuaternionError(error_msg)
        
        quat_norm = np.linalg.norm(quaternion)
        error_msg = (
            f"Degenerate quaternion detected in {context}. "
            f"Quaternion: {quaternion}, norm: {quat_norm:.2e} (threshold: {QUAT_NORM_THRESHOLD:.2e}). "
            f"This indicates a severe numerical problem in the simulation. "
            f"تم اكتشاف رباعي متدهور في {context}. "
            f"الرباعي: {quaternion}، norm: {quat_norm:.2e}. "
            f"هذا يشير إلى مشكلة عددية خطيرة في المحاكاة."
        )

        if self.degenerate_quaternion_handling == 'raise':
            raise DegenerateQuaternionError(error_msg)
        
        # warn_and_recover: atomic recovery + sync all quaternion references
        is_allowed, recovery_count, recovery_quat = self.quaternion_state_manager.perform_recovery(
            identity_fallback=True
        )
        # Determine recovery source for logging
        last_valid = self.quaternion_state_manager.get_recovery_quaternion()
        if last_valid is not None and np.allclose(recovery_quat, last_valid, atol=1e-10):
            recovery_source = "last valid quaternion / آخر رباعي صالح"
        else:
            recovery_source = "identity quaternion (no valid history) / رباعي الهوية (لا يوجد تاريخ صالح)"
        
        # Check if we've exceeded the maximum allowed recoveries
        if not is_allowed:
            raise DegenerateQuaternionError(
                f"{error_msg} "
                f"Maximum quaternion recoveries ({self.max_quaternion_recoveries}) exceeded. "
                f"Recovery count: {recovery_count}. "
                f"The simulation has too many numerical issues to continue safely. "
                f"تم تجاوز الحد الأقصى لاستعادات الرباعي ({self.max_quaternion_recoveries}). "
                f"المحاكاة لديها مشاكل عددية كثيرة جداً للاستمرار بأمان."
            )
        
        logger.warning(
            f"{error_msg} "
            f"Recovering with {recovery_source}. "
            f"Recovery count: {recovery_count}/{self.max_quaternion_recoveries}. "
            f"WARNING: Results may be non-physical! "
            f"الاستعادة باستخدام {recovery_source}. "
            f"عدد الاستعادات: {recovery_count}/{self.max_quaternion_recoveries}. "
            f"تحذير: النتائج قد تكون غير فيزيائية!"
        )
        
        return recovery_quat
    
    def _update_last_valid_quaternion(self, quaternion: np.ndarray) -> None:
        """Update last valid quaternion via QuaternionStateManager (recovery + sign continuity)."""
        # Shared QuaternionStateManager updates both recovery and sign-continuity states atomically
        if self.quaternion_state_manager.update_valid_quaternion(quaternion):
            # Initialize intermediate reference (prevent sign flip on first transformation)
            if self.quaternion_state_manager.get_intermediate_quaternion_reference() is None:
                self.quaternion_state_manager.set_intermediate_quaternion_reference(quaternion)

    def _conserve_angular_momentum_at_separation(
        self, I_old: np.ndarray, I_new: np.ndarray, omega_old: np.ndarray
    ) -> np.ndarray:
        """Compute new angular velocity to conserve L = I×ω at stage separation using ω_new = I_new⁻¹×I_old×ω_old."""
        # Compute angular momentum before separation
        L = I_old @ omega_old

        # Compute new angular velocity to conserve angular momentum
        # Using np.linalg.solve for better numerical stability than inv
        try:
            omega_new = np.linalg.solve(I_new, L)
        except np.linalg.LinAlgError:
            # If I_new is singular (should not happen for valid inertia), fall back
            logger.warning(
                "Failed to solve for new angular velocity (singular inertia matrix). "
                "Keeping original angular velocity. This may cause numerical issues."
            )
            return omega_old.copy()

        return omega_new

    def _effective_use_mass_model(self) -> bool:
        """Return True if mass model should be used; False when thrust scaling or error injection is active
        (must use integrated mass state[13] to keep F=ma consistent)."""
        if not self.dynamics.use_mass_model:
            return False

        # Check if thrust scaling is active
        thrust_scale_active = getattr(self.propulsion, 'thrust_scale', 1.0) != 1.0

        # Check if error injection modifies mass flow
        error_injection_active = self.use_error_injection

        # If either modifier is active, bypass mass model for consistency
        if thrust_scale_active or error_injection_active:
            # Log warning once per simulation
            if not hasattr(self, '_mass_model_bypass_warned'):
                self._mass_model_bypass_warned = True
                logger.warning(
                    "Mass properties model bypassed due to thrust scaling or error injection. "
                    "Using integrated mass (state[13]) and mass-based inertia for consistency. "
                    "تم تجاوز نموذج خصائص الكتلة بسبب تحجيم الدفع أو حقن الأخطاء. "
                    "استخدام الكتلة المتكاملة والعطالة المبنية على الكتلة للاتساق."
                )
            return False

        return True

    def _get_effective_mass(self, state, t, effective_use_mass_model):
        """
        Resolve the authoritative mass value based on the active mass model policy.

        Args:
            state: State array containing mass at index 13
            t: Current simulation time (s)
            effective_use_mass_model: Whether the mass model is authoritative

        Returns:
            float: Effective mass clamped to dry_mass
        """
        if effective_use_mass_model and t is not None:
            return max(self.dynamics.get_mass(t), self.dry_mass)

        return max(float(state[13]), self.dry_mass)

    def _get_consistent_mass_and_inertia(
        self, state: np.ndarray, t: float, mass_flow: float
    ) -> tuple:
        """Return (mass, I, I_dot, use_mass_based_inertia) from a single consistent source for F=ma."""
        effective_use_mass_model = self._effective_use_mass_model()
        
        # Get mass from the authoritative source
        mass = self._get_effective_mass(state, t, effective_use_mass_model)
        
        if effective_use_mass_model:
            # Nominal case: mass model uses time as independent variable
            I = self.dynamics.get_inertia_matrix(t)
            I_dot = self.dynamics.get_inertia_derivative(t)
            use_mass_based_inertia = False
        else:
            # Thrust scaling/error injection: use integrated mass (MUST match F=ma mass for consistency)
            I = self.dynamics.get_inertia_matrix_from_mass(mass)
            
            # Compute I_dot using adaptive finite difference
            remaining_propellant = mass - self.dry_mass
            
            if mass_flow > 0 and remaining_propellant > 0:
                # Adaptive step: small fraction of remaining propellant
                dm_target = min(0.01 * remaining_propellant, 0.005 * mass)
                dm_upper = 0.5 * remaining_propellant
                dm = min(dm_target, dm_upper)
                dm = max(1e-6, dm)
                if dm_upper > 0.0:
                    dm = min(dm, dm_upper)
                
                if dm <= 0.0:
                    I_dot = np.zeros((3, 3))
                elif remaining_propellant > 2 * dm:
                    # Central difference (more accurate)
                    I_plus = self.dynamics.get_inertia_matrix_from_mass(mass - dm)
                    I_minus = self.dynamics.get_inertia_matrix_from_mass(mass + dm)
                    dI_dm = (I_minus - I_plus) / (2 * dm)
                else:
                    # One-sided difference near dry mass
                    I_higher = self.dynamics.get_inertia_matrix_from_mass(mass + dm)
                    dI_dm = (I_higher - I) / dm
                
                I_dot = dI_dm * (-mass_flow)  # dm/dt = -mass_flow
                
                # Cap derivative to avoid anomalous values
                I_dot_norm = np.linalg.norm(I_dot)
                I_norm = np.linalg.norm(I)
                max_I_dot = 10.0 * I_norm / max(remaining_propellant / mass_flow, 0.1)
                if I_dot_norm > max_I_dot:
                    I_dot = I_dot * (max_I_dot / I_dot_norm)
            else:
                I_dot = np.zeros((3, 3))
            
            use_mass_based_inertia = True
        

        # Validation: Ensure mass and inertia are physically consistent
        self._validate_mass_inertia_consistency(mass, I, t)
        
        return mass, I, I_dot, use_mass_based_inertia

    def _validate_mass_inertia_consistency(self, mass: float, I: np.ndarray, t: float) -> None:
        """Check inertia matrix for NaN/Inf, positive diagonals, and reasonable scaling with mass."""
        # Check for NaN/Inf
        if not np.all(np.isfinite(I)):
            logger.error(
                f"Inertia matrix contains NaN/Inf at t={t:.3f}s. "
                f"Inertia: {I}. This will cause simulation failure. "
                f"مصفوفة العطالة تحتوي على NaN/Inf عند t={t:.3f}s. "
                f"العطالة: {I}. هذا سيسبب فشل المحاكاة."
            )
            return
        
        # Check diagonal elements are positive
        diag = np.diag(I)
        if np.any(diag <= 0):
            logger.error(
                f"Inertia matrix has non-positive diagonal at t={t:.3f}s. "
                f"Diagonal: {diag}. This is physically impossible. "
                f"مصفوفة العطالة لها قطر غير موجب عند t={t:.3f}s. "
                f"القطر: {diag}. هذا مستحيل فيزيائياً."
            )
            return
        
        # Check inertia-to-mass ratio is reasonable (expected: 1e-6 to 1000)
        if mass > 0:
            I_over_m = np.max(diag) / mass
            # Very loose bounds to catch obvious errors
            if I_over_m > 1000 or I_over_m < 1e-6:
                if not hasattr(self, '_inertia_ratio_warned'):
                    self._inertia_ratio_warned = True
                    logger.warning(
                        f"Unusual inertia-to-mass ratio at t={t:.3f}s: "
                        f"max(I_diag)/mass = {I_over_m:.4f}. "
                        f"Expected range: 1e-6 to 1000. Check mass and inertia configuration. "
                        f"نسبة عطالة-إلى-كتلة غير عادية عند t={t:.3f}s: "
                        f"max(I_diag)/mass = {I_over_m:.4f}. "
                        f"النطاق المتوقع: 1e-6 إلى 1000. تحقق من تكوين الكتلة والعطالة."
                    )

    def _get_frame_data(self, state, t=0.0):
        """
        Centralized altitude and frame data (single source of truth).
        Returns dict: altitude, lat, lon, C_ecef_to_ned, vel_ned, is_descending, etc.
        """
        position = state[0:3]
        velocity = state[3:6]

        frame_data = self.frame_manager.get_frame_data(position, velocity, t)
        if self.long_range_mode:
            pos_ecef = position
            vel_ecef = velocity
            pos_norm = np.linalg.norm(pos_ecef)
            if pos_norm > 0:
                radial_velocity = np.dot(vel_ecef, pos_ecef) / pos_norm
            else:
                radial_velocity = 0.0
            is_descending = radial_velocity < 0
        else:
            radial_velocity = -velocity[2]
            is_descending = velocity[2] > 0
        frame_data['radial_velocity'] = radial_velocity
        frame_data['is_descending'] = is_descending
        return frame_data

    def _compute_dynamics(self, state, t, control_input, update_state=True):
        """
        Compute state derivatives and diagnostic snapshot.
        update_state=False for RK4 intermediate stages (no side effects).
        """

        quaternion = state[6:10]
        quaternion_norm = np.linalg.norm(quaternion)
        if quaternion_norm > QUAT_NORM_THRESHOLD:  # Use module-level constant
            quaternion = quaternion / quaternion_norm
        else:
            quaternion = self._handle_degenerate_quaternion(
                quaternion, "_compute_dynamics: state quaternion"
            )

        position = state[0:3]
        velocity = state[3:6]
        angular_velocity = state[10:13]

        # When thrust scaling or error injection is active, use integrated mass
        # to ensure mass consumption matches F=ma calculations.
        effective_use_mass_model = self._effective_use_mass_model()
        mass = self._get_effective_mass(state, t, effective_use_mass_model)

        frame_data = self._get_frame_data(state, t)
        altitude = frame_data['altitude']
        lat = frame_data['lat']
        lon = frame_data['lon']
        C_ecef_to_ned = frame_data['C_ecef_to_ned']
        C_ned_to_ecef = frame_data['C_ned_to_ecef']
        vel_ned = frame_data['vel_ned']
        # In long_range_mode: this is ECEF velocity (used for Coriolis/centrifugal)
        # In non-long_range_mode: this is NED velocity (state frame)
        vel_ecef = frame_data['velocity_state_frame']  # Velocity in state frame (ECEF or NED)
        pos_ned_launch = frame_data.get('pos_ned_launch')  # Position in launch-fixed NED
        vel_ned_launch = frame_data.get('vel_ned_launch')  # Velocity in launch-fixed NED

        if self.long_range_mode:
            pos_ecef = position
            atm_vel_ecef = self.dynamics.earth_model.atmosphere_velocity_ecef(pos_ecef)
            C_ecef_to_ned_launch = frame_data['C_ecef_to_ned_launch']
            atm_vel_ned_launch = C_ecef_to_ned_launch @ atm_vel_ecef
            velocity_for_aero = vel_ned_launch - atm_vel_ned_launch
        else:
            velocity_for_aero = velocity

        atm = self.atmosphere.get_properties(altitude)

        # Subtract wind velocity to get true air-relative velocity
        # Wind is defined in local NED; convert to launch-fixed NED for consistency
        wind_ned = atm.get('wind_ned', np.zeros(3))
        if self.long_range_mode:
            C_ned_current_to_launch = frame_data.get('C_ned_current_to_launch')
            if C_ned_current_to_launch is not None:
                wind_ned = C_ned_current_to_launch @ wind_ned
        velocity_for_aero = velocity_for_aero - wind_ned

        airspeed = np.linalg.norm(velocity_for_aero)
        q_dyn = 0.5 * atm['density'] * airspeed * airspeed

        # Compute Mach number
        _current_mach = airspeed / atm['speed_of_sound'] if atm['speed_of_sound'] > 0 else 0.0

        prop = self.propulsion.get_thrust(
            t, ambient_pressure=atm['pressure'],
            mach=_current_mach,
            altitude=altitude,
            density=atm['density'],
            speed_of_sound=atm['speed_of_sound'],
            dt=self._current_dt,
            update_state=update_state
        )
        thrust = prop['thrust']
        motor_on = prop['motor_on']
        self._current_thrust = thrust  # حفظ الدفع الحالي لتمريره لنظام التحكم

        # Apply attitude mode control (تطبيق وضع التحكم بالاتجاه)
        # Use frame_manager for velocity alignment if available, otherwise fall back to local method
        if self.attitude_mode == 'velocity_aligned':
            # Always align with velocity vector (alpha ≈ 0, beta ≈ 0)
            # This minimizes aerodynamic loads throughout the flight
            quaternion = self.frame_manager.compute_velocity_aligned_quaternion(
                velocity_for_aero, quaternion
            )
            angular_velocity = np.zeros(3)
        elif self.attitude_mode == 'max_range':
            # Maximum range: velocity-aligned during thrust (drag only), free 6DOF after burnout (lift extends range)
            if update_state and not motor_on and not self._motor_has_burned_out:
                self._motor_has_burned_out = True
                logger.info(f"max_range: Motor burnout at t={t:.2f}s - switching to free 6DOF dynamics")
                logger.info(f"max_range: انطفاء المحرك عند t={t:.2f}s - التحول إلى ديناميكا 6DOF حرة")

            if motor_on:
                # During thrust: velocity-aligned (alpha ≈ 0) to minimize drag losses
                quaternion = self.frame_manager.compute_velocity_aligned_quaternion(
                    velocity_for_aero, quaternion
                )
                angular_velocity = np.zeros(3)
            # else: after burnout — do nothing, let 6DOF dynamics evolve naturally
        elif self.attitude_mode == '3dof':
                # 3DOF point-mass mode: always velocity-aligned with translational dynamics and drag only
            quaternion = self.frame_manager.compute_velocity_aligned_quaternion(
                velocity_for_aero, quaternion
            )
            angular_velocity = np.zeros(3)

        if (self.attitude_mode == 'none'
                and hasattr(self, '_current_stage_index')
                and self._current_stage_index == 0
                and self._initial_quaternion is not None):
            # Boost phase without cruise config: hold initial pitch (legacy fallback)
            quaternion = self._initial_quaternion.copy()
            angular_velocity = np.zeros(3)

        # mass_flow source: mass_model→dynamics (consistent w/ inertia), else→propulsion (Isp consistent)
        if effective_use_mass_model:
            # استخدام mass_flow من الديناميكيات (متسق مع العطالة/مركز الثقل)
            mass_flow = self.dynamics.get_mass_flow_rate(t)
        else:
            # استخدام mass_flow من الدفع (يحترم thrust_scale و scale_mass_flow_with_thrust)
            mass_flow = prop['mass_flow']

        # تطبيق خطأ وقت الاحتراق: قطع المحرك مبكراً أو تمديده (محدود بالاسمي)
        if self.use_error_injection and hasattr(self, 'effective_burn_time'):
            if t >= self.effective_burn_time:
                thrust = 0.0
                mass_flow = 0.0
                motor_on = False

        fin_authority = 1.0  # Full fin authority (safety monitor removed)
        scaled_control = control_input * fin_authority
        actuator_commanded = scaled_control.copy()  # حفظ الأوامر قبل تأخير السيرفو

        # max_range/3dof: zero fins to avoid CA_fin_total drag (uncontrolled ballistic)
        if self.attitude_mode in ('max_range', '3dof'):
            scaled_control = np.zeros(4)


        # Actuator dynamics (only on update_state=True; skipped in RK4 intermediate steps)
        if self.use_actuator_dynamics and self.actuator is not None:
            dt = getattr(self, '_current_dt', 0.01)
            if update_state:
                # Update actuator state and return actual deflections
                scaled_control = self.actuator.update(scaled_control, dt)
                
            else:
                # Use current deflections without state update (for intermediate steps)
                # This prevents integrator-dependent side effects
                scaled_control = self.actuator.delta_current.copy()

        # Get current CG position for moment transfer correction
        # This is critical for accurate stability margins as CG moves during propellant burn
        cg_position = self.dynamics.get_cg_location(t)
        
        aero = self.aerodynamics.get_forces_moments(
            velocity_for_aero, angular_velocity, quaternion,
            q_dyn, atm['density'], scaled_control,
            motor_on=motor_on,
            sound_speed=atm['speed_of_sound'],
            dynamic_viscosity=atm['dynamic_viscosity'],
            cg_position=cg_position
        )

        # Cruise diagnostic: print alpha, CA, CN, thrust every 1s after separation
        if (hasattr(self, '_current_stage_index') and self._current_stage_index > 0
                and abs(t - round(t)) < 0.006 and t < 30.0
                and update_state):
            _alpha_aero = np.degrees(aero.alpha) if hasattr(aero, 'alpha') else 0.0
            _CA = getattr(aero, 'CA', 0.0)
            _CN = getattr(aero, 'CN', 0.0)
            _alpha_cmd = getattr(self, '_cruise_alpha_cmd_deg', 0.0)
            print(f"[CRUISE t={t:.1f}] M={_current_mach:.2f} alt={altitude:.0f}m "
                  f"aoa={_alpha_aero:.1f}° cmd={_alpha_cmd:.1f}° "
                  f"CA={_CA:.4f} CN={_CN:.4f} "
                  f"T={thrust:.0f}N Fx={aero.forces[0]:.0f}N Fz={aero.forces[2]:.0f}N "
                  f"q={q_dyn:.0f}")

        thrust_vector = np.array([thrust, 0.0, 0.0])
        if self.use_error_injection and self.error_model is not None and thrust > 0:
            thrust_vector = self.error_model.apply_thrust_misalignment(
                thrust_vector, self.errors
            )

            # Apply aero scale errors to forces [F_axial, F_side, F_normal] and moments [M_roll, M_pitch, M_yaw]
        aero_forces = aero.forces.copy()
        aero_moments = aero.moments.copy()

        # Maximum range mode: Zero out lift, side forces, and moments during thrust
        # "Before motor burnout, only air resistance factor is taken"
        if self.attitude_mode == 'max_range' and motor_on:
            # Keep only axial force (drag) - index 0
            # Zero out side force (index 1) and normal force/lift (index 2)
            aero_forces[1] = 0.0  # Side force = 0
            aero_forces[2] = 0.0  # Normal force (lift) = 0
            # Zero out all aerodynamic moments
            aero_moments[:] = 0.0  # [M_roll, M_pitch, M_yaw] = 0

        # 3DOF mode: Zero out lift, side forces, and moments throughout ENTIRE flight
        # Point mass simulation - only drag (CA) is used, no lift or moments
        if self.attitude_mode == '3dof':
            # Keep only axial force (drag) - index 0
            # Zero out side force (index 1) and normal force/lift (index 2)
            aero_forces[1] = 0.0  # Side force = 0
            aero_forces[2] = 0.0  # Normal force (lift) = 0
            # Zero out all aerodynamic moments
            aero_moments[:] = 0.0  # [M_roll, M_pitch, M_yaw] = 0

        if self.use_error_injection and self.errors is not None:
            # Scale axial force (CA) - index 0
            aero_forces[0] *= self.errors['CA_scale_error']
            # Scale normal force (CN) - index 2
            aero_forces[2] *= self.errors['CN_scale_error']
            # Scale pitching moment (CM) - index 1
            aero_moments[1] *= self.errors['CM_scale_error']

                # Apply CG-shift moment correction: +X shift adds -cg_error*F_normal to pitch and +cg_error*F_side to yaw
            if hasattr(self, 'cg_error') and self.cg_error != 0.0:
                # CG error is in meters along body X-axis (positive = forward)
                # Moment correction: M_pitch += -cg_error * F_normal (F_z)
                #                    M_yaw += cg_error * F_side (F_y)
                aero_moments[1] += -self.cg_error * aero_forces[2]  # pitch moment
                aero_moments[2] += self.cg_error * aero_forces[1]   # yaw moment

        forces_body = thrust_vector + aero_forces
        moments_body = aero_moments

        R = self.dynamics.quaternion_to_rotation_matrix(quaternion)

            # Launch rail constraint: while on rail, override normal 6DOF with 1DOF motion along the rail axis
        on_rail = False
        if self.use_launcher:
            # Get rail axis in current NED frame
            if self.long_range_mode:
                C_ecef_to_ned_rail = self.dynamics.earth_model.ecef_to_ned_rotation(
                    self.launch_lat_rad, self.launch_lon_rad
                )
                rail_axis = self.launcher.get_rail_axis_ned(C_ecef_to_ned_rail)
                # Use launch-fixed NED position for rail distance calculation
                pos_ned_for_rail = pos_ned_launch
            else:
                rail_axis = self.launcher.get_rail_axis_ned()
                pos_ned_for_rail = position
            
            # Check if still on rail (pure function of state)
            on_rail = self.launcher.is_on_rail(pos_ned_for_rail, rail_axis)

            # Update launcher state only on accepted integration steps
            if update_state:
                self.launcher.update_state(t, pos_ned_for_rail, rail_axis)

        # Non-grav specific force (thrust+aero)/mass — NOT real accelerometer (no ground reaction)
        # Valid for flight phase detection after ignition, not on-pad simulation
        accel_body = forces_body / mass if mass > 0 else np.zeros(3)

        # Store CN_delta/CM_delta for logging using the maximum absolute fin deflection as delta
        if update_state:
            mach_for_delta = airspeed / atm['speed_of_sound'] if atm['speed_of_sound'] > 0 else 0.0
            alpha_deg_for_delta = np.degrees(aero.alpha) if hasattr(aero, 'alpha') else 0.0
            # Calculate maximum absolute fin deflection in degrees
            max_delta_rad = np.max(np.abs(control_input))
            max_delta_deg = np.degrees(max_delta_rad)
            CN_delta, CM_delta = self.aerodynamics._get_fin_effectiveness(
                mach_for_delta, alpha_deg_for_delta, delta_deg=max_delta_deg
            )
            self._last_CN_delta = CN_delta
            self._last_CM_delta = CM_delta

        # Standard uses NED with constant gravity; long-range uses ECEF with J2-J4/Coriolis, but rotational dynamics are unchanged.

        if self.long_range_mode:
            if mass > 0:
                forces_ned = R @ forces_body

                gravity_ecef = self.dynamics.earth_model.gravity_ecef(pos_ecef, self.dynamics.gravity_model)
                gravity_ned = C_ecef_to_ned @ gravity_ecef

                accel_ned = forces_ned / mass + gravity_ned

                if getattr(self, 'include_coriolis', True):
                    coriolis_ecef = self.dynamics.earth_model.coriolis_acceleration(vel_ecef)
                    coriolis_ned = C_ecef_to_ned @ coriolis_ecef
                    accel_ned = accel_ned + coriolis_ned

                if getattr(self, 'include_centrifugal', True):
                    centrifugal_ecef = self.dynamics.earth_model.centrifugal_acceleration(pos_ecef)
                    centrifugal_ned = C_ecef_to_ned @ centrifugal_ecef
                    accel_ned = accel_ned + centrifugal_ned

                accel_ecef = C_ned_to_ecef @ accel_ned

            else:
                accel_ned = np.zeros(3)
                accel_ecef = np.zeros(3)

            pos_dot = vel_ecef
            vel_dot = accel_ecef

            # Quaternion body→NED: launch-fixed (Earth rotation only) or local (+ transport rate)
            if getattr(self, 'use_launch_fixed_ned', True):
                # Launch-fixed NED: only Earth rotation rate, no transport rate
                # Use launch site latitude for Earth rotation rate calculation
                omega_ned_inertial = self.dynamics.earth_model.earth_rotation_rate_ned(
                    self.launch_lat_rad
                )
            else:
                # Local NED: full frame rotation rate (Earth + transport)
                omega_ned_inertial = self.dynamics.earth_model.ned_frame_rotation_rate(
                    lat, lon, altitude, vel_ned
                )
            omega_ned_body = R.T @ omega_ned_inertial
            omega_body_relative = angular_velocity - omega_ned_body
            quat_dot = self.dynamics.quaternion_derivative(quaternion, omega_body_relative)

            # Single source of truth: same mass for F=ma and inertia matrix
            consistent_mass, I, I_dot, _ = self._get_consistent_mass_and_inertia(
                state, t, mass_flow
            )
            
            # Verify mass consistency (should be identical since we use same source)
            if abs(consistent_mass - mass) > 1e-10:
                logger.warning(
                    f"Mass inconsistency detected in long_range_mode: "
                    f"mass={mass:.6f}, consistent_mass={consistent_mass:.6f}. "
                    f"Using consistent_mass for inertia calculations. "
                    f"تم اكتشاف عدم اتساق في الكتلة: mass={mass:.6f}, "
                    f"consistent_mass={consistent_mass:.6f}. "
                    f"استخدام consistent_mass لحسابات العطالة."
                )
            I_omega = I @ angular_velocity
            omega_cross_I_omega = np.cross(angular_velocity, I_omega)
            
            # Conditionally include İ·ω term based on configuration
            if self.dynamics._use_inertia_derivative:
                I_dot_omega = I_dot @ angular_velocity
            else:
                # Simplified Euler equation: ignore İ·ω term
                I_dot_omega = np.zeros(3)
            
            # Compute xbc, the CG offset used for moment correction.
            rocket_config = self.config.get('rocket', {})
            xcg_0 = rocket_config.get('xcg_0', 0.0)  # CG at full propellant
            xcg_end = rocket_config.get('xcg_end', 0.0)  # CG at burnout (dry)
            
            # Get propellant fraction consumed from mass properties model
            if hasattr(self, 'mass_properties_model') and self.mass_properties_model is not None:
                current_stage = self.mass_properties_model.get_current_stage()
                local_t = self.mass_properties_model.get_stage_local_time(t)
                fraction_consumed = current_stage.get_propellant_fraction_consumed(local_t)
                coef = 1.0 - fraction_consumed  # coef = propellant fraction remaining
                # xbc grows with propellant consumed from 0 to xcg_end - xcg_0.
                xbc = (xcg_end - xcg_0) * (1.0 - coef)
            else:
                coef = 1.0  # Default: full propellant (no offset)
                xbc = 0.0
            
            # Store xbc for history logging | تخزين xbc لسجل التاريخ
            self._last_xbc = xbc
            self._last_propellant_fraction = coef  # نسبة الوقود المتبقية | Propellant fraction remaining
            self._last_cg_position = cg_position  # موقع مركز الثقل (م) | CG position (m)

            # CG offset moment correction | تصحيح عزم إزاحة مركز الثقل
            # M_pitch = xbc × F_normal, M_yaw = -xbc × F_side
            M_cg_offset = np.array([
                0.0,                      # Roll moment (no contribution from xbc)
                xbc * forces_body[2],     # Pitch moment: xbc × F_normal
                -xbc * forces_body[1]     # Yaw moment: -xbc × F_side
            ])
            
            # Total moments including CG offset correction
            total_moments = moments_body + M_cg_offset
            
            # Euler equation with time-varying inertia (consistent with calculate_derivatives)
            angular_accel = np.linalg.solve(I, total_moments - omega_cross_I_omega - I_dot_omega)

            # Override rotational derivatives for modes that enforce attitude externally
            # This ensures RK4 intermediate states maintain the commanded attitude
            # 
            if self.attitude_mode in ('max_range', '3dof', 'velocity_aligned'):
                quat_dot = np.zeros(4)
                angular_accel = np.zeros(3)

            derivatives = np.concatenate([pos_dot, vel_dot, quat_dot, angular_accel])

        else:
            # Standard mode treats NED as inertial, so calculate_derivatives uses ω_body directly.
            use_mass_based_inertia = not self._effective_use_mass_model()
            
            # Calculate xbc (CG offset from reference point) for moment correction
            # xbc = (xcg_end - xcg_0) * (1 - coef)
            # where coef is propellant fraction remaining (1 at start, 0 at burnout)
            rocket_config = self.config.get('rocket', {})
            xcg_0 = rocket_config.get('xcg_0', 0.0)  # CG at full propellant
            xcg_end = rocket_config.get('xcg_end', 0.0)  # CG at burnout (dry)
            
            # Get propellant fraction consumed from mass properties model
            if hasattr(self, 'mass_properties_model') and self.mass_properties_model is not None:
                # Get current stage for propellant fraction calculation
                current_stage = self.mass_properties_model.get_current_stage()
                local_t = self.mass_properties_model.get_stage_local_time(t)
                fraction_consumed = current_stage.get_propellant_fraction_consumed(local_t)
                coef = 1.0 - fraction_consumed  # coef = propellant fraction remaining
            else:
                coef = 1.0  # Default: full propellant (no offset)
            
            # xbc increases from 0 to xcg_end - xcg_0 as propellant is depleted.
            xbc = (xcg_end - xcg_0) * (1.0 - coef)
            
            # Store xbc for history logging | تخزين xbc لسجل التاريخ
            self._last_xbc = xbc
            self._last_propellant_fraction = coef  # نسبة الوقود المتبقية | Propellant fraction remaining
            self._last_cg_position = cg_position  # موقع مركز الثقل (م) | CG position (m)

            derivatives = self.dynamics.calculate_derivatives(
                position, velocity, quaternion, angular_velocity,
                forces_body, moments_body, mass, t,
                use_mass_based_inertia=use_mass_based_inertia,
                mass_flow=mass_flow,
                dry_mass=self.dry_mass,
                xbc=xbc
            )

            # Zero rotational derivatives when attitude is externally enforced.
            if self.attitude_mode in ('max_range', '3dof', 'velocity_aligned'):
                derivatives[6:10] = 0.0  # quat_dot = 0
                derivatives[10:13] = 0.0  # angular_accel = 0

        # Constrain motion to the rail axis while the rocket is still on the launcher.
        if on_rail and self.use_launcher:
            # Compute constrained dynamics using launcher model
            if self.long_range_mode:
                # Use launch-fixed NED frame for rail constraint
                gravity_ned_rail = np.array([0.0, 0.0, self.dynamics.g])
                pos_dot_rail, vel_dot_rail, quat_dot_rail, angular_accel_rail, _ = \
                    self.launcher.compute_constrained_dynamics(
                        pos_ned_for_rail, vel_ned_launch, quaternion, mass,
                        thrust_vector, aero_forces, R, rail_axis, gravity_ned_rail
                    )
                # Convert constrained NED derivatives to ECEF
                pos_dot_ecef = C_ned_to_ecef @ pos_dot_rail
                vel_dot_ecef = C_ned_to_ecef @ vel_dot_rail
                derivatives = np.concatenate([pos_dot_ecef, vel_dot_ecef, quat_dot_rail, angular_accel_rail])
            else:
                # Standard mode: use NED frame directly
                gravity_ned_rail = np.array([0.0, 0.0, self.dynamics.g])
                pos_dot_rail, vel_dot_rail, quat_dot_rail, angular_accel_rail, _ = \
                    self.launcher.compute_constrained_dynamics(
                        position, velocity, quaternion, mass,
                        thrust_vector, aero_forces, R, rail_axis, gravity_ned_rail
                    )
                derivatives = np.concatenate([pos_dot_rail, vel_dot_rail, quat_dot_rail, angular_accel_rail])

        if self.long_range_mode:
            vertical_velocity = vel_ned[2]
        else:
            vertical_velocity = velocity[2]

        # Update flight phase only on accepted steps, not RK4 intermediate stages.
        if update_state:
            flight_phase = self.flight_phase_manager.update(
                t, state, thrust, accel_body, atm['density'], altitude,
                vertical_velocity=vertical_velocity
            )
        else:
            # For intermediate steps, use current phase without updating state machine
            flight_phase = self.flight_phase_manager.phase

        # mass_dot=0 when mass_model active (synced in _normalize_state), else integrate from flow
        if effective_use_mass_model:
            # Mass is managed by mass_properties model - don't integrate
            mass_dot = 0.0
        else:
            # Legacy mode or thrust scaling/error injection: integrate mass from mass flow
            mass_dot = -mass_flow
            if mass <= self.dry_mass and mass_dot < 0:
                mass_dot = 0.0

        derivatives = np.concatenate([derivatives, np.array([mass_dot])])

        # Compute felt G-load from body acceleration.
        g0 = 9.80665  # Standard gravity (m/s²)
        g_force_vector = accel_body / g0  # G-force components in body frame
        g_force_magnitude = np.linalg.norm(accel_body) / g0  # Total G-force magnitude
        g_force_axial = accel_body[0] / g0  # Axial G-force (along rocket axis)

        # Calculate Mach number from airspeed and speed of sound
        speed_of_sound = atm['speed_of_sound']
        mach = airspeed / speed_of_sound if speed_of_sound > 0 else 0.0

        # Get current stage number for multi-stage tracking
        if hasattr(self, '_current_stage_index'):
            stage_number = self._current_stage_index + 1  # 1-based for display
        else:
            stage_number = 1

        snapshot = {
            'time': t,
            'position': position.copy(),
            'velocity': velocity.copy(),
            'attitude': quaternion.copy(),
            'angular_velocity': angular_velocity.copy(),
            'mass': mass,
            'forces': forces_body.copy(),
            'thrust_vector': thrust_vector.copy(),
            'moments': moments_body.copy(),
            'flight_phase': flight_phase.name if hasattr(flight_phase, 'name') else str(flight_phase),
            'q_dynamic': q_dyn,
            'alpha': aero.alpha,
            'beta': aero.beta,  # Sideslip angle (rad) from aerodynamics | زاوية الانزلاق الجانبي من الديناميكا الهوائية
            'mach_aero': aero.mach,  # Mach number used in aero calculations (clamped) | رقم ماخ المستخدم في الحسابات الهوائية (مقيد)
            'fin_authority': fin_authority,
            'safety_violations': [],  # قائمة انتهاكات السلامة (فارغة افتراضياً)
            'g_force': g_force_magnitude,  # Total G-force magnitude
            'g_force_axial': g_force_axial,  # Axial G-force (along rocket axis)
            'g_force_vector': g_force_vector.copy(),  # G-force components [Gx, Gy, Gz]
            'airspeed': airspeed,  # Air-relative speed (m/s) - السرعة النسبية للهواء
            'speed_of_sound': speed_of_sound,  # Speed of sound (m/s) - سرعة الصوت
            'mach': mach,  # Mach number = airspeed / speed_of_sound - رقم ماخ
            'stage_number': stage_number,  # Current stage number (1-based) - رقم المرحلة
            'CA': aero.CA,  # Axial force coefficient (معامل القوة المحورية / السحب)
            'CN_total': aero.CN,  # Total normal force coefficient (معامل القوة العمودية الكلي)
            'CM_total': aero.CM,  # Total pitching moment coefficient (معامل عزم الميل الكلي)
            'CY_total': aero.CY,  # Total side force coefficient (معامل القوة الجانبية الكلي)
            'Cn_total': aero.Cn,  # Total yawing moment coefficient (معامل عزم الانعراج الكلي)
            'CN_delta': getattr(self, '_last_CN_delta', 0.0),  # Fin normal force effectiveness (فعالية القوة العمودية للزعانف)
            'CM_delta': getattr(self, '_last_CM_delta', 0.0),  # Fin pitching moment effectiveness (فعالية عزم الميل للزعانف)
            # Control coefficients from fin deflections | معاملات التحكم من انحرافات الزعانف
            'CN_control': aero.CN_control,  # Normal force control coefficient | معامل قوة التحكم العمودية
            'CM_control': aero.CM_control,  # Pitching moment control coefficient | معامل عزم التحكم في الميل
            'CY_control': aero.CY_control,  # Side force control coefficient | معامل قوة التحكم الجانبية
            'Cn_control': aero.Cn_control,  # Yawing moment control coefficient | معامل عزم التحكم في الانعراج
            # Raw aerodynamic moments from aerodynamics.py (before any modifications)
            'M_roll_aero': aero.M_roll,  # Roll moment (N-m) | عزم اللف
            'M_pitch_aero': aero.M_pitch,  # Pitch moment (N-m) | عزم الميل
            'M_yaw_aero': aero.M_yaw,  # Yaw moment (N-m) | عزم الانعراج
            'xbc': getattr(self, '_last_xbc', 0.0),  # CG offset from reference point (m) | إزاحة مركز الثقل
            'propellant_fraction': getattr(self, '_last_propellant_fraction', 1.0),  # نسبة الوقود المتبقية (1=ممتلئ, 0=فارغ)
            'cg_position': getattr(self, '_last_cg_position', 0.0),  # موقع مركز الثقل (م)
            'pos_ned_launch': pos_ned_launch.copy() if pos_ned_launch is not None else position.copy(),
            'vel_ned_launch': vel_ned_launch.copy() if vel_ned_launch is not None else velocity.copy(),
            'vel_ned': vel_ned.copy(),
            'velocity_for_aero': velocity_for_aero.copy(),
            'wind_ned': wind_ned.copy(),
            'control_fins_rad': scaled_control.copy(),
            'mpc_gamma_ref_rad': getattr(getattr(self, '_mpc_controller', None), '_gamma_ref_prev', 0.0),
            'mpc_chi_ref_rad': getattr(getattr(self, '_mpc_controller', None), '_chi_ref_prev', 0.0),
            # Actual tracking errors: نَحسبهم من vel_ned (frame-correct)
            # γ = arcsin(-v_d/|v|),  χ = atan2(v_e, v_n)
            'mpc_gamma_error_rad': float(
                np.arcsin(np.clip(-vel_ned[2] / max(np.linalg.norm(vel_ned), 1e-3), -1, 1))
                - getattr(getattr(self, '_mpc_controller', None), '_gamma_ref_prev', 0.0)
            ) if vel_ned is not None else 0.0,
            'mpc_chi_error_rad': float(
                ((np.arctan2(vel_ned[1], vel_ned[0])
                  - getattr(getattr(self, '_mpc_controller', None), '_chi_ref_prev', 0.0)
                  + np.pi) % (2*np.pi)) - np.pi
            ) if vel_ned is not None else 0.0,
            # أوامر MPC الافتراضية (قبل خلط الزعانف) | MPC virtual axis commands (before fin mixing)
            'mpc_delta_e': getattr(getattr(self, '_mpc_controller', None), '_last_delta_e', 0.0),
            'mpc_delta_r': getattr(getattr(self, '_mpc_controller', None), '_last_delta_r', 0.0),
            'mpc_delta_a': getattr(getattr(self, '_mpc_controller', None), '_last_delta_a', 0.0),
            # تشخيصات حل MPC | MPC solver diagnostics
            'mpc_solve_time_ms': getattr(getattr(self, '_mpc_controller', None), '_last_solve_time_ms', 0.0),
            'mpc_solver_status': getattr(getattr(self, '_mpc_controller', None), '_last_solver_status', 0),
            'mpc_sqp_iterations': getattr(getattr(self, '_mpc_controller', None), '_last_sqp_iterations', 0),
            # الانحرافات المأمورة قبل تأخير السيرفو | Commanded deflections before actuator lag
            'actuator_commanded_rad': actuator_commanded.copy(),
            # التسارع الزاوي من معادلة أويلر | Angular acceleration from Euler equation (rad/s^2)
            'angular_acceleration': derivatives[10:13].copy(),
            # العزوم الكلية بعد تصحيح إزاحة مركز الثقل | Total moments after CG offset correction (N·m)
            'total_moments': (moments_body + np.array([0.0, getattr(self, '_last_xbc', 0.0) * forces_body[2], -getattr(self, '_last_xbc', 0.0) * forces_body[1]])).copy(),
        }

        # إضافة قياسات الحساسات المنحازة إذا كان حقن الأخطاء مفعلاً
        if self.use_error_injection and hasattr(self, 'sensor_biases'):
            # قياس مقياس التسارع المنحاز (التسارع الحقيقي + الانحياز)
            accel_meas = accel_body + self.sensor_biases['accel_bias']
            # قياس الجيروسكوب المنحاز (السرعة الزاوية الحقيقية + الانحياز)
            gyro_meas = angular_velocity + self.sensor_biases['gyro_bias']

            snapshot['accel_measured'] = accel_meas.copy()
            snapshot['gyro_measured'] = gyro_meas.copy()
            snapshot['accel_bias'] = self.sensor_biases['accel_bias'].copy()
            snapshot['gyro_bias'] = self.sensor_biases['gyro_bias'].copy()

        if self.long_range_mode:
            # نَحفظ بالـ degrees (يَتطابق مع اسم العمود latitude_deg/longitude_deg في CSV)
            snapshot['position_lla'] = (np.degrees(lat), np.degrees(lon), altitude)
            ground_range = self.dynamics.earth_model.geodesic_distance(
                self.launch_lat_rad, self.launch_lon_rad, lat, lon
            )
            snapshot['ground_range_km'] = ground_range / 1000.0
            snapshot['altitude_km'] = altitude / 1000.0


        return derivatives, snapshot

    def _build_mhe_init_state(self, snapshot, t):
        """Build MHE initial state [17] from a simulation snapshot."""
        import math
        vel = np.asarray(snapshot['velocity'], dtype=float)
        V = max(np.linalg.norm(vel), 10.0)
        omega = np.asarray(snapshot['angular_velocity'], dtype=float)

        vn = snapshot.get('vel_ned_launch', snapshot.get('vel_ned', vel))
        vx, vy, vz = vn[0], vn[1], vn[2]
        v_h = math.sqrt(vx**2 + vy**2)
        gamma = math.atan2(-vz, max(v_h, 1.0))
        chi = math.atan2(vy, max(abs(vx), 1.0))

        quat = snapshot['attitude']
        from dynamics.quaternion_utils import quaternion_to_euler
        roll, pitch, yaw = quaternion_to_euler(quat)
        alpha = pitch - gamma
        beta = -(yaw - chi)

        pos = snapshot['position']
        if 'altitude_km' in snapshot:
            alt = snapshot['altitude_km'] * 1000.0
        else:
            alt = -pos[2]
        launch_alt = self.config.get('launch', {}).get('altitude', 1200.0)

        H_SCALE = 100.0
        X_SCALE = 1000.0
        Y_SCALE = 1000.0

        return np.array([
            V, gamma, chi,
            float(omega[0]), float(omega[1]), float(omega[2]),
            alpha, beta, float(roll),
            (alt - launch_alt) / H_SCALE,
            pos[0] / X_SCALE,
            pos[1] / Y_SCALE,
            0.0, 0.0, 0.0,  # gyro bias (unknown)
            0.0, 0.0,        # wind (unknown)
        ])

    def _should_append_history(self, new_time, min_time_delta=1e-6):
        """
        Check if history entry should be added (min time delta + output rate control).
        """
        if len(self.history['time']) == 0:
            return True

        last_time = self.history['time'][-1]
        time_delta = abs(new_time - last_time)

        # Check minimum time delta to avoid duplicate entries
        if time_delta < min_time_delta:
            logger.debug(
                "Skipping history entry at t=%.6fs (too close to last entry at t=%.6fs, delta=%.9fs)",
                new_time, last_time, time_delta
            )
            return False

        # Check output_interval if configured (from simulation.output_rate)
        if self.output_interval is not None:
            if time_delta < self.output_interval - min_time_delta:
                logger.debug(
                    "Skipping history entry at t=%.6fs (output_rate: interval=%.4fs, delta=%.6fs)",
                    new_time, self.output_interval, time_delta
                )
                return False

        return True

    def _append_history(self, snapshot, force=False):
        """
        Append a snapshot to the simulation history.

        Args:
            snapshot: Dictionary containing simulation state at a point in time
            force: If True, append regardless of time delta check (for final state)
        """
        if not force and not self._should_append_history(snapshot['time']):
            return

        self.history['time'].append(snapshot['time'])
        self.history['position'].append(snapshot['position'])
        self.history['velocity'].append(snapshot['velocity'])
        self.history['attitude'].append(snapshot['attitude'])
        self.history['angular_velocity'].append(snapshot['angular_velocity'])
        self.history['forces'].append(snapshot['forces'])
        self.history['thrust_vector'].append(snapshot.get('thrust_vector', np.zeros(3)))
        self.history['moments'].append(snapshot['moments'])
        self.history['flight_phase'].append(snapshot['flight_phase'])
        self.history['q_dynamic'].append(snapshot['q_dynamic'])
        self.history['alpha'].append(snapshot['alpha'])
        self.history['beta'].append(snapshot.get('beta', 0.0))  # Sideslip angle from aerodynamics
        self.history['mach_aero'].append(snapshot.get('mach_aero', 0.0))  # Mach used in aero calculations
        self.history['fin_authority'].append(snapshot['fin_authority'])
        self.history['safety_violations'].append(snapshot['safety_violations'])
        self.history['mass'].append(snapshot['mass'])

        # إضافة بيانات قوة G
        self.history['g_force'].append(snapshot['g_force'])
        self.history['g_force_axial'].append(snapshot['g_force_axial'])
        self.history['g_force_vector'].append(snapshot['g_force_vector'])

        # Add airspeed and Mach number data
        # Defensive handling: use .get() with fallback to prevent KeyError if snapshot is incomplete
        if 'airspeed' in snapshot:
            self.history['airspeed'].append(snapshot['airspeed'])
            self.history['speed_of_sound'].append(snapshot['speed_of_sound'])
            self.history['mach'].append(snapshot['mach'])
            self.history['stage_number'].append(snapshot['stage_number'])
        else:
            # Fallback: compute airspeed from velocity magnitude (ground-relative, less accurate)
            velocity = snapshot.get('velocity', np.zeros(3))
            fallback_airspeed = np.linalg.norm(velocity)
            fallback_sos = 340.0  # Sea-level approximation
            fallback_mach = fallback_airspeed / fallback_sos

            self.history['airspeed'].append(fallback_airspeed)
            self.history['speed_of_sound'].append(fallback_sos)
            self.history['mach'].append(fallback_mach)
            self.history['stage_number'].append(1)

            # Warn once about missing airspeed data
            if not hasattr(self, '_airspeed_warning_shown'):
                self._airspeed_warning_shown = True
                warnings.warn(
                    "Snapshot missing 'airspeed' key - using velocity magnitude as fallback. "
                    "This may indicate an outdated or partially updated rocket_6dof_sim.py. "
                    "Please ensure your code is up to date with 'git pull origin main'. "
                    "تحذير: snapshot يفتقد مفتاح 'airspeed' - استخدام مقدار السرعة كقيمة احتياطية. "
                    "قد يشير هذا إلى ملف rocket_6dof_sim.py قديم أو محدث جزئياً. "
                    "يرجى التأكد من تحديث الكود باستخدام 'git pull origin main'.",
                    RuntimeWarning
                )

        # Add axial force coefficient
        self.history['CA'].append(snapshot.get('CA', 0.0))

        # Add total aerodynamic coefficients
        self.history['CN_total'].append(snapshot.get('CN_total', 0.0))
        self.history['CM_total'].append(snapshot.get('CM_total', 0.0))
        self.history['CY_total'].append(snapshot.get('CY_total', 0.0))
        self.history['Cn_total'].append(snapshot.get('Cn_total', 0.0))
        
        # Add fin effectiveness coefficients
        self.history['CN_delta'].append(snapshot.get('CN_delta', 0.0))
        self.history['CM_delta'].append(snapshot.get('CM_delta', 0.0))
        
        # Add control coefficients from fin deflections
        self.history['CN_control'].append(snapshot.get('CN_control', 0.0))
        self.history['CM_control'].append(snapshot.get('CM_control', 0.0))
        self.history['CY_control'].append(snapshot.get('CY_control', 0.0))
        self.history['Cn_control'].append(snapshot.get('Cn_control', 0.0))
        
        # Add raw aerodynamic moments from aerodynamics.py
        self.history['M_roll_aero'].append(snapshot.get('M_roll_aero', 0.0))
        self.history['M_pitch_aero'].append(snapshot.get('M_pitch_aero', 0.0))
        self.history['M_yaw_aero'].append(snapshot.get('M_yaw_aero', 0.0))
        
        # Add CG offset from reference point
        self.history['xbc'].append(snapshot.get('xbc', 0.0))
        self.history['propellant_fraction'].append(snapshot.get('propellant_fraction', 1.0))
        self.history['cg_position'].append(snapshot.get('cg_position', 0.0))
        self.history['vel_ned_launch'].append(snapshot.get('vel_ned_launch', snapshot.get('velocity', np.zeros(3))))
        self.history['vel_ned'].append(snapshot.get('vel_ned', snapshot.get('velocity', np.zeros(3))))
        self.history['velocity_for_aero'].append(snapshot.get('velocity_for_aero', np.zeros(3)))
        self.history['wind_ned'].append(snapshot.get('wind_ned', np.zeros(3)))
        self.history['control_fins_rad'].append(snapshot.get('control_fins_rad', np.zeros(4)))
        self.history['mpc_gamma_ref_rad'].append(snapshot.get('mpc_gamma_ref_rad', 0.0))
        self.history['mpc_chi_ref_rad'].append(snapshot.get('mpc_chi_ref_rad', 0.0))
        # MPC virtual axis commands | أوامر MPC الافتراضية
        self.history['mpc_delta_e'].append(snapshot.get('mpc_delta_e', 0.0))
        self.history['mpc_delta_r'].append(snapshot.get('mpc_delta_r', 0.0))
        self.history['mpc_delta_a'].append(snapshot.get('mpc_delta_a', 0.0))
        # MPC solver diagnostics | تشخيصات حل MPC
        self.history['mpc_solve_time_ms'].append(snapshot.get('mpc_solve_time_ms', 0.0))
        self.history['mpc_solver_status'].append(snapshot.get('mpc_solver_status', 0))
        self.history['mpc_sqp_iterations'].append(snapshot.get('mpc_sqp_iterations', 0))
        # Tracking errors | أخطاء التتبع
        self.history['mpc_gamma_error_rad'].append(snapshot.get('mpc_gamma_error_rad', 0.0))
        self.history['mpc_chi_error_rad'].append(snapshot.get('mpc_chi_error_rad', 0.0))
        # Commanded deflections | الانحرافات المأمورة
        self.history['actuator_commanded_rad'].append(snapshot.get('actuator_commanded_rad', np.zeros(4)))
        # التسارع الزاوي | Angular acceleration (rad/s^2)
        self.history['angular_acceleration'].append(snapshot.get('angular_acceleration', np.zeros(3)))
        # العزوم الكلية بعد تصحيح CG | Total moments after CG offset correction (N·m)
        self.history['total_moments'].append(snapshot.get('total_moments', np.zeros(3)))

        if self.long_range_mode:
            self.history['position_lla'].append(snapshot.get('position_lla', (0, 0, 0)))
            self.history['ground_range_km'].append(snapshot.get('ground_range_km', 0.0))
            self.history['altitude_km'].append(snapshot.get('altitude_km', 0.0))

        # Add biased sensor measurements and their biases if available
        if 'accel_measured' in snapshot:
            self.history['accel_measured'].append(snapshot['accel_measured'])
            self.history['gyro_measured'].append(snapshot['gyro_measured'])
            # Store sensor biases for analysis (تخزين انحيازات الحساسات للتحليل)
            self.history['accel_bias'].append(snapshot.get('accel_bias', np.zeros(3)))
            self.history['gyro_bias'].append(snapshot.get('gyro_bias', np.zeros(3)))

        # MHE estimation data (تسجيل بيانات تقدير MHE)
        if 'mhe_x_hat' in snapshot:
            self.history['mhe_x_hat'].append(snapshot['mhe_x_hat'])
            self.history['mhe_quality'].append(snapshot['mhe_quality'])
            self.history['mhe_solve_ms'].append(snapshot['mhe_solve_ms'])

    def _integrate_one_step(self, state, t, dt, control_function, *,
                             update_state=True):
        """
        Advance state by one RK4 step.

        Args:
            state: Current state array
            t: Current time (s)
            dt: Time step (s)
            control_function: Control function for fin deflections
            update_state: If True (default), update internal state (flight phase, motor burnout).
                         Set to False for bisection/interpolation to avoid side effects.

        Returns:
            tuple: (next_state, snapshot, t_end, None)
        """

        self._current_dt = dt

        state_dict = self._state_array_to_dict(state, t)
        control = control_function(state_dict, t)
        k1, snapshot = self.state_derivative(state, t, control, update_state=update_state)

        next_state = self._rk4_step(state, t, dt, control_function, k1)

        return next_state, snapshot, t + dt, None

    def _rk4_step(self, state, t, dt, control_function, k1):
        """
        Explicit RK4 step with control-law sampling.

        Note: k1 is computed in _integrate_one_step with update_state=True (default).
        k2, k3, k4 are computed here with update_state=False to avoid side effects
        during intermediate RK4 stages.
        """

        state2 = state + 0.5 * dt * k1
        state2_dict = self._state_array_to_dict(state2, t + 0.5 * dt)
        control2 = control_function(state2_dict, t + 0.5 * dt)
        k2, _ = self.state_derivative(state2, t + 0.5 * dt, control2, update_state=False)

        state3 = state + 0.5 * dt * k2
        state3_dict = self._state_array_to_dict(state3, t + 0.5 * dt)
        control3 = control_function(state3_dict, t + 0.5 * dt)
        k3, _ = self.state_derivative(state3, t + 0.5 * dt, control3, update_state=False)

        state4 = state + dt * k3
        state4_dict = self._state_array_to_dict(state4, t + dt)
        control4 = control_function(state4_dict, t + dt)
        k4, _ = self.state_derivative(state4, t + dt, control4, update_state=False)

        next_state = state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

        return next_state

    def _normalize_state(self, state, t=None):
        """
        Normalize quaternion, sync mass with model, apply attitude mode.
        تطبيع الرباعي، مزامنة الكتلة مع النموذج، تطبيق وضع التوجيه.
        """
        quat_norm = np.linalg.norm(state[6:10])
        if quat_norm > QUAT_NORM_THRESHOLD:  # Use module-level constant
            state[6:10] = state[6:10] / quat_norm
            self._update_last_valid_quaternion(state[6:10])
        else:
            state[6:10] = self._handle_degenerate_quaternion(
                state[6:10], "_normalize_state: state quaternion"
            )

        # Synchronize state mass with the model, or clamp it to dry mass.
        effective_use_mass_model = self._effective_use_mass_model()
        if effective_use_mass_model and t is not None:
            # Synchronize state[13] with authoritative mass from model
            model_mass = self.dynamics.get_mass(t)
            state[13] = max(model_mass, self.dry_mass)
        elif state[13] < self.dry_mass:
            if not hasattr(self, '_mass_clamp_warned') or not self._mass_clamp_warned:
                logger.warning(
                    "Mass (%.2f kg) fell below dry_mass (%.2f kg) and was clamped. "
                    "This may indicate an error in thrust curve or fuel consumption calculations. "
                    "الكتلة انخفضت تحت الكتلة الجافة وتم تثبيتها. قد يشير هذا إلى خطأ في منحنى الدفع.",
                    state[13], self.dry_mass
                )
                self._mass_clamp_warned = True
            state[13] = self.dry_mass
        else:
            state[13] = max(state[13], self.dry_mass)

        # Apply attitude mode control (تطبيق وضع التحكم بالاتجاه)
        if self.attitude_mode in ('velocity_aligned', 'max_range', '3dof'):
            position = state[0:3]
            velocity = state[3:6]
            previous_quaternion = state[6:10].copy()

            # velocity_aligned / 3dof: always align
            # max_range: only during thrust (after burnout → free 6DOF)
            should_align = (self.attitude_mode == 'velocity_aligned' or
                           self.attitude_mode == '3dof' or
                           (self.attitude_mode == 'max_range' and not self._motor_has_burned_out))

            if should_align:
                frame_data = self._get_frame_data(state)
                altitude = frame_data['altitude']
                vel_ned = frame_data['vel_ned']
                C_ecef_to_ned = frame_data['C_ecef_to_ned']

                if self.long_range_mode:
                    pos_ecef = position
                    atm_vel_ecef = self.dynamics.earth_model.atmosphere_velocity_ecef(pos_ecef)
                    atm_vel_ned = C_ecef_to_ned @ atm_vel_ecef
                    velocity_for_aero = vel_ned - atm_vel_ned
                else:
                    velocity_for_aero = velocity.copy()

                atm = self.atmosphere.get_properties(altitude)
                wind_ned = atm.get('wind_ned', np.zeros(3))
                velocity_for_aero = velocity_for_aero - wind_ned

                aligned_quat = self.frame_manager.compute_velocity_aligned_quaternion(
                    velocity_for_aero, previous_quaternion
                )
                state[6:10] = aligned_quat

            # Zero angular velocity when attitude is being forced
            # Exception: max_range after burnout allows free 6DOF dynamics
            if not (self.attitude_mode == 'max_range' and self._motor_has_burned_out):
                state[10:13] = np.zeros(3)

    def _state_array_to_dict(self, state_array, t=None):
        """
        Convert state array to dict for control function. | تحويل مصفوفة الحالة لقاموس.
        Quaternion is normalized (important for RK4 intermediate states).
        Mass from dynamics.get_mass(t) when effective_use_mass_model=True, else state_array[13].
        """
        quaternion = state_array[6:10].copy()
        quat_norm = np.linalg.norm(quaternion)
        if quat_norm > QUAT_NORM_THRESHOLD:  # Use module-level constant
            quaternion = quaternion / quat_norm
        else:
            quaternion = self._handle_degenerate_quaternion(
                quaternion, "_state_array_to_dict: state quaternion"
            )

        effective_use_mass_model = self._effective_use_mass_model()
        if effective_use_mass_model and t is not None:
            # استخدام نموذج الكتلة (نفس المصدر المستخدم في الحسابات الفيزيائية)
            mass = max(self.dynamics.get_mass(t), self.dry_mass)
        elif len(state_array) > 13:
            mass = float(state_array[13])
        else:
            mass = self.dry_mass

        result = {
            'position': state_array[0:3].copy(),
            'velocity': state_array[3:6].copy(),
            'quaternion': quaternion,
            'angular_velocity': state_array[10:13].copy(),
            'mass': mass,
            'thrust': getattr(self, '_current_thrust', 0.0),  # الدفع الحالي لنظام التحكم
        }

        # تمرير مواقع المشغل إلى نظام التحكم (MPC)
        # Use get_feedback() if available: returns encoder-quantized values
        # matching what the MHE/EKF observer actually sees via CAN SDO feedback.
        # Falls back to exact delta_current for older actuator models.
        if self.actuator is not None:
            if hasattr(self.actuator, 'get_feedback'):
                result['actuator_positions'] = self.actuator.get_feedback()
            else:
                result['actuator_positions'] = self.actuator.delta_current.copy()

        if self.long_range_mode:
            frame_data = self._get_frame_data(state_array)
            lat = frame_data.get('lat', self.launch_lat_rad)
            lon = frame_data.get('lon', self.launch_lon_rad)
            alt = frame_data.get('altitude', -state_array[2])
            ground_range = self.dynamics.earth_model.geodesic_distance(
                self.launch_lat_rad, self.launch_lon_rad, lat, lon
            )
            result['altitude_km']     = alt / 1000.0
            result['ground_range_km'] = ground_range / 1000.0
            result['vel_ned']         = frame_data.get('vel_ned', state_array[3:6].copy())
            result['vel_ned_launch']  = frame_data.get('vel_ned_launch', state_array[3:6].copy())
            result['C_ecef_to_ned']   = frame_data.get('C_ecef_to_ned', np.eye(3))

        return result

    def _get_altitude(self, state):
        """
        Get altitude from state based on simulation mode.

        Args:
            state: State array [pos, vel, quat, omega, mass]

        Returns:
            float: Altitude in meters
                   - long_range_mode: altitude above WGS-84 ellipsoid
                   - standard mode: -z (NED frame, z points down)
        """
        frame_data = self._get_frame_data(state)
        return frame_data['altitude']

    def _is_descending(self, state):
        """
        Check if rocket is descending.

        Args:
            state: State array

        Returns:
            bool: True if descending
        """
        frame_data = self._get_frame_data(state)
        return frame_data['is_descending']

    # Stage separation event detection.

    def _separation_event_value(self, state, t):
        """
        Compute event function value for stage separation detection.

        Returns a value that:
        - Is positive when separation should NOT occur
        - Is negative (or zero) when separation SHOULD occur
        - Crosses zero at the exact separation time

        Args:
            state: Current state array
            t: Current time (s)

        Returns:
            float: Event function value (positive = no separation, negative = separation)
        """
        if self.multi_stage_data is None:
            return 1.0

        if not hasattr(self, 'flight_phase_manager'):
            return 1.0

        fpm = self.flight_phase_manager

        altitude = self._get_altitude(state)
        velocity = np.linalg.norm(state[3:6])

        if hasattr(self, 'propulsion') and self.propulsion is not None:
            atm = self.atmosphere.get_properties(altitude)
            _sep_airspeed = velocity  # velocity is scalar here (np.linalg.norm(state[3:6]))
            _sep_mach = _sep_airspeed / atm['speed_of_sound'] if atm['speed_of_sound'] > 0 else 0.0
            prop = self.propulsion.get_thrust(
                t, ambient_pressure=atm['pressure'],
                mach=_sep_mach,
                altitude=altitude,
                density=atm['density'],
                speed_of_sound=atm['speed_of_sound'],
                dt=self._current_dt,
                update_state=False
            )
            thrust = prop['thrust']
        else:
            thrust = 0.0

        return fpm.get_separation_event_value(t, thrust, altitude, velocity)

    def _detect_separation_event(self, prev_state, prev_t, state, t):
        """
        Check if a separation event occurred during the integration step.

        A separation event is detected when the event function value
        changes sign from positive to negative (or zero).

        Args:
            prev_state: State before integration step
            prev_t: Time before integration step
            state: State after integration step
            t: Time after integration step

        Returns:
            bool: True if separation event occurred during step
        """
        if self.multi_stage_data is None:
            return False

        prev_value = self._separation_event_value(prev_state, prev_t)
        curr_value = self._separation_event_value(state, t)

        # Event occurs when value crosses from positive to non-positive
        return prev_value > 0 and curr_value <= 0

    def _interpolate_separation_point(self, state_before, t_before, state_after, t_after,
                                       control_function, tol=1e-4, max_iter=20):
        """
        Find the stage-separation point between two states using bisection.

        Args:
            state_before: State array before separation (event value > 0)
            t_before: Time before separation
            state_after: State array after separation (event value <= 0)
            t_after: Time after separation
            control_function: Control function for integration
            tol: Time convergence tolerance in seconds
            max_iter: Maximum bisection iterations

        Returns:
            tuple: (separation_state, separation_time)
        """
        t_low = t_before
        t_high = t_after
        state_low = state_before.copy()

        event_low = self._separation_event_value(state_low, t_low)
        event_high = self._separation_event_value(state_after, t_high)

        logger.info(
            "Starting separation interpolation: t=[%.6f, %.6f]s, event=[%.4f, %.4f]",
            t_low, t_high, event_low, event_high
        )

        state_mid = state_low.copy()
        t_mid = t_low

        for iteration in range(max_iter):
            # Bisection: try midpoint
            # التنصيف: جرب نقطة المنتصف
            t_mid = (t_low + t_high) / 2.0
            dt_to_mid = t_mid - t_low

            if dt_to_mid < 1e-10:
                # Time interval too small, use current state
                break

            # Integrate from state_low to t_mid
            state_mid, _, _, _ = self._integrate_one_step(
                state_low, t_low, dt_to_mid, control_function,
                update_state=False
            )
            self._normalize_state(state_mid, t_mid)

            event_mid = self._separation_event_value(state_mid, t_mid)

            # Check convergence (time tolerance)
            if (t_high - t_low) < tol:
                logger.debug(
                    "Separation interpolation converged in %d iterations: "
                    "t=%.6fs, event=%.6f",
                    iteration + 1, t_mid, event_mid
                )
                # Return state just before separation (event > 0)
                if event_mid > 0:
                    return state_mid, t_mid
                else:
                    return state_low, t_low

            # Update bounds based on event value
            if event_mid > 0:
                # Still before separation, move lower bound up
                t_low = t_mid
                state_low = state_mid.copy()
                event_low = event_mid
            else:
                # After separation, move upper bound down
                t_high = t_mid
                event_high = event_mid

        # If max iterations reached, return best estimate
        logger.warning(
            "Separation interpolation did not converge after %d iterations. "
            "Final time interval: %.6fs",
            max_iter, t_high - t_low
        )

        # Return state just before separation
        if self._separation_event_value(state_mid, t_mid) > 0:
            return state_mid, t_mid
        else:
            return state_low, t_low

    def _execute_separation_at_time(self, state, t):
        """
        Execute stage separation at the precise time and reinitialize models.

        This method:
        1. Executes the separation via FlightPhaseManager
        2. Updates mass properties models for the new stage
        3. Updates state[13] with the new stage mass
        4. Logs the separation event

        Args:
            state: Current state array (will be modified in place)
            t: Separation time (s)
        Returns:
            bool: True if separation was executed successfully
        """
        if self.multi_stage_data is None:
            return False

        fpm = self.flight_phase_manager

        # Time guard to prevent duplicate separation execution.
        SEPARATION_TIME_EPSILON = 1e-9
        if hasattr(self, '_last_separation_time') and self._last_separation_time >= 0:
            if t <= self._last_separation_time + SEPARATION_TIME_EPSILON:
                logger.debug(
                    "Separation time guard triggered: t=%.9fs <= last_separation_time=%.9fs + eps. "
                    "Separation already executed at this time.",
                    t, self._last_separation_time
                )
                return False

        # Precondition: Verify stage indices are synchronized before separation
        expected_fpm_stage = self._current_stage_index + 1
        if fpm.current_stage != expected_fpm_stage:
            logger.warning(
                "Stage index mismatch before separation: FPM stage=%d, expected=%d (sim index=%d). "
                "This may indicate a synchronization issue. Aborting separation.",
                fpm.current_stage, expected_fpm_stage, self._current_stage_index
            )
            return False

        # Guard against re-running separation for an already-advanced stage.
        fpm_stage_idx = fpm.current_stage - 1
        if fpm_stage_idx != self._last_separation_check_stage_idx:
            logger.debug(
                "Separation guard triggered: FPM stage index (%d) != last check stage index (%d). "
                "Separation may have already been executed this step.",
                fpm_stage_idx, self._last_separation_check_stage_idx
            )
            return False

        # Execute separation in FlightPhaseManager
        if not fpm.execute_separation(t):
            return False

        # Complete the separation process
        fpm.complete_separation(t)

        # Get new stage index (0-based)
        new_stage_index = fpm.current_stage - 1

        # Calculate ignition time including ignition delay
        ignition_delay = fpm.get_ignition_delay(new_stage_index)
        ignition_time = t + ignition_delay

        # Start the new stage timeline at ignition_time (sync handled in _switch_to_stage).
        fpm.stage_start_time = ignition_time
        logger.debug(f"FlightPhaseManager: stage_start_time set to {ignition_time:.4f}s (ignition_delay={ignition_delay:.3f}s)")

        if new_stage_index < len(self.multi_stage_data.stages):
            new_stage = self.multi_stage_data.stages[new_stage_index]

            # Update state mass to new stage initial mass
            if new_stage.rocket_data.properties is not None:
                props = new_stage.rocket_data.properties
                new_mass = props.mass_dry_kg + props.propellant_mass_kg
                old_mass = state[13]
                state[13] = new_mass

                logger.info(
                    "=== STAGE SEPARATION at t=%.4fs ===\n"
                    "  Previous mass: %.2f kg\n"
                    "  New stage mass: %.2f kg\n"
                    "  Mass change: %.2f kg (jettisoned)",
                    t, old_mass, new_mass, old_mass - new_mass
                )

            # Switch to new stage models
            if not self._switch_to_stage(new_stage_index, t, state):
                logger.warning(
                    "Stage switch failed for stage %d at t=%.4fs. "
                    "Not updating separation tracking variables.",
                    new_stage_index, t
                )
                return False

            # Update separation tracking after a successful switch (0-based stage index).
            self._last_separation_check_stage_idx = fpm.current_stage - 1
            if hasattr(self, '_last_separation_time'):
                self._last_separation_time = t

            # Post-condition: Verify stage indices are synchronized after separation
            expected_fpm_stage_after = self._current_stage_index + 1
            if fpm.current_stage != expected_fpm_stage_after:
                logger.warning(
                    "Stage index mismatch after separation: FPM stage=%d, expected=%d (sim index=%d). "
                    "This may indicate a synchronization issue.",
                    fpm.current_stage, expected_fpm_stage_after, self._current_stage_index
                )

            return True

        return False

    def _interpolate_impact_point(self, state_before, t_before, t_after,
                                   control_function, tol=0.01, max_iter=20):
        """
        Bisection to find precise impact point. Error: O(v*dt) → O(v*tol).
        تنصيف للعثور على نقطة الاصطدام الدقيقة.

        Returns:
            tuple: (impact_state, impact_time, impact_altitude)
        """
        t_low = t_before
        t_high = t_after
        state_low = state_before.copy()

        for iteration in range(max_iter):
            # Bisection: try midpoint
            t_mid = (t_low + t_high) / 2.0
            dt_to_mid = t_mid - t_low

            # Integrate from state_low to t_mid
            state_mid, _, _, _ = self._integrate_one_step(
                state_low, t_low, dt_to_mid, control_function,
                update_state=False
            )
            self._normalize_state(state_mid, t_mid)

            alt_mid = self._get_altitude(state_mid)
            frame_data_mid = self._get_frame_data(state_mid, t_mid)
            target_alt = self._get_impact_altitude_reference_m(frame_data_mid)

            # Check convergence
            if abs(alt_mid - target_alt) < tol:
                logger.debug(
                    "Impact interpolation converged in %d iterations: "
                    "t=%.6fs, alt=%.4fm (target=%.1fm)",
                    iteration + 1, t_mid, alt_mid, target_alt
                )
                return state_mid, t_mid, alt_mid

            # Update bounds based on altitude
            if alt_mid > target_alt:
                # Still above ground, move lower bound up
                t_low = t_mid
                state_low = state_mid
            else:
                # Below ground, move upper bound down
                t_high = t_mid

        # If max iterations reached, return best estimate (midpoint)
        logger.warning(
            "Impact interpolation did not converge after %d iterations. "
            "Final altitude error: %.4fm",
            max_iter, abs(alt_mid - target_alt)
        )
        return state_mid, t_mid, alt_mid

    def _get_impact_altitude_reference_m(self, frame_data: Dict[str, Any] = None) -> float:
        """
        Impact altitude reference = target altitude (constant).
        """
        if self.long_range_mode:
            sim_cfg = self.config.get('simulation', {})
            if 'long_range_impact_altitude_m' in sim_cfg:
                return float(sim_cfg['long_range_impact_altitude_m'])
            return self.target_altitude_m
        return 0.0

    def _detect_ground_impact(self, state, t):
        """
        long_range: ECEF→LLA vs target alt | standard: NED z>0 with tolerance.
        """

        # Use centralized frame data computation
        frame_data = self._get_frame_data(state)
        altitude = frame_data['altitude']
        is_descending = frame_data['is_descending']

        if self.long_range_mode:
            # Liftoff detection uses altitude above LAUNCH (not target) to avoid false impact
            if not hasattr(self, '_has_lifted_off_lr'):
                self._has_lifted_off_lr = False
                self._max_altitude_above_launch = 0.0
                self._prev_altitude_above_target_lr = None
                self._prev_altitude_lr = None
                self._has_been_above_impact_ref = False

            impact_alt_ref = self._get_impact_altitude_reference_m(frame_data)

            # Calculate altitude above impact reference (for impact detection)
            altitude_above_target = altitude - impact_alt_ref

            # Calculate altitude above launch (for liftoff detection)
            altitude_above_launch = altitude - self.launch_alt_m

            # Update max altitude above launch
            if altitude_above_launch > self._max_altitude_above_launch:
                self._max_altitude_above_launch = altitude_above_launch

            # Liftoff = altitude >= threshold OR phase > SAFE_ON_PAD (handles shallow trajectories)
            if altitude_above_launch >= self.liftoff_threshold:
                self._has_lifted_off_lr = True

            # Also treat powered-ascent phase as liftoff for shallow trajectories.
            if hasattr(self, 'flight_phase_manager'):
                current_phase = self.flight_phase_manager.phase
                # Treat phase >= POWERED_ASCENT as confirmed liftoff.
                if current_phase is not None and current_phase.value >= 4:
                    self._has_lifted_off_lr = True

            # تتبع: هل ارتفع الصاروخ فوق مرجع الاصطدام؟ (حماية حالة الهدف أعلى من الإطلاق)
            if altitude_above_target > 0:
                self._has_been_above_impact_ref = True

            ground_tolerance_lr = 0.1  # 10 cm tolerance / سماحية 10 سم

            impact_detected = False
            if (self._has_lifted_off_lr and
                self._has_been_above_impact_ref and
                self._max_altitude_above_launch > self.liftoff_threshold):
                descending_trend = (
                    is_descending or
                    (self._prev_altitude_lr is not None and
                     altitude < self._prev_altitude_lr - 1e-3)
                )
                if (altitude_above_target <= ground_tolerance_lr and
                    descending_trend):
                    impact_detected = True

            # Update previous altitude for next iteration
            self._prev_altitude_above_target_lr = altitude_above_target
            self._prev_altitude_lr = altitude

            if impact_detected:
                logger.warning(
                    "Ground impact detected at t=%.2fs, altitude=%.1fm (impact_ref=%.1fm)",
                    t, altitude, impact_alt_ref
                )
                return True


        else:
            # Avoids false triggers during launch settling vs actual impact after flight
            raw_altitude = -state[2]  # NED: altitude = -z (can be negative)

            # Track if rocket has achieved significant altitude (liftoff detection)
            if not hasattr(self, '_has_lifted_off'):
                self._has_lifted_off = False
                self._max_altitude_achieved = 0.0

            # Update max altitude and liftoff status
            if raw_altitude > self._max_altitude_achieved:
                self._max_altitude_achieved = raw_altitude

            # Liftoff is confirmed when altitude exceeds the configurable threshold.
            if raw_altitude > self.liftoff_threshold:
                self._has_lifted_off = True

            # Ground impact requires prior liftoff, below-ground state, and descent.

            ground_tolerance = 0.1  # 10 cm tolerance / سماحية 10 سم

            if self._has_lifted_off and state[2] > ground_tolerance and is_descending:
                logger.warning("Ground impact detected at t=%.2fs", t)
                return True

        return False

    def state_derivative(self, state, t, control_input, update_state=True):
        """
        Return derivatives and diagnostics for integrator.

        Args:
            state: State vector
            t: Current time (s)
            control_input: Fin deflection commands array [δ1, δ2, δ3, δ4] (rad)
            update_state: If True, update internal state (flight phase, motor burnout).
                         Set to False for RK4 intermediate stages to avoid side effects.
        """
        return self._compute_dynamics(state, t, control_input, update_state=update_state)

    def simulate(self, duration, dt=0.01, control_function=None,
                 on_step=None, callback_stride=100):
        """Run simulation and return history.

        Args:
            duration: Total simulation time (s).
            dt: Output time step (s).
            control_function: Optional f(state_dict, t) -> np.array[4] fin deflections (rad).
            on_step: Optional progress callback.
            callback_stride: Callback period in steps.

        Returns:
            dict: Time-history arrays; long-range mode adds LLA/range/altitude fields.
        """

        # Fin deflection priority: file > MPC > ballistic > control_function.
        control_type = self.config.get('simulation', {}).get('control_type', 'none')

        if self.use_fin_deflection_file and self.fin_deflection_profile is not None:
            fin_profile = self.fin_deflection_profile
            control_function = lambda state, t: fin_profile.get_deflections(t)
            logger.info(
                "Using fin deflections from external file (استخدام انحرافات الزعانف من ملف خارجي)"
            )
        elif control_type == 'mpc':
            mpc = MpcController(self)
            control_function = mpc.control_function
            self._mpc_controller = mpc
            logger.info("Using MPC controller (استخدام نظام تحكم MPC)")

            # --- MHE estimation layer (optional) ---
            est_cfg = self.config.get('estimation', {})
            est_mode = est_cfg.get('mode', 'off')
            self._mhe_enabled = (est_mode == 'mhe')
            self._mhe_output = None

            if self._mhe_enabled:
                from mpc.m130_sensor_bus import SensorBus
                from mpc.m130_mhe_estimator import MheEstimator
                self._sensor_bus = SensorBus(est_cfg, rng_seed=42)
                self._mhe_estimator = MheEstimator(est_cfg)
                self._mhe_last_fins = np.zeros(3)

                # If error_injection has biases, use them for sensor bus
                if self.use_error_injection and hasattr(self, 'sensor_biases'):
                    self._sensor_bus.set_biases(
                        self.sensor_biases['accel_bias'],
                        self.sensor_biases['gyro_bias'],
                    )

                # Wrap control function to inject mhe_output into state_dict
                _original_ctrl = control_function
                _sim_ref = self  # closure reference

                def _mhe_control_fn(state_dict, t):
                    if _sim_ref._mhe_output is not None:
                        state_dict['mhe_output'] = _sim_ref._mhe_output
                    return _original_ctrl(state_dict, t)

                control_function = _mhe_control_fn
                logger.info("MHE estimation enabled (تقدير MHE مفعّل)")
            else:
                self._mhe_enabled = False
        elif self.ballistic_mode:
            ballistic_deflection = self.ballistic_fin_deflection.copy()
            control_function = lambda state, t: ballistic_deflection
        elif control_function is None:
            control_function = lambda state, t: np.zeros(4)

        # Reset liftoff detection state to prevent state leakage between simulate() calls
        self._has_lifted_off = False
        self._max_altitude_achieved = 0.0

        # Reset long-range mode liftoff detection state
        self._has_lifted_off_lr = False
        self._max_altitude_above_launch = 0.0
        self._prev_altitude_above_target_lr = None
        self._prev_altitude_lr = None
        self._has_been_above_impact_ref = False

        # ==========================================================================
        # تهيئة السجل
        self.history = {
            'time': [],
            'position': [],
            'velocity': [],
            'attitude': [],
            'angular_velocity': [],
            'forces': [],
            'thrust_vector': [],
            'moments': [],
            'flight_phase': [],
            'q_dynamic': [],
            'alpha': [],
            'beta': [],  # Sideslip angle (rad) from aerodynamics | زاوية الانزلاق الجانبي
            'mach_aero': [],  # Mach number used in aero calculations (clamped) | رقم ماخ المستخدم في الحسابات الهوائية
            'fin_authority': [],
            'safety_violations': [],
            'mass': [],
            'g_force': [],  # Total G-force magnitude (قوة G الكلية)
            'g_force_axial': [],  # Axial G-force along rocket axis (قوة G المحورية)
            'g_force_vector': [],  # G-force components [Gx, Gy, Gz] (مكونات قوة G)
            'airspeed': [],  # Air-relative speed for Mach calculation (السرعة النسبية للهواء)
            'speed_of_sound': [],  # Speed of sound from atmosphere model (سرعة الصوت)
            'mach': [],  # Mach number = airspeed / speed_of_sound (رقم ماخ)
            'stage_number': [],  # Current stage number for multi-stage rockets (رقم المرحلة الحالية)
            'CA': [],  # Axial force coefficient (معامل القوة المحورية / السحب)
            'CN_total': [],  # Total normal force coefficient (معامل القوة العمودية الكلي)
            'CM_total': [],  # Total pitching moment coefficient (معامل عزم الميل الكلي)
            'CY_total': [],  # Total side force coefficient (معامل القوة الجانبية الكلي)
            'Cn_total': [],  # Total yawing moment coefficient (معامل عزم الانعراج الكلي)
            'CN_delta': [],  # Fin normal force effectiveness (فعالية القوة العمودية للزعانف)
            'CM_delta': [],  # Fin pitching moment effectiveness (فعالية عزم الميل للزعانف)
            # Control coefficients from fin deflections | معاملات التحكم من انحرافات الزعانف
            'CN_control': [],  # Normal force control coefficient | معامل قوة التحكم العمودية
            'CM_control': [],  # Pitching moment control coefficient | معامل عزم التحكم في الميل
            'CY_control': [],  # Side force control coefficient | معامل قوة التحكم الجانبية
            'Cn_control': [],  # Yawing moment control coefficient | معامل عزم التحكم في الانعراج
            # Raw aerodynamic moments from aerodynamics.py (before any modifications)
            # العزوم الديناميكية الهوائية الأصلية من aerodynamics.py (قبل أي تعديلات)
            'M_roll_aero': [],  # Roll moment (N-m) | عزم اللف
            'M_pitch_aero': [],  # Pitch moment (N-m) | عزم الميل
            'M_yaw_aero': [],  # Yaw moment (N-m) | عزم الانعراج
            'xbc': [],  # CG offset from reference point (m) | إزاحة مركز الثقل من نقطة المرجع
            'propellant_fraction': [],  # نسبة الوقود المتبقية (1=ممتلئ, 0=فارغ) | Propellant fraction remaining
            'cg_position': [],  # موقع مركز الثقل (م) | CG position (m)
            'vel_ned_launch': [],  # Velocity in launch-fixed NED (m/s) | السرعة في إطار NED الثابت عند الإطلاق
            'vel_ned': [],  # Velocity in local NED (m/s) | السرعة في إطار NED المحلي
            'velocity_for_aero': [],  # Air-relative velocity in NED (m/s) | السرعة النسبية للهواء في إطار NED
            'wind_ned': [],  # Wind vector in NED (m/s) | متجه الرياح في إطار NED
            'control_fins_rad': [],  # 4 fin deflections (rad) | انحرافات الزعانف الأربع
            'mpc_gamma_ref_rad': [],  # MPC gamma reference (rad) | مرجع زاوية مسار الطيران من MPC
            'mpc_chi_ref_rad': [],  # MPC chi reference (rad) | مرجع اتجاه الطيران من MPC
            # أوامر MPC الافتراضية | MPC virtual axis commands
            'mpc_delta_e': [],  # Elevator virtual deflection (rad) | أمر الميل الافتراضي
            'mpc_delta_r': [],  # Rudder virtual deflection (rad) | أمر الانعراج الافتراضي
            'mpc_delta_a': [],  # Aileron virtual deflection (rad) | أمر اللف الافتراضي
            # تشخيصات حل MPC | MPC solver diagnostics
            'mpc_solve_time_ms': [],  # Solve time (ms) | زمن الحل
            'mpc_solver_status': [],  # Solver status (0=ok, 2=approx) | حالة الحل
            'mpc_sqp_iterations': [],  # SQP iterations | تكرارات SQP
            # أخطاء التتبع | Tracking errors
            'mpc_gamma_error_rad': [],  # gamma tracking error (rad) | خطأ تتبع زاوية مسار الطيران
            'mpc_chi_error_rad': [],  # chi tracking error (rad) | خطأ تتبع الاتجاه
            # الانحرافات المأمورة | Commanded fin deflections before actuator
            'actuator_commanded_rad': [],  # Commanded deflections (rad) | الانحرافات المأمورة
            # التسارع الزاوي | Angular acceleration from Euler equation (rad/s^2)
            'angular_acceleration': [],
            # العزوم الكلية بعد تصحيح CG | Total moments after CG offset correction (N·m)
            'total_moments': [],
        }

        if self.long_range_mode:
            self.history['position_lla'] = []
            self.history['ground_range_km'] = []
            self.history['altitude_km'] = []

        # إضافة سجل قياسات الحساساتإذا كان حقن الأخطاء مفعلاً
        if self.use_error_injection and hasattr(self, 'sensor_biases'):
            self.history['accel_measured'] = []
            self.history['gyro_measured'] = []
            self.history['accel_bias'] = []
            self.history['gyro_bias'] = []

        # MHE estimation history (سجل تقدير MHE)
        if getattr(self, '_mhe_enabled', False):
            self.history['mhe_x_hat'] = []
            self.history['mhe_quality'] = []
            self.history['mhe_solve_ms'] = []


        # Input validation to prevent edge case crashes
        if duration <= 0:
            raise ValueError(
                f"duration must be positive, got {duration}. "
                f"يجب أن تكون المدة موجبة، تم الحصول على {duration}."
            )
        if dt <= 0:
            raise ValueError(
                f"dt must be positive, got {dt}. "
                f"يجب أن تكون خطوة الزمن موجبة، تم الحصول على {dt}."
            )

        # linspace (not arange) لتجنب مشاكل دقة الفاصلة العائمة
        # ceil (not round) لتجنب banker's rounding حيث round(0.5)=0
        n_steps = max(1, int(np.ceil(duration / dt)))
        t_array = np.linspace(0, duration, n_steps + 1)
        state = self._initialize_state()
        
        self._update_last_valid_quaternion(state[6:10])
        self._quaternion_recovery_count = 0

        last_t = 0.0
        prev_state = None  # Store previous state for impact interpolation
        prev_t = None      # Store previous time for impact interpolation
        impact_detected = False
        precise_impact_time = None  # Set by interpolation for direct use in final_time
        numerical_error_termination = False  # Flag for graceful termination on numerical errors
        termination_reason = None  # Store reason for early termination

        for i in range(len(t_array) - 1):
            t = t_array[i]
            # Use actual dt from t_array to handle any rounding in linspace
            actual_dt = t_array[i + 1] - t
            last_t = t

            # Store previous state before integration for impact interpolation
            prev_state = state.copy()
            prev_t = t

            try:
                next_state, snapshot, t_end, _ = self._integrate_one_step(
                    state, t, actual_dt, control_function
                )
            except DegenerateQuaternionError as e:
                # Graceful termination on degenerate quaternion
                logger.error(
                    "DegenerateQuaternionError at t=%.4fs (step %d/%d): %s. "
                    "Terminating simulation gracefully with partial history. "
                    "خطأ رباعي منحط عند t=%.4fs (خطوة %d/%d): %s. "
                    "إنهاء المحاكاة بشكل متحكم مع سجل جزئي.",
                    t, i, len(t_array) - 1, str(e), t, i, len(t_array) - 1, str(e)
                )
                numerical_error_termination = True
                termination_reason = f"DegenerateQuaternionError at t={t:.4f}s: {str(e)}"
                break
            
            # --- MHE estimation update (after step, for next control call) ---
            if getattr(self, '_mhe_enabled', False):
                try:
                    # Always collect measurements from the start
                    pkt = self._sensor_bus.update(snapshot, t_end)
                    self._mhe_estimator.push_measurement(t_end, pkt.y_meas, gps_fresh=pkt.gps_fresh)

                    # Build MHE parameter vector [de_act, dr_act, da_act, mass, thrust, Ixx, Iyy, Izz, launch_alt]
                    # Use actual fin positions (from actuator model) instead of commands.
                    mpc_p = mpc._get_mhe_params(t_end)  # [mass, thrust, Ixx, Iyy, Izz, launch_alt]
                    if 'actuator_positions' in snapshot:
                        ap = snapshot['actuator_positions']
                        de_act = 0.25 * (-ap[0] - ap[1] + ap[2] + ap[3])
                        dr_act = 0.25 * (-ap[0] + ap[1] + ap[2] - ap[3])
                        da_act = 0.25 * ( ap[0] + ap[1] + ap[2] + ap[3])
                    else:
                        de_act = self._mhe_last_fins[0]
                        dr_act = self._mhe_last_fins[1]
                        da_act = self._mhe_last_fins[2]
                    fins_actual = np.array([de_act, dr_act, da_act])
                    mhe_params = np.array([
                        de_act,    # delta_e_act (actual)
                        dr_act,    # delta_r_act (actual)
                        da_act,    # delta_a_act (actual)
                        mpc_p[0],  # mass
                        mpc_p[1],  # thrust
                        mpc_p[2],  # Ixx
                        mpc_p[3],  # Iyy
                        mpc_p[4],  # Izz
                        mpc_p[5],  # launch_alt
                    ])
                    self._mhe_estimator.push_control_and_params(
                        t_end, fins_actual, mhe_params
                    )

                    # Initialize MHE once we have enough measurements
                    if not self._mhe_estimator._initialized:
                        if self._mhe_estimator.ready_to_init():
                            x0_mhe = self._build_mhe_init_state(snapshot, t_end)
                            self._mhe_estimator.init_state(x0_mhe)
                        # Skip update until initialized
                    else:
                        mhe_out = self._mhe_estimator.update(t_end)
                        self._mhe_output = mhe_out

                    # Track fin commands for MHE params (de, dr, da from MPC)
                    if hasattr(mpc, '_last_delta_e'):
                        self._mhe_last_fins = np.array([
                            mpc._last_delta_e, mpc._last_delta_r, mpc._last_delta_a
                        ])

                    # Log MHE to snapshot (will be saved by _append_history below)
                    if self._mhe_output is not None and self._mhe_output.valid:
                        snapshot['mhe_x_hat'] = self._mhe_output.x_hat.copy()
                        snapshot['mhe_quality'] = self._mhe_output.quality
                        snapshot['mhe_solve_ms'] = self._mhe_output.solve_time_ms
                except Exception as e:
                    if i < 10 or i % 500 == 0:
                        logger.warning(f"MHE update failed at t={t_end:.3f}: {e}")

            self._append_history(snapshot)

            state = next_state
            self._normalize_state(state, t_end)

            # استدعاء دالة التقدم إذا تم توفيرها
            if on_step is not None and i % callback_stride == 0:
                # حساب الارتفاع بناءً على وضع المحاكاة
                altitude = self._get_altitude(state)

                velocity_mag = np.linalg.norm(state[3:6])

                step_info = {
                    'step': i,
                    'total_steps': n_steps,
                    'time': t,
                    'duration': duration,
                    'progress': (i / n_steps) * 100,
                    'altitude': altitude,
                    'velocity': velocity_mag,
                    'flight_phase': snapshot.get('flight_phase', 'UNKNOWN'),
                }
                on_step(step_info)

            # Stage separation event detection.
            if self._detect_separation_event(prev_state, prev_t, state, t_end):
                logger.info(
                    "Separation event detected between t=%.4fs and t=%.4fs",
                    prev_t, t_end
                )

                # Find precise separation point using bisection
                sep_state, sep_time = self._interpolate_separation_point(
                    prev_state, prev_t, state, t_end, control_function
                )

                # Execute separation at the precise time
                if self._execute_separation_at_time(sep_state, sep_time):
                    # Continue integration from separation point to end of step
                    remaining_dt = (t + actual_dt) - sep_time
                    if remaining_dt > 1e-10:
                        # Integrate the remaining time with new mass/inertia
                        state, post_sep_snapshot, _, _ = self._integrate_one_step(
                            sep_state, sep_time, remaining_dt, control_function
                        )
                        self._normalize_state(state, t + actual_dt)
                        # Append history for post-separation state to avoid gaps
                        self._append_history(post_sep_snapshot)
                    else:
                        state = sep_state
                else:
                    # Log warning instead of fallback 
                    logger.warning(
                        "Separation execution failed at t=%.6fs (bisection path). "
                        "Possible causes: already at last stage, stage mismatch, or separation_in_progress. "
                        "فشل تنفيذ الفصل في t=%.6fs (مسار التنصيف). الأسباب المحتملة: بالفعل في المرحلة الأخيرة، "
                        "عدم تطابق المراحل، أو الفصل قيد التنفيذ.",
                        sep_time, sep_time
                    )
            # With external_separation_handling=True, stage advance is event-driven only.

            if self._detect_ground_impact(state, t + actual_dt):
                # Use bisection interpolation to find precise impact point
                if prev_state is not None and self._is_descending(state):
                    logger.info(
                        "Performing impact interpolation between t=%.4fs and t=%.4fs",
                        prev_t, t + actual_dt
                    )
                    impact_state, impact_time, impact_alt = self._interpolate_impact_point(
                        prev_state, prev_t, t + actual_dt, control_function
                    )

                    # Update state to precise impact point
                    state = impact_state
                    precise_impact_time = impact_time  # Store for direct use in final_time

                    logger.info(
                        "Impact interpolation complete: t=%.6fs, altitude=%.4fm",
                        impact_time, impact_alt
                    )

                impact_detected = True
                break

        self.state = state

        # Final time: precise_impact_time → last_t (numerical error) → last_t + actual_dt
        if precise_impact_time is not None:
            final_time = precise_impact_time
        elif numerical_error_termination:
            final_time = last_t
        else:
            final_time = min(last_t + actual_dt, duration)

        # Append final state only if termination was not due to numerical error.
        if not numerical_error_termination:
            final_state_dict = self._state_array_to_dict(state, final_time)
            final_control = control_function(final_state_dict, final_time)
            _, final_snapshot = self.state_derivative(state, final_time, final_control)
            self._append_history(final_snapshot, force=True)

        self.history['meta'] = {
            'completed_normally': not numerical_error_termination and not impact_detected,
            'impact_detected': impact_detected,
            'numerical_error_termination': numerical_error_termination,
            'termination_reason': termination_reason,
            'final_time': final_time,
            'requested_duration': duration,
        }

        return self.history

    def get_state(self, t: float = None):
        """Get current state.

        If t is provided and mass model is active, mass is read from the model;
        otherwise mass comes from state[13]. Returns position, velocity,
        quaternion, angular_velocity, and mass (NED or ECEF in long-range mode).
        """
        # When use_mass_model=True, state[13] is already synchronized with model
        # but explicit t lookup is still supported for arbitrary time queries
        effective_use_mass_model = self._effective_use_mass_model()
        if t is not None and effective_use_mass_model:
            # استخدام نموذج الكتلة للبحث الصريح
            mass = max(self.dynamics.get_mass(t), self.dry_mass)
        else:
            # استخدام الكتلة من متجه الحالة (متزامنة مع النموذج عند use_mass_model=True)
            mass = float(self.state[13])

        return {
            'position': self.state[0:3].copy(),
            'velocity': self.state[3:6].copy(),
            'quaternion': self.state[6:10].copy(),
            'angular_velocity': self.state[10:13].copy(),
            'mass': mass,
        }


def export_comprehensive_log(history: Dict, sim: 'Rocket6DOFSimulation', output_path: str) -> None:
    """
    Export comprehensive simulation history to CSV.
    
    Args:
        history: Dictionary containing simulation history data
        sim: Rocket6DOFSimulation instance
        output_path: Path to save the CSV file
    """
    import csv

    n_points = len(history['time'])

    def _safe_2d(arr_list, cols, n):
        """Convert list to 2D array, handling ragged/scalar entries."""
        try:
            a = np.array(arr_list)
            if a.ndim == 2 and a.shape == (n, cols):
                return a
        except (ValueError, TypeError):
            pass
        out = np.zeros((n, cols))
        for i, v in enumerate(arr_list):
            v = np.asarray(v).ravel()
            out[i, :len(v)] = v[:cols]
        return out

    # تحضير البيانات | Prepare data
    time_data = np.array(history['time'])
    position = _safe_2d(history['position'], 3, n_points)
    velocity = _safe_2d(history['velocity'], 3, n_points)
    attitude = _safe_2d(history['attitude'], 4, n_points)
    angular_velocity = _safe_2d(history['angular_velocity'], 3, n_points)
    forces = _safe_2d(history.get('forces', [np.zeros(3)] * n_points), 3, n_points)
    thrust_vector = _safe_2d(history.get('thrust_vector', [np.zeros(3)] * n_points), 3, n_points)
    moments = _safe_2d(history.get('moments', [np.zeros(3)] * n_points), 3, n_points)
    
    # حساب السرعة الكلية | Calculate total velocity
    velocity_mag = np.linalg.norm(velocity, axis=1)
    
    # الارتفاع | Altitude
    if sim.long_range_mode and 'altitude_km' in history:
        altitude_m = np.array(history['altitude_km']) * 1000.0
    else:
        altitude_m = -position[:, 2]  # NED frame: -Z is up
    
    # المدى الأرضي | Ground range
    if 'ground_range_km' in history:
        ground_range_m = np.array(history['ground_range_km']) * 1000.0
    else:
        ground_range_m = np.sqrt(position[:, 0]**2 + position[:, 1]**2)
    
    # العوامل الايروديناميكية | Aerodynamic factors
    alpha = np.array(history.get('alpha', [0.0] * n_points))
    beta = np.array(history.get('beta', [0.0] * n_points))  # Sideslip angle from aerodynamics
    mach_aero = np.array(history.get('mach_aero', [0.0] * n_points))  # Mach used in aero calculations
    q_dynamic = np.array(history.get('q_dynamic', [0.0] * n_points))
    mach = np.array(history.get('mach', [0.0] * n_points))
    airspeed = np.array(history.get('airspeed', [0.0] * n_points))
    
    # المعاملات الديناميكية الهوائية الكلية | Total aerodynamic coefficients
    CN_total = np.array(history.get('CN_total', [0.0] * n_points))
    CM_total = np.array(history.get('CM_total', [0.0] * n_points))
    CY_total = np.array(history.get('CY_total', [0.0] * n_points))
    Cn_total = np.array(history.get('Cn_total', [0.0] * n_points))
    
    # معاملات فعالية الزعانف | Fin effectiveness coefficients
    CN_delta = np.array(history.get('CN_delta', [0.0] * n_points))
    CM_delta = np.array(history.get('CM_delta', [0.0] * n_points))
    
    # معاملات التحكم من انحرافات الزعانف | Control coefficients from fin deflections
    CN_control = np.array(history.get('CN_control', [0.0] * n_points))
    CM_control = np.array(history.get('CM_control', [0.0] * n_points))
    CY_control = np.array(history.get('CY_control', [0.0] * n_points))
    Cn_control = np.array(history.get('Cn_control', [0.0] * n_points))
    
    # العزوم الديناميكية الهوائية الأصلية من aerodynamics.py | Raw aerodynamic moments from aerodynamics.py
    M_roll_aero = np.array(history.get('M_roll_aero', [0.0] * n_points))
    M_pitch_aero = np.array(history.get('M_pitch_aero', [0.0] * n_points))
    M_yaw_aero = np.array(history.get('M_yaw_aero', [0.0] * n_points))
    
    # إزاحة مركز الثقل | CG offset from reference point
    xbc = np.array(history.get('xbc', [0.0] * n_points))
    propellant_fraction = np.array(history.get('propellant_fraction', [1.0] * n_points))
    cg_position = np.array(history.get('cg_position', [0.0] * n_points))

    # بيانات التحكم والرياح ومرجع MPC | Control, wind, MPC ref data
    control_fins = _safe_2d(history.get('control_fins_rad', [np.zeros(4)] * n_points), 4, n_points)
    wind_ned_arr = _safe_2d(history.get('wind_ned', [np.zeros(3)] * n_points), 3, n_points)
    mpc_gamma_ref = np.array(history.get('mpc_gamma_ref_rad', [0.0] * n_points))
    mpc_chi_ref = np.array(history.get('mpc_chi_ref_rad', [0.0] * n_points))
    # MPC virtual axis commands | أوامر MPC الافتراضية
    mpc_delta_e = np.array(history.get('mpc_delta_e', [0.0] * n_points))
    mpc_delta_r = np.array(history.get('mpc_delta_r', [0.0] * n_points))
    mpc_delta_a = np.array(history.get('mpc_delta_a', [0.0] * n_points))
    # MPC solver diagnostics | تشخيصات حل MPC
    mpc_solve_time_ms = np.array(history.get('mpc_solve_time_ms', [0.0] * n_points))
    mpc_solver_status = np.array(history.get('mpc_solver_status', [0] * n_points))
    mpc_sqp_iterations = np.array(history.get('mpc_sqp_iterations', [0] * n_points))
    # Tracking errors | أخطاء التتبع
    mpc_gamma_error = np.array(history.get('mpc_gamma_error_rad', [0.0] * n_points))
    mpc_chi_error = np.array(history.get('mpc_chi_error_rad', [0.0] * n_points))
    # Commanded deflections | الانحرافات المأمورة
    actuator_commanded = _safe_2d(history.get('actuator_commanded_rad', [np.zeros(4)] * n_points), 4, n_points)
    # Virtual deflections (X-config mixing)
    delta_pitch = 0.25 * (-control_fins[:,0] - control_fins[:,1] + control_fins[:,2] + control_fins[:,3])
    delta_yaw   = 0.25 * (-control_fins[:,0] + control_fins[:,1] + control_fins[:,2] - control_fins[:,3])
    delta_roll  = 0.25 * ( control_fins[:,0] + control_fins[:,1] + control_fins[:,2] + control_fins[:,3])
    # Static margin (calibers) — computed from base aero coefficients only
    # (CN_total - CN_control) approximates CN_base (removes control, keeps damping)
    # This gives the true airframe stability, not masked by MPC corrections.
    CN_aero = CN_total - CN_control
    CM_aero = CM_total - CM_control
    with np.errstate(divide='ignore', invalid='ignore'):
        sm_raw = -CM_aero / np.where(np.abs(CN_aero) > 0.05, CN_aero, np.nan)
    static_margin = np.where(np.isfinite(sm_raw), sm_raw, np.nan)
    # Physically plausible SM band: ±2 caliber for a stable/marginal airframe.
    static_margin = np.clip(static_margin, -2.0, 2.0)
    # Interpolate NaN gaps (where |CN_aero| was too small to compute SM)
    nan_mask = np.isnan(static_margin)
    if nan_mask.any() and not nan_mask.all():
        valid_idx = np.where(~nan_mask)[0]
        static_margin[nan_mask] = np.interp(
            np.where(nan_mask)[0], valid_idx, static_margin[valid_idx]
        )
    
    
    
    
    # الكتلة | Mass
    mass = np.array(history.get('mass', [0.0] * n_points))
    
    # مرحلة الطيران | Flight phase
    flight_phase = history.get('flight_phase', ['N/A'] * n_points)

    # ==================== البيانات المفقودة سابقاً | Previously missing data ====================
    # صلاحية الزعانف | Fin authority
    fin_authority = np.array(history.get('fin_authority', [0.0] * n_points))
    # انتهاكات السلامة | Safety violations
    safety_violations = history.get('safety_violations', [0] * n_points)
    # قوة G | G-force
    g_force = np.array(history.get('g_force', [0.0] * n_points))
    g_force_axial = np.array(history.get('g_force_axial', [0.0] * n_points))
    g_force_vector = _safe_2d(history.get('g_force_vector', [np.zeros(3)] * n_points), 3, n_points)
    # سرعة الصوت | Speed of sound
    speed_of_sound = np.array(history.get('speed_of_sound', [0.0] * n_points))
    # رقم المرحلة | Stage number
    stage_number = np.array(history.get('stage_number', [0] * n_points))
    # معامل القوة المحورية | Axial force coefficient
    CA_coeff = np.array(history.get('CA', [0.0] * n_points))
    # السرعة في إطار NED الثابت عند الإطلاق | Velocity in launch-fixed NED
    vel_ned_launch = _safe_2d(history.get('vel_ned_launch', [np.zeros(3)] * n_points), 3, n_points)
    # السرعة في إطار NED المحلي | Velocity in local NED
    vel_ned = _safe_2d(history.get('vel_ned', [np.zeros(3)] * n_points), 3, n_points)
    # السرعة النسبية للهواء في NED | Air-relative velocity in NED
    velocity_for_aero = _safe_2d(history.get('velocity_for_aero', [np.zeros(3)] * n_points), 3, n_points)
    # التسارع الزاوي | Angular acceleration (rad/s^2)
    angular_acceleration = _safe_2d(history.get('angular_acceleration', [np.zeros(3)] * n_points), 3, n_points)
    # العزوم الكلية بعد تصحيح CG | Total moments after CG offset correction (N·m)
    total_moments = _safe_2d(history.get('total_moments', [np.zeros(3)] * n_points), 3, n_points)
    # الموقع الجغرافي (شرطي) | Position LLA (conditional)
    position_lla = _safe_2d(history.get('position_lla', [np.zeros(3)] * n_points), 3, n_points)
    # بيانات المستشعرات (شرطي) | Sensor data (conditional)
    accel_measured = _safe_2d(history.get('accel_measured', [np.zeros(3)] * n_points), 3, n_points)
    gyro_measured = _safe_2d(history.get('gyro_measured', [np.zeros(3)] * n_points), 3, n_points)
    accel_bias = _safe_2d(history.get('accel_bias', [np.zeros(3)] * n_points), 3, n_points)
    gyro_bias = _safe_2d(history.get('gyro_bias', [np.zeros(3)] * n_points), 3, n_points)
    # بيانات MHE (شرطي) | MHE data (conditional)
    mhe_x_hat_raw = history.get('mhe_x_hat', None)
    mhe_quality_raw = history.get('mhe_quality', [])
    mhe_solve_ms_raw = history.get('mhe_solve_ms', [])
    # MHE starts after warmup, so pad to n_points with zeros at the front
    mhe_quality = np.zeros(n_points)
    mhe_solve_ms = np.zeros(n_points)
    # Unpack MHE x_hat[17] into individual arrays for CSV export
    MHE_STATE_DIM = 17
    mhe_x_hat = np.zeros((n_points, MHE_STATE_DIM))
    if mhe_x_hat_raw is not None and len(mhe_x_hat_raw) > 0:
        mhe_x_hat_arr = np.array(mhe_x_hat_raw)
        offset = n_points - len(mhe_x_hat_arr)
        if offset >= 0:
            mhe_x_hat[offset:] = mhe_x_hat_arr
        else:
            mhe_x_hat[:] = mhe_x_hat_arr[:n_points]
    if len(mhe_quality_raw) > 0:
        offset = n_points - len(mhe_quality_raw)
        if offset >= 0:
            mhe_quality[offset:] = np.array(mhe_quality_raw)
            mhe_solve_ms[offset:] = np.array(mhe_solve_ms_raw)
        else:
            mhe_quality[:] = np.array(mhe_quality_raw[:n_points])
            mhe_solve_ms[:] = np.array(mhe_solve_ms_raw[:n_points])
    # تحديد وجود البيانات الشرطية | Detect presence of conditional data
    has_position_lla = 'position_lla' in history and len(history['position_lla']) > 0
    has_sensor_data = 'accel_measured' in history and len(history['accel_measured']) > 0
    has_mhe_data = 'mhe_x_hat' in history and len(history['mhe_x_hat']) > 0
    
    # Body-frame specific force (thrust + aero) / mass; gravity excluded.
    G_ACCEL = 9.81  # تسارع الجاذبية للتحويل إلى وحدات g | Gravity for g-unit conversion
    
    # Forces are stored in body frame (thrust + aero)
    acceleration_body = np.zeros_like(forces)
    for i in range(n_points):
        if mass[i] > 0:
            # Acceleration in body frame in g-units
            acceleration_body[i] = (forces[i] / mass[i]) / G_ACCEL
    
    # Transform to launch-fixed FUR frame (NED->FUR; long-range: ECEF->NED->FUR).
    
    # Get transformation parameters from ECEF to launch-fixed NED
    if sim.long_range_mode and sim.frame_configuration is not None:
        # الحصول على موقع الإطلاق في ECEF | Get launch position in ECEF
        launch_ecef = sim.frame_configuration.get_launch_ecef_array()
        # الحصول على مصفوفة التحويل من ECEF إلى NED عند الإطلاق
        # Get transformation matrix from ECEF to launch-fixed NED
        C_ecef_to_ned_launch = sim.frame_configuration.get_C_ecef_to_ned_launch_array()
        
        if C_ecef_to_ned_launch is not None and launch_ecef is not None:
            # Transform position: ECEF → Launch-Fixed NED → FUR
            position_fur = np.zeros_like(position)
            for i in range(n_points):
                # الموقع النسبي في ECEF | Relative position in ECEF
                pos_ecef_rel = position[i] - launch_ecef
                # تحويل إلى NED ثابت عند الإطلاق | Transform to launch-fixed NED
                pos_ned_launch = C_ecef_to_ned_launch @ pos_ecef_rel
                # تحويل إلى FUR | Transform to FUR
                position_fur[i] = transform_ned_to_fur(pos_ned_launch)
            
            # Transform velocity: ECEF → Launch-Fixed NED → FUR
            velocity_fur = np.zeros_like(velocity)
            for i in range(n_points):
                # تحويل السرعة إلى NED ثابت عند الإطلاق | Transform velocity to launch-fixed NED
                vel_ned_launch_i = C_ecef_to_ned_launch @ velocity[i]
                # تحويل إلى FUR | Transform to FUR
                velocity_fur[i] = transform_ned_to_fur(vel_ned_launch_i)
            
            # Compute FUR acceleration from velocity difference and convert to g-units.
            acceleration_fur = np.zeros_like(forces)
            dt = np.diff(time_data)
            for i in range(n_points - 1):
                if dt[i] > 0:
                    # Compute acceleration from velocity change in FUR
                    dv_fur = velocity_fur[i + 1] - velocity_fur[i]
                    acceleration_fur[i] = (dv_fur / dt[i]) / G_ACCEL  # وحدات g | g-units
            # Last point takes same value as previous
            if n_points > 1:
                acceleration_fur[-1] = acceleration_fur[-2]
        else:
            # احتياطي: استخدام التحويل البسيط | Fallback: use simple transformation
            position_fur = np.zeros_like(position)
            velocity_fur = np.zeros_like(velocity)
            acceleration_fur = np.zeros_like(forces)
            for i in range(n_points):
                position_fur[i] = transform_ned_to_fur(position[i])
                velocity_fur[i] = transform_ned_to_fur(velocity[i])
                if mass[i] > 0:
                    acc_ned = forces[i] / mass[i]
                    acceleration_fur[i] = transform_ned_to_fur(acc_ned) / G_ACCEL
    else:
        # Short-range mode: data is already in NED
        position_fur = np.zeros_like(position)
        velocity_fur = np.zeros_like(velocity)
        acceleration_fur = np.zeros_like(forces)
        
        for i in range(n_points):
            position_fur[i] = transform_ned_to_fur(position[i])
            velocity_fur[i] = transform_ned_to_fur(velocity[i])
        
        # Compute acceleration from velocity change and divide by 9.81
        dt = np.diff(time_data)
        for i in range(n_points - 1):
            if dt[i] > 0:
                dv_fur = velocity_fur[i + 1] - velocity_fur[i]
                acceleration_fur[i] = (dv_fur / dt[i]) / G_ACCEL  # وحدات g | g-units
        if n_points > 1:
            acceleration_fur[-1] = acceleration_fur[-2]
    
    
    # تحويل الاتجاه إلى زوايا أويلر | Convert attitude to Euler angles
    def quaternion_to_euler_angles(q):
        """Convert quaternion [w, x, y, z] to Euler angles [roll, pitch, yaw] in degrees."""
        w, x, y, z = q
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)
    
    # كتابة ملف CSV | Write CSV file
    with open(output_path, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        
        # كتابة العناوين | Write headers
        headers = [
            # الوقت | Time
            'time_s',
            # السرعة | Velocity
            'velocity_x_m_s', 'velocity_y_m_s', 'velocity_z_m_s', 'velocity_total_m_s',
            # الموقع | Position
            'position_x_m', 'position_y_m', 'position_z_m',
            # الارتفاع والمدى | Altitude and Range
            'altitude_m', 'ground_range_m',
            # العوامل الايروديناميكية | Aerodynamic factors
            'alpha_rad', 'alpha_deg', 'beta_rad', 'beta_deg', 'mach', 'mach_aero', 'q_dynamic_Pa', 'airspeed_m_s',
            # الاتجاه | Attitude
            'quat_w', 'quat_x', 'quat_y', 'quat_z',
            'roll_deg', 'pitch_deg', 'yaw_deg',
            # السرعة الزاوية | Angular velocity
            'omega_x_rad_s', 'omega_y_rad_s', 'omega_z_rad_s',
            'omega_x_deg_s', 'omega_y_deg_s', 'omega_z_deg_s',
            # القوى والعزوم | Forces and moments
            'force_x_N', 'force_y_N', 'force_z_N',
            # متجه الدفع | Thrust vector
            'thrust_x_N', 'thrust_y_N', 'thrust_z_N',
            'moment_x_Nm', 'moment_y_Nm', 'moment_z_Nm',
            # الكتلة ومرحلة الطيران | Mass and flight phase
            'mass_kg', 'flight_phase',
            # المعاملات الديناميكية الهوائية الكلية | Total aerodynamic coefficients
            'CN_total', 'CM_total', 'CY_total', 'Cn_total',
            # معاملات فعالية الزعانف | Fin effectiveness coefficients
            'CN_delta', 'CM_delta',
            # معاملات التحكم من انحرافات الزعانف | Control coefficients from fin deflections
            'CN_control', 'CM_control', 'CY_control', 'Cn_control',
            # العزوم الديناميكية الهوائية الأصلية | Raw aerodynamic moments from aerodynamics.py
            'M_roll_aero', 'M_pitch_aero', 'M_yaw_aero',
            # إزاحة مركز الثقل | CG offset from reference point
            'xbc_m',
            # نسبة الوقود المتبقية | Propellant fraction remaining
            'propellant_fraction',
            # موقع مركز الثقل | CG position
            'cg_position_m',
            # FUR frame data (Forward-Up-Right)
            # Position in FUR frame
            'position_fur_x_m', 'position_fur_y_m', 'position_fur_z_m',
            # Velocity in FUR frame
            'velocity_fur_x_m_s', 'velocity_fur_y_m_s', 'velocity_fur_z_m_s',
            # Acceleration in FUR frame (from velocity change)
            'acceleration_fur_x_m_s2', 'acceleration_fur_y_m_s2', 'acceleration_fur_z_m_s2',
            # Acceleration in body frame
            # Ax = axial (along rocket axis)
            # Ay = lateral
            # Az = normal
            'acceleration_body_x_g', 'acceleration_body_y_g', 'acceleration_body_z_g',
            # Control, wind, and MPC reference data
            'fin_1_rad', 'fin_2_rad', 'fin_3_rad', 'fin_4_rad',
            'delta_pitch_rad', 'delta_yaw_rad', 'delta_roll_rad',
            'wind_north_m_s', 'wind_east_m_s', 'wind_down_m_s',
            'static_margin_cal',
            'mpc_gamma_ref_deg', 'mpc_chi_ref_deg',
            # أوامر MPC الافتراضية | MPC virtual axis commands
            'mpc_delta_e_rad', 'mpc_delta_r_rad', 'mpc_delta_a_rad',
            # تشخيصات حل MPC | MPC solver diagnostics
            'mpc_solve_time_ms', 'mpc_solver_status', 'mpc_sqp_iterations',
            # أخطاء التتبع | Tracking errors
            'mpc_gamma_error_deg', 'mpc_chi_error_deg',
            # الانحرافات المأمورة | Commanded fin deflections before actuator
            'actuator_cmd_fin1_rad', 'actuator_cmd_fin2_rad', 'actuator_cmd_fin3_rad', 'actuator_cmd_fin4_rad',
            # ==================== البيانات المضافة | Added data ====================
            # صلاحية الزعانف والسلامة | Fin authority and safety
            'fin_authority', 'safety_violations',
            # قوة G | G-force
            'g_force_total', 'g_force_axial', 'g_force_x', 'g_force_y', 'g_force_z',
            # سرعة الصوت | Speed of sound
            'speed_of_sound_m_s',
            # رقم المرحلة | Stage number
            'stage_number',
            # معامل القوة المحورية | Axial force coefficient
            'CA',
            # السرعة في إطار NED الثابت عند الإطلاق | Velocity in launch-fixed NED
            'vel_ned_launch_north_m_s', 'vel_ned_launch_east_m_s', 'vel_ned_launch_down_m_s',
            # السرعة في إطار NED المحلي | Velocity in local NED
            'vel_ned_north_m_s', 'vel_ned_east_m_s', 'vel_ned_down_m_s',
            # السرعة النسبية للهواء في NED | Air-relative velocity in NED
            'vel_aero_north_m_s', 'vel_aero_east_m_s', 'vel_aero_down_m_s',
            # التسارع الزاوي | Angular acceleration (rad/s^2)
            'angular_accel_x_rad_s2', 'angular_accel_y_rad_s2', 'angular_accel_z_rad_s2',
            'angular_accel_x_deg_s2', 'angular_accel_y_deg_s2', 'angular_accel_z_deg_s2',
            # العزوم الكلية بعد تصحيح CG | Total moments after CG offset correction (N·m)
            'total_moment_x_Nm', 'total_moment_y_Nm', 'total_moment_z_Nm',
        ]
        # إضافة العناوين الشرطية | Add conditional headers
        if has_position_lla:
            headers.extend(['latitude_deg', 'longitude_deg', 'altitude_lla_m'])
        if has_sensor_data:
            headers.extend([
                'accel_meas_x', 'accel_meas_y', 'accel_meas_z',
                'gyro_meas_x', 'gyro_meas_y', 'gyro_meas_z',
                'accel_bias_x', 'accel_bias_y', 'accel_bias_z',
                'gyro_bias_x', 'gyro_bias_y', 'gyro_bias_z',
            ])
        if has_mhe_data:
            headers.extend([
                'mhe_quality', 'mhe_solve_ms',
                # حالة MHE المقدّرة [17] | MHE estimated state vector [17]
                'mhe_V_m_s', 'mhe_gamma_rad', 'mhe_chi_rad',
                'mhe_p_rad_s', 'mhe_q_rad_s', 'mhe_r_rad_s',
                'mhe_alpha_rad', 'mhe_beta_rad', 'mhe_phi_rad',
                'mhe_h_scaled', 'mhe_xg_scaled', 'mhe_yg_scaled',
                'mhe_bgx_rad_s', 'mhe_bgy_rad_s', 'mhe_bgz_rad_s',
                'mhe_wn_m_s', 'mhe_we_m_s',
            ])
        writer.writerow(headers)
        
        # كتابة البيانات | Write data
        for i in range(n_points):
            # تحويل الاتجاه إلى زوايا أويلر
            roll, pitch, yaw = quaternion_to_euler_angles(attitude[i])
            
            row = [
                # الوقت
                f'{time_data[i]:.3f}',
                # السرعة
                f'{velocity[i, 0]:.4f}', f'{velocity[i, 1]:.4f}', f'{velocity[i, 2]:.4f}', f'{velocity_mag[i]:.4f}',
                # الموقع
                f'{position[i, 0]:.4f}', f'{position[i, 1]:.4f}', f'{position[i, 2]:.4f}',
                # الارتفاع والمدى
                f'{altitude_m[i]:.4f}', f'{ground_range_m[i]:.4f}',
                # العوامل الايروديناميكية
                f'{alpha[i]:.6f}', f'{np.degrees(alpha[i]):.4f}', f'{beta[i]:.6f}', f'{np.degrees(beta[i]):.4f}', f'{mach[i]:.4f}', f'{mach_aero[i]:.4f}', f'{q_dynamic[i]:.4f}', f'{airspeed[i]:.4f}',
                # الاتجاه
                f'{attitude[i, 0]:.6f}', f'{attitude[i, 1]:.6f}', f'{attitude[i, 2]:.6f}', f'{attitude[i, 3]:.6f}',
                f'{roll:.4f}', f'{pitch:.4f}', f'{yaw:.4f}',
                # السرعة الزاوية
                f'{angular_velocity[i, 0]:.6f}', f'{angular_velocity[i, 1]:.6f}', f'{angular_velocity[i, 2]:.6f}',
                f'{np.degrees(angular_velocity[i, 0]):.4f}', f'{np.degrees(angular_velocity[i, 1]):.4f}', f'{np.degrees(angular_velocity[i, 2]):.4f}',
                # القوى والعزوم
                f'{forces[i, 0]:.4f}' if i < len(forces) else '0',
                f'{forces[i, 1]:.4f}' if i < len(forces) else '0',
                f'{forces[i, 2]:.4f}' if i < len(forces) else '0',
                # متجه الدفع | Thrust vector
                f'{thrust_vector[i, 0]:.4f}' if i < len(thrust_vector) else '0',
                f'{thrust_vector[i, 1]:.4f}' if i < len(thrust_vector) else '0',
                f'{thrust_vector[i, 2]:.4f}' if i < len(thrust_vector) else '0',
                f'{moments[i, 0]:.4f}' if i < len(moments) else '0',
                f'{moments[i, 1]:.4f}' if i < len(moments) else '0',
                f'{moments[i, 2]:.4f}' if i < len(moments) else '0',
                # الكتلة ومرحلة الطيران
                f'{mass[i]:.4f}' if i < len(mass) else '0',
                str(flight_phase[i]) if i < len(flight_phase) else 'N/A',
                # المعاملات الديناميكية الهوائية الكلية
                f'{CN_total[i]:.6f}' if i < len(CN_total) else '0',
                f'{CM_total[i]:.6f}' if i < len(CM_total) else '0',
                f'{CY_total[i]:.6f}' if i < len(CY_total) else '0',
                f'{Cn_total[i]:.6f}' if i < len(Cn_total) else '0',
                # معاملات فعالية الزعانف
                f'{CN_delta[i]:.6f}' if i < len(CN_delta) else '0',
                f'{CM_delta[i]:.6f}' if i < len(CM_delta) else '0',
                # معاملات التحكم من انحرافات الزعانف
                f'{CN_control[i]:.6f}' if i < len(CN_control) else '0',
                f'{CM_control[i]:.6f}' if i < len(CM_control) else '0',
                f'{CY_control[i]:.6f}' if i < len(CY_control) else '0',
                f'{Cn_control[i]:.6f}' if i < len(Cn_control) else '0',
                # العزوم الديناميكية الهوائية الأصلية | Raw aerodynamic moments
                f'{M_roll_aero[i]:.6f}' if i < len(M_roll_aero) else '0',
                f'{M_pitch_aero[i]:.6f}' if i < len(M_pitch_aero) else '0',
                f'{M_yaw_aero[i]:.6f}' if i < len(M_yaw_aero) else '0',
                # إزاحة مركز الثقل | CG offset from reference point
                f'{xbc[i]:.6f}' if i < len(xbc) else '0',
                # نسبة الوقود المتبقية | Propellant fraction remaining
                f'{propellant_fraction[i]:.6f}' if i < len(propellant_fraction) else '1.0',
                # موقع مركز الثقل | CG position
                f'{cg_position[i]:.6f}' if i < len(cg_position) else '0',
                # ==========================================================================
                # FUR frame data (Forward-Up-Right)
                # ==========================================================================
                # الموقع في إطار FUR | Position in FUR frame
                f'{position_fur[i, 0]:.4f}', f'{position_fur[i, 1]:.4f}', f'{position_fur[i, 2]:.4f}',
                # السرعة في إطار FUR | Velocity in FUR frame
                f'{velocity_fur[i, 0]:.4f}', f'{velocity_fur[i, 1]:.4f}', f'{velocity_fur[i, 2]:.4f}',
                # التسارع في إطار FUR | Acceleration in FUR frame
                f'{acceleration_fur[i, 0]:.4f}', f'{acceleration_fur[i, 1]:.4f}', f'{acceleration_fur[i, 2]:.4f}',
                # التسارع في إطار الجسم | Acceleration in body frame
                f'{acceleration_body[i, 0]:.4f}', f'{acceleration_body[i, 1]:.4f}', f'{acceleration_body[i, 2]:.4f}',
                # بيانات التحكم والرياح ومرجع MPC | Control, wind, MPC ref
                f'{control_fins[i,0]:.6f}', f'{control_fins[i,1]:.6f}', f'{control_fins[i,2]:.6f}', f'{control_fins[i,3]:.6f}',
                f'{delta_pitch[i]:.6f}', f'{delta_yaw[i]:.6f}', f'{delta_roll[i]:.6f}',
                f'{wind_ned_arr[i,0]:.4f}', f'{wind_ned_arr[i,1]:.4f}', f'{wind_ned_arr[i,2]:.4f}',
                f'{static_margin[i]:.4f}',
                f'{np.degrees(mpc_gamma_ref[i]):.4f}', f'{np.degrees(mpc_chi_ref[i]):.4f}',
                # أوامر MPC الافتراضية | MPC virtual axis commands
                f'{mpc_delta_e[i]:.6f}', f'{mpc_delta_r[i]:.6f}', f'{mpc_delta_a[i]:.6f}',
                # تشخيصات حل MPC | MPC solver diagnostics
                f'{mpc_solve_time_ms[i]:.4f}', str(int(mpc_solver_status[i])), str(int(mpc_sqp_iterations[i])),
                # أخطاء التتبع | Tracking errors
                f'{np.degrees(mpc_gamma_error[i]):.4f}', f'{np.degrees(mpc_chi_error[i]):.4f}',
                # الانحرافات المأمورة | Commanded fin deflections
                f'{actuator_commanded[i,0]:.6f}', f'{actuator_commanded[i,1]:.6f}', f'{actuator_commanded[i,2]:.6f}', f'{actuator_commanded[i,3]:.6f}',
                # ==================== البيانات المضافة | Added data ====================
                # صلاحية الزعانف والسلامة
                f'{fin_authority[i]:.6f}' if i < len(fin_authority) else '0',
                str(safety_violations[i]) if i < len(safety_violations) else '0',
                # قوة G
                f'{g_force[i]:.4f}' if i < len(g_force) else '0',
                f'{g_force_axial[i]:.4f}' if i < len(g_force_axial) else '0',
                f'{g_force_vector[i, 0]:.4f}', f'{g_force_vector[i, 1]:.4f}', f'{g_force_vector[i, 2]:.4f}',
                # سرعة الصوت
                f'{speed_of_sound[i]:.4f}' if i < len(speed_of_sound) else '0',
                # رقم المرحلة
                str(int(stage_number[i])) if i < len(stage_number) else '0',
                # معامل القوة المحورية
                f'{CA_coeff[i]:.6f}' if i < len(CA_coeff) else '0',
                # السرعة في إطار NED الثابت عند الإطلاق
                f'{vel_ned_launch[i, 0]:.4f}', f'{vel_ned_launch[i, 1]:.4f}', f'{vel_ned_launch[i, 2]:.4f}',
                # السرعة في إطار NED المحلي
                f'{vel_ned[i, 0]:.4f}', f'{vel_ned[i, 1]:.4f}', f'{vel_ned[i, 2]:.4f}',
                # السرعة النسبية للهواء في NED
                f'{velocity_for_aero[i, 0]:.4f}', f'{velocity_for_aero[i, 1]:.4f}', f'{velocity_for_aero[i, 2]:.4f}',
                # التسارع الزاوي | Angular acceleration
                f'{angular_acceleration[i, 0]:.6f}', f'{angular_acceleration[i, 1]:.6f}', f'{angular_acceleration[i, 2]:.6f}',
                f'{np.degrees(angular_acceleration[i, 0]):.4f}', f'{np.degrees(angular_acceleration[i, 1]):.4f}', f'{np.degrees(angular_acceleration[i, 2]):.4f}',
                # العزوم الكلية بعد تصحيح CG | Total moments after CG offset correction
                f'{total_moments[i, 0]:.6f}', f'{total_moments[i, 1]:.6f}', f'{total_moments[i, 2]:.6f}',
            ]
            # إضافة البيانات الشرطية | Add conditional data
            if has_position_lla:
                row.extend([f'{position_lla[i, 0]:.8f}', f'{position_lla[i, 1]:.8f}', f'{position_lla[i, 2]:.4f}'])
            if has_sensor_data:
                row.extend([
                    f'{accel_measured[i, 0]:.6f}', f'{accel_measured[i, 1]:.6f}', f'{accel_measured[i, 2]:.6f}',
                    f'{gyro_measured[i, 0]:.6f}', f'{gyro_measured[i, 1]:.6f}', f'{gyro_measured[i, 2]:.6f}',
                    f'{accel_bias[i, 0]:.6f}', f'{accel_bias[i, 1]:.6f}', f'{accel_bias[i, 2]:.6f}',
                    f'{gyro_bias[i, 0]:.6f}', f'{gyro_bias[i, 1]:.6f}', f'{gyro_bias[i, 2]:.6f}',
                ])
            if has_mhe_data:
                row.extend([f'{mhe_quality[i]:.6f}', f'{mhe_solve_ms[i]:.4f}'])
                # Unpack MHE x_hat[17] into individual columns
                for k in range(MHE_STATE_DIM):
                    row.append(f'{mhe_x_hat[i, k]:.6f}')
            writer.writerow(row)
    
    print(f"Comprehensive data log exported to: {output_path}")
    print(f"  Total data points: {n_points}")
    print(f"  Columns: {len(headers)}")


def main():
    """Example usage of 6-DOF simulation."""
    import argparse
    parser = argparse.ArgumentParser(description='6-DOF Rocket Simulation')
    parser.add_argument('--config', type=str, default=None,
                        help='Path to configuration YAML file (default: config/6dof_config_advanced.yaml)')
    parser.add_argument('--seed', type=str, default=None,
                        help='Override error_injection.seed. Use integer (e.g. 42), '
                             '"none"/"null"/"random" for random, or "auto" to use clock time.')
    args = parser.parse_args()

    config_file = args.config
    long_range_mode = False

    # Handle --seed override: parse string to int|None
    seed_override = None
    seed_was_set = False
    if args.seed is not None:
        seed_was_set = True
        s = args.seed.strip().lower()
        if s in ('none', 'null', 'random', 'auto', ''):
            seed_override = None  # null in YAML → time-based random
        else:
            try:
                seed_override = int(args.seed)
            except ValueError:
                raise SystemExit(f"--seed must be int or 'none'/'random'/'auto', got '{args.seed}'")

    # If user provides a config overlay, merge it with the base config before construction
    if config_file is not None:
        import yaml as _yaml
        import tempfile

        # Load base config
        base_config_path = Path(__file__).parent / 'config' / '6dof_config_advanced.yaml'
        with open(base_config_path, 'r', encoding='utf-8') as _f:
            base_config = _yaml.safe_load(_f)

        # Load overlay config
        with open(config_file, 'r', encoding='utf-8') as _f:
            overlay = _yaml.safe_load(_f)

        # Deep merge: overlay onto base
        for key, value in overlay.items():
            if isinstance(value, dict) and key in base_config and isinstance(base_config[key], dict):
                base_config[key].update(value)
            else:
                base_config[key] = value

        long_range_mode = base_config.get('long_range_mode', False)

        # Write merged config to temp file
        _tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False, encoding='utf-8')
        _yaml.dump(base_config, _tmp, allow_unicode=True, default_flow_style=False)
        _tmp_path = _tmp.name
        _tmp.close()
        config_file = _tmp_path

    try:
        # Initialize simulation
        print("Initializing 6-DOF simulation...")
        sim = Rocket6DOFSimulation(config_file=config_file, long_range_mode=long_range_mode)

        # Apply --seed override (after sim init to access config + error_model)
        if seed_was_set:
            ei_cfg = sim.config.get('error_injection', {})
            ei_cfg['seed'] = seed_override
            sim.config['error_injection'] = ei_cfg
            # Re-init error model RNG with new seed
            if hasattr(sim, 'error_model') and sim.error_model is not None:
                import numpy as _np
                sim.error_model.seed = seed_override
                sim.error_model._rng = _np.random.default_rng(seed_override)
                sim.error_model._errors_generated = False  # force regen on next use
                seed_str = 'random (clock)' if seed_override is None else str(seed_override)
                print(f"  [--seed] error_injection.seed overridden to: {seed_str}")

        # قراءة زمن المحاكاة وخطوة الزمن من ملف الإعدادات
        sim_config = sim.config.get('simulation', {})
        duration = sim_config.get('duration', 300.0)  # الافتراضي 300 ثانية
        dt = sim_config.get('dt', 0.01)
        
        # Read fin deflections from config file
        ballistic_fin_deflection = np.array(
            sim_config.get('ballistic_fin_deflection', [0.0, 0.0, 0.0, 0.0])
        )
        
        def control_function(state, t):
            """Control function with fin deflection from config.
            """
            # Fin deflections are read from config file (ballistic_fin_deflection)
            return ballistic_fin_deflection.copy()

        def progress_callback(info):
            """
            Progress callback to display simulation progress.
            دالة callback لعرض تقدم المحاكاة.
            """
            print(f"\r[{info['progress']:5.1f}%] t={info['time']:6.1f}s | "
                  f"alt={info['altitude']/1000:7.1f}km | "
                  f"vel={info['velocity']:7.1f}m/s | "
                  f"phase={info['flight_phase']}", end="", flush=True)

        print("Starting simulation...")
        print(f"  Duration: {duration} s, dt: {dt} s")
        print("Progress: (updating every 1 second of simulation time)")
        history = sim.simulate(
            duration=duration,
            dt=dt,
            control_function=control_function,
            on_step=progress_callback,
            callback_stride=100  # كل 100 خطوة = كل 1 ثانية (dt=0.01)
        )
        print()  # New line after progress

        final_state = sim.get_state()
        print("\nSimulation completed successfully!")
        print("Final state:")

        # Display position and velocity based on simulation mode
        if hasattr(sim, 'long_range_mode') and sim.long_range_mode:
            # ECEF mode - convert to LLA for meaningful display
            pos = final_state['position']
            vel = final_state['velocity']

            # Convert ECEF to LLA using WGS84 ellipsoid model for consistent altitude
            if hasattr(sim, 'dynamics') and hasattr(sim.dynamics, 'earth_model'):
                lat, lon, alt = sim.dynamics.earth_model.ecef_to_lla(pos[0], pos[1], pos[2])
                print(f"  Position (LLA): lat={np.degrees(lat):.4f} deg, lon={np.degrees(lon):.4f} deg, alt={alt:.2f} m")
                # Use WGS84 geodetic altitude as the primary altitude display
                altitude = alt
            else:
                print(f"  Position (ECEF): {pos}")
                # Fallback to spherical approximation only if EarthModel is not available
                earth_radius = 6371000.0  # Mean Earth radius (m)
                altitude = np.linalg.norm(pos) - earth_radius

            # Calculate velocity magnitude
            vel_mag = np.linalg.norm(vel)
            print(f"  Velocity (ECEF): {vel}")
            print(f"  Speed: {vel_mag:.2f} m/s")
            print(f"  Altitude (WGS84): {altitude:.2f} m")
        else:
            # NED mode - standard display
            print(f"  Position (NED): {final_state['position']}")
            print(f"  Velocity (NED): {final_state['velocity']}")
            print(f"  Altitude: {-final_state['position'][2]:.2f} m")
            print(f"  Speed: {np.linalg.norm(final_state['velocity']):.2f} m/s")

        print(f"  Mass: {final_state['mass']:.2f} kg")
        print(f"  Data points: {len(history['time'])}")

        # Create results directory if it doesn't exist
        output_file = Path(__file__).parent / 'results' / '6dof_results.npz'
        output_file.parent.mkdir(parents=True, exist_ok=True)

        # Save results
        # Build save dictionary with core fields
        save_data = {
            'time': np.array(history['time']),
            'position': np.array(history['position']),
            'velocity': np.array(history['velocity']),
            'attitude': np.array(history['attitude']),
            'angular_velocity': np.array(history['angular_velocity']),
        }


        # Add vel_ned_launch if available
        if 'vel_ned_launch' in history and len(history['vel_ned_launch']) > 0:
            save_data['vel_ned_launch'] = np.array(history['vel_ned_launch'])

        if 'vel_ned' in history and len(history['vel_ned']) > 0:
            save_data['vel_ned'] = np.array(history['vel_ned'])

        if 'velocity_for_aero' in history and len(history['velocity_for_aero']) > 0:
            save_data['velocity_for_aero'] = np.array(history['velocity_for_aero'])


        # Add position_fur if available (for FUR frame visualization)
        if 'position_fur' in history and len(history['position_fur']) > 0:
            save_data['position_fur'] = np.array(history['position_fur'])

        # Add long-range mode fields if available
        if sim.long_range_mode and 'ground_range_km' in history:
            save_data['ground_range_km'] = np.array(history['ground_range_km'])
            save_data['altitude_km'] = np.array(history['altitude_km'])
            save_data['position_lla'] = np.array(history['position_lla'])

            # Print final ground range for user convenience
            final_range = history['ground_range_km'][-1]
            max_alt = np.max(history['altitude_km'])
            print(f"  Ground Range: {final_range:.2f} km")
            print(f"  Max Altitude: {max_alt:.2f} km")

        # MHE estimation data (بيانات تقدير MHE)
        # MHE starts after warmup so arrays may be shorter than n_points
        if 'mhe_x_hat' in history and len(history['mhe_x_hat']) > 0:
            save_data['mhe_x_hat'] = np.array(history['mhe_x_hat'])
            n_total = len(history['time'])
            n_mhe = len(history['mhe_quality'])
            # Pad with zeros at front to align with time array
            mhe_q = np.zeros(n_total)
            mhe_s = np.zeros(n_total)
            mhe_q[n_total - n_mhe:] = np.array(history['mhe_quality'])
            mhe_s[n_total - n_mhe:] = np.array(history['mhe_solve_ms'])
            save_data['mhe_quality'] = mhe_q
            save_data['mhe_solve_ms'] = mhe_s

        np.savez(output_file, **save_data)

        print(f"Results saved to: {output_file}")

        # Get rocket name and date for filenames
        # الحصول على اسم الصاروخ والتاريخ لأسماء الملفات
        from datetime import datetime as dt
        if hasattr(sim, 'rocket_data') and sim.rocket_data is not None:
            rocket_name = sim.rocket_data.name
        else:
            rocket_name = sim.config.get('rocket_data', {}).get('name', 'unknown')
        date_str = dt.now().strftime('%Y-%m-%d_%H-%M-%S')

        # Export comprehensive data log to CSV
        try:
            log_filename = f"{rocket_name}_{date_str}_log.csv"
            log_file = Path(__file__).parent / 'results' / log_filename
            export_comprehensive_log(history, sim, str(log_file))
        except Exception as log_error:
            import traceback
            print(f"Warning: Could not export data log: {log_error}")
            traceback.print_exc()

        # Auto-run interactive HTML analysis on the saved CSV
        try:
            sys.path.insert(0, str(Path(__file__).parent / 'results'))
            from advanced_analysis import analyze_csv
            analyze_csv(log_file, open_browser=True)
        except Exception as analysis_error:
            print(f"Warning: Post-sim analysis failed: {analysis_error}")

    except Exception as e:
        print(f"Error running simulation: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
