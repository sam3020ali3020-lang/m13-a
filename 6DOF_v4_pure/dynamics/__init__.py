"""
Dynamics modules for 6-DOF rocket simulation.

This package contains physics models for rocket dynamics:
- equations_of_motion: Rigid body dynamics
- propulsion: Rocket motor model (AdvancedPropulsionModel)
- aerodynamics: Aerodynamic forces and moments (AdvancedAerodynamicsModel)
- atmosphere: Table-based atmosphere model using atmosphere_table.csv
- flight_phase: Flight phase management
- mass_properties: Time-varying mass properties
- actuator: Fin actuator dynamics with first-order lag, rate limits, and saturation

Note: Each rocket model must include an atmosphere_table.csv file.
ملاحظة: يجب أن يتضمن كل نموذج صاروخ ملف atmosphere_table.csv
"""

from .equations_of_motion import RocketDynamics
from .propulsion import AdvancedPropulsionModel as PropulsionModel
from .aerodynamics import AdvancedAerodynamicsModel as AerodynamicsModel
from .atmosphere import AtmosphereModel
from .flight_phase import FlightPhaseManager, FlightPhase
from .mass_properties import MultiStageMassPropertiesModel
from .enums import SeparationTrigger
from .actuator import ActuatorModel, ActuatorWithBacklash, SecondOrderActuatorModel
from .frame_manager import (
    FrameConsistencyManager,
    FrameConsistencyError,
    FrameConfiguration,
    FrameMode,
    create_frame_manager_from_config,
)
__all__ = [
    'RocketDynamics',
    'PropulsionModel',
    'AerodynamicsModel',
    'AtmosphereModel',
    'FlightPhaseManager',
    'FlightPhase',
    'MultiStageMassPropertiesModel',
    'SeparationTrigger',
    'ActuatorModel',
    'ActuatorWithBacklash',
    'SecondOrderActuatorModel',
    'FrameConsistencyManager',
    'FrameConsistencyError',
    'FrameConfiguration',
    'FrameMode',
    'create_frame_manager_from_config',
]
