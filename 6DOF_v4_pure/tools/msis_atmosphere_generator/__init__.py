"""
MSIS Atmosphere Table Generator
مولد جداول الغلاف الجوي MSIS

This package provides tools for generating high-fidelity atmosphere tables
using the NRLMSISE-00/MSIS2.1 empirical atmospheric model.

هذه الحزمة توفر أدوات لتوليد جداول غلاف جوي عالية الدقة
باستخدام نموذج MSIS التجريبي.
"""

from .msis_generator import (
    MSISAtmosphereGenerator,
    find_rocket_models,
    get_rocket_launch_coordinates,
    load_config,
    generate_for_all_rockets,
)

__version__ = '1.0.0'
__author__ = ''

__all__ = [
    'MSISAtmosphereGenerator',
    'find_rocket_models',
    'get_rocket_launch_coordinates',
    'load_config',
    'generate_for_all_rockets',
]
