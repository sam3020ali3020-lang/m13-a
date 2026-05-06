"""
Shared enumerations for 6-DOF rocket simulation dynamics.

This module contains enumerations that are shared across multiple dynamics modules
to avoid duplication and ensure consistency.
"""

from enum import IntEnum


class SeparationTrigger(IntEnum):
    """Separation trigger types for multi-stage rockets.

    BURNOUT/TIME/ALTITUDE/VELOCITY are auto-checked by separation methods.
    MANUAL requires direct call to execute_separation().
    Use SeparationTrigger.from_config(value) to parse from int, str, or enum.
    """
    BURNOUT = 0      # Separate at burnout of current stage
    TIME = 1         # Separate at specific time
    ALTITUDE = 2     # Separate at specific altitude
    VELOCITY = 3     # Separate at specific velocity
    MANUAL = 4       # Manual separation - requires direct call to execute_separation()
    
    @classmethod
    def from_config(cls, value) -> 'SeparationTrigger':
        """Parse trigger from int, str, or SeparationTrigger. Raises ValueError if invalid."""
        # Already a SeparationTrigger
        if isinstance(value, cls):
            return value
        
        # Integer value (exclude bool since bool is subclass of int in Python)
        # True would be parsed as TIME (1), False as BURNOUT (0) - unexpected behavior
        if isinstance(value, int) and not isinstance(value, bool):
            try:
                return cls(value)
            except ValueError:
                valid_values = [f"{t.name}={t.value}" for t in cls]
                raise ValueError(
                    f"Invalid separation trigger value: {value}. "
                    f"Valid values: {', '.join(valid_values)}"
                )
        
        # String name (case-insensitive)
        if isinstance(value, str):
            value_upper = value.upper().strip()
            for trigger in cls:
                if trigger.name == value_upper:
                    return trigger
            valid_names = [t.name for t in cls]
            raise ValueError(
                f"Invalid separation trigger name: '{value}'. "
                f"Valid names: {', '.join(valid_names)}"
            )
        
        raise ValueError(
            f"Cannot parse separation trigger from type {type(value).__name__}. "
            f"Expected int, str, or SeparationTrigger."
        )
