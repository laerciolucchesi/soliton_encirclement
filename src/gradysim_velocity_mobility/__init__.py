"""Compatibility shim for the legacy module name.

The implementation has moved to the `velocity_mobility` package.

This module keeps backward compatibility for existing code that imports:

- `gradysim_velocity_mobility`
- `gradysim_velocity_mobility.core`
- `gradysim_velocity_mobility.config`
- `gradysim_velocity_mobility.handler`
"""

from velocity_mobility import (  # noqa: F401
    VelocityMobilityConfiguration,
    VelocityMobilityHandler,
    apply_acceleration_limits,
    apply_velocity_limits,
    apply_velocity_tracking_first_order,
    integrate_position,
)

from velocity_mobility import __version__  # noqa: F401

__all__ = [
    "VelocityMobilityConfiguration",
    "VelocityMobilityHandler",
    "apply_acceleration_limits",
    "apply_velocity_limits",
    "apply_velocity_tracking_first_order",
    "integrate_position",
]
