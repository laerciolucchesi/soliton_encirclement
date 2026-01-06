"""Velocity-mobility building blocks.

This package contains the reusable velocity mobility handler and its pure
math core. It is intended to be imported by a larger project.

Back-compat:
- The legacy module name `gradysim_velocity_mobility` is kept as a shim.

Author: Laércio Lucchesi
"""

from .config import VelocityMobilityConfiguration
from .handler import VelocityMobilityHandler
from .core import (
    apply_acceleration_limits,
    apply_velocity_limits,
    apply_velocity_tracking_first_order,
    integrate_position,
)

__version__ = "0.1.0"

__all__ = [
    "VelocityMobilityConfiguration",
    "VelocityMobilityHandler",
    "apply_acceleration_limits",
    "apply_velocity_limits",
    "apply_velocity_tracking_first_order",
    "integrate_position",
]
