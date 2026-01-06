"""Legacy shim module.

Implementation moved to `velocity_mobility.core`.
"""

from velocity_mobility.core import (  # noqa: F401
    apply_acceleration_limits,
    apply_velocity_limits,
    apply_velocity_tracking_first_order,
    integrate_position,
)

__all__ = [
    "apply_acceleration_limits",
    "apply_velocity_limits",
    "apply_velocity_tracking_first_order",
    "integrate_position",
]
