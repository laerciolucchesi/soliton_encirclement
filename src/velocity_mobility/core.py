"""Pure mathematical functions for velocity-based mobility.

This module contains stateless mathematical operations for:
- Acceleration limiting
- Velocity saturation
- Position integration

All functions operate on simple tuples and floats, making them
easy to test and reuse independently of the simulation framework.

Author: Laércio Lucchesi
Date: December 27, 2025
"""

import math
from typing import Optional, Tuple


def apply_acceleration_limits(
    v_current: Tuple[float, float, float],
    v_desired: Tuple[float, float, float],
    dt: float,
    max_acc_xy: float,
    max_acc_z: float,
) -> Tuple[float, float, float]:
    """Limit velocity change based on acceleration constraints.

    Applies independent horizontal and vertical acceleration limits:
    - Horizontal: ||a_xy|| ≤ max_acc_xy
    - Vertical: |a_z| ≤ max_acc_z

    Args:
        v_current: Current velocity (vx, vy, vz) in m/s.
        v_desired: Desired velocity (vx, vy, vz) in m/s.
        dt: Time step in seconds.
        max_acc_xy: Maximum horizontal acceleration in m/s².
        max_acc_z: Maximum vertical acceleration in m/s².

    Returns:
        New velocity after applying acceleration limits.
    """
    vx_cur, vy_cur, vz_cur = v_current
    vx_des, vy_des, vz_des = v_desired

    # Horizontal acceleration limiting
    dvx = vx_des - vx_cur
    dvy = vy_des - vy_cur
    dv_xy_norm = math.hypot(dvx, dvy)

    max_dv_xy = max_acc_xy * dt

    if not math.isfinite(dv_xy_norm):
        # Extreme desired velocities: fall back to max acceleration step in the
        # direction implied by dv.
        sx = 0.0 if dvx == 0 else math.copysign(1.0, dvx)
        sy = 0.0 if dvy == 0 else math.copysign(1.0, dvy)
        s_norm = math.hypot(sx, sy)
        if s_norm > 0:
            dvx = (max_dv_xy * sx) / s_norm
            dvy = (max_dv_xy * sy) / s_norm
        else:
            dvx = 0.0
            dvy = 0.0
    elif dv_xy_norm > max_dv_xy:
        scale = max_dv_xy / dv_xy_norm
        dvx *= scale
        dvy *= scale

    vx_new = vx_cur + dvx
    vy_new = vy_cur + dvy

    # Vertical acceleration limiting
    dvz = vz_des - vz_cur
    max_dvz = max_acc_z * dt

    if abs(dvz) > max_dvz:
        dvz = math.copysign(max_dvz, dvz)

    vz_new = vz_cur + dvz

    return (vx_new, vy_new, vz_new)


def apply_velocity_tracking_first_order(
    v_current: Tuple[float, float, float],
    v_desired: Tuple[float, float, float],
    dt: float,
    max_acc_xy: float,
    max_acc_z: float,
    tau_xy: Optional[float],
    tau_z: Optional[float],
) -> Tuple[float, float, float]:
    """Track desired velocity using a 1st-order lag + acceleration saturation.

    Trajectory-level model:

        a* = (v_des - v) / tau
        a  = sat(a*, max_acc)
        v+ = v + a * dt

    Horizontal saturation is applied on the norm of (ax, ay), while vertical
    saturation is applied on |az|.

    When tau_xy/tau_z are None, this function falls back to the legacy
    acceleration-limited step behavior for that axis.
    """
    if dt <= 0:
        raise ValueError("dt must be > 0")
    if max_acc_xy < 0 or max_acc_z < 0:
        raise ValueError("max_acc must be >= 0")
    if tau_xy is not None and tau_xy <= 0:
        raise ValueError("tau_xy must be > 0 when provided")
    if tau_z is not None and tau_z <= 0:
        raise ValueError("tau_z must be > 0 when provided")

    vx_cur, vy_cur, vz_cur = v_current
    vx_des, vy_des, vz_des = v_desired

    # Horizontal (xy)
    if tau_xy is None:
        dvx = vx_des - vx_cur
        dvy = vy_des - vy_cur
        dv_xy_norm = math.hypot(dvx, dvy)
        max_dv_xy = max_acc_xy * dt
        if not math.isfinite(dv_xy_norm):
            sx = 0.0 if dvx == 0 else math.copysign(1.0, dvx)
            sy = 0.0 if dvy == 0 else math.copysign(1.0, dvy)
            s_norm = math.hypot(sx, sy)
            if s_norm > 0:
                dvx = (max_dv_xy * sx) / s_norm
                dvy = (max_dv_xy * sy) / s_norm
            else:
                dvx = 0.0
                dvy = 0.0
        elif dv_xy_norm > max_dv_xy and dv_xy_norm > 0:
            scale = max_dv_xy / dv_xy_norm
            dvx *= scale
            dvy *= scale
        vx_new = vx_cur + dvx
        vy_new = vy_cur + dvy
    else:
        ax_des = (vx_des - vx_cur) / tau_xy
        ay_des = (vy_des - vy_cur) / tau_xy
        a_xy_norm = math.hypot(ax_des, ay_des)
        if not math.isfinite(a_xy_norm):
            # Extreme desired acceleration: saturate using only direction signs.
            sx = 0.0 if ax_des == 0 else math.copysign(1.0, ax_des)
            sy = 0.0 if ay_des == 0 else math.copysign(1.0, ay_des)
            s_norm = math.hypot(sx, sy)
            if s_norm > 0:
                ax_des = (max_acc_xy * sx) / s_norm
                ay_des = (max_acc_xy * sy) / s_norm
            else:
                ax_des = 0.0
                ay_des = 0.0
        elif a_xy_norm > max_acc_xy and a_xy_norm > 0:
            scale = max_acc_xy / a_xy_norm
            ax_des *= scale
            ay_des *= scale
        vx_new = vx_cur + ax_des * dt
        vy_new = vy_cur + ay_des * dt

    # Vertical (z)
    if tau_z is None:
        dvz = vz_des - vz_cur
        max_dvz = max_acc_z * dt
        if abs(dvz) > max_dvz:
            dvz = math.copysign(max_dvz, dvz)
        vz_new = vz_cur + dvz
    else:
        az_des = (vz_des - vz_cur) / tau_z
        if abs(az_des) > max_acc_z:
            az_des = math.copysign(max_acc_z, az_des)
        vz_new = vz_cur + az_des * dt

    return (vx_new, vy_new, vz_new)


def apply_velocity_limits(
    v: Tuple[float, float, float],
    max_speed_xy: float,
    max_speed_z: float,
) -> Tuple[float, float, float]:
    """Apply velocity saturation constraints."""
    vx, vy, vz = v

    v_xy_norm = math.hypot(vx, vy)
    if not math.isfinite(v_xy_norm):
        # If overflowed upstream, force a safe saturated velocity.
        sx = 0.0 if vx == 0 else math.copysign(1.0, vx)
        sy = 0.0 if vy == 0 else math.copysign(1.0, vy)
        s_norm = math.hypot(sx, sy)
        if s_norm > 0:
            vx = (max_speed_xy * sx) / s_norm
            vy = (max_speed_xy * sy) / s_norm
        else:
            vx = 0.0
            vy = 0.0
    elif v_xy_norm > max_speed_xy:
        scale = max_speed_xy / v_xy_norm
        vx *= scale
        vy *= scale

    if abs(vz) > max_speed_z:
        vz = math.copysign(max_speed_z, vz)

    return (vx, vy, vz)


def integrate_position(
    position: Tuple[float, float, float],
    velocity: Tuple[float, float, float],
    dt: float,
) -> Tuple[float, float, float]:
    """Update position using simple Euler integration."""
    x, y, z = position
    vx, vy, vz = velocity

    return (
        x + vx * dt,
        y + vy * dt,
        z + vz * dt,
    )
