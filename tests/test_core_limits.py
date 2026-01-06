"""
Tests for velocity and acceleration limit functions.

Tests the mathematical correctness of velocity saturation and
acceleration limiting in the core module.
"""

import pytest
import math
from velocity_mobility.core import (
    apply_velocity_limits,
    apply_acceleration_limits,
    apply_velocity_tracking_first_order,
)


class TestVelocityLimits:
    """Test velocity saturation constraints."""
    
    def test_no_saturation_needed(self):
        """Velocity within limits should pass through unchanged."""
        v = (5.0, 5.0, 2.0)
        v_sat = apply_velocity_limits(v, max_speed_xy=10.0, max_speed_z=5.0)
        assert v_sat == pytest.approx(v)
    
    def test_horizontal_saturation(self):
        """Horizontal velocity exceeding limit should be scaled down."""
        v = (10.0, 10.0, 0.0)  # Norm is 14.14 m/s
        v_sat = apply_velocity_limits(v, max_speed_xy=10.0, max_speed_z=5.0)
        
        # Check horizontal norm is at limit
        vx, vy, vz = v_sat
        horizontal_norm = math.sqrt(vx**2 + vy**2)
        assert horizontal_norm == pytest.approx(10.0)
        
        # Check direction is preserved
        assert vx == pytest.approx(vy)
        assert vz == 0.0
    
    def test_vertical_saturation_positive(self):
        """Positive vertical velocity exceeding limit should be clamped."""
        v = (0.0, 0.0, 10.0)
        v_sat = apply_velocity_limits(v, max_speed_xy=10.0, max_speed_z=5.0)
        assert v_sat == pytest.approx((0.0, 0.0, 5.0))
    
    def test_vertical_saturation_negative(self):
        """Negative vertical velocity exceeding limit should be clamped."""
        v = (0.0, 0.0, -10.0)
        v_sat = apply_velocity_limits(v, max_speed_xy=10.0, max_speed_z=5.0)
        assert v_sat == pytest.approx((0.0, 0.0, -5.0))
    
    def test_both_axes_saturation(self):
        """Both horizontal and vertical saturation can apply simultaneously."""
        v = (15.0, 15.0, 10.0)
        v_sat = apply_velocity_limits(v, max_speed_xy=10.0, max_speed_z=5.0)
        
        vx, vy, vz = v_sat
        horizontal_norm = math.sqrt(vx**2 + vy**2)
        
        assert horizontal_norm == pytest.approx(10.0)
        assert abs(vz) == pytest.approx(5.0)
    
    def test_zero_velocity(self):
        """Zero velocity should remain zero."""
        v = (0.0, 0.0, 0.0)
        v_sat = apply_velocity_limits(v, max_speed_xy=10.0, max_speed_z=5.0)
        assert v_sat == pytest.approx((0.0, 0.0, 0.0))


class TestAccelerationLimits:
    """Test acceleration-limited velocity tracking."""
    
    def test_no_limiting_needed(self):
        """Small velocity changes should pass through unchanged."""
        v_cur = (0.0, 0.0, 0.0)
        v_des = (0.1, 0.1, 0.05)
        v_new = apply_acceleration_limits(
            v_cur, v_des, dt=0.1, max_acc_xy=2.0, max_acc_z=1.0
        )
        assert v_new == pytest.approx(v_des)
    
    def test_horizontal_acceleration_limiting(self):
        """Large horizontal velocity change should be limited by max_acc_xy."""
        v_cur = (0.0, 0.0, 0.0)
        v_des = (10.0, 10.0, 0.0)  # Desired change norm is 14.14 m/s
        v_new = apply_acceleration_limits(
            v_cur, v_des, dt=0.1, max_acc_xy=2.0, max_acc_z=1.0
        )
        
        # Maximum allowed change is 2.0 m/s² * 0.1 s = 0.2 m/s
        vx, vy, vz = v_new
        change_norm = math.sqrt(vx**2 + vy**2)
        assert change_norm == pytest.approx(0.2)
        
        # Direction should be preserved
        assert vx == pytest.approx(vy)
        assert vz == 0.0
    
    def test_vertical_acceleration_limiting_positive(self):
        """Large positive vertical change should be limited by max_acc_z."""
        v_cur = (0.0, 0.0, 0.0)
        v_des = (0.0, 0.0, 10.0)
        v_new = apply_acceleration_limits(
            v_cur, v_des, dt=0.1, max_acc_xy=2.0, max_acc_z=1.0
        )
        
        # Maximum allowed change is 1.0 m/s² * 0.1 s = 0.1 m/s
        assert v_new == pytest.approx((0.0, 0.0, 0.1))
    
    def test_vertical_acceleration_limiting_negative(self):
        """Large negative vertical change should be limited by max_acc_z."""
        v_cur = (0.0, 0.0, 5.0)
        v_des = (0.0, 0.0, -5.0)
        v_new = apply_acceleration_limits(
            v_cur, v_des, dt=0.1, max_acc_xy=2.0, max_acc_z=1.0
        )
        
        # Maximum allowed change is 1.0 m/s² * 0.1 s = 0.1 m/s
        assert v_new == pytest.approx((0.0, 0.0, 4.9))
    
    def test_both_axes_acceleration_limiting(self):
        """Both horizontal and vertical limiting can apply simultaneously."""
        v_cur = (0.0, 0.0, 0.0)
        v_des = (10.0, 10.0, 10.0)
        v_new = apply_acceleration_limits(
            v_cur, v_des, dt=0.1, max_acc_xy=2.0, max_acc_z=1.0
        )
        
        vx, vy, vz = v_new
        horizontal_change = math.sqrt(vx**2 + vy**2)
        
        assert horizontal_change == pytest.approx(0.2)
        assert abs(vz) == pytest.approx(0.1)
    
    def test_incremental_approach_to_target(self):
        """Multiple steps should gradually approach desired velocity."""
        v_cur = (0.0, 0.0, 0.0)
        v_des = (1.0, 0.0, 0.0)
        dt = 0.1
        max_acc_xy = 2.0
        max_acc_z = 1.0
        
        # After 5 steps, we should reach the target
        for _ in range(5):
            v_cur = apply_acceleration_limits(
                v_cur, v_des, dt, max_acc_xy, max_acc_z
            )
        
        assert v_cur == pytest.approx(v_des, abs=1e-6)
    
    def test_stopping_gradually(self):
        """Commanding zero velocity should decelerate gradually."""
        v_cur = (2.0, 0.0, 1.0)
        v_des = (0.0, 0.0, 0.0)
        v_new = apply_acceleration_limits(
            v_cur, v_des, dt=0.1, max_acc_xy=2.0, max_acc_z=1.0
        )
        
        # Should move toward zero but not reach it immediately
        vx, vy, vz = v_new
        assert abs(vx) < 2.0
        assert abs(vx) > 0.0
        assert abs(vz) < 1.0
        assert abs(vz) > 0.0


class TestFirstOrderTracking:
    """Test optional 1st-order velocity tracking (tau) + acceleration saturation."""

    def test_xy_first_order_no_saturation(self):
        """With tau, dv should be proportional to error (Euler discretization)."""
        v_cur = (0.0, 0.0, 0.0)
        v_des = (1.0, 0.0, 0.0)
        dt = 0.1
        tau_xy = 1.0

        v_new = apply_velocity_tracking_first_order(
            v_cur,
            v_des,
            dt=dt,
            max_acc_xy=10.0,
            max_acc_z=10.0,
            tau_xy=tau_xy,
            tau_z=None,
        )

        # a = (1-0)/1 = 1 m/s^2, dv = a*dt = 0.1 m/s
        assert v_new == pytest.approx((0.1, 0.0, 0.0))

    def test_xy_first_order_with_saturation(self):
        """With tau but large error, acceleration should saturate."""
        v_cur = (0.0, 0.0, 0.0)
        v_des = (10.0, 0.0, 0.0)
        dt = 0.1
        tau_xy = 0.1  # a* = 100 m/s^2

        v_new = apply_velocity_tracking_first_order(
            v_cur,
            v_des,
            dt=dt,
            max_acc_xy=2.0,
            max_acc_z=10.0,
            tau_xy=tau_xy,
            tau_z=None,
        )

        # saturated a = 2 m/s^2 => dv = 0.2 m/s
        assert v_new == pytest.approx((0.2, 0.0, 0.0))

    def test_z_first_order_with_saturation(self):
        """Vertical axis uses |az| saturation."""
        v_cur = (0.0, 0.0, 0.0)
        v_des = (0.0, 0.0, 5.0)
        dt = 0.1
        tau_z = 0.1  # a* = 50 m/s^2

        v_new = apply_velocity_tracking_first_order(
            v_cur,
            v_des,
            dt=dt,
            max_acc_xy=10.0,
            max_acc_z=1.0,
            tau_xy=None,
            tau_z=tau_z,
        )

        # saturated az = 1 m/s^2 => dvz = 0.1 m/s
        assert v_new == pytest.approx((0.0, 0.0, 0.1))

    def test_invalid_tau_raises(self):
        """Non-positive tau is invalid."""
        with pytest.raises(ValueError):
            apply_velocity_tracking_first_order(
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                dt=0.1,
                max_acc_xy=1.0,
                max_acc_z=1.0,
                tau_xy=0.0,
                tau_z=None,
            )
