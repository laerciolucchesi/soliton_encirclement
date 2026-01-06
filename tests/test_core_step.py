"""
Tests for position integration step.

Tests the numerical integration function that updates position
based on velocity and time step.
"""

import pytest
from velocity_mobility.core import integrate_position


class TestPositionIntegration:
    """Test position integration using Euler method."""
    
    def test_stationary_node(self):
        """Zero velocity should result in no position change."""
        pos = (10.0, 20.0, 30.0)
        vel = (0.0, 0.0, 0.0)
        new_pos = integrate_position(pos, vel, dt=0.1)
        assert new_pos == pytest.approx(pos)
    
    def test_horizontal_motion(self):
        """Horizontal velocity should update x and y coordinates."""
        pos = (0.0, 0.0, 0.0)
        vel = (1.0, 2.0, 0.0)
        new_pos = integrate_position(pos, vel, dt=0.1)
        assert new_pos == pytest.approx((0.1, 0.2, 0.0))
    
    def test_vertical_motion(self):
        """Vertical velocity should update z coordinate."""
        pos = (0.0, 0.0, 0.0)
        vel = (0.0, 0.0, 5.0)
        new_pos = integrate_position(pos, vel, dt=0.1)
        assert new_pos == pytest.approx((0.0, 0.0, 0.5))
    
    def test_3d_motion(self):
        """All three velocity components should affect position."""
        pos = (1.0, 2.0, 3.0)
        vel = (10.0, 20.0, 30.0)
        new_pos = integrate_position(pos, vel, dt=0.1)
        assert new_pos == pytest.approx((2.0, 4.0, 6.0))
    
    def test_negative_velocity(self):
        """Negative velocities should move in opposite directions."""
        pos = (10.0, 10.0, 10.0)
        vel = (-5.0, -5.0, -5.0)
        new_pos = integrate_position(pos, vel, dt=1.0)
        assert new_pos == pytest.approx((5.0, 5.0, 5.0))
    
    def test_time_step_scaling(self):
        """Different time steps should scale displacement proportionally."""
        pos = (0.0, 0.0, 0.0)
        vel = (1.0, 1.0, 1.0)
        
        new_pos_small_dt = integrate_position(pos, vel, dt=0.1)
        new_pos_large_dt = integrate_position(pos, vel, dt=1.0)
        
        # Large dt should give 10x the displacement
        assert new_pos_small_dt == pytest.approx((0.1, 0.1, 0.1))
        assert new_pos_large_dt == pytest.approx((1.0, 1.0, 1.0))
    
    def test_multiple_steps_trajectory(self):
        """Multiple integration steps should produce expected trajectory."""
        pos = (0.0, 0.0, 0.0)
        vel = (1.0, 0.0, 0.5)
        dt = 0.1
        
        # Simulate 10 steps (1 second)
        for _ in range(10):
            pos = integrate_position(pos, vel, dt)
        
        # After 1 second at constant velocity
        assert pos == pytest.approx((1.0, 0.0, 0.5))
    
    def test_non_origin_start(self):
        """Integration should work correctly from any starting position."""
        pos = (100.0, 200.0, 50.0)
        vel = (2.0, -1.0, 0.5)
        new_pos = integrate_position(pos, vel, dt=2.0)
        assert new_pos == pytest.approx((104.0, 198.0, 51.0))
    
    def test_precision_with_small_velocities(self):
        """Small velocities should produce accurate small displacements."""
        pos = (0.0, 0.0, 0.0)
        vel = (0.001, 0.002, 0.003)
        new_pos = integrate_position(pos, vel, dt=0.1)
        assert new_pos == pytest.approx((0.0001, 0.0002, 0.0003))
