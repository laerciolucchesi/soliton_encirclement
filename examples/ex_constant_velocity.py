"""Core-only example (no GrADyS-SIM runtime required).

This script demonstrates the *pure* mathematical functions exposed by
`velocity_mobility.core`:

- acceleration-limited velocity tracking
- velocity saturation
- Euler position integration

It intentionally does NOT build a GrADyS-SIM NG simulation. For the full
integration example (handler + protocol + visualization), use `main.py` and
`protocol.py` at the repository root.

Usage:
    python .\examples\ex_constant_velocity.py
"""

from velocity_mobility import (
    VelocityMobilityConfiguration,
    apply_acceleration_limits,
    apply_velocity_limits,
    integrate_position
)


def simulate_constant_velocity():
    """
    Simulate a node moving with constant velocity using the core functions.
    """
    print("Core-only demo: acceleration limits + velocity saturation + Euler integration")
    
    # Configuration
    config = VelocityMobilityConfiguration(
        update_rate=0.1,        # Update every 0.1 seconds
        max_speed_xy=10.0,      # Max horizontal speed: 10 m/s
        max_speed_z=5.0,        # Max vertical speed: 5 m/s
        max_acc_xy=5.0,         # Max horizontal acceleration: 5 m/s²
        max_acc_z=2.0,          # Max vertical acceleration: 2 m/s²
    )
    
    # Initial state
    position = (0.0, 0.0, 0.0)
    velocity = (0.0, 0.0, 0.0)
    desired_velocity = (3.54, 3.54, 2.0)  # Northeast at 5 m/s, ascending at 2 m/s
    
    print(f"Desired velocity: {desired_velocity} m/s")
    print(f"dt: {config.update_rate} s")
    print("-" * 60)
    print(f"{'t (s)':>6} | {'pos (m)':^26} | {'vel (m/s)':^26}")
    print("-" * 60)
    
    # Simulate for 10 seconds
    duration = 10.0
    num_steps = int(duration / config.update_rate)
    
    for step in range(num_steps + 1):
        time = step * config.update_rate
        
        # Print every ~1 second (works nicely for dt=0.1)
        if step % max(1, int(round(1.0 / config.update_rate))) == 0:
            pos_str = f"({position[0]:6.2f}, {position[1]:6.2f}, {position[2]:6.2f})"
            vel_str = f"({velocity[0]:5.2f}, {velocity[1]:5.2f}, {velocity[2]:5.2f})"
            print(f"{time:>6.1f} | {pos_str:^26} | {vel_str:^26}")
        
        # Apply acceleration limits
        velocity = apply_acceleration_limits(
            velocity,
            desired_velocity,
            config.update_rate,
            config.max_acc_xy,
            config.max_acc_z
        )
        
        # Apply velocity limits
        velocity = apply_velocity_limits(
            velocity,
            config.max_speed_xy,
            config.max_speed_z
        )
        
        # Update position
        position = integrate_position(position, velocity, config.update_rate)
    
    print("-" * 60)
    print(f"Final position: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}) m")
    print(f"Final velocity: ({velocity[0]:.2f}, {velocity[1]:.2f}, {velocity[2]:.2f}) m/s")


if __name__ == "__main__":
    simulate_constant_velocity()
