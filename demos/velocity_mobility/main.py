"""Velocity mobility demo (isolated).

This script builds a GrADyS-SIM simulation with a single node controlled by
VelocityMobilityHandler. The node's velocity is commanded by VelocityProtocol
in initialize(), and then changed periodically via a timer (every 10 seconds).

Run:
    python demos/velocity_mobility/main.py
"""

import logging

# Suppress websockets handshake warnings
logging.getLogger('websockets').setLevel(logging.CRITICAL)

from gradysim.simulator.handler.communication import CommunicationHandler, CommunicationMedium
from gradysim.simulator.handler.timer import TimerHandler
from gradysim.simulator.handler.visualization import VisualizationHandler, VisualizationConfiguration
from gradysim.simulator.simulation import SimulationBuilder, SimulationConfiguration
from velocity_mobility import VelocityMobilityHandler, VelocityMobilityConfiguration

# Allow running as either:
# - module:  python -m demos.velocity_mobility.main
# - script:  python demos/velocity_mobility/main.py
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    repo_root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(repo_root))
    from demos.velocity_mobility.protocol import VelocityProtocol
else:
    from .protocol import VelocityProtocol


# ============================================================
# Mobility presets (choose by editing ONE variable)
#
# Profiles: Cinematic, Survey, Cargo, Racing, Micro, Custom
# - For the first five profiles, values below are ALREADY the final values
#   (80% of the README table maxima, rounded to the same decimals as the table).
# - Custom uses the explicit values defined below (the original values).
# ============================================================

MOBILITY_PROFILE: str = "Cargo"  # Choose mobility profile here


CUSTOM_MOBILITY_CONFIG = VelocityMobilityConfiguration(
    update_rate=0.01,        # Update every 0.01 seconds
    max_speed_xy=5.0,        # Max horizontal speed: 5 m/s
    max_speed_z=5.0,         # Max vertical speed: 5 m/s
    max_acc_xy=2.5,          # Max horizontal acceleration: 2.5 m/s²
    max_acc_z=2.5,           # Max vertical acceleration: 2.5 m/s²
    tau_xy=0.5,              # Optional: 1st-order horizontal tracking time constant (s)
    tau_z=0.8,               # Optional: 1st-order vertical tracking time constant (s)
    send_telemetry=True,     # Enable telemetry
    telemetry_decimation=1,  # Send telemetry every update
)


MOBILITY_PRESETS: dict[str, VelocityMobilityConfiguration] = {
    "Cinematic": VelocityMobilityConfiguration(
        update_rate=0.04,
        max_speed_xy=10.0,
        max_speed_z=5.0,
        max_acc_xy=4.0,
        max_acc_z=5.0,
        tau_xy=1.0,
        tau_z=1.2,
        send_telemetry=True,
        telemetry_decimation=1,
    ),
    "Survey": VelocityMobilityConfiguration(
        update_rate=0.04,
        max_speed_xy=12.0,
        max_speed_z=2.0,
        max_acc_xy=3.0,
        max_acc_z=4.0,
        tau_xy=1.0,
        tau_z=1.4,
        send_telemetry=True,
        telemetry_decimation=1,
    ),
    "Cargo": VelocityMobilityConfiguration(
        update_rate=0.04,
        max_speed_xy=8.0,
        max_speed_z=3.0,
        max_acc_xy=2.0,
        max_acc_z=4.0,
        tau_xy=1.2,
        tau_z=1.6,
        send_telemetry=True,
        telemetry_decimation=1,
    ),
    "Racing": VelocityMobilityConfiguration(
        update_rate=0.02,
        max_speed_xy=32.0,
        max_speed_z=16.0,
        max_acc_xy=20.0,
        max_acc_z=20.0,
        tau_xy=0.3,
        tau_z=0.5,
        send_telemetry=True,
        telemetry_decimation=1,
    ),
    "Micro": VelocityMobilityConfiguration(
        update_rate=0.02,
        max_speed_xy=5.0,
        max_speed_z=2.0,
        max_acc_xy=8.0,
        max_acc_z=10.0,
        tau_xy=0.6,
        tau_z=0.7,
        send_telemetry=True,
        telemetry_decimation=1,
    ),
    "Custom": CUSTOM_MOBILITY_CONFIG,
}


def main():
    """Execute the velocity mobility simulation."""

    duration = 50
    debug = False
    real_time = True

    builder = SimulationBuilder(
        SimulationConfiguration(
            duration=duration,
            debug=debug,
            real_time=real_time,
        )
    )

    transmission_range = 200
    delay = 0.0
    failure_rate = 0.0
    medium = CommunicationMedium(
        transmission_range=transmission_range,
        delay=delay,
        failure_rate=failure_rate,
    )
    builder.add_handler(CommunicationHandler(medium))

    builder.add_handler(TimerHandler())

    profile = (MOBILITY_PROFILE or "").strip()
    mobility_config = MOBILITY_PRESETS.get(profile)
    if mobility_config is None:
        valid = ", ".join(sorted(MOBILITY_PRESETS.keys()))
        raise ValueError(f"Unknown MOBILITY_PROFILE={MOBILITY_PROFILE!r}. Valid options: {valid}")

    print(
        "Mobility preset: "
        f"{profile} "
        f"(update_rate={mobility_config.update_rate}, "
        f"max_speed_xy={mobility_config.max_speed_xy}, max_speed_z={mobility_config.max_speed_z}, "
        f"max_acc_xy={mobility_config.max_acc_xy}, max_acc_z={mobility_config.max_acc_z}, "
        f"tau_xy={mobility_config.tau_xy}, tau_z={mobility_config.tau_z})"
    )

    velocity_handler = VelocityMobilityHandler(mobility_config)
    builder.add_handler(velocity_handler)

    vis_config = VisualizationConfiguration(
        open_browser=True,
        update_rate=0.1,
    )
    builder.add_handler(VisualizationHandler(vis_config))

    builder.add_node(VelocityProtocol, (-25, -25, -25))

    simulation = builder.build()
    print("=" * 60)
    print("Starting velocity mobility simulation")
    print("Node velocity is commanded by VelocityProtocol (and changes every 10 seconds)")
    print("Starting position: (-25, -25, -25)")
    print("Visualization will open in browser automatically")
    print("=" * 60)
    try:
        simulation.start_simulation()
    except (BrokenPipeError, EOFError) as e:
        logging.getLogger(__name__).debug(f"Ignored visualization shutdown error: {e}")
    finally:
        print("=" * 60)
        print("Simulation completed!")
        print("=" * 60)


if __name__ == "__main__":
    main()
