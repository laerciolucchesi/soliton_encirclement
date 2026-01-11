"""This script builds a GrADyS-SIM simulation with a 1 target node and n agent 
nodes controlled by soliton-like to enforce encirclement behavior.
The target node moves according to a random trajectory, while the agent nodes
attempt to encircle the target at a specified radius equally spaced between them.
Run:
    python main.py
"""

# Suppress websockets handshake warnings
import logging
logging.getLogger('websockets').setLevel(logging.CRITICAL)

import os
from datetime import datetime

from gradysim.simulator.handler.communication import CommunicationHandler, CommunicationMedium
from gradysim.simulator.handler.timer import TimerHandler
from gradysim.simulator.handler.visualization import VisualizationHandler, VisualizationConfiguration
from gradysim.simulator.simulation import SimulationBuilder, SimulationConfiguration
from protocol_target import TargetProtocol
from protocol_agent import AgentProtocol
from protocol_adversary import AdversaryProtocol
from velocity_mobility import VelocityMobilityHandler, VelocityMobilityConfiguration
from config_param import (
    COMMUNICATION_DELAY,
    COMMUNICATION_FAILURE_RATE,
    COMMUNICATION_TRANSMISSION_RANGE,
    ENCIRCLEMENT_RADIUS,
    NUM_AGENTS,
    SIM_DEBUG,
    SIM_DURATION,
    SIM_REAL_TIME,
    VIS_OPEN_BROWSER,
    VIS_UPDATE_RATE,
    VM_MAX_ACC_XY,
    VM_MAX_ACC_Z,
    VM_MAX_SPEED_XY,
    VM_MAX_SPEED_Z,
    VM_SEND_TELEMETRY,
    VM_TAU_XY,
    VM_TAU_Z,
    VM_TELEMETRY_DECIMATION,
    VM_UPDATE_RATE,
)
import math
import random


mobility_config = VelocityMobilityConfiguration(
    update_rate=VM_UPDATE_RATE,        # Update every in seconds
    max_speed_xy=VM_MAX_SPEED_XY,      # Max horizontal speed in m/s
    max_speed_z=VM_MAX_SPEED_Z,        # Max vertical speed in m/s
    max_acc_xy=VM_MAX_ACC_XY,          # Max horizontal acceleration in m/s²
    max_acc_z=VM_MAX_ACC_Z,            # Max vertical acceleration in m/s²
    tau_xy=VM_TAU_XY,                  # Optional: 1st-order horizontal tracking time constant (s)
    tau_z=VM_TAU_Z,                    # Optional: 1st-order vertical tracking time constant (s)
    send_telemetry=VM_SEND_TELEMETRY,  # Enable telemetry
    telemetry_decimation=VM_TELEMETRY_DECIMATION,  # Send telemetry every update
)


def main():
    """Execute the simulation."""

    # Create a shared CSV file for agent telemetry logs.
    # Each agent will append its rows on protocol finish().
    csv_path = os.path.join(os.getcwd(), "agent_telemetry.csv")
    os.environ["AGENT_LOG_CSV_PATH"] = csv_path
    with open(csv_path, "w", encoding="utf-8") as f:
        f.write("node_id,timestamp,u,velocity_norm\n")

    # Create a shared CSV file for target telemetry logs.
    # The target will append its rows on protocol finish().
    target_csv_path = os.path.join(os.getcwd(), "target_telemetry.csv")
    os.environ["TARGET_LOG_CSV_PATH"] = target_csv_path
    with open(target_csv_path, "w", encoding="utf-8") as f:
        f.write("timestamp,E_r,E_vr,rho,G_max,E_gap\n")

    duration = SIM_DURATION        # Simulation duration (seconds)
    real_time = SIM_REAL_TIME      # Run in real time (True) or as-fast-as-possible (False)
    debug = SIM_DEBUG              # Enable simulator debug mode

    builder = SimulationBuilder(
        SimulationConfiguration(
            duration=duration,
            debug=debug,
            real_time=real_time,
        )
    )

    transmission_range = COMMUNICATION_TRANSMISSION_RANGE  # Communication range (meters)
    delay = COMMUNICATION_DELAY                       # Communication delay (seconds)
    failure_rate = COMMUNICATION_FAILURE_RATE         # Packet loss probability [0.0, 1.0]
    medium = CommunicationMedium(
        transmission_range=transmission_range,
        delay=delay,
        failure_rate=failure_rate,
    )
    builder.add_handler(CommunicationHandler(medium))

    builder.add_handler(TimerHandler())

    velocity_handler = VelocityMobilityHandler(mobility_config)
    builder.add_handler(velocity_handler)

    vis_config = VisualizationConfiguration(
        open_browser=VIS_OPEN_BROWSER,
        update_rate=VIS_UPDATE_RATE,
    )
    builder.add_handler(VisualizationHandler(vis_config))

    # Add target node at origin
    builder.add_node(TargetProtocol, (0, 0, 0))

    # Add adversary node at a fixed initial position
    builder.add_node(AdversaryProtocol, (40, 40, 0))

    # Add agent nodes at random positions around the target
    # and randomly vary the desired encirclement radius

    num_agents = NUM_AGENTS # Number of agent nodes
    encirclement_radius = ENCIRCLEMENT_RADIUS # Desired encirclement radius in meters
    for i in range(num_agents):
        angle = random.uniform(0, 2 * math.pi)
        x = encirclement_radius * random.uniform(0.8, 1.2) * math.cos(angle)
        y = encirclement_radius * random.uniform(0.8, 1.2) * math.sin(angle)
        z = 0.0 # Keep agents at ground level
        builder.add_node(AgentProtocol, (x, y, z))

    simulation = builder.build()
    simulation.start_simulation()

if __name__ == "__main__":
    main()
