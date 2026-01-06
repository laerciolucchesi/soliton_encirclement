"""
Protocol for the target node.
"""
import logging
import math
import os
import random
from typing import Dict, Optional, Tuple

import pandas as pd

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except Exception:  # pragma: no cover
    plt = None

from gradysim.protocol.interface import IProtocol
from gradysim.protocol.messages.telemetry import Telemetry
from gradysim.protocol.messages.communication import CommunicationCommand, CommunicationCommandType

import json

from config_param import (
    AGENT_STATE_TIMEOUT,
    ENCIRCLEMENT_RADIUS,
    PRUNE_EXPIRED_STATES,
    SIM_DEBUG,
    TARGET_MOTION_BOUNDARY_XY,
    TARGET_MOTION_PERIOD,
    TARGET_MOTION_SPEED_XY,
    TARGET_MOTION_TIMER_STR,
    TARGET_STATE_BROADCAST_PERIOD,
    TARGET_STATE_BROADCAST_TIMER_STR,
)
from protocol_messages import AgentState, TargetState


class TargetProtocol(IProtocol):
    """Implementation of target protocol."""

    def __init__(self):
        super().__init__()
        self._logger = logging.getLogger()

    def initialize(self):
        self.node_id = self.provider.get_id() # Get the node ID from the provider
        self.target_state_broadcast_period = TARGET_STATE_BROADCAST_PERIOD  # Broadcast period in seconds
        # Schedule the broadcast timer for the first time
        self.schedule_broadcast_timer()

        # Access the VelocityMobilityHandler if available
        handlers = getattr(self.provider, "handlers", {}) or {}
        self.velocity_handler = handlers.get("VelocityMobilityHandler")

        # Schedule target motion timer (if mobility handler exists)
        if self.velocity_handler is not None:
            self.schedule_motion_timer()

        # Initialize sequence number
        self.target_state_seq = 1

        # Telemetry logging (in-memory). The main.py creates the CSV and sets the path
        # via environment variable to avoid tight coupling with the simulator builder.
        self._csv_path: Optional[str] = os.environ.get("TARGET_LOG_CSV_PATH")
        self._telemetry_rows = []

        # Latest received AgentState messages (filled by handle_packet)
        # Stored as (state, rxtime) to support timeout detection.
        self.agent_states: Dict[int, Tuple[AgentState, float]] = {}
        self.last_seq_agent: Dict[int, int] = {}
    
    def schedule_broadcast_timer(self):
        self.provider.schedule_timer(TARGET_STATE_BROADCAST_TIMER_STR, self.provider.current_time() + self.target_state_broadcast_period)

    def schedule_motion_timer(self):
        self.provider.schedule_timer(TARGET_MOTION_TIMER_STR, self.provider.current_time() + float(TARGET_MOTION_PERIOD))

    def handle_timer(self, timer: str):
        self._logger.debug("Target %s: handle_timer called with timer=%s", self.node_id, timer)
        if timer == TARGET_STATE_BROADCAST_TIMER_STR:
            # Get current position and velocity from the mobility handler
            position = self.velocity_handler.get_node_position(self.node_id) 
            velocity = self.velocity_handler.get_node_velocity(self.node_id)
            seq = self.target_state_seq # Get current sequence number
            # Create TargetState message
            target_state = TargetState(
                target_id=self.node_id,
                seq=seq,
                position=position,
                velocity=velocity
            )
            message_json = target_state.to_json() # Convert to JSON
            command = CommunicationCommand(CommunicationCommandType.BROADCAST,message_json)
            self.provider.send_communication_command(command)  # send the message to all agent nodes
            if SIM_DEBUG:
                print(f"Target {self.node_id} broadcasted {TargetState.TYPE} seq={seq}, position={position}, velocity={velocity}")
            # Increment sequence number
            self.target_state_seq = seq + 1
            # Reschedule the broadcast timer
            self.schedule_broadcast_timer()

        elif timer == TARGET_MOTION_TIMER_STR:
            # Move the target at a fixed speed in XY, changing direction every period.
            if self.velocity_handler is None:
                # Mobility handler not available; try again later.
                self.schedule_motion_timer()
                return

            position = self.velocity_handler.get_node_position(self.node_id)
            if position is None:
                self.schedule_motion_timer()
                return

            x = float(position[0])
            y = float(position[1])

            speed_xy = float(TARGET_MOTION_SPEED_XY)
            if not (math.isfinite(speed_xy) and speed_xy > 0.0):
                self.schedule_motion_timer()
                return

            boundary = float(TARGET_MOTION_BOUNDARY_XY)
            outside = (abs(x) > boundary) or (abs(y) > boundary)

            if outside and math.isfinite(x) and math.isfinite(y):
                # Point velocity toward the center (0,0) when outside bounds.
                dx = -x
                dy = -y
                norm = math.hypot(dx, dy)
                if math.isfinite(norm) and norm > 1e-12:
                    vx = speed_xy * (dx / norm)
                    vy = speed_xy * (dy / norm)
                else:
                    two_pi = 2.0 * math.pi
                    angle = random.random() * two_pi
                    vx = speed_xy * math.cos(angle)
                    vy = speed_xy * math.sin(angle)
            else:
                two_pi = 2.0 * math.pi
                angle = random.random() * two_pi
                vx = speed_xy * math.cos(angle)
                vy = speed_xy * math.sin(angle)
            vz = 0.0
            self.velocity_handler.set_velocity(self.node_id, (vx, vy, vz))

            if SIM_DEBUG:
                print(f"Target {self.node_id} updated motion: v_des=({vx:.3f}, {vy:.3f}, {vz:.3f})")

            self.schedule_motion_timer()


    def handle_packet(self, message: str):
        try:
            data = json.loads(message)
        except Exception as exc:
            self._logger.warning("Target %s: failed to parse packet as JSON (%s): %r", self.node_id, exc, message)
            return

        msg_type = data.get("type")

        if msg_type == AgentState.TYPE:
            try:
                state = AgentState.from_json(message)
            except Exception as exc:
                self._logger.warning("Target %s: failed to decode AgentState (%s): %r", self.node_id, exc, message)
                return

            # Discard out-of-order agent messages.
            now = self.provider.current_time()
            last_seq = self.last_seq_agent.get(state.agent_id, -1)

            # Recovery rule: if this agent was considered expired (no recent AgentState),
            # accept a sequence reset after a restart.
            agent_expired = True
            prev_entry = self.agent_states.get(state.agent_id)
            if prev_entry is not None:
                _, last_rxtime = prev_entry
                agent_expired = (now - last_rxtime) > AGENT_STATE_TIMEOUT

            if state.seq <= last_seq and not agent_expired:
                return

            self.last_seq_agent[state.agent_id] = state.seq

            rxtime = now
            if state.agent_id != self.node_id:
                self.agent_states[state.agent_id] = (state, rxtime)

            if SIM_DEBUG:
                print(
                    f"Target {self.node_id} received AgentState "
                    f"rxtime={rxtime:.3f}, seq={state.seq}, agent_id={state.agent_id}, position={state.position}, "
                    f"velocity={state.velocity}, u={state.u}"
                )
            return

        # Ignore TargetState or unknown message types at the target.
        self._logger.debug("Target %s: ignoring packet type=%r", self.node_id, msg_type)

    def _prune_expired_states(self, now: float) -> None:
        if not PRUNE_EXPIRED_STATES:
            return

        expired_agent_ids = [
            agent_id
            for agent_id, (_, rxtime) in self.agent_states.items()
            if (now - rxtime) > AGENT_STATE_TIMEOUT
        ]
        for agent_id in expired_agent_ids:
            self.agent_states.pop(agent_id, None)
            self.last_seq_agent.pop(agent_id, None)

    def handle_telemetry(self, telemetry: Telemetry) -> None:
        if not self._csv_path:
            return

        now = float(self.provider.current_time())
        self._prune_expired_states(now)

        if not self.velocity_handler:
            return

        target_pos = self.velocity_handler.get_node_position(self.node_id)

        # Compute global radial error (RMS) over all alive agent nodes.
        # For each alive node j:
        #   e_j = ||p_j - p_target||/R - 1
        # Then compute RMS:
        #   E_rms = sqrt((1/M) * sum_j e_j^2)
        # where M is the number of valid (finite) terms accumulated.
        sum_sq = 0.0
        count = 0
        for _agent_id, (state, _rxtime) in self.agent_states.items():
            dx = float(state.position[0] - target_pos[0])
            dy = float(state.position[1] - target_pos[1])
            r = math.hypot(dx, dy)

            if not (math.isfinite(r) and math.isfinite(ENCIRCLEMENT_RADIUS) and ENCIRCLEMENT_RADIUS > 0.0):
                continue

            e = (r / ENCIRCLEMENT_RADIUS) - 1.0
            if not math.isfinite(e):
                continue
            sum_sq += e * e

            count += 1

        global_radial_error = float(math.sqrt(sum_sq / count)) if count > 0 else 0.0

        # Compute global tangential error (RMS) using the angular gaps between neighbors.
        # Steps:
        #   1) ideal_angle = 2*pi/N
        #   2) for each gap (neighbors in sorted angular order): e = gap/ideal_angle - 1
        #   3) accumulate sum of squares and number of valid terms
        #   4) global_tangential_error = sqrt(sum_sq / count)
        angles = []
        two_pi = 2.0 * math.pi
        for _agent_id, (state, _rxtime) in self.agent_states.items():
            dx = float(state.position[0] - target_pos[0])
            dy = float(state.position[1] - target_pos[1])
            theta = math.atan2(dy, dx)
            if not math.isfinite(theta):
                continue
            theta = (theta + two_pi) % two_pi
            angles.append(theta)

        angles.sort()
        n = len(angles)
        sum_sq_tan = 0.0
        count_tan = 0
        if n > 0:
            ideal_angle = two_pi / float(n)
            if math.isfinite(ideal_angle) and ideal_angle > 0.0:
                for i in range(n):
                    if i < n - 1:
                        gap = angles[i + 1] - angles[i]
                    else:
                        gap = angles[0] + two_pi - angles[-1]

                    if not math.isfinite(gap):
                        continue
                    e_tan = (gap / ideal_angle) - 1.0
                    if not math.isfinite(e_tan):
                        continue
                    sum_sq_tan += e_tan * e_tan

                    count_tan += 1

        global_tangential_error = float(math.sqrt(sum_sq_tan / count_tan)) if count_tan > 0 else 0.0

        self._telemetry_rows.append(
            {
                "timestamp": now,
                "global_radial_error": global_radial_error,
                "global_tangential_error": global_tangential_error,
            }
        )

    def finish(self):
        if not self._csv_path or not self._telemetry_rows:
            return

        try:
            df = pd.DataFrame(
                self._telemetry_rows,
                columns=["timestamp", "global_radial_error", "global_tangential_error"],
            )

            # Append without header; main.py already created the file with header.
            # Still guard for the case where the file was removed mid-run.
            file_exists = os.path.exists(self._csv_path)
            df.to_csv(self._csv_path, mode="a", header=not file_exists, index=False)

            # Also write a PNG plot next to the CSV.
            if plt is not None:
                plot_df = df.copy()
                plot_df["timestamp"] = pd.to_numeric(plot_df["timestamp"], errors="coerce")
                plot_df["global_radial_error"] = pd.to_numeric(plot_df["global_radial_error"], errors="coerce")
                plot_df["global_tangential_error"] = pd.to_numeric(plot_df["global_tangential_error"], errors="coerce")
                plot_df = plot_df.dropna(subset=["timestamp"]).sort_values("timestamp")

                out_dir = os.path.dirname(os.path.abspath(self._csv_path))

                # Preserve radial error plot.
                radial_df = plot_df.dropna(subset=["global_radial_error"])
                if not radial_df.empty:
                    fig, ax = plt.subplots(1, 1, figsize=(10, 4))
                    ax.plot(radial_df["timestamp"], radial_df["global_radial_error"], linewidth=1.2)
                    ax.set_xlabel("timestamp (s)")
                    ax.set_ylabel("global_radial_error")
                    ax.grid(True, alpha=0.3)
                    fig.tight_layout()

                    radial_png_path = os.path.join(out_dir, "target_telemetry_radial_error.png")
                    fig.savefig(radial_png_path, dpi=150)
                    plt.close(fig)

                # Tangential error plot.
                tangential_df = plot_df.dropna(subset=["global_tangential_error"])
                if not tangential_df.empty:
                    fig, ax = plt.subplots(1, 1, figsize=(10, 4))
                    ax.plot(tangential_df["timestamp"], tangential_df["global_tangential_error"], linewidth=1.2)
                    ax.set_xlabel("timestamp (s)")
                    ax.set_ylabel("global_tangential_error")
                    ax.grid(True, alpha=0.3)
                    fig.tight_layout()

                    tangential_png_path = os.path.join(out_dir, "target_telemetry_tangential_error.png")
                    fig.savefig(tangential_png_path, dpi=150)
                    plt.close(fig)
        except Exception as exc:
            self._logger.warning(
                "Target %s: failed to write telemetry CSV (%s): %r",
                getattr(self, "node_id", "?"),
                exc,
                self._csv_path,
            )

