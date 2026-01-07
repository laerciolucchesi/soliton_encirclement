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
        target_vel = self.velocity_handler.get_node_velocity(self.node_id)
        if target_pos is None:
            return
        if target_vel is None:
            target_vel = (0.0, 0.0, 0.0)

        # Compute 5 encirclement metrics over currently alive agents.
        # Notation uses the XY plane only.
        #   E_r   : RMS normalized radial orbit error (dimensionless)
        #   E_vr  : RMS radial speed (m/s)
        #   rho   : Kuramoto order parameter in [0, 1]
        #   G_max : max_k (Delta theta_k / (2*pi/M)) using M = alive agents (dimensionless)
        #   E_gap : RMS normalized angular spacing error (dimensionless)

        two_pi = 2.0 * math.pi
        R = float(ENCIRCLEMENT_RADIUS)

        # Accumulators
        sum_sq_Er = 0.0
        count_Er = 0

        sum_sq_vr = 0.0
        count_vr = 0

        angles = []
        sum_cos = 0.0
        sum_sin = 0.0
        count_theta = 0

        for _agent_id, (state, _rxtime) in self.agent_states.items():
            dx = float(state.position[0] - target_pos[0])
            dy = float(state.position[1] - target_pos[1])
            r = math.hypot(dx, dy)
            if not (math.isfinite(dx) and math.isfinite(dy) and math.isfinite(r) and r > 1e-9):
                continue

            # theta in [0, 2*pi)
            theta = math.atan2(dy, dx)
            if not math.isfinite(theta):
                continue
            theta = (theta + two_pi) % two_pi
            angles.append(theta)
            sum_cos += math.cos(theta)
            sum_sin += math.sin(theta)
            count_theta += 1

            # E_r: normalized radial orbit error
            if math.isfinite(R) and R > 0.0:
                e_r = (r / R) - 1.0
                if math.isfinite(e_r):
                    sum_sq_Er += e_r * e_r
                    count_Er += 1

            # E_vr: RMS radial speed (relative to the target)
            vx = float(state.velocity[0] - target_vel[0])
            vy = float(state.velocity[1] - target_vel[1])
            if math.isfinite(vx) and math.isfinite(vy):
                e_rx = dx / r
                e_ry = dy / r
                v_r = vx * e_rx + vy * e_ry
                if math.isfinite(v_r):
                    sum_sq_vr += v_r * v_r
                    count_vr += 1

        # Metric 1: E_r
        E_r = float(math.sqrt(sum_sq_Er / count_Er)) if count_Er > 0 else 0.0

        # Metric 2: E_vr
        E_vr = float(math.sqrt(sum_sq_vr / count_vr)) if count_vr > 0 else 0.0

        # Metric 3: rho (Kuramoto order parameter)
        if count_theta > 0:
            z_re = sum_cos / float(count_theta)
            z_im = sum_sin / float(count_theta)
            rho = float(math.hypot(z_re, z_im))
        else:
            rho = 0.0

        # Metrics 4 and 5: G_max and E_gap from sorted angular gaps
        angles.sort()
        M = len(angles)
        G_max = 0.0
        E_gap = 0.0
        if M > 0:
            ideal_gap = two_pi / float(M)
            if math.isfinite(ideal_gap) and ideal_gap > 0.0:
                max_ratio = 0.0
                sum_sq_gap = 0.0
                count_gap = 0
                for i in range(M):
                    if i < M - 1:
                        gap = angles[i + 1] - angles[i]
                    else:
                        gap = angles[0] + two_pi - angles[-1]

                    if not math.isfinite(gap):
                        continue

                    ratio = gap / ideal_gap
                    if math.isfinite(ratio):
                        if ratio > max_ratio:
                            max_ratio = ratio
                        e_gap = ratio - 1.0
                        if math.isfinite(e_gap):
                            sum_sq_gap += e_gap * e_gap
                            count_gap += 1

                G_max = float(max_ratio) if count_gap > 0 else 0.0
                E_gap = float(math.sqrt(sum_sq_gap / count_gap)) if count_gap > 0 else 0.0

        self._telemetry_rows.append(
            {
                "timestamp": now,
                "E_r": E_r,
                "E_vr": E_vr,
                "rho": rho,
                "G_max": G_max,
                "E_gap": E_gap,
            }
        )

    def finish(self):
        if not self._csv_path or not self._telemetry_rows:
            return

        try:
            df = pd.DataFrame(
                self._telemetry_rows,
                columns=["timestamp", "E_r", "E_vr", "rho", "G_max", "E_gap"],
            )

            # Append without header; main.py already created the file with header.
            # Still guard for the case where the file was removed mid-run.
            file_exists = os.path.exists(self._csv_path)
            df.to_csv(self._csv_path, mode="a", header=not file_exists, index=False)

            # Also write a PNG plot next to the CSV.
            if plt is not None:
                plot_df = df.copy()
                plot_df["timestamp"] = pd.to_numeric(plot_df["timestamp"], errors="coerce")
                plot_df["E_r"] = pd.to_numeric(plot_df["E_r"], errors="coerce")
                plot_df["E_vr"] = pd.to_numeric(plot_df["E_vr"], errors="coerce")
                plot_df["rho"] = pd.to_numeric(plot_df["rho"], errors="coerce")
                plot_df["G_max"] = pd.to_numeric(plot_df["G_max"], errors="coerce")
                plot_df["E_gap"] = pd.to_numeric(plot_df["E_gap"], errors="coerce")
                plot_df = plot_df.dropna(subset=["timestamp"]).sort_values("timestamp")

                out_dir = os.path.dirname(os.path.abspath(self._csv_path))

                def _plot_metric(
                    metric: str,
                    *,
                    title: str,
                    ylabel: str,
                    filename: str,
                    definition: str,
                ) -> None:
                    metric_df = plot_df.dropna(subset=[metric])
                    if metric_df.empty:
                        return
                    fig, ax = plt.subplots(1, 1, figsize=(10, 4))
                    ax.plot(metric_df["timestamp"], metric_df[metric], linewidth=1.2)
                    ax.set_title(title)
                    ax.set_xlabel("timestamp (s)")
                    ax.set_ylabel(ylabel)
                    if definition:
                        ax.text(
                            0.98,
                            0.98,
                            definition,
                            transform=ax.transAxes,
                            va="top",
                            ha="right",
                            fontsize=11,
                            bbox={
                                "boxstyle": "round,pad=0.25",
                                "facecolor": "white",
                                "edgecolor": "black",
                                "alpha": 0.85,
                            },
                        )
                    ax.grid(True, alpha=0.3)
                    fig.tight_layout()
                    fig.savefig(os.path.join(out_dir, filename), dpi=150)
                    plt.close(fig)

                _plot_metric(
                    "E_r",
                    title="Normalized radial orbit error (RMS)",
                    ylabel="E_r",
                    filename="metric_E_r.png",
                    definition=r"$E_r(t)=\sqrt{\frac{1}{M}\sum_j\left(\frac{r_j(t)}{R}-1\right)^2}$",
                )
                _plot_metric(
                    "E_vr",
                    title="Radial speed (RMS)",
                    ylabel="E_vr (m/s)",
                    filename="metric_E_vr.png",
                    definition=r"$E_{vr}(t)=\sqrt{\frac{1}{M}\sum_j v_{r,j}(t)^2}$",
                )
                _plot_metric(
                    "rho",
                    title="Kuramoto order parameter",
                    ylabel="rho",
                    filename="metric_rho.png",
                    definition=r"$\rho(t)=\left|\frac{1}{M}\sum_j e^{i\theta_j(t)}\right|$",
                )
                _plot_metric(
                    "G_max",
                    title="Normalized maximum angular gap",
                    ylabel="G_max",
                    filename="metric_G_max.png",
                    definition=r"$G_{\max}(t)=\max_k\frac{\Delta\theta_k(t)}{2\pi/M}$",
                )
                _plot_metric(
                    "E_gap",
                    title="RMS normalized angular spacing error",
                    ylabel="E_gap",
                    filename="metric_E_gap.png",
                    definition=r"$E_{gap}(t)=\sqrt{\frac{1}{M}\sum_k\left(\frac{\Delta\theta_k(t)}{2\pi/M}-1\right)^2}$",
                )
        except Exception as exc:
            self._logger.warning(
                "Target %s: failed to write telemetry CSV (%s): %r",
                getattr(self, "node_id", "?"),
                exc,
                self._csv_path,
            )

