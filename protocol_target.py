"""
Protocol for the target node.
"""
import logging
from bisect import bisect_right
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
    PROTECTION_ANGLE_DEG,
    PRUNE_EXPIRED_STATES,
    SIM_DEBUG,
    TARGET_SWARM_SPIN_ENABLE,
    TARGET_SWARM_OMEGA_REF,
    TARGET_SWARM_OMEGA_PD_KP,
    TARGET_SWARM_OMEGA_PD_KD,
    TARGET_SWARM_OMEGA_PD_MAX_ABS,
    TARGET_SWARM_SPIN_RHO_MIN,
    TARGET_MOTION_BOUNDARY_XY,
    TARGET_MOTION_PERIOD,
    TARGET_MOTION_SPEED_XY,
    TARGET_MOTION_TIMER_STR,
    TARGET_STATE_BROADCAST_PERIOD,
    TARGET_STATE_BROADCAST_TIMER_STR,
)
from protocol_messages import AdversaryState, AgentState, TargetState


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
        # Map of alive agents -> lp (lambda) weight used for non-uniform spacing.
        self.alive_lambdas: Dict[int, float] = {}

        # Latest received AdversaryState (best-effort), used for omega_ref control.
        self.adversary_state: Optional[Tuple[AdversaryState, float]] = None
        self.last_seq_adversary: int = -1

        # PD state for omega_ref control
        self._omega_err_prev: Optional[float] = None
        self._omega_err_prev_time: Optional[float] = None


        # Exactly one alive agent should hold the edge lambda value.
        # The holder is determined geometrically from the current formation (largest angular gap).
        # The value itself is dynamic and depends on alive_count and PROTECTION_ANGLE_DEG.
        self._special_lambda_value: float = 27.0
        self._special_agent_id: Optional[int] = None

        # Anti-chattering controls for token transfers.
        # - cooldown: minimum time between transfers
        # - hysteresis: require the new max gap to exceed the current holder's arc gap by this margin
        self._special_last_switch_time: float = -1e9
        self._special_cooldown_s: float = 1.0
        self._special_hysteresis_rad: float = 0.05

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        """Wrap an angle to (-pi, pi]."""
        two_pi = 2.0 * math.pi
        a = (float(angle) + math.pi) % two_pi
        a -= math.pi
        return float(a)

    @staticmethod
    def _unit2(vec, eps: float = 1e-6) -> tuple[tuple[float, float], float]:
        x = float(vec[0])
        y = float(vec[1])
        n = math.hypot(x, y)
        if not (math.isfinite(n) and n > eps):
            return (1.0, 0.0), 0.0
        return (x / n, y / n), float(n)

    @staticmethod
    def _signed_angle(u_from: tuple[float, float], u_to: tuple[float, float]) -> float:
        """Signed angle (rad) from u_from to u_to in the XY plane."""
        ax, ay = float(u_from[0]), float(u_from[1])
        bx, by = float(u_to[0]), float(u_to[1])
        dot = ax * bx + ay * by
        cross = ax * by - ay * bx
        return float(math.atan2(cross, dot))

    def _compute_sorted_angles(self, *, target_pos: Tuple[float, float, float]) -> list[tuple[float, int]]:
        two_pi = 2.0 * math.pi
        angles_and_ids: list[tuple[float, int]] = []
        for aid, (state, _rxtime) in self.agent_states.items():
            dx = float(state.position[0] - target_pos[0])
            dy = float(state.position[1] - target_pos[1])
            theta = math.atan2(dy, dx)
            if not math.isfinite(theta):
                continue
            theta = (theta + two_pi) % two_pi
            angles_and_ids.append((theta, int(aid)))
        angles_and_ids.sort(key=lambda t: t[0])
        return angles_and_ids

    def _gap_of_arc_start(self, angles_and_ids: list[tuple[float, int]], arc_start_id: int) -> Optional[float]:
        if not angles_and_ids:
            return None
        two_pi = 2.0 * math.pi
        ids = [aid for (_t, aid) in angles_and_ids]
        if arc_start_id not in ids:
            return None
        i = ids.index(int(arc_start_id))
        theta_i = float(angles_and_ids[i][0])
        theta_next = float(angles_and_ids[(i + 1) % len(angles_and_ids)][0])
        gap = (theta_next - theta_i) % two_pi
        return float(gap) if math.isfinite(gap) else None

    def _max_gap_predecessor(self, angles_and_ids: list[tuple[float, int]]) -> Optional[tuple[int, float]]:
        """Return (arc_start_id, max_gap) where max_gap is the largest angular gap.

        With the arc-based convention, the node that should receive lambda=27 is the
        predecessor of the largest gap, i.e., the start of that arc (node -> successor).
        """
        if not angles_and_ids:
            return None
        two_pi = 2.0 * math.pi
        best_gap = -1.0
        best_id: Optional[int] = None
        n = len(angles_and_ids)
        for i in range(n):
            theta_i, aid_i = angles_and_ids[i]
            theta_next, _aid_next = angles_and_ids[(i + 1) % n]
            gap = (float(theta_next) - float(theta_i)) % two_pi
            if not math.isfinite(gap):
                continue
            if gap > best_gap:
                best_gap = gap
                best_id = int(aid_i)
        if best_id is None or not math.isfinite(best_gap):
            return None
        return int(best_id), float(best_gap)

    def _min_gap_predecessor(self, angles_and_ids: list[tuple[float, int]]) -> Optional[tuple[int, float]]:
        """Return (arc_start_id, min_gap) where min_gap is the smallest angular gap."""
        if not angles_and_ids:
            return None
        two_pi = 2.0 * math.pi
        best_gap = float("inf")
        best_id: Optional[int] = None
        n = len(angles_and_ids)
        for i in range(n):
            theta_i, aid_i = angles_and_ids[i]
            theta_next, _aid_next = angles_and_ids[(i + 1) % n]
            gap = (float(theta_next) - float(theta_i)) % two_pi
            if not math.isfinite(gap):
                continue
            if gap < best_gap:
                best_gap = gap
                best_id = int(aid_i)
        if best_id is None or not math.isfinite(best_gap) or best_gap == float("inf"):
            return None
        return int(best_id), float(best_gap)

    def _update_special_lambda_by_geometry(self, *, now: float, target_pos: Tuple[float, float, float]) -> None:
        """Assign the edge lambda to the predecessor of the extreme gap among alive agents.

        If edge_lambda > 1, the boundary arc is the largest gap.
        If edge_lambda < 1, the boundary arc is the smallest gap.
        """
        alive_ids = list(self.agent_states.keys())
        if not alive_ids:
            self._special_agent_id = None
            return

        # Ensure alive agents have explicit lambdas.
        for aid in alive_ids:
            if aid not in self.alive_lambdas:
                self.alive_lambdas[aid] = 1.0

        angles_and_ids = self._compute_sorted_angles(target_pos=target_pos)
        if not angles_and_ids:
            # Fallback: pick deterministic holder.
            self._assign_special_lambda(int(sorted(alive_ids)[0]))
            self._special_last_switch_time = float(now)
            return

        track_max = float(self._special_lambda_value) >= (1.0 + 1e-6)
        candidate = self._max_gap_predecessor(angles_and_ids) if track_max else self._min_gap_predecessor(angles_and_ids)
        if candidate is None:
            return
        candidate_id, max_gap = candidate

        current_id = self._special_agent_id
        if current_id is not None and current_id not in self.agent_states:
            current_id = None

        if current_id is None:
            self._assign_special_lambda(candidate_id)
            self._special_last_switch_time = float(now)
            return

        # Keep the invariant even if maps drift.
        if candidate_id == int(current_id):
            self._assign_special_lambda(int(current_id))
            return

        # Cooldown before switching.
        if (float(now) - float(self._special_last_switch_time)) < float(self._special_cooldown_s):
            return

        current_gap = self._gap_of_arc_start(angles_and_ids, int(current_id))
        if current_gap is None:
            # If we can't compute current gap, allow switch.
            self._assign_special_lambda(candidate_id)
            self._special_last_switch_time = float(now)
            return

        if track_max:
            # Hysteresis: require candidate max gap to be sufficiently larger than current holder's gap.
            if float(max_gap) <= float(current_gap) + float(self._special_hysteresis_rad):
                return
        else:
            # Hysteresis (min-gap tracking): require candidate min gap to be sufficiently smaller.
            if float(max_gap) >= float(current_gap) - float(self._special_hysteresis_rad):
                return

        self._assign_special_lambda(candidate_id)
        self._special_last_switch_time = float(now)

    def _assign_special_lambda(self, agent_id: int) -> None:
        agent_id_int = int(agent_id)

        # Ensure all currently known alive agents have a defined lambda.
        for aid in list(self.agent_states.keys()):
            if aid not in self.alive_lambdas:
                self.alive_lambdas[aid] = 1.0

        # Enforce uniqueness of the special lambda.
        for aid in list(self.alive_lambdas.keys()):
            self.alive_lambdas[aid] = 1.0

        self.alive_lambdas[agent_id_int] = float(self._special_lambda_value)
        self._special_agent_id = agent_id_int

    def _pick_predecessor_by_angle(self, *, angle_ref: float, target_pos: Tuple[float, float, float], alive_ids: list[int]) -> int:
        """Pick the predecessor (in sorted target-centric angle order) of a reference angle.

        This implements the policy for transferring the special lambda when its owner fails.

        Rationale (arc-based lambda interpretation):
        - A node's lambda applies to the arc (node -> successor).
        - If the node holding lambda=27 fails, assigning 27 to its predecessor makes the new
          enlarged arc span across the failed node's location, preserving the formation shape.
        """
        two_pi = 2.0 * math.pi
        angles_and_ids: list[tuple[float, int]] = []
        for aid in alive_ids:
            entry = self.agent_states.get(aid)
            if entry is None:
                continue
            state, _ = entry
            dx = float(state.position[0] - target_pos[0])
            dy = float(state.position[1] - target_pos[1])
            theta = math.atan2(dy, dx)
            if not math.isfinite(theta):
                continue
            theta = (theta + two_pi) % two_pi
            angles_and_ids.append((theta, aid))

        if not angles_and_ids:
            return int(alive_ids[0])

        angles_and_ids.sort(key=lambda t: t[0])
        angles = [t[0] for t in angles_and_ids]
        ids = [t[1] for t in angles_and_ids]

        # Find where angle_ref would be inserted to keep ordering; predecessor is just before.
        idx = bisect_right(angles, float(angle_ref))
        pred_idx = (idx - 1) % len(ids)
        return int(ids[pred_idx])
    
    def schedule_broadcast_timer(self):
        self.provider.schedule_timer(TARGET_STATE_BROADCAST_TIMER_STR, self.provider.current_time() + self.target_state_broadcast_period)

    def schedule_motion_timer(self):
        self.provider.schedule_timer(TARGET_MOTION_TIMER_STR, self.provider.current_time() + float(TARGET_MOTION_PERIOD))

    def handle_timer(self, timer: str):
        self._logger.debug("Target %s: handle_timer called with timer=%s", self.node_id, timer)
        if timer == TARGET_STATE_BROADCAST_TIMER_STR:
            # Keep internal tracking consistent before broadcasting.
            now = float(self.provider.current_time())
            self._prune_expired_states(now)

            # Get current position and velocity from the mobility handler
            position = self.velocity_handler.get_node_position(self.node_id) 
            velocity = self.velocity_handler.get_node_velocity(self.node_id)
            seq = self.target_state_seq # Get current sequence number

            if position is None:
                position = (0.0, 0.0, 0.0)
            if velocity is None:
                velocity = (0.0, 0.0, 0.0)

            # Dynamic edge lambda value based on the current number of alive agents.
            alive_count = int(len(self.agent_states))

            # --- omega_ref control (runs at broadcast period, typically CONTROL_PERIOD) ---
            omega_base = float(TARGET_SWARM_OMEGA_REF)

            if not bool(TARGET_SWARM_SPIN_ENABLE):
                # Spin tracking disabled: do not compute angular error to the adversary.
                # Keep omega_ref at the configured base value (usually 0.0).
                omega_ref = float(omega_base)
                # Reset PD memory so that re-enabling doesn't create derivative spikes.
                self._omega_err_prev = None
                self._omega_err_prev_time = None
            else:
                # Vector 1 (unit): target -> adversary.
                adv_unit = (1.0, 0.0)
                if self.adversary_state is not None:
                    adv_state, _adv_rx = self.adversary_state
                    dx = float(adv_state.position[0] - position[0])
                    dy = float(adv_state.position[1] - position[1])
                    adv_unit, _ = self._unit2((dx, dy))

                # Vector 2 (unit): normalized sum of unit vectors target -> alive agents.
                sx = 0.0
                sy = 0.0
                for _aid, (astate, _rxt) in self.agent_states.items():
                    dx = float(astate.position[0] - position[0])
                    dy = float(astate.position[1] - position[1])
                    u, _n = self._unit2((dx, dy))
                    sx += float(u[0])
                    sy += float(u[1])
                swarm_unit, swarm_sum_norm = self._unit2((sx, sy))

                # If the swarm is close to uniform, the resultant direction is ill-defined.
                # Use rho = ||sum||/N as a reliability measure and disable the error when small.
                n_agents = max(0, int(len(self.agent_states)))
                rho = 0.0
                if n_agents > 0 and math.isfinite(swarm_sum_norm):
                    rho = float(swarm_sum_norm) / float(n_agents)

                # Angular error: signed angle from swarm vector to adversary vector.
                if math.isfinite(rho) and rho >= float(TARGET_SWARM_SPIN_RHO_MIN):
                    err = self._signed_angle(swarm_unit, adv_unit)
                else:
                    err = 0.0

                # PD on err -> omega_ref.
                kp = float(TARGET_SWARM_OMEGA_PD_KP)
                kd = float(TARGET_SWARM_OMEGA_PD_KD)
                max_abs = float(TARGET_SWARM_OMEGA_PD_MAX_ABS)
                if not math.isfinite(kp):
                    kp = 0.0
                if not math.isfinite(kd):
                    kd = 0.0
                if not (math.isfinite(max_abs) and max_abs > 0.0):
                    max_abs = float("inf")

                derr = 0.0
                if err == 0.0:
                    # Avoid derivative spikes when the direction is ill-defined.
                    self._omega_err_prev = float(err)
                    self._omega_err_prev_time = float(now)
                elif self._omega_err_prev is not None and self._omega_err_prev_time is not None:
                    dt = float(now - self._omega_err_prev_time)
                    if math.isfinite(dt) and dt > 1e-6:
                        derr = self._wrap_to_pi(err - float(self._omega_err_prev)) / dt

                # If the PD gains are disabled, keep a pure open-loop spin.
                # Otherwise, generate omega_ref purely from the angular error (no constant bias).
                if kp == 0.0 and kd == 0.0:
                    omega_ref = float(omega_base)
                else:
                    omega_ref = (kp * err) + (kd * derr)
                if math.isfinite(omega_ref):
                    omega_ref = max(-max_abs, min(max_abs, omega_ref))
                else:
                    omega_ref = float(omega_base)

                self._omega_err_prev = float(err)
                self._omega_err_prev_time = float(now)

                # end TARGET_SWARM_SPIN_ENABLE

            # Interpret PROTECTION_ANGLE_DEG as the protected/covered arc; the edge gap is its complement.
            prot_deg = float(PROTECTION_ANGLE_DEG)
            if not math.isfinite(prot_deg):
                prot_deg = 0.0
            prot_deg = max(0.0, min(360.0, prot_deg))
            edge_gap_deg = float(360.0 - prot_deg)

            # If there is no edge gap (prot=360) or not enough agents, use uniform lambdas.
            if alive_count < 2 or edge_gap_deg <= 1e-6:
                self._special_agent_id = None
                self._special_lambda_value = 1.0
                for aid in list(self.agent_states.keys()):
                    self.alive_lambdas[int(aid)] = 1.0
            else:
                # For desired edge_gap_deg in (0, 360), edge_lambda is:
                #   edge_lambda = edge_gap_deg * (alive_count - 1) / (360 - edge_gap_deg)
                denom = 360.0 - edge_gap_deg
                edge_lambda = 1.0
                if denom > 1e-9:
                    edge_lambda = float((edge_gap_deg * float(alive_count - 1)) / denom)
                if not math.isfinite(edge_lambda) or edge_lambda <= 0.0:
                    edge_lambda = 1.0
                self._special_lambda_value = float(edge_lambda)

                # Recompute who should hold the edge lambda based on formation geometry.
                self._update_special_lambda_by_geometry(now=now, target_pos=tuple(position))

            # Populate alive_lambdas with only currently alive agents.
            # Values come from the target-maintained map, defaulting to 1.0.
            alive_lambdas = {
                int(agent_id): float(self.alive_lambdas.get(int(agent_id), 1.0))
                for agent_id in self.agent_states.keys()
            }
            # Keep a local copy too (useful when future logic updates lambdas).
            self.alive_lambdas = dict(alive_lambdas)

            # Create TargetState message
            target_state = TargetState(
                target_id=self.node_id,
                seq=seq,
                position=position,
                velocity=velocity,
                alive_lambdas=alive_lambdas,
                omega_ref=float(omega_ref),
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

        if msg_type == AdversaryState.TYPE:
            try:
                state = AdversaryState.from_json(message)
            except Exception as exc:
                self._logger.warning("Target %s: failed to decode AdversaryState (%s): %r", self.node_id, exc, message)
                return

            now = self.provider.current_time()
            try:
                seq_int = int(state.seq)
            except Exception:
                seq_int = None

            if seq_int is not None and seq_int <= int(self.last_seq_adversary):
                return
            if seq_int is not None:
                self.last_seq_adversary = int(seq_int)
            self.adversary_state = (state, float(now))
            return

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

                # Target is the sole authority for lambda weights.
                # Initialize on first discovery only.
                agent_id_int = int(state.agent_id)
                if agent_id_int not in self.alive_lambdas:
                    self.alive_lambdas[agent_id_int] = 1.0

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

        if self._special_agent_id is not None and self._special_agent_id in expired_agent_ids:
            self._special_agent_id = None
        for agent_id in expired_agent_ids:
            self.agent_states.pop(agent_id, None)
            self.last_seq_agent.pop(agent_id, None)
            self.alive_lambdas.pop(agent_id, None)

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
                        # Initialize new agents with lp=1.0 (default). Keep it even if it already exists.
                        if state.agent_id not in self.alive_lambdas:
                            self.alive_lambdas[state.agent_id] = 1.0
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

        # print average metrics to console
        df_avg = df.mean(numeric_only=True)
        print(  f"Target telemetry averages: "
                f"E_r={df_avg['E_r']:.6f}, E_vr={df_avg['E_vr']:.6f}, "
                f"rho={df_avg['rho']:.6f}, G_max={df_avg['G_max']:.6f}, E_gap={df_avg['E_gap']:.6f}"
            )

