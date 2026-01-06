"""
Protocol for the agent node.
"""

import logging
from typing import Dict, Optional, Tuple
import math
import os
import random

import pandas as pd

from gradysim.protocol.interface import IProtocol
from gradysim.protocol.messages.telemetry import Telemetry
from gradysim.protocol.messages.communication import CommunicationCommand, CommunicationCommandType

try:
    from gradysim.simulator.extension.visualization_controller import VisualizationController
except Exception:  # pragma: no cover
    VisualizationController = None

import json

from config_param import (
    CONTROL_LOOP_TIMER_STR,
    CONTROL_PERIOD,
    SIM_DEBUG,
    AGENT_STATE_TIMEOUT,
    TARGET_STATE_TIMEOUT,
    HYSTERESIS_RAD,
    PRUNE_EXPIRED_STATES,
    ENCIRCLEMENT_RADIUS,
    R_MIN,
    K_R,
    K_DR,
    VM_MAX_SPEED_XY,
    VM_MAX_SPEED_Z,
    K_TAU,
    BETA_U,
    ALPHA_U,
    C_COUPLING,
    K_E_TAU,
    K_OMEGA_DAMP,
    USE_SOFT_LIMITER_U,
    U_S,
    FAILURE_CHECK_PERIOD,
    FAILURE_CHECK_TIMER_STR,
    FAILURE_ENABLE,
    FAILURE_MEAN_FAILURES_PER_MIN,
    FAILURE_OFF_TIME,
    FAILURE_RANDOM_SEED,
    FAILURE_RECOVER_TIMER_STR,
)
from protocol_messages import AgentState, TargetState


class AgentProtocol(IProtocol):
    """Implementation of agent protocol."""

    def __init__(self):
        super().__init__()
        self._logger = logging.getLogger()

    def initialize(self):
        self.node_id = self.provider.get_id() # Get the node ID from the provider
        self.control_period = CONTROL_PERIOD  # Control loop period in seconds
        # Schedule the control loop timer for the first time
        self.schedule_control_loop_timer()

        # Failure injection state
        self._failed: bool = False
        if FAILURE_ENABLE:
            if FAILURE_RANDOM_SEED is not None:
                try:
                    random.seed(int(FAILURE_RANDOM_SEED) + int(self.node_id))
                except Exception:
                    pass
            # Defer scheduling the first failure-check timer until after
            # visualization controller initialization. This avoids the case
            # where a node enters failure before _vis exists and thus isn't
            # painted red.

        # Access the VelocityMobilityHandler if available
        handlers = getattr(self.provider, "handlers", {}) or {}
        self.velocity_handler = handlers.get("VelocityMobilityHandler")

        # Latest received states (filled by handle_packet)
        # Stored as (state, rxtime) to support future timeout detection.
        self.target_state: Optional[Tuple[TargetState, float]] = None
        self.agent_states: Dict[int, Tuple[AgentState, float]] = {}

        # Sequence tracking to discard out-of-order messages.
        self.last_seq_agent: Dict[int, int] = {}
        self.last_seq_target: int = -1

        # Neighbor selection results (updated every control tick).
        self.neighbor_pred_id: Optional[int] = None
        self.neighbor_succ_id: Optional[int] = None
        self.neighbor_pred_state: Optional[AgentState] = None
        self.neighbor_succ_state: Optional[AgentState] = None
        self._neighbor_pred_gap: Optional[float] = None
        self._neighbor_succ_gap: Optional[float] = None

        # Broadcast sequence number
        self.agent_state_seq = 1

        # Soliton internal state (scalar). Start at zero by default.
        self.u: float = 0.0

        # Last commanded velocity (world coordinates). Used by the optional local
        # angular-rate damping term to estimate omega_self from the command.
        self.desired_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)

        # Telemetry logging (in-memory). The main.py creates the CSV and sets the path
        # via environment variable to avoid tight coupling with the simulator builder.
        self._csv_path: Optional[str] = os.environ.get("AGENT_LOG_CSV_PATH")
        self._telemetry_rows = []

        # Visualization controller
        self._vis = None
        if VisualizationController is not None:
            try:
                self._vis = VisualizationController(self)
                # Default agent color: blue
                self._vis.paint_node(self.node_id, (0.0, 0.0, 255.0))
            except Exception:
                self._vis = None

        # Now that visualization is initialized (best-effort), schedule the
        # first failure-check timer.
        if FAILURE_ENABLE:
            self.schedule_failure_check_timer()
    
    def schedule_control_loop_timer(self):
        self.provider.schedule_timer(CONTROL_LOOP_TIMER_STR, self.provider.current_time() + self.control_period)

    def schedule_failure_check_timer(self):
        self.provider.schedule_timer(
            FAILURE_CHECK_TIMER_STR,
            self.provider.current_time() + float(FAILURE_CHECK_PERIOD),
        )

    def schedule_failure_recover_timer(self, off_time: float):
        self.provider.schedule_timer(
            FAILURE_RECOVER_TIMER_STR,
            self.provider.current_time() + float(off_time),
        )

    @staticmethod
    def _dot2(a, b) -> float:
        return a[0] * b[0] + a[1] * b[1]

    @staticmethod
    def _norm2(a) -> float:
        return math.sqrt(a[0] * a[0] + a[1] * a[1])

    @staticmethod
    def _unit2(a, eps: float = 1e-6):
        """Return a unit 2D vector with a safe fallback for near-zero norms."""
        n = AgentProtocol._norm2(a)
        if not math.isfinite(n) or n < eps:
            return (1.0, 0.0), 0.0
        return (a[0] / n, a[1] / n), n

    @staticmethod
    def wrap_to_2pi(angle: float) -> float:
        """Wrap angle to [0, 2*pi)."""
        two_pi = 2.0 * math.pi
        wrapped = angle % two_pi
        # Python's % already yields [0, 2*pi) for positive modulus.
        return wrapped

    @staticmethod
    def _theta_2d(target_pos, agent_pos) -> float:
        """Compute target-centric angle in the horizontal plane."""
        dx = agent_pos[0] - target_pos[0]
        dy = agent_pos[1] - target_pos[1]
        return math.atan2(dy, dx)

    def _target_is_alive(self, now: float) -> bool:
        if self.target_state is None:
            return False
        _, rxtime = self.target_state
        return (now - rxtime) <= TARGET_STATE_TIMEOUT

    def _agent_is_alive(self, agent_id: int, now: float) -> bool:
        entry = self.agent_states.get(agent_id)
        if entry is None:
            return False
        _, rxtime = entry
        return (now - rxtime) <= AGENT_STATE_TIMEOUT

    def _prune_expired_states(self, now: float) -> None:
        """Drop expired cached states to prevent unbounded growth.

        Note: pruning is a local bookkeeping action only; it does not affect
        message dissemination, only what we keep in memory.
        """
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

    def compute_tangential_unit_vector(self, target_pos, own_pos) -> Tuple[float, float]:
        """Compute unit tangential direction t_hat in the XY plane.

        Given target-centered radial unit vector r_hat, define:
            t_hat = (-r_hat_y, r_hat_x)

        Uses a safe normalization with fallback to avoid division by zero.
        """
        r_vec = (own_pos[0] - target_pos[0], own_pos[1] - target_pos[1])
        r_hat, _ = self._unit2(r_vec, eps=1e-6)
        return (-r_hat[1], r_hat[0])

    @staticmethod
    def compute_omega_about_target_xy(
        *,
        target_pos: Tuple[float, float, float],
        target_vel: Tuple[float, float, float],
        pos: Tuple[float, float, float],
        vel: Tuple[float, float, float],
        eps: float = 1e-6,
    ) -> Optional[float]:
        """Estimate angular rate omega (rad/s) around the target in the XY plane.

        Uses only instantaneous kinematics:
          omega = v_tan / r
        where v_tan is the tangential component of relative velocity w.r.t. the target.
        """
        rx = float(pos[0] - target_pos[0])
        ry = float(pos[1] - target_pos[1])
        r = math.hypot(rx, ry)
        if not (math.isfinite(r) and r > eps):
            return None

        r_hat_x = rx / r
        r_hat_y = ry / r
        t_hat_x = -r_hat_y
        t_hat_y = r_hat_x

        vrel_x = float(vel[0] - target_vel[0])
        vrel_y = float(vel[1] - target_vel[1])
        if not (math.isfinite(vrel_x) and math.isfinite(vrel_y)):
            return None

        v_tan = vrel_x * t_hat_x + vrel_y * t_hat_y
        omega = v_tan / r
        if not math.isfinite(omega):
            return None
        return float(omega)

    @staticmethod
    def compute_spacing_error(gap_pred: Optional[float], gap_succ: Optional[float]) -> float:
        """Local spacing imbalance error (normalized, no global N required).

        Uses only the two local gaps:
            e_tau = (gap_succ - gap_pred) / (gap_succ + gap_pred)

        This yields a dimensionless error in [-1, 1] (when gaps are positive),
        improving conditioning under partial visibility (e.g., limited range).

        If gaps are unavailable (e.g., missing neighbors/target), returns 0.0.
        """
        if gap_pred is None or gap_succ is None:
            return 0.0
        denom = float(gap_succ + gap_pred)
        if not math.isfinite(denom) or denom <= 1e-9:
            return 0.0
        return float((gap_succ - gap_pred) / denom)

    def update_soliton_state(self, *, u: float, u_pred: float, u_succ: float, e_tau: float, dt: float) -> float:
        """Discrete-time soliton-like internal state update.

        u_next = u + dt * (
            C_COUPLING * (u_succ - u_pred)
            - BETA_U * u
            - ALPHA_U * u^3
            + K_E_TAU * e_tau
        )
        """
        if USE_SOFT_LIMITER_U and math.isfinite(U_S) and U_S > 0.0:
            # Soft limiter with p=2:
            #   g(u) = u^3 / (1 + (|u|/U_S)^2)
            u_abs = abs(u)
            u3 = u * u * u
            nonlinear = u3 / (1.0 + (u_abs / U_S) * (u_abs / U_S))
        else:
            nonlinear = u * u * u

        du = (
            C_COUPLING * (u_succ - u_pred)
            - BETA_U * u
            - ALPHA_U * nonlinear
            + K_E_TAU * e_tau
        )
        return float(u + dt * du)

    @staticmethod
    def compute_tangential_velocity(
        u: float,
        t_hat: Tuple[float, float],
        *,
        r_eff: float = 1.0,
    ) -> Tuple[float, float, float]:
        """Convert soliton state into tangential velocity (XYZ).

        Note: scaling the linear tangential speed by r_eff makes the induced
        angular rate approximately omega ≈ v_tau / r ≈ K_TAU * u (for r > R_MIN),
        independent of ENCIRCLEMENT_RADIUS.
        """
        if not math.isfinite(r_eff) or r_eff <= 0.0:
            r_eff = 1.0
        v_tau_corr = K_TAU * u * r_eff
        return (v_tau_corr * t_hat[0], v_tau_corr * t_hat[1], 0.0)

    @staticmethod
    def compose_final_velocity(
        v_rad: Tuple[float, float, float],
        v_tau: Tuple[float, float, float],
        v_target: Tuple[float, float, float],
    ) -> Tuple[float, float, float]:
        """Compose final commanded velocity: v_cmd = v_rad + v_tau + v_target."""
        return (
            v_rad[0] + v_tau[0] + v_target[0],
            v_rad[1] + v_tau[1] + v_target[1],
            v_rad[2] + v_tau[2] + v_target[2],
        )

    @staticmethod
    def _clamp_velocity_to_limits(v: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Clamp a commanded velocity to configured mobility limits.

        This is a safety guard to prevent extreme values from reaching the
        mobility model (which may otherwise overflow before saturating).
        """
        vx, vy, vz = v
        if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(vz)):
            return (0.0, 0.0, 0.0)

        v_xy = math.hypot(vx, vy)
        if v_xy > VM_MAX_SPEED_XY and v_xy > 0.0:
            scale = VM_MAX_SPEED_XY / v_xy
            vx *= scale
            vy *= scale

        if abs(vz) > VM_MAX_SPEED_Z:
            vz = math.copysign(VM_MAX_SPEED_Z, vz)

        return (vx, vy, vz)

    def get_two_neighbors(self, now: float, own_position) -> Tuple[Optional[int], Optional[int], Optional[float], Optional[float], int, Optional[float]]:
        """Select predecessor/successor around the target using only locally received states.

        Returns:
            pred_id, succ_id: Neighbor node IDs (or None)
            pred_gap, succ_gap: Angular gaps in (0, 2*pi) (or None)
            alive_count: Number of other alive agents considered
            theta_i: Own target-centric angle (or None if target unavailable)

        Degenerate cases:
            - If no other alive agents exist: (None, None)
            - If exactly one other alive agent exists: it is both predecessor and successor
        """

        if not self._target_is_alive(now):
            return None, None, None, None, 0, None

        target_state, _ = self.target_state  # type: ignore[assignment]
        target_pos = target_state.position
        theta_i = self._theta_2d(target_pos, own_position)

        candidates = []  # list of (agent_id, theta_j)
        for agent_id, (state, rxtime) in self.agent_states.items():
            if agent_id == self.node_id:
                continue
            if (now - rxtime) > AGENT_STATE_TIMEOUT:
                continue
            theta_j = self._theta_2d(target_pos, state.position)
            candidates.append((agent_id, theta_j))

        theta_by_id = {int(agent_id): float(theta_j) for agent_id, theta_j in candidates}

        alive_count = len(candidates)
        if alive_count == 0:
            return None, None, None, None, 0, theta_i

        # Robust circular neighbor selection:
        # Sort by (theta, agent_id) to break ties deterministically when two agents
        # share the same target-centric angle. This avoids the "d == 0" exclusion
        # case that can cause two agents to effectively ignore each other.
        ring: list[tuple[float, int]] = [(theta_i, int(self.node_id))]
        ring.extend((theta_j, int(agent_id)) for agent_id, theta_j in candidates)
        ring.sort(key=lambda x: (x[0], x[1]))

        self_idx = next((k for k, (_, aid) in enumerate(ring) if aid == int(self.node_id)), None)
        if self_idx is None:
            return None, None, None, None, 0, theta_i

        pred_theta, pred_id_int = ring[(self_idx - 1) % len(ring)]
        succ_theta, succ_id_int = ring[(self_idx + 1) % len(ring)]

        # With exactly one other alive agent, predecessor and successor are the same.
        pred_id = int(pred_id_int)
        succ_id = int(succ_id_int)
        pred_gap = self.wrap_to_2pi(theta_i - pred_theta)
        succ_gap = self.wrap_to_2pi(succ_theta - theta_i)

        # If neighbors crossed (topological swap), accept the swap immediately.
        # Otherwise hysteresis can keep outdated neighbor IDs and make two agents
        # apply nearly identical tangential commands, causing them to remain close.
        crossed_swap = (
            self.neighbor_pred_id is not None
            and self.neighbor_succ_id is not None
            and pred_id == self.neighbor_succ_id
            and succ_id == self.neighbor_pred_id
        )

        # Hysteresis to reduce jitter: keep current neighbors unless there is a meaningful improvement.
        # If current neighbors are missing/expired, always accept new selection.
        def _expired_or_missing(agent_id: Optional[int]) -> bool:
            return agent_id is None or not self._agent_is_alive(agent_id, now)

        # Predecessor hysteresis
        if (
            not crossed_swap
            and not _expired_or_missing(self.neighbor_pred_id)
            and pred_id != self.neighbor_pred_id
        ):
            if pred_gap is not None and self.neighbor_pred_id is not None:
                old_theta = theta_by_id.get(int(self.neighbor_pred_id))
                if old_theta is not None:
                    old_gap = self.wrap_to_2pi(theta_i - old_theta)
                    improvement = old_gap - pred_gap
                    if improvement <= HYSTERESIS_RAD:
                        pred_id = int(self.neighbor_pred_id)
                        pred_gap = old_gap

        # Successor hysteresis
        if (
            not crossed_swap
            and not _expired_or_missing(self.neighbor_succ_id)
            and succ_id != self.neighbor_succ_id
        ):
            if succ_gap is not None and self.neighbor_succ_id is not None:
                old_theta = theta_by_id.get(int(self.neighbor_succ_id))
                if old_theta is not None:
                    old_gap = self.wrap_to_2pi(old_theta - theta_i)
                    improvement = old_gap - succ_gap
                    if improvement <= HYSTERESIS_RAD:
                        succ_id = int(self.neighbor_succ_id)
                        succ_gap = old_gap

        return pred_id, succ_id, pred_gap, succ_gap, alive_count, theta_i

    def handle_timer(self, timer: str):
        if timer == FAILURE_CHECK_TIMER_STR:
            if self._failed:
                return

            # Convert a desired mean failure rate (failures/min) into a per-check probability.
            # We model failures approximately as a Poisson process with rate lambda (1/s):
            #   p = 1 - exp(-lambda * dt)
            # where dt = FAILURE_CHECK_PERIOD.
            dt = float(FAILURE_CHECK_PERIOD)
            if not (math.isfinite(dt) and dt > 0.0):
                dt = 1.0

            rate_per_min = float(FAILURE_MEAN_FAILURES_PER_MIN)
            if math.isfinite(rate_per_min) and rate_per_min > 0.0:
                lam = rate_per_min / 60.0
                try:
                    p = 1.0 - math.exp(-lam * dt)
                except Exception:
                    p = 0.0
            else:
                p = 0.0

            if not math.isfinite(p):
                p = 0.0
            p = max(0.0, min(1.0, p))

            if p > 0.0 and random.random() < p:
                self._failed = True

                if self._vis is not None:
                    try:
                        # Change agent color to red to indicate failure
                        self._vis.paint_node(self.node_id, (255.0, 0.0, 0.0))
                    except Exception:
                        pass

                # Cancel main control timer for this node.
                self.provider.cancel_timer(CONTROL_LOOP_TIMER_STR)

                # Freeze commanded velocity (best-effort).
                if self.velocity_handler is not None:
                    try:
                        self.velocity_handler.set_velocity(self.node_id, (0.0, 0.0, 0.0))
                    except Exception:
                        pass

                off_time = float(FAILURE_OFF_TIME)
                if not (math.isfinite(off_time) and off_time > 0.0):
                    off_time = 0.0
                self.schedule_failure_recover_timer(off_time)

                if SIM_DEBUG:
                    print(f"Agent {self.node_id} ENTERED FAILURE for {off_time:.2f}s")
                return

            self.schedule_failure_check_timer()
            return

        if timer == FAILURE_RECOVER_TIMER_STR:
            self._failed = False

            if self._vis is not None:
                try:
                    # Restore agent color to blue upon recovery
                    self._vis.paint_node(self.node_id, (0.0, 0.0, 255.0))
                except Exception:
                    pass

            self.schedule_control_loop_timer()
            self.schedule_failure_check_timer()
            if SIM_DEBUG:
                print(f"Agent {self.node_id} RECOVERED from FAILURE")
            return

        if timer == CONTROL_LOOP_TIMER_STR:
            if self._failed:
                return
            
            # 1) Broadcast the agent current state
            if self.velocity_handler:
                position = self.velocity_handler.get_node_position(self.node_id)
                velocity = self.velocity_handler.get_node_velocity(self.node_id)
            else:
                position = (0.0, 0.0, 0.0)
                velocity = (0.0, 0.0, 0.0)

            seq = self.agent_state_seq

            agent_state = AgentState(
                agent_id=self.node_id,
                seq=seq,
                position=position,
                velocity=velocity,
                u=self.u,
            )
            message_json = agent_state.to_json()
            command = CommunicationCommand(CommunicationCommandType.BROADCAST, message_json)
            self.provider.send_communication_command(command)

            if SIM_DEBUG:
                print(
                    f"Agent {self.node_id} broadcasted AgentState "
                    f"seq={seq}, position={position}, velocity={velocity}, u={self.u}"
                )

            self.agent_state_seq = seq + 1

            # 2) Neighbor calculations
            now = self.provider.current_time()
            self._prune_expired_states(now)
            pred_id, succ_id, pred_gap, succ_gap, alive_count, theta_i = self.get_two_neighbors(now, position)

            self.neighbor_pred_id = pred_id
            self.neighbor_succ_id = succ_id
            self._neighbor_pred_gap = pred_gap
            self._neighbor_succ_gap = succ_gap

            self.neighbor_pred_state = None
            self.neighbor_succ_state = None
            if pred_id is not None and pred_id in self.agent_states:
                self.neighbor_pred_state = self.agent_states[pred_id][0]
            if succ_id is not None and succ_id in self.agent_states:
                self.neighbor_succ_state = self.agent_states[succ_id][0]

            if SIM_DEBUG:
                theta_str = f"{theta_i:.3f}" if theta_i is not None else "None"
                pred_gap_str = f"{pred_gap:.3f}" if pred_gap is not None else "None"
                succ_gap_str = f"{succ_gap:.3f}" if succ_gap is not None else "None"
                print(
                    f"Agent {self.node_id} neighbors: "
                    f"theta={theta_str}, pred={pred_id}, succ={succ_id}, "
                    f"pred_gap={pred_gap_str}, succ_gap={succ_gap_str}, alive={alive_count}"
                )

            # 3) Radial Control loop logic goes here

            # Robust PD-like radial controller in the horizontal plane.
            # The commanded velocity is expressed in world coordinates and includes the target velocity,
            # because the target may move.
            v_rad: Tuple[float, float, float] = (0.0, 0.0, 0.0)
            v_target: Tuple[float, float, float] = (0.0, 0.0, 0.0)

            if self.target_state is not None:
                target_state, _ = self.target_state
                v_target = target_state.velocity

            if self.velocity_handler is not None and self.target_state is not None:
                target_state, _ = self.target_state
                p_t = target_state.position
                v_t = target_state.velocity

                # Radial geometry in XY
                r_vec = (position[0] - p_t[0], position[1] - p_t[1])
                e_r, r = self._unit2(r_vec, eps=1e-6)

                # Radial error: positive if too far, negative if too close.
                e = r - ENCIRCLEMENT_RADIUS

                # Relative radial speed wrt the target.
                v_rel_xy = (velocity[0] - v_t[0], velocity[1] - v_t[1])
                v_r = self._dot2(v_rel_xy, e_r)

                # PD-like correction (relative to the target).
                v_r_corr = -K_R * e - K_DR * v_r

                # Radial velocity contribution (do NOT add target velocity here).
                v_rad_xy = (v_r_corr * e_r[0], v_r_corr * e_r[1])
                v_rad_z = 0.0

                # Guard against NaNs/Infs propagating.
                if not (
                    math.isfinite(v_rad_xy[0])
                    and math.isfinite(v_rad_xy[1])
                    and math.isfinite(v_rad_z)
                ):
                    v_rad_xy = (0.0, 0.0)
                    v_rad_z = 0.0

                v_rad = (v_rad_xy[0], v_rad_xy[1], v_rad_z)

                if SIM_DEBUG:
                    print(
                        f"Agent {self.node_id} radial: r={r:.3f}, e={e:.3f}, "
                        f"v_r={v_r:.3f}, v_r_corr={v_r_corr:.3f}, v_rad={v_rad}"
                    )


            # 4) Tangential Control loop logic goes here

            v_tau: Tuple[float, float, float] = (0.0, 0.0, 0.0)
            if self.target_state is not None:
                target_state, _ = self.target_state

                # Tangential direction in XY plane.
                t_hat = self.compute_tangential_unit_vector(target_state.position, position)

                # Effective radius used by the tangential mapping and omega estimation.
                r_xy = math.hypot(position[0] - target_state.position[0], position[1] - target_state.position[1])
                r_min = float(R_MIN)
                if not (math.isfinite(r_min) and r_min > 0.0):
                    r_min = 1e-6
                if math.isfinite(r_xy):
                    r_eff = max(r_xy, r_min)
                else:
                    r_eff = r_min

                # Local spacing imbalance error uses only neighbor gaps.
                e_tau = self.compute_spacing_error(pred_gap, succ_gap)

                # Optional local angular-rate damping using only target + 2-neighbor kinematics.
                # e_tau_eff = e_tau - K_OMEGA_DAMP * (omega_self - omega_ref)
                e_tau_eff = float(e_tau)
                if math.isfinite(K_OMEGA_DAMP) and K_OMEGA_DAMP > 0.0:
                    # Compute omega_self from the commanded tangential component (m/s) to keep
                    # the damping term consistent with the tangential mapping. Best-effort.
                    v_cmd_prev = getattr(self, "desired_velocity", (0.0, 0.0, 0.0))
                    v_tan_cmd = float(v_cmd_prev[0] * t_hat[0] + v_cmd_prev[1] * t_hat[1])
                    omega_self = None
                    if math.isfinite(v_tan_cmd) and math.isfinite(r_eff) and r_eff > 0.0:
                        omega_self = float(v_tan_cmd / r_eff)

                    omega_pred = None
                    omega_succ = None
                    if self.neighbor_pred_state is not None:
                        omega_pred = self.compute_omega_about_target_xy(
                            target_pos=target_state.position,
                            target_vel=target_state.velocity,
                            pos=self.neighbor_pred_state.position,
                            vel=self.neighbor_pred_state.velocity,
                        )
                    if self.neighbor_succ_state is not None:
                        omega_succ = self.compute_omega_about_target_xy(
                            target_pos=target_state.position,
                            target_vel=target_state.velocity,
                            pos=self.neighbor_succ_state.position,
                            vel=self.neighbor_succ_state.velocity,
                        )

                    omega_ref = None
                    if omega_pred is not None and omega_succ is not None:
                        omega_ref = 0.5 * (omega_pred + omega_succ)
                    elif omega_pred is not None:
                        omega_ref = omega_pred
                    elif omega_succ is not None:
                        omega_ref = omega_succ

                    if omega_self is not None and omega_ref is not None:
                        domega = omega_self - omega_ref
                        if math.isfinite(domega):
                            e_tau_eff = float(e_tau - (K_OMEGA_DAMP * domega))
                        if not math.isfinite(e_tau_eff):
                            e_tau_eff = float(e_tau)

                # Neighbor soliton states (0.0 if unavailable).
                u_pred = 0.0
                u_succ = 0.0
                if self.neighbor_pred_state is not None:
                    u_pred = float(self.neighbor_pred_state.u)
                if self.neighbor_succ_state is not None:
                    u_succ = float(self.neighbor_succ_state.u)

                # Update internal soliton state (persistent) and compute tangential velocity.
                dt = float(self.control_period)
                self.u = self.update_soliton_state(u=self.u, u_pred=u_pred, u_succ=u_succ, e_tau=e_tau_eff, dt=dt)
                if not math.isfinite(self.u):
                    self.u = 0.0

                # Tangential velocity mapping (linear m/s command):
                #   v_tau = (K_TAU * u * r_eff) * t_hat
                # This yields omega ≈ v_tau/r ≈ K_TAU * u (for r > R_MIN), independent of R.
                v_tau = self.compute_tangential_velocity(self.u, t_hat, r_eff=r_eff)

                if SIM_DEBUG:
                    print(
                        f"Agent {self.node_id} tangential: e_tau={e_tau:.3f}, e_tau_eff={e_tau_eff:.3f}, "
                        f"u_pred={u_pred:.3f}, u_succ={u_succ:.3f}, u={self.u:.3f}, v_tau={v_tau}"
                    )

            # Final velocity composition and single command application.
            # v_cmd = v_rad + v_tau + v_target
            if self.velocity_handler is not None:
                v_cmd = self.compose_final_velocity(v_rad, v_tau, v_target)

                # Guard against NaNs/Infs propagating into the mobility handler.
                if not (math.isfinite(v_cmd[0]) and math.isfinite(v_cmd[1]) and math.isfinite(v_cmd[2])):
                    v_cmd = v_target

                # Safety clamp to mobility limits (prevents numeric overflow upstream).
                v_cmd = self._clamp_velocity_to_limits(v_cmd)

                self.desired_velocity = v_cmd
                self.velocity_handler.set_velocity(self.node_id, self.desired_velocity)

                if SIM_DEBUG:
                    print(f"Agent {self.node_id} v_cmd={self.desired_velocity}")

            # Reschedule the control loop timer
            self.schedule_control_loop_timer()

    def handle_packet(self, message: str):
        if getattr(self, "_failed", False):
            return
        try:
            data = json.loads(message)
        except Exception as exc:
            self._logger.warning("Agent %s: failed to parse packet as JSON (%s): %r", self.node_id, exc, message)
            return

        msg_type = data.get("type")
        if msg_type == TargetState.TYPE:
            try:
                state = TargetState.from_json(message)
            except Exception as exc:
                self._logger.warning("Agent %s: failed to decode TargetState (%s): %r", self.node_id, exc, message)
            else:
                # Discard out-of-order target messages.
                # Recovery rule: if the target was considered expired (no recent TargetState),
                # accept a sequence reset after a restart.
                now = self.provider.current_time()
                target_expired = True
                if self.target_state is not None:
                    _, last_rxtime = self.target_state
                    target_expired = (now - last_rxtime) > TARGET_STATE_TIMEOUT

                if state.seq <= self.last_seq_target and not target_expired:
                    return
                self.last_seq_target = state.seq
                rxtime = now
                self.target_state = (state, rxtime)
                if SIM_DEBUG:
                    ts, rxtime = self.target_state
                    print(
                        f"Agent {self.node_id} received TargetState "
                        f"rxtime={rxtime:.3f}, seq={ts.seq}, target_id={ts.target_id}, position={ts.position}, velocity={ts.velocity}"
                    )
            return

        if msg_type == AgentState.TYPE:
            try:
                state = AgentState.from_json(message)
            except Exception as exc:
                self._logger.warning("Agent %s: failed to decode AgentState (%s): %r", self.node_id, exc, message)
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

            if SIM_DEBUG:
                print(
                    f"Agent {self.node_id} received AgentState "
                    f"rxtime={rxtime:.3f}, seq={state.seq}, agent_id={state.agent_id}, position={state.position}, "
                    f"velocity={state.velocity}, u={state.u}"
                )

            if state.agent_id != self.node_id:
                self.agent_states[state.agent_id] = (state, rxtime)
            return

        # Unknown/unsupported message type
        self._logger.debug("Agent %s: ignoring packet type=%r", self.node_id, msg_type)

        
        

    def handle_telemetry(self, telemetry: Telemetry) -> None:
        if not self._csv_path:
            return

        now = float(self.provider.current_time())

        if self.velocity_handler:
            vx, vy, vz = self.velocity_handler.get_node_velocity(self.node_id)
        else:
            vx = vy = vz = 0.0

        v_norm = float(math.sqrt(vx * vx + vy * vy + vz * vz))

        self._telemetry_rows.append(
            {
                "node_id": int(self.node_id),
                "timestamp": now,
                "u": float(self.u),
                "velocity_norm": v_norm,
            }
        )

    def finish(self):
        if not self._csv_path or not self._telemetry_rows:
            return

        try:
            df = pd.DataFrame(self._telemetry_rows, columns=["node_id", "timestamp", "u", "velocity_norm"])

            # Append without header; main.py already created the file with header.
            # Still guard for the case where the file was removed mid-run.
            file_exists = os.path.exists(self._csv_path)
            df.to_csv(self._csv_path, mode="a", header=not file_exists, index=False)
        except Exception as exc:
            self._logger.warning(
                "Agent %s: failed to write telemetry CSV (%s): %r",
                getattr(self, "node_id", "?"),
                exc,
                self._csv_path,
            )

