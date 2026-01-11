"""Protocol for the adversary node.

Skeleton implementation:
- Broadcasts AdversaryState containing node_id, seq, position, velocity.
- Motion strategy is intentionally left minimal/undefined for now.
"""

import json
import logging
import math
import random

from gradysim.protocol.interface import IProtocol
from gradysim.protocol.messages.communication import CommunicationCommand, CommunicationCommandType
from gradysim.protocol.messages.telemetry import Telemetry

from config_param import (
    ADVERSARY_MIN_TARGET_DISTANCE,
    ADVERSARY_ROAM_BOUND_XY,
    ADVERSARY_ROAM_SPEED_XY,
    ADVERSARY_STATE_BROADCAST_PERIOD,
    ADVERSARY_STATE_BROADCAST_TIMER_STR,
)
from protocol_messages import AdversaryState, TargetState


class AdversaryProtocol(IProtocol):
    """Implementation of adversary protocol (skeleton)."""

    def __init__(self):
        super().__init__()
        self._logger = logging.getLogger()

    def initialize(self):
        self.node_id = self.provider.get_id()
        self.control_period = float(ADVERSARY_STATE_BROADCAST_PERIOD)

        handlers = getattr(self.provider, "handlers", {}) or {}
        self.velocity_handler = handlers.get("VelocityMobilityHandler")

        # Latest target state (best-effort).
        self.target_state = None  # (TargetState, rxtime)
        self._last_seq_target = -1

        # Random roaming goal in XY.
        self._roam_goal_xy = None  # (x, y)

        self.state_seq = 1
        self._schedule_control_loop_timer()

    @staticmethod
    def _unit2(vec, eps: float = 1e-6):
        n = math.hypot(float(vec[0]), float(vec[1]))
        if not (math.isfinite(n) and n > eps):
            return (1.0, 0.0), 0.0
        return (float(vec[0]) / n, float(vec[1]) / n), n

    def _pick_roam_goal(self, target_pos_xy) -> tuple[float, float]:
        bound = float(ADVERSARY_ROAM_BOUND_XY)
        if not (math.isfinite(bound) and bound > 0.0):
            bound = 50.0

        min_d = float(ADVERSARY_MIN_TARGET_DISTANCE)
        if not (math.isfinite(min_d) and min_d >= 0.0):
            min_d = 0.0

        tx, ty = float(target_pos_xy[0]), float(target_pos_xy[1])

        # Try sampling a point inside the square that also respects min distance.
        for _ in range(50):
            gx = random.uniform(-bound, bound)
            gy = random.uniform(-bound, bound)
            if min_d <= 0.0:
                return gx, gy
            if math.hypot(gx - tx, gy - ty) >= min_d:
                return gx, gy

        # Fallback: choose a random direction and place goal at min distance from target.
        ang = random.uniform(0.0, 2.0 * math.pi)
        gx = tx + min_d * math.cos(ang)
        gy = ty + min_d * math.sin(ang)
        gx = max(-bound, min(bound, gx))
        gy = max(-bound, min(bound, gy))
        return gx, gy

    def _schedule_control_loop_timer(self) -> None:
        self.provider.schedule_timer(
            ADVERSARY_STATE_BROADCAST_TIMER_STR,
            float(self.provider.current_time()) + float(self.control_period),
        )

    def handle_timer(self, timer: str):
        if timer != ADVERSARY_STATE_BROADCAST_TIMER_STR:
            return

        if self.velocity_handler is not None:
            position = self.velocity_handler.get_node_position(self.node_id)
            velocity = self.velocity_handler.get_node_velocity(self.node_id)
        else:
            position = (0.0, 0.0, 0.0)
            velocity = (0.0, 0.0, 0.0)

        if position is None:
            position = (0.0, 0.0, 0.0)
        if velocity is None:
            velocity = (0.0, 0.0, 0.0)

        # --- Simple random roaming with minimum distance to target ---
        # Uses last received TargetState if available; otherwise assumes target at origin.
        target_pos = (0.0, 0.0, 0.0)
        if self.target_state is not None:
            ts, _ = self.target_state
            target_pos = ts.position

        px, py = float(position[0]), float(position[1])
        tx, ty = float(target_pos[0]), float(target_pos[1])

        min_d = float(ADVERSARY_MIN_TARGET_DISTANCE)
        if not (math.isfinite(min_d) and min_d >= 0.0):
            min_d = 0.0

        # If too close to the target, move directly away until we clear min_d.
        away_vec = (px - tx, py - ty)
        away_hat, dist_to_target = self._unit2(away_vec)

        speed = float(ADVERSARY_ROAM_SPEED_XY)
        if not (math.isfinite(speed) and speed > 0.0):
            speed = 0.0

        v_cmd_xy = (0.0, 0.0)
        if min_d > 0.0 and dist_to_target < min_d:
            v_cmd_xy = (speed * away_hat[0], speed * away_hat[1])
        else:
            # Maintain/refresh a random goal inside [-bound, bound] and outside min distance.
            goal = self._roam_goal_xy
            if goal is None:
                goal = self._pick_roam_goal((tx, ty))
                self._roam_goal_xy = goal

            gx, gy = float(goal[0]), float(goal[1])
            to_goal = (gx - px, gy - py)
            to_goal_hat, d_goal = self._unit2(to_goal)

            # If reached goal (or degenerate), pick a new one.
            if not (math.isfinite(d_goal) and d_goal > 1.0):
                goal = self._pick_roam_goal((tx, ty))
                self._roam_goal_xy = goal
                gx, gy = float(goal[0]), float(goal[1])
                to_goal = (gx - px, gy - py)
                to_goal_hat, d_goal = self._unit2(to_goal)

            v_cmd_xy = (speed * to_goal_hat[0], speed * to_goal_hat[1])

        if self.velocity_handler is not None and speed > 0.0:
            try:
                self.velocity_handler.set_velocity(self.node_id, (v_cmd_xy[0], v_cmd_xy[1], 0.0))
            except Exception:
                pass

        msg = AdversaryState(
            node_id=self.node_id,
            seq=self.state_seq,
            position=position,
            velocity=velocity,
        )
        message_json = msg.to_json()
        command = CommunicationCommand(CommunicationCommandType.BROADCAST, message_json)
        self.provider.send_communication_command(command)

        self.state_seq += 1
        self._schedule_control_loop_timer()

    def handle_packet(self, message: str):
        try:
            data = json.loads(message)
        except Exception:
            return

        if data.get("type") != TargetState.TYPE:
            return

        try:
            state = TargetState.from_json(message)
        except Exception:
            return

        # Discard out-of-order messages (simple guard).
        seq = getattr(state, "seq", None)
        if seq is not None:
            try:
                seq_int = int(seq)
            except Exception:
                seq_int = None
            if seq_int is not None and seq_int <= int(getattr(self, "_last_seq_target", -1)):
                return
            if seq_int is not None:
                self._last_seq_target = seq_int

        self.target_state = (state, float(self.provider.current_time()))

    def handle_telemetry(self, telemetry: Telemetry) -> None:
        pass

    def finish(self):
        pass
