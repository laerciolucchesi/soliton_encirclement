"""Velocity-driven mobility handler for GrADyS-SIM NG.

This handler provides realistic, velocity-based mobility for nodes with
explicit acceleration and velocity constraints.

Author: Laércio Lucchesi
Date: December 27, 2025
"""

from typing import Dict, Tuple

from gradysim.simulator.event import EventLoop
from gradysim.simulator.node import Node
from gradysim.simulator.handler.interface import INodeHandler
from gradysim.protocol.messages.telemetry import Telemetry

from .config import VelocityMobilityConfiguration
from .core import (
    apply_acceleration_limits,
    apply_velocity_limits,
    apply_velocity_tracking_first_order,
    integrate_position,
)


class VelocityMobilityHandler(INodeHandler):
    """Velocity-driven mobility handler for GrADyS-SIM NG."""

    def __init__(self, config: VelocityMobilityConfiguration):
        self._config = config
        self._loop: EventLoop = None
        self._nodes: Dict[int, Node] = {}

        self._current_velocity: Dict[int, Tuple[float, float, float]] = {}
        self._desired_velocity: Dict[int, Tuple[float, float, float]] = {}

        self._update_counter: Dict[int, int] = {}

    def get_label(self) -> str:
        return "VelocityMobilityHandler"

    def register_node(self, node: Node):
        node_id = node.id
        self._nodes[node_id] = node
        self._current_velocity[node_id] = (0.0, 0.0, 0.0)
        self._desired_velocity[node_id] = (0.0, 0.0, 0.0)
        self._update_counter[node_id] = 0

    def inject(self, event_loop: EventLoop):
        self._loop = event_loop

    def initialize(self):
        if self._nodes:
            self._loop.schedule_event(
                self._loop.current_time + self._config.update_rate,
                self._mobility_update,
            )

    def handle_timer(self, timer: str):
        pass

    def handle_packet(self, message: str):
        pass

    def finish(self):
        pass

    def finalize(self):
        pass

    def after_simulation_step(self, iteration: int, time: float):
        pass

    def set_velocity(self, node_id: int, v_des: Tuple[float, float, float]) -> None:
        if node_id not in self._desired_velocity:
            self._current_velocity[node_id] = (0.0, 0.0, 0.0)
            self._desired_velocity[node_id] = v_des
            self._update_counter[node_id] = 0
        else:
            self._desired_velocity[node_id] = v_des

    def get_node_velocity(self, node_id: int) -> Tuple[float, float, float] | None:
        return self._current_velocity.get(node_id)

    def get_node_position(self, node_id: int) -> Tuple[float, float, float] | None:
        node = self._nodes.get(node_id)
        return node.position if node is not None else None

    def _mobility_update(self):
        dt = self._config.update_rate

        for node_id, node in self._nodes.items():
            v_current = self._current_velocity[node_id]
            v_desired = self._desired_velocity[node_id]

            if self._config.tau_xy is None and self._config.tau_z is None:
                v_new = apply_acceleration_limits(
                    v_current,
                    v_desired,
                    dt,
                    self._config.max_acc_xy,
                    self._config.max_acc_z,
                )
            else:
                v_new = apply_velocity_tracking_first_order(
                    v_current,
                    v_desired,
                    dt,
                    self._config.max_acc_xy,
                    self._config.max_acc_z,
                    tau_xy=self._config.tau_xy,
                    tau_z=self._config.tau_z,
                )

            v_new = apply_velocity_limits(
                v_new,
                self._config.max_speed_xy,
                self._config.max_speed_z,
            )

            new_pos = integrate_position(node.position, v_new, dt)
            node.position = new_pos

            self._current_velocity[node_id] = v_new

            self._update_counter[node_id] += 1
            if self._should_emit_telemetry(node_id):
                self._emit_telemetry(node)

        self._loop.schedule_event(
            self._loop.current_time + self._config.update_rate,
            self._mobility_update,
        )

    def _should_emit_telemetry(self, node_id: int) -> bool:
        if not self._config.send_telemetry:
            return False

        count = self._update_counter[node_id]
        return (count % self._config.telemetry_decimation) == 0

    def _emit_telemetry(self, node: Node):
        telemetry = Telemetry(current_position=node.position)

        def send_telemetry():
            node.protocol_encapsulator.handle_telemetry(telemetry)

        self._loop.schedule_event(
            self._loop.current_time,
            send_telemetry,
            f"Node {node.id} handle_telemetry",
        )
