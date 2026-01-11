"""Message/data structures shared by multiple protocols.

This module exists to avoid circular imports between protocols.
"""

from __future__ import annotations

import json


class AgentState:
    """Agent state broadcast message."""

    TYPE = "AgentState"

    def __init__(self, agent_id, seq, position, velocity, u):
        self.agent_id = agent_id
        self.seq = seq
        self.position = position  # (x, y, z)
        self.velocity = velocity  # (vx, vy, vz)
        self.u = u  # soliton internal state (scalar)

    def to_json(self) -> str:
        return json.dumps(
            {
                "type": self.TYPE,
                "agent_id": self.agent_id,
                "seq": self.seq,
                "position": {"x": self.position[0], "y": self.position[1], "z": self.position[2]},
                "velocity": {"x": self.velocity[0], "y": self.velocity[1], "z": self.velocity[2]},
                "u": self.u,
                "sender_id": self.agent_id,
            }
        )

    @staticmethod
    def from_json(json_str: str) -> AgentState:
        message_dict = json.loads(json_str)
        message_type = message_dict.get("type")
        if message_type != AgentState.TYPE:
            raise ValueError(f"Unexpected message type: {message_type!r}")
        pos = message_dict["position"]
        vel = message_dict["velocity"]
        return AgentState(
            agent_id=message_dict["agent_id"],
            seq=message_dict["seq"],
            position=(pos["x"], pos["y"], pos["z"]),
            velocity=(vel["x"], vel["y"], vel["z"]),
            u=message_dict["u"],
        )


class TargetState:
    """Target state broadcast message."""

    TYPE = "TargetState"

    def __init__(
        self,
        target_id: int,
        seq: int,
        position,
        velocity,
        alive_lambdas: dict | None = None,
        omega_ref: float | None = None,
    ):
        self.target_id = target_id
        self.seq = seq
        self.position = position  # (x, y, z)
        self.velocity = velocity  # (vx, vy, vz)
        # Optional map: agent_id -> lp (lambda) weight, for non-uniform spacing.
        # Keys may arrive as str (JSON) or int (local); consumers should normalize.
        self.alive_lambdas = alive_lambdas or {}
        # Desired angular velocity (rad/s) for agents to spin around the target.
        # Optional for backward compatibility.
        self.omega_ref = float(omega_ref) if omega_ref is not None else 0.0

    def to_json(self) -> str:
        """Convert TargetState to JSON string."""
        return json.dumps(
            {
                "type": self.TYPE,
                "target_id": self.target_id,
                "seq": self.seq,
                "position": {"x": self.position[0], "y": self.position[1], "z": self.position[2]},
                "velocity": {"x": self.velocity[0], "y": self.velocity[1], "z": self.velocity[2]},
                "alive_lambdas": self.alive_lambdas,
                "omega_ref": self.omega_ref,
                "sender_id": self.target_id,
            }
        )

    @staticmethod
    def from_json(json_str: str) -> TargetState:
        message_dict = json.loads(json_str)
        message_type = message_dict.get("type")
        if message_type != TargetState.TYPE:
            raise ValueError(f"Unexpected message type: {message_type!r}")
        pos = message_dict["position"]
        vel = message_dict["velocity"]
        return TargetState(
            target_id=message_dict["target_id"],
            seq=message_dict["seq"],
            position=(pos["x"], pos["y"], pos["z"]),
            velocity=(vel["x"], vel["y"], vel["z"]),
            alive_lambdas=message_dict.get("alive_lambdas") or {},
            omega_ref=message_dict.get("omega_ref", 0.0),
        )


class AdversaryState:
    """Adversary state broadcast message."""

    TYPE = "AdversaryState"

    def __init__(self, node_id, seq, position, velocity):
        self.node_id = node_id
        self.seq = seq
        self.position = position  # (x, y, z)
        self.velocity = velocity  # (vx, vy, vz)

    def to_json(self) -> str:
        return json.dumps(
            {
                "type": self.TYPE,
                "node_id": self.node_id,
                "seq": self.seq,
                "position": {"x": self.position[0], "y": self.position[1], "z": self.position[2]},
                "velocity": {"x": self.velocity[0], "y": self.velocity[1], "z": self.velocity[2]},
                "sender_id": self.node_id,
            }
        )

    @staticmethod
    def from_json(json_str: str) -> "AdversaryState":
        message_dict = json.loads(json_str)
        message_type = message_dict.get("type")
        if message_type != AdversaryState.TYPE:
            raise ValueError(f"Unexpected message type: {message_type!r}")
        pos = message_dict["position"]
        vel = message_dict["velocity"]
        return AdversaryState(
            node_id=message_dict["node_id"],
            seq=message_dict["seq"],
            position=(pos["x"], pos["y"], pos["z"]),
            velocity=(vel["x"], vel["y"], vel["z"]),
        )
