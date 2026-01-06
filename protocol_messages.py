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

    def __init__(self, target_id: int, seq: int, position, velocity):
        self.target_id = target_id
        self.seq = seq
        self.position = position  # (x, y, z)
        self.velocity = velocity  # (vx, vy, vz)

    def to_json(self) -> str:
        """Convert TargetState to JSON string."""
        return json.dumps(
            {
                "type": self.TYPE,
                "target_id": self.target_id,
                "seq": self.seq,
                "position": {"x": self.position[0], "y": self.position[1], "z": self.position[2]},
                "velocity": {"x": self.velocity[0], "y": self.velocity[1], "z": self.velocity[2]},
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
        )
