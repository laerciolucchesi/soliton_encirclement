"""
Protocol demonstrating velocity-based mobility using VelocityMobilityHandler.

Uses direct method calls instead of standard GrADyS mobility commands.
"""

from gradysim.protocol.interface import IProtocol
from gradysim.protocol.messages.telemetry import Telemetry

import pandas as pd


class VelocityProtocol(IProtocol):
    """Protocol that commands velocity through VelocityMobilityHandler."""
    
    def __init__(self):
        super().__init__()
        self.node_id = None
        self.initial_position = None
        self.final_position = None
        self.final_velocity = None
        self.desired_velocity = None
        self.df = None
        self.velocity_handler = None

        self._velocity_setpoints = [
            (0.0, 5.0, 5.0),
            (5.0, 0.0, -5.0),
            (0.0, -5.0, 5.0),
            (-5.0, 0.0, -5.0),
            (0.0, 0.0, 0.0),
        ]
        self._velocity_setpoint_index = 0
    
    def initialize(self):
        """Initialize and command initial velocity."""
        self.node_id = self.provider.get_id()
        self._velocity_setpoint_index = 0
        self.desired_velocity = self._velocity_setpoints[self._velocity_setpoint_index]
       
        handlers = getattr(self.provider, "handlers", {}) or {}
        self.velocity_handler = handlers.get("VelocityMobilityHandler")

        if self.velocity_handler:
            self.velocity_handler.set_velocity(self.node_id, self.desired_velocity)
            
            # Calculate and display velocity magnitude and direction
            vx, vy, vz = self.desired_velocity
            speed = (vx**2 + vy**2 + vz**2)**0.5
            
            if abs(vy) > abs(vx) and abs(vy) > abs(vz):
                direction = "north" if vy > 0 else "south"
            elif abs(vx) > abs(vy) and abs(vx) > abs(vz):
                direction = "east" if vx > 0 else "west"
            else:
                direction = "vertical"
            
            print(f"Node {self.node_id} initialized")
            print(f"Velocity commanded: {speed:.2f} m/s heading {direction}")

            # Capture and show initial position directly (no telemetry needed)
            self.initial_position = self.velocity_handler.get_node_position(self.node_id)
            if self.initial_position is not None:
                print(f"Initial position: ({self.initial_position[0]:.1f}, {self.initial_position[1]:.1f}, {self.initial_position[2]:.1f})")

            # Create dataframe to store time, position and velocity
            self.df = pd.DataFrame(columns=[
                "t",
                "x", "y", "z",
                "vx", "vy", "vz",
                "vxd", "vyd", "vzd",
            ])

            t0 = self.provider.current_time()
            v0 = self.velocity_handler.get_node_velocity(self.node_id) or (0.0, 0.0, 0.0)
            p0 = self.velocity_handler.get_node_position(self.node_id) or (0.0, 0.0, 0.0)
            v0d = self.desired_velocity or (0.0, 0.0, 0.0)
            self.df.loc[len(self.df)] = [t0, p0[0], p0[1], p0[2], v0[0], v0[1], v0[2], v0d[0], v0d[1], v0d[2]]

            self.schedule_change_speed_timer(10.0) # Schedule next change speed in 10 seconds
    
    def schedule_change_speed_timer(self, timeout: float = None):
        self.provider.schedule_timer("change_speed_timer", self.provider.current_time() + timeout) 

    def handle_timer(self, timer: str):
        """Handle timer events."""
        if timer == "change_speed_timer":
            # Change speed (advance through setpoints; once stopped, stay stopped)
            if self.velocity_handler:
                self._velocity_setpoint_index = min(
                    self._velocity_setpoint_index + 1,
                    len(self._velocity_setpoints) - 1,
                )
                self.desired_velocity = self._velocity_setpoints[self._velocity_setpoint_index]
                self.velocity_handler.set_velocity(self.node_id, self.desired_velocity)
            self.schedule_change_speed_timer(10.0) # Schedule next speed change in 10 seconds
    
    def handle_packet(self, message: str):
        """Handle incoming packets."""
        pass
    
    def handle_telemetry(self, telemetry: Telemetry) -> None:
        """Collect time, position and velocity on telemetry."""
        if not self.velocity_handler:
            return
        if self.df is None:
            return

        t = self.provider.current_time()
        pos = self.velocity_handler.get_node_position(self.node_id)
        vel = self.velocity_handler.get_node_velocity(self.node_id)
        if pos is None or vel is None:
            return

        vdes = self.desired_velocity or (0.0, 0.0, 0.0)
        self.df.loc[len(self.df)] = [t, pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], vdes[0], vdes[1], vdes[2]]
    
    def finish(self):
        """Called when simulation ends."""
        if self.initial_position and self.desired_velocity and self.velocity_handler:
            # Fetch final state from handler
            final_position = self.velocity_handler.get_node_position(self.node_id)
            final_velocity = self.velocity_handler.get_node_velocity(self.node_id)
            
            if final_position is None:
                return
            
            # Calculate actual movement
            dx = final_position[0] - self.initial_position[0]
            dy = final_position[1] - self.initial_position[1]
            dz = final_position[2] - self.initial_position[2]
            net_displacement = (dx**2 + dy**2 + dz**2)**0.5

            
            print()
            print("=" * 60)
            print("SIMULATION RESULTS")
            print("=" * 60)
            print(f"Node {self.node_id}")
            print(f"  Initial position: ({self.initial_position[0]:.2f}, {self.initial_position[1]:.2f}, {self.initial_position[2]:.2f})")
            print(f"  Final position:   ({final_position[0]:.2f}, {final_position[1]:.2f}, {final_position[2]:.2f})")
            print(f"  Movement vector:  ({dx:.2f}, {dy:.2f}, {dz:.2f})")
            print(f"  Net displacement: {net_displacement:.2f} m")
            if final_velocity:
                final_speed = (final_velocity[0]**2 + final_velocity[1]**2 + final_velocity[2]**2)**0.5
                print(f"  Final velocity:   {final_speed:.2f} m/s")
            print("=" * 60)

            # Plot position and velocity over time
            if self.df is not None and len(self.df) > 1:
                try:
                    import matplotlib.pyplot as plt

                    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(10, 9))
                    ax_pos, ax_vx, ax_vy, ax_vz = axes

                    x_line = ax_pos.plot(self.df["t"], self.df["x"], label="x")[0]
                    y_line = ax_pos.plot(self.df["t"], self.df["y"], label="y")[0]
                    z_line = ax_pos.plot(self.df["t"], self.df["z"], label="z")[0]
                    ax_pos.set_ylabel("position (m)")
                    ax_pos.grid(True)
                    ax_pos.legend(loc="best")

                    vx_meas_line = ax_vx.plot(
                        self.df["t"],
                        self.df["vx"],
                        label="vx",
                        color=x_line.get_color(),
                    )[0]
                    ax_vx.plot(
                        self.df["t"],
                        self.df["vxd"],
                        label="vx_des",
                        linestyle="--",
                        drawstyle="steps-post",
                        color=vx_meas_line.get_color(),
                    )
                    ax_vx.set_ylabel("vx (m/s)")
                    ax_vx.grid(True)
                    ax_vx.legend(loc="best")

                    vy_meas_line = ax_vy.plot(
                        self.df["t"],
                        self.df["vy"],
                        label="vy",
                        color=y_line.get_color(),
                    )[0]
                    ax_vy.plot(
                        self.df["t"],
                        self.df["vyd"],
                        label="vy_des",
                        linestyle="--",
                        drawstyle="steps-post",
                        color=vy_meas_line.get_color(),
                    )
                    ax_vy.set_ylabel("vy (m/s)")
                    ax_vy.grid(True)
                    ax_vy.legend(loc="best")

                    vz_meas_line = ax_vz.plot(
                        self.df["t"],
                        self.df["vz"],
                        label="vz",
                        color=z_line.get_color(),
                    )[0]
                    ax_vz.plot(
                        self.df["t"],
                        self.df["vzd"],
                        label="vz_des",
                        linestyle="--",
                        drawstyle="steps-post",
                        color=vz_meas_line.get_color(),
                    )
                    ax_vz.set_xlabel("time (s)")
                    ax_vz.set_ylabel("vz (m/s)")
                    ax_vz.grid(True)
                    ax_vz.legend(loc="best")

                    fig.suptitle(f"Node {self.node_id}: position and velocity vs time")
                    plt.tight_layout(rect=(0, 0, 1, 0.96))
                    plt.show()
                except ImportError:
                    print("matplotlib not available; skipping plots")
