# GrADyS-SIM NG Velocity Mobility Handler

A velocity-driven mobility handler for the [GrADyS-SIM NG](https://github.com/Project-GrADyS/gradys-sim-nextgen) simulator, designed for distributed controllers that output velocity commands rather than waypoints.

## Overview

This handler provides realistic, physics-constrained mobility for simulated nodes based on direct velocity control. Unlike waypoint-based navigation, this approach is ideal for:

- **Distributed control algorithms** that output velocity vectors
- **Swarm encirclement** using soliton-like wave control laws
- **Real-time reactive behaviors** without explicit path planning
- **Continuous motion control** without arrival semantics

## Key Features

- **Direct velocity control** — Command velocity vectors directly, no waypoints
- **Independent constraints** — Separate horizontal (xy) and vertical (z) limits
- **Acceleration limiting** — Realistic velocity tracking with configurable acceleration bounds
- **Optional 1st-order tracking (τ)** — Time-constant-based velocity response (quadrotor-like) with acceleration saturation
- **Velocity saturation** — Enforces maximum speed constraints
- **Optional telemetry** — Configurable position broadcasts with decimation support

## Installation

### From source

The recommended setup is to install this repository into a **virtual environment** and use an **editable install** while you are developing.

Why a virtual environment?

- **Isolation:** avoids mixing `gradysim`, `pandas`, `matplotlib`, etc. with other projects on your machine.
- **Reproducibility:** you can recreate the same environment later (or on another machine) with fewer surprises.
- **Safety:** upgrades won’t accidentally break your system Python or other research code.

Why an editable install (`-e`)?

- **Fast iteration:** Python imports the package from your working tree, so edits under `src/` take effect immediately.
- **No reinstall loop:** you don’t need to reinstall after every code change.

#### Step-by-step (Windows + PowerShell)

1) **Enter the project folder**

Reason: `pip install` needs to run where `pyproject.toml` is.

```bash
cd velocity_mobility
```

2) **Create and activate a virtual environment** (recommended)

Reason: ensures all dependencies are installed in an isolated folder (`.venv/`).

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
```

3) **Upgrade packaging tools**

Reason: prevents common build/metadata issues with editable installs and dependency resolution.

```powershell
python -m pip install --upgrade pip setuptools wheel
```

4) **Install this package**

Choose one mode:

- **Editable (recommended for development):** changes in `src/` are picked up immediately.

```powershell
python -m pip install -e .
```

- **Non-editable (closer to “end-user” install):** useful to validate packaging, but slower for development.

```powershell
python -m pip install .
```

5) **(Optional) Install development dependencies**

Reason: the `dev` extra installs tools used for testing and coverage (e.g., `pytest`, `pytest-cov`).

```powershell
python -m pip install -e ".[dev]"
```

#### Quick verification

Reason: confirms imports and that you’re using the expected environment.

```powershell
python -c "import gradysim_velocity_mobility as m; print(m.__version__)"
```

#### Common troubleshooting

- **PowerShell blocks activation scripts:** you may need to allow local scripts for your user.

    ```powershell
    Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned
    ```

- **You installed to the wrong Python:** prefer `python -m pip ...` (it targets the active interpreter), especially when multiple Pythons are installed.
 

### Requirements

- Python 3.8+
- gradysim >= 0.7.0
- pandas >= 2.0
- matplotlib >= 3.7

## Quick Start

If you want a runnable end-to-end demo (GrADyS-SIM NG simulation builder + handler + protocol + visualization), jump to **Demo (single node + visualization)** and run:

```bash
python main.py
```

```python
from gradysim_velocity_mobility import VelocityMobilityHandler, VelocityMobilityConfiguration


# Configure the handler
config = VelocityMobilityConfiguration(
    update_rate=0.1,        # Update every 0.1 seconds
    max_speed_xy=10.0,      # Max horizontal speed: 10 m/s
    max_speed_z=5.0,        # Max vertical speed: 5 m/s
    max_acc_xy=2.0,         # Max horizontal acceleration: 2 m/s²
    max_acc_z=1.0,          # Max vertical acceleration: 1 m/s²
    # Optional: 1st-order lag (time constant) for velocity tracking
    # tau_xy=0.5,            # Horizontal tracking time constant (s)
    # tau_z=0.8,             # Vertical tracking time constant (s)
    send_telemetry=True,
    telemetry_decimation=10,
)


# Create the handler and add it to your SimulationBuilder:
handler = VelocityMobilityHandler(config)


# In your protocol, command velocity (typical pattern in GrADyS-SIM NG):
#
# handlers = getattr(self.provider, "handlers", {}) or {}
# mobility = handlers.get("VelocityMobilityHandler")
# if mobility is not None:
#     mobility.set_velocity(self.provider.get_id(), (vx, vy, vz))
pass
```

### Typical parameter ranges by drone “profile” (rule-of-thumb)

These values are meant as **starting points** for *trajectory-level* simulations (i.e., you care about path/velocity smoothness more than full attitude/thrust dynamics).
Real drones vary widely with mass, prop size, battery voltage, controller tuning, and mission constraints.

How to read the table:

- `max_speed_xy` limits $\|v_{xy}\|$ (norm in the horizontal plane); `max_speed_z` limits $|v_z|$.
- `max_acc_xy` limits $\|a_{xy}\|$ (norm); `max_acc_z` limits $|a_z|$.
- `tau_xy`/`tau_z` are optional; when set, the model behaves like a 1st-order system toward the commanded velocity before acceleration saturation.

| Profile | update_rate (s) | max_speed_xy (m/s) | max_speed_z (m/s) | max_acc_xy (m/s²) | max_acc_z (m/s²) | tau_xy (s) | tau_z (s) |
|---|---:|---:|---:|---:|---:|---:|---:|
| Cinematic / smooth filming | 0.02–0.05 | 3–12 | 1–6 | 1–5 | 2–6 | 0.5–1.2 | 0.7–1.5 |
| Survey / mapping / inspection | 0.02–0.05 | 5–15 | 1–3 | 1–4 | 2–5 | 0.6–1.2 | 0.8–1.8 |
| Cargo / heavy-lift | 0.02–0.05 | 3–10 | 1–4 | 1–3 | 2–5 | 0.7–1.5 | 0.8–2.0 |
| Racing / agile FPV | 0.005–0.02 | 15–40 | 8–20 | 8–25 | 10–25 | 0.15–0.4 | 0.2–0.6 |
| Micro / indoor (ducted, tiny quads) | 0.01–0.03 | 1–6 | 1–3 | 2–10 | 3–12 | 0.2–0.7 | 0.3–0.9 |

Practical tuning tips:

- If motion looks **too “snappy”**, increase `tau_*` (or reduce `max_acc_*`).
- If it feels **too sluggish**, decrease `tau_*` and/or increase `max_acc_*`.
- If you don’t want 1st-order lag, leave `tau_xy=None` and `tau_z=None` (legacy acceleration-limited tracking).

### Demo (single node + visualization)

This repository includes a runnable demo that builds a single-node simulation using:

- `VelocityMobilityHandler` for motion
- `VelocityProtocol` (in `protocol.py`) to command velocities (changes every 10 seconds)
- Telemetry logging to a pandas DataFrame and matplotlib plots at the end

Run:

```bash
python main.py
```

If you see an error like `OSError: [Errno 10048] ... bind ... port 5678`, the visualization WebSocket port is already in use.
Change the port in `VisualizationConfiguration(...)` (e.g., `port=5679`) or stop the process using that port.

## Architecture

The package is organized into three core modules:

- **`config.py`** — Configuration dataclass
- **`core.py`** — Pure mathematical functions (testable, reusable)
- **`handler.py`** — Main mobility handler implementing `INodeHandler`

This separation allows the mathematical core to be tested independently and potentially reused in other contexts.

## Examples

### Constant Velocity Motion

```bash
python .\examples\ex_constant_velocity.py
```

This is a **core-only** demo: it uses only the pure functions in `gradysim_velocity_mobility.core` (no GrADyS-SIM NG runtime required).
For the main, end-to-end example (simulation builder + handler + protocol + visualization), use `main.py` and `protocol.py`.

## Testing

Run the test suite with pytest:

```bash
# Run all tests
python -m pytest

# Run with coverage
python -m pytest --cov=gradysim_velocity_mobility --cov-report=html

# Run specific test file
python -m pytest tests/test_core_limits.py -v
```

## Physics Model

### Velocity Limits

Two independent scalar constraints:

- **Horizontal**: `||v_xy|| ≤ max_speed_xy`
- **Vertical**: `|v_z| ≤ max_speed_z`

### Acceleration Limits

Two independent scalar constraints:

- **Horizontal**: `||a_xy|| ≤ max_acc_xy`
- **Vertical**: `|a_z| ≤ max_acc_z`

### Position Integration

Simple Euler integration:

```
x_{k+1} = x_k + v_k * dt
```

where `dt` is the update rate.

### Optional 1st-order velocity tracking (τ model)

If `tau_xy` and/or `tau_z` are provided, the handler tracks the commanded velocity with a first-order response before applying acceleration saturation.
Conceptually:

- Desired acceleration: $a^* = (v_{des} - v) / \tau$
- Apply bounds: $\|a^*_{xy}\| \le \text{max\_acc\_xy}$ and $|a^*_z| \le \text{max\_acc\_z}$
- Euler update: $v \leftarrow v + a^*\,dt$

This is useful when you want a more “quadrotor-like” transient response (smooth exponential-like tracking) without simulating full attitude/thrust dynamics.

Important detail: horizontal (xy) limits are applied to the **norm** of the (x,y) vector, while vertical (z) is applied to the **absolute value** of the z component.
So, even with equal limits configured, the x/y components can appear numerically smaller than z during combined motion.

## Design Philosophy

### No Waypoint Semantics

Unlike traditional mobility handlers, this implementation:

- **Does not interpret waypoints** — Only velocity commands
- **Does not detect arrival** — No concept of "reaching" a destination
- **Does not stop automatically** — Node continues moving until commanded to stop

To stop a node, explicitly command zero velocity:

```python
handler.set_velocity(node_id, (0.0, 0.0, 0.0))
```

### Persistent Velocity Commands

Velocity commands persist until updated. If you command a velocity once, the node will continue moving at that velocity indefinitely (subject to constraints).

This enables:
- Event-driven control updates
- Reduced communication overhead
- Natural integration with feedback controllers

## API Reference

### VelocityMobilityConfiguration

Configuration dataclass for the handler.

**Parameters:**
- `update_rate` (float) — Time between updates in seconds
- `max_speed_xy` (float) — Maximum horizontal speed in m/s
- `max_speed_z` (float) — Maximum vertical speed in m/s
- `max_acc_xy` (float) — Maximum horizontal acceleration in m/s²
- `max_acc_z` (float) — Maximum vertical acceleration in m/s²
- `tau_xy` (float | None) — Optional horizontal velocity tracking time constant (seconds). Must be > 0 if set.
- `tau_z` (float | None) — Optional vertical velocity tracking time constant (seconds). Must be > 0 if set.
- `send_telemetry` (bool) — Enable telemetry broadcasts (default: True)
- `telemetry_decimation` (int) — Emit telemetry every N updates (default: 1)

### VelocityMobilityHandler

Main handler class implementing `INodeHandler`.

**Methods:**

#### `set_velocity(node_id: int, v_des: tuple[float, float, float]) -> None`

Command a node to move with desired velocity.

**Parameters:**
- `node_id` — Identifier of the node to control
- `v_des` — Desired velocity as `(vx, vy, vz)` in m/s

**Example:**
```python
# Move northeast at 5 m/s, ascending at 2 m/s
handler.set_velocity(node_id, (3.54, 3.54, 2.0))

# Stop the node
handler.set_velocity(node_id, (0.0, 0.0, 0.0))
```

## Use Cases

### Swarm Encirclement

Perfect for implementing distributed encirclement algorithms where each agent computes a desired velocity based on neighbors' positions.

### Reactive Navigation

Ideal for obstacle avoidance and reactive behaviors where velocity commands are generated in real-time.

### Formation Control

Suitable for formation control algorithms that output velocity vectors rather than target positions.

### Coverage and Exploration

Works well with coverage control and exploration strategies that produce continuous velocity commands.

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

## License

MIT License

## Acknowledgments

Built for the [GrADyS-SIM NG](https://github.com/Project-GrADyS/gradys-sim-nextgen) simulator, a next-generation framework for simulating ground-aerial networks and distributed systems.

## Citation

If you use this handler in your research, please cite the GrADyS-SIM NG project.

---

**Author:** Laércio Lucchesi (GrADyS-SIM NG contributors)  
**Date:** December 29, 2025
