# Soliton Encirclement

Soliton-inspired swarm encirclement experiment for the
[GrADyS-SIM NG](https://github.com/Project-GrADyS/gradys-sim-nextgen) simulator.

The main focus of this repository is the end-to-end encirclement simulation:

- [main.py](main.py): simulation builder (handlers + nodes)
- [protocol_agent.py](protocol_agent.py): distributed agent controller + failure injection
- [protocol_target.py](protocol_target.py): target state broadcast + optional target motion + global error metrics
- [config_param.py](config_param.py): centralized configuration knobs

This repository also contains a reusable velocity-driven mobility handler in `src/velocity_mobility`, used by the simulation to apply speed/acceleration limits to commanded velocities.

## Quick Start

### Install (Windows + PowerShell)

```powershell
python -m venv .venv
\.venv\Scripts\Activate.ps1
python -m pip install -U pip setuptools wheel
python -m pip install -e .
```

If PowerShell blocks activation scripts:

```powershell
Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned
```

### Run the encirclement simulation

```powershell
python main.py
```

Most parameters are in [config_param.py](config_param.py) (simulation duration, number of agents, desired radius, controller gains, failure injection, target motion).

## Outputs

The simulation writes files next to where you run it:

- `agent_telemetry.csv` (written by agents on `finish()`)
    - Columns: `node_id,timestamp,u,velocity_norm`
- `target_telemetry.csv` (written by the target on `finish()`)
    - Columns: `timestamp,global_radial_error,global_tangential_error`
- `target_telemetry_radial_error.png`, `target_telemetry_tangential_error.png`
    - Generated at the end of the simulation if `matplotlib` is available.

### Plotting agent control outputs (plot_telemetry.py)

To inspect how the control output evolves over time (the soliton internal state `u` and the commanded speed magnitude `||v||`), run:

```powershell
python plot_telemetry.py
```

This script reads `agent_telemetry.csv` and creates **one figure per `node_id`** with two subplots:

- `u` vs time
- `velocity_norm` (i.e., `||v||`) vs time

These figures are shown interactively via `matplotlib.pyplot.show()` and are **not saved as PNGs**, because the number of generated figures can be large and depends on the number of agent nodes in the simulation.

### Visualizing the soft limiter nonlinearity (plot_limiter_soft.py)

If you enabled the soft limiter for the soliton state update (`USE_SOFT_LIMITER_U=True`), you can visualize the corresponding nonlinearity with:

```powershell
python plot_limiter_soft.py
```

## How the simulation is built (main.py)

[main.py](main.py) configures:

- `CommunicationHandler` with `CommunicationMedium` (range, delay, packet loss)
- `TimerHandler` (protocol timers)
- `VelocityMobilityHandler` (applies speed/acceleration limits to commanded velocities)
- `VisualizationHandler` (WebSocket visualization)

It also creates two shared CSV files and passes their paths to protocols via env vars:

- `AGENT_LOG_CSV_PATH` → agents append agent telemetry
- `TARGET_LOG_CSV_PATH` → the target appends global error telemetry

Finally it adds:

- 1 target node at the origin (running `TargetProtocol`)
- `NUM_AGENTS` agent nodes around the desired radius (running `AgentProtocol`)

## Agent protocol (protocol_agent.py)

Each agent runs a periodic control loop (timer `CONTROL_LOOP_TIMER_STR`) that:

1) Broadcasts its own `AgentState` (position, velocity, soliton state `u`).
2) Selects two neighbors (predecessor/successor) around the target using locally cached states, with:
     - timeouts (`AGENT_STATE_TIMEOUT`, `TARGET_STATE_TIMEOUT`)
     - optional pruning (`PRUNE_EXPIRED_STATES`)
     - neighbor switching hysteresis (`HYSTERESIS_RAD`)
3) Computes a radial velocity correction (PD-like, relative to the moving target).
4) Updates the tangential “soliton” state and converts it into a tangential velocity.
5) Composes the final command and sends it to the mobility handler.

### Radial controller

Let $p$ be the agent position, $p_T$ the target position, and $R$ be `ENCIRCLEMENT_RADIUS`.
Define the horizontal distance $r = \|p_{xy} - p_{T,xy}\|$ and radial error $e = r - R$.

The controller estimates relative radial speed $v_r$ (w.r.t. target velocity) and applies:

$$v_{r,\text{corr}} = -K_R\,e - K_{DR}\,v_r$$

This is converted to a 2D vector along the target-centric radial direction and used as `v_rad`.

### Neighbor selection and spacing error

Agents sort neighbors by target-centric angle and pick predecessor/successor in this ring.
The local spacing imbalance error uses only the two local gaps (in radians):

$$e_\tau = \frac{\Delta\theta_{succ} - \Delta\theta_{pred}}{\Delta\theta_{succ} + \Delta\theta_{pred}}$$

If gaps are missing/degenerate, `e_tau = 0`.

### Tangential “soliton” dynamics

Each agent maintains a scalar state $u$ that evolves as:

$$u_{k+1} = u_k + dt\Big(C_{coupling}(u_{succ} - u_{pred}) - \beta u_k - \alpha\,g(u_k) + K_{E\tau}\,e_{\tau,eff}\Big)$$

where:

- $dt$ is `CONTROL_PERIOD`
- $g(u)=u^3$ (or a soft-limited variant when `USE_SOFT_LIMITER_U=True`)
- $e_{\tau,eff} = e_\tau$ by default
- optionally, when `K_OMEGA_DAMP > 0`, a purely local angular-rate damping term is used:
    $$e_{\tau,eff} = e_\tau - K_{\omega}\,(\omega_{self} - \omega_{ref})$$

Then $u$ is converted into a tangential velocity in the XY plane:

$$v_{\tau} = (K_{\tau}\,u\,s)\,\hat t$$

where $\hat t$ is the tangential unit direction around the target and $s$ is a radius-based scale factor:

$$s = \frac{r}{R}\ (\text{best-effort; fallback } s=1)$$

### Final commanded velocity

The final command is:

$$v_{cmd} = v_{rad} + v_{\tau} + v_T$$

and is clamped to mobility limits (`VM_MAX_SPEED_XY`, `VM_MAX_SPEED_Z`) before being sent via `VelocityMobilityHandler.set_velocity(...)`.

## Failure injection (protocol_agent.py)

When enabled (`FAILURE_ENABLE=True`), each agent schedules a periodic failure-check timer (`FAILURE_CHECK_TIMER_STR`).
Every `FAILURE_CHECK_PERIOD` seconds it draws a Bernoulli trial with probability derived from a mean rate (failures/min):

$$p = 1 - \exp(-\lambda\,dt),\quad \lambda = \frac{\text{FAILURE\_MEAN\_FAILURES\_PER\_MIN}}{60}$$

On failure:

- agent enters `_failed=True`
- (best-effort) node is painted red in the visualization
- the main control-loop timer is cancelled
- commanded velocity is set to zero
- a recovery timer is scheduled for `FAILURE_OFF_TIME`

During failure the agent ignores packets and does not run control updates.
On recovery the node is painted blue again and normal timers are rescheduled.

## Target protocol (protocol_target.py)

The target periodically broadcasts `TargetState` (position, velocity) every `TARGET_STATE_BROADCAST_PERIOD`.

If a mobility handler exists, it can also move in the XY plane:

- every `TARGET_MOTION_PERIOD` the target chooses a new velocity direction
- if it is outside `TARGET_MOTION_BOUNDARY_XY`, it steers back toward the origin

### Global error metrics (RMS)

The target maintains a cache of recent `AgentState` messages and (optionally) prunes expired entries.
It computes two global errors (RMS) over currently alive agents/gaps:

- Global radial error (RMS):
    $$e_{r,j} = \frac{\|p_j - p_T\|}{R} - 1,\qquad E_r = \sqrt{\frac{1}{M}\sum_j e_{r,j}^2}$$
    where $M$ is the number of valid alive agent terms.

- Global tangential error (RMS):
    - sort alive agents by target-centric angle
    - compute angular gaps $\Delta\theta_i$ on the circle
    - with ideal gap $\Delta\theta^* = 2\pi/N$ for the current number of alive agents $N$:
        $$e_{\tau,i} = \frac{\Delta\theta_i}{\Delta\theta^*} - 1,\qquad E_\tau = \sqrt{\frac{1}{N}\sum_i e_{\tau,i}^2}$$

## Parameters

All project parameters are centralized in [config_param.py](config_param.py). The most commonly adjusted knobs are:

- **Simulation:** `SIM_DURATION`, `SIM_REAL_TIME`, `CONTROL_PERIOD`
- **Swarm geometry:** `NUM_AGENTS`, `ENCIRCLEMENT_RADIUS`
- **Communication:** `COMMUNICATION_TRANSMISSION_RANGE`, `COMMUNICATION_DELAY`, `COMMUNICATION_FAILURE_RATE`
- **Mobility limits:** `VM_MAX_SPEED_XY`, `VM_MAX_SPEED_Z`, `VM_MAX_ACC_XY`, `VM_MAX_ACC_Z`, `VM_TAU_XY`, `VM_TAU_Z`
- **Radial control:** `K_R`, `K_DR`
- **Tangential control:** `K_TAU`, `BETA_U`, `ALPHA_U`, `C_COUPLING`, `K_E_TAU`, `K_OMEGA_DAMP`
- **Failure injection:** `FAILURE_ENABLE`, `FAILURE_CHECK_PERIOD`, `FAILURE_MEAN_FAILURES_PER_MIN`, `FAILURE_OFF_TIME`
- **Liveness:** `AGENT_STATE_TIMEOUT`, `TARGET_STATE_TIMEOUT`, `HYSTERESIS_RAD`, `PRUNE_EXPIRED_STATES`

## velocity_mobility (brief)

The encirclement controllers output desired velocities; `velocity_mobility` is the mobility layer that applies speed/acceleration limits and integrates positions.
If you want to look at it in isolation, there is a small demo:

```powershell
python -m demos.velocity_mobility.main
```

## Testing

```powershell
python -m pytest -q
```

This separation allows the mathematical core to be tested independently and potentially reused in other contexts.

## Examples

### Constant Velocity Motion

```bash
python .\examples\ex_constant_velocity.py
```

This is a **core-only** demo: it uses only the pure functions in `velocity_mobility.core` (no GrADyS-SIM NG runtime required).
For the main, end-to-end example (simulation builder + handler + protocol + visualization), use `main.py` and `protocol.py`.

## Testing

Run the test suite with pytest:

```bash
# Run all tests
python -m pytest

# Run with coverage
python -m pytest --cov=velocity_mobility --cov-report=html

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

**Author:** Laércio Lucchesi  
**Date:** January 06, 2026
