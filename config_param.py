"""Centralized parameter/config constants for the project.

This module is intended to be the single source of truth for shared
configuration parameters used across protocols and other components.
"""

# --------------------------------------------------------------------------------------
# 1) Simulation framework (timing + simulator timers)
# --------------------------------------------------------------------------------------

# Base control loop period (seconds)
CONTROL_PERIOD: float = 0.01

# TargetState broadcast period (seconds). Keep equal to control loop by default.
TARGET_STATE_BROADCAST_PERIOD: float = CONTROL_PERIOD

# Timer IDs (string keys used by the simulator)
CONTROL_LOOP_TIMER_STR: str = "control_loop_timer"
TARGET_STATE_BROADCAST_TIMER_STR: str = "broadcast_timer"

# Simulation defaults (used by main simulation entrypoints)
SIM_DURATION: float = 600           # Simulation duration (seconds)
SIM_REAL_TIME: bool = False          # Run in real time
SIM_DEBUG: bool = False             # Enable simulator debug mode

# --------------------------------------------------------------------------------------
# 2) Communication + visualization (medium + UI)
# --------------------------------------------------------------------------------------

# Communication medium defaults
COMMUNICATION_TRANSMISSION_RANGE: float = 200  # Communication range (meters)
COMMUNICATION_DELAY: float = 0.0               # Communication delay (seconds)
COMMUNICATION_FAILURE_RATE: float = 0.0        # Packet loss probability [0.0, 1.0]

# Visualization defaults (used by main simulation entrypoints)
VIS_OPEN_BROWSER: bool = True       # Open the visualization in a browser
VIS_UPDATE_RATE: float = 0.1        # Visualization update period (seconds)

# --------------------------------------------------------------------------------------
# 3) Engine node / mobility model (VelocityMobility)
# --------------------------------------------------------------------------------------

# Velocity mobility default parameters (used by main simulation entrypoints)
VM_UPDATE_RATE: float = 0.01        # Update every 0.01 seconds
VM_MAX_SPEED_XY: float = 10.0       # Max horizontal speed: 10 m/s
VM_MAX_SPEED_Z: float = 5.0         # Max vertical speed: 5 m/s
VM_MAX_ACC_XY: float = 4.0          # Max horizontal acceleration: 4.0 m/s²
VM_MAX_ACC_Z: float = 5.0           # Max vertical acceleration: 5.0 m/s²
VM_TAU_XY: float = 1.0              # Optional: 1st-order horizontal tracking time constant (s)
VM_TAU_Z: float = 1.2               # Optional: 1st-order vertical tracking time constant (s)
VM_SEND_TELEMETRY: bool = True      # Enable telemetry
VM_TELEMETRY_DECIMATION: int = 1    # Send telemetry every update

# --------------------------------------------------------------------------------------
# 4) Target motion (optional)
# --------------------------------------------------------------------------------------

# Move the target with a constant speed setpoint in the XY plane,
# changing direction randomly at a fixed period.
TARGET_MOTION_TIMER_STR: str = "target_motion_timer"
TARGET_MOTION_PERIOD: float = 1.0        # change velocity direction every this many seconds
TARGET_MOTION_SPEED_XY: float = 5.0      # target speed (m/s)
TARGET_MOTION_BOUNDARY_XY: float = 30.0  # meters; if |x| or |y| exceeds this, steer back to (0,0)

# --------------------------------------------------------------------------------------
# 5) Failure injection (agent outages)
# --------------------------------------------------------------------------------------

# Failure injection: simulate agent node outages by cancelling timers for a while.
#
# Interpretation:
# - Every FAILURE_CHECK_PERIOD seconds, each agent draws a random trial whose probability is
#   computed from FAILURE_MEAN_FAILURES_PER_MIN (Poisson-like approximation).
# - If it "fails", it goes OFF for FAILURE_OFF_TIME seconds.
# - While OFF, the agent ignores packets and does not reschedule its main timers.
#
# Note: in this project the target never fails; only agents can.
FAILURE_CHECK_TIMER_STR: str = "failure_check_timer"
FAILURE_RECOVER_TIMER_STR: str = "failure_recover_timer"
FAILURE_ENABLE: bool = True           # Whether to enable failure injection
FAILURE_CHECK_PERIOD: float = 0.1     # seconds
FAILURE_MEAN_FAILURES_PER_MIN: float = 1.0  # mean failures per minute
FAILURE_OFF_TIME: float = 8.0         # seconds
FAILURE_RANDOM_SEED = None            # set to an int for reproducibility

# --------------------------------------------------------------------------------------
# 6) Failure detection and liveness (timeouts, neighbor selection)
# --------------------------------------------------------------------------------------

# Protocol liveness / neighbor selection tuning
# Note: these values are local (not transmitted) and are expressed in seconds/radians.
# Timeout guards are scaled with CONTROL_PERIOD.
AGENT_STATE_TIMEOUT: float = 3.0 * CONTROL_PERIOD    # AgentState liveness timeout (s)
TARGET_STATE_TIMEOUT: float = 5.0 * CONTROL_PERIOD   # TargetState liveness timeout (s)
HYSTERESIS_RAD: float = 0.05                         # Neighbor switching hysteresis (rad)

# Optional housekeeping: prune expired cached states to avoid unbounded growth.
PRUNE_EXPIRED_STATES: bool = True

# --------------------------------------------------------------------------------------
# 7) Swarm formation structure
# --------------------------------------------------------------------------------------

# Swarm/encirclement defaults
NUM_AGENTS: int = 10                # Number of agent nodes
ENCIRCLEMENT_RADIUS: float = 25.0   # Desired encirclement radius in meters

# Minimum effective radius used by the tangential mapping to avoid division by
# near-zero radii and to keep the angular-rate interpretation well-conditioned.
# This does NOT change the desired encirclement radius; it only bounds r_eff.
R_MIN: float = 1.0

# --------------------------------------------------------------------------------------
# 8) Radial Controller (Proportional & Derivative terms)
# --------------------------------------------------------------------------------------

# Encirclement control gains (radial controller)
# The controller outputs a *radial* velocity correction (m/s) in the horizontal plane.
# - K_R scales radial distance error (m) into a velocity correction (m/s): units ~ 1/s.
# - K_DR damps radial motion using relative radial speed (m/s): units ~ dimensionless.
K_R: float = 1.0
K_DR: float = 0.5

# --------------------------------------------------------------------------------------
# 9) Tangential Controller (Soliton-like dynamics)
# --------------------------------------------------------------------------------------

# Soliton-like tangential controller parameters
#
#   1) First we update the internal scalar state u ("soliton" state):
#
#      u_next = u + dt * (
#          C_COUPLING * (u_succ - u_pred)
#          - BETA_U * u
#          - ALPHA_U * u^3
#          + K_E_TAU * e_tau_eff
#      )
#
#      where:
#        - gap_pred, gap_succ are the target-centric angular gaps (radians) to the
#          predecessor and successor, wrapped to (0, 2*pi)
#        - e_tau is the local spacing imbalance error (normalized):
#            e_tau = (gap_succ - gap_pred) / (gap_succ + gap_pred)
#          (returns 0.0 when gaps are missing/degenerate)
#        - e_tau_eff is either e_tau or (optionally) a damped version:
#            e_tau_eff = e_tau - K_OMEGA_DAMP * (omega_self - omega_ref)
#
#   2) Then we convert u into a tangential velocity vector in the XY plane:
#
#      v_tau_vec = (K_TAU * u * r_eff) * t_hat
#
#      where t_hat is the unit tangential direction around the target and
#      r_eff = max(r_xy, R_MIN). This yields an induced angular rate
#      omega ≈ v_tau / r ≈ K_TAU * u (for r > R_MIN), independent of ENCIRCLEMENT_RADIUS.

# Optional alternative to the cubic containment term (-ALPHA_U*u^3):
#
# Soft limiter (recommended p=2):
#   g(u) = u^3 / (1 + (|u|/U_S)^2)
#
# This matches u^3 near u=0, but becomes ~U_S^2*u for |u| >> |U_S|.
USE_SOFT_LIMITER_U: bool = False  # Whether to use soft limiter on u update
U_S: float = 2.0                  # Soft limiter scale (only used if USE_SOFT_LIMITER_U is True)

# Tangential controller gains
K_TAU: float = 0.2        # tangential control gain
BETA_U: float = 7.0       # linear damping coefficient
ALPHA_U: float = 0.5      # nonlinear amplitude containment
C_COUPLING: float = 2.0   # antisymmetric coupling strength
K_E_TAU: float = 25.0     # spacing error injection gain (e_tau multiplier)

# Optional local angular-rate damping (no global information required).
# Use e_tau_eff instead of e_tau in the u update above.
# When enabled, the agent forms an omega reference from its two neighbors and
# subtracts a proportional term from the spacing error injection:
#   e_tau_eff = e_tau - K_OMEGA_DAMP * (omega_self - omega_ref)
K_OMEGA_DAMP: float = 0.1  # angular-rate damping gain (0.0 to disable)
