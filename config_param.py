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

# AdversaryState broadcast period (seconds). Keep equal to control loop by default.
ADVERSARY_STATE_BROADCAST_PERIOD: float = CONTROL_PERIOD

# Timer IDs (string keys used by the simulator)
CONTROL_LOOP_TIMER_STR: str = "control_loop_timer"
TARGET_STATE_BROADCAST_TIMER_STR: str = "broadcast_timer"

# Adversary timer IDs
ADVERSARY_STATE_BROADCAST_TIMER_STR: str = "adversary_state_broadcast_timer"

# Simulation defaults (used by main simulation entrypoints)
SIM_DURATION: float = 120          # Simulation duration (seconds)
SIM_REAL_TIME: bool = True          # Run in real time
SIM_DEBUG: bool = False             # Enable simulator debug mode

# --------------------------------------------------------------------------------------
# Global reproducibility control
# --------------------------------------------------------------------------------------
# If True, all randomness (initialization, target/adversary motion, agent failures, etc.)
# will be seeded deterministically for fully reproducible experiments.
# If False, all random draws will be non-deterministic (true randomness).
EXPERIMENT_REPRODUCIBLE: bool = False

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
# 4a) Target motion (optional)
# --------------------------------------------------------------------------------------

# Move the target with a constant speed setpoint in the XY plane,
# changing direction randomly at a fixed period.
TARGET_MOTION_TIMER_STR: str = "target_motion_timer"
TARGET_MOTION_PERIOD: float = 1.0        # change velocity direction every this many seconds
TARGET_MOTION_SPEED_XY: float = 5.0      # target speed (m/s)
TARGET_MOTION_BOUNDARY_XY: float = 20.0  # meters; if |x| or |y| exceeds this, steer back to (0,0)

# --------------------------------------------------------------------------------------
# 4b) Adversary motion (random roaming)
# --------------------------------------------------------------------------------------

# Adversary random roaming region in XY: [-ADVERSARY_ROAM_BOUND_XY, +ADVERSARY_ROAM_BOUND_XY]
ADVERSARY_ROAM_BOUND_XY: float = 40.0
# Minimum allowed distance between adversary and target in XY (meters)
ADVERSARY_MIN_TARGET_DISTANCE: float = 30.0
# Nominal adversary roaming speed in XY (m/s)
ADVERSARY_ROAM_SPEED_XY: float = 4.0 #4.0

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
ENCIRCLEMENT_RADIUS: float = 20.0   # Desired encirclement radius in meters

# Desired angular velocity for the whole swarm to spin around the target (rad/s).
# This value is broadcast by the target inside TargetState.
TARGET_SWARM_OMEGA_REF: float = 0.0

# Protection angle (degrees): desired protected/covered arc between the two boundary nodes.
#
# The controller uses `lambda` as an arc *weight* (dimensionless): at equilibrium, each
# arc size is proportional to its lambda, and all arcs sum to 360 degrees.
#
# We assign one special arc (edge node -> successor) with weight `edge_lambda`, and the
# other (alive_count-1) arcs with weight 1.0. The complement of the protected arc is the
# "edge gap":
#   edge_gap_deg = 360 - PROTECTION_ANGLE_DEG
#
# For any desired edge_gap_deg in [0, 360), the corresponding edge_lambda is:
#   edge_lambda = edge_gap_deg * (alive_count - 1) / (360 - edge_gap_deg)
#
# Notes:
# - PROTECTION_ANGLE_DEG = 360 means edge_gap_deg = 0 => no boundary arc (uniform lambdas=1).
# - If edge_gap_deg is smaller than the uniform gap (360/alive_count), then edge_lambda < 1 and
#   the boundary arc is the *smallest* gap (the token should track the minimum gap, not maximum).
PROTECTION_ANGLE_DEG: float = 90.0

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
#   g(u) = u^3 / (1 + (|u|/U_lim)^2)
#
# This matches u^3 near u=0, but becomes ~U_lim^2*u for |u| >> |U_lim|.
USE_SOFT_LIMITER_U: bool = False  # Whether to use soft limiter on u update
U_lim: float = 2.0                # Soft limiter scale (only used if USE_SOFT_LIMITER_U is True)

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

# Optional diffusion (discrete Laplacian) on the soliton state u:
#   + KAPPA_U_DIFF * (u_succ - 2u + u_pred)
# Helps damp high-frequency spatial oscillations of u along the ring.
KAPPA_U_DIFF: float = 0.1  # diffusion gain (0.0 to disable)

# Optional additional KdV-like terms (default disabled).
# Steepening / nonlinear transport:
#   - K_U_STEEPEN * u * u_s
# where u_s = (u_succ - u_pred) (central 1-hop; scaling absorbed into the gain).
#K_U_STEEPEN: float = 0.6
K_U_STEEPEN: float = 1.2

# Dispersion (KdV-like) using the 1-hop gradient of curvature:
#   + KAPPA_U_DISP * u_sss
# where u_sss = (u_ss_succ - u_ss_pred) and u_ss is received from 1-hop neighbors.
#KAPPA_U_DISP: float = 0.1
KAPPA_U_DISP: float = 0.2

# --------------------------------------------------------------------------------------
# 10) Spin Controller (Proportional & Derivative terms)
# --------------------------------------------------------------------------------------

# Enable/disable the swarm spin controller that tracks the adversary direction.
# - If False: the target will NOT try to align the swarm spin with the adversary; it will
#   broadcast omega_ref = TARGET_SWARM_OMEGA_REF (typically 0.0 for no spin).
# - If True: omega_ref is generated by the PD controller below (with optional open-loop
#   bias when KP=KD=0).
TARGET_SWARM_SPIN_ENABLE: bool = True

# PD controller for omega_ref generation (based on angular error in radians).
# - If KP=KD=0: omega_ref = TARGET_SWARM_OMEGA_REF (pure open-loop spin)
# - Otherwise:  omega_ref = KP * err + KD * derr (no constant bias)
TARGET_SWARM_OMEGA_PD_KP: float = 1.0
TARGET_SWARM_OMEGA_PD_KD: float = 0.2
TARGET_SWARM_OMEGA_PD_MAX_ABS: float = 1.0

# If the swarm is close to uniformly distributed, the sum of unit vectors
# target->agents has near-zero magnitude and its direction becomes ill-defined.
# This threshold (Kuramoto-like rho in [0,1]) disables the angular error when
# rho is too small, avoiding a fixed-direction bias.
TARGET_SWARM_SPIN_RHO_MIN: float = 0.05