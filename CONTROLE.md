# Radial and tangential control laws (soliton_encirclement)

This document provides an IEEE-style, implementation-faithful description of the **radial** and **tangential** control laws executed by each agent to achieve encirclement around a moving target.

The equations below correspond directly to the agent control loop implemented in `protocol_agent.py`, with parameters defined in `config_param.py`.

---

## I. Notation

All quantities are considered in the horizontal plane (XY), unless otherwise stated.

- Agent position: $p_i = (x_i, y_i)$
- Agent velocity: $v_i = (v_{x,i}, v_{y,i})$
- Target position: $p_T = (x_T, y_T)$
- Target velocity: $v_T = (v_{x,T}, v_{y,T})$

Define the target-centric radial vector:

$$
\mathbf{r}_i = p_i - p_T, \qquad r_i = \|\mathbf{r}_i\|.
$$

Define unit vectors:

$$
\hat{e}_{r,i} = \frac{\mathbf{r}_i}{r_i}, \qquad \hat{e}_{\tau,i} = (-\hat{e}_{r,i,y},\ \hat{e}_{r,i,x}).
$$

In the code, $\hat{e}_{\tau,i}$ is `t_hat`.

Key parameters:

- Desired encirclement radius: $R$ (`ENCIRCLEMENT_RADIUS`)
- Control period: $\Delta t$ (`CONTROL_PERIOD`)
- Radial gains: `K_R`, `K_DR`
- Soliton-like tangential dynamics/gains: `K_TAU`, `K_E_TAU`, `BETA_U`, `ALPHA_U`, `C_COUPLING`
- Optional local angular-rate damping: `K_OMEGA_DAMP`

**Distributed-design constraint.** The tangential law is formulated to avoid any dependence on the global agent count $N$. Under limited communication range and/or packet loss, an agent may not observe the entire formation; therefore, normalizations requiring global $N$ are explicitly avoided.

---

## II. Radial control

The radial objective is $r_i \to R$.

### A. Radial error and relative radial speed

Radial distance error (m):

$$
e_r = r_i - R.
$$

Relative velocity with respect to the target:

$$
v_{rel} = v_i - v_T.
$$

Relative radial speed:

$$
v_r = v_{rel} \cdot \hat{e}_{r,i}.
$$

### B. PD-like radial law

Scalar radial correction (m/s):

$$
v_{r,\,corr} = -K_R\,e_r - K_{DR}\,v_r.
$$

Radial velocity contribution:

$$
v_{rad} = v_{r,\,corr}\,\hat{e}_{r,i}.
$$

---

## III. Tangential control

The tangential objective is to reduce local angular-spacing imbalance around the target. The implementation uses an internal scalar state $u_i$ (the soliton-like state), which is updated each tick and then mapped to a tangential velocity.

### A. Neighbor selection and angular gaps

At each control tick, the agent selects two neighbors around the target using locally cached states:

- predecessor (`pred`) and successor (`succ`)
- angular gaps `pred_gap` and `succ_gap` (wrapped to $[0,2\pi)$)

This procedure is **local** and may operate under partial visibility.

### B. Normalized local spacing error (no global $N$)

The controller uses a dimensionless local imbalance error. In its most general form it supports
**arbitrary (non-uniform) spacing** by weighting the two local gaps with per-arc coefficients.

$$
e_{\tau} = \frac{\lambda_{pred}\,\text{succ\_gap} - \lambda_{self}\,\text{pred\_gap}}{\lambda_{pred}\,\text{succ\_gap} + \lambda_{self}\,\text{pred\_gap}}.
$$

For positive gaps, $e_{\tau} \in [-1,1]$ and $e_{\tau}=0$ corresponds to local balance.

Uniform spacing is recovered by setting $\lambda_{pred}=\lambda_{self}=1$.

**Implementation note (mapping of $\lambda$):** the target may broadcast a map `alive_lambdas` (carried inside `TargetState`) where each value $\lambda_j$ is associated with the arc $(j \to succ(j))$. Agent $i$ then uses:

- $\lambda_{pred}$ = $\lambda_{i^-}$ (predecessor's arc)
- $\lambda_{self}$ = $\lambda_i$ (its own successor arc)

Missing entries default to 1.0.

### C. Optional local damping via angular rate

To increase robustness and reduce oscillations, an optional damping term can be applied using the instantaneous angular rate (no history required).

1) Relative tangential speed (scalar):

$$
v_{\tau,rel} = (v_i - v_T) \cdot \hat{e}_{\tau,i}.
$$

2) Angular rate estimate (rad/s):

$$
\omega_i = \frac{v_{\tau,rel}}{r_i}.
$$

3) Local reference from neighbors:

$$
\omega_{ref} = \tfrac{1}{2}(\omega_{pred} + \omega_{succ}),
$$

with the practical implementation using the available neighbor(s) when one side is missing.

4) Effective spacing injection with damping:

$$
e_{\tau,eff} = e_{\tau} - K_{\omega}(\omega_i - \omega_{ref}),
$$

where $K_{\omega}$ is `K_OMEGA_DAMP`. Setting `K_OMEGA_DAMP = 0` disables damping.

### D. Soliton-like internal dynamics

The internal state is updated via explicit Euler integration:

$$
u_i(t+\Delta t) = u_i(t) + \Delta t\Big[ C(u_{succ}-u_{pred}) - \beta u_i(t) - \alpha g(u_i(t)) + K_e\,e_{\tau,eff} \Big].
$$

Parameter mapping:

- $C$ → `C_COUPLING`
- $\beta$ → `BETA_U`
- $\alpha$ → `ALPHA_U`
- $K_e$ → `K_E_TAU`

Nonlinearity $g(\cdot)$:

- Default: $g(u)=u^3$.
- Optional soft limiter (if `USE_SOFT_LIMITER_U=True`):

$$
g(u) = \frac{u^3}{1+(|u|/U_lim)^2}.
$$

### E. Mapping to tangential velocity

The tangential speed magnitude is proportional to $u_i$ and scaled by an effective radius:

$$
r_{eff} = \max(r_i, R_{min}), \qquad v_{\tau,mag} = K_{\tau}\,u_i\,r_{eff}.
$$

This yields an induced angular rate $\omega \approx v_{\tau}/r_i \approx K_{\tau} u_i$ for $r_i > R_{min}$, independent of $R$.

The tangential velocity contribution is:

$$
v_{\tau} = v_{\tau,mag}\,\hat{e}_{\tau,i}.
$$

---

## IV. Final commanded velocity and saturation

The final commanded velocity (in world coordinates) is composed as:

$$
v_{cmd} = v_T + v_{rad} + v_{\tau}.
$$

The implementation additionally applies a safety saturation/clamp to respect `VM_MAX_SPEED_XY` and `VM_MAX_SPEED_Z`.

---

## V. Compact pseudo-code (single-tick law)

```text
v_cmd = sat_V( v_target + v_rad + v_tau )

e_r   = r - R
e_r̂   = r_vec / r
v_r   = dot(v_i - v_target, e_r̂)
v_rad = [-(K_R*e_r + K_DR*v_r)] * e_r̂

e_tau    = (succ_gap - pred_gap) / (succ_gap + pred_gap)
omega    = dot((v_i - v_target), t_hat) / r
omega_ref= mean(omega_pred, omega_succ)   # uses available neighbor(s)
e_eff    = e_tau - K_OMEGA_DAMP*(omega - omega_ref)

u_next = u + dt * ( C_COUPLING*(u_succ - u_pred) - BETA_U*u - ALPHA_U*g(u) + K_E_TAU*e_eff )
v_tau  = (K_TAU * u_next * (r/R)) * t_hat
```

---

## VI. Parameter tuning guidelines (IEEE-style practical procedure)

This section provides a recommended tuning workflow emphasizing: (i) tangential performance and (ii) robustness under changes in network connectivity, neighbor switching, and agent count—without using global $N$.

### A. Minimal instrumentation

For each run, monitor jointly:

- `target_telemetry_radial_error.png` and `target_telemetry_tangential_error.png` (convergence rate, overshoot, oscillatory envelopes).
- `agent_telemetry.csv` (in particular `u` and `velocity_norm`, to detect saturation and internal dynamics).

Persistent saturation (`velocity_norm` near `VM_MAX_SPEED_XY` for long intervals) indicates that the effective closed-loop behavior is dominated by the actuator limit; tuning should then be interpreted through the lens of saturation.

### B. Tune radial first

1) Increase `K_R` to obtain fast radial convergence.

2) Increase `K_DR` to reduce radial overshoot and oscillations.

Practical criterion: after a short transient, radial error should remain small and well damped; otherwise, tangential behavior may be polluted by persistent variations in $r$.

### C. Tune tangential in two stages

The tangential chain is:

`e_tau` → (`K_E_TAU`) → `u` dynamics → (`K_TAU`) → `v_tau`.

**Stage 1 — tune `K_E_TAU` (injection into u):**

- Increase `K_E_TAU` until `u` becomes responsive and the tangential error starts decreasing promptly.
- If `K_E_TAU` is too large, `u` typically exhibits sign-flipping and oscillations (and may drive saturation).

**Stage 2 — tune `K_TAU` (u-to-velocity mapping):**

- Increase `K_TAU` until tangential action is effective, avoiding persistent saturation.
- If `K_TAU` is too large, the actuator limit can induce oscillations and degrade steady-state accuracy.

### D. Oscillation control via `BETA_U` and `C_COUPLING`

- `BETA_U` (linear damping): increasing `BETA_U` reduces oscillations/overshoot in `u` at the cost of slower tangential response.
- `C_COUPLING` (neighbor coupling): increasing `C_COUPLING` tends to strengthen wave-like propagation along the ring, potentially improving redistribution speed but often increasing oscillations—especially as the number of agents grows.

If oscillations appear (commonly when increasing the agent count), a robust adjustment is:

1) Increase `BETA_U` moderately (e.g., +10% to +30%).
2) If oscillations persist, reduce `C_COUPLING` (e.g., −10% to −30%).

Numerical note: since the $u$ update is explicit Euler with step `CONTROL_PERIOD`, excessively large `C_COUPLING` can reduce stability margins as the network supports more spatial modes and as neighbor switching introduces high-frequency excitation.

### E. Optional damping via `K_OMEGA_DAMP`

`K_OMEGA_DAMP` adds local angular-rate damping without global information. Recommended procedure:

1) Start with `K_OMEGA_DAMP = 0` (disabled).
2) Increase gradually until oscillatory envelopes reduce noticeably.
3) If tangential convergence becomes too slow, compensate with small adjustments of `K_E_TAU` and/or `K_TAU`, while checking for saturation.

### F. Robustness under partial visibility

Under limited range, agents may operate with incomplete neighbor information. In this case, the local error $e_{\tau}$ promotes **local balance** (front/back gaps comparable), while global convergence depends on the effective connectivity and information flow along the network.

Consequently, a tuning that is stable for a smaller formation may become oscillatory for a larger one. Without using global $N$, robustness is typically improved by:

- increasing damping (`BETA_U` and/or `K_OMEGA_DAMP`),
- reducing wave-like coupling (`C_COUPLING`), and
- when feasible, decreasing `CONTROL_PERIOD` (smaller integration step increases stability margin).
