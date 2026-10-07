# Bicycle Gym — Self-Driving Car Control in ROS 2

A ROS 2 project in which a simulated self-driving car (extended kinematic bicycle model) is controlled around a racetrack. The car must stay on the reference path, regulate its speed, and complete laps quickly and accurately. Four steering/speed strategies are implemented and benchmarked: **Lateral PID**, **Pure Pursuit**, **Extended Kinematic MPC**, plus a longitudinal **PID cruise controller** and a **curvature-based velocity profiler**.

---

## 1. Student Information

| Field | Details |
|---|---|
| **Name** | Youssef Cherif Elprince |
| **Phone** | 01202863094 |
| **Email** | youssef.yasca.elprince@gmail.com |

---

## 2. Table of Contents

1. [Student Information](#1-student-information)
2. [System Architecture](#3-system-architecture)
3. [Mathematical Formulations](#4-mathematical-formulations)
4. [Benchmark Results](#5-benchmark-results)
5. [Critical Comparison of the Controllers](#6-critical-comparison-of-the-controllers)
6. [Why MPC Tracks Better than Pure Pursuit and Lateral PID](#7-why-mpc-tracks-better-than-pure-pursuit-and-lateral-pid)
7. [Milestone 6 — Free Exploration Summary (Nav2 MPPI)](#8-milestone-6--free-exploration-summary)
8. [Reproduction Guide](#9-reproduction-guide)

---

## 3. System Architecture

### 3.1 Packages

| Package | Responsibility |
|---|---|
| `bicycle_sim` | Vehicle model (`bicycle_model.py`), simulator node, URDF/Xacro robot description, RViz configuration |
| `bicycle_control` | Teleoperation bridge (`teleop_bridge.py`), longitudinal PID (`longitudinal_pid.py`), velocity profiler, and lateral controllers: `lateral_pid.py`, `pure_pursuit.py`, `mpc.py` |
| `track_environment` | Racetrack CSV loading, path publishing, boundary-cone visualisation, lap analysis (`lap_analyzer.py`) |

### 3.2 Vehicle and track

| Parameter | Value |
|---|---|
| Wheelbase `L` | 1.25 m |
| Track width | 1.18 m |
| Wheel radius | 0.5 m |
| Wheel width | 0.3 m |
| Default track | `track_environment/tracks/centerline_0.csv` (1000 waypoints, closed loop, ≈ 528.2 m perimeter) |
| Cones | `random_track0.csv` (boundary markers, same origin offset as the centerline) |

**Actuator inputs**

- `/throttle` — normalised throttle/brake in `[-1.0, 1.0]`
- `/steer` — front steering angle in rad (positive = left)

**State:** `[x, y, θ, v]` — rear-axle position, yaw, forward speed. Velocity is a *state*, not an input, so powertrain lag and speed/steering coupling are part of the problem.

### 3.3 Signal flow

```
                        ┌──────────────────────┐
  centerline_0.csv ───► │  track_environment   │──► /path, cones (RViz)
                        │  (path publisher)    │
                        └──────────┬───────────┘
                                   │ path
                                   ▼
                        ┌──────────────────────┐
                        │  Velocity profiler   │──► v_target(s)
                        │  (curvature limits)  │
                        └──────────┬───────────┘
                                   │
        ┌──────────────────────────┼───────────────────────────┐
        │                          │                           │
        ▼                          ▼                           ▼
┌───────────────┐        ┌───────────────────┐        ┌────────────────┐
│ Longitudinal  │        │  Lateral control  │        │  Teleop bridge │
│ PID (cruise)  │        │  PID | PP | MPC   │        │ (manual mode)  │
└──────┬────────┘        └─────────┬─────────┘        └───────┬────────┘
       │ /throttle                 │ /steer                   │ throttle/steer
       └───────────────┬───────────┴──────────────────────────┘
                       ▼
              ┌──────────────────┐
              │   bicycle_sim    │  extended kinematic bicycle model
              │ (Euler, 4 states)│  ──► odometry / pose / TF
              └────────┬─────────┘
                       │ state feedback
                       ▼
              ┌──────────────────┐
              │   lap_analyzer   │──► CTE, speed, heading error, lap time,
              │                  │    RViz markers, benchmark log
              └──────────────────┘
```

The three lateral controllers share the same interface (path + vehicle state in, steering angle out), so they can be swapped without touching anything else. This keeps the comparison fair.

### 3.4 Source files

| File | Package | Milestone | Purpose |
|---|---|---|---|
| `bicycle_model.py` | `bicycle_sim` | 2 | Equations of motion, Euler integration, heading wrapping, speed clamping |
| `sim_node.py` | `bicycle_sim` | 1–2 | Simulator ROS node: subscribes to `/throttle` and `/steer`, publishes vehicle state |
| `teleop_bridge.py` | `bicycle_control` | 3 | `Twist` → throttle/steer with 0.5 s watchdog |
| `longitudinal_pid.py` | `bicycle_control` | 4 | Speed PID with anti-windup |
| `velocity_profiler.py` | `bicycle_control` | 5.1 | Curvature and speed limits along the path |
| `lateral_pid.py` | `bicycle_control` | 5.2 | Cross-track + heading feedback |
| `pure_pursuit.py` | `bicycle_control` | 5.3 | Adaptive look-ahead geometric steering |
| `mpc.py` | `bicycle_control` | 5.4 | Frenet-frame constrained MPC |
| `controller_node.py` | `bicycle_control` | 5 | Node that ties the profiler, longitudinal PID and the selected lateral controller together |
| `track.py`, `path_gen.py` | `track_environment` | — | Track CSV loading, closing the loop, path generation and publishing |
| `lap_analyzer.py` | `track_environment` | 5.5 | Lap timing, telemetry topics, RViz dashboard markers |

---|---|---|
| `bicycle_model.py` | 2 | Equations of motion, Euler integration, heading wrapping, speed clamping |
| `teleop_bridge.py` | 3 | `Twist` → throttle/steer with 0.5 s watchdog |
| `longitudinal_pid.py` | 4 | Speed PID with anti-windup |
| velocity profiler | 5.1 | Curvature and speed limits along the path |
| `lateral_pid.py` | 5.2 | Cross-track + heading feedback |
| `pure_pursuit.py` | 5.3 | Adaptive look-ahead geometric steering |
| `mpc.py` | 5.4 | Frenet-frame constrained MPC |
| `lap_analyzer.py` | 5.5 | Lap timing, telemetry topics, RViz dashboard |

---

## 4. Mathematical Formulations

Notation: `L` wheelbase, `δ` front steering angle, `a` longitudinal acceleration, `Δt` simulation step.

### 4.1 Extended kinematic bicycle model (Milestone 2)

Reference point is the rear-axle centre:

```
ẋ = v · cos θ
ẏ = v · sin θ
θ̇ = (v / L) · tan δ
v̇ = a(u_throttle, v)
```

The acceleration includes the powertrain and resistive forces:

```
a = k_thr · u_throttle − c_drag · v · |v| − c_roll · sign(v)
```

where `k_thr` is the throttle gain, `c_drag` the aerodynamic drag coefficient and `c_roll` the rolling/friction term (values are defined in the simulator configuration).

**Forward Euler integration**

```
x_{k+1} = x_k + v_k cos θ_k · Δt
y_{k+1} = y_k + v_k sin θ_k · Δt
θ_{k+1} = wrap( θ_k + (v_k / L) tan δ_k · Δt )
v_{k+1} = clamp( v_k + a_k · Δt, v_min, v_max )
```

Heading wrapping keeps `θ ∈ (−π, π]`:

```
wrap(θ) = atan2( sin θ, cos θ )
```

### 4.2 Teleoperation mapping (Milestone 3)

A `Twist` message with linear velocity `v_cmd` and angular velocity `ω_cmd` is mapped open-loop:

```
u_throttle = clamp( K_v · v_cmd , −1, 1 )
δ          = clamp( atan( L · ω_cmd / v_cmd ) , −δ_max, δ_max )     (v_cmd ≠ 0)
```

A watchdog sets throttle and steering to zero if no `Twist` has been received for **0.5 s**.

### 4.3 Longitudinal PID with anti-windup (Milestone 4)

With `e_v = v_target − v`:

```
u = K_p e_v + K_i ∫ e_v dt + K_d (d e_v / dt)
u_throttle = clamp( u, −1, 1 )
```

**Anti-windup (clamping):** the integral is only accumulated while the output is not saturated, or while the error would drive the output back out of saturation:

```
if  |u| < 1   or   sign(e_v) ≠ sign(u):   I ← I + e_v · Δt
```

The integral term is additionally bounded to `[−I_max, I_max]`. This prevents overshoot after long saturation periods (e.g., accelerating from standstill against drag).

### 4.4 Velocity profiler (Milestone 5.1)

**Path curvature** at waypoint `i`, using the Menger curvature of three consecutive points `P_{i−1}, P_i, P_{i+1}`:

```
κ_i = 4 · Area(P_{i−1}, P_i, P_{i+1}) / ( |P_i − P_{i−1}| · |P_{i+1} − P_i| · |P_{i+1} − P_{i−1}| )
```

(signed by the cross product of the two segment vectors). **Curvature-limited speed**, from a lateral acceleration limit `a_lat,max`:

```
v_lim,i = min( v_max , sqrt( a_lat,max / max(|κ_i|, ε) ) )
```

A backward then forward pass enforces longitudinal acceleration/braking limits:

```
v_i ≤ sqrt( v_{i+1}² + 2 a_brake · Δs_i )     (backward pass)
v_i ≤ sqrt( v_{i−1}² + 2 a_accel · Δs_i )     (forward pass)
```

The result is smoothed and used as `v_target` for the longitudinal PID.

### 4.5 Lateral PID (Milestone 5.2)

Errors are measured relative to the closest point on the path. With path tangent heading `θ_p` and the vector from the path to the car:

```
e_y  = cross-track error (positive when the car is left of the path)
e_ψ  = wrap( θ − θ_p )                      (heading error)
```

Steering law (sign convention: positive δ = left turn, so a car on the left of the path must steer right):

```
δ = −( K_p e_y + K_i ∫ e_y dt + K_d ė_y ) − K_ψ e_ψ
δ = clamp( δ, −δ_max, δ_max )
```

### 4.6 Pure Pursuit (Milestone 5.3)

**Adaptive look-ahead distance**

```
L_d = clamp( L_0 + k_v · v , L_min , L_max )
```

Let `α` be the angle between the vehicle heading and the line to the look-ahead point on the path (in the vehicle frame). The vehicle follows the circular arc through that point:

```
δ = atan( 2 L sin α / L_d )
```

Equivalent arc curvature: `κ = 2 sin α / L_d`. A larger `L_d` gives smoother but corner-cutting behaviour; a smaller `L_d` tracks tightly but oscillates.

### 4.7 Extended kinematic MPC in the Frenet frame (Milestone 5.4)

**Frenet error states** relative to the path (arc length `s`, curvature `κ(s)`):

```
ė_y = v sin e_ψ
ė_ψ = (v / L) tan δ − κ(s) · v cos e_ψ / (1 − κ e_y)
ṡ   = v cos e_ψ / (1 − κ e_y)
v̇   = a
```

State `z = [e_y, e_ψ, v]`, input `u = [a, δ]`. The model is discretised (forward Euler, step `Δt`) and linearised around the reference to give `z_{k+1} = A_k z_k + B_k u_k + c_k`.

**Optimisation problem** over a horizon of `N` steps:

```
min   Σ_{k=0}^{N−1} [ q_y e_{y,k}² + q_ψ e_{ψ,k}² + q_v (v_k − v_ref,k)²
                      + r_a a_k² + r_δ δ_k²
                      + r_Δδ (δ_k − δ_{k−1})² + r_Δa (a_k − a_{k−1})² ]
      + terminal cost  z_N^T P z_N

s.t.  z_{k+1} = f(z_k, u_k)                         (prediction model)
      δ_min ≤ δ_k ≤ δ_max                           (steering limits)
      |δ_k − δ_{k−1}| ≤ Δδ_max                      (steering rate limit)
      a_min ≤ a_k ≤ a_max                           (acceleration limits)
      v_min ≤ v_k ≤ v_max
      |e_{y,k}| ≤ e_{y,max}                         (track boundary, optional)
```

**Receding horizon:** only `u_0*` is applied; the problem is re-solved at the next control step with the horizon shifted forward.

**Warm start:** the previous solution shifted by one step, `[u_1*, …, u_{N−1}*, u_{N−1}*]`, initialises the solver. This reduces solve time and keeps the optimiser near the previous (smooth) solution.

### 4.8 Telemetry and metrics (Milestone 5.5)

For `M` samples over the logged laps:

```
Mean CTE = (1/M) Σ |e_y,i|
Max CTE  = max_i |e_y,i|
RMS CTE  = sqrt( (1/M) Σ e_y,i² )
Heading error (deg) = (180/π) · e_ψ
```

Published quantities: CTE, speed, heading error (deg), lap time. RViz shows the car, path, cones and dashboard markers.

---

## 5. Benchmark Results

> **To be completed.** Results will be added after running at least **three full laps** per controller with the lap analyzer.

| Controller | Laps completed | Best lap time (s) | Top speed (m/s) | Mean CTE (m) | Max CTE (m) | RMS CTE (m) |
|---|---|---|---|---|---|---|
| Lateral PID | | | | | | |
| Pure Pursuit | | | | | | |
| MPC | | | | | | |

*Test conditions: track `centerline_0.csv`, same velocity profile and longitudinal PID for all controllers, same start pose.*

---

## 6. Critical Comparison of the Controllers

| Aspect | Lateral PID | Pure Pursuit | Extended Kinematic MPC |
|---|---|---|---|
| **Principle** | Reactive feedback on `e_y` and `e_ψ` | Geometric arc to a look-ahead point | Constrained optimisation over a prediction horizon |
| **Preview of the road** | None (reacts only after error appears) | Yes, single point at `L_d` | Yes, full horizon with curvature `κ(s)` |
| **Use of vehicle model** | None | Kinematic geometry only | Full prediction model |
| **Actuator limits** | Clamped after the fact | Clamped after the fact | Handled inside the optimisation |
| **Tuning** | `K_p, K_i, K_d, K_ψ`; speed-dependent | Mainly `L_0`, `k_v` | Weights `Q, R`, horizon `N`, `Δt` |
| **Compute cost** | Negligible | Negligible | Highest (solver per step) |
| **Strengths** | Simple, easy to debug, cheap | Smooth, intuitive, robust on moderate curves | Best tracking, anticipates curvature, respects constraints, couples speed and steering |
| **Weaknesses** | Lags in curves, steady-state error in constant-curvature sections without integral action, gains depend on speed | Cuts corners with large `L_d`, oscillates with small `L_d`, no constraint handling | Heavier to implement and tune, needs a solver, performance depends on model accuracy and solve time |
| **Typical failure mode** | Oscillation at high speed; late reaction at corner entry | Corner cutting at high speed | Infeasible/slow solves if horizon or weights are poorly chosen |

**Discussion.**
- *Lateral PID* is the baseline: easy to implement and cheap, but because it only reacts to error that already exists, it turns late into corners. Any gain set tuned for one speed is a compromise at others, since the effect of steering on lateral motion scales with `v`.
- *Pure Pursuit* adds a preview via the look-ahead point, which makes it smoother than PID and a good middle ground. Its single parameter trades responsiveness against smoothness, and the geometric law ignores dynamics and limits.
- *MPC* gives the most accurate and anticipatory tracking because it plans over a horizon with the model and the actual constraints, at the price of computational cost and tuning complexity.

*(Numerical conclusions from the benchmark table will be added once the results are available.)*

---

## 7. Why MPC Tracks Better than Pure Pursuit and Lateral PID

1. **Optimal preview over a horizon.** PID uses no preview at all; Pure Pursuit uses a single geometric look-ahead point. MPC evaluates the *entire* upcoming curvature profile `κ(s)` over `N` steps and chooses the steering sequence that minimises accumulated error. It begins to steer *before* a corner and unwinds *before* it ends, rather than reacting after the error appears.

2. **Explicit use of the vehicle model.** The prediction model `θ̇ = (v/L) tan δ` is embedded in the optimisation, so the controller knows how a steering input will change the future heading and cross-track error at the *current* speed. PID and Pure Pursuit have no such internal model, so a fixed set of gains or `L_d` is only right at a particular speed and curvature.

3. **Constraint handling inside the optimiser.** Steering angle, steering rate and acceleration limits are constraints, so the solution is the best *feasible* one. For PID and Pure Pursuit, saturation is applied afterwards by clipping, which breaks the intended control law (e.g., integrator windup, or an arc the car cannot physically follow).

4. **Trade-off of competing objectives.** The cost function balances `e_y`, `e_ψ`, speed tracking and actuator effort (including steering-rate smoothness) in one principled objective. PID and Pure Pursuit each address essentially one of these and must be tuned heuristically.

5. **Coupling of speed and steering.** Because velocity is a state of the vehicle, the MPC can take into account that the car is still accelerating or braking and that effective steering authority changes with `v`. Separate longitudinal and lateral loops (as in PID/Pure Pursuit) ignore this interaction.

6. **Removal of Pure Pursuit's geometric bias.** Pure Pursuit tracks a point `L_d` ahead rather than the path under the car, so it systematically cuts the inside of curves; a small `L_d` removes this but causes oscillations. MPC minimises error at *every* predicted step, so there is no such trade-off.

7. **Receding horizon feedback.** Re-solving at each step with a warm start provides feedback correction against model mismatch and disturbances while still retaining the benefit of preview.

**Caveat.** MPC's advantage depends on model fidelity and on solving within the control period; with a poor model or a late solution, a simple controller can outperform it.

---

## 8. Milestone 6 — Free Exploration Summary

**Chosen topic: Sampling-based predictive control with Nav2 MPPI.** Milestone 6 offers three options (four-wheel Ackermann kinematics, 3D simulation with Gazebo/MVSim, or Nav2 MPPI). I investigated **Nav2 MPPI** because it is the closest to the controllers built in this project: Lateral PID and Pure Pursuit are reactive/geometric, MPC is optimisation-based, and MPPI is also predictive but replaces the numerical optimiser with random sampling. The full report is `Milestone6_Nav2_MPPI_Report_Styled.docx`. Ackermann kinematics and 2D-vs-3D simulation are summarised briefly in 8.6 and 8.7 as supporting context.

### 8.1 Summary of findings

MPPI is a derivative-free, sampling-based variant of MPC. It simulates a large batch of noisy control sequences through the vehicle model, scores each with a sum of plugin cost functions ("critics"), and blends them with exponential weights. Because the cost never has to be differentiable, costmap-based obstacle avoidance comes naturally. The price is compute (thousands of rollouts per cycle), stochastic output that needs smoothing, and no hard-constraint guarantees. Nav2's implementation (`nav2_mppi_controller`) is CPU-only and vectorised, and supports Ackermann vehicles through a minimum-turning-radius constraint.

### 8.2 From MPC to MPPI

Standard MPC solves a finite-horizon optimal control problem at every step (QP, SQP or interior-point solvers), so the cost and constraints must be smooth, and obstacles usually have to be expressed as convex or linearised constraints. MPPI (Williams et al., ICRA 2016) keeps the receding-horizon structure but solves the optimisation by sampling: the optimal control distribution is approximated by weighting random rollouts by how well they score. No gradients are needed, so any cost that can be evaluated along a trajectory (including an occupancy-costmap lookup) can be used directly.

### 8.3 The MPPI algorithm

Each control cycle in Nav2:

1. **Warm start:** take the best sequence from the previous cycle, shifted forward one step, as the nominal sequence.
2. **Sample:** add Gaussian perturbations to the nominal controls to create a batch of candidate sequences.
3. **Roll out:** forward-simulate every candidate through the motion model from the current state.
4. **Score:** evaluate each trajectory with the critics and sum the costs.
5. **Combine:** weight the perturbations with a soft-max over the costs and add the weighted average to the nominal sequence.
6. **Apply:** send the first control to the vehicle and keep the rest as the next warm start.

With nominal sequence `u`, k-th perturbation `δu_k` and k-th rollout cost `S_k`, the update is:

```
w_k = exp( −(S_k − S_min) / λ ) / η
u  ← u + Σ_k w_k · δu_k
```

`λ` is the temperature and `η` normalises the weights to sum to one (subtracting `S_min` is for numerical stability). A temperature near zero behaves like picking the single best rollout; a very large temperature reduces to a plain average of all samples. The weighted average gives a smooth blend of many good candidates, which can lie in a region none of the individual samples visited exactly.

### 8.4 The Nav2 implementation

- **Architecture:** `nav2_mppi_controller` implements the `nav2_core::Controller` interface and runs as a plugin inside `controller_server`, acting as a local trajectory planner that tracks a global path while avoiding obstacles.
- **Plugin-based critics:** cost terms (e.g., Constraint, Goal, Goal Angle, path-following and costmap obstacle critics) can be swapped or tuned without touching the core, each with a `cost_weight` and `cost_power`.
- **CPU-only, vectorised:** performance comes from vectorisation and tensor operations rather than a GPU. The package README reports 50+ Hz on a modest Intel 4th-generation i5.
- **Fallbacks:** soft failures (no feasible trajectory) trigger retries via `retry_attempt_limit` before recovery behaviours.
- **Motion models:** differential drive, omnidirectional and Ackermann. Sampled controls are body-frame velocities (`vx`, `vy` for holonomic bases, and `wz`) and rollouts use a kinematic model.
- **Ackermann support:** a single geometric limit, `min_turning_r`, restricts how sharply sampled trajectories may curve. This is the same constraint as in this project's bicycle model: the steering limit sets `R_min = L / tan δ_max`. Because MPPI samples velocity and yaw rate rather than steering angle, a downstream layer must convert the commanded curvature into a steering command.

**Key parameters** (package README defaults; they vary slightly between distributions):

| Parameter | Default | Role |
|---|---|---|
| `batch_size` | 1000 | Candidate trajectories sampled per iteration; more samples cover the space better but cost proportionally more compute |
| `time_steps` | 56 | Steps per sampled trajectory |
| `model_dt` | 0.05 s | Time between trajectory points (56 × 0.05 = 2.8 s horizon) |
| `iteration_count` | 1 | MPPI iterations per cycle; keep 1 and use more samples instead |
| `vx_std` / `wz_std` | 0.2 / 0.4 | Std. dev. of the Gaussian sampling noise on linear and angular velocity |
| `temperature` | 0.3 | Selectiveness of the soft-max weighting |
| `gamma` | 0.015 | Trade-off between control smoothness and low control energy |
| `min_turning_r` | 0.2 m* | Ackermann minimum turning radius (*Lyrical default; none listed in earlier READMEs) |

### 8.5 MPPI vs deterministic MPC

| Aspect | Deterministic MPC (this project) | Nav2 MPPI |
|---|---|---|
| **Optimiser** | Gradient-based solver (QP/NLP) on a linearised or smooth problem | Derivative-free: random sampling plus exponential weighting |
| **Cost function** | Must be smooth (ideally convex) for reliable convergence | Any cost evaluable along a trajectory, including non-smooth and costmap lookups |
| **Obstacle handling** | Needs explicit, usually convex or linearised constraints; non-convex obstacles are hard | Obstacles enter as critic costs on sampled trajectories; arbitrary shapes work out of the box |
| **Constraints** | Hard constraints, guaranteed when feasible | Soft: penalties, clipping or the motion model; no hard guarantee |
| **Local minima** | Can get trapped; depends on the initial guess | Random exploration around the warm start helps, but only locally |
| **Compute profile** | Few iterations of a heavier solver; grows with horizon and model size | Many cheap rollouts: ≈ `batch_size × time_steps × critics` per iteration; highly parallel |
| **Output** | Deterministic: same state gives the same control | Stochastic: noise can cause jitter, so smoothing and tuning matter |
| **Tuning** | Weights, horizon, solver tolerances | Critic weights, sampling noise, temperature, batch size |

- **Flexibility:** MPPI is the more flexible. Adding a behaviour means adding a critic rather than reformulating a constrained optimisation problem, which suits cluttered local navigation with a costmap.
- **Obstacle handling:** a sampled trajectory either touches a costly region or it does not, so the cost landscape can be discontinuous and still work. A gradient-based MPC would need smooth constraint approximations.
- **Compute:** with the defaults, one iteration simulates 1000 trajectories of 56 steps (56,000 state updates) before the critics run. This is cheap per sample and parallelises well (hence 50+ Hz on a CPU), but the load scales linearly with batch size. A tuned MPC solver on a small problem can be cheaper per cycle and gives a smoother, more repeatable result.
- **When to choose each:** for tracking a known racetrack line with tight, well-defined limits and a smooth cost, deterministic MPC is the natural fit. For navigation among obstacles, with grid-defined costs or behaviours that are awkward to write as constraints, MPPI is easier to extend.

### 8.6 Relevance to Bicycle Gym and limitations

- The lab's kinematic bicycle model is exactly the kind of model MPPI rolls out; Ackermann MPPI uses a curvature-limited version of it, so the min-radius reasoning from Milestone 2 applies.
- The MPC tracking cost against the reference path would become a path-following critic in MPPI; speed and steering limits would be expressed through sampling bounds and the Constraint critic.
- MPPI relies on a good model and a short, fast cycle, so control-loop timing matters just as much as in the earlier milestones.
- **Stochasticity:** results vary between cycles. Nav2 reduces jitter by reusing a fixed noise distribution (`regenerate_noises` defaults to false) and by tuning `gamma` and the sampling standard deviations.
- **Sample efficiency:** narrow feasible regions (tight corridors, high-speed cornering) need enough samples to land inside them; the remedy is more samples or less noise, both with compute trade-offs.
- **Model fidelity:** rollouts use a kinematic model with no tyre slip; at higher speeds a dynamic model would be needed.
- **Tuning surface:** many interacting knobs (critic weights and powers, noise, temperature); Nav2 ships pre-tuned defaults and newer releases offer critic statistics to debug which critic dominates.

**Conclusion.** MPPI turns the predictive-control idea into a sampling problem. Its advantages over deterministic MPC are freedom in the cost function and simple, general obstacle handling; its costs are compute, stochastic output and soft constraints. For this project's racetrack-tracking task, deterministic MPC remains the cleaner tool, while MPPI shows how the same predictive framework scales to obstacle-rich environments in production ROS 2 systems.

### 8.7 Supporting context: Ackermann kinematics and 2D vs 3D simulation

*These two topics were not the focus of the investigation; they are summarised to complete the synthesis.*

**Ackermann vs bicycle model.** The bicycle model collapses the two front wheels into one. On a real four-wheel car the inner and outer wheels travel on circles of different radius around the same instantaneous centre of rotation, so the inner wheel must steer more (`δ_inner > δ_outer`). For turn radius `R` at the rear-axle centre, wheelbase `L` and track width `w`:

```
δ_inner = atan( L / (R − w/2) )
δ_outer = atan( L / (R + w/2) )
Ackermann condition:  cot δ_outer − cot δ_inner = w / L
```

With this project's geometry (`L = 1.25 m`, `w = 1.18 m`) and `R = 5 m`: `δ_inner ≈ 15.8°`, `δ_outer ≈ 12.6°`, versus the bicycle model's single angle `atan(L/R) ≈ 14.0°`. In `ros2_control`, steering vehicles are handled by chainable controllers such as the Ackermann and bicycle steering controllers from `ros2_controllers`, which convert a `Twist` into a steering angle (position interface) and wheel speeds (velocity interface).

**2D kinematic simulation vs 3D physics (Gazebo / MVSim).** The 2D simulator integrates four states, runs far faster than real time, is deterministic and trivial to set up, which makes it ideal for tuning and fair benchmarking, but it has no slip, suspension, load transfer or simulated sensors. Gazebo and MVSim add rigid-body dynamics, contacts, tyre/ground models and noisy sensors (LiDAR, camera, IMU), at the cost of heavier computation, more setup (URDF/SDF, inertias, friction, `ros2_control` configuration) and less reproducible results. A sensible workflow is to design and benchmark in 2D, then validate in 3D before real deployment.

### 8.8 References

1. G. Williams, N. Wagener, B. Goldfain, P. Drews, J. Rehg, B. Boots, E. Theodorou, "Aggressive driving with model predictive path integral control," ICRA 2016. https://ieeexplore.ieee.org/document/7487277
2. G. Williams et al., "Information Theoretic Model Predictive Control: Theory and Applications to Autonomous Driving."
3. Nav2 MPPI Controller, package overview and README, ROS Index. https://index.ros.org/p/nav2_mppi_controller/
4. Nav2 source: https://github.com/ros-planning/navigation2/tree/humble/nav2_mppi_controller
5. ROSCon 2023 talk on the MPPI Controller. https://vimeo.com/879001391

---

## 9. Reproduction Guide

> Developed and tested on Ubuntu 22.04 (under WSL) with ROS 2 Humble.

### 9.1 Prerequisites

```bash
# ROS 2 (Humble or newer) and build tools
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-pip \
    ros-$ROS_DISTRO-rviz2 ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-rqt-plot ros-$ROS_DISTRO-plotjuggler-ros

# Python dependencies (adjust to the repository requirements)
pip3 install numpy scipy matplotlib pandas
# MPC solver (use the one imported in mpc.py, e.g.:)
pip3 install cvxpy
```

### 9.2 Clone and build

```bash
mkdir -p ~/control_ros2_ws/src && cd ~/control_ros2_ws/src
git clone https://github.com/Dawy007/Control_Project.git
cd ~/control_ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 9.3 Run the simulation (Milestone 1)

Open a terminal for each command below, and source the workspace in every one (`source ~/control_ros2_ws/install/setup.bash`).

```bash
# Base simulation (simulator + track + RViz)
ros2 launch bicycle_sim bicycle_sim.launch.py
```

Inspect the system from another terminal:

```bash
ros2 node list
ros2 topic list -t
ros2 topic echo /throttle
ros2 topic echo /steer
```

Live plots:

```bash
ros2 run rqt_plot rqt_plot
ros2 run plotjuggler plotjuggler
```

### 9.4 Manual driving and cruise control (Milestones 2–4)

With the base simulation running:

```bash
# Direct actuator commands (verify the physics)
ros2 topic pub /throttle std_msgs/msg/Float64 "{data: 0.5}" -r 10
ros2 topic pub /steer std_msgs/msg/Float64 "{data: 0.2}" -r 10

# Interactive keyboard teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The teleoperation bridge converts the `Twist` commands into throttle and steering, and the linear velocity is used as the target speed of the longitudinal PID. If no command is received for more than 0.5 s, the watchdog stops the car.

### 9.5 Autonomous driving (Milestone 5)

Start the base simulation first (9.3), then run **one** controller at a time in a second terminal. The `velocity_mode:=curvature` parameter enables the curvature-based velocity profiler (Milestone 5.1).

```bash
# Lateral PID (reactive)
ros2 run bicycle_control controller --ros-args -p control_mode:=lateral_pid -p velocity_mode:=curvature

# Pure Pursuit
ros2 run bicycle_control controller --ros-args -p control_mode:=pure_pursuit -p velocity_mode:=curvature

# Model Predictive Control
ros2 run bicycle_control controller --ros-args -p control_mode:=mpc -p velocity_mode:=curvature
```

Stop the running controller with `Ctrl+C` before starting the next one. The RViz window shows the car, the track path, the boundary cones and the lap analyzer dashboard markers.

### 9.6 Reproducing the benchmark

1. Start the base simulation with the same track (`centerline_0.csv`) and start pose for every run.
2. Run one controller (9.5) and let the car complete **at least 3 full laps**.
3. Read the metrics from the lap analyzer (best lap time, top speed, mean/max/RMS CTE, laps completed) and record them in the table in [Section 5](#5-benchmark-results).
4. Repeat for the other two controllers.

### 9.7 Telemetry topics

While the simulation is running, `ros2 topic list` shows the following topics:

| Topic | Description |
|---|---|
| `/state` | Vehicle state published by the simulator |
| `/throttle` | Throttle/brake command in `[-1, 1]` |
| `/steer` | Front steering angle command (rad, positive = left) |
| `/path` | Reference centerline path |
| `/track_bounds` | Boundary-cone markers |
| `/telemetry/cte` | Cross-track error |
| `/telemetry/heading_err_deg` | Heading error (degrees) |
| `/telemetry/speed` | Vehicle speed |
| `/telemetry/lap_time` | Lap time |
| `/lap/metrics` | Aggregated lap metrics (best lap, top speed, mean/max/RMS CTE, laps completed) |
| `/lap/visualization` | RViz dashboard markers |
| `/joint_states`, `/robot_description`, `/tf`, `/tf_static` | Robot description and transforms for RViz |

Inspect or plot them live:

```bash
ros2 topic echo /lap/metrics
ros2 topic echo /telemetry/cte
ros2 run plotjuggler plotjuggler      # then add /telemetry/* topics
ros2 run rqt_plot rqt_plot /telemetry/cte/data /telemetry/speed/data
```

### 9.8 Repository layout

```
Control_Project/
├── assets/
│   └── demo.gif
├── bicycle_control/
│   ├── bicycle_control/
│   │   ├── controller_node.py
│   │   ├── lateral_pid.py
│   │   ├── longitudinal_pid.py
│   │   ├── mpc.py
│   │   ├── pure_pursuit.py
│   │   ├── teleop_bridge.py
│   │   └── velocity_profiler.py
│   ├── resource/  test/  package.xml  setup.cfg  setup.py
├── bicycle_sim/
│   ├── bicycle_sim/
│   │   ├── bicycle_model.py
│   │   └── sim_node.py
│   ├── launch/
│   ├── urdf/
│   ├── resource/  test/  package.xml  setup.cfg  setup.py
├── track_environment/
│   ├── track_environment/
│   │   ├── lap_analyzer.py
│   │   ├── path_gen.py
│   │   └── track.py
│   ├── tracks/           # centerline_0.csv, random_track0.csv
│   ├── resource/  test/  package.xml  setup.cfg  setup.py
├── .gitignore
├── LICENSE
└── README.md
```

### 9.9 Deliverables

- `README.md` (this file)
- Source files: `bicycle_model.py`, `teleop_bridge.py`, `longitudinal_pid.py`, `lateral_pid.py`, `pure_pursuit.py`, `mpc.py`, `lap_analyzer.py`
- Video walkthrough (3–5 min): code (anti-windup, heading wrapping, preview calculations), live RViz2 demos of all four controller modes with telemetry, and analysis.

---

*Author: Youssef Cherif Elprince — youssef.yasca.elprince@gmail.com*
