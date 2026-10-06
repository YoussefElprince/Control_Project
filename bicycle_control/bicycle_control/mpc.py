"""
High-Level Lateral Steering Controller: Extended Kinematic Bicycle MPC.
Solves a constrained non-linear program over prediction horizon N using SciPy,
optimizing steering angle and longitudinal acceleration (mapped to throttle).
"""

import math  # noqa: F401
import numpy as np  # noqa: F401
from scipy.optimize import minimize  # noqa: F401


class KinematicBicycleMPC:
    """Nonlinear Model Predictive Control for an Extended Kinematic Bicycle Model.

    Optimizes future control sequences u = [delta_k, a_k] where steering angle delta_k
    and longitudinal acceleration a_k (mapped to throttle effort) are the control inputs,
    forward-simulating a 4-state extended kinematic bicycle model x = [x, y, theta, v]^T.
    """

    def __init__(self, wheelbase=1.25, dt=0.1, horizon=10,
                 max_steer_rad=math.radians(35.0), k_a=4.0,
                 max_accel=None, max_brake=None, c_drag=0.005, c_roll=0.05):
        self.L = wheelbase
        self.dt = dt
        self.N = horizon
        self.max_steer_rad = max_steer_rad
        self.k_a = float(max_accel if max_accel is not None else k_a)
        self.c_drag = c_drag
        self.c_roll = c_roll

        # Weights: heavily penalize lateral CTE, heading error, and steering rate
        self.w_lat = 30.0
        self.w_long = 1.0
        self.w_yaw = 10.0
        self.w_v = 1.0
        self.w_steer = 0.2
        self.w_dsteer = 6.0
        self.w_accel = 0.1

        self.last_u = np.zeros(2 * self.N)  # warm-start [delta_0, a_0, delta_1, a_1, ...]

    def solve(self, x0, ref_trajectory, current_steer=0.0):
        N = min(self.N, len(ref_trajectory))
        if N < 2:
            return 0.0, 0.0

        ref = [list(map(float, r)) for r in ref_trajectory[:N]]
        x_init, y_init, yaw_init, v_init = [float(c) for c in x0]
        L, dt = self.L, self.dt

        # Bounds: u = [delta_0, a_0, delta_1, a_1, ...]
        bounds = []
        for _ in range(N):
            bounds.append((-self.max_steer_rad, self.max_steer_rad))
            bounds.append((-self.k_a, self.k_a))

        def objective(u):
            x, y, yaw, v = x_init, y_init, yaw_init, v_init
            prev_delta = current_steer
            cost = 0.0
            for k in range(N):
                delta = u[2 * k]
                a = u[2 * k + 1]

                x += v * math.cos(yaw) * dt
                y += v * math.sin(yaw) * dt
                yaw += (v / L) * math.tan(delta) * dt
                v += (a - self.c_drag * v * abs(v) - self.c_roll * v) * dt

                xr, yr, yaw_r, v_r = ref[k]
                dx = x - xr
                dy = y - yr
                e_long = math.cos(yaw_r) * dx + math.sin(yaw_r) * dy
                e_lat = -math.sin(yaw_r) * dx + math.cos(yaw_r) * dy
                e_yaw = math.atan2(math.sin(yaw - yaw_r), math.cos(yaw - yaw_r))
                e_v = v - v_r

                cost += (self.w_lat * e_lat ** 2 + self.w_long * e_long ** 2
                         + self.w_yaw * e_yaw ** 2 + self.w_v * e_v ** 2
                         + self.w_steer * delta ** 2
                         + self.w_dsteer * (delta - prev_delta) ** 2
                         + self.w_accel * a ** 2)
                prev_delta = delta
            return cost

        u_init = np.concatenate([self.last_u[2:], self.last_u[-2:]])[:2 * N]
        lo = np.array([b[0] for b in bounds])
        hi = np.array([b[1] for b in bounds])
        u_init = np.clip(u_init, lo, hi)

        res = minimize(objective, u_init, bounds=bounds, method='SLSQP',
                       options={'maxiter': 25, 'ftol': 1e-3})

        u_opt = res.x if np.all(np.isfinite(res.x)) else u_init

        full = np.tile(u_opt[-2:], self.N)
        full[:2 * N] = u_opt
        self.last_u = full

        delta_cmd = float(u_opt[0])
        throttle_cmd = float(np.clip(u_opt[1] / self.k_a, -1.0, 1.0))
        return delta_cmd, throttle_cmd
