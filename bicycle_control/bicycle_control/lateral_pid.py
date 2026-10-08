"""
High-Level Lateral Steering Controller: Reactive Lateral PID.
Steers based on instantaneous Cross-Track Error (CTE) and Heading Error.
"""

import math
import numpy as np  # noqa: F401


class LateralPIDController:
    """Lateral PID steering controller based on Cross-Track Error (CTE) and Heading Error.

    Commands front wheel steering based on instantaneous lateral offset (cross-track error)
    and orientation error relative to the nearest path waypoint.
    """

    def __init__(self, kp=0.8, ki=0.02, kd=0.15, k_yaw=0.5, dt=0.1,
                 max_steer_rad=math.radians(35.0), integral_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.k_yaw = k_yaw
        self.dt = dt
        self.max_steer_rad = max_steer_rad
        self.integral_limit = integral_limit

        self.integral_cte = 0.0
        self.prev_cte = 0.0

    def compute_steering(self, cte, heading_err):
        d_cte = (cte - self.prev_cte) / self.dt
        self.prev_cte = cte

        u_pre = -(self.kp * cte + self.ki * self.integral_cte + self.kd * d_cte) \
                - self.k_yaw * heading_err

        saturated = abs(u_pre) >= self.max_steer_rad
        if not (saturated and cte * u_pre < 0.0):
            self.integral_cte += cte * self.dt
            self.integral_cte = float(np.clip(self.integral_cte,
                                              -self.integral_limit, self.integral_limit))

        delta = -(self.kp * cte + self.ki * self.integral_cte + self.kd * d_cte) \
                - self.k_yaw * heading_err

        return float(np.clip(delta, -self.max_steer_rad, self.max_steer_rad))

    def reset(self):
        """Resets integrator and previous error state."""
        self.integral_cte = 0.0
        self.prev_cte = 0.0
