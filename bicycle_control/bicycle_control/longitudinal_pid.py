"""
Low-Level Powertrain Cruise Controller (Longitudinal PID).
Regulates vehicle speed via normalized throttle/braking effort.
"""

import numpy as np  # noqa: F401


class PIDLongitudinalController:
    """Low-Level Powertrain Cruise Controller / Electronic Speed Control (ESC).

    Translates high-level velocity requests into normalized throttle/brake effort.
    Because physical vehicles experience friction and speed-squared aerodynamic drag,
    a closed-loop speed regulator is required to maintain target velocity.
    """

    def __init__(self, kp=1.0, ki=0.2, kd=0.05, dt=0.1,
                 max_throttle=1.0, max_brake=1.0, integral_limit=2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        self.integral_limit = integral_limit

        self.integral = 0.0
        self.prev_error = 0.0

        self.prev_vel = None
        self.prev_output = 0.0
        self.max_step = 0.3

    def compute(self, target_vel, current_vel):
        error = target_vel - current_vel
        p_term = self.kp * error
        if self.prev_vel is None:
            d_meas = 0.0
        else:
            d_meas = (current_vel - self.prev_vel) / self.dt
        d_term = -self.kd * d_meas
        self.prev_vel = current_vel

        u_pre = p_term + self.ki * self.integral + d_term
        winding_up = ((u_pre > self.max_throttle and error > 0.0) or
                      (u_pre < -self.max_brake and error < 0.0))
        if not winding_up:
            self.integral += error * self.dt
            self.integral = float(np.clip(self.integral,
                                          -self.integral_limit, self.integral_limit))

        u = p_term + self.ki * self.integral + d_term

        u = float(np.clip(u, -self.max_brake, self.max_throttle))

        u = float(np.clip(u, self.prev_output - self.max_step,
                          self.prev_output + self.max_step))

        self.prev_output = u
        self.prev_error = error
        return u

    def reset(self):
        self.prev_vel = None
        self.prev_output = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
