"""
High-Level Lateral Steering Controller: Geometric Pure Pursuit.
Calculates steering curvature from lookahead arc geometry.
"""

import math  # noqa: F401
import numpy as np  # noqa: F401


class PurePursuitController:
    """Adaptive Pure Pursuit lateral controller."""

    def __init__(self, wheelbase=1.25, kv=0.25, l_min=0.8, l_max=2.5,
                 max_steer_rad=math.radians(35.0)):
        self.L = wheelbase
        self.kv = kv
        self.l_min = l_min
        self.l_max = l_max
        self.max_steer_rad = max_steer_rad

    def compute_lookahead(self, v):
        return float(np.clip(self.kv * abs(v) + self.l_min, self.l_min, self.l_max))

    def find_target_waypoint(self, x, y, path_points, lookahead):
        n = len(path_points)
        if n == 0:
            return 0, None

        nearest = min(range(n),
                      key=lambda i: (path_points[i][0] - x) ** 2 + (path_points[i][1] - y) ** 2)

        for k in range(n):
            i = (nearest + k) % n
            if math.hypot(path_points[i][0] - x, path_points[i][1] - y) >= lookahead:
                return i, path_points[i]

        return nearest, path_points[nearest]

    def compute_steering(self, x, y, yaw, target_pt, lookahead):
        if target_pt is None:
            return 0.0

        dx = target_pt[0] - x
        dy = target_pt[1] - y

        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy

        alpha = math.atan2(local_y, local_x)
        ld = max(math.hypot(dx, dy), 1e-3)

        delta = math.atan2(2.0 * self.L * math.sin(alpha), ld)
        return float(np.clip(delta, -self.max_steer_rad, self.max_steer_rad))
