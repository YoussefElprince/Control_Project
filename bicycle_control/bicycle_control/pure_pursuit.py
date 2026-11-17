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

    def compute_lookahead(self, v): #compute_lookahead(self, v): decides how far ahead the car should look
        return float(np.clip(self.kv * abs(v) + self.l_min, self.l_min, self.l_max)) #kol ma tamshy faster lazam ta look ab3ad

    def find_target_waypoint(self, x, y, path_points, lookahead): #find_target_waypoint (choose a point on the path for the car to steer)
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

    def compute_steering(self, x, y, yaw, target_pt, lookahead): #compute_steering(calculates the (angle) to turn the front wheels toward the target point)
        if target_pt is None:
            return 0.0

        dx = target_pt[0] - x
        dy = target_pt[1] - y

        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy  # how far forward it is
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy # how far left it is.

        alpha = math.atan2(local_y, local_x) #angle between the car's forward direction and the target
        ld = max(math.hypot(dx, dy), 1e-3)  #the distance to the target

        delta = math.atan2(2.0 * self.L * math.sin(alpha), ld) #steer angle to follow circular arc from the car rear axis toward the target
        return float(np.clip(delta, -self.max_steer_rad, self.max_steer_rad)) #limit el steering angle and return it in radian
