"""
Target Velocity Profiler based on track curvature.
Calculates maximum safe cornering speeds subject to lateral acceleration limits.
"""

import math  # noqa: F401


class VelocityProfiler:
    """Generates target speed profiles based on track curvature or precomputed data."""

    def __init__(self, default_speed=4.0, max_speed=8.0, max_lat_accel=5.0):
        self.default_speed = default_speed
        self.max_speed = max_speed
        self.max_lat_accel = max_lat_accel

    def compute_target_speed(self, kappa, fallback_speed=None):
        if kappa is None or not math.isfinite(kappa):
            return fallback_speed if fallback_speed is not None else self.default_speed

        k = abs(kappa)
        if k < 1e-4:
            v = self.max_speed                       # straight: no curvature limit
        else:
            v = math.sqrt(self.max_lat_accel / k)    # v = sqrt(a_lat_max / |kappa|)

        # Clamp: never above max_speed, and keep a 1 m/s floor so the car never stalls in a hairpin
        return max(1.0, min(self.max_speed, v))
