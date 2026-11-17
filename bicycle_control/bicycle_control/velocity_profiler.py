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

    def compute_target_speed(self, kappa, fallback_speed=None):#compute_target_speed/ chooses target speed based on how sharply the track curves
        if kappa is None or not math.isfinite(kappa): #if the curvature is missing
            return fallback_speed if fallback_speed is not None else self.default_speed

        k = abs(kappa) #get the curvature magnitude
        if k < 1e-4: # k is very small
            v = self.max_speed
        else:
            v = math.sqrt(self.max_lat_accel / k)

        return max(1.0, min(self.max_speed, v)) # return the speed limited between 1 and max_speed
