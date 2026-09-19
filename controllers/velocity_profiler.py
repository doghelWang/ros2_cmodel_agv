#!/usr/bin/env python3
"""
Velocity Profiler for AMR Studio V4 (梯形与 S 曲线速度平滑规划器)
Supports: Trapezoidal & Jerk-Limited S-Curve Profiling
"""

import math
from typing import Tuple


class VelocityProfiler:
    def __init__(self, max_v: float = 1.2, max_a: float = 0.8, max_jerk: float = 2.0):
        self.max_v = max_v
        self.max_a = max_a
        self.max_jerk = max_jerk
        self.current_v = 0.0
        self.current_a = 0.0

    def reset(self, v: float = 0.0, a: float = 0.0):
        self.current_v = v
        self.current_a = a

    def compute_trapezoidal_step(self, target_dist: float, dt: float) -> float:
        """
        Trapezoidal deceleration: v_target = sqrt(2 * a * dist)
        Returns desired linear velocity.
        """
        # Braking distance needed to stop from current_v
        v_stop = math.sqrt(max(0.0, 2.0 * self.max_a * max(0.0, target_dist)))
        desired_v = min(self.max_v, v_stop)

        # Acceleration slope
        dv = desired_v - self.current_v
        max_dv = self.max_a * dt
        if abs(dv) > max_dv:
            self.current_v += math.copysign(max_dv, dv)
        else:
            self.current_v = desired_v

        return self.current_v

    def compute_scurve_step(self, target_dist: float, dt: float) -> float:
        """
        S-Curve Jerk-limited profile step:
        da = jerk * dt
        dv = a * dt
        """
        # Safe target velocity calculated from distance with jerk limit
        v_limit = math.sqrt(max(0.0, 2.0 * self.max_a * target_dist))
        target_v = min(self.max_v, v_limit)

        # Calculate desired acceleration
        desired_a = (target_v - self.current_v) / max(dt, 0.01)
        desired_a = max(-self.max_a, min(self.max_a, desired_a))

        # Apply Jerk limit to acceleration
        da = desired_a - self.current_a
        max_da = self.max_jerk * dt
        if abs(da) > max_da:
            self.current_a += math.copysign(max_da, da)
        else:
            self.current_a = desired_a

        # Integrate velocity
        self.current_v += self.current_a * dt
        self.current_v = max(0.0, min(self.max_v, self.current_v))

        return self.current_v
