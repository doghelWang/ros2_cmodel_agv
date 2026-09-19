#!/usr/bin/env python3
"""
2D LiDAR Raycasting & Multi-LiDAR Extrinsic Engine (激光雷达多传感器仿真)
"""

import math
import random
from typing import List, Tuple, Dict, Any


class LidarSimulator:
    def __init__(self, angle_min: float = -2.356, angle_max: float = 2.356, num_beams: float = 360, range_min: float = 0.05, range_max: float = 12.0):
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.num_beams = int(num_beams)
        self.range_min = range_min
        self.range_max = range_max
        self.angle_inc = (angle_max - angle_min) / float(self.num_beams - 1)

    @staticmethod
    def _ray_line_intersect(p0: Tuple[float, float], p1: Tuple[float, float], w0: Tuple[float, float], w1: Tuple[float, float]) -> float:
        """Find intersection distance along ray p0->p1 with segment w0->w1."""
        rx = p1[0] - p0[0]
        ry = p1[1] - p0[1]
        sx = w1[0] - w0[0]
        sy = w1[1] - w0[1]

        denom = rx * sy - ry * sx
        if abs(denom) < 1e-9:
            return 999.0

        t = ((w0[0] - p0[0]) * sy - (w0[1] - p0[1]) * sx) / denom
        u = ((w0[0] - p0[0]) * ry - (w0[1] - p0[1]) * rx) / denom

        if t > 0.0 and 0.0 <= u <= 1.0:
            return t
        return 999.0

    def cast_rays(self, lidar_global_x: float, lidar_global_y: float, lidar_global_yaw: float, walls: List[Tuple[float, float, float, float]], noise_std: float = 0.008) -> List[float]:
        ranges = []
        p0 = (lidar_global_x, lidar_global_y)

        for i in range(self.num_beams):
            angle = lidar_global_yaw + self.angle_min + i * self.angle_inc
            p1 = (p0[0] + math.cos(angle), p0[1] + math.sin(angle))

            min_dist = self.range_max
            for w in walls:
                w0 = (w[0], w[1])
                w1 = (w[2], w[3])
                dist = self._ray_line_intersect(p0, p1, w0, w1)
                if dist < min_dist:
                    min_dist = dist

            # Apply range limit and sensor measurement noise
            if min_dist < self.range_max:
                if noise_std > 0.0:
                    min_dist += random.gauss(0.0, noise_std)
                min_dist = max(self.range_min, min(self.range_max, min_dist))
            else:
                min_dist = self.range_max

            ranges.append(round(min_dist, 3))

        return ranges
