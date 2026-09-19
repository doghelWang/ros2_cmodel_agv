#!/usr/bin/env python3
"""
Vision & AprilTag/QR Marker Sensor Simulator (视觉传感器与地标码识别仿真)
"""

import math
from typing import List, Dict, Any, Optional


class VisionSimulator:
    def __init__(self, fov_deg: float = 80.0, max_dist: float = 3.5):
        self.fov_rad = math.radians(fov_deg)
        self.max_dist = max_dist

        # Simulated ground & workstation visual markers in the facility
        self.known_markers = [
            {"id": 101, "name": "Tag_Station_A", "x": 3.5, "y": 2.0, "yaw": 0.0},
            {"id": 102, "name": "Tag_Station_B", "x": -4.5, "y": 0.0, "yaw": math.pi},
            {"id": 103, "name": "Tag_Charging_Dock", "x": 0.0, "y": -6.5, "yaw": -math.pi / 2.0},
            {"id": 104, "name": "Tag_Center_Junction", "x": 0.0, "y": 0.0, "yaw": 0.0},
            {"id": 105, "name": "Tag_Aisle_1", "x": 2.0, "y": -2.0, "yaw": 0.0}
        ]

    def detect_markers(self, robot_x: float, robot_y: float, robot_yaw: float) -> List[Dict[str, Any]]:
        detected = []

        for m in self.known_markers:
            dx = m["x"] - robot_x
            dy = m["y"] - robot_y
            dist = math.hypot(dx, dy)

            if dist > self.max_dist:
                continue

            # Check if within camera field of view
            bearing = math.atan2(dy, dx)
            angle_diff = (bearing - robot_yaw + math.pi) % (2.0 * math.pi) - math.pi

            if abs(angle_diff) <= (self.fov_rad / 2.0):
                # Detected in FOV!
                # Relative pose in robot frame
                rel_x = dx * math.cos(-robot_yaw) - dy * math.sin(-robot_yaw)
                rel_y = dx * math.sin(-robot_yaw) + dy * math.cos(-robot_yaw)
                rel_yaw = (m["yaw"] - robot_yaw + math.pi) % (2.0 * math.pi) - math.pi

                detected.append({
                    "id": m["id"],
                    "name": m["name"],
                    "distance_m": round(dist, 2),
                    "bearing_deg": round(math.degrees(angle_diff), 1),
                    "rel_x": round(rel_x, 3),
                    "rel_y": round(rel_y, 3),
                    "rel_yaw_deg": round(math.degrees(rel_yaw), 1)
                })

        return detected
