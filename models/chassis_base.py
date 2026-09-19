#!/usr/bin/env python3
"""
Base chassis interface for AMR Studio V4 multi-chassis kinematics.
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, Tuple
import math


class BaseChassis(ABC):
    def __init__(self, name: str, length: float, width: float, wheel_radius: float):
        self.name = name
        self.length = length
        self.width = width
        self.wheel_radius = wheel_radius
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

    def reset_pose(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

    @abstractmethod
    def inverse_kinematics(self, vx: float, vy: float, wz: float) -> Dict[str, Any]:
        """Convert chassis velocities to actuator setpoints (wheel speeds and steer angles)."""
        pass

    @abstractmethod
    def forward_kinematics(self, actuator_states: Dict[str, Any]) -> Tuple[float, float, float]:
        """Convert actual actuator states to chassis velocities (vx, vy, wz)."""
        pass

    @abstractmethod
    def update_physics(self, cmd_vx: float, cmd_vy: float, cmd_wz: float, dt: float) -> Dict[str, Any]:
        """Update numerical integration for one simulation timestep."""
        pass

    @abstractmethod
    def get_actuator_telemetry(self) -> Dict[str, Any]:
        """Return actuator details (RPM, angles, currents, torques)."""
        pass
