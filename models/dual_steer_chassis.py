#!/usr/bin/env python3
"""
Dual Steer-Drive Chassis (双舵轮全向底盘 / 重载 AGV)
Kinematics: 2 Independent Steer-Drive Wheels (Front & Rear)
Supports: Longitudinal, Lateral (Crab Steer), Spin In-Place, and Combined Motion
"""

from typing import Dict, Any, Tuple
import math
from .chassis_base import BaseChassis


class DualSteerChassis(BaseChassis):
    def __init__(self, length: float = 2.0, width: float = 1.0, wheel_radius: float = 0.125, wheel_dist: float = 1.4):
        super().__init__("dual_steer_drive", length, width, wheel_radius)
        self.d = wheel_dist / 2.0   # Half distance from center to each steering wheel
        # Wheel 1 (Front): (x = +d, y = 0)
        # Wheel 2 (Rear):  (x = -d, y = 0)
        self.max_speed = 1.2        # m/s
        self.max_accel = 0.8        # m/s^2
        self.max_steer_rate = math.radians(90.0) # 90 deg/s

        # Wheel 1 (Front) States
        self.steer1 = 0.0
        self.target_steer1 = 0.0
        self.speed1 = 0.0
        self.target_speed1 = 0.0
        self.rad1 = 0.0

        # Wheel 2 (Rear) States
        self.steer2 = 0.0
        self.target_steer2 = 0.0
        self.speed2 = 0.0
        self.target_speed2 = 0.0
        self.rad2 = 0.0

    @staticmethod
    def _optimize_steer_angle(target_angle: float, target_speed: float, current_angle: float) -> Tuple[float, float]:
        """
        Steer Angle Optimization: Nearest angle flip.
        If angular diff > 90 deg, flip angle by 180 deg and invert drive speed.
        """
        diff = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi
        if abs(diff) > (math.pi / 2.0):
            # Flip
            if diff > 0:
                opt_angle = target_angle - math.pi
            else:
                opt_angle = target_angle + math.pi
            opt_speed = -target_speed
        else:
            opt_angle = target_angle
            opt_speed = target_speed

        # Normalize to [-pi, pi]
        opt_angle = (opt_angle + math.pi) % (2 * math.pi) - math.pi
        return opt_angle, opt_speed

    def inverse_kinematics(self, vx: float, vy: float, wz: float) -> Dict[str, Any]:
        """
        Decompose chassis (vx, vy, wz) into front & rear wheel speeds and steer angles.
        Front Wheel (x1 = +d, y1 = 0):
          v1x = vx - wz * y1 = vx
          v1y = vy + wz * x1 = vy + wz * d
        Rear Wheel (x2 = -d, y2 = 0):
          v2x = vx - wz * y2 = vx
          v2y = vy + wz * x2 = vy - wz * d
        """
        # Wheel 1 (Front)
        v1x = vx
        v1y = vy + wz * self.d
        speed1_raw = math.hypot(v1x, v1y)
        angle1_raw = math.atan2(v1y, v1x) if speed1_raw > 1e-4 else self.steer1

        # Wheel 2 (Rear)
        v2x = vx
        v2y = vy - wz * self.d
        speed2_raw = math.hypot(v2x, v2y)
        angle2_raw = math.atan2(v2y, v2x) if speed2_raw > 1e-4 else self.steer2

        # Optimize with nearest angle flip
        opt_angle1, opt_speed1 = self._optimize_steer_angle(angle1_raw, speed1_raw, self.steer1)
        opt_angle2, opt_speed2 = self._optimize_steer_angle(angle2_raw, speed2_raw, self.steer2)

        return {
            "wheel1": {"steer": opt_angle1, "speed": opt_speed1},
            "wheel2": {"steer": opt_angle2, "speed": opt_speed2}
        }

    def forward_kinematics(self, actuator_states: Dict[str, Any]) -> Tuple[float, float, float]:
        w1 = actuator_states.get("wheel1", {})
        w2 = actuator_states.get("wheel2", {})

        s1 = w1.get("speed", 0.0)
        a1 = w1.get("steer", 0.0)
        s2 = w2.get("speed", 0.0)
        a2 = w2.get("steer", 0.0)

        v1x = s1 * math.cos(a1)
        v1y = s1 * math.sin(a1)
        v2x = s2 * math.cos(a2)
        v2y = s2 * math.sin(a2)

        vx = (v1x + v2x) / 2.0
        vy = (v1y + v2y) / 2.0
        wz = (v1y - v2y) / (2.0 * self.d)

        return vx, vy, wz

    def update_physics(self, cmd_vx: float, cmd_vy: float, cmd_wz: float, dt: float) -> Dict[str, Any]:
        ik = self.inverse_kinematics(cmd_vx, cmd_vy, cmd_wz)
        self.target_steer1 = ik["wheel1"]["steer"]
        self.target_speed1 = max(-self.max_speed, min(self.max_speed, ik["wheel1"]["speed"]))
        self.target_steer2 = ik["wheel2"]["steer"]
        self.target_speed2 = max(-self.max_speed, min(self.max_speed, ik["wheel2"]["speed"]))

        # 1. Steer servo rate limit for Wheel 1
        d_angle1 = (self.target_steer1 - self.steer1 + math.pi) % (2 * math.pi) - math.pi
        max_rate = self.max_steer_rate * dt
        if abs(d_angle1) > max_rate:
            self.steer1 += math.copysign(max_rate, d_angle1)
        else:
            self.steer1 = self.target_steer1
        self.steer1 = (self.steer1 + math.pi) % (2 * math.pi) - math.pi

        # Steer servo rate limit for Wheel 2
        d_angle2 = (self.target_steer2 - self.steer2 + math.pi) % (2 * math.pi) - math.pi
        if abs(d_angle2) > max_rate:
            self.steer2 += math.copysign(max_rate, d_angle2)
        else:
            self.steer2 = self.target_steer2
        self.steer2 = (self.steer2 + math.pi) % (2 * math.pi) - math.pi

        # 2. Drive speed rate limit
        dv1 = self.target_speed1 - self.speed1
        max_dv = self.max_accel * dt
        if abs(dv1) > max_dv:
            self.speed1 += math.copysign(max_dv, dv1)
        else:
            self.speed1 = self.target_speed1

        dv2 = self.target_speed2 - self.speed2
        if abs(dv2) > max_dv:
            self.speed2 += math.copysign(max_dv, dv2)
        else:
            self.speed2 = self.target_speed2

        # 3. Compute actual chassis velocity
        self.vx, self.vy, self.wz = self.forward_kinematics({
            "wheel1": {"speed": self.speed1, "steer": self.steer1},
            "wheel2": {"speed": self.speed2, "steer": self.steer2}
        })

        # 4. Integrate chassis pose (Holonomic planar integration)
        delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
        delta_theta = self.wz * dt

        self.x += delta_x
        self.y += delta_y
        self.theta = (self.theta + delta_theta + math.pi) % (2.0 * math.pi) - math.pi

        # Wheel revolutions
        self.rad1 += (self.speed1 / self.wheel_radius) * dt
        self.rad2 += (self.speed2 / self.wheel_radius) * dt

        return {
            "x": self.x,
            "y": self.y,
            "theta": self.theta,
            "vx": self.vx,
            "vy": self.vy,
            "wz": self.wz,
            "actuators": self.get_actuator_telemetry()
        }

    def get_actuator_telemetry(self) -> Dict[str, Any]:
        rpm1 = (self.speed1 / self.wheel_radius * 60.0) / (2.0 * math.pi)
        rpm2 = (self.speed2 / self.wheel_radius * 60.0) / (2.0 * math.pi)
        return {
            "type": "dual_steer",
            "front_wheel": {
                "steer_angle_deg": round(math.degrees(self.steer1), 1),
                "target_steer_deg": round(math.degrees(self.target_steer1), 1),
                "speed_mps": round(self.speed1, 2),
                "rpm": round(rpm1, 1),
                "current_a": round(1.2 + abs(self.speed1) * 2.5, 2)
            },
            "rear_wheel": {
                "steer_angle_deg": round(math.degrees(self.steer2), 1),
                "target_steer_deg": round(math.degrees(self.target_steer2), 1),
                "speed_mps": round(self.speed2, 2),
                "rpm": round(rpm2, 1),
                "current_a": round(1.2 + abs(self.speed2) * 2.5, 2)
            }
        }
