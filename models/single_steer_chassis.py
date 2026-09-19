#!/usr/bin/env python3
"""
Single Steer-Drive Chassis (单舵轮底盘 / 叉车三轮底盘)
Kinematics: 1 Steer-Drive Wheel (Front/Center) + 2 Passive Fixed Wheels (Rear Axle)
"""

from typing import Dict, Any, Tuple
import math
from .chassis_base import BaseChassis


class SingleSteerChassis(BaseChassis):
    def __init__(self, length: float = 1.8, width: float = 1.0, wheel_radius: float = 0.125, wheelbase: float = 1.2):
        super().__init__("single_steer_drive", length, width, wheel_radius)
        self.wheelbase = wheelbase  # Distance from rear axle center to front steering wheel (D)
        self.max_speed = 1.2        # m/s
        self.max_accel = 0.8        # m/s^2
        self.max_steer_angle = math.radians(75.0)  # Max steer angle +/- 75 deg
        self.max_steer_rate = math.radians(60.0)   # 60 deg/sec servo rotation speed

        # Actuator Setpoints & States
        self.steer_angle = 0.0        # rad (current actual)
        self.target_steer_angle = 0.0 # rad (commanded)
        self.drive_speed = 0.0        # m/s (linear speed of steer wheel)
        self.target_drive_speed = 0.0
        
        self.drive_wheel_rad = 0.0
        self.drive_motor_rpm = 0.0
        self.drive_motor_current = 0.0
        self.steer_motor_current = 0.0

    def inverse_kinematics(self, vx: float, vy: float, wz: float) -> Dict[str, Any]:
        """
        Calculate steer angle delta and steer-wheel drive velocity vd from (vx, wz).
        Rear axle center is origin.
        """
        # Minimum threshold to avoid division by zero
        if abs(vx) < 1e-4:
            if abs(wz) > 1e-4:
                # In-place turn attempt -> single steer turns to 90 deg
                target_steer = math.copysign(math.pi / 2.0, wz)
                target_vd = wz * self.wheelbase
            else:
                target_steer = self.steer_angle
                target_vd = 0.0
        else:
            # delta = atan2(wz * D, vx)
            target_steer = math.atan2(wz * self.wheelbase, vx)
            target_vd = math.copysign(math.hypot(vx, wz * self.wheelbase), vx)

        # Clamp to steer limits
        target_steer = max(-self.max_steer_angle, min(self.max_steer_angle, target_steer))

        omega_drive = target_vd / self.wheel_radius
        rpm_drive = (omega_drive * 60.0) / (2.0 * math.pi)

        return {
            "steer_angle": target_steer,
            "drive_speed": target_vd,
            "omega_drive": omega_drive,
            "rpm_drive": rpm_drive
        }

    def forward_kinematics(self, actuator_states: Dict[str, Any]) -> Tuple[float, float, float]:
        delta = actuator_states.get("steer_angle", 0.0)
        vd = actuator_states.get("drive_speed", 0.0)

        vx = vd * math.cos(delta)
        vy = 0.0
        wz = (vd * math.sin(delta)) / self.wheelbase
        return vx, vy, wz

    def update_physics(self, cmd_vx: float, cmd_vy: float, cmd_wz: float, dt: float) -> Dict[str, Any]:
        ik = self.inverse_kinematics(cmd_vx, 0.0, cmd_wz)
        self.target_steer_angle = ik["steer_angle"]
        self.target_drive_speed = max(-self.max_speed, min(self.max_speed, ik["drive_speed"]))

        # 1. Steer servo rate limit (simulating physical servo response)
        d_angle = self.target_steer_angle - self.steer_angle
        max_d_angle = self.max_steer_rate * dt
        if abs(d_angle) > max_d_angle:
            self.steer_angle += math.copysign(max_d_angle, d_angle)
            self.steer_motor_current = 4.5 # Peak steering torque current
        else:
            self.steer_angle = self.target_steer_angle
            self.steer_motor_current = 0.8 # Holding current

        # 2. Drive wheel acceleration limit
        dv = self.target_drive_speed - self.drive_speed
        max_dv = self.max_accel * dt
        if abs(dv) > max_dv:
            self.drive_speed += math.copysign(max_dv, dv)
        else:
            self.drive_speed = self.target_drive_speed

        # 3. Compute effective chassis motion from actual actuator states
        self.vx, self.vy, self.wz = self.forward_kinematics({
            "steer_angle": self.steer_angle,
            "drive_speed": self.drive_speed
        })

        # 4. Integrate chassis pose
        delta_x = (self.vx * math.cos(self.theta)) * dt
        delta_y = (self.vx * math.sin(self.theta)) * dt
        delta_theta = self.wz * dt

        self.x += delta_x
        self.y += delta_y
        self.theta = (self.theta + delta_theta + math.pi) % (2.0 * math.pi) - math.pi

        # 5. Actuator rotation updates
        omega_wheel = self.drive_speed / self.wheel_radius
        self.drive_wheel_rad += omega_wheel * dt
        self.drive_motor_rpm = (omega_wheel * 60.0) / (2.0 * math.pi)

        # Drive motor electrical current simulation
        load = 1.8 + abs(self.drive_speed) * 0.8
        self.drive_motor_current = (load / 0.18) if abs(self.drive_speed) > 0.01 else 0.2

        return {
            "x": self.x,
            "y": self.y,
            "theta": self.theta,
            "vx": self.vx,
            "vy": self.vy,
            "wz": self.wz,
            "steer_angle_deg": math.degrees(self.steer_angle),
            "drive_speed": self.drive_speed,
            "actuators": self.get_actuator_telemetry()
        }

    def get_actuator_telemetry(self) -> Dict[str, Any]:
        return {
            "type": "single_steer",
            "steer_wheel": {
                "steer_angle_deg": round(math.degrees(self.steer_angle), 1),
                "target_steer_deg": round(math.degrees(self.target_steer_angle), 1),
                "speed_mps": round(self.drive_speed, 2),
                "rpm": round(self.drive_motor_rpm, 1),
                "drive_current_a": round(self.drive_motor_current, 2),
                "steer_current_a": round(self.steer_motor_current, 2)
            }
        }
