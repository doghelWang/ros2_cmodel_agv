#!/usr/bin/env python3
"""
Differential Drive Chassis (两轮差速底盘)
Kinematics: 2 Active Drive Wheels + Passive Casters
"""

from typing import Dict, Any, Tuple
import math
from .chassis_base import BaseChassis


class DiffDriveChassis(BaseChassis):
    def __init__(self, length: float = 1.788, width: float = 0.994, wheel_radius: float = 0.115, track_width: float = 0.795):
        super().__init__("differential_drive", length, width, wheel_radius)
        self.track_width = track_width
        self.max_speed = 1.5       # m/s
        self.max_accel = 1.0       # m/s^2
        self.max_ang_speed = 2.0   # rad/s
        self.max_ang_accel = 2.5   # rad/s^2

        # Actuator States
        self.left_wheel_rad = 0.0
        self.right_wheel_rad = 0.0
        self.left_wheel_speed = 0.0   # rad/s
        self.right_wheel_speed = 0.0  # rad/s
        
        # Motor Electrical Simulation
        self.left_motor_rpm = 0.0
        self.right_motor_rpm = 0.0
        self.left_motor_current = 0.0  # A
        self.right_motor_current = 0.0 # A
        self.left_motor_torque = 0.0   # N.m
        self.right_motor_torque = 0.0  # N.m

    def inverse_kinematics(self, vx: float, vy: float, wz: float) -> Dict[str, Any]:
        # vy is non-holonomic constraint (must be 0 for diff drive)
        v_left = vx - (wz * self.track_width / 2.0)
        v_right = vx + (wz * self.track_width / 2.0)

        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius

        rpm_left = (omega_left * 60.0) / (2.0 * math.pi)
        rpm_right = (omega_right * 60.0) / (2.0 * math.pi)

        return {
            "v_left": v_left,
            "v_right": v_right,
            "omega_left": omega_left,
            "omega_right": omega_right,
            "rpm_left": rpm_left,
            "rpm_right": rpm_right
        }

    def forward_kinematics(self, actuator_states: Dict[str, Any]) -> Tuple[float, float, float]:
        omega_l = actuator_states.get("omega_left", 0.0)
        omega_r = actuator_states.get("omega_right", 0.0)
        
        vl = omega_l * self.wheel_radius
        vr = omega_r * self.wheel_radius

        vx = (vr + vl) / 2.0
        vy = 0.0
        wz = (vr - vl) / self.track_width
        return vx, vy, wz

    def update_physics(self, cmd_vx: float, cmd_vy: float, cmd_wz: float, dt: float) -> Dict[str, Any]:
        # Clamp command velocities
        target_vx = max(-self.max_speed, min(self.max_speed, cmd_vx))
        target_wz = max(-self.max_ang_speed, min(self.max_ang_speed, cmd_wz))

        # Acceleration slope limits
        dv = target_vx - self.vx
        max_dv = self.max_accel * dt
        if abs(dv) > max_dv:
            self.vx += math.copysign(max_dv, dv)
        else:
            self.vx = target_vx

        dw = target_wz - self.wz
        max_dw = self.max_ang_accel * dt
        if abs(dw) > max_dw:
            self.wz += math.copysign(max_dw, dw)
        else:
            self.wz = target_wz

        self.vy = 0.0

        # Pose integration
        delta_x = (self.vx * math.cos(self.theta)) * dt
        delta_y = (self.vx * math.sin(self.theta)) * dt
        delta_theta = self.wz * dt

        self.x += delta_x
        self.y += delta_y
        self.theta = (self.theta + delta_theta + math.pi) % (2.0 * math.pi) - math.pi

        # Actuator updates
        ik = self.inverse_kinematics(self.vx, 0.0, self.wz)
        self.left_wheel_speed = ik["omega_left"]
        self.right_wheel_speed = ik["omega_right"]
        self.left_wheel_rad += self.left_wheel_speed * dt
        self.right_wheel_rad += self.right_wheel_speed * dt

        self.left_motor_rpm = ik["rpm_left"]
        self.right_motor_rpm = ik["rpm_right"]

        # Motor electrical load simulation: Kt = 0.15 Nm/A, friction = 1.2 Nm
        load_torque_l = 1.2 + abs(self.left_wheel_speed) * 0.4
        load_torque_r = 1.2 + abs(self.right_wheel_speed) * 0.4
        self.left_motor_torque = load_torque_l if abs(self.left_wheel_speed) > 0.01 else 0.0
        self.right_motor_torque = load_torque_r if abs(self.right_wheel_speed) > 0.01 else 0.0
        self.left_motor_current = self.left_motor_torque / 0.15
        self.right_motor_current = self.right_motor_torque / 0.15

        return {
            "x": self.x,
            "y": self.y,
            "theta": self.theta,
            "vx": self.vx,
            "vy": self.vy,
            "wz": self.wz,
            "left_wheel_rad": self.left_wheel_rad,
            "right_wheel_rad": self.right_wheel_rad,
            "actuators": self.get_actuator_telemetry()
        }

    def get_actuator_telemetry(self) -> Dict[str, Any]:
        return {
            "type": "diff_drive",
            "wheels": {
                "left": {
                    "rad": round(self.left_wheel_rad, 2),
                    "speed_radps": round(self.left_wheel_speed, 2),
                    "rpm": round(self.left_motor_rpm, 1),
                    "current_a": round(self.left_motor_current, 2),
                    "torque_nm": round(self.left_motor_torque, 2)
                },
                "right": {
                    "rad": round(self.right_wheel_rad, 2),
                    "speed_radps": round(self.right_wheel_speed, 2),
                    "rpm": round(self.right_motor_rpm, 1),
                    "current_a": round(self.right_motor_current, 2),
                    "torque_nm": round(self.right_motor_torque, 2)
                }
            }
        }
