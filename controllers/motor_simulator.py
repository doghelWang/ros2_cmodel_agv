#!/usr/bin/env python3
"""
Motor & Actuator Electrical Simulation (工业电机与执行器电气仿真)
Simulates: DC Brushless Drive Motors, Steer Servos, Encoders, Current & Torque Loops
"""

import math
from typing import Dict, Any


class MotorSimulator:
    def __init__(self, rated_rpm: float = 3000.0, kt: float = 0.12, r_armature: float = 0.45):
        self.rated_rpm = rated_rpm
        self.kt = kt              # Torque constant (Nm/A)
        self.r_armature = r_armature # Armature resistance (Ohms)
        self.bus_voltage = 48.0   # Standard AGV 48V DC bus

        # Dynamic State
        self.actual_rpm = 0.0
        self.target_rpm = 0.0
        self.current_a = 0.0
        self.torque_nm = 0.0
        self.encoder_pulses = 0
        self.cpr = 4096           # 4096 Counts Per Revolution

    def update(self, target_rpm: float, load_torque: float, dt: float) -> Dict[str, Any]:
        self.target_rpm = max(-self.rated_rpm, min(self.rated_rpm, target_rpm))

        # Motor inertia response: first order lag (tau = 0.05s)
        alpha = dt / (0.05 + dt)
        self.actual_rpm += (self.target_rpm - self.actual_rpm) * alpha

        # Torque generation & current: I = (load_torque + acc_torque) / Kt
        acc_torque = 0.002 * ((self.actual_rpm - target_rpm) / max(dt, 0.01)) * (2 * math.pi / 60.0)
        self.torque_nm = load_torque + abs(acc_torque)
        
        # Current draw with baseline idle current (0.35A)
        if abs(self.actual_rpm) > 1.0 or abs(self.target_rpm) > 1.0:
            self.current_a = 0.35 + (self.torque_nm / max(1e-3, self.kt))
        else:
            self.current_a = 0.05

        # Encoder accumulation
        delta_rev = (self.actual_rpm / 60.0) * dt
        self.encoder_pulses = int(self.encoder_pulses + delta_rev * self.cpr) % (self.cpr * 100000)

        return {
            "target_rpm": round(self.target_rpm, 1),
            "actual_rpm": round(self.actual_rpm, 1),
            "current_a": round(self.current_a, 2),
            "torque_nm": round(self.torque_nm, 2),
            "encoder_pulses": self.encoder_pulses,
            "bus_voltage_v": self.bus_voltage
        }
