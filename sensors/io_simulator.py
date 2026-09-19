#!/usr/bin/env python3
"""
Industrial Digital I/O Signal Simulator (工业数字量输入输出仿真)
Simulates: E-Stop, Safety Bumpers, Photoelectric sensors, Lift limits, Brakes, Tower Lights
"""

from typing import Dict, Any


class IOSimulator:
    def __init__(self):
        # Digital Inputs (DI)
        self.di = {
            "di_estop": False,          # Emergency stop active (True = Pressed)
            "di_bumper_front": False,    # Front safety contact edge
            "di_bumper_rear": False,     # Rear safety contact edge
            "di_cargo_present": False,   # Cargo/pallet on chassis
            "di_lift_top": False,        # Lift mechanism at upper limit
            "di_lift_bottom": True       # Lift mechanism at home bottom limit
        }

        # Digital Outputs (DO)
        self.do = {
            "do_brake_release": True,    # Electromagnetic brake release (True = drive enabled)
            "do_tower_green": True,      # Normal operation green light
            "do_tower_yellow": False,    # Standby or navigating yellow light
            "do_tower_red": False,       # Fault or E-stop red light
            "do_buzzer": False,          # Audible alarm
            "do_lift_motor_up": False,   # Jack lifting up
            "do_lift_motor_down": False  # Jack descending down
        }

        self.lift_height_m = 0.0         # 0.0m (bottom) to 0.08m (lifted 80mm)

    def set_di(self, key: str, value: bool):
        if key in self.di:
            self.di[key] = bool(value)
            self._update_safety_interlock()

    def set_do(self, key: str, value: bool):
        if key in self.do:
            self.do[key] = bool(value)

    def _update_safety_interlock(self):
        """Hardware safety interlock chain logic."""
        if self.di["di_estop"] or self.di["di_bumper_front"] or self.di["di_bumper_rear"]:
            # Hard emergency brake lock
            self.do["do_brake_release"] = False
            self.do["do_tower_red"] = True
            self.do["do_tower_green"] = False
            self.do["do_buzzer"] = True
        else:
            self.do["do_brake_release"] = True
            self.do["do_tower_red"] = False
            self.do["do_buzzer"] = False

    def update_lift_physics(self, dt: float):
        """Simulate lift jack mechanism motion."""
        speed = 0.02 # 20 mm/s
        if self.do["do_lift_motor_up"]:
            self.lift_height_m = min(0.08, self.lift_height_m + speed * dt)
        elif self.do["do_lift_motor_down"]:
            self.lift_height_m = max(0.0, self.lift_height_m - speed * dt)

        self.di["di_lift_top"] = (self.lift_height_m >= 0.078)
        self.di["di_lift_bottom"] = (self.lift_height_m <= 0.002)

    def get_io_state(self) -> Dict[str, Any]:
        return {
            "inputs": dict(self.di),
            "outputs": dict(self.do),
            "lift_height_mm": round(self.lift_height_m * 1000.0, 1),
            "is_emergency_stop": not self.do["do_brake_release"]
        }
