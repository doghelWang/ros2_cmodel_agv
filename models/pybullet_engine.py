#!/usr/bin/env python3
"""
PyBullet AGV Simulation Engine (Bullet Physics 3 C++ Core)
Headless physical simulation for:
- Mobile AGV chassis dynamics & kinematics (Diff-drive, Single-steer, Dual-steer)
- Motor joint control (p.VELOCITY_CONTROL / p.TORQUE_CONTROL)
- Batch LiDAR raycasting (p.rayTestBatch)
- Scenario static collision bodies (Walls & Shelves)
- Real-time physics step time profiling
"""

import os
import math
import time
import random
from typing import List, Tuple, Dict, Any, Optional

try:
    import pybullet as p
    import pybullet_data
    PYBULLET_AVAILABLE = True
except ImportError:
    PYBULLET_AVAILABLE = False


class PyBulletAGVEngine:
    def __init__(self, urdf_path: str, wheel_radius: float = 0.115, track_width: float = 0.795, time_step: float = 0.02):
        if not PYBULLET_AVAILABLE:
            raise RuntimeError("PyBullet is not installed in the current Python environment.")

        self.wheel_radius = wheel_radius
        self.track_width = track_width
        self.time_step = time_step
        self.urdf_path = urdf_path

        # Connect to PyBullet in headless DIRECT mode (Zero GUI/OpenGL overhead)
        self.client_id = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.time_step)

        # Ground plane
        self.plane_id = p.loadURDF("plane.urdf")
        p.changeDynamics(self.plane_id, -1, lateralFriction=1.0, restitution=0.0)

        # Robot MultiBody
        self.robot_id = None
        self.joint_indices = {}
        self.left_wheel_idx = -1
        self.right_wheel_idx = -1
        self._load_robot(0.0, 0.0, 0.0)

        # Scenario obstacle collision bodies
        self.obstacle_body_ids = []

        # Profiling & Diagnostics
        self.last_step_ms = 0.0
        self.physics_step_count = 0

    def _load_robot(self, x: float, y: float, yaw: float):
        if self.robot_id is not None:
            p.removeBody(self.robot_id)

        orn = p.getQuaternionFromEuler([0, 0, yaw])
        # Place base slightly elevated so wheel contacts the plane at z=0
        z_init = self.wheel_radius + 0.005
        self.robot_id = p.loadURDF(self.urdf_path, basePosition=[x, y, z_init], baseOrientation=orn)

        # Query all joint names and indices
        self.joint_indices.clear()
        num_joints = p.getNumJoints(self.robot_id)
        for j in range(num_joints):
            info = p.getJointInfo(self.robot_id, j)
            j_name = info[1].decode("utf-8")
            self.joint_indices[j_name] = j

        self.left_wheel_idx = self.joint_indices.get("left_wheel_joint", -1)
        self.right_wheel_idx = self.joint_indices.get("right_wheel_joint", -1)

        # Set joint dynamics & enable motor control
        for j_idx in [self.left_wheel_idx, self.right_wheel_idx]:
            if j_idx >= 0:
                p.changeDynamics(self.robot_id, j_idx, lateralFriction=1.2, rollingFriction=0.001, spinningFriction=0.001)

    def reset_pose(self, x: float, y: float, yaw: float):
        """Reset robot base position, orientation and reset linear/angular velocities."""
        orn = p.getQuaternionFromEuler([0, 0, yaw])
        z_init = self.wheel_radius + 0.005
        p.resetBasePositionAndOrientation(self.robot_id, [x, y, z_init], orn)
        p.resetBaseVelocity(self.robot_id, [0, 0, 0], [0, 0, 0])

        # Reset joint positions & velocities
        for j_idx in [self.left_wheel_idx, self.right_wheel_idx]:
            if j_idx >= 0:
                p.resetJointState(self.robot_id, j_idx, targetValue=0.0, targetVelocity=0.0)

    def set_scenario_walls(self, walls: List[Tuple[float, float, float, float]]):
        """Create static box collision bodies in PyBullet for each wall/shelf in the scenario."""
        # Clear existing obstacles
        for b_id in self.obstacle_body_ids:
            p.removeBody(b_id)
        self.obstacle_body_ids.clear()

        wall_height = 1.5
        wall_thickness = 0.25

        for w in walls:
            x0, y0, x1, y1 = w
            cx = (x0 + x1) / 2.0
            cy = (y0 + y1) / 2.0
            dx = x1 - x0
            dy = y1 - y0
            length = math.hypot(dx, dy)
            if length < 0.05:
                continue
            angle = math.atan2(dy, dx)

            # PyBullet halfExtents are half the dimensions [L/2, T/2, H/2]
            col_shape = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=[length / 2.0, wall_thickness / 2.0, wall_height / 2.0]
            )
            orn = p.getQuaternionFromEuler([0, 0, angle])
            body_id = p.createMultiBody(
                baseMass=0,  # 0 mass means static immovable body
                baseCollisionShapeIndex=col_shape,
                basePosition=[cx, cy, wall_height / 2.0],
                baseOrientation=orn
            )
            self.obstacle_body_ids.append(body_id)

    def apply_motor_control(self, cmd_vx: float, cmd_vy: float, cmd_wz: float, max_torque: float = 60.0):
        """
        Drive wheels using native PyBullet joint motor velocity & torque control.
        Calculates differential wheel angular velocities and commands the motors.
        """
        if self.left_wheel_idx < 0 or self.right_wheel_idx < 0:
            return

        # Differential drive inverse kinematics
        v_left = cmd_vx - (cmd_wz * self.track_width / 2.0)
        v_right = cmd_vx + (cmd_wz * self.track_width / 2.0)

        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius

        p.setJointMotorControl2(
            bodyUniqueId=self.robot_id,
            jointIndex=self.left_wheel_idx,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=omega_left,
            force=max_torque
        )
        p.setJointMotorControl2(
            bodyUniqueId=self.robot_id,
            jointIndex=self.right_wheel_idx,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=omega_right,
            force=max_torque
        )

    def step_physics(self) -> float:
        """Step the PyBullet physics simulation and measure exact execution latency."""
        t0 = time.perf_counter()
        p.stepSimulation()
        dt_ms = (time.perf_counter() - t0) * 1000.0
        self.last_step_ms = round(dt_ms, 3)
        self.physics_step_count += 1
        return self.last_step_ms

    def get_robot_state(self) -> Dict[str, Any]:
        """
        Retrieve base pose, linear/angular velocity, and joint motor states from PyBullet.
        """
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        lin_vel, ang_vel = p.getBaseVelocity(self.robot_id)
        euler = p.getEulerFromQuaternion(orn)
        yaw = euler[2]

        left_pos, left_vel, left_forces, left_torque = 0.0, 0.0, [0.0], 0.0
        right_pos, right_vel, right_forces, right_torque = 0.0, 0.0, [0.0], 0.0

        if self.left_wheel_idx >= 0:
            js_l = p.getJointState(self.robot_id, self.left_wheel_idx)
            left_pos, left_vel, left_torque = js_l[0], js_l[1], js_l[3]
        if self.right_wheel_idx >= 0:
            js_r = p.getJointState(self.robot_id, self.right_wheel_idx)
            right_pos, right_vel, right_torque = js_r[0], js_r[1], js_r[3]

        # Calculate wheel RPM and equivalent motor current
        rpm_l = (left_vel * 60.0) / (2.0 * math.pi)
        rpm_r = (right_vel * 60.0) / (2.0 * math.pi)

        # Planar body velocity (vx in robot frame)
        # Global vx, vy -> local vx, vy
        c, s = math.cos(yaw), math.sin(yaw)
        vx_local = lin_vel[0] * c + lin_vel[1] * s
        vy_local = -lin_vel[0] * s + lin_vel[1] * c

        return {
            "x": round(pos[0], 4),
            "y": round(pos[1], 4),
            "z": round(pos[2], 4),
            "theta": round(yaw, 4),
            "vx": round(vx_local, 3),
            "vy": round(vy_local, 3),
            "wz": round(ang_vel[2], 3),
            "left_wheel_rad": round(left_pos, 4),
            "right_wheel_rad": round(right_pos, 4),
            "left_wheel_speed": round(left_vel, 3),
            "right_wheel_speed": round(right_vel, 3),
            "left_motor_rpm": round(rpm_l, 1),
            "right_motor_rpm": round(rpm_r, 1),
            "left_motor_torque": round(abs(left_torque), 2),
            "right_motor_torque": round(abs(right_torque), 2),
            "physics_step_ms": self.last_step_ms
        }

    def raycast_lidar(self, num_beams: int = 360, range_max: float = 12.0, z_height: float = 0.35, noise_std: float = 0.005) -> List[float]:
        """
        Perform batch raycasting directly using PyBullet's C++ p.rayTestBatch API.
        Accurately hits all static walls, dynamic obstacles, and scenario boundaries.
        """
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        yaw = p.getEulerFromQuaternion(orn)[2]

        lidar_x = pos[0]
        lidar_y = pos[1]
        lidar_z = z_height

        from_positions = []
        to_positions = []

        angle_inc = (2.0 * math.pi) / num_beams
        for i in range(num_beams):
            angle = yaw - math.pi + i * angle_inc
            from_positions.append([lidar_x, lidar_y, lidar_z])
            to_positions.append([
                lidar_x + range_max * math.cos(angle),
                lidar_y + range_max * math.sin(angle),
                lidar_z
            ])

        # Execute high-speed C++ batch raytest
        ray_results = p.rayTestBatch(from_positions, to_positions)
        ranges = []

        for res in ray_results:
            hit_body_id = res[0]
            hit_fraction = res[2]

            # If hit something other than the robot itself
            if hit_body_id >= 0 and hit_body_id != self.robot_id:
                r = hit_fraction * range_max
                if noise_std > 0.0:
                    r += random.gauss(0.0, noise_std)
                r = max(0.05, min(range_max, r))
            else:
                r = range_max

            ranges.append(round(r, 3))

        return ranges

    def close(self):
        if self.client_id >= 0:
            p.disconnect(self.client_id)
            self.client_id = -1
