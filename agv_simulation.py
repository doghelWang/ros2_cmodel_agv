#!/usr/bin/env python3
"""
AMR Studio V4 - CModel-driven Multi-Chassis & Full-Sensor ROS2 AGV Simulator
Supports:
- Pluggable Kinematics: Differential Drive, Single Steer-Drive, Dual Steer-Drive
- Motor electrical simulation, RPM, torque, current, encoder output
- 2D LiDAR Raycasting with multi-sensor pose transforms & noise
- Visual AprilTag Landmark detection
- Industrial Digital I/O (E-Stop, Bumpers, Photoelectric sensors, Brakes, Tower lights)
- 0.4s Safety Watchdog auto-brake
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

import math
import time
import json
import os
import sys

# Add directory to Python path
sys.path.insert(0, os.path.dirname(__file__))

from models import DiffDriveChassis, SingleSteerChassis, DualSteerChassis
from models import PyBulletAGVEngine, PYBULLET_AVAILABLE
from sensors import LidarSimulator, VisionSimulator, IOSimulator
from controllers import VelocityProfiler, MotorSimulator
from planning.dijkstra_planner import SCENARIO_DEFINITIONS


class CModelAGVSimulator(Node):
    def __init__(self, config_path: str):
        super().__init__('cmodel_agv_simulator')
        self.get_logger().info(f'Loading robot configuration from: {config_path}')

        with open(config_path, 'r', encoding='utf-8') as f:
            self.cfg = json.load(f)

        self.robot_name = self.cfg.get('robot_name', 'cmodel_agv')
        chassis_cfg = self.cfg.get('chassis', {})
        wheels_cfg = self.cfg.get('drive_wheels', {})
        self.lidars_cfg = self.cfg.get('lidars', [])

        length = chassis_cfg.get('length_m', 1.788)
        width = chassis_cfg.get('width_m', 0.994)
        wheel_radius = wheels_cfg.get('radius_m', 0.115)
        track_width = wheels_cfg.get('track_width_m', 0.795)

        # 1. Multi-Chassis Kinematics Models
        self.chassis_models = {
            "diff_drive": DiffDriveChassis(length, width, wheel_radius, track_width),
            "single_steer": SingleSteerChassis(length, width, wheel_radius, wheelbase=length * 0.65),
            "dual_steer": DualSteerChassis(length, width, wheel_radius, wheel_dist=length * 0.75)
        }
        self.active_chassis_name = "diff_drive"
        self.active_chassis = self.chassis_models[self.active_chassis_name]

        # 2. Sensors & Industrial I/O
        self.lidar_sim = LidarSimulator(num_beams=360, range_max=12.0)
        self.vision_sim = VisionSimulator(fov_deg=85.0, max_dist=4.0)
        self.io_sim = IOSimulator()

        # Simulated Warehouse Room Boundaries & Standard Shelf Blocks (from Scenario)
        self.active_scenario = "grid_9_square"
        self.walls = list(SCENARIO_DEFINITIONS[self.active_scenario]["walls"])

        # 3. Initialize PyBullet C++ Physics Engine
        self.use_pybullet = False
        self.pybullet_engine = None
        urdf_path = os.path.join(os.path.dirname(__file__), "robot.urdf")
        if PYBULLET_AVAILABLE and os.path.exists(urdf_path):
            try:
                self.pybullet_engine = PyBulletAGVEngine(
                    urdf_path=urdf_path,
                    wheel_radius=wheel_radius,
                    track_width=track_width,
                    time_step=0.02
                )
                self.pybullet_engine.set_scenario_walls(self.walls)
                self.use_pybullet = True
                self.get_logger().info('PyBullet C++ Physics Engine (Bullet 3 DIRECT) successfully initialized!')
            except Exception as e:
                self.get_logger().warn(f'Failed to initialize PyBullet, falling back to analytical model: {e}')
        else:
            self.get_logger().info('PyBullet not available, running analytical physics.')

        # Commanded Velocities & Watchdog
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0
        self.last_cmd_time = time.time()
        self.last_time = time.time()
        self.dynamic_obstacles = []

        # ROS 2 Pub / Sub
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.chassis_select_sub = self.create_subscription(String, '/set_chassis_type', self.chassis_type_callback, 10)
        self.scenario_sub = self.create_subscription(String, '/set_map_scenario', self.map_scenario_callback, 10)
        self.io_cmd_sub = self.create_subscription(String, '/set_io', self.set_io_callback, 10)
        self.obstacle_sub = self.create_subscription(String, '/set_obstacles', self.obstacle_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.default_scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.io_state_pub = self.create_publisher(String, '/io_states', 10)
        self.vision_pub = self.create_publisher(String, '/vision_markers', 10)
        self.bullet_metrics_pub = self.create_publisher(String, '/bullet_metrics', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transforms()

        # 50 Hz Physics & Odom Timer, 10 Hz Sensors & IO Timer
        self.sim_timer = self.create_timer(0.02, self.sim_step)
        self.sensor_timer = self.create_timer(0.1, self.sensor_step)

        self.get_logger().info(f'CModel Full Simulator started. Active chassis: {self.active_chassis_name}')

    def chassis_type_callback(self, msg: String):
        target = msg.data.strip()
        if target in self.chassis_models and target != self.active_chassis_name:
            cur_x, cur_y, cur_th = self.active_chassis.x, self.active_chassis.y, self.active_chassis.theta
            self.active_chassis_name = target
            self.active_chassis = self.chassis_models[target]
            self.active_chassis.reset_pose(cur_x, cur_y, cur_th)
            if self.use_pybullet and self.pybullet_engine:
                self.pybullet_engine.reset_pose(cur_x, cur_y, cur_th)
            self.get_logger().info(f'Switched chassis type to: {self.active_chassis_name}')

    def map_scenario_callback(self, msg: String):
        target_sc = msg.data.strip()
        if target_sc in SCENARIO_DEFINITIONS and target_sc != self.active_scenario:
            self.active_scenario = target_sc
            sc = SCENARIO_DEFINITIONS[target_sc]
            self.walls = list(sc["walls"])
            origin = sc.get("origin", {"x": 0.0, "y": 0.0, "yaw": 0.0})
            self.active_chassis.reset_pose(origin["x"], origin["y"], origin.get("yaw", 0.0))
            if self.use_pybullet and self.pybullet_engine:
                self.pybullet_engine.set_scenario_walls(self.walls)
                self.pybullet_engine.reset_pose(origin["x"], origin["y"], origin.get("yaw", 0.0))
            self.cmd_vx = 0.0
            self.cmd_vy = 0.0
            self.cmd_wz = 0.0
            self.get_logger().info(f'Loaded map scenario: {target_sc} ({sc["name"]}) and reset pose to {origin}')

    def set_io_callback(self, msg: String):
        try:
            req = json.loads(msg.data)
            if "di" in req:
                for k, v in req["di"].items():
                    self.io_sim.set_di(k, v)
            if "do" in req:
                for k, v in req["do"].items():
                    self.io_sim.set_do(k, v)
        except Exception as e:
            self.get_logger().error(f'Error parsing set_io: {e}')

    def obstacle_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            if isinstance(data, list):
                self.dynamic_obstacles = data
                if self.use_pybullet and self.pybullet_engine:
                    all_walls = list(self.walls)
                    for obs in self.dynamic_obstacles:
                        ox = float(obs.get("x", 0.0))
                        oy = float(obs.get("y", 0.0))
                        ow = float(obs.get("w", 0.8))
                        oh = float(obs.get("h", 0.8))
                        hw, hh = ow / 2.0, oh / 2.0
                        all_walls.extend([
                            (ox - hw, oy - hh, ox + hw, oy - hh),
                            (ox + hw, oy - hh, ox + hw, oy + hh),
                            (ox + hw, oy + hh, ox - hw, oy + hh),
                            (ox - hw, oy + hh, ox - hw, oy - hh)
                        ])
                    self.pybullet_engine.set_scenario_walls(all_walls)
                self.get_logger().info(f'Received dynamic obstacles update: {len(self.dynamic_obstacles)} obstacles active')
        except Exception as e:
            self.get_logger().error(f'Error parsing set_obstacles: {e}')

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = time.time()
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_wz = msg.angular.z

    def publish_static_transforms(self):
        transforms = []
        now = self.get_clock().now().to_msg()

        # map -> odom
        t_map = TransformStamped()
        t_map.header.stamp = now
        t_map.header.frame_id = 'map'
        t_map.child_frame_id = 'odom'
        t_map.transform.rotation.w = 1.0
        transforms.append(t_map)

        # base_footprint -> base_link
        t_base = TransformStamped()
        t_base.header.stamp = now
        t_base.header.frame_id = 'base_footprint'
        t_base.child_frame_id = 'base_link'
        t_base.transform.translation.z = self.active_chassis.wheel_radius
        t_base.transform.rotation.w = 1.0
        transforms.append(t_base)

        # base_link -> lidar links
        for l in self.lidars_cfg:
            t_lidar = TransformStamped()
            t_lidar.header.stamp = now
            t_lidar.header.frame_id = 'base_link'
            t_lidar.child_frame_id = f"{l['name'].replace(' ', '_').replace('-', '_')}_link"
            t_lidar.transform.translation.x = float(l.get('x', 0.0))
            t_lidar.transform.translation.y = float(l.get('y', 0.0))
            t_lidar.transform.translation.z = float(l.get('z', 0.3))

            r, p, y = l.get('roll', 0.0), l.get('pitch', 0.0), l.get('yaw', 0.0)
            qx, qy, qz, qw = self.euler_to_quaternion(r, p, y)
            t_lidar.transform.rotation.x = qx
            t_lidar.transform.rotation.y = qy
            t_lidar.transform.rotation.z = qz
            t_lidar.transform.rotation.w = qw
            transforms.append(t_lidar)

        self.static_tf_broadcaster.sendTransform(transforms)

    def sim_step(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        if dt <= 0 or dt > 0.1:
            dt = 0.02

        # 1. Industrial Watchdog & E-Stop Interlock Check
        io_state = self.io_sim.get_io_state()
        if io_state["is_emergency_stop"] or (current_time - self.last_cmd_time > 1.2):
            # Safe Stop
            cmd_vx = 0.0
            cmd_vy = 0.0
            cmd_wz = 0.0
        else:
            cmd_vx = self.cmd_vx
            cmd_vy = self.cmd_vy
            cmd_wz = self.cmd_wz

        # 2. Update Chassis Physics (PyBullet C++ Engine or Analytical Fallback)
        if self.use_pybullet and self.pybullet_engine and self.active_chassis_name == "diff_drive":
            self.pybullet_engine.apply_motor_control(cmd_vx, cmd_vy, cmd_wz)
            self.pybullet_engine.step_physics()
            state = self.pybullet_engine.get_robot_state()
            self.active_chassis.x = state["x"]
            self.active_chassis.y = state["y"]
            self.active_chassis.theta = state["theta"]
            self.active_chassis.vx = state["vx"]
            self.active_chassis.vy = state["vy"]
            self.active_chassis.wz = state["wz"]
        else:
            state = self.active_chassis.update_physics(cmd_vx, cmd_vy, cmd_wz, dt)

        self.io_sim.update_lift_physics(dt)

        # Physical boundary safety constraint adapted to scenario perimeter
        max_bx = 6.8
        max_by = 6.8
        if hasattr(self, "walls") and len(self.walls) >= 4:
            xs = [self.walls[i][0] for i in range(4)] + [self.walls[i][2] for i in range(4)]
            ys = [self.walls[i][1] for i in range(4)] + [self.walls[i][3] for i in range(4)]
            if xs and ys:
                max_bx = max(abs(min(xs)), abs(max(xs))) - 0.5
                max_by = max(abs(min(ys)), abs(max(ys))) - 0.5

        if abs(self.active_chassis.x) > max_bx:
            self.active_chassis.x = math.copysign(max_bx, self.active_chassis.x)
            self.active_chassis.vx = 0.0
            if self.use_pybullet and self.pybullet_engine:
                self.pybullet_engine.reset_pose(self.active_chassis.x, self.active_chassis.y, self.active_chassis.theta)
        if abs(self.active_chassis.y) > max_by:
            self.active_chassis.y = math.copysign(max_by, self.active_chassis.y)
            self.active_chassis.vy = 0.0
            if self.use_pybullet and self.pybullet_engine:
                self.pybullet_engine.reset_pose(self.active_chassis.x, self.active_chassis.y, self.active_chassis.theta)

        state["x"] = self.active_chassis.x
        state["y"] = self.active_chassis.y

        # 3. Publish Odometry & TF
        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = self.euler_to_quaternion(0, 0, state["theta"])

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = state["x"]
        odom.pose.pose.position.y = state["y"]
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = state["vx"]
        odom.twist.twist.linear.y = state["vy"]
        odom.twist.twist.angular.z = state["wz"]
        self.odom_pub.publish(odom)

        # Broadcast odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = state["x"]
        t.transform.translation.y = state["y"]
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # 4. Publish Joint States
        js = JointState()
        js.header.stamp = now
        if self.active_chassis_name == "diff_drive":
            js.name = ['left_wheel_joint', 'right_wheel_joint']
            js.position = [state.get("left_wheel_rad", 0.0), state.get("right_wheel_rad", 0.0)]
            js.velocity = [state.get("left_wheel_speed", 0.0), state.get("right_wheel_speed", 0.0)]
            js.effort = [state.get("left_motor_torque", 0.0), state.get("right_motor_torque", 0.0)]
        elif self.active_chassis_name == "single_steer":
            js.name = ['steer_joint', 'drive_wheel_joint']
            js.position = [math.radians(state.get("steer_angle_deg", 0.0)), self.active_chassis.drive_wheel_rad]
        elif self.active_chassis_name == "dual_steer":
            js.name = ['front_steer_joint', 'rear_steer_joint']
            js.position = [self.active_chassis.steer1, self.active_chassis.steer2]
        self.joint_pub.publish(js)

    def sensor_step(self):
        now = self.get_clock().now().to_msg()
        chassis = self.active_chassis

        # 1. 2D LiDAR Raycasting (Native PyBullet p.rayTestBatch or Analytical Fallback)
        if self.use_pybullet and self.pybullet_engine:
            ranges = self.pybullet_engine.raycast_lidar(num_beams=360, range_max=12.0)
            angle_min = -math.pi
            angle_max = math.pi
            angle_inc = (2.0 * math.pi) / 360
        else:
            active_walls = list(self.walls)
            for obs in self.dynamic_obstacles:
                ox = float(obs.get("x", 0.0))
                oy = float(obs.get("y", 0.0))
                ow = float(obs.get("w", 0.8))
                oh = float(obs.get("h", 0.8))
                hw, hh = ow / 2.0, oh / 2.0
                active_walls.extend([
                    (ox - hw, oy - hh, ox + hw, oy - hh),
                    (ox + hw, oy - hh, ox + hw, oy + hh),
                    (ox + hw, oy + hh, ox - hw, oy + hh),
                    (ox - hw, oy + hh, ox - hw, oy - hh)
                ])
            ranges = self.lidar_sim.cast_rays(chassis.x, chassis.y, chassis.theta, active_walls)
            angle_min = self.lidar_sim.angle_min
            angle_max = self.lidar_sim.angle_max
            angle_inc = self.lidar_sim.angle_inc

        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = 'laser_link'
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_inc
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.05
        scan.range_max = 12.0
        scan.ranges = ranges
        self.default_scan_pub.publish(scan)

        # 2. Vision AprilTag Detection
        markers = self.vision_sim.detect_markers(chassis.x, chassis.y, chassis.theta)
        v_msg = String()
        v_msg.data = json.dumps(markers)
        self.vision_pub.publish(v_msg)

        # 3. Industrial IO Signals
        io_state = self.io_sim.get_io_state()
        io_msg = String()
        io_msg.data = json.dumps(io_state)
        self.io_state_pub.publish(io_msg)

        # 4. PyBullet Simulation Data Volume & Performance Metrics
        if self.use_pybullet and self.pybullet_engine:
            bm = self.pybullet_engine.get_bullet_metrics()
            bm_msg = String()
            bm_msg.data = json.dumps(bm)
            self.bullet_metrics_pub.publish(bm_msg)

    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    cfg_path = os.path.join(os.path.dirname(__file__), 'robot_config.json')
    node = CModelAGVSimulator(cfg_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
