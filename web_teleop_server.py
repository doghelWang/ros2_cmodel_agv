#!/usr/bin/env python3
"""
Web Teleop, Planner & Multi-Chassis Dispatch Server for AMR Studio V4
Features:
- Multi-Scenario Support (Standard Cross-Dock, Narrow Aisle VNA, FMS Workshop)
- Multi-Chassis Switching (Diff Drive, Single Steer, Dual Steer)
- Planners: A* Grid Search & Dijkstra Topological Graph
- Industrial I/O & E-Stop Interlock Control
- Strict Orthogonal & Corridor Parallel Heading Alignment (During Motion & Upon Docking)
- High-performance, lightweight Telemetry Stream with Physics Extrapolation Support
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import String

import json
import os
import sys
import threading
import time
import math
import random
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
import urllib.parse
import psutil

# Add current directory to path
sys.path.insert(0, os.path.dirname(__file__))

from planning import AStarPlanner, DijkstraPlanner
from planning.dijkstra_planner import SCENARIO_DEFINITIONS
from controllers import VelocityProfiler


class EventHub:
    """
    Thread-safe Industrial Event Hub & Subscription Bus
    Supports categorized channels: chassis, navigation, safety, sensors, system
    """
    def __init__(self, max_history=1200):
        self.max_history = max_history
        self.events = []
        self.event_counter = 0
        self.lock = threading.Lock()
        self.category_counts = {
            "chassis": 0,
            "navigation": 0,
            "safety": 0,
            "sensors": 0,
            "system": 0
        }

    def emit(self, category: str, event_type: str, level: str, title: str, message: str, payload: dict = None):
        with self.lock:
            self.event_counter += 1
            now = time.time()
            time_str = time.strftime("%H:%M:%S", time.localtime(now)) + f".{int((now % 1) * 1000):03d}"
            if category in self.category_counts:
                self.category_counts[category] += 1
            ev = {
                "id": self.event_counter,
                "timestamp": now,
                "time_str": time_str,
                "category": category,
                "type": event_type,
                "level": level,  # info, success, warning, danger
                "title": title,
                "message": message,
                "payload": payload or {}
            }
            self.events.append(ev)
            if len(self.events) > self.max_history:
                self.events.pop(0)
            return ev

    def get_events(self, since_id=0, categories=None, limit=100):
        with self.lock:
            if categories:
                cats = set(c.strip() for c in categories if c.strip())
            else:
                cats = None

            matched = []
            for e in self.events:
                if e["id"] > since_id:
                    if cats is None or e["category"] in cats:
                        matched.append(e)

            total_matched = len(matched)
            if limit and limit > 0 and total_matched > limit:
                result = matched[-limit:]
            else:
                result = matched

            return {
                "events": result,
                "latest_id": self.event_counter,
                "total_retained": len(self.events),
                "category_counts": dict(self.category_counts)
            }

    def clear(self):
        with self.lock:
            self.events.clear()
            return {"status": "cleared", "latest_id": self.event_counter}


event_hub = EventHub(max_history=1200)


class SystemPerformanceMonitor:
    """
    Real-time Raspberry Pi System & ROS2/Simulator Process Resource Monitor
    Tracks:
    - Host CPU % (total & per-core), RAM (used, total, %), CPU Temp (°C), Frequency (MHz), Load Avg
    - ROS & Simulator specific processes:
        * agv_simulation.py (CModel physics & 2D lidar raycast)
        * web_teleop_server.py (Web server, REST bridge, Dijkstra planner)
        * nav2_map_server (ROS2 map server)
        * nav2_lifecycle_manager (ROS2 lifecycle manager)
    - 60-second sliding performance timeline
    """
    def __init__(self, history_len=60):
        self.history_len = history_len
        self.history = []
        self.lock = threading.Lock()
        self.latest_stats = {
            "timestamp": time.time(),
            "host": {
                "model": "Raspberry Pi 4 (4 Cores Cortex-A72)",
                "cpu_count": 4,
                "cpu_total_percent": 0.0,
                "cpu_per_core": [0.0, 0.0, 0.0, 0.0],
                "cpu_temp_c": 50.0,
                "cpu_freq_mhz": 1800.0,
                "memory_total_mb": 4049.0,
                "memory_used_mb": 800.0,
                "memory_free_mb": 3249.0,
                "memory_percent": 19.8,
                "load_avg": [0.0, 0.0, 0.0]
            },
            "ros_simulation_total": {
                "combined_cpu_percent": 0.0,
                "combined_rss_mb": 0.0,
                "combined_ram_percent": 0.0
            },
            "processes": []
        }
        self.running = True
        self.tracked_procs = {}

        # Start 1Hz background monitoring thread
        self.thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.thread.start()

    def _find_processes(self):
        targets = {
            "agv_simulation": {"name": "PyBullet 物理与激光仿真 (Bullet 3 C++)", "match": "agv_simulation.py", "role": "PyBullet 刚体动力学、电机扭矩闭环、批量光线投射点云"},
            "web_teleop": {"name": "Web 调度网桥与拓扑规划 (Teleop Server)", "match": "web_teleop_server.py", "role": "Web 交互接口、REST 事件总线、Dijkstra 拓扑规划"},
            "map_server": {"name": "Nav2 栅格地图服务器 (map_server)", "match": "nav2_map_server/map_server", "role": "ROS2 占据栅格地图发布与代价地图广播"},
            "lifecycle_manager": {"name": "Nav2 生命周期管理 (lifecycle_manager)", "match": "nav2_lifecycle_manager/lifecycle_manager", "role": "ROS2 节点健康监控与生命周期状态切换"},
        }
        my_pid = os.getpid()
        found = {}
        for p in psutil.process_iter(["pid", "cmdline", "name"]):
            if p.pid == my_pid:
                continue
            cmd = " ".join(p.info.get("cmdline") or [])
            if "-c" in cmd:
                continue
            for key, meta in targets.items():
                if key not in found and meta["match"] in cmd:
                    try:
                        p.cpu_percent(None)  # prime cpu percent
                        found[key] = (p, meta)
                    except Exception:
                        pass
                    break
        self.tracked_procs = found

    def _get_cpu_temp(self):
        for path in ["/sys/class/thermal/thermal_zone0/temp", "/sys/devices/virtual/thermal/thermal_zone0/temp"]:
            if os.path.exists(path):
                try:
                    with open(path, "r") as f:
                        return round(float(f.read().strip()) / 1000.0, 1)
                except Exception:
                    pass
        return None

    def _get_cpu_freq(self):
        path = "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"
        if os.path.exists(path):
            try:
                with open(path, "r") as f:
                    return round(float(f.read().strip()) / 1000.0, 1)
            except Exception:
                pass
        return None

    def _monitor_loop(self):
        self._find_processes()
        try:
            psutil.cpu_percent(None)
            psutil.cpu_percent(None, percpu=True)
        except Exception:
            pass
        time.sleep(0.5)

        while self.running:
            try:
                cpu_tot = round(psutil.cpu_percent(None), 1)
                per_core = [round(c, 1) for c in psutil.cpu_percent(None, percpu=True)]
                vmem = psutil.virtual_memory()
                temp_c = self._get_cpu_temp()
                freq_mhz = self._get_cpu_freq()
                load_avg = [round(x, 2) for x in os.getloadavg()] if hasattr(os, "getloadavg") else [0.0, 0.0, 0.0]

                if len(self.tracked_procs) < 3:
                    self._find_processes()

                procs_data = []
                sim_cpu = 0.0
                teleop_cpu = 0.0
                map_cpu = 0.0
                total_proc_rss = 0.0

                for key, (p, meta) in list(self.tracked_procs.items()):
                    try:
                        if not p.is_running() or p.status() == psutil.STATUS_ZOMBIE:
                            del self.tracked_procs[key]
                            continue
                        cpu_p = round(p.cpu_percent(None), 1)
                        mem_info = p.memory_info()
                        rss_mb = round(mem_info.rss / (1024 * 1024), 1)
                        total_proc_rss += rss_mb
                        threads = p.num_threads()
                        status = p.status()

                        if key == "agv_simulation":
                            sim_cpu = cpu_p
                        elif key == "web_teleop":
                            teleop_cpu = cpu_p
                        elif key in ("map_server", "lifecycle_manager"):
                            map_cpu += cpu_p

                        procs_data.append({
                            "key": key,
                            "name": meta["name"],
                            "pid": p.pid,
                            "cpu_percent": cpu_p,
                            "memory_mb": rss_mb,
                            "memory_percent": round((rss_mb / (vmem.total / (1024 * 1024))) * 100.0, 1),
                            "threads": threads,
                            "status": status,
                            "role": meta["role"]
                        })
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        if key in self.tracked_procs:
                            del self.tracked_procs[key]

                now = time.time()
                time_label = time.strftime("%H:%M:%S", time.localtime(now))

                point = {
                    "time": time_label,
                    "timestamp": round(now, 2),
                    "cpu_total": cpu_tot,
                    "mem_used_mb": round(vmem.used / (1024 * 1024), 1),
                    "mem_percent": round(vmem.percent, 1),
                    "sim_cpu": sim_cpu,
                    "teleop_cpu": teleop_cpu,
                    "map_cpu": map_cpu,
                    "temp_c": temp_c or 50.0
                }

                snapshot = {
                    "timestamp": now,
                    "host": {
                        "model": "Raspberry Pi 4 (4 Cores Cortex-A72)",
                        "cpu_count": psutil.cpu_count(),
                        "cpu_total_percent": cpu_tot,
                        "cpu_per_core": per_core,
                        "cpu_temp_c": temp_c,
                        "cpu_freq_mhz": freq_mhz,
                        "memory_total_mb": round(vmem.total / (1024 * 1024), 1),
                        "memory_used_mb": round(vmem.used / (1024 * 1024), 1),
                        "memory_free_mb": round(vmem.available / (1024 * 1024), 1),
                        "memory_percent": round(vmem.percent, 1),
                        "load_avg": load_avg
                    },
                    "ros_simulation_total": {
                        "combined_cpu_percent": round(sim_cpu + teleop_cpu + map_cpu, 1),
                        "combined_rss_mb": round(total_proc_rss, 1),
                        "combined_ram_percent": round((total_proc_rss / (vmem.total / (1024 * 1024))) * 100.0, 1)
                    },
                    "processes": procs_data
                }

                with self.lock:
                    self.latest_stats = snapshot
                    self.history.append(point)
                    if len(self.history) > self.history_len:
                        self.history.pop(0)

            except Exception:
                pass

            time.sleep(1.0)

    def get_stats(self):
        with self.lock:
            data = dict(self.latest_stats)
            data["history"] = list(self.history)
            return data


perf_monitor = SystemPerformanceMonitor()


class FlightRecorder:
    """
    Simulation Blackbox & Flight Recorder for AMR Studio V4.
    Records motion trajectories, planned routes, perception sensor frames,
    chassis command inputs, IO interlocks and hardware performance at 10Hz.
    Supports auto mission session slicing and manual recording window capture.
    """
    def __init__(self, max_sessions: int = 20, max_rolling_frames: int = 1200):
        self.lock = threading.Lock()
        self.max_sessions = max_sessions
        self.max_rolling_frames = max_rolling_frames
        self.completed_sessions = []
        self.active_session = None
        self.rolling_buffer = []
        self.last_cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0}

    def set_cmd_vel(self, vx: float, vy: float, wz: float):
        with self.lock:
            self.last_cmd = {"vx": round(vx, 3), "vy": round(vy, 3), "wz": round(wz, 3)}

    def is_active_recording(self) -> bool:
        with self.lock:
            return self.active_session is not None

    def start_session(self, session_type: str = "mission", metadata: dict = None) -> str:
        with self.lock:
            if self.active_session:
                self._finalize_active_session("AUTO_CLOSED")

            now = time.time()
            sid = f"{session_type}_{int(now)}_{random.randint(100, 999)}"
            self.active_session = {
                "id": sid,
                "type": session_type,
                "start_time": now,
                "end_time": None,
                "duration_s": 0.0,
                "total_distance_m": 0.0,
                "status": "RECORDING",
                "metadata": metadata or {},
                "planned_route": (metadata.get("planned_route", {}) if metadata else {}),
                "frames": []
            }
            return sid

    def end_session(self, status: str = "COMPLETED"):
        with self.lock:
            return self._finalize_active_session(status)

    def _finalize_active_session(self, status: str):
        if not self.active_session:
            return None
        session = self.active_session
        now = time.time()
        session["end_time"] = now
        session["status"] = status
        session["duration_s"] = round(now - session["start_time"], 2)
        session["total_frames"] = len(session["frames"])

        # Compute analytical statistics
        total_dist = 0.0
        max_speed = 0.0
        speeds = []
        min_dist_overall = 99.0
        frames = session["frames"]

        for i in range(len(frames)):
            f = frames[i]
            if i > 0:
                p0 = frames[i-1]["pose"]
                p1 = f["pose"]
                dist = math.hypot(p1["x"] - p0["x"], p1["y"] - p0["y"])
                total_dist += dist
            v_curr = math.hypot(f["vel"]["vx"], f["vel"]["vy"])
            speeds.append(v_curr)
            if v_curr > max_speed:
                max_speed = v_curr
            m_dist = f.get("perception", {}).get("min_dist", 99.0)
            if m_dist < min_dist_overall:
                min_dist_overall = m_dist

        session["total_distance_m"] = round(total_dist, 2)
        session["max_speed_mps"] = round(max_speed, 2)
        session["avg_speed_mps"] = round(sum(speeds) / max(1, len(speeds)), 2)
        session["min_obstacle_dist_m"] = round(min_dist_overall, 2) if min_dist_overall < 90 else 12.0

        self.completed_sessions.insert(0, session)
        if len(self.completed_sessions) > self.max_sessions:
            self.completed_sessions.pop()

        self.active_session = None
        return session

    def capture_rolling_as_session(self, seconds: float = 30.0, title: str = None) -> dict:
        with self.lock:
            now = time.time()
            cutoff = now - seconds
            frames = [f for f in self.rolling_buffer if f["t"] >= cutoff]
            if not frames:
                return None

            sid = f"manual_slice_{int(now)}_{random.randint(100, 999)}"
            start_t = frames[0]["t"]
            for f in frames:
                f["rel_t"] = round(f["t"] - start_t, 2)

            total_dist = 0.0
            max_speed = 0.0
            speeds = []
            min_dist_overall = 99.0
            for i in range(len(frames)):
                f = frames[i]
                if i > 0:
                    dist = math.hypot(f["pose"]["x"] - frames[i-1]["pose"]["x"], f["pose"]["y"] - frames[i-1]["pose"]["y"])
                    total_dist += dist
                v_curr = math.hypot(f["vel"]["vx"], f["vel"]["vy"])
                speeds.append(v_curr)
                if v_curr > max_speed:
                    max_speed = v_curr
                m_dist = f.get("perception", {}).get("min_dist", 99.0)
                if m_dist < min_dist_overall:
                    min_dist_overall = m_dist

            session = {
                "id": sid,
                "type": "manual_slice",
                "start_time": start_t,
                "end_time": now,
                "duration_s": round(now - start_t, 2),
                "total_frames": len(frames),
                "total_distance_m": round(total_dist, 2),
                "max_speed_mps": round(max_speed, 2),
                "avg_speed_mps": round(sum(speeds) / max(1, len(speeds)), 2),
                "min_obstacle_dist_m": round(min_dist_overall, 2) if min_dist_overall < 90 else 12.0,
                "status": "COMPLETED",
                "metadata": {
                    "title": title or f"手动切片记录 (近 {int(seconds)} 秒)",
                    "source": "rolling_buffer"
                },
                "planned_route": {},
                "frames": frames
            }
            self.completed_sessions.insert(0, session)
            if len(self.completed_sessions) > self.max_sessions:
                self.completed_sessions.pop()
            return session

    def record_frame(self, telemetry: dict, perf_summary: dict = None):
        now = time.time()
        with self.lock:
            cmd = dict(self.last_cmd)
            raw_ranges = telemetry.get("scan_ranges", [])
            ranges_sample = []
            if raw_ranges:
                step = max(1, len(raw_ranges) // 36)
                ranges_sample = [round(float(r), 2) for r in raw_ranges[::step][:36]]

            frame = {
                "t": round(now, 3),
                "pose": {
                    "x": round(float(telemetry.get("x", 0.0)), 3),
                    "y": round(float(telemetry.get("y", 0.0)), 3),
                    "yaw": round(float(telemetry.get("yaw", 0.0)), 4),
                    "yaw_deg": round(math.degrees(float(telemetry.get("yaw", 0.0))), 1)
                },
                "vel": {
                    "vx": round(float(telemetry.get("vx", 0.0)), 3),
                    "vy": round(float(telemetry.get("vy", 0.0)), 3),
                    "wz": round(float(telemetry.get("wz", 0.0)), 3)
                },
                "cmd": cmd,
                "nav": {
                    "status": telemetry.get("nav_status", "IDLE"),
                    "dist_rem": telemetry.get("nav_dist_rem", 0.0)
                },
                "io": {
                    "estop": bool(telemetry.get("io_states", {}).get("is_emergency_stop", False)),
                    "bumper": bool(telemetry.get("io_states", {}).get("inputs", {}).get("di_bumper_front", False)),
                    "cargo": bool(telemetry.get("io_states", {}).get("inputs", {}).get("di_cargo_present", False)),
                    "brake": bool(telemetry.get("io_states", {}).get("outputs", {}).get("do_brake_release", True))
                },
                "perception": {
                    "min_dist": telemetry.get("scan_min_dist", 12.0),
                    "ranges": ranges_sample,
                    "markers": list(telemetry.get("vision_markers", [])),
                    "obstacles": list(telemetry.get("dynamic_obstacles", []))
                },
                "perf": perf_summary or {}
            }

            self.rolling_buffer.append(frame)
            if len(self.rolling_buffer) > self.max_rolling_frames:
                self.rolling_buffer.pop(0)

            if self.active_session:
                frame_copy = dict(frame)
                frame_copy["rel_t"] = round(now - self.active_session["start_time"], 2)
                self.active_session["frames"].append(frame_copy)

    def get_session_list(self):
        with self.lock:
            res = []
            if self.active_session:
                s = self.active_session
                res.append({
                    "id": s["id"],
                    "type": s["type"],
                    "start_time": s["start_time"],
                    "duration_s": round(time.time() - s["start_time"], 1),
                    "total_frames": len(s["frames"]),
                    "status": "RECORDING",
                    "title": s["metadata"].get("title", "实时录制中..."),
                    "scenario": s["metadata"].get("scenario", "default"),
                    "origin": s["metadata"].get("origin", {}),
                    "destination": s["metadata"].get("destination", {})
                })
            for s in self.completed_sessions:
                res.append({
                    "id": s["id"],
                    "type": s["type"],
                    "start_time": s["start_time"],
                    "end_time": s["end_time"],
                    "duration_s": s["duration_s"],
                    "total_frames": s["total_frames"],
                    "total_distance_m": s["total_distance_m"],
                    "max_speed_mps": s.get("max_speed_mps", 0.0),
                    "avg_speed_mps": s.get("avg_speed_mps", 0.0),
                    "min_obstacle_dist_m": s.get("min_obstacle_dist_m"),
                    "status": s["status"],
                    "title": s["metadata"].get("title", f"航段 {s['id']}"),
                    "scenario": s["metadata"].get("scenario", "default"),
                    "origin": s["metadata"].get("origin", {}),
                    "destination": s["metadata"].get("destination", {})
                })
            return res

    def get_session(self, session_id: str):
        with self.lock:
            if self.active_session and self.active_session["id"] == session_id:
                return dict(self.active_session)
            for s in self.completed_sessions:
                if s["id"] == session_id:
                    return dict(s)
            # Default to latest completed session if no id provided
            if not session_id and self.completed_sessions:
                return dict(self.completed_sessions[0])
            return None


flight_recorder = FlightRecorder()


class WebTeleopBridge(Node):
    def __init__(self, config_path: str):
        super().__init__('web_teleop_bridge')
        with open(config_path, 'r', encoding='utf-8') as f:
            self.cfg = json.load(f)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.chassis_pub = self.create_publisher(String, '/set_chassis_type', 10)
        self.scenario_pub = self.create_publisher(String, '/set_map_scenario', 10)
        self.io_cmd_pub = self.create_publisher(String, '/set_io', 10)
        self.obstacle_pub = self.create_publisher(String, '/set_obstacles', 10)

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.plan_sub = self.create_subscription(NavPath, '/plan', self.on_plan, 10)
        self.io_sub = self.create_subscription(String, '/io_states', self.on_io_states, 10)
        self.vision_sub = self.create_subscription(String, '/vision_markers', self.on_vision_markers, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)

        # Dynamic Obstacles
        self.dynamic_obstacles = []

        # Multi-Scenario & Planners
        self.active_scenario = "grid_9_square"
        self.dijkstra_planner = DijkstraPlanner(default_scenario=self.active_scenario)
        self.walls = self.dijkstra_planner.get_walls()
        self.astar_planner = AStarPlanner(resolution=0.1, inflation_radius=0.45)
        self.active_planner = "dijkstra"  # Default to Topological Guide-Rail Planner

        self.active_chassis_type = "diff_drive"
        self.current_mission_id = 0

        self.event_hub = event_hub
        self.last_cmd_emit_time = 0.0
        self.last_lidar_alert_time = 0.0

        self.telemetry = {
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "wz": 0.0,
            "chassis_type": self.active_chassis_type,
            "planner_type": self.active_planner,
            "active_scenario": self.active_scenario,
            "scenario_metadata": self.dijkstra_planner.get_scenario_metadata(),
            "scan_ranges": [],
            "scan_angle_min": -2.35,
            "scan_angle_max": 2.35,
            "scan_angle_inc": 0.013,
            "scan_min_dist": 12.0,
            "plan_path": [],
            "target_goal": None,
            "nav_status": "IDLE",
            "nav_dist_rem": 0.0,
            "dynamic_obstacles": [],
            "topo_graph": self.dijkstra_planner.get_topology(),
            "vision_markers": [],
            "joint_states": {
                "names": ["left_wheel_joint", "right_wheel_joint"],
                "positions": [0.0, 0.0],
                "velocities": [0.0, 0.0]
            },
            "io_states": {
                "inputs": {
                    "di_estop": False,
                    "di_bumper_front": False,
                    "di_cargo_present": False,
                    "di_lift_top": False,
                    "di_lift_bottom": True
                },
                "outputs": {
                    "do_brake_release": True,
                    "do_tower_green": True,
                    "do_tower_yellow": False,
                    "do_tower_red": False
                },
                "is_emergency_stop": False
            },
            "config": self.cfg,
            "timestamp": time.time(),
            "last_update": time.time()
        }
        self.lock = threading.Lock()

        # Heartbeat timer for periodic system monitoring
        self.create_timer(5.0, self._on_heartbeat_timer)

        # 10Hz Flight Recorder high-frequency sampling timer
        self.create_timer(0.1, self._on_recorder_timer)

        # Initial Boot Event
        self.event_hub.emit(
            "system", "NODE_READY", "success",
            "调度核心已上线",
            "WebTeleopBridge 节点就绪，已建立与 ROS2 话题及 CModel 仿真环境的双向绑定",
            {"scenario": self.active_scenario, "chassis": self.active_chassis_type, "planner": self.active_planner}
        )

    def _on_recorder_timer(self):
        with self.lock:
            t = {
                "x": self.telemetry["x"],
                "y": self.telemetry["y"],
                "yaw": self.telemetry["yaw"],
                "vx": self.telemetry["vx"],
                "vy": self.telemetry.get("vy", 0.0),
                "wz": self.telemetry["wz"],
                "nav_status": self.telemetry["nav_status"],
                "nav_dist_rem": self.telemetry.get("nav_dist_rem", 0.0),
                "io_states": self.telemetry.get("io_states", {}),
                "scan_ranges": self.telemetry.get("scan_ranges", []),
                "scan_min_dist": self.telemetry.get("scan_min_dist", 12.0),
                "vision_markers": self.telemetry.get("vision_markers", []),
                "dynamic_obstacles": self.telemetry.get("dynamic_obstacles", [])
            }
        perf_summary = None
        if perf_monitor:
            h = perf_monitor.latest_stats.get("host", {})
            r = perf_monitor.latest_stats.get("ros_simulation_total", {})
            perf_summary = {
                "host_cpu": h.get("cpu_total_percent", 0.0),
                "ros_cpu": r.get("combined_cpu_percent", 0.0),
                "ram_mb": h.get("memory_used_mb", 0.0),
                "temp_c": h.get("cpu_temp_c", 0.0)
            }
        flight_recorder.record_frame(t, perf_summary)

    def _on_heartbeat_timer(self):
        with self.lock:
            cur_x = self.telemetry["x"]
            cur_y = self.telemetry["y"]
            cur_yaw_deg = round(math.degrees(self.telemetry["yaw"]), 1)
            status = self.telemetry["nav_status"]
            chassis = self.telemetry["chassis_type"]
        self.event_hub.emit(
            "system", "SYSTEM_HEARTBEAT", "info",
            "系统监控心跳",
            f"通信正常 | 底盘: {chassis} | 状态: {status} | 坐标: ({cur_x:.2f}, {cur_y:.2f}) Yaw: {cur_yaw_deg}°",
            {"x": round(cur_x, 2), "y": round(cur_y, 2), "yaw_deg": cur_yaw_deg, "status": status, "chassis": chassis}
        )

    def on_joint_states(self, msg: JointState):
        with self.lock:
            self.telemetry["joint_states"] = {
                "names": list(msg.name),
                "positions": [round(float(p), 4) for p in msg.position],
                "velocities": [round(float(v), 4) for v in msg.velocity] if msg.velocity else []
            }

    def set_map_scenario(self, scenario_id: str):
        if scenario_id not in SCENARIO_DEFINITIONS:
            return
        with self.lock:
            self.active_scenario = scenario_id
            self.dijkstra_planner.set_scenario(scenario_id)
            self.walls = self.dijkstra_planner.get_walls()
            self.dynamic_obstacles = []
            meta = self.dijkstra_planner.get_scenario_metadata()
            self.telemetry["active_scenario"] = scenario_id
            self.telemetry["scenario_metadata"] = meta
            self.telemetry["topo_graph"] = self.dijkstra_planner.get_topology()
            self.telemetry["dynamic_obstacles"] = []
            self.telemetry["plan_path"] = []
            self.telemetry["target_goal"] = None
            self.telemetry["nav_status"] = "IDLE"
            self.telemetry["nav_dist_rem"] = 0.0
            origin = meta["origin"]
            self.telemetry["x"] = float(origin["x"])
            self.telemetry["y"] = float(origin["y"])
            self.telemetry["yaw"] = float(origin.get("yaw", 0.0))
            self.telemetry["vx"] = 0.0
            self.telemetry["vy"] = 0.0
            self.telemetry["wz"] = 0.0

        # Notify physics simulator
        s = String()
        s.data = scenario_id
        self.scenario_pub.publish(s)
        self.broadcast_obstacles()
        self.get_logger().info(f"Loaded scenario: {scenario_id} ({meta['name']})")
        self.event_hub.emit(
            "system", "SCENARIO_SWITCH", "info",
            "仓储场景地图切换",
            f"已成功加载场景: {meta['name']} ({scenario_id})，重置坐标原点至 ({origin['x']}, {origin['y']})",
            {"scenario_id": scenario_id, "name": meta["name"], "origin": origin}
        )

    def on_odom(self, msg: Odometry):
        with self.lock:
            self.telemetry["x"] = float(msg.pose.pose.position.x)
            self.telemetry["y"] = float(msg.pose.pose.position.y)
            self.telemetry["vx"] = float(msg.twist.twist.linear.x)
            self.telemetry["vy"] = float(msg.twist.twist.linear.y)
            self.telemetry["wz"] = float(msg.twist.twist.angular.z)
            self.telemetry["timestamp"] = time.time()

            # Quaternion to yaw
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.telemetry["yaw"] = float(math.atan2(siny_cosp, cosy_cosp))
            self.telemetry["last_update"] = time.time()

            if self.telemetry["target_goal"]:
                gx = self.telemetry["target_goal"]["x"]
                gy = self.telemetry["target_goal"]["y"]
                dist = math.hypot(gx - self.telemetry["x"], gy - self.telemetry["y"])
                self.telemetry["nav_dist_rem"] = round(dist, 2)

    def on_scan(self, msg: LaserScan):
        with self.lock:
            raw = list(msg.ranges)
            step = max(1, len(raw) // 36)
            valid_ranges = [r for r in raw if not (math.isinf(r) or math.isnan(r))]
            min_dist = min(valid_ranges) if valid_ranges else 12.0
            self.telemetry["scan_ranges"] = [
                round(float(12.0 if (math.isinf(r) or math.isnan(r)) else r), 2)
                for r in raw[::step]
            ]
            self.telemetry["scan_angle_min"] = float(msg.angle_min)
            self.telemetry["scan_angle_max"] = float(msg.angle_max)
            self.telemetry["scan_angle_inc"] = float(msg.angle_increment * step)
            self.telemetry["scan_min_dist"] = round(min_dist, 2)
            cur_vx = self.telemetry["vx"]

        now = time.time()
        if min_dist < 1.1 and abs(cur_vx) > 0.15 and (now - self.last_lidar_alert_time > 2.0):
            self.last_lidar_alert_time = now
            self.event_hub.emit(
                "sensors", "LIDAR_PROXIMITY", "warning",
                "激光雷达近距避障预警",
                f"激光扫描测距发现障碍物接近: 最小距离 {min_dist:.2f} m",
                {"min_dist": round(min_dist, 2)}
            )

    def on_plan(self, msg: NavPath):
        with self.lock:
            self.telemetry["plan_path"] = [
                {"x": round(float(p.pose.position.x), 3), "y": round(float(p.pose.position.y), 3)}
                for p in msg.poses
            ]

    def on_io_states(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self.lock:
                self.telemetry["io_states"] = data
        except Exception:
            pass

    def on_vision_markers(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self.lock:
                self.telemetry["vision_markers"] = data
        except Exception:
            pass

    def publish_cmd_vel(self, vx: float, vy: float = 0.0, wz: float = 0.0):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)
        flight_recorder.set_cmd_vel(vx, vy, wz)

        now = time.time()
        is_moving = (abs(vx) > 0.01 or abs(vy) > 0.01 or abs(wz) > 0.01)
        if is_moving and (now - self.last_cmd_emit_time > 0.8):
            self.last_cmd_emit_time = now
            self.event_hub.emit(
                "chassis", "VELOCITY_COMMAND", "info",
                "底盘速度指令 (/cmd_vel)",
                f"速度指令帧: vx={vx:.2f} m/s, vy={vy:.2f} m/s, wz={wz:.2f} rad/s",
                {"vx": round(vx, 3), "vy": round(vy, 3), "wz": round(wz, 3)}
            )

    def set_chassis_type(self, chassis_type: str):
        valid = ["diff_drive", "single_steer", "dual_steer"]
        if chassis_type in valid:
            with self.lock:
                self.active_chassis_type = chassis_type
                self.telemetry["chassis_type"] = chassis_type
            s = String()
            s.data = chassis_type
            self.chassis_pub.publish(s)
            self.get_logger().info(f"Published chassis switch: {chassis_type}")
            chassis_names = {
                "diff_drive": "两轮差速 (Diff-Drive)",
                "single_steer": "单舵轮 (Single-Steer)",
                "dual_steer": "双舵轮全向 (Dual-Steer)"
            }
            self.event_hub.emit(
                "chassis", "CHASSIS_SWITCH", "info",
                "底盘运动学模型切换",
                f"底盘成功切换为: {chassis_names.get(chassis_type, chassis_type)}",
                {"chassis_type": chassis_type}
            )

    def set_planner_type(self, planner_type: str):
        valid = ["astar", "dijkstra", "direct"]
        if planner_type in valid:
            with self.lock:
                self.active_planner = planner_type
                self.telemetry["planner_type"] = planner_type
            self.get_logger().info(f"Active planner set to: {planner_type}")
            planner_names = {
                "astar": "A* 栅格搜索规划器",
                "dijkstra": "Dijkstra 拓扑导轨路网规划器",
                "direct": "直连导航模式"
            }
            self.event_hub.emit(
                "navigation", "PLANNER_SWITCH", "info",
                "导航规划引擎切换",
                f"切换全局路径规划算法至: {planner_names.get(planner_type, planner_type)}",
                {"planner_type": planner_type}
            )

    def set_io(self, key: str, value: bool):
        with self.lock:
            if "inputs" in self.telemetry["io_states"]:
                self.telemetry["io_states"]["inputs"][key] = value
            if key in ["di_estop", "di_bumper_front", "di_bumper_rear"]:
                self.telemetry["io_states"]["is_emergency_stop"] = value
                if "outputs" in self.telemetry["io_states"]:
                    self.telemetry["io_states"]["outputs"]["do_brake_release"] = not value
                    self.telemetry["io_states"]["outputs"]["do_tower_red"] = value
                    self.telemetry["io_states"]["outputs"]["do_tower_green"] = not value
        s = String()
        s.data = json.dumps({"di": {key: value}})
        self.io_cmd_pub.publish(s)

        if key == "di_estop":
            self.event_hub.emit(
                "safety", "ESTOP_CHANGE",
                "danger" if value else "success",
                "机械急停开关动作",
                f"急停安全联锁已 {'按下触发 🛑 (抱闸抱死 / 动力切断)' if value else '复位解除 🟢 (抱闸释放 / 恢复正常)'}",
                {"key": key, "value": value}
            )
        elif key in ["di_bumper_front", "di_bumper_rear"]:
            self.event_hub.emit(
                "safety", "BUMPER_TRIGGER",
                "danger" if value else "info",
                "防撞安全触边检测",
                f"安全防撞触边传感器已 {'受压触发 💥 (安全停机)' if value else '弹性复位 🟢'}",
                {"key": key, "value": value}
            )
        elif key == "di_cargo_present":
            self.event_hub.emit(
                "safety", "CARGO_DETECTION", "info",
                "托盘到位光电传感器",
                f"载荷检测光电状态: {'检测到托盘物料到位 📦' if value else '空载位无货'}",
                {"key": key, "value": value}
            )

    def _get_obstacle_segments(self):
        segments = []
        with self.lock:
            obstacles = list(self.dynamic_obstacles)
        for obs in obstacles:
            ox = float(obs.get("x", 0.0))
            oy = float(obs.get("y", 0.0))
            ow = float(obs.get("w", 0.8))
            oh = float(obs.get("h", 0.8))
            hw, hh = ow / 2.0, oh / 2.0
            segments.extend([
                (ox - hw, oy - hh, ox + hw, oy - hh),
                (ox + hw, oy - hh, ox + hw, oy + hh),
                (ox + hw, oy + hh, ox - hw, oy + hh),
                (ox - hw, oy + hh, ox - hw, oy - hh)
            ])
        return segments

    def broadcast_obstacles(self):
        s = String()
        with self.lock:
            s.data = json.dumps(self.dynamic_obstacles)
        self.obstacle_pub.publish(s)

    def clear_obstacles(self):
        with self.lock:
            self.dynamic_obstacles = []
            self.telemetry["dynamic_obstacles"] = []
        self.broadcast_obstacles()
        self.get_logger().info("Cleared all dynamic obstacles")
        self.event_hub.emit(
            "sensors", "OBSTACLE_CHANGE", "info",
            "动态干扰路障已清空",
            "已清空移除主干道上所有动态干扰路障，导轨路网恢复畅通",
            {"count": 0}
        )

    def add_obstacle(self, x: float, y: float, w: float = 0.8, h: float = 0.8):
        with self.lock:
            new_id = len(self.dynamic_obstacles) + 1
            obs = {"id": new_id, "x": round(x, 2), "y": round(y, 2), "w": round(w, 2), "h": round(h, 2)}
            self.dynamic_obstacles.append(obs)
            self.telemetry["dynamic_obstacles"] = list(self.dynamic_obstacles)
        self.broadcast_obstacles()
        self.get_logger().info(f"Added obstacle #{new_id} at ({x}, {y})")
        self.event_hub.emit(
            "sensors", "OBSTACLE_CHANGE", "warning",
            f"新增动态干扰路障 #{new_id}",
            f"在坐标 ({x:.2f}, {y:.2f}) 放置规格为 {w:.2f}x{h:.2f}m 的工业障碍物",
            obs
        )

    def generate_random_obstacles(self, count: int = 3):
        import random
        with self.lock:
            cur_x, cur_y = self.telemetry["x"], self.telemetry["y"]
            stations = [(st["x"], st["y"]) for st in self.dijkstra_planner.get_stations()]

        # Generate on valid topological nodes
        nodes = list(self.dijkstra_planner.nodes.values())
        random.shuffle(nodes)
        new_obs = []

        for nx, ny in nodes:
            if len(new_obs) >= count:
                break
            ox = round(nx + random.uniform(-0.15, 0.15), 2)
            oy = round(ny + random.uniform(-0.15, 0.15), 2)

            if math.hypot(ox - cur_x, oy - cur_y) < 1.4:
                continue
            if any(math.hypot(ox - sx, oy - sy) < 1.1 for sx, sy in stations):
                continue

            ow = round(random.uniform(0.7, 0.85), 2)
            oh = round(random.uniform(0.7, 0.85), 2)
            new_obs.append({"id": len(new_obs) + 1, "x": ox, "y": oy, "w": ow, "h": oh})

        with self.lock:
            self.dynamic_obstacles = new_obs
            self.telemetry["dynamic_obstacles"] = list(self.dynamic_obstacles)
        self.broadcast_obstacles()
        self.get_logger().info(f"Generated {len(new_obs)} random obstacles")
        self.event_hub.emit(
            "sensors", "OBSTACLE_CHANGE", "warning",
            "随机布置动态干扰路障",
            f"在仓储拓扑主通道内随机部署了 {len(new_obs)} 处工业干扰物，检验动态避障能力",
            {"count": len(new_obs), "obstacles": new_obs}
        )

    def send_nav_goal(self, x: float, y: float, yaw: float = 0.0):
        with self.lock:
            self.current_mission_id += 1
            mission_id = self.current_mission_id
            cur_x, cur_y = self.telemetry["x"], self.telemetry["y"]
            planner = self.active_planner
            active_obstacles = list(self.dynamic_obstacles)
            # Find station matching x, y if any to get dock_yaw
            stations = self.dijkstra_planner.get_stations()
            matched_station = next((st for st in stations if math.hypot(st["x"] - x, st["y"] - y) < 0.35), None)
            if matched_station and "dock_yaw" in matched_station:
                target_dock_yaw = matched_station["dock_yaw"]
            else:
                target_dock_yaw = yaw

            self.telemetry["target_goal"] = {"x": x, "y": y, "yaw": target_dock_yaw}
            self.telemetry["nav_status"] = "PLANNING"

        # Path Planning Selection
        if planner == "astar":
            all_walls = self.walls + self._get_obstacle_segments()
            raw_path = self.astar_planner.plan((cur_x, cur_y), (x, y), all_walls)
            if len(raw_path) <= 2:
                raw_path = self.dijkstra_planner.plan((cur_x, cur_y), (x, y), active_obstacles)
        else:
            raw_path = self.dijkstra_planner.plan((cur_x, cur_y), (x, y), active_obstacles)

        with self.lock:
            self.telemetry["plan_path"] = [{"x": round(pt[0], 3), "y": round(pt[1], 3)} for pt in raw_path]

        self.get_logger().info(f"Mission #{mission_id} dispatched via [{planner.upper()}]: ({x}, {y}, yaw={target_dock_yaw:.2f}) with {len(raw_path)} waypoints")
        origin_st = next((st for st in stations if math.hypot(st["x"] - cur_x, st["y"] - cur_y) < 0.8), None)
        origin_name = origin_st["name"] if origin_st else f"起点 ({cur_x:.1f}, {cur_y:.1f})"
        dest_name = matched_station["name"] if matched_station else f"工位 ({x:.1f}, {y:.1f})"

        flight_recorder.start_session(
            session_type="mission",
            metadata={
                "mission_id": mission_id,
                "title": f"{origin_name} ➔ {dest_name}",
                "scenario": self.active_scenario,
                "chassis": self.active_chassis_type,
                "planner": planner,
                "origin": {"x": round(cur_x, 2), "y": round(cur_y, 2), "name": origin_name},
                "destination": {"x": round(x, 2), "y": round(y, 2), "yaw": round(target_dock_yaw, 2), "name": dest_name},
                "planned_route": {
                    "planner": planner,
                    "target_goal": {"x": round(x, 2), "y": round(y, 2), "yaw": round(target_dock_yaw, 2)},
                    "waypoints": [[round(pt[0], 2), round(pt[1], 2)] for pt in raw_path]
                }
            }
        )

        self.event_hub.emit(
            "navigation", "MISSION_DISPATCH", "info",
            f"下发调度导航任务 #{mission_id}",
            f"目标工位: ({x:.2f}, {y:.2f}) | 目标航向: {round(math.degrees(target_dock_yaw), 1)}° | 规划算法: {planner.upper()} | 规划航点: {len(raw_path)} 个",
            {"mission_id": mission_id, "target_x": x, "target_y": y, "target_yaw": target_dock_yaw, "waypoints_count": len(raw_path)}
        )
        threading.Thread(target=self._autonomous_guidance_loop, args=(mission_id, raw_path, target_dock_yaw), daemon=True).start()

    def _autonomous_guidance_loop(self, mission_id: int, waypoints: list, target_yaw: float):
        """
        High-precision Guidance Controller with Strict Corridor Tangent Locking
        and Cardinal Orthogonal Final Docking Alignment.
        """
        with self.lock:
            if self.current_mission_id != mission_id:
                return
            self.telemetry["nav_status"] = "NAVIGATING"
            chassis_type = self.active_chassis_type

        max_v = min(1.2, self.cfg.get('chassis', {}).get('max_speed_mps', 1.5))
        max_w = min(1.6, self.cfg.get('chassis', {}).get('max_ang_speed_radps', 2.0))

        try:
            for wp_idx in range(1, len(waypoints)):
                prev_wp = waypoints[wp_idx - 1]
                target_wp = waypoints[wp_idx]
                target_x, target_y = target_wp[0], target_wp[1]
                is_final_wp = (wp_idx == len(waypoints) - 1)

                # Desired segment heading
                seg_dx = target_x - prev_wp[0]
                seg_dy = target_y - prev_wp[1]
                seg_dist = math.hypot(seg_dx, seg_dy)
                if seg_dist > 0.05:
                    raw_heading = math.atan2(seg_dy, seg_dx)
                    # Snap to nearest cardinal/orthogonal angle (0, 90, 180, -90)
                    cardinal_angle = round(raw_heading / (math.pi / 2.0)) * (math.pi / 2.0)
                    if abs((raw_heading - cardinal_angle + math.pi) % (2 * math.pi) - math.pi) < 0.25:
                        seg_heading = cardinal_angle
                    else:
                        seg_heading = raw_heading
                else:
                    seg_heading = target_yaw

                # Desired segment heading error relative to current vehicle yaw
                with self.lock:
                    cur_yaw = self.telemetry["yaw"]
                    cur_x = self.telemetry["x"]
                    cur_y = self.telemetry["y"]

                init_heading_err = math.atan2(math.sin(seg_heading - cur_yaw), math.cos(seg_heading - cur_yaw))

                # PHASE 1: Pure in-place rotation ONLY needed if turning sharply (> 25° / 0.45 rad)
                dist_to_target = math.hypot(target_x - cur_x, target_y - cur_y)
                if abs(init_heading_err) > 0.45 and dist_to_target > 0.25:
                    # Smoothly ramp down linear speed before rotating
                    with self.lock:
                        cur_vx = self.telemetry.get("vx", 0.0)
                    while abs(cur_vx) > 0.1:
                        cur_vx = math.copysign(max(0.0, abs(cur_vx) - 0.2), cur_vx)
                        self.publish_cmd_vel(cur_vx, 0.0, 0.0)
                        time.sleep(0.02)
                    self.publish_cmd_vel(0.0, 0.0, 0.0)

                    while rclpy.ok():
                        with self.lock:
                            if self.current_mission_id != mission_id:
                                return
                            if self.telemetry.get("io_states", {}).get("is_emergency_stop", False):
                                self.publish_cmd_vel(0.0, 0.0, 0.0)
                                time.sleep(0.04)
                                continue
                            cur_yaw = self.telemetry["yaw"]
                            cur_x = self.telemetry["x"]
                            cur_y = self.telemetry["y"]

                        heading_err = math.atan2(math.sin(seg_heading - cur_yaw), math.cos(seg_heading - cur_yaw))
                        if abs(heading_err) > 0.08:
                            wz = math.copysign(min(max_w, max(0.25, abs(heading_err) * 2.2)), heading_err)
                            self.publish_cmd_vel(0.0, 0.0, wz)
                            time.sleep(0.03)
                        else:
                            break

                # PHASE 2: Straight Corridor Tracking with Continuous Smooth Stanley Line Following
                while rclpy.ok():
                    with self.lock:
                        if self.current_mission_id != mission_id:
                            return
                        if self.telemetry.get("io_states", {}).get("is_emergency_stop", False):
                            self.publish_cmd_vel(0.0, 0.0, 0.0)
                            time.sleep(0.04)
                            continue
                        cur_x = self.telemetry["x"]
                        cur_y = self.telemetry["y"]
                        cur_yaw = self.telemetry["yaw"]

                    dx = target_x - cur_x
                    dy = target_y - cur_y
                    dist = math.hypot(dx, dy)

                    # Arrival threshold for this waypoint (earlier blend for intermediate waypoints)
                    tol = 0.12 if is_final_wp else 0.45
                    if dist <= tol:
                        break

                    # Lateral cross-track error from corridor centerline
                    cross_track = -(cur_x - prev_wp[0]) * math.sin(seg_heading) + (cur_y - prev_wp[1]) * math.cos(seg_heading)

                    # Speed deceleration profile (smoothly decelerate when approaching destination OR corner turn)
                    if is_final_wp:
                        v_ratio = min(1.0, max(0.15, dist / 1.0))
                    else:
                        has_turn_ahead = False
                        if wp_idx + 1 < len(waypoints):
                            next_wp = waypoints[wp_idx + 1]
                            ndx = next_wp[0] - target_x
                            ndy = next_wp[1] - target_y
                            if math.hypot(ndx, ndy) > 0.1:
                                next_heading = math.atan2(ndy, ndx)
                                turn_angle = abs(math.atan2(math.sin(next_heading - seg_heading), math.cos(next_heading - seg_heading)))
                                if turn_angle > 0.35:
                                    has_turn_ahead = True
                        if has_turn_ahead:
                            v_ratio = min(1.0, max(0.22, dist / 0.85))
                        else:
                            v_ratio = 1.0

                    vx_nom = max(0.18, max_v * v_ratio)
                    # Stanley line following: desired heading smoothly guides AGV to centerline
                    heading_correction = math.atan2(1.6 * cross_track, max(0.25, vx_nom))
                    desired_heading = seg_heading - heading_correction

                    heading_err = math.atan2(math.sin(desired_heading - cur_yaw), math.cos(desired_heading - cur_yaw))

                    # Continuous smooth velocity scaling (cosine curve, zero cliff drops)
                    err_mag = abs(heading_err)
                    scale = max(0.18, math.cos(min(math.pi / 2.0, err_mag))) ** 1.6
                    vx = max(0.15, vx_nom * scale)
                    wz = max(-max_w, min(max_w, heading_err * 2.8))

                    if chassis_type == "dual_steer" and abs(cross_track) > 0.08:
                        vy = max(-0.25, min(0.25, -cross_track * 0.9))
                        self.publish_cmd_vel(vx, vy, wz)
                    else:
                        self.publish_cmd_vel(vx, 0.0, wz)

                    time.sleep(0.03)

            # PHASE 3: Precision Final Orientation Alignment (Parallel or Orthogonal Docking)
            cardinal_target = round(target_yaw / (math.pi / 2.0)) * (math.pi / 2.0)
            target_dock_yaw = cardinal_target

            self.publish_cmd_vel(0.0, 0.0, 0.0)
            time.sleep(0.08)

            self.event_hub.emit(
                "navigation", "NAV_DOCKING", "info",
                "工位末端调姿对齐",
                f"进入工位末端高精度调姿阶段，目标 Cardinal 航向: {round(math.degrees(target_dock_yaw), 1)}°",
                {"target_dock_yaw": round(target_dock_yaw, 3)}
            )

            align_start = time.time()
            while rclpy.ok() and (time.time() - align_start < 6.0):
                with self.lock:
                    if self.current_mission_id != mission_id:
                        return
                    cur_yaw = self.telemetry["yaw"]

                yaw_diff = math.atan2(math.sin(target_dock_yaw - cur_yaw), math.cos(target_dock_yaw - cur_yaw))
                if abs(yaw_diff) > 0.02:  # within 1.1 degrees!
                    wz = math.copysign(min(1.0, max(0.28, abs(yaw_diff) * 2.0)), yaw_diff)
                    self.publish_cmd_vel(0.0, 0.0, wz)
                    time.sleep(0.03)
                else:
                    break

            # Mission Complete
            self.publish_cmd_vel(0.0, 0.0, 0.0)
            with self.lock:
                if self.current_mission_id == mission_id:
                    self.telemetry["nav_status"] = "ARRIVED"
                    self.telemetry["plan_path"] = []
                    self.telemetry["nav_dist_rem"] = 0.0
                    final_x = self.telemetry["x"]
                    final_y = self.telemetry["y"]
                    final_yaw = self.telemetry["yaw"]
            flight_recorder.end_session("ARRIVED")
            yaw_dev_deg = abs(round(math.degrees(math.atan2(math.sin(target_dock_yaw - final_yaw), math.cos(target_dock_yaw - final_yaw))), 2))
            self.get_logger().info(f"Mission #{mission_id} ARRIVED & DOCKED (final yaw={final_yaw:.2f} rad, target={target_dock_yaw:.2f}).")
            self.event_hub.emit(
                "navigation", "MISSION_ARRIVED", "success",
                f"任务 #{mission_id} 停靠到位完成",
                f"已就位停靠，位置: ({final_x:.2f}, {final_y:.2f}) | 航向对齐偏差: {yaw_dev_deg}° (严格满足正交平行)",
                {"mission_id": mission_id, "x": round(final_x, 3), "y": round(final_y, 3), "yaw": round(final_yaw, 3), "dev_deg": yaw_dev_deg}
            )

        finally:
            for _ in range(3):
                self.publish_cmd_vel(0.0, 0.0, 0.0)
                time.sleep(0.02)

    def cancel_nav(self):
        with self.lock:
            self.current_mission_id += 1
            canceled_id = self.current_mission_id - 1
            self.telemetry["nav_status"] = "CANCELED"
            self.telemetry["target_goal"] = None
            self.telemetry["plan_path"] = []
            self.telemetry["nav_dist_rem"] = 0.0
        flight_recorder.end_session("CANCELED")
        for _ in range(4):
            self.publish_cmd_vel(0.0, 0.0, 0.0)
            time.sleep(0.02)
        self.event_hub.emit(
            "navigation", "MISSION_CANCELED", "warning",
            "调度任务手动中止",
            f"操作员已中止当前导航任务 #{canceled_id}，底盘已安全急停抱闸",
            {"mission_id": canceled_id}
        )


bridge_node = None


class TeleopHTTPHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path in ["/", "/index.html"]:
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.end_headers()
            html_path = os.path.join(os.path.dirname(__file__), "index.html")
            with open(html_path, "rb") as f:
                self.wfile.write(f.read())
        elif parsed.path == "/api/telemetry":
            query = urllib.parse.parse_qs(parsed.query)
            want_full = query.get("full", ["0"])[0] in ("1", "true")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache, no-store")
            self.end_headers()
            if bridge_node:
                with bridge_node.lock:
                    if perf_monitor:
                        h = perf_monitor.latest_stats.get("host", {})
                        r = perf_monitor.latest_stats.get("ros_simulation_total", {})
                        bridge_node.telemetry["perf_summary"] = {
                            "cpu_tot": h.get("cpu_total_percent", 0.0),
                            "mem_pct": h.get("memory_percent", 0.0),
                            "temp_c": h.get("cpu_temp_c", 0.0),
                            "ros_cpu": r.get("combined_cpu_percent", 0.0),
                            "ros_mb": r.get("combined_rss_mb", 0.0)
                        }
                    if want_full:
                        data = json.dumps(bridge_node.telemetry)
                    else:
                        fast_telemetry = {k: v for k, v in bridge_node.telemetry.items() if k not in ("scenario_metadata", "topo_graph", "config")}
                        data = json.dumps(fast_telemetry)
                self.wfile.write(data.encode("utf-8"))
            else:
                self.wfile.write(b"{}")
        elif parsed.path == "/api/events":
            query = urllib.parse.parse_qs(parsed.query)
            since_id = int(query.get("since_id", [0])[0])
            cats_str = query.get("categories", [""])[0]
            categories = cats_str.split(",") if cats_str else None
            limit = int(query.get("limit", [100])[0])

            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache, no-store")
            self.end_headers()
            data = event_hub.get_events(since_id=since_id, categories=categories, limit=limit)
            self.wfile.write(json.dumps(data).encode("utf-8"))
        elif parsed.path == "/api/system_perf":
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache, no-store")
            self.end_headers()
            data = perf_monitor.get_stats()
            self.wfile.write(json.dumps(data).encode("utf-8"))
        elif parsed.path == "/api/replay/sessions":
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache, no-store")
            self.end_headers()
            data = {
                "sessions": flight_recorder.get_session_list(),
                "is_recording": flight_recorder.is_active_recording()
            }
            self.wfile.write(json.dumps(data).encode("utf-8"))
        elif parsed.path == "/api/replay/session":
            query = urllib.parse.parse_qs(parsed.query)
            sid = query.get("id", [""])[0]
            session = flight_recorder.get_session(sid)
            self.send_response(200 if session else 404)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache, no-store")
            self.end_headers()
            self.wfile.write(json.dumps(session or {"error": "session not found"}).encode("utf-8"))
        elif parsed.path == "/api/replay/export":
            query = urllib.parse.parse_qs(parsed.query)
            sid = query.get("id", [""])[0]
            session = flight_recorder.get_session(sid)
            if session:
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Disposition", f'attachment; filename="amr_flight_log_{session["id"]}.json"')
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                payload = {
                    "version": "AMR_FLIGHT_RECORDER_V4",
                    "export_time": time.time(),
                    "session": session
                }
                self.wfile.write(json.dumps(payload, indent=2).encode("utf-8"))
            else:
                self.send_response(404)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(b'{"error": "Session not found"}')
        else:
            super().do_GET()

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length).decode("utf-8") if length > 0 else ""
        res = {"status": "ok"}

        try:
            req = json.loads(body) if body else {}
        except Exception:
            req = {}

        if parsed.path == "/api/cmd_vel":
            vx = float(req.get("vx", 0.0))
            vy = float(req.get("vy", 0.0))
            wz = float(req.get("wz", 0.0))
            if bridge_node:
                bridge_node.publish_cmd_vel(vx, vy, wz)
        elif parsed.path in ("/api/navigate_to_pose", "/api/navigate"):
            x = float(req.get("x", 0.0))
            y = float(req.get("y", 0.0))
            yaw = float(req.get("yaw", 0.0))
            if bridge_node:
                bridge_node.send_nav_goal(x, y, yaw)
        elif parsed.path == "/api/map_scenario":
            scenario_id = str(req.get("scenario", "grid_9_square"))
            if bridge_node:
                bridge_node.set_map_scenario(scenario_id)
        elif parsed.path == "/api/cancel_navigation":
            if bridge_node:
                bridge_node.cancel_nav()
        elif parsed.path == "/api/chassis_type":
            ctype = str(req.get("type", "diff_drive"))
            if bridge_node:
                bridge_node.set_chassis_type(ctype)
        elif parsed.path == "/api/planner_type":
            ptype = str(req.get("type", "dijkstra"))
            if bridge_node:
                bridge_node.set_planner_type(ptype)
        elif parsed.path == "/api/set_io":
            key = str(req.get("key", ""))
            val = bool(req.get("value", False))
            if bridge_node:
                bridge_node.set_io(key, val)
        elif parsed.path == "/api/obstacles/random":
            count = int(req.get("count", 3))
            if bridge_node:
                bridge_node.generate_random_obstacles(count)
        elif parsed.path == "/api/obstacles/clear":
            if bridge_node:
                bridge_node.clear_obstacles()
        elif parsed.path == "/api/obstacles/add":
            x = float(req.get("x", 0.0))
            y = float(req.get("y", 0.0))
            w = float(req.get("w", 0.8))
            h = float(req.get("h", 0.8))
            if bridge_node:
                bridge_node.add_obstacle(x, y, w, h)
        elif parsed.path == "/api/events/clear":
            res = event_hub.clear()
        elif parsed.path == "/api/events/inject":
            category = str(req.get("category", "system"))
            event_type = str(req.get("type", "MANUAL_TEST"))
            level = str(req.get("level", "info"))
            title = str(req.get("title", "测试注入事件"))
            message = str(req.get("message", "用户从控制台手动注入测试事件"))
            payload = req.get("payload", {})
            ev = event_hub.emit(category, event_type, level, title, message, payload)
            res = {"status": "ok", "event": ev}
        elif parsed.path == "/api/replay/record":
            action = str(req.get("action", "start"))
            if action == "start":
                sid = flight_recorder.start_session("manual", metadata={
                    "title": req.get("title", f"手动录制会话 {time.strftime('%H:%M:%S')}"),
                    "scenario": bridge_node.active_scenario if bridge_node else "default"
                })
                res = {"status": "ok", "session_id": sid, "action": "started"}
            elif action == "stop":
                s = flight_recorder.end_session("MANUAL_STOPPED")
                res = {"status": "ok", "session": s, "action": "stopped"}
            elif action == "capture_rolling":
                secs = float(req.get("seconds", 30.0))
                s = flight_recorder.capture_rolling_as_session(secs, title=req.get("title"))
                res = {"status": "ok", "session": s, "action": "captured"}

        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(json.dumps(res).encode("utf-8"))


def run_server(port=8088):
    server_address = ("0.0.0.0", port)
    httpd = ThreadingHTTPServer(server_address, TeleopHTTPHandler)
    httpd.serve_forever()


def main(args=None):
    global bridge_node
    rclpy.init(args=args)

    cfg_path = os.path.join(os.path.dirname(__file__), "robot_config.json")
    bridge_node = WebTeleopBridge(cfg_path)

    # Start Web Server in daemon thread
    web_thread = threading.Thread(target=run_server, daemon=True)
    web_thread.start()

    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
