# AMR Studio V4 · ROS 2 仿真与自主导航后端部署指南

本项目是 **AMR Studio V4** 的物理仿真与自主调度导航后端，基于 **ROS 2 Humble**、**Nav2 (Navigation 2)** 与轻量化 **Web 遥控调度台（Web Teleop Dashboard）** 构建。

可在 **树莓派 4B / 5（ARM64）**、**Ubuntu 22.04 LTS（x86_64 / ARM64）** 或 **Docker 容器** 中实现开箱即用的一键部署。

---

## 🌟 系统架构概览

```
+-------------------------------------------------------------------------+
|                  Web 调度与态势监控前端 (HTML5 / Canvas)                   |
|  - 60/100 FPS 航位推算平滑渲染 (Dead-Reckoning Latency-Aligned Extrapolator) |
|  - 4 套工业仓储地图实时无缝切换 (九宫格/窄巷道/长方形环线/标准十字仓)               |
|  - 非阻塞事件总线订阅与调试窗口 (SSE/WebSocket/Events Hub)                 |
|  - 树莓派 CPU/内存/温度全链路性能监视器 (psutil 进程级统计)                     |
|  - 仿真全流程黑匣子录制、时间轴回放与 JSON 格式数据导出                        |
+------------------------------------+------------------------------------+
                                     | HTTP REST & Telemetry (/api/*)
                                     v
+-------------------------------------------------------------------------+
|                Web Teleop 调度网桥服务 (web_teleop_server.py)             |
|  - 多场景 Dijkstra 拓扑路网导航与 Stanley 航向/偏航闭环导引控制器            |
|  - 拐弯前瞻梯形减速规划 (Trapezoidal Deceleration Blending)               |
|  - 遥测数据动态轻量化打包 (1.4 KB, 消除 Cloudflare 隧道延迟)                 |
|  - 全息飞行记录器引擎 (FlightRecorder 1200 帧时序环形缓冲)                  |
+------------------------------------+------------------------------------+
                                     | ROS 2 话题通信 (/cmd_vel, /odom, /scan)
                                     v
+-------------------------------------------------------------------------+
|              CModel AGV 物理与传感器仿真引擎 (agv_simulation.py)            |
|  - 3 大工业级底盘动力学模型: 两轮差速 (Diff-Drive) / 单舵轮叉车 / 双舵轮全向   |
|  - 2D 激光雷达实时几何求交仿真 (360° 视场, 障碍物/货架精准碰撞遮挡)          |
|  - 工业安全联锁 IO 与机械抱闸仿真 (急停/防撞触边/货载到位/声光塔灯)          |
|  - 0.4s~1.2s 工业级安全看门狗 (Watchdog Interlock)                       |
+-------------------------------------------------------------------------+
```

---

## 📦 部署方式一：Docker 一键部署（推荐）

适用于树莓派（Raspberry Pi 4B/5）、Ubuntu 服务器或任何支持 Docker 的 Linux 设备。

### 1. 安装 Docker 与 Docker Compose
```bash
# Ubuntu / Debian / Raspberry Pi OS:
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
# 重启或重新登录使 docker 组生效
```

### 2. 构建或拉取 Docker 镜像
在 `ros2_cmodel_agv` 目录下执行：
```bash
cd ros2_cmodel_agv

# 通过 Dockerfile 构建镜像 (国内源自动加速)
docker build -t ros:humble-nav2 -f Dockerfile .
```

### 3. 一键启动容器
通过 `docker-compose` 启动：
```bash
docker compose up -d
```

或者使用原生 `docker run` 命令启动（推荐使用 host 网络模式以便于局域网发现）：
```bash
docker run -d \
  --name ros2_agv_sim \
  --restart unless-stopped \
  --network host \
  -v $(pwd):/workspace \
  -e ROS_DOMAIN_ID=0 \
  -e PYTHONUNBUFFERED=1 \
  ros:humble-nav2 \
  /workspace/start_sim.sh
```

### 4. 检查运行状态与日志
```bash
# 查看容器状态
docker ps -f name=ros2_agv_sim

# 查看实时仿真与调度日志
docker logs -f ros2_agv_sim
```

---

## 💻 部署方式二：原生 ROS 2 环境部署

如果您已在本地或开发机上安装了原生 **ROS 2 Humble Desktop / Base**：

### 1. 安装依赖包
```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager \
  python3-psutil \
  python3-numpy \
  curl
```

### 2. 配置环境并启动仿真脚本
```bash
cd ros2_cmodel_agv

# 加载 ROS 2 环境
source /opt/ros/humble/setup.bash

# 赋予执行权限并运行总入口脚本
chmod +x start_sim.sh
./start_sim.sh
```

启动后控制台将输出：
```text
==========================================================
 Starting AMR Studio V4 - CModel ROS2 AGV Simulation & Nav2
 Web Mission Dispatcher Dashboard on: http://0.0.0.0:8088
==========================================================
```

---

## 🌐 部署方式三：通过 Cloudflare Tunnel 实现全球公网免公网 IP 访问

如果需要通过公网安全访问部署在树莓派或内网工控机上的调度台：

### 1. 安装 cloudflared
```bash
# ARM64 (树莓派):
curl -L --output cloudflared.deb https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm64.deb
sudo dpkg -i cloudflared.deb

# x86_64:
curl -L --output cloudflared.deb https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64.deb
sudo dpkg -i cloudflared.deb
```

### 2. 运行隧道代理
在 Cloudflare Zero Trust 控制台创建 Tunnel，或使用命令行快速启动：
```bash
# 映射本地 8088 端口至指定公网域名
cloudflared tunnel run --token <YOUR_TUNNEL_TOKEN>
```
当前公网已稳定接入：`https://agv.cloud-ai.work`

---

## 🛠️ 核心模块与文件清单

| 文件 / 目录 | 职责与功能说明 |
| :--- | :--- |
| `start_sim.sh` | 仿真总入口脚本：拉起动力学引擎、Nav2 Map Server 及 Web Teleop 调度服务 |
| `agv_simulation.py` | ROS 2 核心节点：底盘动力学解算、多线激光雷达几何求交、安全看门狗、`/odom` 与 TF 广播 |
| `web_teleop_server.py` | Python HTTP 调度网桥：REST API、Dijkstra 拓扑路网规划、Stanley 路径跟踪、飞行记录器 |
| `index.html` | 现代化全栈 Web 调度台：Canvas 60 FPS 动力学渲染、多地图切换、非阻塞调试、树莓派性能监控、全流程回放 |
| `robot.urdf` | AGV 机器人运动学树与连杆几何定义（Base, Wheels, LiDAR, Stereo Camera） |
| `robot_config.json` | 导出自 AMR Studio V4 的 CModel 硬件参数（轮距、轮径、减速比、最大速度、传感器位姿） |
| `nav2_cmodel_params.yaml`| ROS 2 Navigation 2 标准配置文件（代价地图 Costmap、DWB 控制器、Recoveries） |
| `models/` | 3 种底盘动力学驱动实现：两轮差速 (`diff_drive_chassis.py`)、单舵轮 (`single_steer_chassis.py`)、双舵轮 (`dual_steer_chassis.py`) |
| `planning/` | 路径规划器：Dijkstra 拓扑路网规划 (`dijkstra_planner.py`) 与 A* 栅格避障规划 (`a_star_planner.py`) |
| `sensors/` | 传感器仿真器：激光雷达点云 (`lidar_simulator.py`)、IO 开关量与顶升机构 (`io_simulator.py`)、视觉二维码识别 (`vision_simulator.py`) |
| `maps/` | 仿真仓储地图与 YAML 元数据文件 |
| `Dockerfile` | 标准化 Docker 容器构建定义文件 |
| `docker-compose.yml` | 标准化容器编排部署文件 |

---

## 📡 常用 API 接口规范

仿真后台提供完整的 RESTful 接口供外部系统（MES/WCS/调度系统）对接：

* **遥测状态**：`GET /api/telemetry`（高频轻量包）或 `GET /api/telemetry?full=1`（含全量场景几何）
* **下发导航目标**：`POST /api/navigate_to_pose`
  ```json
  { "x": 3.0, "y": 4.0, "yaw": 1.57 }
  ```
* **手动速度控制**：`POST /api/cmd_vel`
  ```json
  { "vx": 0.5, "vy": 0.0, "wz": 0.2 }
  ```
* **切换地图场景**：`POST /api/map_scenario`
  ```json
  { "scenario": "grid_9_square" }
  ```
  可选场景：`grid_9_square` (正方形九宫格), `narrow_aisle` (高密多巷道), `rect_loop` (长方形环线), `standard_cross` (标准十字仓)。
* **切换底盘模型**：`POST /api/chassis_type`
  ```json
  { "type": "diff_drive" }
  ```
  可选模型：`diff_drive`, `single_steer`, `dual_steer`。
* **回放会话列表**：`GET /api/replay/sessions`
* **提取回放时序数据**：`GET /api/replay/session?id=<SESSION_ID>`
* **导出回放 JSON 包**：`GET /api/replay/export?id=<SESSION_ID>`
* **订阅系统事件流**：`GET /api/events?since_id=0&limit=50`

---

## 🧪 自动化测试与验证

本项目遵守严格的工程规范，提供完整的 Playwright 端到端验证脚本：

```bash
# 验证 60/100 FPS 动力学连续性、时延补偿与公网性能：
node scratch/test_cloud_ai_motion.js
```
测试脚本将自动驱动真实无头浏览器，在高频运动中进行逐帧抓取与连续性审计，确保最大位移步长与角速度跳变严格处于安全阈值之内（Zero Discontinuity）。
