#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

echo "=========================================================="
echo " Starting AMR Studio V4 - CModel ROS2 AGV Simulation & Nav2"
echo " Web Mission Dispatcher Dashboard on: http://0.0.0.0:8088"
echo "=========================================================="

# 1. Start CModel AGV Simulation Core
python3 /workspace/agv_simulation.py &
SIM_PID=$!

# 2. Start Map Server and Lifecycle Manager (Warehouse Map)
if [ -f "/workspace/maps/warehouse_map.yaml" ]; then
    echo "Starting Map Server for warehouse_map.yaml..."
    ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/workspace/maps/warehouse_map.yaml -p use_sim_time:=False &
    MAP_PID=$!

    ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p autostart:=true -p node_names:="['map_server']" -p bond_timeout:=4.0 &
    LIFE_PID=$!
fi

# 3. Start Web Teleop & Autonomous Mission Dispatcher Server
python3 /workspace/web_teleop_server.py &
WEB_PID=$!

trap "kill $SIM_PID $MAP_PID $LIFE_PID $WEB_PID 2>/dev/null; exit 0" SIGINT SIGTERM

wait $SIM_PID $WEB_PID
