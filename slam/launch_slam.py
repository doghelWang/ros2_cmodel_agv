#!/usr/bin/env python3
"""
SLAM Toolbox Launch & Management Node for CModel AGV
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    params_file = os.path.join(
        os.path.dirname(__file__),
        'slam_toolbox_params.yaml'
    )

    start_async_slam_toolbox_node = Node(
        parameters=[
            params_file,
            {'use_sim_time': False}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(start_async_slam_toolbox_node)
    return ld


if __name__ == '__main__':
    import sys
    import subprocess
    params = os.path.join(os.path.dirname(__file__), 'slam_toolbox_params.yaml')
    cmd = [
        "ros2", "run", "slam_toolbox", "async_slam_toolbox_node",
        "--ros-args", "--params-file", params, "-p", "use_sim_time:=False"
    ]
    print(f"Launching SLAM Toolbox: {' '.join(cmd)}")
    subprocess.run(cmd)
