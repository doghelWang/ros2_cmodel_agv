FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install Navigation2 stack, Map Server, Lifecycle Manager, PyBullet, and system tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    python3-psutil \
    python3-numpy \
    python3-pip \
    curl \
    && pip3 install --no-cache-dir pybullet \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Copy repository files
COPY . /workspace

# Set executable permissions
RUN chmod +x /workspace/start_sim.sh

# Web Teleoperation & Mission Dashboard port
EXPOSE 8088

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/workspace/start_sim.sh"]
