FROM ros:humble-ros-base

# ===========================
# System dependencies
# ===========================
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    python3-opencv \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/ros2_ws

# Copy ROS sources
COPY src ./src

# ML + utilities
RUN pip install --no-cache-dir torch==2.0.1 torchvision==0.15.2 Pillow==11.3.0 requests==2.28.1

# ===========================
# Build ROS2 workspace
# ===========================
RUN . /opt/ros/humble/setup.sh && colcon build

# ===========================
# Default entrypoint
# ===========================
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch recycling_robot robot.launch.py"]
