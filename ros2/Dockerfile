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
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# Create a user with video group access
RUN groupadd -r robot && useradd -r -g robot -G video robot
RUN mkdir -p /workspace/ros2_ws && chown -R robot:robot /workspace
RUN mkdir -p /home/robot && chown -R robot:robot /home/robot
ENV HOME=/home/robot


# ML + utilities
RUN pip install --no-cache-dir torch==2.0.1 torchvision==0.15.2 Pillow==11.3.0 requests==2.28.1

USER robot
WORKDIR /workspace/ros2_ws

# Copy ROS sources
COPY --chown=robot:robot src ./src


# ===========================
# Build ROS2 workspace
# ===========================
RUN . /opt/ros/humble/setup.sh && colcon build

# ===========================
# Default entrypoint
# ===========================
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch recycling_robot robot.launch.py"]