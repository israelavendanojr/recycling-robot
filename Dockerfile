FROM ros:humble-ros-base

# Install build dependencies, ROS2 packages, and system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-dev \
    build-essential \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-rclpy \
    python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY requirements-docker.txt /workspace/
RUN python3 -m pip install --no-cache-dir -r requirements-docker.txt

COPY . /workspace/

# Set up ROS2 environment with proper configuration
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc && \
    echo "export ROS_LOCALHOST_ONLY=1" >> /root/.bashrc