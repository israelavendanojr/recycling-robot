FROM ubuntu:22.04

# ===========================
# System dependencies and ROS2 installation
# ===========================
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    python3-pip \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    python3-venv \
    build-essential \
    cmake \
    git \
    wget \
    v4l-utils \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Add ROS2 repository and install ROS2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install cv-bridge for ROS2 and GPIO libraries
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-cv-bridge \
    python3-rpi.gpio \
    python3-gpiozero \
    python3-pigpio \
    python3-lgpio \
    libgpiod-dev \
    gpiod \
    && rm -rf /var/lib/apt/lists/*

# Force pip packages to take precedence over system packages
ENV PYTHONPATH="/usr/local/lib/python3.10/dist-packages:$PYTHONPATH"

# Create a user with video and GPIO group access (matching host GPIO gid)
RUN groupadd -r robot && groupadd -g 993 gpio && useradd -r -g robot -G video,gpio robot
RUN mkdir -p /workspace/ros2_ws && chown -R robot:robot /workspace
RUN mkdir -p /home/robot && chown -R robot:robot /home/robot
RUN mkdir -p /shared && chown -R robot:robot /shared
ENV HOME=/home/robot

# Install Python packages with compatible versions
RUN pip install --no-cache-dir \
    numpy==1.24.3 \
    opencv-python==4.8.1.78 \
    torch==2.0.1 \
    torchvision==0.15.2 \
    Pillow==11.3.0 \
    flask==2.2.2 \
    flask-cors==3.0.10 \
    werkzeug==2.2.2 \
    jinja2==3.1.2 \
    click==8.1.3 \
    itsdangerous==2.1.2 \
    markupsafe==2.1.2

USER robot
WORKDIR /workspace/ros2_ws

# Set up ROS2 environment
ENV ROS_DISTRO=humble
ENV PYTHONPATH="/usr/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages"

# Create a verification script to check package versions
RUN echo '#!/usr/bin/env python3\n\
import sys\n\
import cv2\n\
import numpy as np\n\
print(f"OpenCV version: {cv2.__version__}")\n\
print(f"OpenCV path: {cv2.__file__}")\n\
print(f"NumPy version: {np.__version__}")\n\
print(f"NumPy path: {np.__file__}")\n\
print(f"Python path: {sys.path}")\n\
' > /home/robot/verify_packages.py && chmod +x /home/robot/verify_packages.py

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