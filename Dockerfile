FROM ros:humble-ros-base

# System deps + ROS2 pkgs
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-dev build-essential \
    ros-humble-cv-bridge ros-humble-image-transport ros-humble-sensor-msgs \
    ros-humble-std-msgs ros-humble-geometry-msgs ros-humble-rclpy \
    ros-humble-web-video-server ros-humble-rosbridge-server ros-humble-action-msgs \
    python3-colcon-common-extensions \
    v4l-utils libcamera-tools \
    gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
 && rm -rf /var/lib/apt/lists/*

# Make pip-installed packages win over distro site-packages
ENV PYTHONPATH=/usr/local/lib/python3.10/dist-packages:$PYTHONPATH
ENV PYTHONNOUSERSITE=1 PIP_DISABLE_PIP_VERSION_CHECK=1

# Pin a compatible NumPy before the rest of your stack
RUN pip3 install --no-cache-dir "numpy==1.26.4"

WORKDIR /workspace
COPY requirements-docker.txt /workspace/

# Now install the rest (will reuse that NumPy)
RUN python3 -m pip install --no-cache-dir -r requirements-docker.txt

COPY . /workspace/

# Shell conveniences
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc && \
    echo "export ROS_LOCALHOST_ONLY=1" >> /root/.bashrc && \
    echo "export ROS_WS=/workspace" >> /root/.bashrc
