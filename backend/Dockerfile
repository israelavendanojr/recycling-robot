FROM ros:humble-ros-base

# System deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-dev build-essential \
    ros-humble-cv-bridge ros-humble-sensor-msgs ros-humble-std-msgs \
    ros-humble-std-srvs ros-humble-rclpy \
    python3-opencv libopencv-dev \
    v4l-utils udev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    gstreamer1.0-tools \
    libcamera-apps \
    libcamera-dev \
    python3-picamera2 \
    python3-libcamera \
 && rm -rf /var/lib/apt/lists/*

# Copy requirements into image
COPY requirements-docker.txt /tmp/requirements-docker.txt

# Install pinned Python deps from requirements
RUN pip3 install --no-cache-dir -r /tmp/requirements-docker.txt \
    --extra-index-url https://download.pytorch.org/whl/cpu

WORKDIR /workspace

# Source ROS env automatically
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
