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
 && rm -rf /var/lib/apt/lists/*

# Python deps (numpy from PyPI, torch from PyTorch index)
RUN pip3 install --no-cache-dir \
    numpy \
    opencv-python==4.8.1.78 \
    flask==2.3.3 \
    pillow==10.0.1 && \
    pip3 install torch==2.0.1 torchvision==0.15.2 \
    --index-url https://download.pytorch.org/whl/cpu

WORKDIR /workspace

# ROS env
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /root/.bashrc

# Default to bash (keeps container alive)
CMD ["bash"]
