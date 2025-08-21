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

# Python deps (pin to versions you tested)
RUN pip3 install --no-cache-dir \
    numpy==1.26.4 \
    opencv-python==4.8.1.78 \
    flask==2.3.3 \
    pillow==10.0.1

# Optional torch CPU (fine for Pi dev containers or x86 host; can remove on Pi if heavy)
RUN pip3 install --no-cache-dir \
    torch==2.0.1 torchvision==0.15.2 \
    --index-url https://download.pytorch.org/whl/cpu

WORKDIR /workspace

# Copy your code
COPY src/recycling_robot/nodes/  /workspace/src/recycling_robot/nodes/
COPY src/recycling_robot/launch/ /workspace/src/recycling_robot/launch/
COPY src/recycling_robot/models/ /workspace/src/recycling_robot/models/

# Make scripts executable (best-effort)
RUN chmod +x /workspace/src/recycling_robot/nodes/*.py /workspace/src/recycling_robot/launch/*.py || true

# Default command: launch all nodes
CMD ["python3", "/workspace/src/recycling_robot/launch/run_all.py"]
