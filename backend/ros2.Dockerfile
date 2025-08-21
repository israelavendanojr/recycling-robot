FROM ros:humble-ros-base
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-colcon-ros \
    build-essential \
    cmake \
    git \
    v4l-utils \
    ffmpeg \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-image-transport \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
# COPY the workspace from the repo root (compose context: .)
COPY ros2_ws /workspace/ros2_ws

SHELL ["/bin/bash", "-lc"]
RUN source /opt/ros/humble/setup.bash && \
    cd /workspace/ros2_ws && \
    colcon build --symlink-install

WORKDIR /workspace/ros2_ws
CMD ["bash", "-lc", "tail -f /dev/null"]
