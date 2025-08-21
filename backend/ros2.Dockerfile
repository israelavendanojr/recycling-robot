FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    python3-opencv \
    curl \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Build ROS2 workspace
COPY ros2_ws ./
RUN . /opt/ros/humble/setup.sh && colcon build

# Source ROS2 in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/install/setup.bash" >> ~/.bashrc

CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch recycling_robot robot.launch.py"]