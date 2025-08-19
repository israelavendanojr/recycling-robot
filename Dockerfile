FROM ros:humble-ros-base

# Install build dependencies and Python packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-opencv \
    python3-dev \
    build-essential \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY requirements-docker.txt /workspace/
RUN python3 -m pip install --no-cache-dir -r requirements-docker.txt

COPY . /workspace/

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc