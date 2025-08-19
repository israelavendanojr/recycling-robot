FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-cups \
    python3-cupshelpers \
    # optional: add if you prefer system OpenCV over pip
    # python3-opencv \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY requirements.txt /workspace/
RUN python3 -m pip install --no-cache-dir -r requirements.txt

COPY . /workspace/

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
