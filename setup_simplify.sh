#!/usr/bin/env bash
set -euo pipefail

# ------------------------------------------------------------------------------
# Simplify Docker setup for recycling-robot
# - Adjusts Dockerfile COPY paths to src/recycling_robot/{nodes,launch}
# - Writes sane .dockerignore (and ensures src/ is included)
# - Writes docker-compose.yml without 'version:' and maps camera devices
# - Makes backups with a timestamp suffix before overwriting
# Usage:
#   bash setup_simplify.sh            # just write files
#   bash setup_simplify.sh --run      # write files, then docker compose up --build
# ------------------------------------------------------------------------------

RUN_AFTER=false
if [[ "${1:-}" == "--run" ]]; then
  RUN_AFTER=true
fi

ts="$(date +%Y%m%d-%H%M%S)"

# Basic sanity check
need() { command -v "$1" >/dev/null 2>&1 || { echo "Missing command: $1"; exit 1; }; }
need docker
need bash

# Verify expected layout
if [[ ! -d "src/recycling_robot/nodes" ]]; then
  echo "ERROR: Expected src/recycling_robot/nodes not found. Run this from your repo root."
  exit 1
fi
if [[ ! -d "src/recycling_robot/launch" ]]; then
  echo "ERROR: Expected src/recycling_robot/launch not found. Run this from your repo root."
  exit 1
fi
if [[ ! -f "src/recycling_robot/launch/run_all.py" ]]; then
  echo "WARNING: src/recycling_robot/launch/run_all.py not found. The compose will still run,"
  echo "         but update CMD in Dockerfile later if your launcher has a different name."
fi

# Detect available /dev/video* devices for compose
video_devices=()
for dev in /dev/video0 /dev/video1 /dev/video2; do
  if [[ -e "$dev" ]]; then
    video_devices+=("$dev")
  fi
done

# Backup helper
backup_if_exists() {
  local f="$1"
  if [[ -e "$f" ]]; then
    cp -a "$f" "${f}.${ts}.bak"
    echo "Backed up $f -> ${f}.${ts}.bak"
  fi
}

# ------------------------------------------------------------------------------
# Write .dockerignore
# ------------------------------------------------------------------------------
backup_if_exists ".dockerignore"
cat > .dockerignore <<'EOF'
# Keep Docker build contexts slim but allow our source in
.git
__pycache__/
*.pyc
.DS_Store
venv/
node_modules/
build/
install/
log/
dataset/
models/

# Explicitly allow these (negation overrides ignores above)
!Dockerfile
!docker-compose.yml
!src/**
EOF
echo "Wrote .dockerignore"

# ------------------------------------------------------------------------------
# Write Dockerfile (with optional models copy if folder exists)
# ------------------------------------------------------------------------------
backup_if_exists "Dockerfile"

COPY_MODELS=""
if [[ -d "src/recycling_robot/models" ]]; then
  COPY_MODELS=$'COPY src/recycling_robot/models/ /workspace/src/recycling_robot/models/\n'
fi

cat > Dockerfile <<EOF
FROM ros:humble-ros-base

# System deps
RUN apt-get update && apt-get install -y --no-install-recommends \\
    python3-pip python3-dev build-essential \\
    ros-humble-cv-bridge ros-humble-sensor-msgs ros-humble-std-msgs \\
    ros-humble-std-srvs ros-humble-rclpy \\
    python3-opencv libopencv-dev \\
    v4l-utils udev \\
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \\
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \\
    gstreamer1.0-tools \\
 && rm -rf /var/lib/apt/lists/*

# Python deps (pin to versions you tested)
RUN pip3 install --no-cache-dir \\
    numpy==1.26.4 \\
    opencv-python==4.8.1.78 \\
    flask==2.3.3 \\
    pillow==10.0.1

# Optional torch CPU (fine for Pi dev containers or x86 host; can remove on Pi if heavy)
RUN pip3 install --no-cache-dir \\
    torch==2.0.1+cpu torchvision==0.15.2+cpu \\
    --index-url https://download.pytorch.org/whl/cpu

WORKDIR /workspace

# Copy your code
COPY src/recycling_robot/nodes/  /workspace/src/recycling_robot/nodes/
COPY src/recycling_robot/launch/ /workspace/src/recycling_robot/launch/
$COPY_MODELS
# Make scripts executable (best-effort)
RUN chmod +x /workspace/src/recycling_robot/nodes/*.py /workspace/src/recycling_robot/launch/*.py || true

# Default command: launch all nodes
CMD ["python3", "/workspace/src/recycling_robot/launch/run_all.py"]
EOF

echo "Wrote Dockerfile"

# ------------------------------------------------------------------------------
# Write docker-compose.yml
# ------------------------------------------------------------------------------
backup_if_exists "docker-compose.yml"

# Build device mapping YAML
devices_yaml=""
if ((${#video_devices[@]} > 0)); then
  devices_yaml="    devices:"
  for d in "${video_devices[@]}"; do
    devices_yaml+="
      - \"${d}:${d}\""
  done
fi

cat > docker-compose.yml <<EOF
services:
  recycling-robot:
    build:
      context: .
      dockerfile: Dockerfile
    ports:
      - "8000:8000"
$devices_yaml
    group_add:
      - "video"
    environment:
      - PYTHONUNBUFFERED=1
    # For live-edit instead of COPY at build, uncomment:
    # volumes:
    #   - ./src:/workspace/src
EOF

echo "Wrote docker-compose.yml"

# ------------------------------------------------------------------------------
# Summary + optional run
# ------------------------------------------------------------------------------
echo
echo "✅ Files updated:"
echo "   - .dockerignore"
echo "   - Dockerfile"
echo "   - docker-compose.yml"
echo
echo "Next steps:"
echo "  1) Build and run:   docker compose up --build"
echo "  2) Open dashboard:  http://localhost:8000"
echo
if \$RUN_AFTER; then
  echo "Running: docker compose up --build"
  docker compose up --build
fi
