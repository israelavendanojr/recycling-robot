#!/bin/bash
set -euo pipefail

# Config
CAMERA_SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
STREAM_URL="http://localhost:8554/feed.mjpg"   # change to /stream.mjpg if your script uses that path
BACKEND_DIR="$(cd "$CAMERA_SCRIPT_DIR/.." && pwd)"

# 1) Start camera stream if not running (prefer browser_stream.py to avoid V4L2)
if ! pgrep -f "rpicam-vid|ffmpeg|browser_stream.py|start_camera_stream.sh" >/dev/null 2>&1; then
  echo "Starting camera stream..."
  if [ -f "$CAMERA_SCRIPT_DIR/browser_stream.py" ]; then
    nohup python3 "$CAMERA_SCRIPT_DIR/browser_stream.py" 8554 >/tmp/browser_stream.log 2>&1 &
  elif [ -x "$CAMERA_SCRIPT_DIR/start_camera_stream.sh" ]; then
    FORCE_RPICAM=1 "$CAMERA_SCRIPT_DIR/start_camera_stream.sh" start || true
  elif [ -x "$CAMERA_SCRIPT_DIR/working_camera_stream.sh" ]; then
    "$CAMERA_SCRIPT_DIR/working_camera_stream.sh" || true
  else
    echo "No camera stream script found in $CAMERA_SCRIPT_DIR" >&2
    exit 1
  fi
else
  echo "Camera stream appears to be running."
fi

# 2) Wait until stream is reachable
printf "Waiting for camera stream";
for i in {1..20}; do
  if curl -fsI "$STREAM_URL" >/dev/null 2>&1; then
    echo "\nStream is up: $STREAM_URL"
    break
  fi
  printf "."; sleep 1
  if [ "$i" -eq 20 ]; then
    echo "\nTimeout waiting for stream at $STREAM_URL" >&2
    exit 2
  fi
done

# 3) Ensure config points to host stream via docker host alias
#    (edit backend/src/recycling_robot/config/camera.yaml if you need to change the path)

# 4) Bring up ROS2 stack in Docker
cd "$BACKEND_DIR"
export COMPOSE_DOCKER_CLI_BUILD=1
export DOCKER_BUILDKIT=1

docker compose up --build -d

echo "\n✅ ROS2 stack is up."
echo "- Web dashboard: http://<pi-ip>:8000"
echo "- Camera stream: $STREAM_URL"
