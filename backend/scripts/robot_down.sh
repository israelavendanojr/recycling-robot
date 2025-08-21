#!/bin/bash
set -euo pipefail

CAMERA_SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BACKEND_DIR="$(cd "$CAMERA_SCRIPT_DIR/.." && pwd)"

# 1) Bring down ROS2 stack
cd "$BACKEND_DIR"
docker compose down || true

# 2) Stop camera stream if using provided script(s)
if [ -x "$CAMERA_SCRIPT_DIR/start_camera_stream.sh" ]; then
  "$CAMERA_SCRIPT_DIR/start_camera_stream.sh" stop || true
fi

# Fallback: kill known processes
pkill -f "rpicam-vid|ffmpeg|browser_stream.py" 2>/dev/null || true

echo "✅ ROS2 stack and camera stream stopped."
