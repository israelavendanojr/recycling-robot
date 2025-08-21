#!/bin/bash
set -e

echo "📷 Camera (ffmpeg) Stream"
echo "================================="

start_cam_stream() {
  local PORT=${1:-8554}
  echo "🎬 Starting ffmpeg MJPEG stream on port $PORT..."

  # Stop any existing streams
  pkill -f "ffmpeg.*video0" 2>/dev/null || true
  sleep 1

  # Start ffmpeg with listen mode
  ffmpeg -f v4l2 -input_format yuyv422 -video_size 640x480 -i /dev/video0 \
         -f mjpeg -q:v 5 -r 10 -listen 1 http://0.0.0.0:$PORT/feed.mjpg \
         >/tmp/camera_stream.log 2>&1 &

  echo $! > /tmp/camera_stream.pid
  echo "🌐 Stream URL: http://$(hostname -I | awk '{print $1}'):$PORT/feed.mjpg"
}

stop_all() {
  echo "🛑 Stopping..."
  [[ -f /tmp/camera_stream.pid ]] && kill "$(cat /tmp/camera_stream.pid)" 2>/dev/null || true
  rm -f /tmp/camera_stream.pid
  pkill -f "ffmpeg.*video0" 2>/dev/null || true
  echo "✅ Stopped"
}

main() {
  case "$1" in
    start) start_cam_stream "${2:-8554}" ;;
    stop)  stop_all ;;
    *) echo "Usage: $0 {start [port]|stop}" ;;
  esac
}

main "$@"
