#!/bin/bash
set -e

PORT=${2:-8554}   # default port 8554
WIDTH=${3:-640}
HEIGHT=${4:-480}
FPS=${5:-10}

PID_FILE="/tmp/camera_stream.pid"
LOG_FILE="/tmp/camera_stream.log"

start_stream() {
  echo "🎬 Starting rpicam MJPEG stream on port $PORT ($WIDTH x $HEIGHT @ $FPS fps)..."

  # Kill old stream if still running
  [[ -f "$PID_FILE" ]] && kill "$(cat $PID_FILE)" 2>/dev/null || true
  sleep 1

  # Start stream pipeline
  rpicam-vid -t 0 \
    --width $WIDTH --height $HEIGHT \
    --framerate $FPS \
    --codec mjpeg \
    -n -o - \
  | ffmpeg -hide_banner -loglevel info \
      -f mjpeg -i - \
      -f mjpeg -q:v 5 -r $FPS \
      -listen 1 "http://0.0.0.0:$PORT/feed.mjpg" \
      >"$LOG_FILE" 2>&1 &

  echo $! > "$PID_FILE"
  echo "🌐 Stream running at: http://$(hostname -I | awk '{print $1}'):$PORT/feed.mjpg"
  echo "📝 Logs: tail -f $LOG_FILE"
}

stop_stream() {
  echo "🛑 Stopping camera stream..."
  if [[ -f "$PID_FILE" ]]; then
    kill "$(cat $PID_FILE)" 2>/dev/null || true
    rm -f "$PID_FILE"
    echo "✅ Stream stopped."
  else
    echo "⚠️ No stream running."
  fi
}

case "$1" in
  start) start_stream ;;
  stop) stop_stream ;;
  restart) stop_stream ;;
  *) echo "Usage: $0 {start|stop|restart} [PORT] [WIDTH] [HEIGHT] [FPS]" ;;
esac
