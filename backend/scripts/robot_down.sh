#!/bin/bash

echo "🛑 Stopping Recycling Robot System..."

# Stop containers
echo "📦 Stopping containers..."
docker compose down

# Kill camera stream
if [ -f /tmp/camera_stream.pid ]; then
    PID=$(cat /tmp/camera_stream.pid)
    if kill -0 $PID 2>/dev/null; then
        echo "📹 Stopping camera stream (PID: $PID)..."
        kill $PID
        sleep 2
        kill -9 $PID 2>/dev/null || true
    fi
    rm -f /tmp/camera_stream.pid
fi

# Kill any remaining ffmpeg processes
pkill -f "ffmpeg.*8554" || true

echo "✅ System stopped"