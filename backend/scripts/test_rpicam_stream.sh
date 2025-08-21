#!/bin/bash
# Simple test for rpicam streaming

set -e

echo "Testing rpicam streaming..."

# Configuration
PORT=8554
WIDTH=640
HEIGHT=480
FPS=10
QUALITY=80

# Kill any existing streams
pkill -f "ffmpeg.*http" 2>/dev/null || true
pkill -f "rpicam-vid" 2>/dev/null || true
sleep 2

echo "Starting rpicam-vid → FFmpeg pipeline..."

# Start rpicam-vid with FFmpeg pipeline
rpicam-vid -t 0 \
    --width "$WIDTH" --height "$HEIGHT" \
    --framerate "$FPS" \
    --codec mjpeg \
    --nopreview \
    -o - \
| ffmpeg -hide_banner -loglevel warning \
    -f mjpeg -i - \
    -c:v mjpeg -q:v "$QUALITY" -r "$FPS" \
    -listen 1 \
    "http://0.0.0.0:$PORT/feed.mjpg" &

FFMPEG_PID=$!
echo "Stream started with PID: $FFMPEG_PID"

# Wait a moment for stream to start
sleep 3

# Check if stream is running
if kill -0 "$FFMPEG_PID" 2>/dev/null; then
    PI_IP=$(hostname -I | awk '{print $1}')
    echo "✅ Stream started successfully!"
    echo "🌐 Stream URL: http://$PI_IP:$PORT/feed.mjpg"
    echo "📝 Stream PID: $FFMPEG_PID"
    echo ""
    echo "Test the stream in your browser or with:"
    echo "curl -s http://localhost:$PORT/feed.mjpg | wc -c"
    echo ""
    echo "To stop: kill $FFMPEG_PID"
else
    echo "❌ Stream failed to start"
    exit 1
fi
