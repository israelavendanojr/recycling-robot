#!/bin/bash
# Simple Working Camera Stream for Raspberry Pi 5 + ArduCam IMX708

set -e

# Configuration
PORT=${1:-8080}
WIDTH=${2:-640}
HEIGHT=${3:-480}
FPS=${4:-10}

echo "🎥 Simple Camera Stream - Port $PORT"
echo "=================================="

# Kill any existing streams
pkill -f "rpicam-vid" 2>/dev/null || true
pkill -f "ffmpeg" 2>/dev/null || true
sleep 2

echo "Starting rpicam-vid with HTTP output..."

# Use rpicam-vid to stream directly to stdout, then use ffmpeg to serve HTTP
rpicam-vid -t 0 \
    --width "$WIDTH" --height "$HEIGHT" \
    --framerate "$FPS" \
    --inline \
    --nopreview \
    --codec mjpeg \
    -o - \
| ffmpeg -hide_banner -loglevel error \
    -f mjpeg -i - \
    -c copy \
    -f mjpeg \
    -content_type "multipart/x-mixed-replace; boundary=ffmpeg" \
    -listen 1 \
    -http_seekable 0 \
    "http://0.0.0.0:$PORT" &

STREAM_PID=$!
echo "Stream started with PID: $STREAM_PID"

# Wait for stream to initialize
sleep 5

# Get Pi IP
PI_IP=$(hostname -I | awk '{print $1}')

echo ""
echo "✅ Camera stream is running!"
echo "🌐 Stream URL: http://$PI_IP:$PORT"
echo "📝 Stream PID: $STREAM_PID"
echo ""
echo "Test in browser or with:"
echo "  curl http://$PI_IP:$PORT"
echo ""
echo "To stop:"
echo "  kill $STREAM_PID"

# Save PID for cleanup
echo "$STREAM_PID" > /tmp/simple_camera_stream.pid

# Test connection
echo ""
echo "Testing stream..."
if timeout 5s curl -s -I "http://localhost:$PORT" | grep -q "HTTP"; then
    echo "✅ Stream is accessible!"
else
    echo "⚠️  Stream test inconclusive - check manually in browser"
fi
