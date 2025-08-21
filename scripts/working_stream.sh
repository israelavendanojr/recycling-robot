#!/bin/bash
# Working Camera Stream for Browser/Dashboard Integration

set -e

PORT=${1:-8554}
WIDTH=${2:-640}
HEIGHT=${3:-480}
FPS=${4:-10}

echo "🎥 Starting Browser-Compatible Camera Stream"
echo "==========================================="

# Kill any existing streams
pkill -f "rpicam-vid" 2>/dev/null || true
pkill -f "ffmpeg.*$PORT" 2>/dev/null || true
sleep 2

echo "Starting rpicam-vid → FFmpeg → HTTP stream..."

# Start the stream with proper HTTP headers for browser compatibility
rpicam-vid -t 0 \
    --width "$WIDTH" --height "$HEIGHT" \
    --framerate "$FPS" \
    --codec mjpeg \
    --nopreview \
    -o - \
| ffmpeg -hide_banner -loglevel warning \
    -f mjpeg -i - \
    -c copy \
    -f mjpeg \
    -content_type "multipart/x-mixed-replace; boundary=ffmpeg" \
    -listen 1 \
    -http_seekable 0 \
    -http_persistent 1 \
    "http://0.0.0.0:$PORT/feed.mjpg" &

STREAM_PID=$!
echo "Stream started with PID: $STREAM_PID"

# Wait for stream to initialize
sleep 5

# Get Pi IP
PI_IP=$(hostname -I | awk '{print $1}')

echo ""
echo "✅ Camera stream is running!"
echo "🌐 Stream URL: http://$PI_IP:$PORT/feed.mjpg"
echo "📝 Stream PID: $STREAM_PID"
echo ""
echo "Testing stream..."

# Test the stream
sleep 2
if curl -s -I "http://localhost:$PORT/feed.mjpg" | grep -q "multipart/x-mixed-replace"; then
    echo "✅ Stream is properly configured for browsers!"
else
    echo "⚠️  Stream may need a moment to initialize"
fi

echo ""
echo "🎯 For your React dashboard, use:"
echo "  <img src=\"http://$PI_IP:$PORT/feed.mjpg\" alt=\"Camera Feed\" />"
echo ""
echo "To stop: kill $STREAM_PID"

# Save PID for cleanup
echo "$STREAM_PID" > /tmp/working_stream.pid
