#!/bin/bash
# host_camera_setup.sh - Run camera on Pi host, stream to Docker container

set -e

echo "🎥 Setting up ArduCam on Pi Host → Docker Pipeline"
echo "=================================================="

# Install dependencies on Pi host if needed
install_host_deps() {
    echo "📦 Installing host camera dependencies..."
    
    # Check if running on Raspberry Pi
    if ! grep -q "Raspberry Pi" /proc/cpuinfo; then
        echo "⚠️  Not a Raspberry Pi - camera setup may differ"
    fi
    
    # Install essential camera tools
    sudo apt-get update
    sudo apt-get install -y \
        v4l-utils \
        libgstreamer1.0-dev \
        gstreamer1.0-tools \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        ffmpeg
    
    echo "✅ Host dependencies installed"
}

# Test camera detection
test_camera() {
    echo "🔍 Testing camera detection..."
    
    # List video devices
    echo "Available video devices:"
    ls -la /dev/video* || echo "❌ No video devices found"
    
    # Test with v4l2
    for device in /dev/video*; do
        if [ -c "$device" ]; then
            echo "Testing $device:"
            v4l2-ctl --device="$device" --list-formats-ext 2>/dev/null || echo "  ❌ Cannot access $device"
        fi
    done
    
    # Try to capture a test frame
    echo "📸 Testing frame capture..."
    timeout 5s ffmpeg -f v4l2 -i /dev/video0 -vframes 1 -y /tmp/test_frame.jpg 2>/dev/null && \
        echo "✅ Successfully captured test frame" || \
        echo "❌ Failed to capture frame"
}

# Start camera streaming service
start_camera_stream() {
    local DEVICE=${1:-/dev/video0}
    local PORT=${2:-8554}
    
    echo "🎬 Starting camera stream from $DEVICE on port $PORT..."
    
    # Kill any existing streams
    pkill -f "ffmpeg.*$DEVICE" || true
    
    # Option 1: Simple HTTP MJPEG stream (recommended for development)
    echo "Starting MJPEG HTTP stream on port $PORT..."
    ffmpeg -f v4l2 -input_format mjpeg -video_size 640x480 -framerate 10 -i "$DEVICE" \
        -c:v mjpeg -f mjpeg \
        -listen 1 -http_port "$PORT" \
        http://0.0.0.0:$PORT/stream.mjpg &
    
    FFMPEG_PID=$!
    echo "📡 Camera stream started (PID: $FFMPEG_PID)"
    echo "🌐 Stream URL: http://$(hostname -I | awk '{print $1}'):$PORT/stream.mjpg"
    
    # Save PID for cleanup
    echo $FFMPEG_PID > /tmp/camera_stream.pid
}

# Stop camera stream
stop_camera_stream() {
    echo "🛑 Stopping camera stream..."
    if [ -f /tmp/camera_stream.pid ]; then
        PID=$(cat /tmp/camera_stream.pid)
        kill $PID 2>/dev/null || true
        rm /tmp/camera_stream.pid
    fi
    pkill -f "ffmpeg.*video" || true
    echo "✅ Camera stream stopped"
}

# Create systemd service for persistent streaming
create_camera_service() {
    local DEVICE=${1:-/dev/video0}
    local PORT=${2:-8554}
    
    echo "🔧 Creating systemd service for camera streaming..."
    
    sudo tee /etc/systemd/system/arducam-stream.service > /dev/null << EOF
[Unit]
Description=ArduCam MJPEG Stream
After=network.target
Wants=network.target

[Service]
Type=simple
User=pi
Group=video
WorkingDirectory=/home/pi
ExecStart=/usr/bin/ffmpeg -f v4l2 -input_format mjpeg -video_size 640x480 -framerate 10 -i $DEVICE -c:v mjpeg -f mjpeg -listen 1 -http_port $PORT http://0.0.0.0:$PORT/stream.mjpg
Restart=always
RestartSec=5
KillMode=mixed
TimeoutStopSec=5

[Install]
WantedBy=multi-user.target
EOF
    
    # Enable and start service
    sudo systemctl daemon-reload
    sudo systemctl enable arducam-stream.service
    
    echo "✅ Systemd service created"
    echo "   Start: sudo systemctl start arducam-stream"
    echo "   Stop:  sudo systemctl stop arducam-stream"
    echo "   Status: sudo systemctl status arducam-stream"
}

# Main menu
case "${1:-help}" in
    "install")
        install_host_deps
        ;;
    "test")
        test_camera
        ;;
    "start")
        start_camera_stream "${2:-/dev/video0}" "${3:-8554}"
        ;;
    "stop")
        stop_camera_stream
        ;;
    "service")
        create_camera_service "${2:-/dev/video0}" "${3:-8554}"
        ;;
    "help"|*)
        echo "Usage: $0 [command] [options]"
        echo ""
        echo "Commands:"
        echo "  install           - Install camera dependencies on Pi host"
        echo "  test              - Test camera detection and capture"
        echo "  start [device]    - Start camera stream (default: /dev/video0)"
        echo "  stop              - Stop camera stream"
        echo "  service [device]  - Create systemd service for persistent streaming"
        echo ""
        echo "Examples:"
        echo "  $0 install                    # Install dependencies"
        echo "  $0 test                       # Test camera"
        echo "  $0 start /dev/video0          # Start streaming"
        echo "  $0 service /dev/video0 8554   # Create systemd service"
        ;;
esac