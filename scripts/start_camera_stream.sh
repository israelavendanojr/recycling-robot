#!/bin/bash
# Camera Streaming Script for Raspberry Pi 5 + ArduCam IMX708
# This script provides multiple streaming methods with automatic fallback

set -e

# Configuration
PORT=${2:-8554}
WIDTH=${3:-640}
HEIGHT=${4:-480}
FPS=${5:-10}
QUALITY=${6:-80}

# Files
PID_FILE="/tmp/camera_stream.pid"
LOG_FILE="/tmp/camera_stream.log"
STREAM_URL=""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}🎥 Camera Streaming Script for Raspberry Pi 5 + ArduCam IMX708${NC}"
echo "================================================================"

# Function to log messages
log() {
    echo -e "$1"
    echo "$(date): $1" >> "$LOG_FILE"
}

# Function to get Pi IP address
get_pi_ip() {
    hostname -I | awk '{print $1}'
}

# Function to check if stream is running
is_stream_running() {
    if [ -f "$PID_FILE" ]; then
        PID=$(cat "$PID_FILE")
        if kill -0 "$PID" 2>/dev/null; then
            return 0
        else
            rm -f "$PID_FILE"
        fi
    fi
    return 1
}

# Function to stop existing stream
stop_existing_stream() {
    if is_stream_running; then
        log "${YELLOW}🛑 Stopping existing stream...${NC}"
        PID=$(cat "$PID_FILE")
        kill "$PID" 2>/dev/null || true
        sleep 2
        kill -9 "$PID" 2>/dev/null || true
        rm -f "$PID_FILE"
        log "${GREEN}✅ Existing stream stopped${NC}"
    fi
}

# Function to check dependencies
check_dependencies() {
    log "${BLUE}📦 Checking dependencies...${NC}"
    
    MISSING_DEPS=()
    
    # Check essential tools
    if ! command -v ffmpeg &> /dev/null; then
        MISSING_DEPS+=("ffmpeg")
    fi
    
    if ! command -v rpicam-vid &> /dev/null; then
        MISSING_DEPS+=("rpicam-apps")
    fi
    
    if ! command -v v4l2-ctl &> /dev/null; then
        MISSING_DEPS+=("v4l-utils")
    fi
    
    if [ ${#MISSING_DEPS[@]} -gt 0 ]; then
        log "${RED}❌ Missing dependencies: ${MISSING_DEPS[*]}${NC}"
        log "Install with: sudo apt update && sudo apt install ${MISSING_DEPS[*]}"
        return 1
    fi
    
    log "${GREEN}✅ All dependencies available${NC}"
    return 0
}

# Function to find best video device
find_best_device() {
    log "${BLUE}🔍 Finding best video device...${NC}"
    
    # Method 1: Try rpicam first (recommended for Pi 5)
    if command -v rpicam-vid &> /dev/null; then
        log "${GREEN}✅ Using rpicam-vid (recommended for Pi 5)${NC}"
        echo "rpicam"
        return 0
    fi
    
    # Method 2: Find V4L2 device with MJPEG support
    for device in /dev/video*; do
        if [ -c "$device" ]; then
            if v4l2-ctl --device="$device" --list-formats-ext 2>/dev/null | grep -q "MJPG"; then
                log "${GREEN}✅ Found MJPEG-capable device: $device${NC}"
                echo "$device"
                return 0
            fi
        fi
    done
    
    # Method 3: Fallback to first available V4L2 device
    FIRST_DEVICE=$(ls /dev/video* 2>/dev/null | head -1)
    if [ -n "$FIRST_DEVICE" ]; then
        log "${YELLOW}⚠️  Using fallback device: $FIRST_DEVICE${NC}"
        echo "$FIRST_DEVICE"
        return 0
    fi
    
    log "${RED}❌ No suitable video device found${NC}"
    return 1
}

# Function to start rpicam stream
start_rpicam_stream() {
    log "${BLUE}🎬 Starting rpicam stream...${NC}"
    
    # Kill any existing streams
    stop_existing_stream
    
    # Start rpicam-vid with FFmpeg pipeline
    log "Starting rpicam-vid → FFmpeg pipeline..."
    
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
        "http://0.0.0.0:$PORT/feed.mjpg" \
        >> "$LOG_FILE" 2>&1 &
    
    FFMPEG_PID=$!
    echo $FFMPEG_PID > "$PID_FILE"
    
    # Wait a moment for stream to start
    sleep 3
    
    # Check if stream is running
    if kill -0 "$FFMPEG_PID" 2>/dev/null; then
        STREAM_URL="http://$(get_pi_ip):$PORT/feed.mjpg"
        log "${GREEN}✅ rpicam stream started successfully${NC}"
        log "🌐 Stream URL: $STREAM_URL"
        log "📝 Logs: tail -f $LOG_FILE"
        return 0
    else
        log "${RED}❌ rpicam stream failed to start${NC}"
        return 1
    fi
}

# Function to start V4L2 stream
start_v4l2_stream() {
    local device=$1
    log "${BLUE}🎬 Starting V4L2 stream from $device...${NC}"
    
    # Kill any existing streams
    stop_existing_stream
    
    # Try different input formats
    local formats=("mjpeg" "yuyv422" "yuv420p")
    local success=false
    
    for format in "${formats[@]}"; do
        log "Trying format: $format..."
        
        ffmpeg -hide_banner -loglevel warning \
            -f v4l2 \
            -input_format "$format" \
            -video_size "${WIDTH}x${HEIGHT}" \
            -framerate "$FPS" \
            -i "$device" \
            -c:v mjpeg -q:v "$QUALITY" -r "$FPS" \
            -listen 1 \
            "http://0.0.0.0:$PORT/feed.mjpg" \
            >> "$LOG_FILE" 2>&1 &
        
        FFMPEG_PID=$!
        echo $FFMPEG_PID > "$PID_FILE"
        
        # Wait a moment for stream to start
        sleep 3
        
        # Check if stream is running
        if kill -0 "$FFMPEG_PID" 2>/dev/null; then
            STREAM_URL="http://$(get_pi_ip):$PORT/feed.mjpg"
            log "${GREEN}✅ V4L2 stream started successfully with format: $format${NC}"
            log "🌐 Stream URL: $STREAM_URL"
            log "📝 Logs: tail -f $LOG_FILE"
            success=true
            break
        else
            log "${YELLOW}⚠️  Format $format failed, trying next...${NC}"
            rm -f "$PID_FILE"
        fi
    done
    
    if [ "$success" = false ]; then
        log "${RED}❌ All V4L2 formats failed${NC}"
        return 1
    fi
    
    return 0
}

# Function to start stream with automatic method selection
start_stream() {
    log "${BLUE}🚀 Starting camera stream...${NC}"
    
    # Check dependencies
    if ! check_dependencies; then
        return 1
    fi
    
    # Find best device/method
    BEST_METHOD=$(find_best_device)
    if [ $? -ne 0 ]; then
        log "${RED}❌ No suitable camera method found${NC}"
        return 1
    fi
    
    # Start stream based on method
    if [ "$BEST_METHOD" = "rpicam" ]; then
        if start_rpicam_stream; then
            return 0
        else
            log "${YELLOW}⚠️  rpicam failed, trying V4L2 fallback...${NC}"
        fi
    fi
    
    # Try V4L2 as fallback - find first available video device
    V4L2_DEVICE=""
    for device in /dev/video*; do
        if [ -c "$device" ]; then
            # Check if device supports MJPEG
            if v4l2-ctl --device="$device" --list-formats-ext 2>/dev/null | grep -q "MJPG"; then
                V4L2_DEVICE="$device"
                break
            fi
        fi
    done
    
    if [ -z "$V4L2_DEVICE" ]; then
        # Fallback to first available device
        V4L2_DEVICE=$(ls /dev/video* 2>/dev/null | head -1)
    fi
    
    if [ -n "$V4L2_DEVICE" ]; then
        if start_v4l2_stream "$V4L2_DEVICE"; then
            return 0
        fi
    fi
    
    log "${RED}❌ All streaming methods failed${NC}"
    return 1
}

# Function to stop stream
stop_stream() {
    log "${BLUE}🛑 Stopping camera stream...${NC}"
    
    if is_stream_running; then
        PID=$(cat "$PID_FILE")
        kill "$PID" 2>/dev/null || true
        sleep 2
        kill -9 "$PID" 2>/dev/null || true
        rm -f "$PID_FILE"
        log "${GREEN}✅ Stream stopped${NC}"
    else
        log "${YELLOW}⚠️  No stream running${NC}"
    fi
}

# Function to show status
show_status() {
    log "${BLUE}📊 Stream Status:${NC}"
    
    if is_stream_running; then
        PID=$(cat "$PID_FILE")
        log "${GREEN}✅ Stream is running (PID: $PID)${NC}"
        log "🌐 Stream URL: http://$(get_pi_ip):$PORT/feed.mjpg"
        log "📝 Logs: tail -f $LOG_FILE"
        
        # Show recent log entries
        if [ -f "$LOG_FILE" ]; then
            log "📋 Recent log entries:"
            tail -5 "$LOG_FILE" | while read -r line; do
                log "   $line"
            done
        fi
    else
        log "${RED}❌ Stream is not running${NC}"
    fi
}

# Function to test stream
test_stream() {
    log "${BLUE}🧪 Testing stream...${NC}"
    
    if ! is_stream_running; then
        log "${RED}❌ Stream is not running${NC}"
        return 1
    fi
    
    # Test HTTP connection
    if command -v curl &> /dev/null; then
        log "Testing HTTP connection..."
        if curl -s --max-time 5 "http://localhost:$PORT/feed.mjpg" > /dev/null; then
            log "${GREEN}✅ HTTP stream accessible${NC}"
        else
            log "${RED}❌ HTTP stream not accessible${NC}"
            return 1
        fi
    fi
    
    # Test browser access
    PI_IP=$(get_pi_ip)
    log "${GREEN}✅ Stream should be accessible at: http://$PI_IP:$PORT/feed.mjpg${NC}"
    log "   Try opening this URL in your browser"
    
    return 0
}

# Function to install dependencies
install_dependencies() {
    log "${BLUE}📦 Installing dependencies...${NC}"
    
    sudo apt-get update
    sudo apt-get install -y \
        ffmpeg \
        libcamera-apps \
        v4l-utils \
        curl
    
    log "${GREEN}✅ Dependencies installed${NC}"
}

# Function to show help
show_help() {
    echo "Usage: $0 [command] [options]"
    echo ""
    echo "Commands:"
    echo "  start [port] [width] [height] [fps] [quality]  - Start camera stream"
    echo "  stop                                           - Stop camera stream"
    echo "  restart                                        - Restart camera stream"
    echo "  status                                         - Show stream status"
    echo "  test                                           - Test stream accessibility"
    echo "  install                                        - Install dependencies"
    echo "  help                                           - Show this help"
    echo ""
    echo "Options:"
    echo "  port     - HTTP port (default: 8554)"
    echo "  width    - Video width (default: 640)"
    echo "  height   - Video height (default: 480)"
    echo "  fps      - Frame rate (default: 10)"
    echo "  quality  - JPEG quality 1-100 (default: 80)"
    echo ""
    echo "Examples:"
    echo "  $0 start                           # Start with defaults"
    echo "  $0 start 8080 1280 720 15 90      # Start with custom settings"
    echo "  $0 stop                            # Stop stream"
    echo "  $0 status                          # Show status"
    echo "  $0 test                            # Test stream"
}

# Main command handler
case "${1:-help}" in
    "start")
        start_stream
        ;;
    "stop")
        stop_stream
        ;;
    "restart")
        stop_stream
        sleep 2
        start_stream
        ;;
    "status")
        show_status
        ;;
    "test")
        test_stream
        ;;
    "install")
        install_dependencies
        ;;
    "help"|*)
        show_help
        ;;
esac
