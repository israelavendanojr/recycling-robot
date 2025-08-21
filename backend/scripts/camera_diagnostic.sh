#!/bin/bash
# Comprehensive Camera Diagnostic for Raspberry Pi 5 + ArduCam IMX708
# This script will diagnose and fix camera issues step by step

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging
LOG_FILE="/tmp/camera_diagnostic.log"
PID_FILE="/tmp/camera_diagnostic.pid"

echo -e "${BLUE}🔍 Comprehensive Camera Diagnostic for Raspberry Pi 5 + ArduCam IMX708${NC}"
echo "================================================================"
echo "Log file: $LOG_FILE"
echo ""

# Function to log messages
log() {
    echo -e "$1"
    echo "$(date): $1" >> "$LOG_FILE"
}

# Check if running as root
check_root() {
    if [[ $EUID -eq 0 ]]; then
        log "${YELLOW}⚠️  Running as root - this might cause permission issues${NC}"
        log "   Consider running as regular user with sudo when needed"
    fi
}

# Check system information
check_system() {
    log "${BLUE}📋 System Information:${NC}"
    
    # Check if this is a Raspberry Pi
    if grep -q "Raspberry Pi" /proc/cpuinfo; then
        log "${GREEN}✅ Raspberry Pi detected${NC}"
        
        # Get Pi model
        PI_MODEL=$(grep "Model" /proc/cpuinfo | head -1 | cut -d: -f2 | xargs)
        log "   Model: $PI_MODEL"
        
        # Check OS version
        if [ -f /etc/os-release ]; then
            OS_VERSION=$(grep "PRETTY_NAME" /etc/os-release | cut -d'"' -f2)
            log "   OS: $OS_VERSION"
        fi
    else
        log "${RED}❌ Not a Raspberry Pi - camera setup may differ${NC}"
    fi
    
    # Check kernel version
    KERNEL_VERSION=$(uname -r)
    log "   Kernel: $KERNEL_VERSION"
    
    # Check available memory
    MEMORY=$(free -h | grep "Mem:" | awk '{print $2}')
    log "   Memory: $MEMORY"
}

# Check camera hardware detection
check_hardware() {
    log "${BLUE}🔧 Camera Hardware Detection:${NC}"
    
    # Method 1: vcgencmd (Pi-specific)
    if command -v vcgencmd &> /dev/null; then
        log "📋 vcgencmd get_camera:"
        CAMERA_STATUS=$(vcgencmd get_camera 2>/dev/null || echo "failed")
        log "   $CAMERA_STATUS"
    fi
    
    # Method 2: dmesg for camera messages
    log "📋 Recent camera-related kernel messages:"
    CAMERA_MESSAGES=$(dmesg | grep -i -E "(camera|arducam|imx|ov)" | tail -5)
    if [ -n "$CAMERA_MESSAGES" ]; then
        echo "$CAMERA_MESSAGES" | while read -r line; do
            log "   $line"
        done
    else
        log "   ${YELLOW}No camera messages in dmesg${NC}"
    fi
    
    # Method 3: I2C detection (for CSI ArduCams)
    log "📋 I2C camera detection:"
    if command -v i2cdetect &> /dev/null; then
        I2C_OUTPUT=$(i2cdetect -y 1 2>/dev/null | grep -E "[0-9a-f]{2}" || echo "No I2C devices found")
        log "   $I2C_OUTPUT"
    else
        log "   ${YELLOW}i2cdetect not available${NC}"
    fi
}

# Check video devices
check_video_devices() {
    log "${BLUE}📹 Video Device Detection:${NC}"
    
    # List all video devices
    VIDEO_DEVICES=$(ls /dev/video* 2>/dev/null || echo "No video devices found")
    log "Available video devices:"
    if [ "$VIDEO_DEVICES" != "No video devices found" ]; then
        for device in $VIDEO_DEVICES; do
            if [ -c "$device" ]; then
                log "   $device"
                
                # Get device capabilities
                DEVICE_INFO=$(v4l2-ctl --device="$device" --list-formats-ext 2>/dev/null | head -10 || echo "Cannot query device")
                echo "$DEVICE_INFO" | while read -r line; do
                    log "     $line"
                done
            fi
        done
    else
        log "   ${RED}❌ No video devices found${NC}"
    fi
}

# Check libcamera support
check_libcamera() {
    log "${BLUE}📸 libcamera Support:${NC}"
    
    # Check if libcamera tools are available
    if command -v libcamera-hello &> /dev/null; then
        log "${GREEN}✅ libcamera-hello available${NC}"
        
        # List cameras
        log "📋 Available cameras:"
        CAMERAS=$(libcamera-hello --list-cameras 2>/dev/null || echo "Failed to list cameras")
        log "   $CAMERAS"
        
        # Test camera detection
        log "📋 Testing camera detection:"
        timeout 5s libcamera-hello --timeout 1000 2>/dev/null && \
            log "   ${GREEN}✅ libcamera camera test successful${NC}" || \
            log "   ${RED}❌ libcamera camera test failed${NC}"
    else
        log "${RED}❌ libcamera-hello not available${NC}"
        log "   Install with: sudo apt install libcamera-apps"
    fi
    
    # Check other libcamera tools
    LIBCAMERA_TOOLS=("libcamera-vid" "libcamera-still" "libcamera-raw")
    for tool in "${LIBCAMERA_TOOLS[@]}"; do
        if command -v "$tool" &> /dev/null; then
            log "   ${GREEN}✅ $tool available${NC}"
        else
            log "   ${YELLOW}⚠️ $tool not available${NC}"
        fi
    done
}

# Check dependencies
check_dependencies() {
    log "${BLUE}📦 Dependency Check:${NC}"
    
    DEPENDENCIES=(
        "ffmpeg:FFmpeg for video processing"
        "v4l2-ctl:V4L2 utilities"
        "libcamera-hello:libcamera test tool"
        "libcamera-vid:libcamera video capture"
    )
    
    for dep in "${DEPENDENCIES[@]}"; do
        TOOL=$(echo "$dep" | cut -d: -f1)
        DESC=$(echo "$dep" | cut -d: -f2)
        
        if command -v "$TOOL" &> /dev/null; then
            log "   ${GREEN}✅ $TOOL - $DESC${NC}"
        else
            log "   ${RED}❌ $TOOL - $DESC${NC}"
        fi
    done
}

# Fix common issues
fix_issues() {
    log "${BLUE}🔧 Fixing Common Issues:${NC}"
    
    # Add user to video group
    if ! groups | grep -q video; then
        log "Adding user to video group..."
        sudo usermod -a -G video $USER
        log "${GREEN}✅ User added to video group${NC}"
        log "${YELLOW}⚠️  You may need to log out and back in for changes to take effect${NC}"
    else
        log "${GREEN}✅ User already in video group${NC}"
    fi
    
    # Set proper permissions on video devices
    if [ -d /dev/video* ]; then
        log "Setting video device permissions..."
        sudo chmod 666 /dev/video* 2>/dev/null || true
        log "${GREEN}✅ Video device permissions set${NC}"
    fi
    
    # Check camera configuration
    CONFIG_FILES=("/boot/firmware/config.txt" "/boot/config.txt")
    CAMERA_CONFIG_FOUND=false
    
    for config_file in "${CONFIG_FILES[@]}"; do
        if [ -f "$config_file" ]; then
            log "Checking $config_file for camera settings..."
            
            if grep -q "^camera_auto_detect=1" "$config_file"; then
                log "${GREEN}✅ camera_auto_detect=1 found in $config_file${NC}"
                CAMERA_CONFIG_FOUND=true
            fi
            
            if grep -q "^dtoverlay=imx708" "$config_file"; then
                log "${GREEN}✅ IMX708 overlay found in $config_file${NC}"
                CAMERA_CONFIG_FOUND=true
            fi
        fi
    done
    
    if [ "$CAMERA_CONFIG_FOUND" = false ]; then
        log "${YELLOW}⚠️  No camera configuration found in config.txt${NC}"
        log "   Consider adding: camera_auto_detect=1"
        log "   Or for IMX708: dtoverlay=imx708"
    fi
}

# Test camera capture methods
test_capture_methods() {
    log "${BLUE}🧪 Testing Camera Capture Methods:${NC}"
    
    # Find the best video device
    BEST_DEVICE=""
    for device in /dev/video*; do
        if [ -c "$device" ]; then
            # Check if device supports MJPEG
            if v4l2-ctl --device="$device" --list-formats-ext 2>/dev/null | grep -q "MJPG"; then
                BEST_DEVICE="$device"
                log "${GREEN}✅ Found MJPEG-capable device: $device${NC}"
                break
            fi
        fi
    done
    
    if [ -z "$BEST_DEVICE" ]; then
        # Fallback to first available device
        BEST_DEVICE=$(ls /dev/video* 2>/dev/null | head -1)
        if [ -n "$BEST_DEVICE" ]; then
            log "${YELLOW}⚠️  No MJPEG device found, using: $BEST_DEVICE${NC}"
        else
            log "${RED}❌ No video devices available${NC}"
            return 1
        fi
    fi
    
    # Test different capture methods
    log "Testing capture methods with $BEST_DEVICE:"
    
    # Method 1: libcamera-vid
    if command -v libcamera-vid &> /dev/null; then
        log "📸 Testing libcamera-vid..."
        timeout 5s libcamera-vid -t 1000 -o /tmp/test_libcamera.jpg 2>/dev/null && \
            log "   ${GREEN}✅ libcamera-vid success${NC}" || \
            log "   ${RED}❌ libcamera-vid failed${NC}"
    fi
    
    # Method 2: FFmpeg with V4L2
    log "📸 Testing FFmpeg V4L2..."
    timeout 10s ffmpeg -f v4l2 -i "$BEST_DEVICE" -vframes 1 -y /tmp/test_ffmpeg_v4l2.jpg 2>/dev/null && \
        log "   ${GREEN}✅ FFmpeg V4L2 success${NC}" || \
        log "   ${RED}❌ FFmpeg V4L2 failed${NC}"
    
    # Method 3: FFmpeg with different formats
    for format in mjpeg yuyv422 yuv420p; do
        log "📸 Testing FFmpeg with format: $format..."
        timeout 10s ffmpeg -f v4l2 -input_format "$format" -video_size 640x480 -i "$BEST_DEVICE" -vframes 1 -y "/tmp/test_ffmpeg_$format.jpg" 2>/dev/null && \
            log "   ${GREEN}✅ FFmpeg $format success${NC}" || \
            log "   ${RED}❌ FFmpeg $format failed${NC}"
    done
}

# Generate recommendations
generate_recommendations() {
    log "${BLUE}💡 Recommendations:${NC}"
    
    # Check if any test images were created
    TEST_IMAGES=$(ls /tmp/test_*.jpg 2>/dev/null | wc -l)
    
    if [ "$TEST_IMAGES" -gt 0 ]; then
        log "${GREEN}✅ Camera is working! Found $TEST_IMAGES test images${NC}"
        log "   Working test images:"
        ls -la /tmp/test_*.jpg | while read -r line; do
            log "     $line"
        done
        
        log ""
        log "${GREEN}🎯 RECOMMENDED NEXT STEPS:${NC}"
        log "1. Use libcamera-vid for best compatibility with Pi 5"
        log "2. Run: ./start_camera_stream.sh start"
        log "3. Test stream in browser: http://$(hostname -I | awk '{print $1}'):8554/feed.mjpg"
    else
        log "${RED}❌ No working camera capture methods found${NC}"
        log ""
        log "${YELLOW}🔧 TROUBLESHOOTING STEPS:${NC}"
        log "1. Check physical camera connection (CSI cable orientation)"
        log "2. Ensure camera is enabled in /boot/firmware/config.txt"
        log "3. Reboot the Pi: sudo reboot"
        log "4. Install libcamera-apps: sudo apt install libcamera-apps"
        log "5. Check camera compatibility with Pi 5"
    fi
}

# Main diagnostic flow
main() {
    echo "Starting diagnostic at $(date)" > "$LOG_FILE"
    
    check_root
    check_system
    echo ""
    check_hardware
    echo ""
    check_video_devices
    echo ""
    check_libcamera
    echo ""
    check_dependencies
    echo ""
    fix_issues
    echo ""
    test_capture_methods
    echo ""
    generate_recommendations
    
    echo ""
    log "${BLUE}📋 Diagnostic complete! Check $LOG_FILE for detailed logs${NC}"
}

# Run main function
main "$@"
