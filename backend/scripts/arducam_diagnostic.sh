#!/bin/bash
# ArduCam Diagnostic and Fix Script for Raspberry Pi

set -e

echo "🔍 ArduCam Diagnostic and Fix Script"
echo "=================================="

# Function to check if running as root
check_root() {
    if [[ $EUID -eq 0 ]]; then
        echo "⚠️  Running as root - this might cause permission issues later"
        echo "   Consider running as regular user with sudo when needed"
    fi
}

# Check camera hardware detection
check_hardware() {
    echo "🔧 Checking camera hardware detection..."
    
    # Check if camera is detected by the system
    echo "Camera detection methods:"
    
    # Method 1: vcgencmd (Pi-specific)
    if command -v vcgencmd &> /dev/null; then
        echo "📋 vcgencmd get_camera:"
        vcgencmd get_camera || echo "   ❌ vcgencmd failed"
    fi
    
    # Method 2: dmesg for camera messages
    echo "📋 Recent camera-related kernel messages:"
    dmesg | grep -i -E "(camera|arducam|imx|ov)" | tail -10 || echo "   ❌ No camera messages in dmesg"
    
    # Method 3: USB detection (for USB ArduCams)
    echo "📋 USB camera devices:"
    lsusb | grep -i -E "(camera|arducam|imaging)" || echo "   ❌ No USB cameras detected"
    
    # Method 4: I2C detection (for CSI ArduCams)
    echo "📋 I2C camera detection:"
    if command -v i2cdetect &> /dev/null; then
        echo "I2C bus 1:" && i2cdetect -y 1 2>/dev/null || echo "   ❌ i2cdetect failed"
    fi
}

# Test specific video device
test_device() {
    local device=$1
    echo "🧪 Testing $device..."
    
    # Check device permissions
    if [[ ! -r "$device" ]]; then
        echo "   ❌ Cannot read $device (permission issue)"
        return 1
    fi
    
    # Get device info
    echo "   📋 Device capabilities:"
    v4l2-ctl --device="$device" --all 2>/dev/null | head -20 || echo "   ❌ Cannot query device"
    
    # Try to capture a single frame with different methods
    echo "   📸 Testing frame capture methods:"
    
    # Method 1: FFmpeg with V4L2
    echo "      Testing FFmpeg V4L2..."
    timeout 10s ffmpeg -f v4l2 -input_format yuyv422 -video_size 640x480 -i "$device" -vframes 1 -y "/tmp/test_$(basename $device).jpg" 2>/dev/null && \
        echo "      ✅ FFmpeg V4L2 success" || echo "      ❌ FFmpeg V4L2 failed"
    
    # Method 2: FFmpeg auto-detect
    echo "      Testing FFmpeg auto..."
    timeout 10s ffmpeg -f v4l2 -i "$device" -vframes 1 -y "/tmp/test_auto_$(basename $device).jpg" 2>/dev/null && \
        echo "      ✅ FFmpeg auto success" || echo "      ❌ FFmpeg auto failed"
    
    # Method 3: fswebcam (if available)
    if command -v fswebcam &> /dev/null; then
        echo "      Testing fswebcam..."
        timeout 10s fswebcam -d "$device" -r 640x480 --no-banner "/tmp/fswebcam_$(basename $device).jpg" 2>/dev/null && \
            echo "      ✅ fswebcam success" || echo "      ❌ fswebcam failed"
    fi
}

# Fix permissions
fix_permissions() {
    echo "🔐 Fixing camera permissions..."
    
    # Add user to video group
    sudo usermod -a -G video $USER || echo "❌ Failed to add user to video group"
    
    # Set proper permissions on video devices
    sudo chmod 666 /dev/video* 2>/dev/null || echo "❌ Failed to set device permissions"
    
    # Check if udev rules exist for ArduCam
    if [[ ! -f /etc/udev/rules.d/99-arducam.rules ]]; then
        echo "Creating ArduCam udev rules..."
        sudo tee /etc/udev/rules.d/99-arducam.rules > /dev/null << 'EOF'
# ArduCam udev rules
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="0c45", MODE="0666", GROUP="video"
SUBSYSTEM=="video4linux", KERNEL=="video[0-9]*", MODE="0666", GROUP="video"
EOF
        sudo udevadm control --reload-rules
        sudo udevadm trigger
        echo "✅ udev rules created"
    fi
}

# Try to initialize camera
init_camera() {
    echo "🎬 Attempting camera initialization..."
    
    # For CSI cameras, ensure camera is enabled
    if grep -q "^camera_auto_detect=1" /boot/firmware/config.txt 2>/dev/null || \
       grep -q "^camera_auto_detect=1" /boot/config.txt 2>/dev/null; then
        echo "✅ Camera auto-detect enabled in config.txt"
    else
        echo "⚠️  Camera auto-detect not found in config.txt"
        echo "   You may need to add 'camera_auto_detect=1' to /boot/firmware/config.txt"
    fi
    
    # Try libcamera commands (for newer Pi OS)
    if command -v libcamera-hello &> /dev/null; then
        echo "📸 Testing libcamera..."
        timeout 5s libcamera-hello --list-cameras 2>/dev/null && \
            echo "✅ libcamera detected cameras" || echo "❌ libcamera failed"
    fi
}

# Test promising devices
test_promising_devices() {
    echo "🎯 Testing most promising devices based on your v4l2-ctl output..."
    
    # Based on the formats, video3 looks most promising for ArduCam
    promising_devices=("/dev/video3" "/dev/video0" "/dev/video1")
    
    for device in "${promising_devices[@]}"; do
        if [[ -e "$device" ]]; then
            test_device "$device"
            echo ""
        fi
    done
}

# Create a test stream
create_test_stream() {
    local device=${1:-/dev/video3}
    echo "🎥 Creating test stream from $device..."
    
    # Kill any existing streams
    pkill -f "ffmpeg.*$device" 2>/dev/null || true
    
    # Test different streaming approaches
    echo "Method 1: Direct MJPEG stream..."
    timeout 10s ffmpeg -f v4l2 -input_format yuyv422 -video_size 640x480 -framerate 10 -i "$device" \
        -c:v mjpeg -f mjpeg - > /tmp/test_stream.mjpg 2>/dev/null && \
        echo "✅ MJPEG stream created ($(wc -c < /tmp/test_stream.mjpg) bytes)" || \
        echo "❌ MJPEG stream failed"
    
    echo "Method 2: HTTP server stream..."
    # This would start the actual HTTP server for Docker
    echo "To start HTTP stream server, run:"
    echo "  ffmpeg -f v4l2 -input_format yuyv422 -video_size 640x480 -framerate 10 -i $device \\"
    echo "    -c:v mjpeg -f mjpeg -listen 1 -http_port 8554 http://0.0.0.0:8554/stream.mjpg"
}

# Main diagnostic flow
main() {
    check_root
    check_hardware
    echo ""
    fix_permissions
    echo ""
    init_camera
    echo ""
    test_promising_devices
    echo ""
    create_test_stream "/dev/video3"
    
    echo ""
    echo "📋 DIAGNOSTIC SUMMARY"
    echo "===================="
    
    # Check if any test files were created
    if ls /tmp/test_*.jpg 1> /dev/null 2>&1; then
        echo "✅ Successfully captured test images:"
        ls -la /tmp/test_*.jpg
        echo ""
        echo "🎯 RECOMMENDATION: Use the device that created working images"
        echo "   Most likely candidate: /dev/video3 (based on format support)"
    else
        echo "❌ No test images captured successfully"
        echo ""
        echo "🔧 TROUBLESHOOTING STEPS:"
        echo "1. Ensure ArduCam is properly connected (CSI or USB)"
        echo "2. Check /boot/firmware/config.txt has camera_auto_detect=1"
        echo "3. Reboot the Pi after making config changes"
        echo "4. Try running: sudo rpi-update (if using older Pi OS)"
        echo "5. For CSI cameras, check cable connection and orientation"
        echo "6. For USB cameras, try different USB ports"
    fi
    
    echo ""
    echo "🚀 NEXT STEPS for Docker integration:"
    echo "If test images were created successfully:"
    echo "1. Start the stream: ffmpeg -f v4l2 -input_format yuyv422 -video_size 640x480 -framerate 10 -i /dev/video3 -c:v mjpeg -f mjpeg -listen 1 -http_port 8554 http://0.0.0.0:8554/stream.mjpg"
    echo "2. Test in browser: http://$(hostname -I | awk '{print $1}'):8554/stream.mjpg"
    echo "3. Update Docker container to use: http://host.docker.internal:8554/stream.mjpg"
}

# Run main function
main "$@"