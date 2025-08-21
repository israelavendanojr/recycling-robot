#!/bin/bash
# Quick Camera Test Script for Raspberry Pi 5 + ArduCam IMX708

set -e

echo "🔍 Quick Camera Test for Raspberry Pi 5 + ArduCam IMX708"
echo "========================================================"

# Test 1: Check if this is a Raspberry Pi
echo "📋 System Check:"
if grep -q "Raspberry Pi" /proc/cpuinfo; then
    echo "✅ Raspberry Pi detected"
    PI_MODEL=$(grep "Model" /proc/cpuinfo | head -1 | cut -d: -f2 | xargs)
    echo "   Model: $PI_MODEL"
else
    echo "❌ Not a Raspberry Pi"
    exit 1
fi

# Test 2: Check camera hardware
echo ""
echo "🔧 Camera Hardware Check:"
if command -v vcgencmd &> /dev/null; then
    CAMERA_STATUS=$(vcgencmd get_camera 2>/dev/null || echo "failed")
    echo "vcgencmd get_camera: $CAMERA_STATUS"
else
    echo "⚠️ vcgencmd not available"
fi

# Test 3: Check video devices
echo ""
echo "📹 Video Devices:"
if ls /dev/video* 1> /dev/null 2>&1; then
    echo "Available video devices:"
    for device in /dev/video*; do
        if [ -c "$device" ]; then
            echo "  $device"
            # Show device capabilities
            v4l2-ctl --device="$device" --list-formats-ext 2>/dev/null | head -5 | sed 's/^/    /'
        fi
    done
else
    echo "❌ No video devices found"
fi

# Test 4: Check rpicam
echo ""
echo "📸 rpicam Check:"
if command -v rpicam-hello &> /dev/null; then
    echo "✅ rpicam-hello available"
    echo "Available cameras:"
    rpicam-hello --list-cameras 2>/dev/null || echo "  Failed to list cameras"
else
    echo "❌ rpicam-hello not available"
    echo "Install rpicam tools"
fi

# Test 5: Quick capture test
echo ""
echo "📸 Quick Capture Test:"
if command -v rpicam-vid &> /dev/null; then
    echo "Testing rpicam-vid..."
    timeout 3s rpicam-vid -t 1000 -o /tmp/test_capture.jpg 2>/dev/null && \
        echo "✅ rpicam-vid test successful" || \
        echo "❌ rpicam-vid test failed"
else
    echo "⚠️ rpicam-vid not available"
fi

# Test 6: Check dependencies
echo ""
echo "📦 Dependencies Check:"
DEPS=("ffmpeg" "v4l2-ctl" "rpicam-vid")
for dep in "${DEPS[@]}"; do
    if command -v "$dep" &> /dev/null; then
        echo "✅ $dep available"
    else
        echo "❌ $dep not available"
    fi
done

echo ""
echo "🎯 Next Steps:"
echo "1. If all checks pass, run: ./start_camera_stream.sh start"
echo "2. If issues found, run: ./camera_diagnostic.sh"
echo "3. Test stream in browser: http://$(hostname -I | awk '{print $1}'):8554/feed.mjpg"
