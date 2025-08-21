#!/bin/bash
# Debug camera access inside ROS2 container
# Run this inside the ros2 container to diagnose camera issues

echo "🔍 Camera Debug Script"
echo "======================"

# Check if we're running as the right user
echo "Current user: $(whoami)"
echo "Groups: $(groups)"

# Check video devices
echo -e "\n📹 Video devices:"
ls -la /dev/video* 2>/dev/null || echo "No /dev/video* devices found"

# Check permissions
echo -e "\n🔑 Device permissions:"
ls -la /dev/video0 2>/dev/null || echo "/dev/video0 not found"

# Test v4l2 tools
echo -e "\n🔧 V4L2 info:"
v4l2-ctl --list-devices 2>/dev/null || echo "v4l2-ctl not available"

# Test basic OpenCV access
echo -e "\n🐍 Python OpenCV test:"
python3 -c "
import cv2
import os
print(f'OpenCV version: {cv2.__version__}')
print(f'Available backends: {[cv2.videoio_registry.getBackendName(b) for b in cv2.videoio_registry.getBackends()]}')

# Test device access
if os.path.exists('/dev/video0'):
    print('Testing /dev/video0...')
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            print(f'✅ Successfully captured frame: {frame.shape}')
        else:
            print('❌ Device opened but cannot read frames')
        cap.release()
    else:
        print('❌ Cannot open /dev/video0')
else:
    print('❌ /dev/video0 does not exist')
" 2>&1

echo -e "\n🏁 Debug complete!"
echo "If camera still fails:"
echo "1. Check that ArduCam is properly connected"
echo "2. Try 'sudo modprobe bcm2835-v4l2' on the Pi host"
echo "3. Verify camera works on host: 'raspistill -o test.jpg'"