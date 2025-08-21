#!/usr/bin/env python3
"""
Camera Detection and Test Script
Run this before launching the ROS nodes to diagnose camera issues
"""

import cv2
import glob
import os
import subprocess
import sys

def check_video_devices():
    """Check for video devices in /dev"""
    print("🔍 Checking for video devices...")
    video_devices = glob.glob('/dev/video*')
    
    if not video_devices:
        print("❌ No /dev/video* devices found")
        return []
    
    print(f"✅ Found video devices: {sorted(video_devices)}")
    return sorted(video_devices)

def check_camera_permissions():
    """Check if we have permission to access video devices"""
    print("\n🔑 Checking camera permissions...")
    try:
        # Check if we're in video group
        result = subprocess.run(['groups'], capture_output=True, text=True)
        groups = result.stdout.strip()
        if 'video' in groups:
            print("✅ User is in video group")
        else:
            print("⚠️ User not in video group (may cause permission issues)")
    except Exception as e:
        print(f"❓ Could not check groups: {e}")

def test_opencv_backends():
    """Test which OpenCV backends are available"""
    print("\n🔧 Testing OpenCV backends...")
    backends = [
        (cv2.CAP_V4L2, "V4L2"),
        (cv2.CAP_GSTREAMER, "GStreamer"),
        (cv2.CAP_FFMPEG, "FFMPEG"),
    ]
    
    for backend_id, name in backends:
        try:
            cap = cv2.VideoCapture()
            if cap.open(0, backend_id):
                print(f"✅ {name} backend available")
                cap.release()
            else:
                print(f"❌ {name} backend not available")
        except Exception as e:
            print(f"❌ {name} backend error: {e}")

def test_camera_devices():
    """Test each video device"""
    print("\n📹 Testing camera devices...")
    video_devices = glob.glob('/dev/video*')
    working_devices = []
    
    for device_path in sorted(video_devices):
        try:
            device_num = int(device_path.replace('/dev/video', ''))
            print(f"\n  Testing {device_path} (device {device_num})...")
            
            # Test with V4L2 backend (best for Pi cameras)
            cap = cv2.VideoCapture(device_num, cv2.CAP_V4L2)
            
            if not cap.isOpened():
                print(f"    ❌ Could not open with V4L2")
                cap.release()
                continue
            
            # Try to get some properties
            width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            fps = cap.get(cv2.CAP_PROP_FPS)
            fourcc = cap.get(cv2.CAP_PROP_FOURCC)
            
            print(f"    📐 Resolution: {width}x{height}")
            print(f"    🎥 FPS: {fps}")
            
            # Try to read a frame
            ret, frame = cap.read()
            if ret and frame is not None:
                print(f"    ✅ Successfully read frame: {frame.shape}")
                working_devices.append(device_num)
            else:
                print(f"    ❌ Could not read frame")
            
            cap.release()
            
        except Exception as e:
            print(f"    ❌ Error testing {device_path}: {e}")
    
    return working_devices

def check_libcamera():
    """Check if libcamera tools are available"""
    print("\n📸 Checking libcamera...")
    try:
        result = subprocess.run(['libcamera-hello', '--version'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print("✅ libcamera-hello available")
        else:
            print("❌ libcamera-hello not available")
    except FileNotFoundError:
        print("❌ libcamera tools not installed")
    except subprocess.TimeoutExpired:
        print("⚠️ libcamera-hello timeout")
    except Exception as e:
        print(f"❓ libcamera check error: {e}")

def main():
    print("🤖 ArduCam Detection and Diagnostics")
    print("=" * 50)
    
    # Basic checks
    check_video_devices()
    check_camera_permissions()
    test_opencv_backends()
    check_libcamera()
    
    # Device testing
    working_devices = test_camera_devices()
    
    # Summary
    print("\n" + "=" * 50)
    print("📋 SUMMARY")
    
    if working_devices:
        print(f"✅ Working camera devices: {working_devices}")
        print(f"💡 Recommended device_id for ROS launch: {working_devices[0]}")
        
        # Generate launch command
        print(f"\n🚀 Try launching with:")
        print(f"   ros2 run recycling_robot camera.py --ros-args -p device_id:={working_devices[0]}")
        
    else:
        print("❌ No working camera devices found")
        print("\n🔧 Troubleshooting tips:")
        print("   1. Check camera is properly connected")
        print("   2. Run 'lsusb' or 'vcgencmd get_camera' to verify hardware")
        print("   3. Try 'sudo usermod -a -G video $USER' and restart")
        print("   4. For ArduCam, check CSI cable connection")
        print("   5. Enable camera in raspi-config if using Pi camera")

if __name__ == "__main__":
    main()