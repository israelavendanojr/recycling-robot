#!/usr/bin/env python3
"""
Enhanced ROS2 Camera Node with ArduCam detection and device auto-discovery
Publishes images to /camera/image_raw topic
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading
import os
import glob


class SimpleCameraNode(Node):
    def __init__(self):
        super().__init__('simple_camera')
        
        # Parameters
        self.declare_parameter('device_id', -1)   # -1 means auto-detect
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 10.0)
        
        self.device_id = self.get_parameter('device_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        
        # Setup
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Camera initialization
        self.cap = None
        self._init_camera()
        
        # Publishing thread
        self._running = True
        self.thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.thread.start()
        
        self.get_logger().info(
            f'Camera node started: {self.width}x{self.height} @ {self.fps}fps'
        )

    def _detect_available_cameras(self):
        """Detect available camera devices"""
        available_devices = []
        
        # Check for video devices
        video_devices = glob.glob('/dev/video*')
        self.get_logger().info(f'Found video devices: {video_devices}')
        
        # Test each device
        for device_path in sorted(video_devices):
            try:
                device_num = int(device_path.replace('/dev/video', ''))
                cap = cv2.VideoCapture(device_num)
                
                if cap.isOpened():
                    # Try to read a frame to verify it works
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        available_devices.append(device_num)
                        self.get_logger().info(f'✓ Working camera found: /dev/video{device_num}')
                cap.release()
                
            except (ValueError, Exception) as e:
                self.get_logger().debug(f'Skipping {device_path}: {e}')
        
        return available_devices

    def _init_camera(self):
        """Initialize camera with device detection and ArduCam support"""
        try:
            # Auto-detect if device_id is -1
            if self.device_id == -1:
                available = self._detect_available_cameras()
                if available:
                    self.device_id = available[0]  # Use first working device
                    self.get_logger().info(f'Auto-detected camera: /dev/video{self.device_id}')
                else:
                    raise RuntimeError("No working camera devices found")
            
            self.get_logger().info(f'Initializing camera /dev/video{self.device_id}')
            
            # Try different initialization methods
            cap = None
            
            # Method 1: Direct V4L2 (works best for ArduCam on Pi)
            try:
                cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
                if cap.isOpened():
                    self.get_logger().info('✓ Opened with V4L2 backend')
                else:
                    cap.release()
                    cap = None
            except Exception as e:
                self.get_logger().debug(f'V4L2 failed: {e}')
            
            # Method 2: GStreamer (backup for some Pi configurations)
            if cap is None:
                try:
                    # GStreamer pipeline for ArduCam
                    gst_pipeline = (
                        f'v4l2src device=/dev/video{self.device_id} ! '
                        f'video/x-raw,width={self.width},height={self.height},framerate={int(self.fps)}/1 ! '
                        'videoconvert ! appsink'
                    )
                    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
                    if cap.isOpened():
                        self.get_logger().info('✓ Opened with GStreamer backend')
                    else:
                        cap.release()
                        cap = None
                except Exception as e:
                    self.get_logger().debug(f'GStreamer failed: {e}')
            
            # Method 3: Default OpenCV (fallback)
            if cap is None:
                cap = cv2.VideoCapture(self.device_id)
                if cap.isOpened():
                    self.get_logger().info('✓ Opened with default backend')
                else:
                    cap.release()
                    cap = None
            
            if cap is None:
                raise RuntimeError(f"Could not open /dev/video{self.device_id} with any backend")
            
            # Configure capture settings
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            # Try different pixel formats for ArduCam compatibility
            formats_to_try = [
                cv2.VideoWriter_fourcc('M','J','P','G'),  # MJPEG (common)
                cv2.VideoWriter_fourcc('Y','U','Y','V'),  # YUYV
                cv2.VideoWriter_fourcc('U','Y','V','Y'),  # UYVY
            ]
            
            for fmt in formats_to_try:
                cap.set(cv2.CAP_PROP_FOURCC, fmt)
                ret, test_frame = cap.read()
                if ret and test_frame is not None:
                    fourcc = cap.get(cv2.CAP_PROP_FOURCC)
                    fmt_str = "".join([chr(int(fourcc) >> 8 * i & 0xFF) for i in range(4)])
                    self.get_logger().info(f'✓ Using pixel format: {fmt_str}')
                    break
            else:
                # No format worked, but continue anyway
                self.get_logger().warn('No pixel format worked perfectly, continuing...')
            
            # Final test
            ret, frame = cap.read()
            if not ret or frame is None:
                raise RuntimeError("Failed to read test frame")
            
            self.cap = cap
            self.get_logger().info(f'✅ Camera successfully opened: /dev/video{self.device_id}')
            
            # Log actual settings
            actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = cap.get(cv2.CAP_PROP_FPS)
            self.get_logger().info(
                f'Actual settings: {actual_w}x{actual_h} @ {actual_fps:.2f}fps'
            )
            
        except Exception as e:
            self.get_logger().warn(f'Camera initialization failed: {e}')
            self.get_logger().info('Falling back to mock camera mode')
            self.cap = None

    def _publish_loop(self):
        """Main publishing loop"""
        rate = 1.0 / max(1.0, self.fps)
        
        while self._running:
            try:
                if self.cap and self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if not ret or frame is None:
                        self.get_logger().warn('Failed to read frame, using mock')
                        frame = self._create_mock_frame()
                else:
                    frame = self._create_mock_frame()
                
                # Convert BGR to RGB
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # Create ROS message
                msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding='rgb8')
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_link'
                
                # Publish
                self.publisher.publish(msg)
                time.sleep(rate)
                
            except Exception as e:
                self.get_logger().error(f'Publishing error: {e}')
                time.sleep(0.5)

    def _create_mock_frame(self):
        """Create a test pattern when no camera is available"""
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        t = time.time()
        x = int((np.sin(t) * 0.3 + 0.5) * (self.width - 100))
        y = int((np.cos(t * 0.7) * 0.3 + 0.5) * (self.height - 60))
        cv2.rectangle(frame, (x, y), (x + 100, y + 60), (0, 255, 128), -1)
        cv2.putText(frame, "MOCK CAMERA", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(frame, f"No /dev/video{self.device_id}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return frame

    def destroy_node(self):
        self._running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.cap:
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    try:
        node = SimpleCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()