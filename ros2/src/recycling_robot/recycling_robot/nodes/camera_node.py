#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header, String
import cv2
import numpy as np
import os
import time
import threading
import signal
import sys
from rclpy.executors import ExternalShutdownException
from PIL import Image
import io

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Set logging level to INFO to reduce debug spam
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        
        # Parameters
        self.declare_parameter('device_id', 0)
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('jpeg_quality', 85)
        
        self.device_id = self.get_parameter('device_id').value
        self.fps = self.get_parameter('fps').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        
        # Publisher for compressed images (same format as mock camera)
        self.publisher = self.create_publisher(CompressedImage, '/camera/image_raw', 10)
        
        # Pipeline state subscription
        self.pipeline_state_sub = self.create_subscription(
            String, 
            '/pipeline/state', 
            self.pipeline_state_callback, 
            10
        )
        
        # State
        self.cap = None
        self._running = True
        self._shutdown_event = threading.Event()
        self.pipeline_state = "idle"  # Track pipeline state
        self._init_camera()
        
        # Publishing thread
        self.thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.thread.start()
        
        self.get_logger().info('[Camera] Real Camera node started')
        self.get_logger().info(f'[Camera] Target device: /dev/video{self.device_id}')
        self.get_logger().info(f'[Camera] Target FPS: {self.fps}')
        self.get_logger().info(f'[Camera] Target resolution: {self.width}x{self.height}')

    def pipeline_state_callback(self, msg):
        """Handle pipeline state updates"""
        try:
            self.pipeline_state = msg.data
            self.get_logger().debug(f'[Camera] Pipeline state: {self.pipeline_state}')
        except Exception as e:
            self.get_logger().error(f'[Camera] Pipeline state callback error: {e}')

    def _detect_cameras(self):
        """Detect available camera devices"""
        available_devices = []
        for i in range(10):  # Check first 10 video devices
            device_path = f'/dev/video{i}'
            if os.path.exists(device_path):
                try:
                    # Quick test to see if device can be opened
                    cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
                    if cap.isOpened():
                        # Try to read a frame to verify it's working
                        ret, frame = cap.read()
                        if ret and frame is not None:
                            available_devices.append(i)
                            self.get_logger().info(f'[Camera] Found working camera: /dev/video{i}')
                    cap.release()
                except:
                    pass
        return available_devices

    def _init_camera(self):
        """Initialize camera source with better device handling"""
        self.get_logger().info('[Camera] Detecting available cameras...')
        available_devices = self._detect_cameras()
        
        if not available_devices:
            self.get_logger().warn('[Camera] No working cameras found')
            self.cap = None
            return
        
        # Try to use requested device first, then fall back to first available
        devices_to_try = [self.device_id] if self.device_id in available_devices else available_devices
        
        for device_id in devices_to_try:
            try:
                self.get_logger().info(f'[Camera] Trying camera /dev/video{device_id}...')
                
                # Initialize with V4L2 backend for better Linux support
                self.cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
                
                if not self.cap.isOpened():
                    self.get_logger().warn(f'[Camera] Could not open /dev/video{device_id}')
                    continue
                
                # Set camera properties
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_FPS, self.fps)
                
                # For Logitech cameras, set format to MJPEG for better performance
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
                
                # Verify settings
                actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
                
                # Test frame capture
                ret, test_frame = self.cap.read()
                if ret and test_frame is not None:
                    self.get_logger().info(f'[Camera] Camera /dev/video{device_id} initialized successfully!')
                    self.get_logger().info(f'[Camera] Actual resolution: {actual_width}x{actual_height}')
                    self.get_logger().info(f'[Camera] Actual FPS: {actual_fps:.1f}')
                    self.get_logger().info(f'[Camera] Test frame shape: {test_frame.shape}')
                    
                    # Update our device_id to the working one
                    self.device_id = device_id
                    return
                else:
                    self.get_logger().warn(f'[Camera] /dev/video{device_id} opened but cannot read frames')
                    self.cap.release()
                    self.cap = None
                    
            except Exception as e:
                self.get_logger().warn(f'[Camera] Failed to initialize /dev/video{device_id}: {e}')
                if self.cap:
                    self.cap.release()
                    self.cap = None
        
        # All cameras failed
        self.get_logger().error('[Camera] Could not initialize any camera')
        self.cap = None

    def _publish_loop(self):
        """Publishing loop with compressed image output"""
        rate = 1.0 / self.fps
        frame_count = 0
        last_log_time = time.time()
        
        while self._running and not self._shutdown_event.is_set():
            try:
                if self.cap and self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if not ret or frame is None:
                        if self._running:
                            self.get_logger().warn('[Camera] Failed to read frame from camera')
                            # Try to reinitialize camera
                            self._init_camera()
                        continue
                    
                    # Convert OpenCV frame (BGR) to PIL Image (RGB) for JPEG compression
                    try:
                        # Convert BGR to RGB
                        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        pil_image = Image.fromarray(rgb_frame)
                        
                        # Compress to JPEG
                        buffer = io.BytesIO()
                        pil_image.save(buffer, format='JPEG', quality=self.jpeg_quality)
                        compressed_data = buffer.getvalue()
                        
                        # Check pipeline state before publishing
                        if self.pipeline_state == "processing":
                            # Skip publishing if pipeline is busy
                            if frame_count % 30 == 0:  # Log every 30 frames
                                self.get_logger().info('⏸️ [Camera] Pipeline busy, skipping image publication (waiting for sorting to complete)')
                            continue
                        
                        # Create ROS2 CompressedImage message
                        msg = CompressedImage()
                        msg.format = 'jpeg'
                        msg.data = compressed_data
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = f"camera_{self.device_id}_frame"
                        
                        # Publish
                        self.publisher.publish(msg)
                        frame_count += 1
                        
                        # Log every 30 frames (every ~3 seconds at 10 FPS) or every 10 seconds
                        current_time = time.time()
                        if frame_count % 30 == 0 or (current_time - last_log_time) > 10:
                            self.get_logger().info(f'[Camera] Published frame #{frame_count} ({len(compressed_data)} bytes, {frame.shape[1]}x{frame.shape[0]})')
                            last_log_time = current_time
                        
                    except Exception as e:
                        self.get_logger().error(f'[Camera] Frame compression failed: {e}')
                        continue
                        
                else:
                    # No camera available
                    if frame_count % 30 == 0:
                        self.get_logger().warn('[Camera] No camera available - waiting...')
                        # Try to reinitialize every 30 attempts
                        self._init_camera()
                    frame_count += 1
                
                # Rate limiting
                time.sleep(rate)
                
            except Exception as e:
                if self._running:
                    self.get_logger().error(f'[Camera] Publishing loop error: {e}')
                time.sleep(rate)

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        self.get_logger().info('[Camera] Shutdown signal received')
        self._running = False
        self._shutdown_event.set()

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('[Camera] Camera node shutting down')
        self._running = False
        self._shutdown_event.set()
        
        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info('[Camera] Camera released')
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    ros_shutdown_called = False
    
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    except Exception as e:
        print(f'Unexpected error: {e}')
    finally:
        if node:
            try:
                node.destroy_node()
            except Exception as e:
                print(f'Error during node destruction: {e}')
        try:
            if not ros_shutdown_called:
                rclpy.shutdown()
                ros_shutdown_called = True
        except Exception as e:
            if "rcl_shutdown already called" not in str(e):
                print(f'Error during shutdown: {e}')

if __name__ == '__main__':
    main()