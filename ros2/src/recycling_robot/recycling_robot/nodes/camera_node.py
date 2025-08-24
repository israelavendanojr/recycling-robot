#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time
import threading
import requests
import signal
import sys
from rclpy.executors import ExternalShutdownException

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Set logging level to INFO to reduce debug spam
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        
        # Parameters
        self.declare_parameter('device_id', 0)
        self.declare_parameter('fps', 10.0)
        
        self.device_id = self.get_parameter('device_id').value
        self.fps = self.get_parameter('fps').value
        
        # Setup
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # State
        self.cap = None
        self._running = True
        self._shutdown_event = threading.Event()
        self._init_camera()
        
        # Publishing thread
        self.thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.thread.start()
        
        self.get_logger().info('📷 Camera node started')
        self.get_logger().info(f'🎯 Target device: /dev/video{self.device_id}')
        self.get_logger().info(f'⚡ Target FPS: {self.fps}')

    def _init_camera(self):
        """Initialize camera source with better device handling"""
        # First try local device with proper V4L2 settings
        try:
            self.get_logger().info(f'🔍 Checking camera device /dev/video{self.device_id}...')
            
            # Test device access first
            if not os.path.exists(f'/dev/video{self.device_id}'):
                raise Exception(f'/dev/video{self.device_id} does not exist')
            
            # Try with V4L2 backend and specific settings
            self.cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
            
            if self.cap.isOpened():
                # Set camera properties
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.cap.set(cv2.CAP_PROP_FPS, self.fps)
                
                # Test frame capture
                ret, test_frame = self.cap.read()
                if ret and test_frame is not None:
                    self.get_logger().info('✅ Local camera connected and working')
                    self.get_logger().info(f'📐 Frame size: {test_frame.shape[1]}x{test_frame.shape[0]}')
                    return
                else:
                    self.get_logger().warn('⚠️  Device opened but cannot read frames')
                    self.cap.release()
                    self.cap = None
            else:
                self.cap.release()
                self.cap = None
                
        except Exception as e:
            self.get_logger().warn(f'⚠️  Local camera failed: {e}')
            if self.cap:
                self.cap.release()
                self.cap = None
        
        # Local camera failed - use mock
        self.get_logger().warn('🔄 Falling back to mock camera mode')
        self.cap = None

    def _publish_loop(self):
        """Publishing loop with better error handling"""
        rate = 1.0 / self.fps
        frame_count = 0
        
        while self._running and not self._shutdown_event.is_set():
            try:
                frame = None
                
                if self.cap and self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if not ret or frame is None:
                        if self._running:  # Only log if we're not shutting down
                            self.get_logger().warn('⚠️  Failed to read frame from camera')
                        continue
                    
                    # Convert frame to ROS2 Image message
                    try:
                        ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                        ros_image.header.stamp = self.get_clock().now().to_msg()
                        ros_image.header.frame_id = "camera_frame"
                        
                        # Publish
                        self.publisher.publish(ros_image)
                        frame_count += 1
                        
                        # Log every 30 frames (every 3 seconds at 10 FPS)
                        if frame_count % 30 == 0:
                            self.get_logger().info(f'📸 Published frame #{frame_count} ({frame.shape[1]}x{frame.shape[0]})')
                        
                    except Exception as e:
                        self.get_logger().error(f'❌ Frame conversion failed: {e}')
                        continue
                        
                else:
                    # Mock camera mode - publish a simple test image
                    if frame_count % 30 == 0:  # Log every 30 frames
                        self.get_logger().debug('🖼️  Mock camera mode - no real camera available')
                    
                    # Create a simple test image
                    test_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    test_frame[:] = (100, 100, 100)  # Gray background
                    
                    # Add some text
                    cv2.putText(test_frame, f'Mock Camera - Frame {frame_count}', 
                               (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    
                    try:
                        ros_image = self.bridge.cv2_to_imgmsg(test_frame, "bgr8")
                        ros_image.header.stamp = self.get_clock().now().to_msg()
                        ros_image.header.frame_id = "mock_camera_frame"
                        
                        self.publisher.publish(ros_image)
                        frame_count += 1
                        
                    except Exception as e:
                        self.get_logger().error(f'❌ Mock frame conversion failed: {e}')
                        continue
                
                # Rate limiting
                time.sleep(rate)
                
            except Exception as e:
                if self._running:  # Only log if we're not shutting down
                    self.get_logger().error(f'❌ Publishing loop error: {e}')
                time.sleep(rate)  # Continue trying

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        self.get_logger().info('🛑 Shutdown signal received')
        self._running = False
        self._shutdown_event.set()

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('🛑 Camera node shutting down')
        self._running = False
        self._shutdown_event.set()
        
        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info('📷 Camera released')
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()