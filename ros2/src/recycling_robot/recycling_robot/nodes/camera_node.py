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

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Parameters
        self.declare_parameter('stream_url', 'http://host.docker.internal:8554/feed.mjpg')
        self.declare_parameter('fallback_device', 0)
        self.declare_parameter('fps', 10.0)
        
        self.stream_url = self.get_parameter('stream_url').value
        self.fallback_device = self.get_parameter('fallback_device').value
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
        
        # Setup graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        
        self.get_logger().info(f'Camera node started: {self.stream_url}')

    def _signal_handler(self, sig, frame):
        """Handle SIGINT gracefully"""
        self.get_logger().info('Shutting down camera node...')
        self._running = False
        self._shutdown_event.set()

    def _init_camera(self):
        """Initialize camera source with better device handling"""
        # First try local device with proper V4L2 settings
        try:
            self.get_logger().info(f'Trying to open /dev/video{self.fallback_device}...')
            
            # Test device access first
            if not os.path.exists(f'/dev/video{self.fallback_device}'):
                raise Exception(f'/dev/video{self.fallback_device} does not exist')
            
            # Try with V4L2 backend and specific settings
            self.cap = cv2.VideoCapture(self.fallback_device, cv2.CAP_V4L2)
            
            if self.cap.isOpened():
                # Set camera properties
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.cap.set(cv2.CAP_PROP_FPS, self.fps)
                
                # Test frame capture
                ret, test_frame = self.cap.read()
                if ret and test_frame is not None:
                    self.get_logger().info('✅ Local camera connected and working')
                    return
                else:
                    self.get_logger().warn('Device opened but cannot read frames')
                    self.cap.release()
                    self.cap = None
            else:
                self.cap.release()
                self.cap = None
                
        except Exception as e:
            self.get_logger().warn(f'Local camera failed: {e}')
            if self.cap:
                self.cap.release()
                self.cap = None
        
        # Try HTTP stream fallback
        try:
            response = requests.head(self.stream_url, timeout=5)
            if response.status_code == 200:
                self.cap = cv2.VideoCapture(self.stream_url)
                if self.cap.isOpened():
                    self.get_logger().info('✅ HTTP stream connected')
                    return
        except Exception as e:
            self.get_logger().warn(f'HTTP stream failed: {e}')
        
        # Both failed - use mock
        self.get_logger().warn('Using mock camera')
        self.cap = None

    def _publish_loop(self):
        """Publishing loop with better error handling"""
        rate = 1.0 / self.fps
        
        while self._running and not self._shutdown_event.is_set():
            try:
                frame = None
                
                if self.cap and self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if not ret or frame is None:
                        self.get_logger().warn('Failed to read frame, reinitializing camera...')
                        self._init_camera()
                        frame = None
                
                if frame is None:
                    frame = self._create_mock_frame()
                
                # Convert BGR to RGB and resize
                frame = cv2.resize(frame, (640, 480))
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # Create ROS message
                msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding='rgb8')
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_link'
                
                self.publisher.publish(msg)
                time.sleep(rate)
                
            except Exception as e:
                if self._running:  # Only log if we're not shutting down
                    self.get_logger().error(f'Publishing error: {e}')
                time.sleep(0.5)

    def _create_mock_frame(self):
        """Create mock frame when camera unavailable"""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(frame, "MOCK CAMERA", (200, 240), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, "Check /dev/video0", (180, 280), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        return frame

    def destroy_node(self):
        """Clean shutdown"""
        self._running = False
        self._shutdown_event.set()
        
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        
        if self.cap:
            self.cap.release()
            
        super().destroy_node()

def main():
    rclpy.init()
    node = None
    
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass  # Already shutdown

if __name__ == '__main__':
    main()