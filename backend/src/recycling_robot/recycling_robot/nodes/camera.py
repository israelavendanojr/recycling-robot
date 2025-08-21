#!/usr/bin/env python3
"""
Updated ROS2 Camera Node - Consumes HTTP MJPEG stream from Pi host
This eliminates all the ArduCam driver issues inside Docker
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
import requests
from urllib.parse import urlparse


class HostStreamCameraNode(Node):
    def __init__(self):
        super().__init__('host_stream_camera')
        
        # Parameters
        self.declare_parameter('stream_url', 'http://host.docker.internal:8554/feed.mjpg')
        self.declare_parameter('fallback_device', 0)  # Fallback to local device if stream fails
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 10.0)
        
        self.stream_url = self.get_parameter('stream_url').value
        self.fallback_device = self.get_parameter('fallback_device').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        
        # Setup
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # State
        self.cap = None
        self.stream_mode = 'http'  # 'http' or 'local' or 'mock'
        self._running = True
        
        # Initialize camera source
        self._init_camera_source()
        
        # Publishing thread
        self.thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.thread.start()
        
        self.get_logger().info(f'Host stream camera node started with URL: {self.stream_url}')

    def _init_camera_source(self):
        """Initialize camera source (HTTP stream preferred, local fallback)"""
        
        # Try HTTP stream first
        if self._try_http_stream():
            return
        
        # Try local device fallback
        if self._try_local_device():
            return
            
        # Fall back to mock
        self.get_logger().warn('Using mock camera - no real source available')
        self.stream_mode = 'mock'

    def _try_http_stream(self):
        """Try to connect to HTTP MJPEG stream from Pi host"""
        try:
            self.get_logger().info(f'Attempting HTTP stream: {self.stream_url}')
            
            # Test if stream is accessible
            response = requests.head(self.stream_url, timeout=5)
            if response.status_code != 200:
                raise Exception(f"HTTP {response.status_code}")
            
            # Create OpenCV VideoCapture for HTTP stream
            self.cap = cv2.VideoCapture(self.stream_url)
            if not self.cap.isOpened():
                raise Exception("OpenCV failed to open stream")
            
            # Test frame read
            ret, frame = self.cap.read()
            if not ret or frame is None:
                raise Exception("Failed to read test frame")
            
            self.stream_mode = 'http'
            self.get_logger().info(f'✅ HTTP stream connected: {frame.shape}')
            return True
            
        except Exception as e:
            self.get_logger().warn(f'HTTP stream failed: {e}')
            if self.cap:
                self.cap.release()
                self.cap = None
            return False

    def _try_local_device(self):
        """Try local camera device as fallback"""
        try:
            self.get_logger().info(f'Attempting local device: /dev/video{self.fallback_device}')
            
            self.cap = cv2.VideoCapture(self.fallback_device)
            if not self.cap.isOpened():
                raise Exception("Failed to open local device")
            
            # Configure
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            # Test frame read
            ret, frame = self.cap.read()
            if not ret or frame is None:
                raise Exception("Failed to read test frame")
            
            self.stream_mode = 'local'
            self.get_logger().info(f'✅ Local device connected: {frame.shape}')
            return True
            
        except Exception as e:
            self.get_logger().warn(f'Local device failed: {e}')
            if self.cap:
                self.cap.release()
                self.cap = None
            return False

    def _reconnect_stream(self):
        """Try to reconnect to stream"""
        if self.cap:
            self.cap.release()
            self.cap = None
        
        self.get_logger().info('Attempting to reconnect camera source...')
        self._init_camera_source()

    def _publish_loop(self):
        """Main publishing loop with reconnection logic"""
        rate = 1.0 / max(1.0, self.fps)
        last_reconnect = 0
        reconnect_interval = 10  # seconds
        
        while self._running:
            try:
                frame = None
                
                if self.stream_mode == 'http' or self.stream_mode == 'local':
                    if self.cap and self.cap.isOpened():
                        ret, frame = self.cap.read()
                        if not ret or frame is None:
                            # Frame read failed
                            now = time.time()
                            if now - last_reconnect > reconnect_interval:
                                self._reconnect_stream()
                                last_reconnect = now
                            frame = None
                
                # Fall back to mock if no frame
                if frame is None:
                    frame = self._create_mock_frame()
                
                # Resize frame if needed
                if frame.shape[:2] != (self.height, self.width):
                    frame = cv2.resize(frame, (self.width, self.height))
                
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
        
        # Animated test pattern
        t = time.time()
        x = int((np.sin(t) * 0.3 + 0.5) * (self.width - 100))
        y = int((np.cos(t * 0.7) * 0.3 + 0.5) * (self.height - 60))
        
        cv2.rectangle(frame, (x, y), (x + 100, y + 60), (0, 255, 128), -1)
        
        # Status text
        status_text = {
            'http': f"HTTP STREAM FAILED",
            'local': f"LOCAL CAMERA FAILED", 
            'mock': f"MOCK CAMERA"
        }[self.stream_mode]
        
        cv2.putText(frame, status_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(frame, f"URL: {self.stream_url[:40]}...", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
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
        node = HostStreamCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()