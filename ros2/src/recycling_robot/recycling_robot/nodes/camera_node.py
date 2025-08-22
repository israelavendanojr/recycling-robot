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
        
        # Remove manual signal handler - let ROS2 handle it
        # signal.signal(signal.SIGINT, self._signal_handler)  # Remove this line
        
        self.get_logger().info(f'Camera node started with device /dev/video{self.device_id}')

    def _init_camera(self):
        """Initialize camera source with better device handling"""
        # First try local device with proper V4L2 settings
        try:
            self.get_logger().info(f'Trying to open /dev/video{self.device_id}...')
            
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
        
        # Local camera failed - use mock
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
                        if self._running:  # Only log if we're not shutting down
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
                
                # Use shutdown event for interruptible sleep
                if self._shutdown_event.wait(rate):
                    break  # Shutdown requested
                
            except Exception as e:
                if self._running:  # Only log if we're not shutting down
                    self.get_logger().error(f'Publishing error: {e}')
                if self._shutdown_event.wait(0.5):
                    break

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
        self.get_logger().info('Shutting down camera node...')
        self._running = False
        self._shutdown_event.set()
        
        # Wait for thread to finish
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        
        # Release camera
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
        pass  # Handle Ctrl+C gracefully
    except ExternalShutdownException:
        pass  # Handle ROS2 shutdown gracefully
    finally:
        if node:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass  # Already shutdown

if __name__ == '__main__':
    main()