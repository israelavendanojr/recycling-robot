#!/usr/bin/env python3
"""
Simple ROS2 Camera Node with ArduCam support
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


class SimpleCameraNode(Node):
    def __init__(self):
        super().__init__('simple_camera')
        
        # Parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 10.0)
        
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        
        # Setup
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Camera initialization with ArduCam support
        self.cap = None
        self._init_camera(width, height, fps)
        
        # Publishing thread
        self._running = True
        self.thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.thread.start()
        
        self.get_logger().info(f'Camera node started: {width}x{height} @ {fps}fps')

    def _init_camera(self, width, height, fps):
        """Initialize camera with ArduCam-specific settings"""
        
        # Try different approaches for ArduCam
        for device_id in [0, 1, 2]:  # ArduCam might not be on /dev/video0
            try:
                self.get_logger().info(f'Trying camera device {device_id}')
                cap = cv2.VideoCapture(device_id)
                
                if not cap.isOpened():
                    continue
                
                # ArduCam-specific settings
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                cap.set(cv2.CAP_PROP_FPS, fps)
                
                # Force MJPEG for better ArduCam compatibility
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce latency
                
                # Test read
                ret, frame = cap.read()
                if ret and frame is not None:
                    self.cap = cap
                    self.get_logger().info(f'✓ Camera opened on device {device_id}')
                    
                    # Log actual settings
                    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    actual_fps = cap.get(cv2.CAP_PROP_FPS)
                    self.get_logger().info(f'Actual resolution: {actual_w}x{actual_h} @ {actual_fps}fps')
                    return
                else:
                    cap.release()
                    
            except Exception as e:
                self.get_logger().warn(f'Device {device_id} failed: {e}')
                if cap:
                    cap.release()
        
        # If all devices fail, use mock camera
        self.get_logger().warn('No camera found, using mock mode')
        self.cap = None

    def _publish_loop(self):
        """Main publishing loop"""
        rate = 1.0 / max(1.0, self.get_parameter('fps').value)
        
        while self._running:
            try:
                # Get frame
                if self.cap and self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if not ret or frame is None:
                        self.get_logger().warn('Failed to read frame')
                        time.sleep(0.1)
                        continue
                else:
                    # Mock frame
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
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        
        # Create colorful test pattern
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Moving rectangle
        t = time.time()
        x = int((np.sin(t) * 0.3 + 0.5) * (width - 100))
        y = int((np.cos(t * 0.7) * 0.3 + 0.5) * (height - 60))
        cv2.rectangle(frame, (x, y), (x + 100, y + 60), (0, 255, 128), -1)
        
        # Text
        cv2.putText(frame, "MOCK CAMERA", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(frame, f"{width}x{height}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        
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