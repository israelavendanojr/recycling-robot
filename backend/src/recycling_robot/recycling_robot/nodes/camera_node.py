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
        self._init_camera()
        
        # Publishing thread
        self.thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.thread.start()
        
        self.get_logger().info(f'Camera node started: {self.stream_url}')

    def _init_camera(self):
        """Initialize camera source"""
        try:
            # Test stream availability
            response = requests.head(self.stream_url, timeout=5)
            if response.status_code == 200:
                self.cap = cv2.VideoCapture(self.stream_url)
                if self.cap.isOpened():
                    self.get_logger().info('✅ HTTP stream connected')
                    return
        except:
            pass
        
        # Fallback to local device
        try:
            self.cap = cv2.VideoCapture(self.fallback_device)
            if self.cap.isOpened():
                self.get_logger().info('✅ Local device connected')
                return
        except:
            pass
        
        self.get_logger().warn('Using mock camera')
        self.cap = None

    def _publish_loop(self):
        rate = 1.0 / self.fps
        
        while self._running:
            try:
                frame = None
                
                if self.cap and self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if not ret:
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
                self.get_logger().error(f'Publishing error: {e}')
                time.sleep(0.5)

    def _create_mock_frame(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(frame, "MOCK CAMERA", (200, 240), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        return frame

    def destroy_node(self):
        self._running = False
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.cap:
            self.cap.release()
        super().destroy_node()

def main():
    rclpy.init()
    try:
        node = CameraNode()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()