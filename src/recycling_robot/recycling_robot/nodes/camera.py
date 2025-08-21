#!/usr/bin/env python3
"""
Simple ROS2 Camera Node with ArduCam loopback support
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
        self.declare_parameter('device_id', 10)   # default /dev/video10
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
            f'Camera node started: {self.width}x{self.height} @ {self.fps}fps '
            f'on /dev/video{self.device_id}'
        )

    def _init_camera(self):
        """Initialize camera for ArduCam / loopback device"""
        try:
            self.get_logger().info(f'Trying camera device /dev/video{self.device_id}')
            cap = cv2.VideoCapture(self.device_id)
            
            if not cap.isOpened():
                raise RuntimeError(f"Could not open /dev/video{self.device_id}")
            
            # Configure capture
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            ret, frame = cap.read()
            if not ret or frame is None:
                raise RuntimeError("Failed to read test frame")
            
            self.cap = cap
            self.get_logger().info(f'✓ Camera opened on /dev/video{self.device_id}')
            actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = cap.get(cv2.CAP_PROP_FPS)
            self.get_logger().info(
                f'Actual resolution: {actual_w}x{actual_h} @ {actual_fps:.2f}fps'
            )
        except Exception as e:
            self.get_logger().warn(f'Camera init failed: {e}, using mock mode')
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
