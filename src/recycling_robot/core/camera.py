"""
ROS2 Camera Node - Publishes camera images to /camera/image_raw topic.
Supports Pi Camera, ArduCam, and mock camera for testing.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
from typing import Optional
import threading
import time

from recycling_robot.core.camera import CameraManager

class CameraNode(Node):
    """
    ROS2 node that publishes camera frames as sensor_msgs/Image messages.
    """
    
    def __init__(self):
        super().__init__('camera_node')
        
        # Declare parameters
        self.declare_parameter('camera_type', 'auto')
        self.declare_parameter('resolution_width', 640)
        self.declare_parameter('resolution_height', 480)
        self.declare_parameter('device_id', 0)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('frame_id', 'camera_link')
        
        # Get parameters
        camera_type = self.get_parameter('camera_type').get_parameter_value().string_value
        width = self.get_parameter('resolution_width').get_parameter_value().integer_value
        height = self.get_parameter('resolution_height').get_parameter_value().integer_value
        device_id = self.get_parameter('device_id').get_parameter_value().integer_value
        fps = self.get_parameter('fps').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        self.resolution = (width, height)
        self.publish_rate = fps
        
        # Initialize components
        self.bridge = CvBridge()
        self.camera_manager: Optional[CameraManager] = None
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Threading
        self._publish_thread: Optional[threading.Thread] = None
        self._shutdown_event = threading.Event()
        
        # Statistics
        self.frame_count = 0
        self.start_time = time.time()
        
        # Initialize camera
        self.get_logger().info(f'Initializing camera: {camera_type} @ {self.resolution}')
        try:
            self.camera_manager = CameraManager(
                camera_type=camera_type,
                resolution=self.resolution,
                device_id=device_id
            )
            self.camera_manager.start()
            self.get_logger().info('Camera initialized successfully')
            
            # Start publishing thread
            self._start_publishing()
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize camera: {e}')
            raise
    
    def _start_publishing(self) -> None:
        """Start the image publishing thread."""
        self._publish_thread = threading.Thread(
            target=self._publish_loop,
            daemon=True,
            name="CameraPublishThread"
        )
        self._publish_thread.start()
        self.get_logger().info(f'Started publishing at {self.publish_rate} fps')
    
    def _publish_loop(self) -> None:
        """Main publishing loop running in separate thread."""
        interval = 1.0 / self.publish_rate
        next_publish = time.time()
        
        while not self._shutdown_event.is_set():
            current_time = time.time()
            
            # Wait for next publish time
            if current_time < next_publish:
                sleep_time = min(0.001, next_publish - current_time)
                time.sleep(sleep_time)
                continue
            
            next_publish = current_time + interval
            
            try:
                # Capture frame
                frame = self.camera_manager.capture_frame()
                
                # Convert to ROS2 Image message
                ros_image = self._numpy_to_ros_image(frame)
                
                # Publish
                self.publisher.publish(ros_image)
                
                # Update statistics
                self.frame_count += 1
                if self.frame_count % 100 == 0:
                    elapsed = time.time() - self.start_time
                    avg_fps = self.frame_count / elapsed
                    self.get_logger().info(f'Published {self.frame_count} frames, avg FPS: {avg_fps:.2f}')
                
            except Exception as e:
                self.get_logger().error(f'Error in publish loop: {e}')
                time.sleep(0.1)  # Brief pause on error
    
    def _numpy_to_ros_image(self, frame: np.ndarray) -> Image:
        """
        Convert numpy array to ROS2 Image message.
        
        Args:
            frame: RGB numpy array (H, W, 3)
            
        Returns:
            ROS2 Image message
        """
        # Create header with current timestamp
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        
        # Convert using cv_bridge
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
        ros_image.header = header
        
        return ros_image
    
    def destroy_node(self) -> None:
        """Clean shutdown of camera node."""
        self.get_logger().info('Shutting down camera node...')
        
        # Stop publishing thread
        self._shutdown_event.set()
        if self._publish_thread and self._publish_thread.is_alive():
            self._publish_thread.join(timeout=2.0)
        
        # Stop camera
        if self.camera_manager:
            self.camera_manager.stop()
        
        # Print final statistics
        if self.frame_count > 0:
            elapsed = time.time() - self.start_time
            avg_fps = self.frame_count / elapsed
            self.get_logger().info(f'Final stats: {self.frame_count} frames in {elapsed:.1f}s, avg FPS: {avg_fps:.2f}')
        
        super().destroy_node()

def main(args=None):
    """Main entry point for camera node."""
    rclpy.init(args=args)
    
    try:
        node = CameraNode()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Camera node error: {e}')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()