"""
ROS2 Camera Node - Enhanced with ArduCam support and better debugging.
Publishes camera images to /camera/image_raw topic.
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
import sys
import os

# Import our enhanced camera hardware abstraction
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'utils'))
from camera import CameraManager

class CameraNode(Node):
    """
    ROS2 node that publishes camera frames as sensor_msgs/Image messages.
    Enhanced with ArduCam support and better diagnostics.
    """
    
    def __init__(self):
        super().__init__('camera_node')
        
        # Declare parameters
        self.declare_parameter('camera_type', 'auto')  # auto, arducam, pi, usb, mock
        self.declare_parameter('resolution_width', 640)
        self.declare_parameter('resolution_height', 480)
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('debug_mode', True)  # Extra logging for troubleshooting
        
        # Get parameters
        camera_type = self.get_parameter('camera_type').get_parameter_value().string_value
        width = self.get_parameter('resolution_width').get_parameter_value().integer_value
        height = self.get_parameter('resolution_height').get_parameter_value().integer_value
        fps = self.get_parameter('fps').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        
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
        self.error_count = 0
        self.start_time = time.time()
        
        # Diagnostics timer for ArduCam troubleshooting
        if self.debug_mode:
            self.diag_timer = self.create_timer(10.0, self._print_diagnostics)
        
        # Initialize camera
        self.get_logger().info('='*60)
        self.get_logger().info('🎥 CAMERA NODE INITIALIZATION')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Camera Type: {camera_type}')
        self.get_logger().info(f'Resolution:  {self.resolution}')
        self.get_logger().info(f'FPS Target:  {fps}')
        self.get_logger().info(f'Debug Mode:  {self.debug_mode}')
        
        try:
            self.camera_manager = CameraManager(
                camera_type=camera_type,
                resolution=self.resolution,
                fps=fps
            )
            self.camera_manager.start()
            
            # Get detailed camera info
            camera_info = self.camera_manager.get_camera_info()
            self.get_logger().info('✓ Camera initialized successfully')
            self.get_logger().info(f'Actual Type: {camera_info["camera_type"]}')
            if camera_info.get("arducam_type"):
                self.get_logger().info(f'ArduCam Method: {camera_info["arducam_type"]}')
            self.get_logger().info(f'Backend: {camera_info["backend"]}')
            self.get_logger().info('='*60)
            
            # Start publishing thread
            self._start_publishing()
            
        except Exception as e:
            self.get_logger().error('='*60)
            self.get_logger().error('✗ CAMERA INITIALIZATION FAILED')
            self.get_logger().error(f'Error: {e}')
            self.get_logger().error('='*60)
            self._print_troubleshooting_tips()
            raise
    
    def _print_troubleshooting_tips(self):
        """Print troubleshooting information for camera issues."""
        self.get_logger().error('')
        self.get_logger().error('🔧 TROUBLESHOOTING TIPS:')
        self.get_logger().error('')
        self.get_logger().error('1. Check camera connection:')
        self.get_logger().error('   - Ensure ribbon cable is properly seated')
        self.get_logger().error('   - Check camera is enabled: raspi-config')
        self.get_logger().error('')
        self.get_logger().error('2. Check camera detection:')
        self.get_logger().error('   - Run: vcgencmd get_camera')
        self.get_logger().error('   - Run: ls /dev/video*')
        self.get_logger().error('')
        self.get_logger().error('3. For ArduCam specifically:')
        self.get_logger().error('   - Check ArduCam drivers are installed')
        self.get_logger().error('   - Try: libcamera-hello --list-cameras')
        self.get_logger().error('   - Ensure camera overlay is in /boot/config.txt')
        self.get_logger().error('')
        self.get_logger().error('4. Force camera type with parameter:')
        self.get_logger().error('   - camera_type:=arducam (force ArduCam)')
        self.get_logger().error('   - camera_type:=usb (force USB/V4L2)')
        self.get_logger().error('   - camera_type:=mock (testing mode)')
        self.get_logger().error('')
    
    def _start_publishing(self) -> None:
        """Start the image publishing thread."""
        self._publish_thread = threading.Thread(
            target=self._publish_loop,
            daemon=True,
            name="CameraPublishThread"
        )
        self._publish_thread.start()
        self.get_logger().info(f'📸 Started publishing at {self.publish_rate} fps')
    
    def _publish_loop(self) -> None:
        """Main publishing loop running in separate thread."""
        interval = 1.0 / self.publish_rate
        next_publish = time.time()
        consecutive_errors = 0
        
        while not self._shutdown_event.is_set():
            current_time = time.time()
            
            # Wait for next publish time
            if current_time < next_publish:
                sleep_time = min(0.001, next_publish - current_time)
                time.sleep(sleep_time)
                continue
            
            next_publish = current_time + interval
            
            try:
                # Capture frame using our enhanced camera abstraction
                frame = self.camera_manager.capture_frame()
                
                # Validate frame
                if frame is None or frame.size == 0:
                    self.error_count += 1
                    consecutive_errors += 1
                    if consecutive_errors <= 3:  # Only log first few errors
                        self.get_logger().warn('Received empty frame from camera')
                    continue
                
                # Reset error counter on success
                consecutive_errors = 0
                
                # Convert to ROS2 Image message
                ros_image = self._numpy_to_ros_image(frame)
                
                # Publish
                self.publisher.publish(ros_image)
                
                # Update statistics
                self.frame_count += 1
                
                # Periodic status updates
                if self.debug_mode and self.frame_count % 100 == 0:
                    elapsed = time.time() - self.start_time
                    avg_fps = self.frame_count / elapsed
                    self.get_logger().info(f'📊 {self.frame_count} frames published, avg FPS: {avg_fps:.2f}')
                
            except Exception as e:
                self.error_count += 1
                consecutive_errors += 1
                if consecutive_errors <= 5:  # Avoid spam
                    self.get_logger().error(f'Error in publish loop: {e}')
                elif consecutive_errors == 6:
                    self.get_logger().error('Too many consecutive errors, suppressing further error messages')
                time.sleep(0.1)  # Brief pause on error
    
    def _print_diagnostics(self):
        """Print diagnostic information periodically."""
        if not self.camera_manager:
            return
            
        try:
            # Get current camera info
            info = self.camera_manager.get_camera_info()
            elapsed = time.time() - self.start_time
            avg_fps = self.frame_count / elapsed if elapsed > 0 else 0
            
            self.get_logger().info('='*50)
            self.get_logger().info('📊 CAMERA DIAGNOSTICS')
            self.get_logger().info(f'Type: {info["camera_type"]} ({info["backend"]})')
            if info.get("arducam_type"):
                self.get_logger().info(f'ArduCam: {info["arducam_type"]}')
            self.get_logger().info(f'Frames: {self.frame_count} (errors: {self.error_count})')
            self.get_logger().info(f'Average FPS: {avg_fps:.2f}')
            self.get_logger().info(f'Uptime: {elapsed:.1f}s')
            self.get_logger().info('='*50)
            
        except Exception as e:
            self.get_logger().error(f'Error getting diagnostics: {e}')
    
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
        self.get_logger().info('🛑 Shutting down camera node...')
        
        # Stop publishing thread
        self._shutdown_event.set()
        if self._publish_thread and self._publish_thread.is_alive():
            self._publish_thread.join(timeout=2.0)
        
        # Stop camera
        if self.camera_manager:
            self.camera_manager.release()
        
        # Print final statistics
        if self.frame_count > 0:
            elapsed = time.time() - self.start_time
            avg_fps = self.frame_count / elapsed
            self.get_logger().info('='*50)
            self.get_logger().info('📊 FINAL STATISTICS')
            self.get_logger().info(f'Total frames: {self.frame_count}')
            self.get_logger().info(f'Total errors: {self.error_count}')
            self.get_logger().info(f'Runtime: {elapsed:.1f}s')
            self.get_logger().info(f'Average FPS: {avg_fps:.2f}')
            success_rate = ((self.frame_count / (self.frame_count + self.error_count)) * 100) if (self.frame_count + self.error_count) > 0 else 0
            self.get_logger().info(f'Success rate: {success_rate:.1f}%')
            self.get_logger().info('='*50)
        
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