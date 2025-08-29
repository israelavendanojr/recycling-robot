#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Empty
import cv2
import os
import io
import time
import threading
from PIL import Image
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        # Parameters
        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('image_quality', 85)
        self.declare_parameter('snapshot_path', '/shared/current_frame.jpg')
        self.declare_parameter('retry_attempts', 3)
        self.declare_parameter('retry_delay', 1.0)

        # Get parameters
        self.device_id = self.get_parameter('device_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.image_quality = self.get_parameter('image_quality').value
        self.snapshot_path = self.get_parameter('snapshot_path').value
        self.retry_attempts = self.get_parameter('retry_attempts').value
        self.retry_delay = self.get_parameter('retry_delay').value

        # State
        self.pipeline_state = "idle"
        self.camera = None
        self.camera_lock = threading.Lock()
        self.camera_connected = False
        self.last_reconnect_attempt = 0
        self.reconnect_cooldown = 5.0  # seconds

        # Publisher
        self.publisher = self.create_publisher(CompressedImage, '/pipeline/image_raw', 10)

        # Manual capture subscription (for Docker environments)
        self.capture_sub = self.create_subscription(
            Empty, 
            '/camera/capture', 
            self.manual_capture_callback, 
            10
        )

        # Pipeline state subscription
        self.pipeline_state_sub = self.create_subscription(
            String, 
            '/pipeline/state', 
            self.pipeline_state_callback, 
            10
        )

        # Initialize camera
        self._initialize_camera()

        # Manual capture mode - no timer, just ready state
        if self.camera_connected:
            self.get_logger().info('[Camera] Ready')
            self.get_logger().info('[Camera] Manual capture mode active')
            self.get_logger().info('[Camera] Use: ros2 topic pub /camera/capture std_msgs/msg/Empty')
        else:
            # Retry initialization periodically
            self.timer = self.create_timer(self.reconnect_cooldown, self._retry_camera_initialization)
            self.get_logger().error('[Camera] Camera initialization failed, will retry periodically')

    def pipeline_state_callback(self, msg):
        """Handle pipeline state updates"""
        try:
            self.pipeline_state = msg.data
            self.get_logger().debug(f'[Camera] Pipeline state: {self.pipeline_state}')
        except Exception as e:
            self.get_logger().error(f'[Camera] Pipeline state callback error: {e}')

    def manual_capture_callback(self, msg):
        """Handle manual capture request from ROS2 topic"""
        self.get_logger().info('[Camera] Manual capture requested')
        self.capture_and_publish_frame()

    def _initialize_camera(self):
        """Initialize camera with robust error handling"""
        try:
            self.get_logger().info(f'[Camera] Initializing camera device {self.device_id}...')
            
            # Release any existing camera
            if self.camera is not None:
                self.camera.release()
                time.sleep(0.5)  # Give time for device to be released

            # Try to open the camera
            for attempt in range(self.retry_attempts):
                try:
                    self.camera = cv2.VideoCapture(self.device_id)
                    
                    if not self.camera.isOpened():
                        self.get_logger().warn(f'[Camera] Attempt {attempt + 1}/{self.retry_attempts}: Camera not opened')
                        if attempt < self.retry_attempts - 1:
                            time.sleep(self.retry_delay)
                        continue

                    # Set camera properties
                    self._configure_camera()
                    
                    # Test camera by capturing a frame
                    ret, frame = self.camera.read()
                    if not ret or frame is None:
                        self.get_logger().warn(f'[Camera] Attempt {attempt + 1}/{self.retry_attempts}: Cannot capture frame')
                        if attempt < self.retry_attempts - 1:
                            time.sleep(self.retry_delay)
                        continue

                    # Success!
                    self.camera_connected = True
                    self._log_camera_info()
                    self.get_logger().info(f'[Camera] ✅ Camera initialized successfully')
                    return True

                except Exception as e:
                    self.get_logger().warn(f'[Camera] Attempt {attempt + 1}/{self.retry_attempts} failed: {e}')
                    if attempt < self.retry_attempts - 1:
                        time.sleep(self.retry_delay)

            # All attempts failed
            self.camera_connected = False
            self.get_logger().error('[Camera] ❌ Failed to initialize camera after all attempts')
            if self.camera is not None:
                self.camera.release()
                self.camera = None
            return False

        except Exception as e:
            self.get_logger().error(f'[Camera] Camera initialization error: {e}')
            self.camera_connected = False
            return False

    def _configure_camera(self):
        """Configure camera properties"""
        try:
            # Set resolution
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

            # Set frame rate
            self.camera.set(cv2.CAP_PROP_FPS, self.fps)

            # Try to set MJPEG format
            self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))

            # Enable auto-exposure
            self.camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)

            # Set buffer size to 1 to get latest frames
            self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            # Enable auto-focus if supported
            self.camera.set(cv2.CAP_PROP_AUTOFOCUS, 1)

            self.get_logger().info('[Camera] Camera properties configured')

        except Exception as e:
            self.get_logger().warn(f'[Camera] Error configuring camera properties: {e}')

    def _log_camera_info(self):
        """Log camera capabilities and current settings"""
        try:
            actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
            fourcc = int(self.camera.get(cv2.CAP_PROP_FOURCC))
            fourcc_str = ''.join([chr((fourcc >> 8 * i) & 0xFF) for i in range(4)])

            self.get_logger().info(f'[Camera] Resolution: {actual_width}x{actual_height}')
            self.get_logger().info(f'[Camera] FPS: {actual_fps}')
            self.get_logger().info(f'[Camera] Format: {fourcc_str}')

        except Exception as e:
            self.get_logger().warn(f'[Camera] Could not read camera info: {e}')

    def _capture_frame(self):
        """Capture a frame from the camera"""
        with self.camera_lock:
            if not self.camera_connected or self.camera is None:
                return None

            try:
                ret, frame = self.camera.read()
                if not ret or frame is None:
                    self.get_logger().warn('[Camera] Failed to capture frame')
                    return None

                # Convert BGR to RGB
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # Convert to PIL Image
                pil_image = Image.fromarray(frame_rgb)
                return pil_image

            except Exception as e:
                self.get_logger().error(f'[Camera] Frame capture error: {e}')
                return None

    def _write_snapshot_atomic(self, image_bytes: bytes, format_type: str):
        """Write snapshot with atomic operation"""
        try:
            # Determine the actual snapshot path based on format
            if format_type == 'JPEG':
                actual_path = '/shared/current_frame.jpg'
            else:  # PNG fallback
                actual_path = '/shared/current_frame.png'
            
            snap_dir = os.path.dirname(actual_path) or '.'
            os.makedirs(snap_dir, exist_ok=True)
            tmp_path = f'{actual_path}.tmp'
            
            with open(tmp_path, 'wb') as f:
                f.write(image_bytes)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp_path, actual_path)  # atomic on POSIX
            
            self.get_logger().debug(f'[Camera] Snapshot saved: {actual_path}')
            
        except Exception as e:
            self.get_logger().error(f'[Camera] Failed to write snapshot: {e}')

    def capture_and_publish_frame(self):
        """Capture and publish a single frame"""
        try:
            # Skip if pipeline is busy
            if self.pipeline_state == "processing":
                self.get_logger().info('[Camera] Pipeline busy, skipping frame capture')
                return

            # Check camera connection
            if not self.camera_connected:
                self.get_logger().warn('[Camera] Camera not connected')
                return

            # Capture frame
            frame = self._capture_frame()
            if frame is None:
                self.get_logger().warn('[Camera] No frame captured')
                return

            # Convert to bytes with fallback
            image_bytes = None
            format_type = 'JPEG'

            try:
                # Try JPEG first
                buf = io.BytesIO()
                frame.save(buf, format='JPEG', quality=self.image_quality)
                image_bytes = buf.getvalue()
                format_type = 'JPEG'

            except Exception as jpeg_error:
                self.get_logger().warn(f'[Camera] JPEG encoding failed, falling back to PNG: {jpeg_error}')
                try:
                    # Fallback to PNG
                    buf = io.BytesIO()
                    frame.save(buf, format='PNG')
                    image_bytes = buf.getvalue()
                    format_type = 'PNG'

                except Exception as png_error:
                    self.get_logger().error(f'[Camera] Both JPEG and PNG encoding failed: {png_error}')
                    return

            # Publish on ROS2 pipeline topic
            msg = CompressedImage()
            msg.format = 'jpeg' if format_type == 'JPEG' else 'png'
            msg.data = image_bytes
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            self.publisher.publish(msg)

            # Write snapshot for backend
            self._write_snapshot_atomic(image_bytes, format_type)

            self.get_logger().info('[Camera] Frame Captured → published to /pipeline/image_raw')

        except Exception as e:
            self.get_logger().error(f'[Camera] Failed to capture and publish frame: {e}')

    def _retry_camera_initialization(self):
        """Periodically retry camera initialization if disconnected"""
        if not self.camera_connected:
            self.get_logger().info('[Camera] Retrying camera initialization...')
            if self._initialize_camera():
                # Successfully reconnected
                self.timer.cancel()
                self.get_logger().info('[Camera] Camera reconnected, ready for manual capture')

    def destroy_node(self):
        """Clean up camera resources"""
        try:
            self.get_logger().info('[Camera] Shutting down camera node...')
            
            with self.camera_lock:
                if self.camera is not None:
                    self.camera.release()
                    self.camera = None
                    self.get_logger().info('[Camera] Camera released')
            
            super().destroy_node()
            
        except Exception as e:
            self.get_logger().error(f'[Camera] Error during shutdown: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = None
    ros_shutdown_called = False
    
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            try: 
                node.destroy_node()
            except Exception: 
                pass
        try: 
            if not ros_shutdown_called:
                rclpy.shutdown()
                ros_shutdown_called = True
        except Exception as e:
            if "rcl_shutdown already called" not in str(e):
                print(f'Error during shutdown: {e}')

if __name__ == '__main__':
    main()
