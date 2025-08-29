#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Empty
import os, io, time
from PIL import Image

class MockCameraNode(Node):
    def __init__(self):
        super().__init__('mock_camera_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        # Resolve default images from installed share/
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory('recycling_robot')
            default_images = os.path.join(share_dir, 'test_images')
        except Exception:
            default_images = 'test_images'

        # Params
        self.declare_parameter('image_folder', default_images)
        self.declare_parameter('publish_rate', 3.0)        # seconds
        self.declare_parameter('image_quality', 85)         # JPEG quality
        self.declare_parameter('snapshot_enabled', True)    # write latest frame to file for backend
        self.declare_parameter('snapshot_path', '/shared/current_frame.jpg')

        self.image_folder = self.get_parameter('image_folder').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.image_quality = int(self.get_parameter('image_quality').value)
        self.snapshot_enabled = bool(self.get_parameter('snapshot_enabled').value)
        self.snapshot_path = self.get_parameter('snapshot_path').value

        # Publisher (use the pipeline topic for downstream nodes)
        self.publisher = self.create_publisher(CompressedImage, '/pipeline/image_raw', 10)

        # Manual capture subscription
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

        # Load images
        self.current_image_index = 0
        self.test_images = []
        self.pipeline_state = "idle"  # Track pipeline state
        self._load_test_images()

        # Timer (only if publish_rate > 0)
        if self.publish_rate > 0:
            self.timer = self.create_timer(self.publish_rate, self.publish_test_image)

        self.get_logger().info('[MockCamera] Mock camera node started')
        self.get_logger().info(f'[MockCamera] Image folder: {self.image_folder}')
        if self.publish_rate > 0:
            self.get_logger().info(f'[MockCamera] Publish rate: {self.publish_rate}s')
        else:
            self.get_logger().info('[MockCamera] Manual capture mode active')
        self.get_logger().info(f'[MockCamera] Found {len(self.test_images)} test images')
        if self.snapshot_enabled:
            self.get_logger().info(f'[MockCamera] Snapshot path: {self.snapshot_path}')

    def manual_capture_callback(self, msg):
        """Handle manual capture request from ROS2 topic"""
        self.get_logger().info('[MockCamera] Manual capture requested')
        self.publish_test_image()

    def pipeline_state_callback(self, msg):
        """Handle pipeline state updates"""
        try:
            self.pipeline_state = msg.data
            self.get_logger().debug(f'[MockCamera] Pipeline state: {self.pipeline_state}')
        except Exception as e:
            self.get_logger().error(f'[MockCamera] Pipeline state callback error: {e}')

    def _load_test_images(self):
        try:
            candidates = [
                self.image_folder,
                os.path.join(os.getcwd(), self.image_folder),
                os.path.join('/workspace/ros2_ws', self.image_folder),
                os.path.join('/workspace/ros2_ws/src/recycling_robot', 'test_images'),
                os.path.join('/workspace/ros2_ws/install/recycling_robot/share/recycling_robot', 'test_images'),
                os.path.expanduser(f'~/{self.image_folder}'),
            ]
            img_dir = next((p for p in candidates if os.path.isdir(p)), None)
            if img_dir is None:
                self.get_logger().error('[MockCamera] Could not find test images directory. Creating fallback.')
                img_dir = os.path.join(os.getcwd(), 'test_images')
                os.makedirs(img_dir, exist_ok=True)
                self._create_sample_images(img_dir)

            exts = ('.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.webp')
            files = [os.path.join(img_dir, f) for f in sorted(os.listdir(img_dir)) if f.lower().endswith(exts)]
            if not files:
                self.get_logger().warn('[MockCamera] No real test images found, creating samples...')
                self._create_sample_images(img_dir)
                files = [os.path.join(img_dir, f) for f in sorted(os.listdir(img_dir)) if f.lower().endswith(exts)]

            self.test_images = files
            if files:
                self.get_logger().info(f'[MockCamera] Successfully loaded {len(files)} images')
                for i, p in enumerate(files[:5]):
                    self.get_logger().info(f'  [{i+1}] {os.path.basename(p)}')
        except Exception as e:
            self.get_logger().error(f'[MockCamera] Failed to load test images: {e}')
            self.test_images = []

    def _create_sample_images(self, directory):
        from PIL import Image, ImageDraw, ImageFont
        os.makedirs(directory, exist_ok=True)
        materials = [
            ('cardboard', (139, 69, 19)), ('glass', (100, 149, 237)),
            ('metal', (192, 192, 192)), ('plastic', (34, 139, 34)),
            ('trash', (47, 79, 79))
        ]
        for name, color in materials:
            img = Image.new('RGB', (640, 480), color)
            draw = ImageDraw.Draw(img)
            try:
                font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 48)
            except Exception:
                font = ImageFont.load_default()
            text = name.upper()
            w, h = draw.textlength(text, font=font), 48
            x, y = (640 - int(w)) // 2, (480 - h) // 2
            draw.rectangle((10, 10, 630, 470), outline=(255, 255, 255), width=5)
            for dx, dy in [(-2,-2),(-2,2),(2,-2),(2,2)]:
                draw.text((x+dx, y+dy), text, font=font, fill=(0,0,0))
            draw.text((x, y), text, font=font, fill=(255,255,255))
            img.save(os.path.join(directory, f'sample_{name}.jpg'), 'JPEG', quality=85)
        self.get_logger().info(f'[MockCamera] Created {len(materials)} sample images in {directory}')

    def _write_snapshot_atomic(self, image_bytes: bytes, format_type: str):
        """Write snapshot with fallback support - always update SNAPSHOT_PATH"""
        try:
            if not self.snapshot_enabled or not self.snapshot_path:
                return
            
            # Determine the actual snapshot path based on format
            if format_type == 'JPEG':
                actual_path = '/shared/current_frame.jpg'
            else:  # PNG fallback
                actual_path = '/shared/current_frame.png'
            
            # Update the snapshot path so backend always knows the latest file
            self.snapshot_path = actual_path
            
            snap_dir = os.path.dirname(actual_path) or '.'
            os.makedirs(snap_dir, exist_ok=True)
            tmp_path = f'{actual_path}.tmp'
            
            with open(tmp_path, 'wb') as f:
                f.write(image_bytes)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp_path, actual_path)  # atomic on POSIX
            
            self.get_logger().info(f'[MockCamera] Snapshot saved: {actual_path} ({len(image_bytes)} bytes, {format_type})')
            
        except Exception as e:
            self.get_logger().error(f'[MockCamera] Failed to write snapshot: {e}')

    def publish_test_image(self):
        try:
            if not self.test_images:
                self.get_logger().warn('[MockCamera] No test images available')
                return
                
            # Skip publishing if pipeline is busy
            if self.pipeline_state == "processing":
                self.get_logger().info('⏸️ [MockCamera] Pipeline busy, skipping image publication (waiting for sorting to complete)')
                return

            path = self.test_images[self.current_image_index]
            name = os.path.basename(path)

            with Image.open(path) as img:
                if img.mode != 'RGB':
                    img = img.convert('RGB')
                # optional downscale guard
                if img.width > 1920 or img.height > 1080:
                    img.thumbnail((1920, 1080), Image.Resampling.LANCZOS)
                
                # Try JPEG first, fallback to PNG if it fails
                jpeg_bytes = None
                png_bytes = None
                format_type = 'JPEG'
                
                try:
                    # Try JPEG
                    buf = io.BytesIO()
                    img.save(buf, format='JPEG', quality=self.image_quality)
                    jpeg_bytes = buf.getvalue()
                    format_type = 'JPEG'
                    image_bytes = jpeg_bytes
                    self.get_logger().debug(f'[MockCamera] JPEG encoding successful ({len(jpeg_bytes)} bytes)')
                except Exception as jpeg_error:
                    self.get_logger().warn(f'[MockCamera] JPEG encoding failed, falling back to PNG: {jpeg_error}')
                    try:
                        # Fallback to PNG
                        buf = io.BytesIO()
                        img.save(buf, format='PNG')
                        png_bytes = buf.getvalue()
                        format_type = 'PNG'
                        image_bytes = png_bytes
                        self.get_logger().info(f'[MockCamera] PNG fallback successful ({len(png_bytes)} bytes)')
                    except Exception as png_error:
                        self.get_logger().error(f'[MockCamera] Both JPEG and PNG encoding failed: {png_error}')
                        return

            # Publish on ROS2 pipeline topic
            msg = CompressedImage()
            msg.format = 'jpeg' if format_type == 'JPEG' else 'png'
            msg.data = image_bytes
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'mock_camera_frame'
            self.publisher.publish(msg)

            # Write snapshot for backend (with fallback support)
            self._write_snapshot_atomic(image_bytes, format_type)

            self.get_logger().info(f'[MockCamera] Published to /pipeline/image_raw: {name} ({len(image_bytes)} bytes, {format_type})')
            self.current_image_index = (self.current_image_index + 1) % len(self.test_images)

        except Exception as e:
            self.get_logger().error(f'[MockCamera] Failed to publish test image: {e}')
            self.current_image_index = (self.current_image_index + 1) % max(1, len(self.test_images))

def main(args=None):
    rclpy.init(args=args)
    node = None
    ros_shutdown_called = False
    
    try:
        node = MockCameraNode()
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
