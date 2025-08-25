#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import os
import time
from PIL import Image
import io

class MockCameraNode(Node):
    def __init__(self):
        super().__init__('mock_camera_node')
        
        # Set logging level to INFO to reduce debug spam
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        
        # Parameters
        self.declare_parameter('image_folder', 'src/recycling_robot/recycling_robot/test_images')
        self.declare_parameter('publish_rate', 3.0)  # Publish every 3 seconds
        self.declare_parameter('image_quality', 85)  # JPEG quality
        self.declare_parameter('auto_fallback', True)  # Automatically use mock if no real camera
        
        # Get parameters
        self.image_folder = self.get_parameter('image_folder').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.image_quality = self.get_parameter('image_quality').value
        self.auto_fallback = self.get_parameter('auto_fallback').value
        
        # Publisher for compressed images
        self.image_publisher = self.create_publisher(
            CompressedImage,
            'camera/image_raw',
            10
        )
        
        # Timer for publishing images
        self.timer = self.create_timer(self.publish_rate, self.publish_test_image)
        
        # State
        self.current_image_index = 0
        self.test_images = []
        
        # Load test images
        self._load_test_images()
        
        self.get_logger().info('[MockCamera] Mock camera node started')
        self.get_logger().info(f'[MockCamera] Image folder: {self.image_folder}')
        self.get_logger().info(f'[MockCamera] Publish rate: {self.publish_rate}s')
        self.get_logger().info(f'[MockCamera] Found {len(self.test_images)} test images')

    def _load_test_images(self):
        """Load test images from the specified folder - prioritize existing real images"""
        try:
            # First, try to find the absolute path to test_images folder
            possible_paths = [
                self.image_folder,  # Direct path as specified
                os.path.join(os.getcwd(), self.image_folder),  # Relative to current working directory
                os.path.join('/workspace/ros2_ws', self.image_folder),  # Docker workspace path
                os.path.expanduser(f'~/{self.image_folder}'),  # User home directory
            ]
            
            test_images_dir = None
            for path in possible_paths:
                if os.path.exists(path) and os.path.isdir(path):
                    test_images_dir = path
                    break
                    
            if test_images_dir is None:
                self.get_logger().error(f'[ERROR] Could not find test images directory. Tried:')
                for path in possible_paths:
                    self.get_logger().error(f'  - {path}')
                # Fall back to creating samples
                test_images_dir = os.path.join(os.getcwd(), 'test_images')
                self.get_logger().warn(f'[WARN] Creating fallback directory: {test_images_dir}')
                self._create_sample_images(test_images_dir)
            
            # Get list of image files
            image_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.webp']
            image_files = []
            
            for filename in os.listdir(test_images_dir):
                if any(filename.lower().endswith(ext) for ext in image_extensions):
                    full_path = os.path.join(test_images_dir, filename)
                    if os.path.isfile(full_path):
                        image_files.append(full_path)
            
            # Sort images for consistent cycling
            image_files.sort()
            
            if image_files:
                self.test_images = image_files
                self.get_logger().info(f'[MockCamera] Successfully loaded {len(self.test_images)} real test images')
                for i, img_path in enumerate(self.test_images[:5]):  # Show first 5 images
                    self.get_logger().info(f'  [{i+1}] {os.path.basename(img_path)}')
                if len(self.test_images) > 5:
                    self.get_logger().info(f'  ... and {len(self.test_images) - 5} more images')
            else:
                self.get_logger().warn('[WARN] No real test images found, creating sample images...')
                self._create_sample_images(test_images_dir)
                # Reload after creating samples
                for filename in os.listdir(test_images_dir):
                    if any(filename.lower().endswith(ext) for ext in image_extensions):
                        full_path = os.path.join(test_images_dir, filename)
                        if os.path.isfile(full_path):
                            self.test_images.append(full_path)
                self.test_images.sort()
            
        except Exception as e:
            self.get_logger().error(f'[ERROR] Failed to load test images: {e}')
            # Create fallback in current directory
            fallback_dir = os.path.join(os.getcwd(), 'fallback_test_images')
            self._create_sample_images(fallback_dir)
            self.image_folder = fallback_dir
            self._load_test_images()

    def _create_sample_images(self, directory):
        """Create sample test images if none exist"""
        try:
            os.makedirs(directory, exist_ok=True)
            
            # Create a simple colored square image for testing
            from PIL import Image, ImageDraw, ImageFont
            
            # Create different colored squares for different materials
            materials_colors = [
                ('cardboard', (139, 69, 19)),   # Brown
                ('glass', (100, 149, 237)),     # Blue
                ('metal', (192, 192, 192)),     # Silver
                ('plastic', (34, 139, 34)),     # Green
                ('trash', (47, 79, 79))         # Dark gray
            ]
            
            for material, color in materials_colors:
                # Create 640x480 image (common camera resolution)
                img = Image.new('RGB', (640, 480), color)
                draw = ImageDraw.Draw(img)
                
                # Add a border
                draw.rectangle([10, 10, 630, 470], outline=(255, 255, 255), width=5)
                
                # Add material name
                try:
                    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 48)
                except:
                    font = ImageFont.load_default()
                
                # Center the text
                text = material.upper()
                bbox = draw.textbbox((0, 0), text, font=font)
                text_width = bbox[2] - bbox[0]
                text_height = bbox[3] - bbox[1]
                x = (640 - text_width) // 2
                y = (480 - text_height) // 2
                
                # Draw text with outline
                for dx, dy in [(-2, -2), (-2, 2), (2, -2), (2, 2)]:
                    draw.text((x+dx, y+dy), text, font=font, fill=(0, 0, 0))
                draw.text((x, y), text, font=font, fill=(255, 255, 255))
                
                # Save image
                filename = f'sample_{material}.jpg'
                filepath = os.path.join(directory, filename)
                img.save(filepath, 'JPEG', quality=self.image_quality)
            
            self.get_logger().info(f'[MockCamera] Created {len(materials_colors)} sample test images in {directory}')
            
        except Exception as e:
            self.get_logger().error(f'[ERROR] Failed to create sample images: {e}')

    def publish_test_image(self):
        """Publish a test image from the loaded images"""
        try:
            if not self.test_images:
                self.get_logger().warn('[WARN] No test images available')
                return
            
            # Get current image
            image_path = self.test_images[self.current_image_index]
            image_name = os.path.basename(image_path)
            
            # Load and potentially resize image
            with Image.open(image_path) as img:
                # Convert to RGB if necessary
                if img.mode != 'RGB':
                    img = img.convert('RGB')
                
                # Optionally resize very large images to prevent memory issues
                max_size = (1920, 1080)
                if img.size[0] > max_size[0] or img.size[1] > max_size[1]:
                    img.thumbnail(max_size, Image.Resampling.LANCZOS)
                    self.get_logger().debug(f'[MockCamera] Resized {image_name} to {img.size}')
                
                # Compress to JPEG
                buffer = io.BytesIO()
                img.save(buffer, format='JPEG', quality=self.image_quality)
                compressed_data = buffer.getvalue()
            
            # Create ROS2 message
            msg = CompressedImage()
            msg.format = 'jpeg'
            msg.data = compressed_data
            
            # Set timestamp
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "mock_camera_frame"
            
            # Publish
            self.image_publisher.publish(msg)
            
            # Log current image being published
            self.get_logger().info(f'[MockCamera] Published: {image_name} ({len(compressed_data)} bytes)')
            
            # Move to next image
            self.current_image_index = (self.current_image_index + 1) % len(self.test_images)
            
        except Exception as e:
            self.get_logger().error(f'[ERROR] Failed to publish test image: {e}')
            # Skip to next image on error
            self.current_image_index = (self.current_image_index + 1) % len(self.test_images) if self.test_images else 0

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MockCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in mock camera node: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()