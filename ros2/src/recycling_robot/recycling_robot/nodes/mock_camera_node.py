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
        self.declare_parameter('image_folder', 'test_images')
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
        """Load test images from the specified folder"""
        try:
            # Get the full path to the test images folder
            package_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
            test_images_dir = os.path.join(package_dir, self.image_folder)
            
            if not os.path.exists(test_images_dir):
                self.get_logger().warn(f'[WARN] Test images directory not found: {test_images_dir}')
                self.get_logger().info('[MockCamera] Creating sample test images...')
                self._create_sample_images(test_images_dir)
            
            # Get list of image files
            image_extensions = ['.jpg', '.jpeg', '.png', '.bmp']
            for filename in os.listdir(test_images_dir):
                if any(filename.lower().endswith(ext) for ext in image_extensions):
                    self.test_images.append(os.path.join(test_images_dir, filename))
            
            # Sort images for consistent cycling
            self.test_images.sort()
            
            if not self.test_images:
                self.get_logger().warn('[WARN] No test images found, creating sample images...')
                self._create_sample_images(test_images_dir)
                self._load_test_images()  # Reload after creating
            
        except Exception as e:
            self.get_logger().error(f'[ERROR] Failed to load test images: {e}')
            self._create_sample_images('test_images')

    def _create_sample_images(self, directory):
        """Create sample test images if none exist"""
        try:
            os.makedirs(directory, exist_ok=True)
            
            # Create a simple colored square image for testing
            from PIL import Image, ImageDraw
            
            # Create different colored squares for different materials
            colors = [
                (139, 69, 19),   # Brown (cardboard)
                (100, 149, 237), # Blue (glass)
                (192, 192, 192), # Silver (metal)
                (34, 139, 34),   # Green (plastic)
                (47, 79, 79)     # Dark (trash)
            ]
            
            for i, color in enumerate(colors):
                # Create 224x224 image (same size as model input)
                img = Image.new('RGB', (224, 224), color)
                draw = ImageDraw.Draw(img)
                
                # Add a simple pattern
                draw.rectangle([50, 50, 174, 174], outline=(255, 255, 255), width=3)
                
                # Add text label
                material_names = ['cardboard', 'glass', 'metal', 'plastic', 'trash']
                draw.text((112, 200), material_names[i], fill=(255, 255, 255))
                
                # Save image
                filename = f'test_{material_names[i]}.jpg'
                filepath = os.path.join(directory, filename)
                img.save(filepath, 'JPEG', quality=self.image_quality)
            
            self.get_logger().info(f'[MockCamera] Created {len(colors)} sample test images in {directory}')
            
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
            
            # Load and compress image
            with Image.open(image_path) as img:
                # Convert to RGB if necessary
                if img.mode != 'RGB':
                    img = img.convert('RGB')
                
                # Compress to JPEG
                buffer = io.BytesIO()
                img.save(buffer, format='JPEG', quality=self.image_quality)
                compressed_data = buffer.getvalue()
            
            # Create ROS2 message
            msg = CompressedImage()
            msg.format = 'jpeg'
            msg.data = compressed_data
            
            # Publish
            self.image_publisher.publish(msg)
            
            # Log every 10th image to reduce spam
            if self.current_image_index % 10 == 0:
                self.get_logger().info(f'[MockCamera] Published test image: {image_name}')
            
            # Move to next image
            self.current_image_index = (self.current_image_index + 1) % len(self.test_images)
            
        except Exception as e:
            self.get_logger().error(f'[ERROR] Failed to publish test image: {e}')

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
