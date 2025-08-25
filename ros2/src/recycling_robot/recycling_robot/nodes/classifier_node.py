#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import time
import requests
import json
import os
import signal
import threading
from rclpy.executors import ExternalShutdownException
import torch
import torch.nn.functional as F
from torchvision import transforms
from PIL import Image as PILImage
import io
import sqlite3
from datetime import datetime

class ClassifierNode(Node):
    def __init__(self):
        super().__init__('classifier_node')
        
        # Set logging level to INFO to reduce debug spam
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        
        # Parameters
        self.declare_parameter('api_base_url', 'http://backend:8000')
        self.declare_parameter('inference_interval', 3.0)
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('model_path', 'src/recycling_robot/recycling_robot/models/recycler.pt')
        
        # Get API URL from environment or parameter
        self.api_base = os.getenv("API_BASE_URL", self.get_parameter("api_base_url").value)
        self.interval = self.get_parameter('inference_interval').value
        self.threshold = self.get_parameter('confidence_threshold').value
        self.model_path = self.get_parameter('model_path').value
        
        # State
        self.latest_image = None
        self.classes = ['cardboard', 'glass', 'metal', 'plastic', 'trash']
        self._running = True
        self._backend_ready = False
        self._shutdown_event = threading.Event()
        
        # PyTorch model setup
        self.model = None
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        # Load the PyTorch model
        self._load_model()
        
        # Initialize SQLite database
        self._init_database()
        
        self.get_logger().info('🚀 Classifier node started')
        self.get_logger().info(f'📱 API endpoint: {self.api_base}')
        self.get_logger().info(f'💻 Using device: {self.device}')
        
        # Wait for backend before starting subscriptions and timers
        self._wait_for_backend()
        
        # Only create subscriptions and timer after backend is ready
        if self._running:  # Check if we haven't been shutdown during backend wait
            self.subscription = self.create_subscription(
                CompressedImage, 'camera/image_raw', self.image_callback, 10
            )
            
            # Publisher for classification results
            self.classification_publisher = self.create_publisher(
                String,
                'classifier/result',
                10
            )
            
            # Auto-classify timer
            self.timer = self.create_timer(self.interval, self.auto_classify)
            self.get_logger().info('✅ Backend ready, classification service active')
        else:
            self.get_logger().error('❌ Failed to start classification service - backend unavailable')

    def _wait_for_backend(self):
        """Wait for backend to be ready with exponential backoff"""
        max_wait_time = 60.0  # Maximum total wait time
        retry_delay = 1.0     # Initial retry delay
        max_retry_delay = 8.0 # Maximum retry delay
        start_time = time.time()
        
        logged_waiting = False
        self.get_logger().info('⏳ Waiting for backend to be ready...')
        
        while self._running and not self._shutdown_event.is_set():
            try:
                response = requests.get(f'{self.api_base}/api/health', timeout=5.0)
                if response.status_code == 200:
                    self._backend_ready = True
                    self.get_logger().info('✅ Backend is ready!')
                    return
                else:
                    if not logged_waiting:
                        self.get_logger().info(f'⏳ Backend responded with status {response.status_code}, retrying...')
                        logged_waiting = True
                        
            except requests.exceptions.RequestException as e:
                if not logged_waiting:
                    self.get_logger().info(f'⏳ Backend not responding: {e}, retrying...')
                    logged_waiting = True
            
            # Check if we've exceeded max wait time
            if time.time() - start_time > max_wait_time:
                self.get_logger().error(f'❌ Backend not ready after {max_wait_time}s, giving up')
                self._running = False
                return
            
            # Exponential backoff
            time.sleep(retry_delay)
            retry_delay = min(retry_delay * 2, max_retry_delay)

    def _load_model(self):
        """Load PyTorch model with clean logging"""
        try:
            self.get_logger().info('📦 Loading PyTorch model...')
            
            # Check if model file exists
            if not os.path.exists(self.model_path):
                self.get_logger().error(f'❌ Model file not found: {self.model_path}')
                self.get_logger().error('❌ Cannot continue without model file')
                self._running = False
                return
            
            # Load the actual model
            self.model = torch.load(self.model_path, map_location=self.device)
            self.model.eval()
            self.get_logger().info('✅ PyTorch model loaded successfully')
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load model: {e}')
            self.get_logger().error('❌ Cannot continue without model')
            self._running = False

    def _init_database(self):
        """Initialize SQLite database with clean logging"""
        try:
            db_path = os.path.expanduser('~/classifications.db')
            os.makedirs(os.path.dirname(db_path), exist_ok=True)
            
            conn = sqlite3.connect(db_path)
            cursor = conn.cursor()
            
            # Create classifications table
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS classifications (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TEXT NOT NULL,
                    label TEXT NOT NULL,
                    confidence REAL NOT NULL,
                    raw_logits TEXT,
                    image_source TEXT,
                    created_at TEXT DEFAULT CURRENT_TIMESTAMP
                )
            ''')
            
            conn.commit()
            conn.close()
            
            self.db_path = db_path
            self.get_logger().info(f'💾 Database initialized: {db_path}')
            
        except Exception as e:
            self.get_logger().error(f'❌ Database initialization failed: {e}')
            self.db_path = None

    def _log_to_database(self, result):
        """Log classification result to SQLite database"""
        if self.db_path is None:
            return
            
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            # Insert classification result
            cursor.execute('''
                INSERT INTO classifications (timestamp, label, confidence, raw_logits, image_source)
                VALUES (?, ?, ?, ?, ?)
            ''', (
                datetime.now().isoformat(),
                result['class'],
                result['confidence'],
                json.dumps(result.get('raw_logits', [])),
                'camera'  # Generic source since both mock and real publish to same topic
            ))
            
            conn.commit()
            conn.close()
            
            # Only log successful database writes at DEBUG level
            self.get_logger().debug(f'💾 Logged to database: {result["class"]} ({result["confidence"]*100:.1f}%)')
            
        except Exception as e:
            self.get_logger().error(f'❌ Database write failed: {e}')

    def _send_to_backend(self, result):
        """Send classification result to backend API"""
        try:
            # Prepare data for backend
            backend_data = {
                'class': result['class'],
                'confidence': result['confidence'],
                'timestamp': time.time()
            }
            
            # Send POST request to backend
            response = requests.post(
                f'{self.api_base}/api/events',
                json=backend_data,
                timeout=5.0
            )
            
            if response.status_code == 200:
                self.get_logger().debug(f'✅ Sent to backend: {result["class"]} ({result["confidence"]*100:.1f}%)')
            else:
                self.get_logger().warn(f'⚠️  Backend responded with status {response.status_code}')
                
        except Exception as e:
            self.get_logger().error(f'❌ Failed to send to backend: {e}')

    def _preprocess_image(self, compressed_image_msg):
        """Preprocess compressed image for model inference using PIL + Torch only"""
        try:
            # Extract image data from CompressedImage message
            if compressed_image_msg.format != 'jpeg':
                self.get_logger().warn(f'⚠️  Unsupported image format: {compressed_image_msg.format}')
                return None
            
            # Convert compressed image data to PIL Image
            image_data = compressed_image_msg.data
            pil_image = PILImage.open(io.BytesIO(image_data))
            
            # Convert to RGB if necessary
            if pil_image.mode != 'RGB':
                pil_image = pil_image.convert('RGB')
            
            # Manual preprocessing without transforms to avoid NumPy issues
            # Resize to 224x224
            pil_image = pil_image.resize((224, 224), PILImage.Resampling.LANCZOS)
            
            # Convert to tensor manually
            import torch
            # Convert PIL image to list of pixel values
            pixel_data = list(pil_image.getdata())
            
            # Reshape to (height, width, channels)
            height, width = 224, 224
            channels = 3
            tensor_data = []
            
            for y in range(height):
                row = []
                for x in range(width):
                    pixel = pixel_data[y * width + x]
                    # Normalize to [0, 1] and convert to float
                    r, g, b = pixel[0] / 255.0, pixel[1] / 255.0, pixel[2] / 255.0
                    row.append([r, g, b])
                tensor_data.append(row)
            
            # Convert to tensor and reshape to (channels, height, width)
            tensor = torch.tensor(tensor_data, dtype=torch.float32)
            tensor = tensor.permute(2, 0, 1)  # (H, W, C) -> (C, H, W)
            
            # Add batch dimension
            tensor = tensor.unsqueeze(0)
            
            # Only log preprocessing at DEBUG level
            self.get_logger().debug(f'🖼️  Image preprocessed: {pil_image.size[0]}x{pil_image.size[1]} → {tensor.shape}')
            return tensor.to(self.device)
            
        except Exception as e:
            self.get_logger().error(f'❌ Image preprocessing failed: {e}')
            return None

    def _run_inference(self, image_tensor):
        """Run inference on the preprocessed image"""
        if self.model is None:
            return None, 0.0
            
        try:
            with torch.no_grad():
                # Run inference
                outputs = self.model(image_tensor)
                
                # Get probabilities
                probs = F.softmax(outputs, dim=1)
                confidence, predicted = torch.max(probs, 1)
                
                # Convert to Python types
                confidence = confidence.item()
                predicted_idx = predicted.item()
                predicted_class = self.classes[predicted_idx]
                
                return predicted_class, confidence
                
        except Exception as e:
            self.get_logger().error(f'[ERROR] Inference failed: {e}')
            return None, 0.0

    def image_callback(self, msg):
        """Handle incoming image messages with clean logging"""
        try:
            self.latest_image = msg
            self.get_logger().debug('[DEBUG] Received image from camera')
        except Exception as e:
            self.get_logger().error(f'[ERROR] Image callback error: {e}')

    def auto_classify(self):
        """Auto-classify latest image with clean logging"""
        if not self._running or self.latest_image is None:
            return
            
        try:
            # Preprocess image
            image_tensor = self._preprocess_image(self.latest_image)
            if image_tensor is None:
                return
            
            # Run inference
            predicted_class, confidence = self._run_inference(image_tensor)
            if predicted_class is None:
                return
            
            # Check confidence threshold
            if confidence < self.threshold:
                self.get_logger().debug(f'[DEBUG] Low confidence prediction: {predicted_class} ({confidence*100:.1f}%) < {self.threshold*100:.1f}%')
                return
            
            # Create result
            result = {
                'class': predicted_class,
                'confidence': confidence,
                'raw_logits': None  # We could add this if needed
            }
            
            # Log to database
            self._log_to_database(result)
            
            # Publish result
            result_msg = String()
            result_msg.data = json.dumps(result)
            self.classification_publisher.publish(result_msg)
            
            # Send to backend API
            self._send_to_backend(result)
            
            # Log successful classification
            self.get_logger().info(f'[Classifier] Predicted: {predicted_class} ({confidence*100:.1f}% confidence)')
            
        except Exception as e:
            self.get_logger().error(f'[ERROR] Auto-classify failed: {e}')

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        self.get_logger().info('[INFO] Shutdown signal received')
        self._running = False
        self._shutdown_event.set()

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('[INFO] Classifier node shutting down')
        self._running = False
        self._shutdown_event.set()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ClassifierNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    except Exception as e:
        print(f'Unexpected error: {e}')
    finally:
        if 'node' in locals():
            try:
                node.destroy_node()
            except Exception as e:
                print(f'Error during node destruction: {e}')
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f'Error during shutdown: {e}')

if __name__ == '__main__':
    main()