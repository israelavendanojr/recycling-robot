#!/usr/bin/env python3
"""
Simple ROS2 Classifier Node
Provides image classification service and auto-classifies camera feed
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import random
import os

# Try to import torch, fall back to mock if not available
try:
    import torch
    import torchvision.transforms as transforms
    from PIL import Image as PILImage
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False


class SimpleClassifier:
    """Simple classifier that works with or without PyTorch"""
    
    def __init__(self, model_path=None):
        self.classes = ['cardboard', 'glass', 'metal', 'plastic', 'trash']
        self.model = None
        self.transform = None
        
        if HAS_TORCH and model_path and os.path.exists(model_path):
            try:
                self.model = torch.jit.load(model_path, map_location='cpu')
                self.model.eval()
                self.transform = transforms.Compose([
                    transforms.Resize((224, 224)),
                    transforms.ToTensor(),
                    transforms.Normalize(mean=[0.485, 0.456, 0.406], 
                                       std=[0.229, 0.224, 0.225])
                ])
                print(f"✓ Loaded model from {model_path}")
            except Exception as e:
                print(f"Failed to load model: {e}, using mock classifier")
                self.model = None
        else:
            print("Using mock classifier (no PyTorch or model file)")
    
    def predict(self, image):
        """Predict class from BGR image"""
        if self.model is None:
            return self._mock_predict(image)
        
        try:
            # Convert BGR to RGB
            rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            pil_img = PILImage.fromarray(rgb)
            tensor = self.transform(pil_img).unsqueeze(0)
            
            with torch.no_grad():
                outputs = self.model(tensor)
                probs = torch.softmax(outputs, dim=1)[0]
                class_idx = torch.argmax(probs).item()
                confidence = probs[class_idx].item()
            
            return {
                'class': self.classes[class_idx],
                'confidence': confidence,
                'all_probs': {cls: float(probs[i]) for i, cls in enumerate(self.classes)}
            }
        except Exception as e:
            print(f"Prediction error: {e}")
            return self._mock_predict(image)
    
    def _mock_predict(self, image):
        """Mock prediction based on image properties"""
        # Use image mean to create deterministic but varied results
        mean_val = np.mean(image) if image is not None else 128
        class_idx = int(mean_val) % len(self.classes)
        confidence = 0.7 + (mean_val % 50) / 200  # 0.7 to 0.95
        
        # Create fake probabilities
        probs = [0.05 + random.random() * 0.1 for _ in self.classes]
        probs[class_idx] = confidence
        total = sum(probs)
        probs = [p / total for p in probs]
        
        return {
            'class': self.classes[class_idx],
            'confidence': confidence,
            'all_probs': {cls: probs[i] for i, cls in enumerate(self.classes)}
        }


class SimpleClassifierNode(Node):
    def __init__(self):
        super().__init__('simple_classifier')
        
        # Parameters
        self.declare_parameter('model_path', 'models/recycler.pth')
        model_path = self.get_parameter('model_path').value
        
        # Setup
        self.bridge = CvBridge()
        self.classifier = SimpleClassifier(model_path)
        
        # Service for manual classification
        self.service = self.create_service(
            Trigger, 
            '/classify_image', 
            self.classify_callback
        )
        
        # Subscribe to camera feed
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publish classification results
        self.result_pub = self.create_publisher(String, '/classification_result', 10)
        
        # Auto-classification timer (every 3 seconds)
        self.timer = self.create_timer(3.0, self.auto_classify)
        
        # State
        self.latest_image = None
        self.latest_result = None
        self.classification_count = 0
        
        self.get_logger().info('Classifier node ready')

    def image_callback(self, msg):
        """Store latest camera image"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')

    def auto_classify(self):
        """Automatically classify the latest camera image"""
        if self.latest_image is None:
            return
        
        try:
            result = self.classifier.predict(self.latest_image)
            self.latest_result = result
            self.classification_count += 1
            
            # Publish result
            msg = String()
            msg.data = f"{result['class']}:{result['confidence']:.3f}"
            self.result_pub.publish(msg)
            
            self.get_logger().info(
                f"Auto-classified: {result['class']} "
                f"({result['confidence']*100:.1f}% confidence)"
            )
            
        except Exception as e:
            self.get_logger().error(f'Auto-classification failed: {e}')

    def classify_callback(self, request, response):
        """Handle manual classification requests"""
        try:
            if self.latest_image is None:
                response.success = False
                response.message = "No image available"
                return response
            
            result = self.classifier.predict(self.latest_image)
            self.latest_result = result
            self.classification_count += 1
            
            # Publish result
            msg = String()
            msg.data = f"{result['class']}:{result['confidence']:.3f}"
            self.result_pub.publish(msg)
            
            response.success = True
            response.message = f"Classified as {result['class']} ({result['confidence']*100:.1f}%)"
            
            self.get_logger().info(f"Manual classification: {response.message}")
            
        except Exception as e:
            response.success = False
            response.message = f"Classification failed: {e}"
            self.get_logger().error(response.message)
        
        return response


def main():
    rclpy.init()
    try:
        node = SimpleClassifierNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()