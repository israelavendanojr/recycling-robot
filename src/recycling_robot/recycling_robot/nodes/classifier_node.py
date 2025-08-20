"""
ROS2 Classifier Node - Provides image classification service.
Migrated from legacy/core/classifier.py with ROS 2 service interface.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
from typing import Optional
import time
import os
import sys

# Import custom messages - fallback for development
try:
    from recycling_robot_msgs.msg import ClassificationResult as ClassificationResultMsg
    from recycling_robot_msgs.srv import ClassifyImage
    CUSTOM_MSGS_AVAILABLE = True
except ImportError:
    from std_msgs.msg import String as ClassificationResultMsg
    from std_srvs.srv import Trigger as ClassifyImage
    CUSTOM_MSGS_AVAILABLE = False

# Import our classifier - fix the circular import by using proper path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'utils'))
from classifier import RecyclingClassifier

class ClassifierNode(Node):
    """
    ROS2 node that provides image classification as a service.
    Uses the proven classifier implementation from legacy code.
    """
    
    def __init__(self):
        super().__init__('classifier_node')
        
        # Declare parameters
        self.declare_parameter('model_path', 'models/recycler.pth')
        self.declare_parameter('device', 'auto')  # 'cpu', 'cuda', or 'auto'
        self.declare_parameter('input_size', 224)
        self.declare_parameter('class_names', ['cardboard', 'glass', 'metal', 'plastic', 'trash'])
        
        # Get parameters
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        device = self.get_parameter('device').get_parameter_value().string_value
        input_size = self.get_parameter('input_size').get_parameter_value().integer_value
        class_names = self.get_parameter('class_names').get_parameter_value().string_array_value
        
        # Convert device parameter
        if device == 'auto':
            device = None  # Let classifier auto-detect
        
        # Initialize components
        self.bridge = CvBridge()
        self.classifier: Optional[RecyclingClassifier] = None
        
        # Statistics
        self.classification_count = 0
        self.total_inference_time = 0.0
        self.start_time = time.time()
        
        # Initialize classifier
        self.get_logger().info(f'Loading model from: {model_path}')
        try:
            # Check if model file exists
            if not os.path.exists(model_path):
                self.get_logger().error(f'Model file not found: {model_path}')
                self.get_logger().info('Available files in models directory:')
                models_dir = os.path.dirname(model_path)
                if os.path.exists(models_dir):
                    for file in os.listdir(models_dir):
                        self.get_logger().info(f'  - {file}')
                else:
                    self.get_logger().info('Models directory does not exist')
                # Continue with mock mode for development
                self.get_logger().warn('Running in MOCK classification mode')
                self.classifier = None
            else:
                self.classifier = RecyclingClassifier(
                    model_path=model_path,
                    class_names=list(class_names) if class_names else None,
                    device=device,
                    input_size=input_size
                )
                self.get_logger().info('✓ Classifier initialized successfully')
                self.get_logger().info(f'Classes: {self.classifier.class_names}')
                self.get_logger().info(f'Device: {self.classifier.device}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize classifier: {e}')
            self.get_logger().warn('Running in MOCK classification mode')
            self.classifier = None
        
        # Create service server
        if CUSTOM_MSGS_AVAILABLE:
            self.service = self.create_service(
                ClassifyImage,
                '/classify_image',
                self.classify_image_callback
            )
        else:
            # Fallback service for development
            self.service = self.create_service(
                ClassifyImage,
                '/classify_image',
                self.classify_image_callback_fallback
            )
        
        # Publisher for classification results (for monitoring/stats)
        self.result_publisher = self.create_publisher(
            ClassificationResultMsg,
            '/classification_results',
            10
        )
        
        self.get_logger().info('✓ Classifier service ready at /classify_image')
        if not CUSTOM_MSGS_AVAILABLE:
            self.get_logger().warn('Using fallback message types - build custom messages for full functionality')
    
    def _mock_classify(self, frame: np.ndarray):
        """Mock classification for development when model isn't available."""
        import random
        classes = ['cardboard', 'glass', 'metal', 'plastic', 'trash']
        predicted_class = random.choice(classes)
        confidence = random.uniform(0.7, 0.95)
        
        # Simulate our ClassificationResult structure
        class MockResult:
            def __init__(self):
                self.predicted_class = predicted_class
                self.class_index = classes.index(predicted_class)
                self.confidence = confidence
                self.all_probabilities = {
                    cls: random.uniform(0.1, 0.9) if cls == predicted_class else random.uniform(0.01, 0.2)
                    for cls in classes
                }
                self.timestamp = time.time()
        
        return MockResult()
    
    def classify_image_callback_fallback(self, request, response):
        """Fallback callback for development without custom messages."""
        try:
            self.get_logger().info('Received classification request (fallback mode)')
            
            # Mock classification
            result = self._mock_classify(None)
            
            # Update statistics
            self.classification_count += 1
            classification_time = 50.0  # Mock timing
            self.total_inference_time += classification_time
            
            # Log results
            self.get_logger().info(
                f'Mock classified as: {result.predicted_class} '
                f'({result.confidence*100:.1f}% confidence) '
                f'in {classification_time:.1f}ms'
            )
            
            # Fill response (basic trigger response)
            response.success = True
            response.message = f'Mock classification: {result.predicted_class} ({result.confidence*100:.1f}%)'
            
            # Publish result for web dashboard
            msg = ClassificationResultMsg()
            msg.data = f"{result.predicted_class}:{result.confidence:.3f}"
            self.result_publisher.publish(msg)
            
            # Log statistics periodically
            if self.classification_count % 5 == 0:
                avg_time = self.total_inference_time / self.classification_count
                uptime = time.time() - self.start_time
                self.get_logger().info(
                    f'Stats: {self.classification_count} classifications, '
                    f'avg {avg_time:.1f}ms, uptime {uptime:.1f}s'
                )
                
        except Exception as e:
            self.get_logger().error(f'Classification failed: {e}')
            response.success = False
            response.message = f'Classification error: {str(e)}'
        
        return response
    
    def classify_image_callback(self, request, response):
        """
        Handle classification service requests with real classifier.
        
        Args:
            request: ClassifyImage.Request with image
            response: ClassifyImage.Response to fill
            
        Returns:
            Filled response
        """
        try:
            self.get_logger().debug('Received classification request')
            
            if self.classifier is None:
                # Use mock classification
                result = self._mock_classify(None)
                classification_time = 50.0
            else:
                # Convert ROS Image to numpy array
                frame = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='rgb8')
                
                # Perform real classification
                start_time = time.time()
                result = self.classifier.predict(frame)  # Use predict() method
                classification_time = (time.time() - start_time) * 1000
            
            # Update statistics
            self.classification_count += 1
            self.total_inference_time += classification_time
            
            # Log results
            self.get_logger().info(
                f'Classified as: {result.predicted_class} '
                f'({result.confidence*100:.1f}% confidence) '
                f'in {classification_time:.1f}ms'
            )
            
            # Fill response
            response.success = True
            response.message = f'Successfully classified as {result.predicted_class}'
            
            # Convert our result to ROS message
            response.result = self._create_classification_message(result, request.image.header)
            
            # Publish result for monitoring
            self.result_publisher.publish(response.result)
            
            # Log statistics periodically
            if self.classification_count % 10 == 0:
                avg_time = self.total_inference_time / self.classification_count
                uptime = time.time() - self.start_time
                self.get_logger().info(
                    f'Stats: {self.classification_count} classifications, '
                    f'avg {avg_time:.1f}ms, uptime {uptime:.1f}s'
                )
                
        except Exception as e:
            self.get_logger().error(f'Classification failed: {e}')
            response.success = False
            response.message = f'Classification error: {str(e)}'
            # Create empty result on error
            response.result = self._create_empty_classification_message()
        
        return response
    
    def _create_classification_message(self, result, original_header) -> ClassificationResultMsg:
        """
        Convert our ClassificationResult to ROS message.
        
        Args:
            result: Our ClassificationResult object
            original_header: Header from original image
            
        Returns:
            ROS ClassificationResult message
        """
        if not CUSTOM_MSGS_AVAILABLE:
            # Return simple string for development
            msg = ClassificationResultMsg()
            msg.data = f"{result.predicted_class}:{result.confidence:.3f}"
            return msg
        
        msg = ClassificationResultMsg()
        
        # Set header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = original_header.frame_id
        
        # Set classification data
        msg.predicted_class = result.predicted_class
        msg.class_index = result.class_index
        msg.confidence = result.confidence
        
        # Set probability arrays
        msg.class_names = list(result.all_probabilities.keys())
        msg.probabilities = list(result.all_probabilities.values())
        
        # Set metadata
        msg.inference_time_ms = (time.time() - result.timestamp) * 1000
        msg.device_used = self.classifier.device if self.classifier else 'mock'
        
        return msg
    
    def _create_empty_classification_message(self) -> ClassificationResultMsg:
        """Create empty classification message for error cases."""
        if not CUSTOM_MSGS_AVAILABLE:
            msg = ClassificationResultMsg()
            msg.data = "error:0.0"
            return msg
        
        msg = ClassificationResultMsg()
        
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        
        msg.predicted_class = 'error'
        msg.class_index = -1
        msg.confidence = 0.0
        msg.class_names = self.classifier.class_names if self.classifier else ['error']
        msg.probabilities = [0.0] * len(msg.class_names)
        msg.inference_time_ms = 0.0
        msg.device_used = 'none'
        
        return msg
    
    def destroy_node(self) -> None:
        """Clean shutdown of classifier node."""
        self.get_logger().info('Shutting down classifier node...')
        
        # Print final statistics
        if self.classification_count > 0:
            avg_time = self.total_inference_time / self.classification_count
            uptime = time.time() - self.start_time
            self.get_logger().info(
                f'Final stats: {self.classification_count} classifications, '
                f'avg {avg_time:.1f}ms per inference, uptime {uptime:.1f}s'
            )
        
        super().destroy_node()

def main(args=None):
    """Main entry point for classifier node."""
    rclpy.init(args=args)
    
    try:
        node = ClassifierNode()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Classifier node error: {e}')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()