#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import requests
import json
import os
import signal
import threading
from rclpy.executors import ExternalShutdownException

class ClassifierNode(Node):
    def __init__(self):
        super().__init__('classifier_node')
        
        # Parameters
        self.declare_parameter('api_base_url', 'http://backend:8000')
        self.declare_parameter('inference_interval', 3.0)
        self.declare_parameter('confidence_threshold', 0.7)
        
        # Get API URL from environment or parameter
        self.api_base = os.getenv("API_BASE_URL", self.get_parameter("api_base_url").value)
        self.interval = self.get_parameter('inference_interval').value
        self.threshold = self.get_parameter('confidence_threshold').value
        
        # Setup
        self.bridge = CvBridge()
        
        # State
        self.latest_image = None
        self.classes = ['cardboard', 'glass', 'metal', 'plastic', 'trash']
        self._running = True
        self._backend_ready = False
        self._shutdown_event = threading.Event()
        
        # Setup graceful shutdown - remove the signal handler as ROS2 handles it
        # signal.signal(signal.SIGINT, self._signal_handler)  # Remove this line
        
        self.get_logger().info(f'Classifier node started, API: {self.api_base}')
        
        # Wait for backend before starting subscriptions and timers
        self._wait_for_backend()
        
        # Only create subscriptions and timer after backend is ready
        if self._running:  # Check if we haven't been shutdown during backend wait
            self.subscription = self.create_subscription(
                Image, '/camera/image_raw', self.image_callback, 10
            )
            
            # Auto-classify timer
            self.timer = self.create_timer(self.interval, self.auto_classify)
            self.get_logger().info('Backend ready, starting classification service')
        else:
            self.get_logger().error('Failed to start classification service - backend unavailable')

    def _wait_for_backend(self):
        """Wait for backend to be ready with exponential backoff"""
        max_wait_time = 60.0  # Maximum total wait time
        retry_delay = 1.0     # Initial retry delay
        max_retry_delay = 8.0 # Maximum retry delay
        start_time = time.time()
        
        logged_waiting = False
        self.get_logger().info('Waiting for backend to be ready...')
        
        while self._running and not self._shutdown_event.is_set():
            try:
                response = requests.get(f'{self.api_base}/api/health', timeout=5.0)
                if response.status_code == 200:
                    self._backend_ready = True
                    self.get_logger().info('✅ Backend is ready!')
                    return
            except Exception as e:
                if not logged_waiting:
                    self.get_logger().info('Backend not ready, waiting...')
                    logged_waiting = True
            
            # Check timeout
            elapsed = time.time() - start_time
            if elapsed > max_wait_time:
                self.get_logger().error(f'Backend not ready after {max_wait_time}s, giving up')
                self._running = False
                return
            
            # Wait with exponential backoff
            if not logged_waiting:
                self.get_logger().info(f'Retrying in {retry_delay:.1f}s...')
            if self._shutdown_event.wait(retry_delay):  # Interruptible wait
                return  # Shutdown requested
            
            retry_delay = min(retry_delay * 1.5, max_retry_delay)

    def image_callback(self, msg):
        """Store latest image for classification"""
        if not self._running:
            return
            
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        except Exception as e:
            if self._running:  # Only log if we're not shutting down
                self.get_logger().error(f'Image conversion error: {e}')

    def auto_classify(self):
        """Perform automatic classification"""
        if not self._running or self.latest_image is None:
            return
        
        try:
            # Check backend health with less frequent logging
            if not self._check_backend_health():
                return
                
            # Mock classification based on image properties
            mean_val = np.mean(self.latest_image)
            class_idx = int(mean_val) % len(self.classes)
            confidence = 0.7 + (mean_val % 50) / 200
            
            result = {
                'class': self.classes[class_idx],
                'confidence': float(confidence),
                'timestamp': time.time()
            }
            
            # POST to API with retry logic
            self._send_classification(result)
                
        except Exception as e:
            if self._running:  # Only log if we're not shutting down
                self.get_logger().error(f'Classification failed: {e}')

    def _check_backend_health(self):
        """Check if backend is available - less verbose after initial connection"""
        try:
            response = requests.get(f'{self.api_base}/api/health', timeout=2.0)
            if response.status_code == 200:
                if not self._backend_ready:
                    self._backend_ready = True
                    self.get_logger().info('Backend reconnected')
                return True
            else:
                if self._backend_ready:
                    self._backend_ready = False
                    self.get_logger().warn('Backend unhealthy, will retry')
                return False
        except Exception:
            if self._backend_ready:
                self._backend_ready = False
                self.get_logger().warn('Backend connection lost, will retry')
            return False

    def _send_classification(self, result):
        """Send classification result to backend with retry"""
        max_retries = 3
        
        for attempt in range(max_retries):
            try:
                response = requests.post(
                    f'{self.api_base}/api/events',
                    json=result,
                    timeout=5.0
                )
                
                if response.status_code == 200:
                    self.get_logger().info(
                        f"Classification: {result['class']} ({result['confidence']*100:.1f}%)"
                    )
                    return
                else:
                    self.get_logger().warn(f'API POST failed: {response.status_code}')
                    
            except requests.exceptions.ConnectionError as e:
                if attempt < max_retries - 1:
                    self.get_logger().debug(f'Connection failed (attempt {attempt+1}/{max_retries}), retrying...')
                    time.sleep(1.0)
                else:
                    self.get_logger().error(f'All connection attempts failed: {e}')
                    self._backend_ready = False  # Mark backend as not ready
            except Exception as e:
                self.get_logger().error(f'API request failed: {e}')
                break

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down classifier node...')
        self._running = False
        self._shutdown_event.set()
        
        # Cancel timer if it exists
        if hasattr(self, 'timer'):
            self.timer.cancel()
        
        super().destroy_node()

def main():
    rclpy.init()
    node = None
    
    try:
        node = ClassifierNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully
    except ExternalShutdownException:
        pass  # Handle ROS2 shutdown gracefully
    finally:
        if node:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass  # Already shutdown

if __name__ == '__main__':
    main()