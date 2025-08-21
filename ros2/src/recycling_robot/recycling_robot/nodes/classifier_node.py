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

class ClassifierNode(Node):
    def __init__(self):
        super().__init__('classifier_node')
        
        # Parameters
        self.declare_parameter('api_base_url', 'http://api:5000')
        self.declare_parameter('inference_interval', 3.0)
        self.declare_parameter('confidence_threshold', 0.7)
        
        self.api_base = os.getenv("API_BASE_URL", self.get_parameter("api_base_url").value)
        self.interval = self.get_parameter('inference_interval').value
        self.threshold = self.get_parameter('confidence_threshold').value
        
        # Setup
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        
        # State
        self.latest_image = None
        self.classes = ['cardboard', 'glass', 'metal', 'plastic', 'trash']
        
        # Auto-classify timer
        self.timer = self.create_timer(self.interval, self.auto_classify)
        
        self.get_logger().info(f'Classifier node started, API: {self.api_base}')

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')

    def auto_classify(self):
        if self.latest_image is None:
            return
        
        try:
            # Mock classification based on image properties
            mean_val = np.mean(self.latest_image)
            class_idx = int(mean_val) % len(self.classes)
            confidence = 0.7 + (mean_val % 50) / 200
            
            result = {
                'class': self.classes[class_idx],
                'confidence': float(confidence),
                'timestamp': time.time()
            }
            
            # POST to API
            response = requests.post(
                f'{self.api_base}/api/events',
                json=result,
                timeout=2.0
            )
            
            if response.status_code == 200:
                self.get_logger().info(
                    f"Classification: {result['class']} ({result['confidence']*100:.1f}%)"
                )
            else:
                self.get_logger().warn(f'API POST failed: {response.status_code}')
                
        except Exception as e:
            self.get_logger().error(f'Classification failed: {e}')

def main():
    rclpy.init()
    try:
        node = ClassifierNode()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()