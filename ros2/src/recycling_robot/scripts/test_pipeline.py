#!/usr/bin/env python3
"""
Simple test script to verify pipeline coordination.
This simulates the message flow between nodes.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class PipelineTester(Node):
    def __init__(self):
        super().__init__('pipeline_tester')
        
        # Subscribe to pipeline state
        self.state_sub = self.create_subscription(
            String,
            '/pipeline/state',
            self.state_callback,
            10
        )
        
        # Publishers to simulate node messages
        self.classification_pub = self.create_publisher(
            String,
            '/pipeline/classification_done',
            10
        )
        
        self.sorting_pub = self.create_publisher(
            String,
            '/pipeline/sorting_done',
            10
        )
        
        self.current_state = "unknown"
        self.test_count = 0
        
        # Timer to run tests
        self.timer = self.create_timer(5.0, self.run_test)
        
        self.get_logger().info('[Tester] Pipeline tester started')
        self.get_logger().info('[Tester] Will run tests every 5 seconds')

    def state_callback(self, msg):
        """Handle pipeline state updates"""
        old_state = self.current_state
        self.current_state = msg.data
        self.get_logger().info(f'[Tester] Pipeline state changed: {old_state} -> {self.current_state}')

    def run_test(self):
        """Run a test cycle"""
        self.test_count += 1
        self.get_logger().info(f'[Tester] Running test #{self.test_count}')
        
        if self.current_state == "idle":
            # Simulate classification completion
            self.get_logger().info('[Tester] Simulating classification completion')
            classification_data = {
                'class': 'plastic',
                'confidence': 0.85,
                'timestamp': time.time()
            }
            msg = String()
            msg.data = json.dumps(classification_data)
            self.classification_pub.publish(msg)
            
        elif self.current_state == "processing":
            # Simulate sorting completion
            self.get_logger().info('[Tester] Simulating sorting completion')
            msg = String()
            msg.data = "complete"
            self.sorting_pub.publish(msg)

def main():
    rclpy.init()
    tester = PipelineTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
