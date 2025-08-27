#!/usr/bin/env python3
"""
Comprehensive Pipeline Test Script
Tests the synchronous pipeline implementation by simulating the complete message flow.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from datetime import datetime

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
        self.state_history = []
        
        # Timer to run tests
        self.timer = self.create_timer(8.0, self.run_test)
        
        self.get_logger().info('🧪 [Tester] Comprehensive Pipeline Tester Started')
        self.get_logger().info('⏱️ [Tester] Will run tests every 8 seconds')
        self.get_logger().info('📊 [Tester] Monitoring pipeline state transitions')

    def state_callback(self, msg):
        """Handle pipeline state updates"""
        old_state = self.current_state
        self.current_state = msg.data
        timestamp = datetime.now().strftime('%H:%M:%S')
        
        # Record state change
        self.state_history.append({
            'timestamp': timestamp,
            'old_state': old_state,
            'new_state': self.current_state
        })
        
        self.get_logger().info(f'🔄 [Tester] {timestamp} - Pipeline state: {old_state} → {self.current_state}')
        
        # Keep only last 10 state changes
        if len(self.state_history) > 10:
            self.state_history.pop(0)

    def run_test(self):
        """Run a test cycle"""
        self.test_count += 1
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.get_logger().info(f'🧪 [Tester] {timestamp} - Running test #{self.test_count}')
        
        if self.current_state == "idle":
            # Simulate classification completion
            self.get_logger().info('📤 [Tester] Simulating classification completion (plastic item)')
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
            self.get_logger().info('📤 [Tester] Simulating sorting completion')
            msg = String()
            msg.data = "complete"
            self.sorting_pub.publish(msg)
            
        # Print state history every few tests
        if self.test_count % 3 == 0:
            self.print_state_summary()

    def print_state_summary(self):
        """Print a summary of recent state changes"""
        self.get_logger().info('📊 [Tester] Recent State Changes:')
        for change in self.state_history[-5:]:  # Last 5 changes
            self.get_logger().info(f'   {change["timestamp"]}: {change["old_state"]} → {change["new_state"]}')
        
        # Calculate state distribution
        idle_count = sum(1 for change in self.state_history if change['new_state'] == 'idle')
        processing_count = sum(1 for change in self.state_history if change['new_state'] == 'processing')
        
        self.get_logger().info(f'📈 [Tester] State Distribution: idle={idle_count}, processing={processing_count}')

def main():
    rclpy.init()
    tester = PipelineTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        print("\n🧪 [Tester] Test interrupted by user")
        print("📊 [Tester] Final State Summary:")
        for change in tester.state_history[-10:]:
            print(f"   {change['timestamp']}: {change['old_state']} → {change['new_state']}")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
