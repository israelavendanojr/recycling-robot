#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from rclpy.executors import ExternalShutdownException

class PipelineCoordinatorNode(Node):
    def __init__(self):
        super().__init__('pipeline_coordinator_node')
        
        # Set logging level to INFO
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        
        # Parameters
        self.declare_parameter('timeout_seconds', 10.0)
        self.declare_parameter('state_file_path', '/tmp/pipeline_state.json')
        
        self.timeout_seconds = self.get_parameter('timeout_seconds').value
        self.state_file_path = self.get_parameter('state_file_path').value
        
        # Pipeline state
        self.state = "idle"  # "idle" or "processing"
        self.last_state_change = time.time()
        self.current_item = None
        
        # Publishers
        self.state_publisher = self.create_publisher(
            String, 
            '/pipeline/state', 
            10
        )
        
        # Subscribers
        self.classification_done_sub = self.create_subscription(
            String,
            '/pipeline/classification_done',
            self.classification_done_callback,
            10
        )
        
        self.sorting_done_sub = self.create_subscription(
            String,
            '/pipeline/sorting_done',
            self.sorting_done_callback,
            10
        )
        
        # Timer for timeout checking and state persistence
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Publish initial state
        self.publish_state()
        
        self.get_logger().info('[PipelineCoordinator] Pipeline coordinator started')
        self.get_logger().info(f'[PipelineCoordinator] Timeout: {self.timeout_seconds}s')
        self.get_logger().info('[PipelineCoordinator] Initial state: idle')

    def publish_state(self):
        """Publish current pipeline state"""
        try:
            msg = String()
            msg.data = self.state
            self.state_publisher.publish(msg)
            
            # Also persist to file for backend access
            self._persist_state()
            
            # Visual indicators for terminal
            if self.state == "idle":
                self.get_logger().info("🟢 PIPELINE: READY FOR NEXT ITEM")
            else:
                self.get_logger().info("🔴 PIPELINE: PROCESSING IN PROGRESS...")
            
            self.get_logger().debug(f'[PipelineCoordinator] Published state: {self.state}')
            
        except Exception as e:
            self.get_logger().error(f'[PipelineCoordinator] Failed to publish state: {e}')

    def _persist_state(self):
        """Persist current state to file for backend access"""
        try:
            state_data = {
                'state': self.state,
                'timestamp': time.time(),
                'last_change': self.last_state_change,
                'current_item': self.current_item
            }
            
            # Write to temporary file first, then atomic replace
            import os
            temp_path = f"{self.state_file_path}.tmp"
            with open(temp_path, 'w') as f:
                json.dump(state_data, f)
            
            os.replace(temp_path, self.state_file_path)
            
        except Exception as e:
            self.get_logger().warn(f'[PipelineCoordinator] Failed to persist state: {e}')

    def classification_done_callback(self, msg):
        """Handle classification completion"""
        try:
            if self.state != "idle":
                self.get_logger().warn('[PipelineCoordinator] Received classification_done while not idle, ignoring')
                return
            
            # Parse classification result
            classification_data = json.loads(msg.data)
            self.current_item = classification_data
            
            # Transition to processing state
            self.state = "processing"
            self.last_state_change = time.time()
            
            self.get_logger().info(f'🔄 [PipelineCoordinator] TRANSITION: idle → processing')
            self.get_logger().info(f'📦 [PipelineCoordinator] Processing item: {classification_data.get("class", "unknown")}')
            self.publish_state()
            
        except Exception as e:
            self.get_logger().error(f'[PipelineCoordinator] Classification callback error: {e}')

    def sorting_done_callback(self, msg):
        """Handle sorting completion"""
        try:
            if self.state != "processing":
                self.get_logger().warn('[PipelineCoordinator] Received sorting_done while not processing, ignoring')
                return
            
            # Transition back to idle state
            self.state = "idle"
            self.last_state_change = time.time()
            self.current_item = None
            
            self.get_logger().info('🔄 [PipelineCoordinator] TRANSITION: processing → idle')
            self.get_logger().info('✅ [PipelineCoordinator] Item processing completed successfully')
            self.publish_state()
            
        except Exception as e:
            self.get_logger().error(f'[PipelineCoordinator] Sorting callback error: {e}')

    def timer_callback(self):
        """Timer callback for timeout checking and state persistence"""
        try:
            # Check for timeout if in processing state
            if self.state == "processing":
                elapsed = time.time() - self.last_state_change
                if elapsed > self.timeout_seconds:
                    self.get_logger().warn(f'[PipelineCoordinator] Processing timeout after {elapsed:.1f}s, resetting to idle')
                    self.state = "idle"
                    self.last_state_change = time.time()
                    self.current_item = None
                    self.publish_state()
            
            # Periodically persist state
            if time.time() % 5 < 1:  # Every ~5 seconds
                self._persist_state()
                
        except Exception as e:
            self.get_logger().error(f'[PipelineCoordinator] Timer callback error: {e}')

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('[PipelineCoordinator] Pipeline coordinator shutting down')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    ros_shutdown_called = False
    
    try:
        node = PipelineCoordinatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    except Exception as e:
        print(f'Unexpected error: {e}')
    finally:
        if node:
            try:
                node.destroy_node()
            except Exception as e:
                print(f'Error during node destruction: {e}')
        try:
            if not ros_shutdown_called:
                rclpy.shutdown()
                ros_shutdown_called = True
        except Exception as e:
            if "rcl_shutdown already called" not in str(e):
                print(f'Error during shutdown: {e}')

if __name__ == '__main__':
    main()
