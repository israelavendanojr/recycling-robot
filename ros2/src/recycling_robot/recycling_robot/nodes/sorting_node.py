#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from recycling_robot.utils.motor_controller import MotorController

class SortingNode(Node):
    def __init__(self):
        super().__init__('sorting_node')
        self.motor = MotorController()
        
        # Set logging level to INFO to reduce debug spam
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        
        # Parameters
        self.declare_parameter('sorting_delay', 1.0)  # Delay before sorting action
        
        # Get parameters
        self.sorting_delay = self.get_parameter('sorting_delay').value
        
        # Subscribe to pipeline classification completion
        self.classification_subscription = self.create_subscription(
            String,
            '/pipeline/classification_done',
            self.classification_callback,
            10
        )
        
        # Pipeline completion publisher
        self.pipeline_completion_pub = self.create_publisher(
            String,
            '/pipeline/sorting_done',
            10
        )
        
        # State
        self.latest_classification = None
        self.sorting_busy = False
        self.last_processed_classification_hash = None  # Track processed classifications
        
        self.get_logger().info('[SortingNode] Sorting node started')
        self.get_logger().info(f'[SortingNode] Processing delay: {self.sorting_delay}s')
        self.get_logger().info('[SortingNode] Subscribed to /pipeline/classification_done')
        self.get_logger().info('[SortingNode] Will publish to /pipeline/sorting_done')

    def _get_classification_hash(self, classification):
        """Generate a simple hash for the classification to detect duplicates"""
        try:
            # Use timestamp and class as a simple hash
            timestamp = classification.get('timestamp', time.time())
            material_class = classification.get('class', 'unknown')
            return f"{timestamp}_{material_class}"
        except Exception:
            # Fallback to class only
            return classification.get('class', 'unknown')

    def classification_callback(self, msg):
        """Handle incoming classification completion messages"""
        try:
            # Parse classification result
            classification = json.loads(msg.data)
            
            # Check if this is a duplicate classification
            classification_hash = self._get_classification_hash(classification)
            if classification_hash == self.last_processed_classification_hash:
                self.get_logger().debug('[SortingNode] Duplicate classification detected, skipping')
                return
            
            # Store the new classification
            self.latest_classification = classification
            self.last_processed_classification_hash = classification_hash
            
            self.get_logger().info(f'[SortingNode] New classification received: {classification["class"]} ({classification["confidence"]*100:.1f}% confidence)')
            
            # Trigger sorting action
            self._perform_sorting(classification)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'[SortingNode] Failed to parse classification message: {e}')
        except Exception as e:
            self.get_logger().error(f'[SortingNode] Error processing classification: {e}')

    def _perform_sorting(self, classification):
        """Perform the sorting action based on classification"""
        if self.sorting_busy:
            self.get_logger().warn('[SortingNode] Already in progress, skipping')
            return
        
        self.sorting_busy = True
        
        try:
            material_class = classification['class']
            confidence = classification['confidence']
            
            # Determine sorting action
            sorting_action = self._get_sorting_action(material_class)
            
            if sorting_action:
                self.get_logger().info(f'[SortingNode] Processing: {material_class.upper()}')
                
                # Simulate sorting delay
                time.sleep(self.sorting_delay)
                
                # Control the motor to move to the target bin
                self._control_actuator(sorting_action)
                
                # Log completion
                self.get_logger().info('[Sorter] Sorting Complete')
                
                # Notify pipeline coordinator of completion
                completion_msg = String()
                completion_msg.data = json.dumps({
                    'status': 'complete',
                    'material': material_class,
                    'action': sorting_action,
                    'timestamp': time.time()
                })
                self.pipeline_completion_pub.publish(completion_msg)
                
            else:
                self.get_logger().warn(f'[SortingNode] Unknown material class: {material_class}')
                
        except Exception as e:
            self.get_logger().error(f'[SortingNode] Processing failed: {e}')
        finally:
            self.sorting_busy = False

    def _get_sorting_action(self, material_class):
        """Map material class to sorting action"""
        sorting_map = {
            'cardboard': 'BIN_1',
            'glass': 'BIN_2', 
            'metal': 'BIN_3',
            'plastic': 'BIN_4',
            'trash': 'BIN_5'
        }
        return sorting_map.get(material_class.lower())

    def _describe_action(self, action):
        """Describe the sorting action in human-readable terms"""
        action_descriptions = {
            'BIN_1': 'Move to Blue Bin (Cardboard)',
            'BIN_2': 'Move to Green Bin (Glass)',
            'BIN_3': 'Move to Yellow Bin (Metal)',
            'BIN_4': 'Move to Red Bin (Plastic)',
            'BIN_5': 'Move to Black Bin (Trash)'
        }
        return action_descriptions.get(action, 'Unknown action')

    def _control_actuator(self, action):
        """Control motor for 1 second on any classification"""
        self.get_logger().info(f'[Motor] Starting motor for classification: {action}')
        try:
            self.get_logger().info('[Motor] Motor forward (speed=0.80) for 1 second')
            self.motor.forward(0.8)
            time.sleep(1.0)  # Spin for exactly 1 second
            self.motor.stop()
            self.get_logger().info('[Motor] Motor stop - cycle completed')
        except Exception as e:
            self.get_logger().error(f'[Motor] Motor failed: {e}')
def main(args=None):
    rclpy.init(args=args)
    node = SortingNode()
    
    # Track ROS shutdown state to prevent duplicate calls
    ros_shutdown_called = False

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error in sorting node: {e}')
    finally:
        # Always cleanup motor + node
        try:
            if hasattr(node, 'motor'):
                node.motor.cleanup()
        except Exception as e:
            print(f'Error during motor cleanup: {e}')

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
                print(f'Error during ROS shutdown: {e}')


if __name__ == '__main__':
    main()
