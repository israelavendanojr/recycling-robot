#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class SortingNode(Node):
    def __init__(self):
        super().__init__('sorting_node')
        
        # Set logging level to INFO to reduce debug spam
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        
        # Parameters
        self.declare_parameter('sorting_delay', 1.0)  # Delay before sorting action
        
        # Get parameters
        self.sorting_delay = self.get_parameter('sorting_delay').value
        
        # Subscribe to classifier results
        self.classification_subscription = self.create_subscription(
            String,
            'classifier/result',
            self.classification_callback,
            10
        )
        
        # State
        self.latest_classification = None
        self.sorting_busy = False
        
        self.get_logger().info('🚀 Sorting node started')
        self.get_logger().info(f'⏱️  Sorting delay: {self.sorting_delay}s')
        
        # Log available sorting actions
        self.get_logger().info('📋 Available sorting actions:')
        self.get_logger().info('  🔵 cardboard → Bin 1 (Blue)')
        self.get_logger().info('  🟢 glass → Bin 2 (Green)')
        self.get_logger().info('  🟡 metal → Bin 3 (Yellow)')
        self.get_logger().info('  🔴 plastic → Bin 4 (Red)')
        self.get_logger().info('  ⚫ trash → Bin 5 (Black)')

    def classification_callback(self, msg):
        """Handle incoming classification results"""
        try:
            # Parse classification result
            classification = json.loads(msg.data)
            self.latest_classification = classification
            
            self.get_logger().info(f'📥 Received: {classification["class"]} ({classification["confidence"]*100:.1f}% confidence)')
            
            # Trigger sorting action
            self._perform_sorting(classification)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'❌ Failed to parse classification message: {e}')
        except Exception as e:
            self.get_logger().error(f'❌ Error processing classification: {e}')

    def _perform_sorting(self, classification):
        """Perform the sorting action based on classification"""
        if self.sorting_busy:
            self.get_logger().warn('⚠️  Sorting already in progress, skipping')
            return
        
        self.sorting_busy = True
        
        try:
            material_class = classification['class']
            confidence = classification['confidence']
            
            # Determine sorting action
            sorting_action = self._get_sorting_action(material_class)
            
            if sorting_action:
                self.get_logger().info(f'🎯 SORTING: {material_class.upper()} → {sorting_action}')
                self.get_logger().info(f'   📊 Confidence: {confidence*100:.1f}%')
                self.get_logger().info(f'   🔧 Action: {self._describe_action(sorting_action)}')
                
                # Simulate sorting delay
                time.sleep(self.sorting_delay)
                
                # Log completion
                self.get_logger().info(f'✅ Sorting completed: {material_class} → {sorting_action}')
                
                # TODO: Add actual actuator control here
                # self._control_actuator(sorting_action)
                
            else:
                self.get_logger().warn(f'⚠️  Unknown material class: {material_class}')
                
        except Exception as e:
            self.get_logger().error(f'❌ Sorting action failed: {e}')
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
        """Control the physical sorting actuator (placeholder for future implementation)"""
        # TODO: Implement actual actuator control
        # This will be implemented when you add motors/actuators
        self.get_logger().info(f'🔧 Actuator control: {action}')
        pass

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SortingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in sorting node: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
