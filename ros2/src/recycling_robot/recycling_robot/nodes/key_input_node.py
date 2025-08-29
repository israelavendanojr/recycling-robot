#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import sys
import tty
import termios
import threading
import time

class KeyInputNode(Node):
    def __init__(self):
        super().__init__('key_input_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        
        # Publisher for camera capture
        self.capture_publisher = self.create_publisher(
            Empty, 
            '/camera/capture', 
            10
        )
        
        # Flag to control the key listening loop
        self.running = True
        
        # Start key listening in a separate thread
        self.key_thread = threading.Thread(target=self._key_listener_loop, daemon=True)
        self.key_thread.start()
        
        self.get_logger().info('[KeyInput] Ready (press "c" to capture, "q" to quit)')

    def _key_listener_loop(self):
        """Main key listening loop running in background thread"""
        try:
            # Get the file descriptor for stdin
            fd = sys.stdin.fileno()
            
            # Save the current terminal settings
            old_settings = termios.tcgetattr(fd)
            
            try:
                # Set terminal to raw mode
                tty.setraw(fd)
                
                while self.running:
                    # Read a single character
                    ch = sys.stdin.read(1)
                    
                    if ch == 'c':
                        # Publish capture command
                        msg = Empty()
                        self.capture_publisher.publish(msg)
                        self.get_logger().info('[KeyInput] c pressed → published /camera/capture')
                        
                    elif ch == 'q':
                        # Quit command
                        self.get_logger().info('[KeyInput] q pressed → exiting')
                        self.running = False
                        break
                        
                    elif ch == '\x03':  # Ctrl+C
                        # Handle Ctrl+C gracefully
                        self.get_logger().info('[KeyInput] Ctrl+C received → exiting')
                        self.running = False
                        break
                        
                    # Small delay to prevent high CPU usage
                    time.sleep(0.01)
                    
            finally:
                # Restore terminal settings
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                
        except Exception as e:
            self.get_logger().error(f'[KeyInput] Key listener error: {e}')
        finally:
            self.running = False

    def destroy_node(self):
        """Clean shutdown of the key input node"""
        try:
            self.get_logger().info('[KeyInput] Shutting down key input node...')
            self.running = False
            
            # Wait for key thread to finish (with timeout)
            if self.key_thread.is_alive():
                self.key_thread.join(timeout=1.0)
                
            super().destroy_node()
            
        except Exception as e:
            self.get_logger().error(f'[KeyInput] Error during shutdown: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = None
    ros_shutdown_called = False
    
    try:
        node = KeyInputNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            try: 
                node.destroy_node()
            except Exception: 
                pass
        try: 
            if not ros_shutdown_called:
                rclpy.shutdown()
                ros_shutdown_called = True
        except Exception as e:
            if "rcl_shutdown already called" not in str(e):
                print(f'Error during shutdown: {e}')

if __name__ == '__main__':
    main()
