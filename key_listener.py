#!/usr/bin/env python3
"""
Simple host-side key listener for recycling robot
Uses standard input() instead of raw terminal mode for better reliability
"""

import subprocess
import signal
import sys

class SimpleKeyListener:
    def __init__(self):
        self.running = True
        signal.signal(signal.SIGINT, self._signal_handler)
        
    def _signal_handler(self, signum, frame):
        print("\n[KeyListener] Shutting down...")
        self.running = False
        # Don't try to cleanup here - let the cleanup method handle it
        
    def send_capture_command(self):
        """Send capture command to ROS2 system in Docker"""
        try:
            cmd = [
                "docker", "compose", "exec", "-T", "ros2", "bash", "-c",
                "cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 topic pub /camera/capture std_msgs/msg/Empty --once"
            ]
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            print("[KeyListener] c pressed → published /camera/capture")
        except subprocess.CalledProcessError as e:
            print(f"[KeyListener] Error sending capture command: {e}")
            if e.stderr:
                print(f"[KeyListener] Error details: {e.stderr}")
        except Exception as e:
            print(f"[KeyListener] Unexpected error: {e}")
    
    def run(self):
        """Main key listening loop using standard input()"""
        print("[KeyListener] Ready (press 'c' to capture, 'q' to quit)")
        print("Enter command: ", end='', flush=True)
        
        while self.running:
            try:
                user_input = input().strip().lower()
                
                if user_input == 'c':
                    self.send_capture_command()
                elif user_input == 'q':
                    print("[KeyListener] q pressed → exiting")
                    self.running = False
                    break
                else:
                    print("Invalid input. Enter 'c' to capture, 'q' to quit.")
                
                if self.running:
                    print("Enter command: ", end='', flush=True)
                    
            except KeyboardInterrupt:
                print("\n[KeyListener] Ctrl+C received → exiting")
                self.running = False
                break
            except EOFError:
                print("\n[KeyListener] EOF received → exiting")
                self.running = False
                break

    def cleanup(self):
        """Clean up ROS2 processes on exit"""
        try:
            import subprocess
            print("[KeyListener] Cleaning up ROS2 processes...")
            # Stop ROS2 processes
            subprocess.run(["docker", "compose", "exec", "ros2", "bash", "-c", "pkill -f ros2"], 
                         capture_output=True, timeout=5)
            # Give it a moment to stop
            import time
            time.sleep(1)
            print("[KeyListener] Cleanup complete")
        except Exception as e:
            print(f"[KeyListener] Cleanup error: {e}")

def main():
    listener = SimpleKeyListener()
    try:
        listener.run()
    except KeyboardInterrupt:
        print("\n[KeyListener] KeyboardInterrupt received")
    except Exception as e:
        print(f"\n[KeyListener] Unexpected error: {e}")
    finally:
        listener.cleanup()
        print("[KeyListener] Exiting")

if __name__ == '__main__':
    main()
