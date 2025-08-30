#!/usr/bin/env python3
"""
Simple HTTP server to trigger ROS2 capture from external containers
This runs alongside the ROS2 nodes and provides an HTTP endpoint for triggering capture
"""

import subprocess
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
import json

class CaptureHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path == '/trigger_capture':
            try:
                print("[CaptureServer] Received capture trigger request")
                
                # Trigger the ROS2 capture command
                cmd = [
                    "bash", "-c",
                    "cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 topic pub /camera/capture std_msgs/msg/Empty --once"
                ]
                
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
                
                if result.returncode == 0:
                    print("[CaptureServer] Capture triggered successfully")
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    response = {"status": "ok", "message": "Capture triggered"}
                    self.wfile.write(json.dumps(response).encode())
                else:
                    print(f"[CaptureServer] Capture command failed: {result.stderr}")
                    self.send_response(500)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    response = {"status": "error", "message": f"Command failed: {result.stderr}"}
                    self.wfile.write(json.dumps(response).encode())
                    
            except Exception as e:
                print(f"[CaptureServer] Error handling capture request: {e}")
                self.send_response(500)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                response = {"status": "error", "message": str(e)}
                self.wfile.write(json.dumps(response).encode())
                
        elif self.path == '/shutdown':
            try:
                print("[CaptureServer] Received shutdown request")
                
                # Stop ROS2 processes gracefully (same as CLI cleanup)
                print("[CaptureServer] Stopping launch process...")
                
                # Kill the launch process and all child processes
                cmd1 = ["pkill", "-f", "robot.launch.py"]
                result1 = subprocess.run(cmd1, capture_output=True, text=True, timeout=3)
                
                # Kill any remaining ros2 processes
                cmd2 = ["pkill", "-f", "ros2"]
                result2 = subprocess.run(cmd2, capture_output=True, text=True, timeout=3)
                
                # More aggressive cleanup - kill by process group
                cmd3 = ["pkill", "-9", "-f", "python3.*ros2"]
                result3 = subprocess.run(cmd3, capture_output=True, text=True, timeout=2)
                
                # Kill any remaining python processes related to launch
                cmd4 = ["pkill", "-f", "launch"]
                result4 = subprocess.run(cmd4, capture_output=True, text=True, timeout=2)
                
                # Use launch kill result as primary for response
                result = result1
                
                print(f"[CaptureServer] Launch kill: {result1.returncode}, ROS2 kill: {result2.returncode}")
                print(f"[CaptureServer] Python kill: {result3.returncode}, Launch kill: {result4.returncode}")
                
                print(f"[CaptureServer] Shutdown command result: {result.returncode}")
                if result.stdout:
                    print(f"[CaptureServer] Stdout: {result.stdout}")
                if result.stderr:
                    print(f"[CaptureServer] Stderr: {result.stderr}")
                
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                response = {"status": "ok", "message": "Shutdown triggered"}
                self.wfile.write(json.dumps(response).encode())
                
                # Schedule server shutdown after responding
                def shutdown_server():
                    time.sleep(1)  # Give time for response to be sent
                    print("[CaptureServer] Shutting down HTTP server...")
                    import os
                    os._exit(0)
                
                threading.Thread(target=shutdown_server, daemon=True).start()
                    
            except Exception as e:
                print(f"[CaptureServer] Error handling shutdown request: {e}")
                self.send_response(500)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                response = {"status": "error", "message": str(e)}
                self.wfile.write(json.dumps(response).encode())
        else:
            self.send_response(404)
            self.end_headers()
    
    def do_GET(self):
        if self.path == '/health':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            response = {"status": "ok", "service": "capture_server"}
            self.wfile.write(json.dumps(response).encode())
        else:
            self.send_response(404)
            self.end_headers()
    
    def log_message(self, format, *args):
        # Suppress default HTTP server logs to reduce noise
        pass

def run_server():
    """Run the HTTP server in a separate thread"""
    server_address = ('', 8001)
    httpd = HTTPServer(server_address, CaptureHandler)
    print("[CaptureServer] Starting HTTP capture server on port 8001...")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("[CaptureServer] Shutting down HTTP capture server")
        httpd.shutdown()

def main():
    """Main entry point"""
    print("[CaptureServer] ROS2 Capture HTTP Server starting...")
    
    # Start the server in a daemon thread so it doesn't block ROS2
    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.start()
    
    # Keep the main thread alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("[CaptureServer] Received interrupt, shutting down...")

if __name__ == '__main__':
    main()
