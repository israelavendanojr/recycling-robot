#!/usr/bin/env python3
"""
Browser-Compatible MJPEG Stream Server for Raspberry Pi 5 + ArduCam IMX708
This creates a proper MJPEG stream that works in all browsers and dashboards.
"""

import subprocess
import threading
import time
import signal
import sys
import os
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

class MJPEGStreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/feed.mjpg' or self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'close')
            self.end_headers()
            
            try:
                # Start rpicam-vid process
                process = subprocess.Popen([
                    'rpicam-vid',
                    '-t', '0',
                    '--width', '640',
                    '--height', '480', 
                    '--framerate', '10',
                    '--codec', 'mjpeg',
                    '--nopreview',
                    '-o', '-'
                ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                
                # Stream the output
                while True:
                    # Read JPEG frame
                    frame_data = b''
                    while not frame_data.endswith(b'\xff\xd9'):  # JPEG end marker
                        chunk = process.stdout.read(1024)
                        if not chunk:
                            break
                        frame_data += chunk
                    
                    if not frame_data:
                        break
                    
                    # Send frame with proper MJPEG headers
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(f'Content-Length: {len(frame_data)}\r\n'.encode())
                    self.wfile.write(b'\r\n')
                    self.wfile.write(frame_data)
                    self.wfile.write(b'\r\n')
                    self.wfile.flush()
                    
            except Exception as e:
                print(f"Stream error: {e}")
            finally:
                if 'process' in locals():
                    process.terminate()
                    
        elif self.path == '/status':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"status": "running", "camera": "imx708"}')
        else:
            self.send_response(404)
            self.end_headers()

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    allow_reuse_address = True

def signal_handler(sig, frame):
    print('\nShutting down server...')
    sys.exit(0)

def main():
    PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8554
    
    signal.signal(signal.SIGINT, signal_handler)
    
    server = ThreadedHTTPServer(('0.0.0.0', PORT), MJPEGStreamHandler)
    
    print(f"🎥 MJPEG Stream Server Starting...")
    print(f"🌐 Server running on port {PORT}")
    print(f"📹 Stream URL: http://localhost:{PORT}/feed.mjpg")
    print(f"📊 Status URL: http://localhost:{PORT}/status")
    print("Press Ctrl+C to stop")
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        server.shutdown()

if __name__ == '__main__':
    main()
