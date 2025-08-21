#!/bin/bash
# Working Camera Stream for Raspberry Pi 5 + ArduCam IMX708

set -e

# Configuration
PORT=${1:-8554}
WIDTH=${2:-640}
HEIGHT=${3:-480}
FPS=${4:-10}

echo "🎥 Starting Camera Stream on Port $PORT"
echo "========================================"

# Kill any existing streams
pkill -f "rpicam-vid" 2>/dev/null || true
pkill -f "python3.*http.server" 2>/dev/null || true
pkill -f "ffmpeg.*mjpeg" 2>/dev/null || true
sleep 2

# Create stream directory
STREAM_DIR="/tmp/camera_stream"
mkdir -p "$STREAM_DIR"
cd "$STREAM_DIR"

echo "Starting camera stream..."

# Method 1: Simple MJPEG stream to file that we serve
rpicam-vid -t 0 \
    --width "$WIDTH" --height "$HEIGHT" \
    --framerate "$FPS" \
    --codec mjpeg \
    --nopreview \
    -o stream.mjpg &

RPICAM_PID=$!
echo "rpicam-vid started with PID: $RPICAM_PID"

# Wait for stream file to be created
sleep 3

# Start a simple HTTP server to serve the stream
echo "Starting HTTP server on port $PORT..."

# Create a simple Python HTTP server for the stream
cat > server.py << 'EOF'
#!/usr/bin/env python3
import http.server
import socketserver
import os
import threading
import time

class StreamHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/feed.mjpg' or self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'close')
            self.end_headers()
            
            try:
                with open('stream.mjpg', 'rb') as f:
                    # Read and send the current content
                    content = f.read()
                    if content:
                        self.wfile.write(content)
            except:
                pass
        else:
            # Serve index page
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            html = '''
            <html>
            <head><title>Pi Camera Stream</title></head>
            <body>
            <h1>Raspberry Pi Camera Stream</h1>
            <img src="/feed.mjpg" width="640" height="480" alt="Camera Stream">
            <p>Direct link: <a href="/feed.mjpg">/feed.mjpg</a></p>
            </body>
            </html>
            '''
            self.wfile.write(html.encode())

if __name__ == "__main__":
    PORT = int(os.environ.get('PORT', 8554))
    with socketserver.TCPServer(("", PORT), StreamHandler) as httpd:
        print(f"Server running on port {PORT}")
        httpd.serve_forever()
EOF

# Start the HTTP server
PORT=$PORT python3 server.py &
SERVER_PID=$!

echo "HTTP server started with PID: $SERVER_PID"

# Get Pi IP
PI_IP=$(hostname -I | awk '{print $1}')

echo ""
echo "✅ Camera stream is running!"
echo "🌐 Stream URL: http://$PI_IP:$PORT/feed.mjpg"
echo "🌐 Web interface: http://$PI_IP:$PORT/"
echo ""
echo "Stream PIDs:"
echo "  rpicam-vid: $RPICAM_PID"
echo "  HTTP server: $SERVER_PID"
echo ""
echo "To stop the stream:"
echo "  kill $RPICAM_PID $SERVER_PID"
echo ""
echo "Testing stream availability..."
sleep 2

# Test the stream
if curl -s -I "http://localhost:$PORT/feed.mjpg" | grep -q "200 OK"; then
    echo "✅ Stream is accessible!"
else
    echo "⚠️  Stream may still be starting up..."
fi

# Save PIDs for easy cleanup
echo "$RPICAM_PID $SERVER_PID" > /tmp/camera_stream_pids.txt
echo "PIDs saved to /tmp/camera_stream_pids.txt"
