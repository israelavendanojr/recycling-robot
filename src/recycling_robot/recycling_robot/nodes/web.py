#!/usr/bin/env python3
"""
Simple ROS2 Web Dashboard Node
Serves camera feed and classification results via HTTP
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from flask import Flask, Response, jsonify, render_template_string
import cv2
import numpy as np
import threading
import time
from collections import Counter


class SimpleWebNode(Node):
    def __init__(self):
        super().__init__('simple_web')
        
        # Parameters
        self.declare_parameter('port', 8000)
        port = self.get_parameter('port').value
        
        # Setup
        self.bridge = CvBridge()
        
        # State (thread-safe)
        self._lock = threading.Lock()
        self.latest_image = None
        self.latest_result = None
        self.classification_counts = Counter()
        self.start_time = time.time()
        
        # Subscribe to camera and classification results
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.result_sub = self.create_subscription(
            String, '/classification_result', self.result_callback, 10
        )
        
        # Service client for manual classification
        self.classify_client = self.create_client(Trigger, '/classify_image')
        
        # Flask app
        self.app = Flask(__name__)
        self._setup_routes()
        
        # Start Flask in background thread
        self.flask_thread = threading.Thread(
            target=lambda: self.app.run(host='0.0.0.0', port=port, debug=False),
            daemon=True
        )
        self.flask_thread.start()
        
        self.get_logger().info(f'Web dashboard started at http://0.0.0.0:{port}')

    def image_callback(self, msg):
        """Store latest camera image"""
        try:
            with self._lock:
                self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')

    def result_callback(self, msg):
        """Store latest classification result"""
        try:
            # Parse "class:confidence" format
            parts = msg.data.split(':')
            if len(parts) == 2:
                with self._lock:
                    self.latest_result = {
                        'class': parts[0],
                        'confidence': float(parts[1]),
                        'timestamp': time.time()
                    }
                    self.classification_counts[parts[0]] += 1
        except Exception as e:
            self.get_logger().error(f'Result parsing error: {e}')

    def _setup_routes(self):
        """Setup Flask routes"""
        
        # Dashboard HTML (simplified)
        DASHBOARD_HTML = '''
<!DOCTYPE html>
<html>
<head>
    <title>Recycling Robot Dashboard</title>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background: #f5f5f5; }
        .container { max-width: 1200px; margin: 0 auto; }
        .header { background: white; padding: 20px; border-radius: 8px; margin-bottom: 20px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        .grid { display: grid; grid-template-columns: 2fr 1fr; gap: 20px; }
        .card { background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        .video { width: 100%; height: auto; border-radius: 4px; }
        .stats { display: grid; grid-template-columns: repeat(2, 1fr); gap: 15px; margin-top: 15px; }
        .stat { background: #f8f9fa; padding: 15px; border-radius: 4px; text-align: center; }
        .stat-value { font-size: 24px; font-weight: bold; color: #2563eb; }
        .stat-label { font-size: 12px; color: #666; margin-top: 5px; }
        button { background: #2563eb; color: white; border: none; padding: 10px 20px; border-radius: 4px; cursor: pointer; }
        button:hover { background: #1d4ed8; }
        button:disabled { background: #9ca3af; cursor: not-allowed; }
        .prediction { font-size: 20px; font-weight: bold; color: #059669; margin: 10px 0; }
        .confidence { color: #666; }
        table { width: 100%; border-collapse: collapse; margin-top: 15px; }
        th, td { padding: 8px 12px; text-align: left; border-bottom: 1px solid #e5e7eb; }
        th { background: #f8f9fa; font-weight: 600; }
        @media (max-width: 768px) { .grid { grid-template-columns: 1fr; } }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>Recycling Robot Dashboard</h1>
            <p>Simple ROS2 Dashboard - <span id="status">Loading...</span></p>
        </div>
        
        <div class="grid">
            <div class="card">
                <h2>Camera Feed</h2>
                <img id="video" class="video" src="/video_feed" alt="Camera Feed">
                <button id="classify-btn" onclick="classifyNow()">Classify Now</button>
                <small style="margin-left: 10px; color: #666;">Auto-classification every 3 seconds</small>
            </div>
            
            <div class="card">
                <h2>Latest Prediction</h2>
                <div class="prediction" id="prediction">—</div>
                <div class="confidence" id="confidence">—</div>
                
                <div class="stats">
                    <div class="stat">
                        <div class="stat-value" id="total-count">—</div>
                        <div class="stat-label">Total Classifications</div>
                    </div>
                    <div class="stat">
                        <div class="stat-value" id="uptime">—</div>
                        <div class="stat-label">Uptime</div>
                    </div>
                </div>
                
                <h3>Class Counts</h3>
                <table id="counts-table">
                    <thead><tr><th>Material</th><th>Count</th><th>%</th></tr></thead>
                    <tbody><tr><td colspan="3">No data yet</td></tr></tbody>
                </table>
                
                <div style="margin-top: 20px;">
                    <button onclick="resetStats()">Reset Stats</button>
                </div>
            </div>
        </div>
    </div>

    <script>
        let isClassifying = false;

        async function loadStats() {
            try {
                const response = await fetch('/api/stats');
                const data = await response.json();
                
                document.getElementById('status').textContent = 'Online';
                document.getElementById('status').style.color = '#059669';
                
                // Update prediction
                if (data.latest_result) {
                    document.getElementById('prediction').textContent = data.latest_result.class.toUpperCase();
                    document.getElementById('confidence').textContent = 
                        `${(data.latest_result.confidence * 100).toFixed(1)}% confidence`;
                } else {
                    document.getElementById('prediction').textContent = '—';
                    document.getElementById('confidence').textContent = '—';
                }
                
                // Update stats
                document.getElementById('total-count').textContent = data.total_classifications || '0';
                
                // Update uptime
                const uptime = Math.floor(data.uptime || 0);
                const hours = Math.floor(uptime / 3600);
                const minutes = Math.floor((uptime % 3600) / 60);
                document.getElementById('uptime').textContent = `${hours}:${minutes.toString().padStart(2, '0')}`;
                
                // Update counts table
                const tbody = document.querySelector('#counts-table tbody');
                const counts = data.class_counts || {};
                const entries = Object.entries(counts);
                const total = entries.reduce((sum, [,count]) => sum + count, 0);
                
                if (entries.length === 0) {
                    tbody.innerHTML = '<tr><td colspan="3">No data yet</td></tr>';
                } else {
                    tbody.innerHTML = entries
                        .sort((a, b) => b[1] - a[1])
                        .map(([cls, count]) => {
                            const percent = total > 0 ? ((count / total) * 100).toFixed(1) : '0';
                            return `<tr><td>${cls}</td><td>${count}</td><td>${percent}%</td></tr>`;
                        })
                        .join('');
                }
                
            } catch (error) {
                document.getElementById('status').textContent = 'Offline';
                document.getElementById('status').style.color = '#dc2626';
            }
        }

        async function classifyNow() {
            if (isClassifying) return;
            
            isClassifying = true;
            const btn = document.getElementById('classify-btn');
            btn.disabled = true;
            btn.textContent = 'Classifying...';
            
            try {
                const response = await fetch('/api/classify', { method: 'POST' });
                const result = await response.json();
                
                if (result.success) {
                    setTimeout(loadStats, 500); // Refresh stats after classification
                }
            } catch (error) {
                console.error('Classification failed:', error);
            } finally {
                isClassifying = false;
                btn.disabled = false;
                btn.textContent = 'Classify Now';
            }
        }

        async function resetStats() {
            if (confirm('Reset all statistics?')) {
                try {
                    await fetch('/api/reset', { method: 'POST' });
                    loadStats();
                } catch (error) {
                    console.error('Reset failed:', error);
                }
            }
        }

        // Load stats every 2 seconds
        loadStats();
        setInterval(loadStats, 2000);
    </script>
</body>
</html>
        '''
        
        @self.app.route('/')
        def dashboard():
            return render_template_string(DASHBOARD_HTML)
        
        @self.app.route('/video_feed')
        def video_feed():
            return Response(self._generate_frames(),
                          mimetype='multipart/x-mixed-replace; boundary=frame')
        
        @self.app.route('/api/stats')
        def get_stats():
            with self._lock:
                total = sum(self.classification_counts.values())
                uptime = time.time() - self.start_time
                
                return jsonify({
                    'latest_result': self.latest_result,
                    'total_classifications': total,
                    'class_counts': dict(self.classification_counts),
                    'uptime': uptime
                })
        
        @self.app.route('/api/classify', methods=['POST'])
        def trigger_classify():
            if not self.classify_client.service_is_ready():
                return jsonify({'success': False, 'message': 'Service not ready'})
            
            try:
                request = Trigger.Request()
                future = self.classify_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
                
                if future.result():
                    return jsonify({'success': future.result().success, 
                                  'message': future.result().message})
                else:
                    return jsonify({'success': False, 'message': 'Timeout'})
            except Exception as e:
                return jsonify({'success': False, 'message': str(e)})
        
        @self.app.route('/api/reset', methods=['POST'])
        def reset_stats():
            with self._lock:
                self.classification_counts.clear()
                self.start_time = time.time()
            return jsonify({'success': True})

    def _generate_frames(self):
        """Generate video stream frames"""
        while True:
            try:
                with self._lock:
                    if self.latest_image is None:
                        # Create placeholder frame
                        frame = np.zeros((480, 640, 3), dtype=np.uint8)
                        cv2.putText(frame, "Waiting for camera...", (50, 240),
                                  cv2.FONT_HERSHEY_SIMPLEX, 1, (128, 128, 128), 2)
                    else:
                        frame = self.latest_image.copy()
                        
                        # Add overlay with latest classification
                        if self.latest_result:
                            pred_class = self.latest_result['class']
                            confidence = self.latest_result['confidence']
                            
                            # Add text overlay
                            cv2.putText(frame, f"{pred_class.upper()}", (10, 30),
                                      cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                            cv2.putText(frame, f"{confidence*100:.1f}%", (10, 65),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Encode frame as JPEG
                ret, buffer = cv2.imencode('.jpg', frame, 
                                         [cv2.IMWRITE_JPEG_QUALITY, 85])
                if not ret:
                    continue
                
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                
                time.sleep(0.1)  # ~10 FPS for web
                
            except Exception as e:
                self.get_logger().error(f'Video streaming error: {e}')
                time.sleep(0.5)


def main():
    rclpy.init()
    try:
        node = SimpleWebNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()