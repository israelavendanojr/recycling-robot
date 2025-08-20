"""
ROS2 Web Bridge Node - HTTP API bridge for dashboard.
Migrated from legacy/web/api.py with ROS 2 topic subscriptions.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from flask import Flask, Response, jsonify, render_template_string
import threading
import time
import cv2
import numpy as np
from typing import Optional, Dict, Any
from collections import Counter, deque

# Import custom messages - fallback for development
try:
    from recycling_robot_msgs.msg import ClassificationResult as ClassificationResultMsg
    from recycling_robot_msgs.srv import ClassifyImage
    CUSTOM_MSGS_AVAILABLE = True
except ImportError:
    from std_msgs.msg import String as ClassificationResultMsg
    from std_srvs.srv import Trigger as ClassifyImage
    CUSTOM_MSGS_AVAILABLE = False

class WebBridgeNode(Node):
    """
    ROS2 node that bridges ROS topics to HTTP API for web dashboard.
    Provides the same great dashboard from legacy code with ROS 2 backend.
    """
    
    def __init__(self):
        super().__init__('web_bridge_node')
        
        # Declare parameters
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8000)
        self.declare_parameter('enable_video_stream', True)
        
        # Get parameters
        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.enable_video = self.get_parameter('enable_video_stream').get_parameter_value().bool_value
        
        # Initialize components
        self.bridge = CvBridge()
        
        # State tracking (thread-safe)
        self._lock = threading.Lock()
        self.latest_image: Optional[np.ndarray] = None
        self.latest_classification: Optional[Dict[str, Any]] = None
        self.classification_counts = Counter()
        self.frame_times = deque(maxlen=60)
        self.start_time = time.time()
        
        # ROS subscriptions
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.classification_subscription = self.create_subscription(
            ClassificationResultMsg,
            '/classification_results',
            self.classification_callback,
            10
        )
        
        # Service client for on-demand classification
        self.classify_client = self.create_client(ClassifyImage, '/classify_image')
        
        # Flask app setup
        self.app = Flask(__name__)
        self._setup_flask_routes()
        
        # Start Flask in separate thread
        self.flask_thread = threading.Thread(
            target=self._run_flask,
            daemon=True,
            name="FlaskWebServer"
        )
        self.flask_thread.start()
        
        self.get_logger().info(f'✓ Web bridge started at http://{self.host}:{self.port}')
        self.get_logger().info('Dashboard available at: http://localhost:8000/')
    
    def image_callback(self, msg: Image) -> None:
        """Handle incoming camera images."""
        try:
            # Convert to numpy array
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            with self._lock:
                self.latest_image = frame
                self.frame_times.append(time.time())
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def classification_callback(self, msg) -> None:
        """Handle incoming classification results."""
        try:
            with self._lock:
                if CUSTOM_MSGS_AVAILABLE:
                    # Parse full classification message
                    self.latest_classification = {
                        'predicted_class': msg.predicted_class,
                        'confidence': msg.confidence,
                        'class_index': msg.class_index,
                        'all_probabilities': dict(zip(msg.class_names, msg.probabilities)),
                        'timestamp': time.time()
                    }
                    self.classification_counts[msg.predicted_class] += 1
                else:
                    # Parse fallback string format "class:confidence"
                    parts = msg.data.split(':')
                    if len(parts) == 2:
                        pred_class = parts[0]
                        confidence = float(parts[1])
                        self.latest_classification = {
                            'predicted_class': pred_class,
                            'confidence': confidence,
                            'class_index': 0,
                            'all_probabilities': {pred_class: confidence},
                            'timestamp': time.time()
                        }
                        self.classification_counts[pred_class] += 1
                
        except Exception as e:
            self.get_logger().error(f'Error processing classification: {e}')
    
    def _setup_flask_routes(self) -> None:
        """Setup Flask routes for web dashboard."""
        
        # Import the great dashboard HTML from legacy
        DASHBOARD_HTML = '''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>🤖 ROS 2 Recycling Robot Dashboard</title>
    <style>
        body { font-family: system-ui; margin: 20px; background: #f5f5f5; }
        .container { max-width: 1200px; margin: 0 auto; }
        .header { text-align: center; margin-bottom: 2rem; }
        .header h1 { color: #2c3e50; font-size: 2.5rem; margin-bottom: 0.5rem; }
        .header p { color: #666; font-size: 1.1rem; }
        .grid { display: grid; grid-template-columns: 2fr 1fr; gap: 2rem; margin-bottom: 2rem; }
        .card { background: white; border-radius: 12px; padding: 1.5rem; box-shadow: 0 2px 8px rgba(0,0,0,0.1); }
        .card h2 { color: #2c3e50; margin-bottom: 1rem; }
        .video-stream { width: 100%; border-radius: 8px; background: #eee; min-height: 300px; }
        .stats { display: grid; grid-template-columns: 1fr 1fr; gap: 1rem; margin-bottom: 1rem; }
        .stat { text-align: center; padding: 1rem; background: #f8f9fa; border-radius: 8px; }
        .stat-value { font-size: 2rem; font-weight: bold; color: #3498db; }
        .stat-label { color: #666; text-transform: uppercase; font-size: 0.9rem; }
        .prediction { text-align: center; padding: 1rem; background: linear-gradient(135deg, #667eea, #764ba2); color: white; border-radius: 8px; margin-bottom: 1rem; }
        .prediction-class { font-size: 1.8rem; font-weight: bold; margin-bottom: 0.5rem; }
        .confidence { font-size: 1.2rem; opacity: 0.9; }
        .btn { background: #3498db; color: white; border: none; padding: 0.75rem 1.5rem; border-radius: 8px; cursor: pointer; margin: 0.5rem; }
        .btn:hover { background: #2980b9; }
        .status-badge { display: inline-block; padding: 4px 8px; border-radius: 4px; font-size: 0.8rem; margin-left: 10px; }
        .status-online { background: #27ae60; color: white; }
        .status-offline { background: #e74c3c; color: white; }
        @media (max-width: 768px) { .grid { grid-template-columns: 1fr; } }
    </style>
</head>
<body>
    <div class="container">
        <header class="header">
            <h1>🤖 ROS 2 Recycling Robot</h1>
            <p>Real-time AI-powered waste classification system</p>
            <div id="ros-status" class="status-badge status-offline">ROS Status: Checking...</div>
        </header>

        <div class="grid">
            <div class="card">
                <h2>📹 Live Camera Feed</h2>
                <img id="video-stream" class="video-stream" alt="Camera feed loading..." 
                     onerror="this.alt='Camera feed unavailable';">
            </div>

            <div class="card">
                <h2>📊 Classification Results</h2>
                
                <div class="prediction">
                    <div id="prediction-class" class="prediction-class">Loading...</div>
                    <div id="confidence-display" class="confidence">Confidence: ---%</div>
                </div>

                <div class="stats">
                    <div class="stat">
                        <div id="fps-value" class="stat-value">--</div>
                        <div class="stat-label">FPS</div>
                    </div>
                    <div class="stat">
                        <div id="total-count" class="stat-value">--</div>
                        <div class="stat-label">Total</div>
                    </div>
                </div>

                <h3>Class Counts</h3>
                <div id="class-counts">Loading...</div>
            </div>
        </div>

        <div class="card">
            <h2>⚙️ System Status</h2>
            <div class="stats">
                <div class="stat">
                    <div id="uptime-value" class="stat-value">--</div>
                    <div class="stat-label">Uptime</div>
                </div>
                <div class="stat">
                    <div id="node-count" class="stat-value">--</div>
                    <div class="stat-label">ROS Nodes</div>
                </div>
            </div>
        </div>

        <div style="text-align: center; margin-top: 2rem;">
            <button class="btn" onclick="refreshData()">🔄 Refresh</button>
            <button class="btn" onclick="resetStats()">🗑️ Reset Stats</button>
            <a href="/health" class="btn">⚕️ Health Check</a>
        </div>
    </div>

    <script>
        let updateInterval;

        async function updateDashboard() {
            try {
                const response = await fetch('/api/stats');
                const data = await response.json();
                
                // Update ROS status
                const statusEl = document.getElementById('ros-status');
                if (data.ros_ok) {
                    statusEl.textContent = 'ROS Status: Online';
                    statusEl.className = 'status-badge status-online';
                } else {
                    statusEl.textContent = 'ROS Status: Offline';
                    statusEl.className = 'status-badge status-offline';
                }
                
                // Update prediction
                document.getElementById('prediction-class').textContent = 
                    data.current_prediction || 'No Detection';
                
                const confidence = (data.confidence || 0) * 100;
                document.getElementById('confidence-display').textContent = 
                    `Confidence: ${confidence.toFixed(1)}%`;
                
                // Update performance
                document.getElementById('fps-value').textContent = data.fps || '--';
                document.getElementById('total-count').textContent = data.total_classifications || '--';
                
                // Update uptime
                const uptime = data.uptime_seconds || 0;
                const hours = Math.floor(uptime / 3600);
                const minutes = Math.floor((uptime % 3600) / 60);
                document.getElementById('uptime-value').textContent = 
                    `${hours}:${minutes.toString().padStart(2, '0')}`;
                
                document.getElementById('node-count').textContent = data.active_nodes || '--';
                
                // Update class counts
                const counts = data.class_counts || {};
                const countsHtml = Object.entries(counts)
                    .map(([cls, cnt]) => `<div style="margin:4px 0;">${cls}: ${cnt}</div>`)
                    .join('') || '<div>No classifications yet</div>';
                document.getElementById('class-counts').innerHTML = countsHtml;
                
            } catch (error) {
                console.error('Error updating dashboard:', error);
                document.getElementById('ros-status').textContent = 'ROS Status: Error';
                document.getElementById('ros-status').className = 'status-badge status-offline';
            }
        }

        async function refreshData() {
            await updateDashboard();
        }

        async function resetStats() {
            if (!confirm('Reset all statistics?')) return;
            try {
                const response = await fetch('/api/stats/reset', { method: 'POST' });
                if (response.ok) {
                    await updateDashboard();
                    alert('Statistics reset!');
                } else {
                    alert('Failed to reset statistics');
                }
            } catch (error) {
                alert('Error resetting statistics');
            }
        }

        // Initialize
        document.addEventListener('DOMContentLoaded', function() {
            // Set video source if enabled
            const videoEl = document.getElementById('video-stream');
            videoEl.src = '/api/video/stream.mjpeg';
            
            // Start updates
            updateDashboard();
            updateInterval = setInterval(updateDashboard, 2000); // 2 second updates
        });

        window.addEventListener('beforeunload', function() {
            if (updateInterval) clearInterval(updateInterval);
        });
    </script>
</body>
</html>
        '''
        
        @self.app.route('/')
        def dashboard():
            """Main dashboard page."""
            return render_template_string(DASHBOARD_HTML)
        
        @self.app.route('/health')
        def health():
            """Health check endpoint."""
            return jsonify({
                "status": "healthy",
                "node_name": self.get_name(),
                "ros_ok": rclpy.ok(),
                "timestamp": time.time()
            })
        
        @self.app.route('/api/stats')
        def get_stats():
            """Get current system statistics."""
            with self._lock:
                # Calculate FPS
                fps = 0.0
                if len(self.frame_times) >= 2:
                    time_span = self.frame_times[-1] - self.frame_times[0]
                    if time_span > 0:
                        fps = (len(self.frame_times) - 1) / time_span
                
                # Current prediction
                current_pred = ""
                confidence = 0.0
                if self.latest_classification:
                    current_pred = self.latest_classification.get('predicted_class', '')
                    confidence = self.latest_classification.get('confidence', 0.0)
                
                # Uptime
                uptime = time.time() - self.start_time
                
                return jsonify({
                    "current_prediction": current_pred,
                    "confidence": confidence,
                    "fps": round(fps, 2),
                    "total_classifications": sum(self.classification_counts.values()),
                    "class_counts": dict(self.classification_counts),
                    "uptime_seconds": uptime,
                    "ros_ok": rclpy.ok(),
                    "active_nodes": 3,  # Mock for now
                    "timestamp": time.time()
                })
        
        @self.app.route('/api/stats/reset', methods=['POST'])
        def reset_stats():
            """Reset statistics."""
            with self._lock:
                self.classification_counts.clear()
                self.frame_times.clear()
                self.start_time = time.time()
            
            return jsonify({"message": "Statistics reset successfully"})
        
        @self.app.route('/api/video/stream.mjpeg')
        def video_stream():
            """MJPEG video stream."""
            if not self.enable_video:
                return jsonify({"error": "Video streaming disabled"}), 404
                
            return Response(
                self._generate_video_stream(),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )
        
        @self.app.route('/api/classify', methods=['POST'])
        def trigger_classification():
            """Trigger on-demand classification."""
            if not self.classify_client.service_is_ready():
                return jsonify({"error": "Classification service not available"}), 503
            
            with self._lock:
                if self.latest_image is None:
                    return jsonify({"error": "No camera image available"}), 400
                
                # Convert to ROS image
                ros_image = self.bridge.cv2_to_imgmsg(self.latest_image, encoding='rgb8')
            
            # Call classification service
            try:
                request = ClassifyImage.Request()
                if CUSTOM_MSGS_AVAILABLE:
                    request.image = ros_image
                    
                future = self.classify_client.call_async(request)
                # Note: In production, this should be async to avoid blocking Flask
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.result() is not None:
                    response = future.result()
                    return jsonify({
                        "success": response.success,
                        "message": response.message
                    })
                else:
                    return jsonify({"error": "Classification service timeout"}), 408
                    
            except Exception as e:
                return jsonify({"error": f"Classification failed: {e}"}), 500
    
    def _generate_video_stream(self):
        """Generate MJPEG video stream."""
        boundary = b'frame'
        
        while True:
            try:
                with self._lock:
                    if self.latest_image is None:
                        # Generate placeholder image
                        frame = np.zeros((480, 640, 3), dtype=np.uint8)
                        cv2.putText(frame, "Waiting for camera...", (50, 240),
                                  cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    else:
                        frame = self.latest_image.copy()
                        
                        # Add overlay with latest classification
                        if self.latest_classification:
                            pred = self.latest_classification['predicted_class']
                            conf = self.latest_classification['confidence']
                            
                            # Add text overlay
                            text = f"{pred} ({conf*100:.1f}%)"
                            cv2.putText(frame, text, (10, 30),
                                      cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Convert RGB to BGR for OpenCV encoding
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                
                # Encode as JPEG
                success, buffer = cv2.imencode('.jpg', frame_bgr, 
                                             [int(cv2.IMWRITE_JPEG_QUALITY), 75])
                
                if not success:
                    time.sleep(0.1)
                    continue
                
                frame_bytes = buffer.tobytes()
                
                # Yield MJPEG frame
                yield (b'--' + boundary + b'\r\n'
                       b'Content-Type: image/jpeg\r\n'
                       b'Content-Length: ' + str(len(frame_bytes)).encode() + b'\r\n\r\n' +
                       frame_bytes + b'\r\n')
                
                time.sleep(0.033)  # ~30 FPS
                
            except Exception as e:
                self.get_logger().error(f'Error in video stream: {e}')
                time.sleep(0.1)
    
    def _run_flask(self):
        """Run Flask server in separate thread."""
        try:
            self.app.run(
                host=self.host,
                port=self.port,
                debug=False,
                threaded=True,
                use_reloader=False
            )
        except Exception as e:
            self.get_logger().error(f'Flask server error: {e}')
    
    def destroy_node(self):
        """Clean shutdown."""
        self.get_logger().info('Shutting down web bridge...')
        super().destroy_node()

def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = WebBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Web bridge error: {e}')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()