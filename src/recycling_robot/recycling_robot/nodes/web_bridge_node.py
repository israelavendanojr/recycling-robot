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
        DASHBOARD_HTML = """
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Recycling Robot Dashboard</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    :root { --bg:#fff; --muted:#f4f5f7; --text:#1f2937; --sub:#6b7280; --line:#e5e7eb; --accent:#2563eb; }
    * { box-sizing: border-box; }
    body { margin: 0; font: 14px/1.5 system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif; color: var(--text); background: var(--bg); }
    header { padding: 16px 20px; border-bottom: 1px solid var(--line); }
    header h1 { margin: 0; font-size: 18px; }
    header .meta { margin-top: 4px; color: var(--sub); font-size: 13px; }
    .wrap { max-width: 1100px; margin: 0 auto; padding: 20px; }
    .grid { display: grid; grid-template-columns: 2fr 1fr; gap: 20px; }
    .card { background: #fff; border: 1px solid var(--line); border-radius: 8px; }
    .card .hd { padding: 12px 14px; border-bottom: 1px solid var(--line); font-weight: 600; }
    .card .bd { padding: 14px; }
    .video { display: block; width: 100%; aspect-ratio: 4 / 3; object-fit: cover; background: var(--muted); border-radius: 6px; }
    .stats { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; }
    .stat { border: 1px solid var(--line); border-radius: 6px; padding: 10px; }
    .stat .label { color: var(--sub); font-size: 12px; }
    .stat .value { font-size: 20px; font-weight: 600; margin-top: 2px; }
    table { width: 100%; border-collapse: collapse; font-size: 13px; }
    th, td { padding: 8px 6px; border-bottom: 1px solid var(--line); text-align: left; }
    .row { display: flex; gap: 10px; align-items: center; }
    .badge { padding: 2px 6px; border-radius: 4px; font-size: 12px; border: 1px solid var(--line); }
    .ok { color: #065f46; background: #ecfdf5; border-color: #a7f3d0; }
    .bad { color: #7f1d1d; background: #fef2f2; border-color: #fecaca; }
    .controls { display: flex; gap: 8px; }
    button, .link { cursor: pointer; border: 1px solid var(--line); background: #fff; padding: 8px 10px; border-radius: 6px; font: inherit; }
    button:hover, .link:hover { border-color: var(--accent); }
    @media (max-width: 900px) { .grid { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <header>
    <h1>Recycling Robot</h1>
    <div class="meta">
      Minimal ROS 2 dashboard · <span id="ros-status" class="badge">checking…</span>
    </div>
  </header>

  <div class="wrap">
    <div class="grid">
      <section class="card">
        <div class="hd">Camera</div>
        <div class="bd">
          <img id="video" class="video" alt="camera stream" />
        </div>
      </section>

      <section class="card">
        <div class="hd">Prediction</div>
        <div class="bd">
          <div class="row" style="justify-content: space-between; margin-bottom: 12px;">
            <div>
              <div class="label" style="color:var(--sub); font-size:12px;">Class</div>
              <div id="pred-class" style="font-weight:600; font-size:18px;">—</div>
            </div>
            <div>
              <div class="label" style="color:var(--sub); font-size:12px;">Confidence</div>
              <div id="pred-conf" style="font-weight:600; font-size:18px;">—</div>
            </div>
          </div>

          <div class="stats" style="margin-top: 6px;">
            <div class="stat">
              <div class="label">FPS</div>
              <div id="fps" class="value">—</div>
            </div>
            <div class="stat">
              <div class="label">Total classified</div>
              <div id="total" class="value">—</div>
            </div>
          </div>
        </div>
      </section>
    </div>

    <section class="card" style="margin-top:20px;">
      <div class="hd">Class counts</div>
      <div class="bd">
        <table id="counts-table">
          <thead><tr><th>Class</th><th>Count</th></tr></thead>
          <tbody><tr><td colspan="2">No data</td></tr></tbody>
        </table>
      </div>
    </section>

    <section class="card" style="margin-top:20px;">
      <div class="hd">System</div>
      <div class="bd row" style="justify-content: space-between;">
        <div class="row" style="gap:20px;">
          <div><span class="label">Uptime</span><div id="uptime" class="value" style="font-size:16px;">—</div></div>
          <div><span class="label">Nodes</span><div id="nodes" class="value" style="font-size:16px;">—</div></div>
        </div>
        <div class="controls">
          <button id="refresh">Refresh</button>
          <button id="reset">Reset stats</button>
          <a class="link" href="/health">Health</a>
        </div>
      </div>
    </section>
  </div>

  <script>
    function setBadge(ok) {
      const el = document.getElementById('ros-status');
      el.textContent = ok ? 'online' : 'offline';
      el.className = 'badge ' + (ok ? 'ok' : 'bad');
    }

    function pad(n){return n.toString().padStart(2,'0');}

    async function loadStats() {
      try {
        const r = await fetch('/api/stats');
        const d = await r.json();

        setBadge(!!d.ros_ok);

        document.getElementById('pred-class').textContent = d.current_prediction || '—';
        document.getElementById('pred-conf').textContent  = d.confidence != null ? (d.confidence*100).toFixed(1)+'%' : '—';
        document.getElementById('fps').textContent        = d.fps != null ? d.fps : '—';
        document.getElementById('total').textContent      = d.total_classifications != null ? d.total_classifications : '—';

        const up = Math.max(0, Math.floor(d.uptime_seconds || 0));
        const h = Math.floor(up/3600), m = Math.floor((up%3600)/60);
        document.getElementById('uptime').textContent = h + ':' + pad(m);

        document.getElementById('nodes').textContent = d.active_nodes != null ? d.active_nodes : '—';

        const tbody = document.querySelector('#counts-table tbody');
        const counts = d.class_counts || {};
        const entries = Object.entries(counts);
        tbody.innerHTML = entries.length
          ? entries.map(([k,v]) => '<tr><td>'+k+'</td><td>'+v+'</td></tr>').join('')
          : '<tr><td colspan="2">No data</td></tr>';
      } catch (e) {
        setBadge(false);
      }
    }

    async function resetStats() {
      if (!confirm('Reset all statistics?')) return;
      try {
        const r = await fetch('/api/stats/reset', { method: 'POST' });
        if (r.ok) loadStats();
      } catch {}
    }

    // wire up
    document.addEventListener('DOMContentLoaded', () => {
      document.getElementById('video').src = '/api/video/stream.mjpeg';
      document.getElementById('refresh').addEventListener('click', loadStats);
      document.getElementById('reset').addEventListener('click', resetStats);
      loadStats();
      setInterval(loadStats, 2000);
    });
  </script>
</body>
</html>
"""

        
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