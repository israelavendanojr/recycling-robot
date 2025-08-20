"""
ROS2 Web Bridge Node - HTTP API bridge for dashboard.
Enhanced version with automatic classification triggering.
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
    Enhanced with automatic classification triggering for better demo experience.
    """
    
    def __init__(self):
        super().__init__('web_bridge_node')
        
        # Declare parameters
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8000)
        self.declare_parameter('enable_video_stream', True)
        self.declare_parameter('auto_classify_interval', 3.0)  # Classify every 3 seconds
        
        # Get parameters
        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.enable_video = self.get_parameter('enable_video_stream').get_parameter_value().bool_value
        self.auto_classify_interval = self.get_parameter('auto_classify_interval').get_parameter_value().double_value
        
        # Initialize components
        self.bridge = CvBridge()
        
        # State tracking (thread-safe)
        self._lock = threading.Lock()
        self.latest_image: Optional[np.ndarray] = None
        self.latest_classification: Optional[Dict[str, Any]] = None
        self.classification_counts = Counter()
        self.frame_times = deque(maxlen=60)
        self.start_time = time.time()
        self.last_auto_classify = 0.0
        
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
        
        # Auto-classification timer
        self.auto_classify_timer = self.create_timer(
            self.auto_classify_interval,
            self.auto_classify_callback
        )
        
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
        self.get_logger().info(f'Auto-classification enabled (every {self.auto_classify_interval}s)')
    
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
    
    def auto_classify_callback(self) -> None:
        """Automatically trigger classification for demo purposes."""
        if not self.classify_client.service_is_ready():
            return
            
        with self._lock:
            if self.latest_image is None:
                return
            
            # Convert to ROS image
            ros_image = self.bridge.cv2_to_imgmsg(self.latest_image, encoding='rgb8')
        
        # Call classification service asynchronously
        try:
            request = ClassifyImage.Request()
            if CUSTOM_MSGS_AVAILABLE:
                request.image = ros_image
                
            future = self.classify_client.call_async(request)
            # Don't block - just fire and forget for auto classification
            
        except Exception as e:
            self.get_logger().debug(f'Auto-classification failed: {e}')
    
    def _setup_flask_routes(self) -> None:
        """Setup Flask routes for web dashboard."""
        
        # Import the great dashboard HTML from legacy with enhancements
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
    button:disabled { opacity: 0.5; cursor: not-allowed; }
    .classify-btn { background: var(--accent); color: white; border-color: var(--accent); }
    .classify-btn:hover:not(:disabled) { background: #1d4ed8; }
    @media (max-width: 900px) { .grid { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <header>
    <h1>🤖 Recycling Robot</h1>
    <div class="meta">
      Enhanced ROS 2 dashboard · <span id="ros-status" class="badge">checking…</span>
    </div>
  </header>

  <div class="wrap">
    <div class="grid">
      <section class="card">
        <div class="hd">Camera Feed</div>
        <div class="bd">
          <img id="video" class="video" alt="camera stream" />
          <div style="margin-top: 12px;">
            <button id="classify-now" class="classify-btn">Classify Now</button>
            <small style="color: var(--sub); margin-left: 10px;">
              Auto-classification every 3 seconds
            </small>
          </div>
        </div>
      </section>

      <section class="card">
        <div class="hd">Current Prediction</div>
        <div class="bd">
          <div class="row" style="justify-content: space-between; margin-bottom: 12px;">
            <div>
              <div class="label" style="color:var(--sub); font-size:12px;">Class</div>
              <div id="pred-class" style="font-weight:600; font-size:18px; color: var(--accent);">—</div>
            </div>
            <div>
              <div class="label" style="color:var(--sub); font-size:12px;">Confidence</div>
              <div id="pred-conf" style="font-weight:600; font-size:18px;">—</div>
            </div>
          </div>

          <div class="stats" style="margin-top: 16px;">
            <div class="stat">
              <div class="label">Camera FPS</div>
              <div id="fps" class="value">—</div>
            </div>
            <div class="stat">
              <div class="label">Classifications</div>
              <div id="total" class="value">—</div>
            </div>
          </div>
        </div>
      </section>
    </div>

    <section class="card" style="margin-top:20px;">
      <div class="hd">Classification History</div>
      <div class="bd">
        <table id="counts-table">
          <thead><tr><th>Material</th><th>Count</th><th>%</th></tr></thead>
          <tbody><tr><td colspan="3" style="text-align: center; color: var(--sub);">No classifications yet</td></tr></tbody>
        </table>
      </div>
    </section>

    <section class="card" style="margin-top:20px;">
      <div class="hd">System Status</div>
      <div class="bd row" style="justify-content: space-between;">
        <div class="row" style="gap:20px;">
          <div><span class="label">Uptime</span><div id="uptime" class="value" style="font-size:16px;">—</div></div>
          <div><span class="label">Nodes</span><div id="nodes" class="value" style="font-size:16px;">—</div></div>
        </div>
        <div class="controls">
          <button id="refresh">Refresh</button>
          <button id="reset">Reset Stats</button>
          <a class="link" href="/health">Health</a>
        </div>
      </div>
    </section>
  </div>

  <script>
    let isClassifying = false;

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
        document.getElementById('fps').textContent        = d.fps != null ? d.fps.toFixed(1) : '—';
        document.getElementById('total').textContent      = d.total_classifications != null ? d.total_classifications : '—';

        const up = Math.max(0, Math.floor(d.uptime_seconds || 0));
        const h = Math.floor(up/3600), m = Math.floor((up%3600)/60);
        document.getElementById('uptime').textContent = h + ':' + pad(m);

        document.getElementById('nodes').textContent = d.active_nodes != null ? d.active_nodes : '—';

        const tbody = document.querySelector('#counts-table tbody');
        const counts = d.class_counts || {};
        const entries = Object.entries(counts);
        const total = entries.reduce((sum, [,v]) => sum + v, 0);
        
        tbody.innerHTML = entries.length
          ? entries.sort((a,b) => b[1] - a[1]).map(([k,v]) => {
              const pct = total > 0 ? ((v/total)*100).toFixed(1) + '%' : '0%';
              return `<tr><td>${k}</td><td>${v}</td><td>${pct}</td></tr>`;
            }).join('')
          : '<tr><td colspan="3" style="text-align: center; color: var(--sub);">No classifications yet</td></tr>';
      } catch (e) {
        setBadge(false);
      }
    }

    async function classifyNow() {
      if (isClassifying) return;
      
      isClassifying = true;
      const btn = document.getElementById('classify-now');
      btn.disabled = true;
      btn.textContent = 'Classifying...';
      
      try {
        const r = await fetch('/api/classify', { method: 'POST' });
        const result = await r.json();
        
        if (!result.success) {
          console.error('Classification failed:', result.error);
        }
        
        // Refresh stats to show new result
        setTimeout(loadStats, 500);
        
      } catch (e) {
        console.error('Classification request failed:', e);
      } finally {
        isClassifying = false;
        btn.disabled = false;
        btn.textContent = 'Classify Now';
      }
    }

    async function resetStats() {
      if (!confirm('Reset all classification statistics?')) return;
      try {
        const r = await fetch('/api/stats/reset', { method: 'POST' });
        if (r.ok) loadStats();
      } catch {}
    }

    // Wire up event handlers
    document.addEventListener('DOMContentLoaded', () => {
      document.getElementById('video').src = '/api/video/stream.mjpeg';
      document.getElementById('refresh').addEventListener('click', loadStats);
      document.getElementById('reset').addEventListener('click', resetStats);
      document.getElementById('classify-now').addEventListener('click', classifyNow);
      
      // Initial load and periodic updates
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
                "camera_active": self.latest_image is not None,
                "classification_service": self.classify_client.service_is_ready(),
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
                    "active_nodes": 3,  # camera, classifier, web_bridge
                    "camera_active": self.latest_image is not None,
                    "classification_service_ready": self.classify_client.service_is_ready(),
                    "timestamp": time.time()
                })
        
        @self.app.route('/api/stats/reset', methods=['POST'])
        def reset_stats():
            """Reset statistics."""
            with self._lock:
                self.classification_counts.clear()
                self.frame_times.clear()
                self.start_time = time.time()
            
            self.get_logger().info('Statistics reset via web dashboard')
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
                # Use a short timeout to avoid blocking the web request too long
                rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
                
                if future.result() is not None:
                    response = future.result()
                    self.get_logger().info('Manual classification triggered via web dashboard')
                    return jsonify({
                        "success": response.success,
                        "message": response.message
                    })
                else:
                    return jsonify({"error": "Classification service timeout"}), 408
                    
            except Exception as e:
                self.get_logger().error(f'Manual classification failed: {e}')
                return jsonify({"error": f"Classification failed: {e}"}), 500
    
    def _generate_video_stream(self):
        """Generate MJPEG video stream with enhanced overlays."""
        boundary = b'frame'
        
        while True:
            try:
                with self._lock:
                    if self.latest_image is None:
                        # Generate placeholder image
                        frame = np.zeros((480, 640, 3), dtype=np.uint8)
                        cv2.putText(frame, "Waiting for camera...", (50, 240),
                                  cv2.FONT_HERSHEY_SIMPLEX, 1, (128, 128, 128), 2)
                    else:
                        frame = self.latest_image.copy()
                        
                        # Add overlay with latest classification
                        if self.latest_classification:
                            pred = self.latest_classification['predicted_class']
                            conf = self.latest_classification['confidence']
                            age = time.time() - self.latest_classification['timestamp']
                            
                            # Main prediction text
                            text = f"{pred.upper()}"
                            conf_text = f"{conf*100:.1f}%"
                            
                            # Add colored background for better readability
                            (text_w, text_h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)
                            (conf_w, conf_h), _ = cv2.getTextSize(conf_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                            
                            # Background rectangle
                            cv2.rectangle(frame, (8, 8), (max(text_w, conf_w) + 20, text_h + conf_h + 20), 
                                        (0, 0, 0), -1)
                            cv2.rectangle(frame, (8, 8), (max(text_w, conf_w) + 20, text_h + conf_h + 20), 
                                        (0, 255, 0) if conf > 0.7 else (0, 255, 255), 2)
                            
                            # Text overlays
                            cv2.putText(frame, text, (15, 30),
                                      cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                            cv2.putText(frame, conf_text, (15, 55),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
                            
                            # Age indicator (fade if old)
                            if age > 5:
                                alpha = max(0.3, 1.0 - (age - 5) / 10)
                                overlay = frame.copy()
                                cv2.putText(overlay, "OLD", (15, 80),
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
                                frame = cv2.addWeighted(frame, 1.0, overlay, alpha, 0)
                        
                        # Add timestamp
                        timestamp = time.strftime("%H:%M:%S")
                        cv2.putText(frame, timestamp, (frame.shape[1] - 100, 25),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
                
                # Convert RGB to BGR for OpenCV encoding
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                
                # Encode as JPEG
                success, buffer = cv2.imencode('.jpg', frame_bgr, 
                                             [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                
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