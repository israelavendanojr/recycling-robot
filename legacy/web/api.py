# src/web/api.py (Updated)
"""
Flask API for the recycling robot dashboard.
Clean REST API design, ready for future React frontend integration.
"""

import cv2
import time
import json
from flask import Flask, Response, jsonify, request, render_template_string
from typing import Optional
import logging

from src.services.inference_service import InferenceService

logger = logging.getLogger(__name__)

# Inline HTML dashboard template (will be moved to separate file later)
DASHBOARD_HTML = '''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Recycling Robot Dashboard</title>
    <style>
        :root {
            --primary-color: #2c3e50;
            --secondary-color: #3498db;
            --success-color: #27ae60;
            --warning-color: #f39c12;
            --danger-color: #e74c3c;
            --light-gray: #f8f9fa;
            --medium-gray: #6c757d;
            --dark-gray: #343a40;
        }

        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background-color: var(--light-gray);
            color: var(--dark-gray);
            line-height: 1.6;
        }

        .container {
            max-width: 1200px;
            margin: 0 auto;
            padding: 20px;
        }

        .header {
            text-align: center;
            margin-bottom: 2rem;
        }

        .header h1 {
            color: var(--primary-color);
            font-size: 2.5rem;
            font-weight: 700;
            margin-bottom: 0.5rem;
        }

        .header p {
            color: var(--medium-gray);
            font-size: 1.1rem;
        }

        .dashboard-grid {
            display: grid;
            grid-template-columns: 2fr 1fr;
            gap: 2rem;
            margin-bottom: 2rem;
        }

        @media (max-width: 768px) {
            .dashboard-grid {
                grid-template-columns: 1fr;
            }
        }

        .card {
            background: white;
            border-radius: 12px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.07);
            padding: 1.5rem;
            border: 1px solid #e9ecef;
        }

        .card h2 {
            color: var(--primary-color);
            font-size: 1.5rem;
            margin-bottom: 1rem;
            font-weight: 600;
        }

        .video-container {
            position: relative;
        }

        .video-stream {
            width: 100%;
            border-radius: 8px;
            background: #f1f3f4;
            min-height: 300px;
            object-fit: contain;
        }

        .status-indicator {
            position: absolute;
            top: 10px;
            right: 10px;
            width: 12px;
            height: 12px;
            border-radius: 50%;
            background: var(--success-color);
            box-shadow: 0 0 0 3px rgba(39, 174, 96, 0.3);
            animation: pulse 2s infinite;
        }

        @keyframes pulse {
            0% { box-shadow: 0 0 0 0 rgba(39, 174, 96, 0.7); }
            70% { box-shadow: 0 0 0 10px rgba(39, 174, 96, 0); }
            100% { box-shadow: 0 0 0 0 rgba(39, 174, 96, 0); }
        }

        .prediction-display {
            text-align: center;
            margin-bottom: 1.5rem;
            padding: 1rem;
            border-radius: 8px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
        }

        .prediction-class {
            font-size: 1.8rem;
            font-weight: bold;
            margin-bottom: 0.5rem;
        }

        .confidence-display {
            font-size: 1.2rem;
            opacity: 0.9;
        }

        .stats-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 1rem;
            margin-bottom: 1.5rem;
        }

        .stat-item {
            text-align: center;
            padding: 1rem;
            background: var(--light-gray);
            border-radius: 8px;
        }

        .stat-value {
            font-size: 2rem;
            font-weight: bold;
            color: var(--secondary-color);
        }

        .stat-label {
            font-size: 0.9rem;
            color: var(--medium-gray);
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }

        .probabilities-section h3 {
            color: var(--primary-color);
            margin-bottom: 1rem;
            font-size: 1.2rem;
        }

        .prob-item {
            display: flex;
            align-items: center;
            justify-content: space-between;
            padding: 0.75rem 0;
            border-bottom: 1px solid #e9ecef;
        }

        .prob-item:last-child {
            border-bottom: none;
        }

        .prob-label {
            font-weight: 500;
            text-transform: capitalize;
        }

        .prob-bar-container {
            flex: 1;
            margin: 0 1rem;
            height: 8px;
            background: #e9ecef;
            border-radius: 4px;
            overflow: hidden;
        }

        .prob-bar {
            height: 100%;
            background: linear-gradient(90deg, var(--secondary-color), var(--success-color));
            border-radius: 4px;
            transition: width 0.5s ease;
        }

        .prob-value {
            font-weight: bold;
            color: var(--primary-color);
            min-width: 50px;
            text-align: right;
        }

        .controls {
            margin-top: 2rem;
            text-align: center;
        }

        .btn {
            background: var(--secondary-color);
            color: white;
            border: none;
            padding: 0.75rem 1.5rem;
            border-radius: 8px;
            font-size: 1rem;
            cursor: pointer;
            transition: all 0.3s ease;
            text-decoration: none;
            display: inline-block;
            margin: 0 0.5rem;
        }

        .btn:hover {
            background: #2980b9;
            transform: translateY(-2px);
            box-shadow: 0 4px 12px rgba(52, 152, 219, 0.3);
        }

        .btn-secondary {
            background: var(--medium-gray);
        }

        .btn-secondary:hover {
            background: #5a6268;
        }

        .footer {
            text-align: center;
            margin-top: 3rem;
            padding: 2rem 0;
            color: var(--medium-gray);
            font-size: 0.9rem;
        }

        .loading {
            display: flex;
            align-items: center;
            justify-content: center;
            height: 200px;
            font-size: 1.1rem;
            color: var(--medium-gray);
        }

        .error {
            color: var(--danger-color);
            text-align: center;
            padding: 1rem;
            background: #f8d7da;
            border-radius: 8px;
            border: 1px solid #f5c6cb;
        }
    </style>
</head>
<body>
    <div class="container">
        <header class="header">
            <h1>🤖 Recycling Robot</h1>
            <p>Real-time AI-powered waste classification system</p>
        </header>

        <div class="dashboard-grid">
            <!-- Video Feed Section -->
            <div class="card video-container">
                <h2>Live Camera Feed</h2>
                <div class="status-indicator" title="System Online"></div>
                <img 
                    src="/api/video/stream.mjpeg" 
                    class="video-stream" 
                    alt="Live camera feed"
                    onerror="this.style.display='none'; document.getElementById('video-error').style.display='block';"
                >
                <div id="video-error" class="error" style="display: none;">
                    Camera feed unavailable. Check system status.
                </div>
            </div>

            <!-- Statistics Section -->
            <div class="card">
                <h2>📊 Classification Results</h2>
                
                <div class="prediction-display">
                    <div id="prediction-class" class="prediction-class">Loading...</div>
                    <div id="confidence-display" class="confidence-display">Confidence: ---%</div>
                </div>

                <div class="stats-grid">
                    <div class="stat-item">
                        <div id="fps-value" class="stat-value">--</div>
                        <div class="stat-label">FPS</div>
                    </div>
                    <div class="stat-item">
                        <div id="total-inferences" class="stat-value">--</div>
                        <div class="stat-label">Total</div>
                    </div>
                </div>

                <div class="probabilities-section">
                    <h3>Class Probabilities</h3>
                    <div id="probabilities-list" class="loading">
                        Loading probabilities...
                    </div>
                </div>
            </div>
        </div>

        <!-- System Status -->
        <div class="card">
            <h2>⚙️ System Status</h2>
            <div class="stats-grid" style="grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));">
                <div class="stat-item">
                    <div id="uptime-value" class="stat-value">--</div>
                    <div class="stat-label">Uptime</div>
                </div>
                <div class="stat-item">
                    <div id="cardboard-count" class="stat-value">--</div>
                    <div class="stat-label">Cardboard</div>
                </div>
                <div class="stat-item">
                    <div id="glass-count" class="stat-value">--</div>
                    <div class="stat-label">Glass</div>
                </div>
                <div class="stat-item">
                    <div id="metal-count" class="stat-value">--</div>
                    <div class="stat-label">Metal</div>
                </div>
                <div class="stat-item">
                    <div id="plastic-count" class="stat-value">--</div>
                    <div class="stat-label">Plastic</div>
                </div>
                <div class="stat-item">
                    <div id="trash-count" class="stat-value">--</div>
                    <div class="stat-label">Trash</div>
                </div>
            </div>
        </div>

        <div class="controls">
            <button class="btn" onclick="refreshStats()">🔄 Refresh Stats</button>
            <button class="btn btn-secondary" onclick="resetStats()">🗑️ Reset Counts</button>
            <a href="/health" class="btn btn-secondary">⚕️ Health Check</a>
        </div>

        <footer class="footer">
            <p>Last updated: <span id="last-updated">--</span></p>
            <p>Recycling Robot Dashboard v1.0 | Phase 3 Modular Architecture</p>
        </footer>
    </div>

    <script>
        let updateInterval;

        async function updateDashboard() {
            try {
                const response = await fetch('/api/stats');
                if (!response.ok) throw new Error('Failed to fetch stats');
                
                const data = await response.json();
                
                // Update main prediction
                document.getElementById('prediction-class').textContent = 
                    data.current_prediction || 'No Detection';
                
                const confidence = data.confidence || 0;
                const confidencePct = (confidence * 100).toFixed(1);
                document.getElementById('confidence-display').textContent = 
                    `Confidence: ${confidencePct}%`;
                
                // Update performance stats
                document.getElementById('fps-value').textContent = data.fps || '--';
                document.getElementById('total-inferences').textContent = 
                    data.total_inferences || '--';
                
                // Update uptime
                const uptimeSeconds = data.uptime_seconds || 0;
                const hours = Math.floor(uptimeSeconds / 3600);
                const minutes = Math.floor((uptimeSeconds % 3600) / 60);
                document.getElementById('uptime-value').textContent = 
                    `${hours}:${minutes.toString().padStart(2, '0')}`;
                
                // Update class counts
                const counts = data.class_counts || {};
                document.getElementById('cardboard-count').textContent = counts.cardboard || 0;
                document.getElementById('glass-count').textContent = counts.glass || 0;
                document.getElementById('metal-count').textContent = counts.metal || 0;
                document.getElementById('plastic-count').textContent = counts.plastic || 0;
                document.getElementById('trash-count').textContent = counts.trash || 0;
                
                // Update probabilities
                updateProbabilities(data.all_probabilities || []);
                
                // Update timestamp
                document.getElementById('last-updated').textContent = 
                    new Date().toLocaleTimeString();
                
            } catch (error) {
                console.error('Error updating dashboard:', error);
                document.getElementById('probabilities-list').innerHTML = 
                    '<div class="error">Error loading data</div>';
            }
        }

        function updateProbabilities(probabilities) {
            const container = document.getElementById('probabilities-list');
            
            if (!probabilities || probabilities.length === 0) {
                container.innerHTML = '<div class="loading">No data available</div>';
                return;
            }
            
            container.innerHTML = '';
            
            probabilities.forEach(([className, probability]) => {
                const probItem = document.createElement('div');
                probItem.className = 'prob-item';
                
                const probPct = (probability * 100).toFixed(1);
                
                probItem.innerHTML = `
                    <span class="prob-label">${className}</span>
                    <div class="prob-bar-container">
                        <div class="prob-bar" style="width: ${probPct}%"></div>
                    </div>
                    <span class="prob-value">${probPct}%</span>
                `;
                
                container.appendChild(probItem);
            });
        }

        async function refreshStats() {
            await updateDashboard();
        }

        async function resetStats() {
            if (!confirm('Are you sure you want to reset all statistics?')) {
                return;
            }
            
            try {
                const response = await fetch('/api/stats/reset', { method: 'POST' });
                if (response.ok) {
                    await updateDashboard();
                    alert('Statistics reset successfully!');
                } else {
                    alert('Failed to reset statistics');
                }
            } catch (error) {
                console.error('Error resetting stats:', error);
                alert('Error resetting statistics');
            }
        }

        // Initialize dashboard
        document.addEventListener('DOMContentLoaded', function() {
            updateDashboard();
            updateInterval = setInterval(updateDashboard, 1000);
        });

        // Cleanup on page unload
        window.addEventListener('beforeunload', function() {
            if (updateInterval) {
                clearInterval(updateInterval);
            }
        });
    </script>
</body>
</html>
'''

class WebAPI:
    """
    Flask-based web API for the recycling robot.
    Provides REST endpoints for statistics and video streaming.
    """
    
    def __init__(self, inference_service: InferenceService):
        self.inference_service = inference_service
        self.app = Flask(__name__)
        self._setup_routes()
        
        # Video streaming settings
        self.jpeg_quality = 75
        self.stream_boundary = "frame"
        
    def _setup_routes(self) -> None:
        """Configure Flask routes."""
        
        @self.app.route('/')
        def dashboard():
            """Main dashboard page."""
            return render_template_string(DASHBOARD_HTML)
        
        @self.app.route('/health')
        def health():
            """Health check endpoint."""
            return jsonify({
                "status": "healthy",
                "service_running": self.inference_service.is_running(),
                "timestamp": time.time()
            })
        
        @self.app.route('/api/stats')
        def get_stats():
            """Get current inference statistics."""
            stats = self.inference_service.get_stats()
            latest_result = self.inference_service.get_latest_result()
            
            response_data = {
                "current_prediction": stats.current_prediction,
                "confidence": stats.current_confidence,
                "fps": stats.fps,
                "total_inferences": stats.total_inferences,
                "class_counts": stats.class_counts,
                "uptime_seconds": stats.uptime_seconds,
                "timestamp": time.time()
            }
            
            # Add detailed probabilities if available
            if latest_result:
                response_data["all_probabilities"] = [
                    [class_name, prob]
                    for class_name, prob in latest_result.all_probabilities.items()
                ]
                response_data["inference_timestamp"] = latest_result.timestamp
            
            return jsonify(response_data)
        
        @self.app.route('/api/stats/reset', methods=['POST'])
        def reset_stats():
            """Reset inference statistics."""
            self.inference_service.reset_stats()
            return jsonify({"message": "Statistics reset successfully"})
        
        @self.app.route('/api/config')
        def get_config():
            """Get current system configuration."""
            return jsonify({
                "inference_rate": self.inference_service.inference_rate,
                "class_names": self.inference_service.classifier.class_names,
                "camera_running": self.inference_service.camera_manager.is_running(),
                "device": self.inference_service.classifier.device
            })
        
        @self.app.route('/api/video/stream.mjpeg')
        def video_stream():
            """MJPEG video stream endpoint."""
            return Response(
                self._generate_video_stream(),
                mimetype=f'multipart/x-mixed-replace; boundary={self.stream_boundary}'
            )
        
        @self.app.errorhandler(404)
        def not_found(error):
            return jsonify({"error": "Endpoint not found"}), 404
        
        @self.app.errorhandler(500)
        def internal_error(error):
            logger.error(f"Internal server error: {error}")
            return jsonify({"error": "Internal server error"}), 500
    
    def _generate_video_stream(self):
        """Generator for MJPEG video stream."""
        while True:
            try:
                # Get current frame and result
                frame = self.inference_service.camera_manager.capture_frame()
                latest_result = self.inference_service.get_latest_result()
                
                # Create overlay
                if latest_result:
                    frame_with_overlay = self._create_overlay(frame, latest_result)
                else:
                    frame_with_overlay = frame.copy()
                
                # Encode as JPEG
                # Convert RGB to BGR for OpenCV
                bgr_frame = cv2.cvtColor(frame_with_overlay, cv2.COLOR_RGB2BGR)
                
                success, buffer = cv2.imencode(
                    '.jpg', 
                    bgr_frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                )
                
                if not success:
                    continue
                
                frame_bytes = buffer.tobytes()
                
                # Yield frame in multipart format
                yield (
                    b'--' + self.stream_boundary.encode() + b'\r\n'
                    b'Content-Type: image/jpeg\r\n'
                    b'Content-Length: ' + str(len(frame_bytes)).encode() + b'\r\n\r\n' +
                    frame_bytes + b'\r\n'
                )
                
            except Exception as e:
                logger.error(f"Error in video stream: {e}")
                time.sleep(0.1)
    
    def _create_overlay(self, frame_rgb, result):
        """Create overlay with classification results on frame."""
        # Convert to BGR for OpenCV operations
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        
        # Overlay settings
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        thickness = 2
        
        # Determine color based on confidence
        confidence = result.confidence
        if confidence > 0.8:
            color = (0, 255, 0)  # Green
        elif confidence > 0.6:
            color = (0, 165, 255)  # Orange
        else:
            color = (0, 0, 255)  # Red
        
        # Main prediction text
        main_text = f"{result.predicted_class} ({confidence*100:.1f}%)"
        (text_w, text_h), _ = cv2.getTextSize(main_text, font, font_scale, thickness)
        
        # Background rectangle
        cv2.rectangle(
            frame_bgr, 
            (10, 10), 
            (text_w + 20, text_h + 30),
            (0, 0, 0), 
            -1
        )
        
        # Main text
        cv2.putText(
            frame_bgr,
            main_text,
            (15, 35),
            font,
            font_scale,
            color,
            thickness,
            cv2.LINE_AA
        )
        
        # Add timestamp
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(
            frame_bgr,
            timestamp,
            (15, frame_bgr.shape[0] - 15),
            font,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )
        
        # Convert back to RGB
        return cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    
    def run(
        self, 
        host: str = "0.0.0.0", 
        port: int = 8000, 
        debug: bool = False
    ) -> None:
        """
        Run the Flask development server.
        
        Args:
            host: Host address to bind to
            port: Port number
            debug: Enable debug mode
        """
        logger.info(f"Starting web API on {host}:{port}")
        self.app.run(host=host, port=port, debug=debug, threaded=True)

def create_web_api(inference_service: InferenceService) -> WebAPI:
    """Factory function to create WebAPI instance."""
    return WebAPI(inference_service)